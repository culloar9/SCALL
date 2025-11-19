//pio run -t erase
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "mqtt_client.h"
#include "certificates.h"
#include "esp_now.h"
#include "esp_rom_sys.h"

// ======================================================
// DEFINICIONES DE PINES
// ======================================================
// SENSORES LOCALES (en esta ESP32 central)
#define SENSOR_TDS ADC1_CHANNEL_6    // GPIO34 - Turbidez/TDS
#define SENSOR_PH  ADC1_CHANNEL_7    // GPIO35 - pH
#define PIN_RELAY  GPIO_NUM_27       // Control de relay

// SENSOR ULTRASÓNICO (LOCAL)
#define TRIG_PIN   GPIO_NUM_4        // Trigger del HC-SR04
#define ECHO_PIN   GPIO_NUM_5        // Echo del HC-SR04

// CONFIGURACIÓN SENSOR ULTRASÓNICO
#define ALTURA_TOTAL_CM  12.0f       // Altura física total del tinaco
#define OFFSET_SENSOR_CM 3.0f        // Margen desde el sensor al nivel máximo

// CONFIGURACIÓN ADC
#define VOLTAJE_REF 1100
#define NUM_MUESTRAS 30             
#define WIFI_CHANNEL 2               // Canal WiFi para ESP-NOW

static const char *TAG = "SCALL-CENTRAL";

// ======================================================
// CÓDIGOS DE EVENTO (deben coincidir con el emisor)
// ======================================================
#define EVT_RUTINA          0
#define EVT_INICIO_LLUVIA   1
#define EVT_INICIO_CAPTACION 2
#define EVT_FIN_CAPTACION   3
#define EVT_FIN_LLUVIA      4
#define EVT_FALLA_SENSOR    5

// Descripciones de eventos para JSON
const char* descripciones_evento[] = {
    "rutina",
    "inicio_lluvia",
    "inicio_captacion",
    "fin_captacion",
    "fin_lluvia",
    "falla_sensor"
};

// ======================================================
// VARIABLES GLOBALES - SENSORES LOCALES
// ======================================================
// Sensor TDS/Turbidez (LOCAL)
float valorNTU = 0, voltajeTDS = 0;
int tdsArray[NUM_MUESTRAS];

// Sensor pH (LOCAL)
float valorPH = 0, voltajePH = 0;
int pHArray[NUM_MUESTRAS];

// Sensor Ultrasónico (LOCAL)
float distanciaUltra = 0;    // Distancia medida en cm
float nivelAgua = 0;         // Nivel de agua en cm
float llenadoTinaco = 0;     // Porcentaje de llenado

// ======================================================
// ESTRUCTURAS DE DATOS REMOTOS (ESP-NOW)
// ======================================================
// Datos de LLUVIA (de ESP32 emisora #1) - ACTUALIZADA
typedef struct {
    float humedad1;
    float humedad2;
    float humedad3;
    float promedio;
    uint8_t evento;        // 0 = rutina, 1-4 = lluvia, 5 = falla
    uint8_t sensor_falla;  // 0 = ninguno, 1 = L1, 2 = L2, 3 = L3
} LluviaData;

LluviaData datosLluvia = {0};

// Datos de NIVEL+FLUJO (de ESP32 emisora #2) - REMOTO
typedef struct {
    float distancia;
    float nivel;
    float llenado;
    float flujo;
    float litros;
} NivelFlujoData;

NivelFlujoData datosNivelFlujo = {0};

// ======================================================
// VARIABLES DE CONECTIVIDAD
// ======================================================
bool wifiConfigurado = false;
bool wifiConectado = false;
bool mqttConectado = false;
char device_id[20] = {0};

// MQTT
esp_mqtt_client_handle_t mqtt_client = NULL;
char mqtt_topic_data[100] = {0};
char mqtt_topic_command[100] = {0};
char mqtt_topic_events[100] = {0};  // NUEVO: Topic para eventos
char mqtt_uri[256] = {0};

// ======================================================
// SISTEMA DE PUBLICACIÓN INTELIGENTE
// ======================================================
float prevNTU, prevPH;
float prevHumedad1, prevHumedad2, prevHumedad3, prevHumedadProm;
float prevDistancia, prevNivel, prevLlenado;  // Ultrasónico LOCAL
float prevDistanciaRemota, prevNivelRemoto, prevLlenadoRemoto;  // Ultrasónico REMOTO
float prevFlujo, prevLitros;
bool primeraLectura = true;
uint64_t ultimaPublicacion = 0;

// Umbrales de cambio
#define UMBRAL_NTU 5.0f
#define UMBRAL_PH 0.2f
#define UMBRAL_HUMEDAD 5.0f
#define UMBRAL_DISTANCIA 1.0f
#define UMBRAL_NIVEL 2.0f
#define UMBRAL_LLENADO 5.0f
#define UMBRAL_FLUJO 0.5f
#define UMBRAL_LITROS 1.0f
#define TIEMPO_HEARTBEAT 60000 // 60s

// ======================================================
// BLE - Configuración WiFi
// ======================================================
#define GATTS_SERVICE_UUID 0x00FF
#define GATTS_CHAR_UUID 0xFF01
#define GATTS_NUM_HANDLE 4

static uint16_t conexion_id = 0;
static uint16_t gatts_if_almacenado = 0;

char ssid_recibido[32] = {0};
char password_recibido[64] = {0};
bool credenciales_recibidas = false;

// ======================================================
// FUNCIONES UTILITARIAS
// ======================================================
float redondear(float v, int d) {
    float f = powf(10.0f, d);
    return roundf(v * f) / f;
}

float calcularNTU(float v) {
    float ntu = -1204.82f * v + 2204.8f;
    if (ntu < 0) ntu = 0;
    if (ntu > 3000) ntu = 3000;
    return ntu;
}

void generarDeviceID(void) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(device_id, sizeof(device_id), "ESP32_%02X%02X%02X",
             mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "✓ Device ID: %s", device_id);
}

// ======================================================
// FILTRO DE MEDIANA
// ======================================================
int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (int i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];

    for (int j = 0; j < iFilterLen - 1; j++) {
        for (int i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                int temp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = temp;
            }
        }
    }

    if ((iFilterLen & 1) > 0)
        return bTab[(iFilterLen - 1) / 2];
    else
        return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

// ======================================================
// SENSOR DE PH (LOCAL)
// ======================================================
float leerPH(esp_adc_cal_characteristics_t *ch) {
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        uint32_t raw = adc1_get_raw(SENSOR_PH);
        pHArray[i] = esp_adc_cal_raw_to_voltage(raw, ch);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    int voltajeMediano_mV = getMedianNum(pHArray, NUM_MUESTRAS);
    voltajePH = voltajeMediano_mV / 1000.0f;

    // Fórmula calibrada para 3.3V
    float ph = 7.0f - ((voltajePH - 1.65f) / 0.1188f);

    if (ph < 0.0f) ph = 0.0f;
    if (ph > 14.0f) ph = 14.0f;

    return redondear(ph, 2);
}

// ======================================================
// SENSOR DE TDS/TURBIDEZ (LOCAL)
// ======================================================
float leerTDS(esp_adc_cal_characteristics_t *ch) {
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        uint32_t raw = adc1_get_raw(SENSOR_TDS);
        tdsArray[i] = esp_adc_cal_raw_to_voltage(raw, ch);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    int voltajeMediano_mV = getMedianNum(tdsArray, NUM_MUESTRAS);
    voltajeTDS = voltajeMediano_mV / 1000.0f;

    float ntu = calcularNTU(voltajeTDS);
    return redondear(ntu, 2);
}

// ======================================================
// SENSOR ULTRASÓNICO (LOCAL)
// ======================================================
void medirNivelAgua(void) {
    // --- Disparo ultrasónico ---
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    // --- Medición del tiempo de eco ---
    int64_t start = esp_timer_get_time();
    int64_t timeout = start + 40000; // 40 ms (máx ~6.8 m)
    
    // Esperar a que ECHO se ponga en HIGH
    while (gpio_get_level(ECHO_PIN) == 0 && esp_timer_get_time() < timeout);
    start = esp_timer_get_time();
    
    // Esperar a que ECHO se ponga en LOW
    while (gpio_get_level(ECHO_PIN) == 1 && esp_timer_get_time() < timeout);
    int64_t end = esp_timer_get_time();

    // --- Cálculo de distancia ---
    float duracion = (float)(end - start);
    distanciaUltra = duracion / 58.0f; // cm

    // --- Compensar el offset del sensor ---
    float distancia_util = distanciaUltra - OFFSET_SENSOR_CM;
    if (distancia_util < 0) distancia_util = 0;                   
    if (distancia_util > ALTURA_TOTAL_CM) distancia_util = ALTURA_TOTAL_CM;

    // --- Calcular nivel real y llenado ---
    nivelAgua = ALTURA_TOTAL_CM - distancia_util;
    llenadoTinaco = (nivelAgua / ALTURA_TOTAL_CM) * 100.0f;

    // --- Asegurar límites ---
    if (llenadoTinaco < 0) llenadoTinaco = 0;
    if (llenadoTinaco > 100) llenadoTinaco = 100;

    // Redondear valores
    distanciaUltra = redondear(distanciaUltra, 2);
    nivelAgua = redondear(nivelAgua, 2);
    llenadoTinaco = redondear(llenadoTinaco, 1);
}

// ======================================================
// PUBLICAR EVENTO A AWS IoT - NUEVO
// ======================================================
void publicarEvento(uint8_t evento, uint8_t sensor_falla) {
    if (!mqttConectado) {
        ESP_LOGW(TAG, "MQTT no conectado - evento no publicado");
        return;
    }

    time_t now = time(NULL);
    
    char payload[300];
    snprintf(payload, sizeof(payload),
             "{"
             "\"device_id\":\"%s\","
             "\"timestamp\":%ld,"
             "\"evento\":%d,"
             "\"descripcion\":\"%s\","
             "\"sensor_falla\":%d,"
             "\"humedad1\":%.1f,"
             "\"humedad2\":%.1f,"
             "\"humedad3\":%.1f,"
             "\"humedad_promedio\":%.1f"
             "}",
             device_id, (long)now,
             evento,
             descripciones_evento[evento],
             sensor_falla,
             datosLluvia.humedad1,
             datosLluvia.humedad2,
             datosLluvia.humedad3,
             datosLluvia.promedio);

    int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_events, payload, 0, 1, 0);
    if (msg_id > 0) {
        ESP_LOGW(TAG, "🔔 EVENTO publicado: %s (msg_id=%d)", descripciones_evento[evento], msg_id);
    } else {
        ESP_LOGE(TAG, "✗ Error al publicar evento");
    }
}

// ======================================================
// ESP-NOW - RECIBIR DATOS REMOTOS - ACTUALIZADO
// ======================================================
static void on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    // Identificar por tamaño de estructura
    if (len == sizeof(LluviaData)) {
        // Datos de LLUVIA
        memcpy(&datosLluvia, data, sizeof(LluviaData));
        
        // Log básico de datos
        ESP_LOGI("ESP-NOW", "🌧 LLUVIA -> H1:%.1f%% H2:%.1f%% H3:%.1f%% Prom:%.1f%%",
                 datosLluvia.humedad1, datosLluvia.humedad2, 
                 datosLluvia.humedad3, datosLluvia.promedio);
        
        // Si hay un evento (no es rutina), procesarlo
        if (datosLluvia.evento != EVT_RUTINA) {
            ESP_LOGW("ESP-NOW", "🔔 EVENTO RECIBIDO: %s", 
                     descripciones_evento[datosLluvia.evento]);
            
            // Publicar evento a AWS IoT
            publicarEvento(datosLluvia.evento, datosLluvia.sensor_falla);
            
            // Log específico según el evento
            switch (datosLluvia.evento) {
                case EVT_INICIO_LLUVIA:
                    ESP_LOGW(TAG, "🌧🌧🌧 ¡EMPIEZA A LLOVER! 🌧🌧🌧");
                    break;
                case EVT_INICIO_CAPTACION:
                    ESP_LOGW(TAG, "💧 Iniciando captación de agua...");
                    break;
                case EVT_FIN_CAPTACION:
                    ESP_LOGW(TAG, "🛑 Terminando captación");
                    break;
                case EVT_FIN_LLUVIA:
                    ESP_LOGW(TAG, "🌤🌤🌤 YA NO ESTÁ LLOVIENDO 🌤🌤🌤");
                    break;
                case EVT_FALLA_SENSOR:
                    ESP_LOGE(TAG, "⚠ FALLA EN SENSOR L%d", datosLluvia.sensor_falla);
                    break;
            }
        }
    } 
    else if (len == sizeof(NivelFlujoData)) {
        // Datos de NIVEL+FLUJO (Ultrasónico REMOTO)
        memcpy(&datosNivelFlujo, data, sizeof(NivelFlujoData));
        ESP_LOGI("ESP-NOW", "📊 NIVEL+FLUJO -> Dist:%.2fcm Nivel:%.2fcm Llenado:%.1f%% Flujo:%.2fL/min Litros:%.2fL",
                 datosNivelFlujo.distancia, datosNivelFlujo.nivel, 
                 datosNivelFlujo.llenado, datosNivelFlujo.flujo, datosNivelFlujo.litros);
    }
}

void iniciarESPNOW(void) {
    uint8_t primary;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);

    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(on_data_recv);

    ESP_LOGI("ESP-NOW", "✓ Receptor inicializado en canal %d", primary);
}

// ======================================================
// NVS - Guardar/Cargar credenciales WiFi
// ======================================================
void guardarCredencialesWiFi(const char *ssid, const char *password) {
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "wifi_ssid", ssid);
        nvs_set_str(h, "wifi_pass", password);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "✓ Credenciales WiFi guardadas");
    }
}

bool cargarCredencialesWiFi(char *ssid, char *password) {
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READONLY, &h) != ESP_OK)
        return false;

    size_t s_len = 32, p_len = 64;
    esp_err_t e1 = nvs_get_str(h, "wifi_ssid", ssid, &s_len);
    esp_err_t e2 = nvs_get_str(h, "wifi_pass", password, &p_len);
    nvs_close(h);
    return (e1 == ESP_OK && e2 == ESP_OK);
}

// ======================================================
// DETECCIÓN DE CAMBIOS
// ======================================================
bool hayaCambiosSignificativos(void) {
    if (primeraLectura) {
        ESP_LOGI(TAG, " Primera lectura - publicando");
        primeraLectura = false;
        return true;
    }

    bool cambio = false;

    // Sensores LOCALES
    if (fabs(valorNTU - prevNTU) > UMBRAL_NTU) {
        ESP_LOGI(TAG, "△ NTU: %.2f → %.2f", prevNTU, valorNTU);
        cambio = true;
    }

    if (fabs(valorPH - prevPH) > UMBRAL_PH) {
        ESP_LOGI(TAG, "△ pH: %.2f → %.2f", prevPH, valorPH);
        cambio = true;
    }

    // Sensor ULTRASÓNICO LOCAL
    if (fabs(distanciaUltra - prevDistancia) > UMBRAL_DISTANCIA) {
        ESP_LOGI(TAG, "△ Distancia LOCAL: %.2f → %.2f cm", prevDistancia, distanciaUltra);
        cambio = true;
    }

    if (fabs(nivelAgua - prevNivel) > UMBRAL_NIVEL) {
        ESP_LOGI(TAG, "△ Nivel LOCAL: %.2f → %.2f cm", prevNivel, nivelAgua);
        cambio = true;
    }

    if (fabs(llenadoTinaco - prevLlenado) > UMBRAL_LLENADO) {
        ESP_LOGI(TAG, "△ Llenado LOCAL: %.1f → %.1f%%", prevLlenado, llenadoTinaco);
        cambio = true;
    }

    // Sensores REMOTOS - Lluvia
    if (fabs(datosLluvia.promedio - prevHumedadProm) > UMBRAL_HUMEDAD) {
        ESP_LOGI(TAG, "△ Humedad: %.1f → %.1f", prevHumedadProm, datosLluvia.promedio);
        cambio = true;
    }

    // Sensores REMOTOS - Nivel+Flujo (Ultrasónico REMOTO)
    if (fabs(datosNivelFlujo.distancia - prevDistanciaRemota) > UMBRAL_DISTANCIA) {
        ESP_LOGI(TAG, "△ Distancia REMOTA: %.2f → %.2f cm", prevDistanciaRemota, datosNivelFlujo.distancia);
        cambio = true;
    }

    if (fabs(datosNivelFlujo.nivel - prevNivelRemoto) > UMBRAL_NIVEL) {
        ESP_LOGI(TAG, "△ Nivel REMOTO: %.2f → %.2f cm", prevNivelRemoto, datosNivelFlujo.nivel);
        cambio = true;
    }

    if (fabs(datosNivelFlujo.llenado - prevLlenadoRemoto) > UMBRAL_LLENADO) {
        ESP_LOGI(TAG, "△ Llenado REMOTO: %.1f → %.1f%%", prevLlenadoRemoto, datosNivelFlujo.llenado);
        cambio = true;
    }

    if (fabs(datosNivelFlujo.flujo - prevFlujo) > UMBRAL_FLUJO) {
        ESP_LOGI(TAG, "△ Flujo: %.2f → %.2f", prevFlujo, datosNivelFlujo.flujo);
        cambio = true;
    }

    if (fabs(datosNivelFlujo.litros - prevLitros) > UMBRAL_LITROS) {
        ESP_LOGI(TAG, "△ Litros: %.2f → %.2f", prevLitros, datosNivelFlujo.litros);
        cambio = true;
    }

    // Heartbeat
    uint64_t ahora = esp_timer_get_time() / 1000;
    if ((ahora - ultimaPublicacion) > TIEMPO_HEARTBEAT) {
        ESP_LOGI(TAG, " Heartbeat (60s)");
        cambio = true;
    }

    return cambio;
}

// ======================================================
// MQTT - PUBLICAR A AWS IoT
// ======================================================
void publicarDatosAWS(void) {
    if (!mqttConectado) {
        ESP_LOGW(TAG, "MQTT no conectado");
        return;
    }

    time_t now = time(NULL);
    int relay_estado = gpio_get_level(PIN_RELAY);

    char payload[1000];
    snprintf(payload, sizeof(payload),
             "{"
             "\"device_id\":\"%s\","
             "\"timestamp\":%ld,"
             // SENSORES LOCALES
             "\"ph\":%.2f,"
             "\"voltaje_ph\":%.2f,"
             "\"ntu\":%.2f,"
             "\"voltaje_tds\":%.2f,"
             "\"relay\":\"%s\","
             // SENSOR ULTRASÓNICO LOCAL (Tinaco Principal)
             "\"distancia_local\":%.2f,"
             "\"nivel_local\":%.2f,"
             "\"llenado_local\":%.1f,"
             // SENSORES REMOTOS - LLUVIA
             "\"humedad1\":%.1f,"
             "\"humedad2\":%.1f,"
             "\"humedad3\":%.1f,"
             "\"humedad_promedio\":%.1f,"
             // SENSORES REMOTOS - NIVEL+FLUJO (Ultrasónico Remoto + Flujo)
             "\"distancia_remota\":%.2f,"
             "\"nivel_remoto\":%.2f,"
             "\"llenado_remoto\":%.1f,"
             "\"flujo\":%.2f,"
             "\"litros\":%.2f"
             "}",
             device_id, (long)now,
             // LOCALES
             valorPH, voltajePH,
             valorNTU, voltajeTDS,
             relay_estado ? "on" : "off",
             // ULTRASÓNICO LOCAL
             distanciaUltra, nivelAgua, llenadoTinaco,
             // REMOTOS - LLUVIA
             datosLluvia.humedad1,
             datosLluvia.humedad2,
             datosLluvia.humedad3,
             datosLluvia.promedio,
             // REMOTOS - NIVEL+FLUJO
             datosNivelFlujo.distancia,
             datosNivelFlujo.nivel,
             datosNivelFlujo.llenado,
             datosNivelFlujo.flujo,
             datosNivelFlujo.litros);

    int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_data, payload, 0, 1, 0);
    if (msg_id > 0) {
        ESP_LOGI(TAG, "✓ Datos publicados a AWS IoT, msg_id=%d", msg_id);
    } else {
        ESP_LOGE(TAG, "✗ Error al publicar datos");
    }
}

// ======================================================
// MQTT - Callback de eventos
// ======================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "✓ MQTT Conectado a AWS IoT Core");
        mqttConectado = true;
        int msg_id = esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_command, 1);
        ESP_LOGI(TAG, "✓ Suscrito a: %s, msg_id=%d", mqtt_topic_command, msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "✗ MQTT Desconectado");
        mqttConectado = false;
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "=== MQTT Mensaje recibido ===");
        ESP_LOGI(TAG, "Topic: %.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "Data: %.*s", event->data_len, event->data);

        if (strncmp(event->data, "{\"action\":\"relay_on\"}", event->data_len) == 0) {
            gpio_set_level(PIN_RELAY, 1);
            ESP_LOGI(TAG, "✓ Relay ON (remoto)");
        } 
        else if (strncmp(event->data, "{\"action\":\"relay_off\"}", event->data_len) == 0) {
            gpio_set_level(PIN_RELAY, 0);
            ESP_LOGI(TAG, "✓ Relay OFF (remoto)");
        } 
        else if (strncmp(event->data, "{\"action\":\"relay_toggle\"}", event->data_len) == 0) {
            int estado = gpio_get_level(PIN_RELAY);
            gpio_set_level(PIN_RELAY, !estado);
            ESP_LOGI(TAG, "✓ Relay TOGGLE: %s", !estado ? "ON" : "OFF");
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        break;

    default:
        break;
    }
}

// ======================================================
// MQTT - Inicializar
// ======================================================
void iniciarMQTT(void) {
    ESP_LOGI(TAG, "Iniciando MQTT a AWS IoT...");

    snprintf(mqtt_topic_data, sizeof(mqtt_topic_data), "device/%s/data", device_id);
    snprintf(mqtt_topic_command, sizeof(mqtt_topic_command), "device/%s/command", device_id);
    snprintf(mqtt_topic_events, sizeof(mqtt_topic_events), "device/%s/events", device_id);  // NUEVO

    ESP_LOGI(TAG, "Topic datos: %s", mqtt_topic_data);
    ESP_LOGI(TAG, "Topic comandos: %s", mqtt_topic_command);
    ESP_LOGI(TAG, "Topic eventos: %s", mqtt_topic_events);  // NUEVO

    strncpy(mqtt_uri, aws_iot_endpoint, sizeof(mqtt_uri) - 1);
    mqtt_uri[sizeof(mqtt_uri) - 1] = '\0';

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .hostname = mqtt_uri,
                .transport = MQTT_TRANSPORT_OVER_SSL,
                .port = 8883,
            },
            .verification = {
                .certificate = root_ca,
            },
        },
        .credentials = {
            .authentication = {
                .certificate = device_cert,
                .key = private_key,
            },
            .client_id = device_id,
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "✗ Error al inicializar MQTT");
        return;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "✓ Cliente MQTT iniciado");
}

// ======================================================
// HTTP - Servidor Web
// ======================================================
static esp_err_t manejadorDatos(httpd_req_t *req) {
    char res[700];
    snprintf(res, sizeof(res),
             "{"
             "\"ph\":%.2f,"
             "\"ntu\":%.2f,"
             "\"distancia_local\":%.2f,"
             "\"nivel_local\":%.2f,"
             "\"llenado_local\":%.1f,"
             "\"humedad_prom\":%.1f,"
             "\"distancia_remota\":%.2f,"
             "\"nivel_remoto\":%.2f,"
             "\"llenado_remoto\":%.1f,"
             "\"flujo\":%.2f,"
             "\"litros\":%.2f"
             "}",
             valorPH, valorNTU,
             distanciaUltra, nivelAgua, llenadoTinaco,
             datosLluvia.promedio,
             datosNivelFlujo.distancia,
             datosNivelFlujo.nivel,
             datosNivelFlujo.llenado,
             datosNivelFlujo.flujo,
             datosNivelFlujo.litros);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, res, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t manejadorRelay(httpd_req_t *req) {
    char buf[10];
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len > 0) {
        if (strncmp(buf, "on", 2) == 0) {
            gpio_set_level(PIN_RELAY, 1);
            ESP_LOGI(TAG, "✓ Relay ON (HTTP)");
        } else if (strncmp(buf, "off", 3) == 0) {
            gpio_set_level(PIN_RELAY, 0);
            ESP_LOGI(TAG, "✓ Relay OFF (HTTP)");
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_handle_t iniciarServidorWeb(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &cfg) == ESP_OK) {
        httpd_uri_t u1 = {.uri = "/datos", .method = HTTP_GET, .handler = manejadorDatos};
        httpd_uri_t u2 = {.uri = "/relay", .method = HTTP_POST, .handler = manejadorRelay};
        httpd_register_uri_handler(srv, &u1);
        httpd_register_uri_handler(srv, &u2);
        ESP_LOGI(TAG, "✓ Servidor HTTP iniciado");
    }
    return srv;
}

// ======================================================
// WiFi - Eventos
// ======================================================
static void eventoWiFi(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifiConectado = false;
        mqttConectado = false;
        ESP_LOGW(TAG, "WiFi desconectado, reintentando...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_wifi_connect();
    } 
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "✓ WiFi conectado! IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        wifiConectado = true;

        iniciarServidorWeb();
        iniciarMQTT();
    }
}

void configurarWiFi(const char *ssid, const char *password) {
    if (!wifiConfigurado) {
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventoWiFi, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventoWiFi, NULL);
        wifiConfigurado = true;
    }

    wifi_config_t conf = {0};
    strncpy((char *)conf.sta.ssid, ssid, sizeof(conf.sta.ssid) - 1);
    strncpy((char *)conf.sta.password, password, sizeof(conf.sta.password) - 1);

    ESP_LOGI(TAG, "Conectando a WiFi: %s", ssid);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &conf);
    esp_wifi_start();
}

// ======================================================
// BLE - GATT Server para configuración WiFi
// ======================================================
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        ESP_LOGI(TAG, "✓ BLE GATT Server registrado");
        gatts_if_almacenado = gatts_if;
        esp_ble_gap_set_device_name("SCALL-ESP32");

        esp_ble_adv_data_t adv_data = {
            .set_scan_rsp = false,
            .include_name = true,
            .include_txpower = true,
            .min_interval = 0x20,
            .max_interval = 0x40,
            .appearance = 0x00,
            .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
        };

        static esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
        };

        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_start_advertising(&adv_params);

        esp_gatt_srvc_id_t sid = {
            .is_primary = true,
            .id.inst_id = 0,
            .id.uuid.len = ESP_UUID_LEN_16,
            .id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID
        };
        esp_ble_gatts_create_service(gatts_if, &sid, GATTS_NUM_HANDLE);
        break;
    }

    case ESP_GATTS_CREATE_EVT: {
        ESP_LOGI(TAG, "✓ Servicio BLE creado");
        esp_ble_gatts_start_service(param->create.service_handle);

        esp_bt_uuid_t uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_CHAR_UUID};
        esp_gatt_char_prop_t prop = ESP_GATT_CHAR_PROP_BIT_READ |
                                    ESP_GATT_CHAR_PROP_BIT_WRITE |
                                    ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        esp_ble_gatts_add_char(param->create.service_handle, &uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               prop, NULL, NULL);
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "✓ Cliente BLE conectado");
        conexion_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(TAG, "✗ Cliente BLE desconectado");
        conexion_id = 0;

        static esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
        };
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        if (!param || !param->write.value || param->write.len <= 0) {
            ESP_LOGW(TAG, "BLE Write vacío");
            break;
        }

        ESP_LOGI(TAG, "=== BLE Datos recibidos ===");

        if (param->write.need_rsp) {
            esp_gatt_rsp_t rsp = {0};
            rsp.attr_value.handle = param->write.handle;
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, &rsp);
        }

        char datos[128] = {0};
        int len_copiar = (param->write.len < sizeof(datos) - 1) ? 
                         param->write.len : sizeof(datos) - 1;
        memcpy(datos, param->write.value, len_copiar);
        datos[len_copiar] = '\0';

        char *separador = strchr(datos, ':');
        if (separador != NULL) {
            *separador = '\0';
            strncpy(ssid_recibido, datos, sizeof(ssid_recibido) - 1);
            strncpy(password_recibido, separador + 1, sizeof(password_recibido) - 1);

            ESP_LOGI(TAG, "✓ Credenciales WiFi recibidas");
            ESP_LOGI(TAG, "  SSID: %s", ssid_recibido);

            credenciales_recibidas = true;
        } else {
            ESP_LOGE(TAG, "✗ Formato inválido (esperado SSID:PASSWORD)");
        }
        break;
    }

    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(TAG, "=== BLE Solicitud de lectura ===");

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = strlen(device_id);
        memcpy(rsp.attr_value.value, device_id, rsp.attr_value.len);

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);

        ESP_LOGI(TAG, "✓ Device ID enviado: %s", device_id);
        break;
    }

    default:
        break;
    }
}

void iniciarBLE(void) {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(NULL);
    esp_ble_gatts_app_register(0);
    ESP_LOGI(TAG, "✓ Servidor BLE iniciado: SCALL-ESP32");
}

// ======================================================
// TAREAS FREERTOS
// ======================================================
void procesar_credenciales_task(void *pv) {
    while (1) {
        if (credenciales_recibidas) {
            credenciales_recibidas = false;
            ESP_LOGI(TAG, "Procesando credenciales WiFi...");
            vTaskDelay(pdMS_TO_TICKS(500));
            guardarCredencialesWiFi(ssid_recibido, password_recibido);
            configurarWiFi(ssid_recibido, password_recibido);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tarea_publicacion_aws(void *pv) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        if (wifiConectado && mqttConectado) {
            // Solo publicar si hay cambios significativos
            if (hayaCambiosSignificativos()) {
                publicarDatosAWS();

                // Actualizar valores previos - SENSORES LOCALES
                prevNTU = valorNTU;
                prevPH = valorPH;
                prevDistancia = distanciaUltra;
                prevNivel = nivelAgua;
                prevLlenado = llenadoTinaco;
                
                // Actualizar valores previos - REMOTOS
                prevHumedadProm = datosLluvia.promedio;
                prevDistanciaRemota = datosNivelFlujo.distancia;
                prevNivelRemoto = datosNivelFlujo.nivel;
                prevLlenadoRemoto = datosNivelFlujo.llenado;
                prevFlujo = datosNivelFlujo.flujo;
                prevLitros = datosNivelFlujo.litros;
                
                ultimaPublicacion = esp_timer_get_time() / 1000;
            }
        }
    }
}

// ======================================================
// APP_MAIN
// ======================================================
void app_main(void) {
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "     SCALL ESP32 - CENTRAL V2.1");
    ESP_LOGI(TAG, "        + SISTEMA DE EVENTOS");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SENSORES LOCALES:");
    ESP_LOGI(TAG, "   ├─ pH (GPIO35)");
    ESP_LOGI(TAG, "   ├─ TDS/NTU (GPIO34)");
    ESP_LOGI(TAG, "   ├─ Ultrasónico LOCAL (GPIO4/5)");
    ESP_LOGI(TAG, "   └─ Relay (GPIO27)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  SENSORES REMOTOS (ESP-NOW):");
    ESP_LOGI(TAG, "   ├─ Emisor #1: Lluvia (3 sensores)");
    ESP_LOGI(TAG, "   └─ Emisor #2: Ultrasónico REMOTO + Flujo");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "   OPTIMIZACIÓN: 30 muestras/sensor");
    ESP_LOGI(TAG, "   OFFSET ULTRASÓNICO: %.1f cm", OFFSET_SENSOR_CM);
    ESP_LOGI(TAG, "========================================\n");

    // Inicializar BLE para configuración
    iniciarBLE();

    // Inicializar red
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg);

    // Generar Device ID único
    generarDeviceID();

    // Intentar cargar credenciales WiFi guardadas
    char ssid[32] = {0}, pass[64] = {0};
    if (cargarCredencialesWiFi(ssid, pass)) {
        ESP_LOGI(TAG, "✓ Credenciales WiFi encontradas");
        configurarWiFi(ssid, pass);
    } else {
        ESP_LOGI(TAG, "⏳ Esperando configuración BLE...");
    }

    // Inicializar ESP-NOW (sensores remotos)
    iniciarESPNOW();

    // Configurar ADC para sensores LOCALES
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_TDS, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SENSOR_PH, ADC_ATTEN_DB_12);

    esp_adc_cal_characteristics_t ch;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, VOLTAJE_REF, &ch);

    // Configurar pines del sensor ULTRASÓNICO
    gpio_reset_pin(TRIG_PIN);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0);
    ESP_LOGI(TAG, "✓ Sensor Ultrasónico configurado (HC-SR04)");

    // Configurar relay
    gpio_reset_pin(PIN_RELAY);
    gpio_set_direction(PIN_RELAY, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RELAY, 0);  // Apagado por defecto
    ESP_LOGI(TAG, "✓ Relay configurado (APAGADO)");

    ESP_LOGI(TAG, "✓ Sensores LOCALES configurados");
    ESP_LOGI(TAG, "========================================\n");

    // Crear tareas FreeRTOS
    xTaskCreate(procesar_credenciales_task, "wifi_task", 4096, NULL, 5, NULL);
    xTaskCreate(tarea_publicacion_aws, "aws_publish_task", 8192, NULL, 5, NULL);

    // Loop principal - lectura de sensores LOCALES
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        if (!wifiConectado) {
            ESP_LOGI(TAG, "Esperando conexión WiFi...");
            continue;
        }

        // ===== LEER SENSORES LOCALES =====
        valorNTU = leerTDS(&ch);
        valorPH = leerPH(&ch);
        medirNivelAgua();

        // Log de TODAS las lecturas (locales + remotas)
        ESP_LOGI(TAG, "\n╔════════════════════════════════════╗");
        ESP_LOGI(TAG, "║      LECTURAS COMPLETAS            ║");
        ESP_LOGI(TAG, "╠════════════════════════════════════╣");
        ESP_LOGI(TAG, "║    SENSORES LOCALES:               ║");
        ESP_LOGI(TAG, "║   pH         : %.2f               ║", valorPH);
        ESP_LOGI(TAG, "║   Voltaje pH : %.2f V              ║", voltajePH);
        ESP_LOGI(TAG, "║   NTU        : %.2f              ║", valorNTU);
        ESP_LOGI(TAG, "║   Voltaje TDS: %.2f V              ║", voltajeTDS);
        ESP_LOGI(TAG, "╠════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  ULTRASÓNICO LOCAL (Tinaco 1):     ║");
        ESP_LOGI(TAG, "║   Distancia  : %.2f cm            ║", distanciaUltra);
        ESP_LOGI(TAG, "║   Nivel Agua : %.2f cm             ║", nivelAgua);
        ESP_LOGI(TAG, "║   Llenado    : %.1f%%               ║", llenadoTinaco);
        ESP_LOGI(TAG, "╠════════════════════════════════════╣");
        ESP_LOGI(TAG, "║     SENSORES REMOTOS - LLUVIA:     ║");
        ESP_LOGI(TAG, "║   Humedad 1  : %.1f%%              ║", datosLluvia.humedad1);
        ESP_LOGI(TAG, "║   Humedad 2  : %.1f%%              ║", datosLluvia.humedad2);
        ESP_LOGI(TAG, "║   Humedad 3  : %.1f%%              ║", datosLluvia.humedad3);
        ESP_LOGI(TAG, "║   Promedio   : %.1f%%              ║", datosLluvia.promedio);
        ESP_LOGI(TAG, "╠════════════════════════════════════╣");
        ESP_LOGI(TAG, "║ ULTRASÓNICO REMOTO (Tinaco 2):     ║");
        ESP_LOGI(TAG, "║   Distancia  : %.2f cm             ║", datosNivelFlujo.distancia);
        ESP_LOGI(TAG, "║   Nivel      : %.2f cm             ║", datosNivelFlujo.nivel);
        ESP_LOGI(TAG, "║   Llenado    : %.1f%%              ║", datosNivelFlujo.llenado);
        ESP_LOGI(TAG, "╠════════════════════════════════════╣");
        ESP_LOGI(TAG, "║   SENSOR REMOTO - FLUJO:           ║");
        ESP_LOGI(TAG, "║   Flujo      : %.2f L/min          ║", datosNivelFlujo.flujo);
        ESP_LOGI(TAG, "║   Litros     : %.2f L              ║", datosNivelFlujo.litros);
        ESP_LOGI(TAG, "╚════════════════════════════════════╝\n");
    }
}
