#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include <math.h>

// ======================
// CONFIGURACIÓN
// ======================
#define CANAL_WIFI         2
#define VOLTAJE_REF        1100
#define NUM_MUESTRAS_ADC   200
#define BUFFER_SIZE        10
#define INTERVALO_ENVIO_MS 2000

#define SENSOR_L1 ADC1_CHANNEL_5   // GPIO32
#define SENSOR_L2 ADC1_CHANNEL_4   // GPIO33
#define SENSOR_L3 ADC1_CHANNEL_6   // GPIO34

// ======================
// UMBRALES DE DETECCIÓN
// ======================
#define UMBRAL_LLUVIAS      40.0f
#define ESTABILIDAD_SENSOR   3.0f
#define VARIACION_MINIMA     0.3f
#define DESCENSO_GLOBAL      5.0f

static const char *TAG = "EMISOR_LLUVIA";

// ======================
// CÓDIGOS DE EVENTO
// ======================
// EVT_RUTINA          0  - Envío rutinario de datos
// EVT_INICIO_LLUVIA   1  - Empieza a llover
// EVT_INICIO_CAPTACION 2 - Inicia captación de agua (60s después de lluvia)
// EVT_FIN_CAPTACION   3  - Termina captación
// EVT_FIN_LLUVIA      4  - Deja de llover
// EVT_FALLA_SENSOR    5  - Falla detectada en sensor (ver sensor_falla: 1=L1, 2=L2, 3=L3)
// ======================================================
#define EVT_RUTINA          0
#define EVT_INICIO_LLUVIA   1
#define EVT_INICIO_CAPTACION 2
#define EVT_FIN_CAPTACION   3
#define EVT_FIN_LLUVIA      4
#define EVT_FALLA_SENSOR    5

// ======================
// ESTRUCTURA DE DATOS
// ======================
typedef struct {
    float humedad1;
    float humedad2;
    float humedad3;
    float promedio;
    uint8_t evento;        // 0 = rutina, 1-4 = lluvia, 5 = falla
    uint8_t sensor_falla;  // 0 = ninguno, 1 = L1, 2 = L2, 3 = L3
} LluviaData;

LluviaData datos;

// MAC del receptor principal (cerebro SCALL)
uint8_t receptor_mac[] = {0xA4, 0xCF, 0x12, 0x88, 0x39, 0x2C};

// ======================
// BUFFERS Y ESTADOS
// ======================
float buf_L1[BUFFER_SIZE];
float buf_L2[BUFFER_SIZE];
float buf_L3[BUFFER_SIZE];
int idx = 0;
bool buffer_lleno = false;

// Estados principales
bool lloviendo = false;
bool captacion_activa = false;

// Sensores lentos
bool lento_L1 = false;
bool lento_L2 = false;
bool lento_L3 = false;

// Valores referencia
float max_lluvia = 0;
int lecturas_estables = 0;

// Tiempos
int64_t tiempo_lluvia_inicio = 0;

// Contadores de falla
int falla_L1_cont = 0;
int falla_L2_cont = 0;
int falla_L3_cont = 0;

// Calibración ADC
esp_adc_cal_characteristics_t cal;

// ======================
// FUNCIÓN DE ENVÍO
// ======================
static void enviar_datos(uint8_t evento, uint8_t sensor_falla) {
    datos.evento = evento;
    datos.sensor_falla = sensor_falla;
    
    esp_err_t res = esp_now_send(receptor_mac, (uint8_t *)&datos, sizeof(datos));
    
    if (res == ESP_OK) {
        if (evento == EVT_RUTINA) {
            ESP_LOGI(TAG, "📡 Datos enviados | H1:%.1f%% H2:%.1f%% H3:%.1f%% Prom:%.1f%%",
                     datos.humedad1, datos.humedad2, datos.humedad3, datos.promedio);
        } else {
            ESP_LOGW(TAG, "📡 EVENTO %d enviado | Falla:%d", evento, sensor_falla);
        }
    } else {
        ESP_LOGE(TAG, "❌ Fallo al enviar datos");
    }
}

// ======================
// CALLBACK DE ENVÍO
// ======================
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "⚠ ACK no recibido");
    }
}

// ======================
// LECTURA ADC
// ======================
static float leer(adc1_channel_t canal) {
    uint32_t suma = 0;
    for (int i = 0; i < NUM_MUESTRAS_ADC; i++)
        suma += adc1_get_raw(canal);

    float prom = suma / (float)NUM_MUESTRAS_ADC;
    float humedad = (prom * 100.0f) / 4095.0f;
    humedad = roundf(humedad * 10.0f) / 10.0f;
    
    if (humedad < 0.0f) humedad = 0.0f;
    if (humedad > 100.0f) humedad = 100.0f;
    
    return humedad;
}

// ======================
// FUNCIONES DE BUFFER
// ======================
static float dif_buffer(float *buf) {
    float min = 99999, max = -99999;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (buf[i] < min) min = buf[i];
        if (buf[i] > max) max = buf[i];
    }
    return max - min;
}

static float promedio(float *buf) {
    float s = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) s += buf[i];
    return s / BUFFER_SIZE;
}

// ======================
// SENSORES LENTOS
// ======================
static bool sensor_lento(float dif) {
    return (dif < VARIACION_MINIMA);
}

static int sensores_mayoria(bool s1, bool s2, bool s3) {
    int c = 0;
    if (s1) c++;
    if (s2) c++;
    if (s3) c++;
    return c;
}

// ======================
// DETECCIÓN DE FALLAS
// ======================
static void evaluar_fallas_cero(float h1, float h2, float h3) {
    bool L1_zero = (h1 <= 0.1f);
    bool L2_zero = (h2 <= 0.1f);
    bool L3_zero = (h3 <= 0.1f);

    bool falla_detectada = false;

    // Dos en 0, uno no
    if (L1_zero && L2_zero && !L3_zero) { falla_L3_cont++; falla_detectada = true; }
    if (L1_zero && L3_zero && !L2_zero) { falla_L2_cont++; falla_detectada = true; }
    if (L2_zero && L3_zero && !L1_zero) { falla_L1_cont++; falla_detectada = true; }

    // Dos > 0, uno en 0
    if (L1_zero && !L2_zero && !L3_zero) { falla_L1_cont++; falla_detectada = true; }
    if (L2_zero && !L1_zero && !L3_zero) { falla_L2_cont++; falla_detectada = true; }
    if (L3_zero && !L1_zero && !L2_zero) { falla_L3_cont++; falla_detectada = true; }

    // Si no hay falla → reset
    if (!falla_detectada) {
        falla_L1_cont = 0;
        falla_L2_cont = 0;
        falla_L3_cont = 0;
    }

    // Reportar fallas después de 10 mediciones consecutivas
    if (falla_L1_cont >= 10) {
        ESP_LOGE(TAG, "⚠ POSIBLE FALLA: Sensor L1");
        enviar_datos(EVT_FALLA_SENSOR, 1);
        falla_L1_cont = 0;
    }

    if (falla_L2_cont >= 10) {
        ESP_LOGE(TAG, "⚠ POSIBLE FALLA: Sensor L2");
        enviar_datos(EVT_FALLA_SENSOR, 2);
        falla_L2_cont = 0;
    }

    if (falla_L3_cont >= 10) {
        ESP_LOGE(TAG, "⚠ POSIBLE FALLA: Sensor L3");
        enviar_datos(EVT_FALLA_SENSOR, 3);
        falla_L3_cont = 0;
    }
}

// ======================
// INICIALIZACIÓN WIFI
// ======================
static void init_wifi(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE);

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "MAC Emisor: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ======================
// INICIALIZACIÓN ESP-NOW
// ======================
static void init_espnow(void) {
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando ESP-NOW");
        return;
    }

    esp_now_register_send_cb(on_data_sent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, receptor_mac, 6);
    peer.channel = CANAL_WIFI;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir peer receptor");
    }
}

// ======================
// PROGRAMA PRINCIPAL
// ======================
void app_main(void) {
    // Inicialización
    nvs_flash_init();
    init_wifi();
    init_espnow();

    // Configurar ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_L1, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SENSOR_L2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SENSOR_L3, ADC_ATTEN_DB_12);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, VOLTAJE_REF, &cal);

    ESP_LOGI(TAG, "=== Sistema Emisor Lluvia Unificado ===");

    while (1) {
        // Leer sensores
        float h1 = leer(SENSOR_L1);
        float h2 = leer(SENSOR_L2);
        float h3 = leer(SENSOR_L3);
        float prom_actual = (h1 + h2 + h3) / 3.0f;

        // Actualizar estructura de datos
        datos.humedad1 = h1;
        datos.humedad2 = h2;
        datos.humedad3 = h3;
        datos.promedio = prom_actual;

        ESP_LOGI(TAG, "L1=%.1f%%  L2=%.1f%%  L3=%.1f%%", h1, h2, h3);

        // Evaluar fallas
        evaluar_fallas_cero(h1, h2, h3);

        // Actualizar buffers
        buf_L1[idx] = h1;
        buf_L2[idx] = h2;
        buf_L3[idx] = h3;
        idx++;

        if (idx >= BUFFER_SIZE) {
            idx = 0;
            buffer_lleno = true;
        }

        // Esperar a tener buffer lleno para análisis
        if (!buffer_lleno) {
            enviar_datos(EVT_RUTINA, 0);
            vTaskDelay(pdMS_TO_TICKS(INTERVALO_ENVIO_MS));
            continue;
        }

        // Calcular diferencias y promedios del buffer
        float dif1 = dif_buffer(buf_L1);
        float dif2 = dif_buffer(buf_L2);
        float dif3 = dif_buffer(buf_L3);

        float prom1 = promedio(buf_L1);
        float prom2 = promedio(buf_L2);
        float prom3 = promedio(buf_L3);

        float prom_global = (prom1 + prom2 + prom3) / 3.0f;

        // Detectar sensores lentos
        lento_L1 = sensor_lento(dif1);
        lento_L2 = sensor_lento(dif2);
        lento_L3 = sensor_lento(dif3);

        // ======================
        // DETECCIÓN INICIO LLUVIA
        // ======================
        bool s1_subio = (h1 > UMBRAL_LLUVIAS);
        bool s2_subio = (h2 > UMBRAL_LLUVIAS);
        bool s3_subio = (h3 > UMBRAL_LLUVIAS);

        if (!lloviendo && sensores_mayoria(s1_subio, s2_subio, s3_subio) >= 2) {
            lloviendo = true;
            captacion_activa = false;
            tiempo_lluvia_inicio = esp_timer_get_time();
            max_lluvia = prom_global;
            lecturas_estables = 0;

            ESP_LOGW(TAG, "🌧🌧🌧 ¡EMPIEZA A LLOVER! 🌧🌧🌧");
            enviar_datos(EVT_INICIO_LLUVIA, 0);
        }

        // ======================
        // DURANTE LLUVIA
        // ======================
        if (lloviendo) {
            if (prom_global > max_lluvia)
                max_lluvia = prom_global;

            float segundos = (esp_timer_get_time() - tiempo_lluvia_inicio) / 1e6;

            // Iniciar captación después de 60 segundos
            if (!captacion_activa && segundos >= 60) {
                captacion_activa = true;
                ESP_LOGW(TAG, "💧 Iniciando captación de agua...");
                enviar_datos(EVT_INICIO_CAPTACION, 0);
            }

            // Detectar fin de lluvia
            bool s1_baja = (h1 < max_lluvia - DESCENSO_GLOBAL) || lento_L1;
            bool s2_baja = (h2 < max_lluvia - DESCENSO_GLOBAL) || lento_L2;
            bool s3_baja = (h3 < max_lluvia - DESCENSO_GLOBAL) || lento_L3;

            bool fin_mayoria = sensores_mayoria(s1_baja, s2_baja, s3_baja) >= 2;
            bool caida_global = prom_global < (max_lluvia - DESCENSO_GLOBAL);

            if (fin_mayoria || caida_global)
                lecturas_estables++;
            else
                lecturas_estables = 0;

            if (lecturas_estables >= 10) {
                if (captacion_activa) {
                    ESP_LOGW(TAG, "🛑 Terminando captación (ya no llueve)");
                    enviar_datos(EVT_FIN_CAPTACION, 0);
                }

                lloviendo = false;
                captacion_activa = false;

                ESP_LOGW(TAG, "🌤🌤🌤 YA NO ESTÁ LLOVIENDO 🌤🌤🌤");
                enviar_datos(EVT_FIN_LLUVIA, 0);
            }
        }

        // Envío rutinario cada 2 segundos
        enviar_datos(EVT_RUTINA, 0);

        vTaskDelay(pdMS_TO_TICKS(INTERVALO_ENVIO_MS));
    }
}
