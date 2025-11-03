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
#include <math.h>

#define CANAL_WIFI 2
#define VOLTAJE_REF 1100
#define NUM_MUESTRAS 800

#define SENSOR_L1 ADC1_CHANNEL_5   // GPIO32
#define SENSOR_L2 ADC1_CHANNEL_4   // GPIO33
#define SENSOR_L3 ADC1_CHANNEL_6   // GPIO34

static const char *TAG = "EMISOR_LLUVIA";

// ====== ESTRUCTURA A ENVIAR ======
typedef struct {
    float humedad1;
    float humedad2;
    float humedad3;
    float promedio;
} LluviaData;

LluviaData datos;

// MAC del receptor principal (reemplaza por la del cerebro SCALL)
uint8_t receptor_mac[] = {0xA4, 0xCF, 0x12, 0x88, 0x39, 0x2C};

// ====== CALLBACK DE ENVÍO ======
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Estado de envío: %s", status == ESP_NOW_SEND_SUCCESS ? "ÉXITO" : "FALLÓ");
}

// ====== FUNCIÓN DE LECTURA ======
static float leerSensor(adc1_channel_t canal, esp_adc_cal_characteristics_t *cal) {
    uint32_t suma = 0;
    for (int i = 0; i < NUM_MUESTRAS; i++) {
        suma += adc1_get_raw(canal);
    }
    uint32_t prom = suma / NUM_MUESTRAS;

    // ⚙️ Fórmula EXACTA a la usada en tu SCALL principal
    float humedad = (prom * 100.0f) / 4095.0f;

    // Redondeo a una cifra decimal, como en tu función original
    humedad = roundf(humedad * 10.0f) / 10.0f;

    if (humedad < 0.0f) humedad = 0.0f;
    if (humedad > 100.0f) humedad = 100.0f;

    return humedad;
}

// ====== INICIALIZAR WIFI Y ESPNOW ======
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
    ESP_LOGI(TAG, "MAC Emisor Lluvia: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

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

// ====== PROGRAMA PRINCIPAL ======
void app_main(void) {
    nvs_flash_init();
    init_wifi();
    init_espnow();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_L1, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SENSOR_L2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SENSOR_L3, ADC_ATTEN_DB_12);

    esp_adc_cal_characteristics_t cal;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, VOLTAJE_REF, &cal);

    ESP_LOGI(TAG, "=== Emisor Lluvia listo ===");

    while (1) {
        datos.humedad1 = leerSensor(SENSOR_L1, &cal);
        datos.humedad2 = leerSensor(SENSOR_L2, &cal);
        datos.humedad3 = leerSensor(SENSOR_L3, &cal);
        datos.promedio = (datos.humedad1 + datos.humedad2 + datos.humedad3) / 3.0f;

        ESP_LOGI(TAG, "H1: %.1f%% | H2: %.1f%% | H3: %.1f%% | Prom: %.1f%%",
                 datos.humedad1, datos.humedad2, datos.humedad3, datos.promedio);

        esp_err_t res = esp_now_send(receptor_mac, (uint8_t *)&datos, sizeof(datos));
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Datos enviados por ESP-NOW");
        } else {
            ESP_LOGW(TAG, "Fallo al enviar datos");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Cada 5 segundos
    }
}
