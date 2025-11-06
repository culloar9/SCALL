#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define TRIG_PIN GPIO_NUM_4
#define ECHO_PIN GPIO_NUM_5
#define PIN_FLUJO GPIO_NUM_26
#define ALTURA_VASO_CM 12.0f
#define CANAL_WIFI 2

static const char *TAG = "EMISOR_ULTRA_FLUJO";

// ====== ESTRUCTURA A ENVIAR ======
typedef struct {
    float distancia;
    float nivel;
    float llenado;
    float flujo;     // L/min
    float litros;    // Total acumulado
} NivelFlujoData;

NivelFlujoData datos;

// Variables internas del sensor de flujo
volatile uint32_t contadorPulsos = 0;
uint64_t totalPulsos = 0;

// MAC del receptor (ajústala a la del cerebro SCALL)
uint8_t receptor_mac[] = {0xA4, 0xCF, 0x12, 0x88, 0x39, 0x2C};

// ====== INTERRUPCIÓN DEL SENSOR DE FLUJO ======
static void IRAM_ATTR interrupcionFlujo(void *arg) {
    contadorPulsos++;
}

// ====== MEDICIÓN DE NIVEL ======
static void medirNivelAgua(void) {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0)
        start = esp_timer_get_time();

    int64_t end = start;
    while (gpio_get_level(ECHO_PIN) == 1)
        end = esp_timer_get_time();

    float duracion = (end - start);
    datos.distancia = duracion / 58.0f;
    datos.nivel = ALTURA_VASO_CM - datos.distancia;

    if (datos.nivel < 0) datos.nivel = 0;
    if (datos.nivel > ALTURA_VASO_CM) datos.nivel = ALTURA_VASO_CM;

    datos.llenado = (datos.nivel / ALTURA_VASO_CM) * 100.0f;
    if (datos.llenado < 0) datos.llenado = 0;
    if (datos.llenado > 100) datos.llenado = 100;
}

// ====== MEDICIÓN DE FLUJO ======
static void medirFlujo(void) {
    contadorPulsos = 0;
    vTaskDelay(pdMS_TO_TICKS(2000)); // mide durante 2 segundos

    totalPulsos += contadorPulsos;
    datos.flujo = contadorPulsos / 7.5f;
    datos.litros = totalPulsos / 450.0f;
}

// ====== CALLBACK DE ENVÍO ======
static void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Estado de envío: %s", status == ESP_NOW_SEND_SUCCESS ? "ÉXITO" : "FALLÓ");
}

// ====== INICIALIZACIÓN WIFI Y ESPNOW ======
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
    ESP_LOGI(TAG, "MAC de este ESP (emisor): %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void init_espnow(void) {
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando ESP-NOW");
        return;
    }
    esp_now_register_send_cb(on_data_sent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receptor_mac, 6);
    peerInfo.channel = CANAL_WIFI;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir peer receptor");
    }
}

// ====== ENVÍO DE DATOS ======
static void enviar_datos(void) {
    for (int intento = 1; intento <= 3; intento++) {
        esp_err_t res = esp_now_send(receptor_mac, (uint8_t *)&datos, sizeof(datos));
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Intento %d: Datos enviados correctamente", intento);
            break;
        } else {
            ESP_LOGW(TAG, "Intento %d falló, reintentando...", intento);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ====== PROGRAMA PRINCIPAL ======
void app_main(void) {
    nvs_flash_init();
    init_wifi();
    init_espnow();

    gpio_reset_pin(TRIG_PIN);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0);

    // Configurar pin de flujo con interrupción
    gpio_config_t cfg = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_FLUJO),
        .pull_down_en = 0,
        .pull_up_en = 1};
    gpio_config(&cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_FLUJO, interrupcionFlujo, NULL);

    ESP_LOGI(TAG, "✓ Emisor Ultrasónico + Flujo listo");

    while (1) {
        medirNivelAgua();
        medirFlujo();

        ESP_LOGI(TAG, "Dist: %.2f cm | Nivel: %.2f cm | Llenado: %.1f%% | Flujo: %.2f L/min | Litros: %.2f L",
                 datos.distancia, datos.nivel, datos.llenado, datos.flujo, datos.litros);

        enviar_datos();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
