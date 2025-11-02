#include <stdio.h>
#include <math.h>
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include "esp_http_server.h"
#include "esp_now.h"

#define sensorTurbidez ADC1_CHANNEL_6 // GPIO34
#define sensorLluvia ADC1_CHANNEL_7   // GPIO35
#define pinFlujo GPIO_NUM_26
#define pinRelay GPIO_NUM_27
#define voltajeRef 1100
#define numMuestras 800
#define WIFI_SSID       "Totalplay-2.4G-6740"
#define WIFI_PASS       "35cxHEMt7MTmG9Dh"

volatile uint32_t contadorPulsos = 0;
uint64_t totalPulsos = 0;

static const char *TAG = "ESP32";

float valorNTU = 0;
float voltaje = 0;
float humedad = 0;
float flujo = 0;
float litros = 0;

// 🔹 Variables del sensor ultrasónico recibido
typedef struct {
    float distancia;
    float nivel;
    float llenado;
} NivelData;

static NivelData nivelAgua = {0};

// ────────────────────────────────
// FUNCIONES EXISTENTES
// ────────────────────────────────

float redondear(float valor, int decimales)
{
    float factor = powf(10.0f, decimales);
    return roundf(valor * factor) / factor;
}

float calcularNTU(float v)
{
    float ntu = -1204.82f * v + 2204.8f;
    if (ntu < 0)
        ntu = 0;
    if (ntu > 3000)
        ntu = 3000;
    return ntu;
}

static void IRAM_ATTR interrupcionFlujo(void *arg)
{
    contadorPulsos++;
}

// 🔹 Manejador de datos HTTP modificado para incluir el nivel de agua
static esp_err_t manejadorDatos(httpd_req_t *req)
{
    char respuesta[300];
    snprintf(respuesta, sizeof(respuesta),
             "{\"ntu\":%.2f,\"voltaje\":%.2f,\"humedad\":%.2f,"
             "\"flujo\":%.2f,\"litros\":%.2f,"
             "\"distancia\":%.2f,\"nivel\":%.2f,\"llenado\":%.1f}",
             valorNTU, voltaje, humedad, flujo, litros,
             nivelAgua.distancia, nivelAgua.nivel, nivelAgua.llenado);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, respuesta, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t manejadorRelay(httpd_req_t *req)
{
    char buf[10];
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len > 0)
    {
        if (strncmp(buf, "on", 2) == 0)
        {
            gpio_set_level(pinRelay, 1);
            ESP_LOGI(TAG, "Relay encendido");
        }
        else if (strncmp(buf, "off", 3) == 0)
        {
            gpio_set_level(pinRelay, 0);
            ESP_LOGI(TAG, "Relay apagado");
        }
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_handle_t iniciarServidorWeb(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t servidor = NULL;
    if (httpd_start(&servidor, &config) == ESP_OK)
    {
        httpd_uri_t uriGet = {
            .uri = "/datos",
            .method = HTTP_GET,
            .handler = manejadorDatos};
        httpd_register_uri_handler(servidor, &uriGet);

        httpd_uri_t uriPost = {
            .uri = "/relay",
            .method = HTTP_POST,
            .handler = manejadorRelay};
        httpd_register_uri_handler(servidor, &uriPost);
    }
    return servidor;
}

static void eventoWiFi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW("WIFI", "Desconectado. Reintentando...");
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *evento = (ip_event_got_ip_t *)data;
        ESP_LOGI("WIFI", "IP obtenida: " IPSTR, IP2STR(&evento->ip_info.ip));
    }
}

void configurarWiFi(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t configWiFi = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS}};

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventoWiFi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventoWiFi, NULL);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &configWiFi);
    esp_wifi_start();
}

// ────────────────────────────────
// 🔹 NUEVO: Funciones ESP-NOW receptor
// ────────────────────────────────

static void on_data_recv(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    if (len == sizeof(NivelData))
    {
        memcpy(&nivelAgua, data, sizeof(NivelData));
        ESP_LOGI(TAG, "Datos recibidos ESP-NOW -> Distancia: %.2f cm | Nivel: %.2f cm | Llenado: %.1f%%",
                 nivelAgua.distancia, nivelAgua.nivel, nivelAgua.llenado);
    }
    else
    {
        ESP_LOGW(TAG, "Tamaño inesperado de datos ESP-NOW: %d bytes", len);
    }
}

static void iniciarESPNOW(void)
{
    if (esp_now_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Error iniciando ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(on_data_recv);
    ESP_LOGI(TAG, "ESP-NOW receptor inicializado correctamente");
}

// ────────────────────────────────
// MAIN
// ────────────────────────────────

void app_main(void)
{
    nvs_flash_init();
    configurarWiFi();
    iniciarESPNOW();      // 🔥 NUEVO: inicializa receptor ESP-NOW
    iniciarServidorWeb();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(sensorTurbidez, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(sensorLluvia, ADC_ATTEN_DB_12);

    esp_adc_cal_characteristics_t caracteristicas;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, voltajeRef, &caracteristicas);

    gpio_config_t configGPIO = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pinFlujo),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&configGPIO);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pinFlujo, interrupcionFlujo, NULL);

    gpio_reset_pin(pinRelay);
    gpio_set_direction(pinRelay, GPIO_MODE_OUTPUT);
    gpio_set_level(pinRelay, 0); // Apagado por defecto

    while (1)
    {
        contadorPulsos = 0;
        int64_t tiempoInicio = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(2000));
        int64_t tiempoFin = esp_timer_get_time();
        float duracion = (tiempoFin - tiempoInicio) / 1000000.0f;

        totalPulsos += contadorPulsos;
        flujo = contadorPulsos / 7.5f;
        litros = totalPulsos / 450.0f;

        uint32_t sumaTurbidez = 0;
        for (int i = 0; i < numMuestras; i++)
            sumaTurbidez += adc1_get_raw(sensorTurbidez);
        uint32_t promedioTurbidez = sumaTurbidez / numMuestras;
        uint32_t voltajeMV = esp_adc_cal_raw_to_voltage(promedioTurbidez, &caracteristicas);
        voltaje = redondear(voltajeMV / 1000.0f, 2);
        valorNTU = calcularNTU(voltaje);

        uint32_t sumaLluvia = 0;
        for (int i = 0; i < numMuestras; i++)
            sumaLluvia += adc1_get_raw(sensorLluvia);
        uint32_t promedioLluvia = sumaLluvia / numMuestras;
        humedad = (promedioLluvia * 100.0f) / 4095.0f;

        ESP_LOGI(TAG, "\n==== LECTURAS ====\n"
                      "Turbidez : %.2f NTU\n"
                      "Voltaje  : %.2f V\n"
                      "Humedad  : %.2f %%\n"
                      "Flujo    : %.2f L/min\n"
                      "Litros   : %.2f L\n"
                      "Nivel agua: %.2f cm\n"
                      "Llenado  : %.1f %%\n"
                      "==================\n",
                 valorNTU, voltaje, humedad, flujo, litros,
                 nivelAgua.nivel, nivelAgua.llenado);
    }
}
