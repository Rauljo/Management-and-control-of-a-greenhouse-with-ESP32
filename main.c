#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"
#include "app.h"

#define TAG "app"
#define SENSOR_NAME "FireBeetle_2_S3"

#include "driver/adc.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

#include <stdio.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include <unistd.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include <stdio.h>
#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "esp_timer.h"
#include "driver/adc.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#include "driver/adc.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

#include <stdio.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include <unistd.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include <stdio.h>
#include <driver/i2c_master.h>
#include <driver/i2c.h>
#include "esp_timer.h"
#include "driver/adc.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "esp_sleep.h"


#define I2C_MASTER_SCL_IO    2               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    1               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0              /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000              /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0            /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0            /*!< I2C master doesn't need buffer */
#define I2C_MASTER_SRC_CLK 0

#define write_data_size_SCD30_op 5
#define write_data_size_SCD30_rm 2
#define write_data_size_SCD30_dr 2
#define write_data_size_SCD30_mi 5
#define read_data_size_SCD30 18
#define read_data_size_SCD30_dr 3

#define LUX0 0x0F
#define LUX1 0x1F
#define LUX2 0xFF
#define HUM 50
#define MAX_LUX_EXT 0x1FF
#define GPIO_PWM GPIO_NUM_6

// Definimos los pines para el LED y el pulsador
#define LED_GPIO 21
#define OnOffPin 47 // Reemplaza con el GPIO del pulsador

i2c_master_dev_handle_t i2c_dev, dev_handle_SCD30;
uint8_t datos[2] = {};
esp_timer_handle_t periodic_timer;
int contador = 0;

//NUEVO
#define PUERTO_HUMEDAD 8
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t calibration_handler;

i2c_master_bus_handle_t i2c_bus;

static int16_t humedad;
static double luz;
uint16_t humidity_percent;
float CO2, temperature, ambient_humidity;
int tmp32;

uint16_t temp, hum;

uint8_t write_data_SCD30_op[write_data_size_SCD30_op] = {0x00, 0x10, 0x00, 0x00, 0x81};
uint8_t write_data_SCD30_rm[write_data_size_SCD30_rm] = {0x03, 0x00};
uint8_t write_data_SCD30_dr[write_data_size_SCD30_dr] = {0x02, 0x02};
uint8_t write_data_SCD30_mi[write_data_size_SCD30_mi] = {0x46, 0x00, 0x00, 0x02, 0xE3};

uint8_t read_data_SCD30[read_data_size_SCD30] = {};
uint8_t read_data_SCD30_dr[read_data_size_SCD30_dr] = {};

static esp_attr_value_t sensor_data = {
    .attr_max_len = (uint16_t)sizeof(temp),
    .attr_len = (uint16_t)sizeof(temp),
    .attr_value = (uint8_t *)(&temp),
};

typedef struct
{
uint16_t dValue1; // Cabecera (opcional)
double dValue2; // Valor de iluminación
uint16_t dValue3; // Valor de humedad
} DataSens_t;

QueueHandle_t QTempHum = NULL;


SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t semaforo_escribir_cola;
SemaphoreHandle_t semaforo_co2;
SemaphoreHandle_t mutex_bluetooth;
SemaphoreHandle_t xButtonSemaphore;

EventGroupHandle_t xEventGroup;
#define EVENT_LUX_LOW    (1 << 0)  // Bit 0 para nivel bajo de luz
#define EVENT_HUM_LOW    (1 << 1)  // Bit 1 para nivel bajo de humedad


uint32_t valorPWM;
ledc_channel_config_t pwm_channel;


int OnOff_flag = 0;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    gpio_intr_disable(OnOffPin);
    gpio_isr_handler_remove(OnOffPin);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    //gpio_set_level(LED_GPIO, OnOff_flag);
    //OnOff_flag = !OnOff_flag;

    xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

    //Reactivamos la interrupción
    gpio_isr_handler_add(OnOffPin, gpio_isr_handler, NULL);
    gpio_intr_enable(OnOffPin);
}



void config_humedad(void) //leer el estado de un pulsador
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t channnel_config1 = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_WIDTH_BIT_12,
    };
    
    adc_oneshot_config_channel(adc1_handle, ADC1_CHANNEL_8, &channnel_config1);


    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_WIDTH_BIT_12,
    };

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &calibration_handler));

}


uint16_t read_hum()
{
    adc_oneshot_get_calibrated_result(adc1_handle, calibration_handler, ADC1_CHANNEL_8, &humedad);
    //NUEVO
    printf("Humedad: %d\n", humedad);


    humidity_percent = humedad * (-0.11186) + 279.87;
    printf("Humedad porcentaje: %u\n", (uint16_t) humidity_percent);

    if (humidity_percent < 20){
        printf("Muy seco\n");
    }
    else if (humidity_percent <= 85){
        printf("Humedad dentro de los límites\n");
    }
    else{
        printf("Humedad excesiva\n");
    }
    return  humidity_percent;

}

double read_luz()
{
    printf("Recibo la informacion\n");
    i2c_master_receive(i2c_dev, datos, 2, 1000);
    printf("Informacion recibida\n");
    double light = ((datos[0] << 8) | datos[1]) / 1.2;
    printf("Sensor valor: %0.02f\n", light);

    if (light < 1000){
        printf("Luz baja\n");
    }
    else if (light < 2000){
        printf("Luz media\n");
    }
    else{
        printf("Luz alta\n");
    }
    return light;
}







static void gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    default:
        break;
    }
}

static void gatt_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        esp_ble_gatts_create_service(gatts_if, &service_def.service_id, GATT_HANDLE_COUNT);
        break;
    case ESP_GATTS_CREATE_EVT:
        service_def.service_handle = param->create.service_handle;

        esp_ble_gatts_start_service(service_def.service_handle);
        esp_ble_gatts_add_char(service_def.service_handle,
                               &service_def.char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                               &sensor_data, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
    {
        service_def.char_handle = param->add_char.attr_handle;
        esp_ble_gatts_add_char_descr(service_def.service_handle, &service_def.descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        service_def.descr_handle = param->add_char_descr.attr_handle;
        esp_ble_gap_config_adv_data(&adv_data);
        break;

    case ESP_GATTS_CONNECT_EVT:
    {
        update_conn_params(param->connect.remote_bda);
        service_def.gatts_if = gatts_if;
        service_def.client_write_conn = param->write.conn_id;
        break;
    }

    case ESP_GATTS_READ_EVT:
    {
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = sensor_data.attr_len;
        memcpy(rsp.attr_value.value, sensor_data.attr_value, sensor_data.attr_len);
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {
        if (service_def.descr_handle == param->write.handle)
        {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value != 0x0000)
            {
                ESP_LOGI(TAG, "notify enable");
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, service_def.char_handle,
                                            sensor_data.attr_len, sensor_data.attr_value, false);
            }
            else
            {
                ESP_LOGI(TAG, "notify disable");
            }
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_WRITE_NOT_PERMIT, NULL);
        }
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        service_def.gatts_if = 0;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

void send_to_queue(void *arg)
{
    while (1){
        xSemaphoreTake(semaforo_escribir_cola, portMAX_DELAY);
        DataSens_t data;
        data.dValue1 = 0;
        data.dValue2 = luz;
        data.dValue3 = hum;
        xQueueSend(QTempHum, &data, portMAX_DELAY);
    }
    
    
}

static void recieve_from_queue(void *arg){
    DataSens_t data;
    BaseType_t xStatus;
    while (1){
        xStatus = xQueueReceive(QTempHum, &data, portMAX_DELAY);
        if (xStatus == pdPASS){
            printf("Recibido luz en cola: %lf\n", data.dValue2);
            printf("Recibido ambiente en cola: %d\n", data.dValue3);

            ESP_LOGI(TAG, "luz: %lf", data.dValue2);
            if (service_def.gatts_if > 0)
            {
                esp_ble_gatts_send_indicate(service_def.gatts_if, service_def.client_write_conn,
                                            service_def.char_handle,
                                            sensor_data.attr_len, sensor_data.attr_value, false);
            }

            ESP_LOGI(TAG, "hum: %u", data.dValue3);
            if (service_def.gatts_if > 0)
            {
                esp_ble_gatts_send_indicate(service_def.gatts_if, service_def.client_write_conn,
                                            service_def.char_handle,
                                            sensor_data.attr_len, sensor_data.attr_value, false);
            }
        }
        else{
            printf("Error al recibir de la cola\n");
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void evaluar_hum(){
    if (hum < HUM){
        printf("Humedad baja, activando humidificador\n");
        //control_hum = 1;
        xEventGroupSetBits(xEventGroup, EVENT_HUM_LOW);
    }
    else{
        //control_hum = 0;
        xEventGroupClearBits(xEventGroup, EVENT_HUM_LOW);
        valorPWM=0;
    }
}

static void read_hum_task(void *arg)
{
    while (1)
    {
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
        
        hum = read_hum();
        evaluar_hum();

        xSemaphoreGive(semaforo_escribir_cola);

    }
}

void evaluar_luz(){
    if (luz < LUX2){
        printf("Luz baja, activando iluminación\n");
        //control_luz = 1;
        xEventGroupSetBits(xEventGroup, EVENT_LUX_LOW);
    }
    else{
        //control_luz = 0;
        xEventGroupClearBits(xEventGroup, EVENT_LUX_LOW);
        valorPWM=0; 
    }

    if (luz < LUX1){
        printf("Luz baja, abriendo ventanas\n");
        gpio_set_level(GPIO_NUM_21, 1);
    }
    else{
        gpio_set_level(GPIO_NUM_11, 0);
    }
}

static void read_light_task(void *arg)
{
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        luz = read_luz();

        //Update pwm
        ledc_set_duty(pwm_channel.speed_mode, pwm_channel.channel, valorPWM);
        ledc_update_duty(pwm_channel.speed_mode, pwm_channel.channel);

        evaluar_luz();
        xSemaphoreGive(xBinarySemaphore);
        xSemaphoreGive(semaforo_co2);
    }
}

float Task_co2(){
    while(1){
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_SCD30, write_data_SCD30_dr, write_data_size_SCD30_dr, -1));
        vTaskDelay(5 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(i2c_master_receive(dev_handle_SCD30, read_data_SCD30_dr, 2, -1));
        if (read_data_SCD30_dr[1])
        {
            ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_SCD30, write_data_SCD30_rm, write_data_size_SCD30_rm, -1));
            vTaskDelay(5 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(i2c_master_receive(dev_handle_SCD30, read_data_SCD30, read_data_size_SCD30, -1));
            tmp32 = read_data_SCD30[0] << 24 | read_data_SCD30[1] << 16 | read_data_SCD30[3] << 8 | read_data_SCD30[4];
            CO2 = *(float*)&tmp32;
            tmp32 = read_data_SCD30[6] << 24 | read_data_SCD30[7] << 16 | read_data_SCD30[9] << 8 | read_data_SCD30[10];
            temperature = *(float*)&tmp32;
            tmp32 = (read_data_SCD30[12] << 24 | read_data_SCD30[13] << 16 | read_data_SCD30[15] << 8 |
            read_data_SCD30[16]);
            ambient_humidity = *(float*)&tmp32;
            //COMPLETAR EL CODIGO PARA SACAR LAS TRES MEDIDAS POR EL MONITOR SERIE//

            printf("Valor de CO2: %f\n", CO2);
            printf("Valor de temperatura: %f\n", temperature);
            printf("Valor de humedad: %f\n", ambient_humidity);

            return (float)read_data_SCD30[7];

        }

        
    }
    
}

static void send_co2_bluetooth(float dato_read){
    xSemaphoreTake(mutex_bluetooth, portMAX_DELAY);
    ESP_LOGI(TAG, "temperatura: %f", dato_read);
            if (service_def.gatts_if > 0)
            {
                esp_ble_gatts_send_indicate(service_def.gatts_if, service_def.client_write_conn,
                                            service_def.char_handle,
                                            sensor_data.attr_len, sensor_data.attr_value, false);
            }
    xSemaphoreGive(mutex_bluetooth);
}

void evaluar_temperatura(float temperature){
    printf("TEMPERATURA: %f\n", temperature);
    if (temperature < 18){
        printf("Temperatura baja, activando calefacción\n");
        gpio_set_level(GPIO_NUM_4, 1);
    }
    else{
        gpio_set_level(GPIO_NUM_4, 0);
    }

    if (temperature > 30){
        printf("Temperatura alta, activando aire acondicionado\n");
        gpio_set_level(GPIO_NUM_5, 1);
    }
    else{
        gpio_set_level(GPIO_NUM_5, 0);
    }
}

static void read_co2_task(void *arg)
{
    while (1)
    {
        xSemaphoreTake(semaforo_co2, portMAX_DELAY);
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        float dato_leido = Task_co2();
        printf("Primer byte temperatura_en_task: %f\n", dato_leido);

        //Ahora evaluamos la temperatura, grabada en una variable global
        evaluar_temperatura(temperature);

        //Enviamos por bluetooth
        send_co2_bluetooth(dato_leido);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        

    }
}

void config_i2c(){
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_new_master_bus(&bus_config, &i2c_bus);
}

void config_luz(){
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x23,
        .scl_speed_hz = 400000,
    };

    i2c_master_bus_add_device(i2c_bus, &device_config, &i2c_dev);

    uint8_t transmit[1] = {0x10};
    printf("Transmito la informacion\n");
    i2c_master_transmit(i2c_dev, transmit, 1, 1000);
}

void config_co2(){
    i2c_device_config_t co2_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x61,
        .scl_speed_hz = 50000,
        .scl_wait_us = 300000
    };

    i2c_master_bus_add_device(i2c_bus, &co2_config, &dev_handle_SCD30);

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_SCD30, write_data_SCD30_op, write_data_size_SCD30_op, -1));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_SCD30, write_data_SCD30_mi, write_data_size_SCD30_mi, -1));
}

void config_leds(){
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NUM_5);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NUM_6);
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NUM_11);
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NUM_21);
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
}

void config_bluetooth(){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gap_set_device_name(SENSOR_NAME);

    esp_ble_gap_register_callback(gap_handler);
    esp_ble_gatts_register_callback(gatt_handler);
    esp_ble_gatts_app_register(0);
}

void config_pwm(){
    ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_12_BIT,
    .freq_hz = 1000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);

    pwm_channel.channel = LEDC_CHANNEL_0;
    pwm_channel.duty = 0;
    pwm_channel.gpio_num = GPIO_PWM;
    pwm_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_channel.hpoint = 0;
    pwm_channel.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&pwm_channel);



}

void control_iluminacion_task(){
    const EventBits_t xBitsToWaitFor = (EVENT_LUX_LOW | EVENT_HUM_LOW);
    const TickType_t xTicksToWait = 2000 / portTICK_PERIOD_MS;

    while(1){
        
        EventBits_t uxBits = xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdFALSE, pdTRUE, xTicksToWait);
        

        //compruebo los bits
        if ((uxBits & xBitsToWaitFor) == xBitsToWaitFor){
            valorPWM = 4196*(MAX_LUX_EXT-luz)/MAX_LUX_EXT;
            //printf("Valor PWM: %ld\n", valorPWM);
        }
        //Delay
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void config_OnOffPIN(void){
    gpio_reset_pin(OnOffPin);
    gpio_set_direction(OnOffPin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(OnOffPin, GPIO_PULLUP_ONLY);

    gpio_set_intr_type(OnOffPin, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(OnOffPin, gpio_isr_handler, NULL);
    gpio_intr_enable(OnOffPin);
}

void cambiar_led_task(){
    while (1){
        //toma el semaforo
        xSemaphoreTake(xButtonSemaphore, portMAX_DELAY);
        

        OnOff_flag = !OnOff_flag;
        gpio_set_level(LED_GPIO, OnOff_flag);
        //printf("Boton pulsado: %d\n", OnOff_flag);

        if (OnOff_flag){
            printf("Activando modo de bajo consumo\n");
            esp_sleep_enable_timer_wakeup(10000000);
            esp_light_sleep_start();
            printf("Valor boton: %d\n", OnOff_flag);
            //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        }
    }
}

void app_main(void)
{
    init_service_def();
    config_humedad();
    config_i2c();
    config_luz();
    config_co2();
    config_leds();
    config_bluetooth();
    

    config_pwm();
    
    xBinarySemaphore = xSemaphoreCreateBinary();
    semaforo_escribir_cola = xSemaphoreCreateBinary();
    semaforo_co2 = xSemaphoreCreateBinary();
    mutex_bluetooth = xSemaphoreCreateMutex();
    xButtonSemaphore = xSemaphoreCreateBinary();

    xEventGroup = xEventGroupCreate();



    config_OnOffPIN();


    //Creamos la cola
    QTempHum = xQueueCreate(5, sizeof(DataSens_t));

    xTaskCreate(read_light_task, "luz", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(read_hum_task, "hum", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(read_co2_task, "co2", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    xTaskCreate(recieve_from_queue, "recieve", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(send_to_queue, "send", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    xTaskCreate(control_iluminacion_task, "control_iluminacion", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(cambiar_led_task, "cambiar_led", configMINIMAL_STACK_SIZE * 3, NULL, 6, NULL);
}