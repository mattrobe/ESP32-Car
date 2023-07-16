#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"
#include "controller.hpp"
#include "buzzer.hpp"
#include "led.hpp"
#include "servo.hpp"
#include "Transceiver.hpp"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>

#define LOG_TAG "main"

/* TEST CONTROLS */
#define ALL_TEST        1
#define LED_TEST        0
#define INT_LED_TEST    0
#define STATUS_LED_TEST 0
#define SERVO_TEST      0
#define MOTOR_TEST      0

#define HALF_SEC    500 / portTICK_PERIOD_MS
#define REST        2*HALF_SEC

#define ON  0
#define OFF 1
#define STATUS_ON 1
#define STATUS_OFF 0

/* PIN ASSIGNMENTS */
#define RedLedPin GPIO_NUM_14
#define GreenLedPin GPIO_NUM_26
#define BlueLedPin GPIO_NUM_27

#define RedIntLedPin GPIO_NUM_25
#define GreenIntLedPin GPIO_NUM_33
#define BlueIntLedPin GPIO_NUM_32

#define StatusLedPin GPIO_NUM_22

#define ServoPin GPIO_NUM_23

#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5

/* GLOBAL VARIABLES */
LED *redLed;
LED *blueLed;
LED *greenLed;

void setRGB(uint8_t aRed, uint8_t aBlue, uint8_t aGreen)
{
    redLed->setBrightness(aRed);
    greenLed->setBrightness(aGreen);
    blueLed->setBrightness(aBlue);
}

extern "C" void app_main(void)
{
    vTaskDelay(HALF_SEC);
    nvs_flash_init();

    ESP_LOGI(LOG_TAG, "--- BEGIN TESTS ---");

    // LED
    if (ALL_TEST || LED_TEST) {
        redLed = new LED(RedLedPin);
        blueLed = new LED(BlueLedPin);
        greenLed = new LED(GreenLedPin);

        ESP_LOGI(LOG_TAG,"LED Test Start...");
        ESP_LOGI(LOG_TAG,"Red");
        setRGB(128, 0, 0);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Green");
        setRGB(0,128,0);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Blue");
        setRGB(0,0,128);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"...LED Test Stop");
        setRGB(0,0,0);
        vTaskDelay(REST);
    }

    if (ALL_TEST || INT_LED_TEST) {
        gpio_reset_pin(RedIntLedPin);
        gpio_reset_pin(GreenIntLedPin);
        gpio_reset_pin(BlueIntLedPin);
        gpio_set_direction(RedIntLedPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(GreenIntLedPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(BlueIntLedPin, GPIO_MODE_OUTPUT);
        gpio_set_level(RedIntLedPin, OFF);
        gpio_set_level(GreenIntLedPin, OFF);
        gpio_set_level(BlueIntLedPin, OFF);

        ESP_LOGI(LOG_TAG,"LED Internal Test Start...");
        ESP_LOGI(LOG_TAG,"Red");
        gpio_set_level(RedIntLedPin, ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Green");
        gpio_set_level(RedIntLedPin, OFF);
        gpio_set_level(GreenIntLedPin, ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Blue");
        gpio_set_level(GreenIntLedPin, OFF);
        gpio_set_level(BlueIntLedPin, ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"...LED Internal Test Stop");
        gpio_set_level(BlueIntLedPin, OFF);
        vTaskDelay(REST);
    }

    if (ALL_TEST || STATUS_LED_TEST) {
        gpio_reset_pin(StatusLedPin);
        gpio_set_direction(StatusLedPin, GPIO_MODE_OUTPUT);
        gpio_set_level(StatusLedPin, STATUS_OFF);

        ESP_LOGI(LOG_TAG,"Status LED Test Start...");
        gpio_set_level(StatusLedPin, STATUS_ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"...Status LED Test Stop");
        gpio_set_level(StatusLedPin, STATUS_OFF);
        vTaskDelay(REST);
    }

    // Servo
    if (ALL_TEST || SERVO_TEST) {
        ServoMotor *servo = new ServoMotor(ServoPin);
        
        ESP_LOGI(LOG_TAG,"Servo Test Start...");
        ESP_LOGI(LOG_TAG,"-90");
        servo->setAngle(-90);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"0");
        servo->setAngle(0);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"90");
        servo->setAngle(90);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"...Servo Test Stop");
        servo->setAngle(-90);
        vTaskDelay(REST);
    }
    
    // Motors
    if (ALL_TEST || MOTOR_TEST) {
        Motor *leftMotor = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
        Motor *rightMotor = new Motor(RightMotorLeftPin, RightMotorRightPin);
        
        gpio_config_t motorSleepPinConfig;
        motorSleepPinConfig.pin_bit_mask = 1ULL << GPIO_NUM_17;
        motorSleepPinConfig.mode = GPIO_MODE_OUTPUT;
        motorSleepPinConfig.pull_up_en = GPIO_PULLUP_ENABLE;
        motorSleepPinConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
        motorSleepPinConfig.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&motorSleepPinConfig);
        Car::enableMotors();

        ESP_LOGI(LOG_TAG,"Motor Test Start...");
        leftMotor->setSpeed(10000);
        rightMotor->setSpeed(0);
        ESP_LOGI(LOG_TAG,"Left Forward");
        leftMotor->forward();
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Left Backward");
        leftMotor->reverse();
        vTaskDelay(REST);

        rightMotor->setSpeed(10000);
        leftMotor->setSpeed(0);
        ESP_LOGI(LOG_TAG,"Right Forward");
        rightMotor->forward();
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Right Backward");
        rightMotor->reverse();
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"...Motor Test Stop");
        rightMotor->setSpeed(0);
        vTaskDelay(REST);
    }

    ESP_LOGI(LOG_TAG, "--- END TESTS ---");

    vTaskDelay(portMAX_DELAY); // delay Main Task 4 eva
}