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
#define ALL_TEST    1
#define LED_TEST    0
#define LED_INT_TEST 1
#define SERVO_TEST  0
#define MOTOR_TEST  0

#define HALF_SEC 500 / portTICK_PERIOD_MS
#define REST 2*HALF_SEC
#define TEST_REST 4*HALF_SEC

#define ON  0
#define OFF 1

/* PIN ASSIGNMENTS */
#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5

#define IRLED GPIO_NUM_19
#define IRDETECT GPIO_NUM_18

#define ServoPin GPIO_NUM_23

#define RedLedIntPin GPIO_NUM_25
#define GreenLedIntPin GPIO_NUM_33
#define BlueLedIntPin GPIO_NUM_32
#define RedLedPin GPIO_NUM_14
#define GreenLedPin GPIO_NUM_26
#define BlueLedPin GPIO_NUM_27

/* GLOBAL VARIABLES */
LED *redLed;
LED *blueLed;
LED *greenLed;

ServoMotor *servo;

Motor *rightMotor;
Motor *leftMotor;

void setRGB(uint8_t aRed, uint8_t aBlue, uint8_t aGreen)
{
    redLed->setBrightness(aRed);
    greenLed->setBrightness(aGreen);
    blueLed->setBrightness(aBlue);
}

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    ESP_LOGI("MAIN", "Address=%04X, Command=%04X\r\n\r\n", address, data);
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
        vTaskDelay(TEST_REST);
    }

    if (ALL_TEST || LED_INT_TEST) {
        gpio_reset_pin(RedLedIntPin);
        gpio_reset_pin(GreenLedIntPin);
        gpio_reset_pin(BlueLedIntPin);
        gpio_set_direction(RedLedIntPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(GreenLedIntPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(BlueLedIntPin, GPIO_MODE_OUTPUT);
        gpio_set_level(RedLedIntPin, OFF);
        gpio_set_level(GreenLedIntPin, OFF);
        gpio_set_level(BlueLedIntPin, OFF);


        ESP_LOGI(LOG_TAG,"LED Internal Test Start...");
        ESP_LOGI(LOG_TAG,"Red");
        gpio_set_level(RedLedIntPin, ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Green");
        gpio_set_level(RedLedIntPin, OFF);
        gpio_set_level(GreenLedIntPin, ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"Blue");
        gpio_set_level(GreenLedIntPin, OFF);
        gpio_set_level(BlueLedIntPin, ON);
        vTaskDelay(REST);
        ESP_LOGI(LOG_TAG,"...LED Internal Test Stop");
        gpio_set_level(BlueLedIntPin, OFF);
        vTaskDelay(TEST_REST);
    }

    // Servo
    if (ALL_TEST || SERVO_TEST) {
        servo = new ServoMotor(ServoPin);
        
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
        vTaskDelay(TEST_REST);
    }
    
    // Motors
    if (ALL_TEST || MOTOR_TEST) {
        leftMotor = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
        rightMotor = new Motor(RightMotorLeftPin, RightMotorRightPin);
        
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
        vTaskDelay(TEST_REST);
    }

    ESP_LOGI(LOG_TAG, "--- END TESTS ---");

    vTaskDelay(portMAX_DELAY); // delay Main Task 4 eva
}