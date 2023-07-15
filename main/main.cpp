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

#define HALF_SEC 500 / portTICK_PERIOD_MS

#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5

#define IRLED GPIO_NUM_19
#define IRDETECT GPIO_NUM_18

#define ServoPin GPIO_NUM_23

#define RedLedPin GPIO_NUM_14
#define BlueLedPin GPIO_NUM_27
#define GreenLedPin GPIO_NUM_26

constexpr auto SpeedSetIRAddress = 0x1254;

/* GLOBAL VARIABLES */
// Reminder: Make your variable names descriptive

// TODO: Add your additional challenge variables
// Syntax Hint: <Class name>* <variable name>;
Motor *rightMotor;
Motor *leftMotor;
LED *redLed;
LED *blueLed;
LED *greenLed;
int LEDState = 0;
ServoMotor *servo;
bool servoAscending = false;
Transceiver *ir;

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

    // Create motors, LEDs, servo, and IR tx
    leftMotor = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
    rightMotor = new Motor(RightMotorLeftPin, RightMotorRightPin);
    redLed = new LED(RedLedPin);
    blueLed = new LED(BlueLedPin);
    greenLed = new LED(GreenLedPin);
    servo = new ServoMotor(ServoPin);
    ir = new Transceiver(IRDETECT, IRLED);

    // Run each test
    vTaskDelay(HALF_SEC);
    // LED
    ESP_LOGI(LOG_TAG,"LED Test Start...");
    ESP_LOGI(LOG_TAG,"Red");
    setRGB(128, 0, 0);
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"Green");
    setRGB(0,128,0);
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"Blue");
    setRGB(0,0,128);
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"...LED Test Stop");
    setRGB(0,0,0);
    vTaskDelay(4*HALF_SEC);

    // Servo
    ESP_LOGI(LOG_TAG,"Servo Test Start...");
    ESP_LOGI(LOG_TAG,"-90");
    servo->setAngle(-90);
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"0");
    servo->setAngle(0);
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"90");
    servo->setAngle(90);
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"...Servo Test Stop");
    servo->setAngle(-90);
    vTaskDelay(4*HALF_SEC);
    
    // Motors
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
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"Left Backward");
    leftMotor->reverse();
    vTaskDelay(HALF_SEC * 2);

    rightMotor->setSpeed(10000);
    leftMotor->setSpeed(0);
    ESP_LOGI(LOG_TAG,"Right Forward");
    rightMotor->forward();
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"Right Backward");
    rightMotor->reverse();
    vTaskDelay(HALF_SEC * 2);
    ESP_LOGI(LOG_TAG,"...Motor Test Stop");
    rightMotor->setSpeed(0);
    vTaskDelay(4*HALF_SEC);

    // ir->mSetReceiveHandler(onReceiveIRData);
    // ir->enableRx();
    // ir->enableTx();

    vTaskDelay(portMAX_DELAY); // delay Main Task 4 eva
}