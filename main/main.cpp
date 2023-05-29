#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"
#include "Transceiver.hpp"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>

#define RightMotorLeftPin 15
#define RightMotorRightPin 2
#define LeftMotorLeftPin 16
#define LeftMotorRightPin 17
#define IRLED GPIO_NUM_4
#define IRDETECT GPIO_NUM_13

Car *car;
Transceiver *ir;

void onAPress(){};
void onARelease(){};
void onBPress() { car->setCruiseSpeed(car->getCruiseSpeed() - 1000); };
void onBRelease(){};
void onXPress() { car->setCruiseSpeed(car->getCruiseSpeed() + 1000); };
void onXRelease(){};
void onYPress(){};
void onYRelease(){};
void onTriggerPress() { car->enableTurbo(); };
void onTriggerRelease() { car->disableTurbo(); };

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    ESP_LOGI("MAIN", "Address=%04X, Command=%04X\r\n\r\n", address, data);
}

void registerJoystickButtonHandlers(std::shared_ptr<Mocute052> aJoystick)
{
    aJoystick->onA(onAPress, onARelease);
    aJoystick->onB(onBPress, onBRelease);
    aJoystick->onX(onXPress, onXRelease);
    aJoystick->onY(onYPress, onYRelease);
    aJoystick->onTrigger(onTriggerPress, onTriggerRelease);
}

constexpr bool isRx = true;

extern "C" void app_main(void)
{
    nvs_flash_init();
    ir = new Transceiver(IRDETECT, IRLED, 64);
    ir->mSetReceiveHandler(onReceiveIRData);
    if (isRx)
    {
        ir->enableRx();
    }
    else
    {
        ir->enableTx();
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    while (true)
    {
        if (!isRx)
        {
            ir->send();
            ESP_LOGI("MAIN", "SEND");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    auto bt = BTClassicHID::getInstance();
    esp_bd_addr_t joystickAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};
    auto joystick = bt->connect<Mocute052>(joystickAddress);

    Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
    Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);
    car = new Car(joystick, left, right);

    registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}
