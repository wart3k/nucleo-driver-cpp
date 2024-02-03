#ifndef NUCLEO_DRIVER_CPP_TESTS_LED_H
#define NUCLEO_DRIVER_CPP_TESTS_LED_H

#include "stm32f446xx_gpio_drv.h"

#include <cstdint>

enum class LedStatusList : std::uint8_t {
    OFF         = 0,
    ON          = 1,
    UNKNOWN     = 2
};

enum class LedErrorCodeList : std::uint8_t {
    OK      = 0,
    UNKNOWN_PARAMETER,
    GPIO_NOT_INITIALIZED
};

class Led {
public:
    Led(GpioRegisterDefinition* gpio_port, GpioPinList pin, GpioPullConfigList pull_config, GpioOutputTypeConfigList op_type);
    ~Led() = default;

    LedErrorCodeList SetStatus(LedStatusList status);
    LedStatusList GetStatus();

    LedErrorCodeList ToggleStatus();

private:
    PinConfiguration pin_config_;
    GpioHandle gpio_handle_;
    LedStatusList current_status_;

    constexpr static GpioModeList DEFAULT_LED_GPIO_MODE = GpioModeList::OUT;
    constexpr static GpioSpeedList DEFAULT_LED_GPIO_SPEED = GpioSpeedList::LOW;
    constexpr static std::uint8_t DEFAULT_LED_ALT_VALUE = 0U;
};

#endif // NUCLEO_DRIVER_CPP_TESTS_LED_H
