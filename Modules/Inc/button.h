#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include <cstdint>
#include <functional>
#include <memory>

#include "stm32f446xx_gpio_drv.h"

enum class ButtonStatusList : std::uint8_t {
    UNPRESSED   = 0,
    PRESSED     = 1,
    UNKNOWN     = 2
};

class Button {
public:
    Button(GpioRegisterDefinition* gpio_port, GpioPinList pin, GpioPullConfigList pull_config, GpioModeList input_mode);
    ~Button() = default;

    ButtonStatusList GetButtonStatus();

private:
    PinConfiguration pin_config_;
    GpioHandle gpio_handle_;
    Gpio gpio_;
    ButtonStatusList current_status_;

    constexpr static GpioSpeedList DEFAULT_BUTTON_GPIO_SPEED = GpioSpeedList::LOW;
    constexpr static GpioOutputTypeConfigList DEFAULT_BUTTON_OUTPUT_CONFIGURATION = GpioOutputTypeConfigList::PUSH_PULL;
    constexpr static std::uint8_t DEFAULT_BUTTON_ALT_VALUE = 0U;
};

#endif /* INC_BUTTON_H_ */
