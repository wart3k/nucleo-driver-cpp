#include "button.h"

#include <cassert>

Button::Button(GpioRegisterDefinition* gpio_port, GpioPinList pin, GpioPullConfigList pull_config, GpioModeList input_mode) {

    pin_config_ = {pin,
            input_mode,
            DEFAULT_BUTTON_GPIO_SPEED,
            pull_config,
            DEFAULT_BUTTON_OUTPUT_CONFIGURATION,
            DEFAULT_BUTTON_ALT_VALUE};

    gpio_handle_ = {gpio_port, pin_config_};
    Gpio::GpioInit(gpio_handle_);
}

ButtonStatusList Button::GetButtonStatus() {
    uint8_t status = Gpio::GetPinStatus(gpio_handle_);
    return static_cast<ButtonStatusList>(status);
}


