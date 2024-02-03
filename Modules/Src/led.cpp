#include "led.h"

Led::Led(GpioRegisterDefinition* gpio_port, GpioPinList pin, GpioPullConfigList pull_config, GpioOutputTypeConfigList op_type) {
    pin_config_ = {pin,
                    DEFAULT_LED_GPIO_MODE,
                    DEFAULT_LED_GPIO_SPEED,
                    pull_config,
                    op_type,
                    DEFAULT_LED_ALT_VALUE};

    gpio_handle_ = {gpio_port, pin_config_};
    Gpio::GpioInit(gpio_handle_);
}

LedErrorCodeList Led::SetStatus(LedStatusList status) {
    if(status > LedStatusList::ON) {
        return LedErrorCodeList::UNKNOWN_PARAMETER;
    }

    Gpio::SetPinStatus(gpio_handle_, static_cast<std::uint8_t>(status));
    current_status_ = status;
    return LedErrorCodeList::OK;
}

LedStatusList Led::GetStatus() {
    return current_status_;
}

LedErrorCodeList Led::ToggleStatus() {
        if (GetStatus() == LedStatusList::ON) {
            SetStatus(LedStatusList::OFF);
        } else {
            SetStatus(LedStatusList::ON);
        }
        return LedErrorCodeList::OK;
}
