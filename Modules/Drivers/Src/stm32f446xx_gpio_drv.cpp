/**
 * @brief: To prevent the warning: compound assignment with 'volatile'-qualified left operand is deprecated [-Wvolatile]
 * the code is not written like this:
 * gpio_handle->base_addr->PUPDR &= ~(0x1 << static_cast<std::uint32_t>(gpio_handle->config.pull_mode)); // clear the register
 *
 * but:
 * regVal = gpio_handle->base_addr->PUPDR
 * andVal = ~(0x1 << static_cast<std::uint32_t>(gpio_handle->config.pull_mode));
 * regVal &= andVal;
 * gpio_handle->base_addr->PUPDR = regVal;
 */

#include "stm32f446xx_gpio_drv.h"

#include "stm32f446xx_interrupt.h"
#include "stm32f446xx_syscfg.h"

void Gpio::GpioInit(const GpioHandle& gpio_handle)  {

    PeripheralControl(gpio_handle, true);

    ConfigureGpioMode(gpio_handle);

    ConfigureGpioSpeed(gpio_handle);

    ConfigureGpioPullUpDown(gpio_handle);

    ConfigureGpioOutputTypes(gpio_handle);

    ConfigureAlternateFunctions(gpio_handle);

}

void Gpio::GpioDeinit(const GpioHandle& gpio_handle) {
    GpioRegisterReset(gpio_handle);
}

std::uint8_t Gpio::GetPinStatus(const GpioHandle& gpio_handle) {
    std::uint8_t value;
    value = static_cast<std::uint8_t>((static_cast<std::uint32_t>(gpio_handle.base_addr->IDR) >> static_cast<std::uint32_t>(gpio_handle.config.pin)) & 0x00000001UL);
    return value;
}

std::uint16_t Gpio::GetPortStatus(const GpioHandle& gpio_handle) {
    std::uint16_t value;
    value = static_cast<std::uint16_t>(gpio_handle.base_addr->IDR);
    return value;
}

void Gpio::SetPinStatus(const GpioHandle& gpio_handle, std::uint8_t status) {
    std::uint32_t regVal;
    std::uint32_t tmp;

    if(status == 1U) {
        regVal = gpio_handle.base_addr->ODR;
        tmp = (1U << static_cast<std::uint32_t>(gpio_handle.config.pin));
        regVal |= tmp;
        gpio_handle.base_addr->ODR = regVal;
    }

    else if(status == 0U) {
        regVal = gpio_handle.base_addr->ODR;
        tmp = ~(1U << static_cast<std::uint32_t>(gpio_handle.config.pin));
        regVal &= tmp;
        gpio_handle.base_addr->ODR = regVal;
    }

    else {
    }
}

void Gpio::SetPortStatus(const GpioHandle& gpio_handle, std::uint16_t status) {
    gpio_handle.base_addr->ODR = status;
}

void Gpio::TogglePinStatus(const GpioHandle& gpio_handle) {
    std::uint32_t regVal = gpio_handle.base_addr->ODR;
    std::uint32_t tmp = (1U << static_cast<std::uint32_t>(gpio_handle.config.pin));
    regVal ^= tmp;
    gpio_handle.base_addr->ODR = regVal;
}

void Gpio::IrqConfig(std::uint8_t irq_number, bool enable) {

    if(enable) {

        if(irq_number <= 31U) {
            auto regVal = *NVIC_ISER0;
            std::uint32_t tmp = (1U << static_cast<std::uint32_t>(irq_number));
            regVal |= tmp;
            *NVIC_ISER0 = regVal;
        }

        else if(irq_number < 64U) {
            auto regVal = *NVIC_ISER1;
            std::uint32_t tmp = (1U << (irq_number % 32U));
            regVal |= tmp;
            *NVIC_ISER1 = regVal;
        }

        else if(irq_number < 96U) {
            auto regVal = *NVIC_ISER2;
            std::uint32_t tmp = (1U << (irq_number % 64U));
            regVal |= tmp;
            *NVIC_ISER2 = regVal;
        }

        else {
        }
    }

    else {

        if(irq_number <= 31U) {
            auto regVal = *NVIC_ICER0;
            std::uint32_t tmp = (1U << static_cast<std::uint32_t>(irq_number));
            regVal |= tmp;
            *NVIC_ICER0 = regVal;
        }

        else if(irq_number < 64U) {
            auto regVal = *NVIC_ICER1;
            std::uint32_t tmp = (1U << (static_cast<std::uint32_t>(irq_number) % 32U ));
            regVal |= tmp;
            *NVIC_ICER1 = regVal;
        }

        else if(irq_number < 96U) {
            auto regVal = *NVIC_ICER2;
            std::uint32_t tmp = (1U << (static_cast<std::uint32_t>(irq_number) % 64U ));
            regVal |= tmp;
            *NVIC_ICER2 = regVal;
        }

        else {
        }
    }
}

void Gpio::SetIrqPrio(std::uint8_t irq_number, std::uint32_t prio) {
    std::uint32_t iprx = irq_number / 4U;
    std::uint32_t iprx_section = irq_number % 4U;
    std::uint32_t shift_amount = (8U * iprx_section) + (8U - NO_OF_IMPLEMENTED_PRIORITY_BITS);

    *(NVIC_PRIORITY_BASE_ADDR + iprx) |= (prio << shift_amount);
}

void Gpio::IrqHandle(const GpioHandle& gpio_handle) {
    if(EXTI->PR & (1U << static_cast<std::uint32_t>(gpio_handle.config.pin))) {
        std::uint32_t regVal = EXTI->PR;
        std::uint32_t tmp = (1U << static_cast<std::uint32_t>(gpio_handle.config.pin));

        regVal |= tmp;
        EXTI->PR = tmp;
    }
}



void Gpio::PeripheralControl(const GpioHandle& gpio_handle, bool enable) {
    if(enable) {
        GpioPeripheryClockEnable(gpio_handle);
    }

    else {
        GpioPeripheryClockDisable(gpio_handle);
    }
}

void Gpio::GpioPeripheryClockEnable(const GpioHandle& gpio_handle) {
    auto shiftAmount = static_cast<std::uint32_t>(GpioBaseAddrToCode(gpio_handle.base_addr));

    if(shiftAmount != static_cast<std::uint32_t>(UINT8_MAX)) {
        std::uint32_t regVal = RCC->AHB1ENR;
        std::uint32_t orVal = 1U << shiftAmount;

        regVal |= orVal;
        RCC->AHB1ENR = regVal;
    }
}

void Gpio::GpioPeripheryClockDisable(const GpioHandle& gpio_handle) {
    auto shiftAmount = static_cast<std::uint32_t>(GpioBaseAddrToCode(gpio_handle.base_addr));

    if(shiftAmount != static_cast<std::uint32_t>(UINT8_MAX)){
        std::uint32_t regVal = RCC->AHB1ENR;
        std::uint32_t andVal = ~( 1U << shiftAmount );

        regVal &= andVal;
        RCC->AHB1ENR = regVal;
    }
}

void Gpio::GpioRegisterReset(const GpioHandle& gpio_handle) {
    auto shiftAmount = static_cast<std::uint32_t>(GpioBaseAddrToCode(gpio_handle.base_addr));

    if(shiftAmount != static_cast<std::uint32_t>(UINT8_MAX)) {
        std::uint32_t regVal = RCC->AHB1_RSTR;
        std::uint32_t orVal = (1U << shiftAmount);
        std::uint32_t andVal = ~(1U << shiftAmount);

        regVal |= orVal;
        RCC->AHB1_RSTR = regVal;

        regVal = RCC->AHB1_RSTR;
        regVal &= andVal;
        RCC->AHB1_RSTR = regVal;
    }
}

void Gpio::ConfigureGpioMode(const GpioHandle& gpio_handle) {
    if(gpio_handle.config.mode <= GpioModeList::ANALOG) {
        std::uint32_t mask = 0x3U << (2U * static_cast<std::uint32_t>(gpio_handle.config.pin));
        std::uint32_t userRegVal = static_cast<std::uint32_t>(gpio_handle.config.mode) << (2U * static_cast<std::uint32_t>(gpio_handle.config.pin));

        ClearRegisterMask(gpio_handle.base_addr->MODER, mask);

        SetRegisterValue(gpio_handle.base_addr->MODER, userRegVal, mask, 2U * static_cast<std::uint32_t>(gpio_handle.config.pin));
    } else {
        ConfigureInterruptMode(gpio_handle);
    }
}

void Gpio::ConfigureInterruptMode(const GpioHandle& gpio_handle) {
    if(gpio_handle.config.mode == GpioModeList::IRQ_FALL) {
        SetRegisterBit(EXTI->FTSR, static_cast<std::uint32_t>(gpio_handle.config.pin));
        ClearRegisterBit(EXTI->RTSR, static_cast<std::uint32_t>(gpio_handle.config.pin));
    } else if(gpio_handle.config.mode == GpioModeList::IRQ_RISING) {
        ClearRegisterBit(EXTI->FTSR, static_cast<std::uint32_t>(gpio_handle.config.pin));
        SetRegisterBit(EXTI->RTSR, static_cast<std::uint32_t>(gpio_handle.config.pin));
    } else if(gpio_handle.config.mode == GpioModeList::IRQ_RISE_FALL) {
        SetRegisterBit(EXTI->FTSR, static_cast<std::uint32_t>(gpio_handle.config.pin));
        SetRegisterBit(EXTI->RTSR, static_cast<std::uint32_t>(gpio_handle.config.pin));
    } else {

    }

    ConfigureSyscfgExtiIcr(gpio_handle);

    SetRegisterBit(EXTI->IMR, static_cast<std::uint32_t>(gpio_handle.config.pin));
}

void Gpio::ConfigureSyscfgExtiIcr(const GpioHandle& gpio_handle) {
    uint8_t tmp1 = static_cast<uint32_t>(gpio_handle.config.pin) / 4U;
    uint8_t tmp2 = static_cast<uint32_t>(gpio_handle.config.pin) % 4U;
    uint8_t portCode = GpioBaseAddrToCode(gpio_handle.base_addr);

    SYSCFG_PCLK_EN();
    SYSCFG->EXTICR[tmp1] = portCode << (tmp2 * 4U);
}


void Gpio::ConfigureGpioSpeed(const GpioHandle& gpio_handle) {
    std::uint32_t mask = 0x3U << (2u * static_cast<std::uint32_t>(gpio_handle.config.pin));
    std::uint32_t userRegVal = static_cast<std::uint32_t>(gpio_handle.config.speed) << (2U * static_cast<std::uint32_t>(gpio_handle.config.pin));

    ClearRegisterMask(gpio_handle.base_addr->OSPEEDR, mask);

    SetRegisterValue(gpio_handle.base_addr->OSPEEDR, userRegVal, mask, 2U * static_cast<std::uint32_t>(gpio_handle.config.pin));
}

void Gpio::ConfigureGpioPullUpDown(const GpioHandle& gpio_handle) {
    std::uint32_t mask = 0x3U << (2U * static_cast<std::uint32_t>(gpio_handle.config.pin));
    std::uint32_t userRegVal = static_cast<std::uint32_t>(gpio_handle.config.pull_mode) << (2U * static_cast<std::uint32_t>(gpio_handle.config.pin));

    ClearRegisterMask(gpio_handle.base_addr->PUPDR, mask);

    SetRegisterValue(gpio_handle.base_addr->PUPDR, userRegVal, mask, 2U * static_cast<std::uint32_t>(gpio_handle.config.pin));
}

void Gpio::ConfigureGpioOutputTypes(const GpioHandle& gpio_handle) {
    std::uint32_t mask = 0x1U << static_cast<std::uint32_t>(gpio_handle.config.pin);
    std::uint32_t userRegVal = static_cast<std::uint32_t>(gpio_handle.config.op_type) << static_cast<std::uint32_t>(gpio_handle.config.pin);

    ClearRegisterMask(gpio_handle.base_addr->OTYPER, mask);

    SetRegisterValue(gpio_handle.base_addr->OTYPER, userRegVal, mask, static_cast<std::uint32_t>(gpio_handle.config.pin));
}

void Gpio::ConfigureAlternateFunctions(const GpioHandle& gpio_handle) {
    if(gpio_handle.config.mode == GpioModeList::ALTERNATE) {
        std::uint8_t tmp1 = static_cast<std::uint32_t>(gpio_handle.config.pin) / 8U;
        std::uint8_t tmp2 = static_cast<std::uint32_t>(gpio_handle.config.pin) % 8U;

        std::uint32_t mask = 0xFU << (4U * tmp2);
        std::uint32_t userRegVal = gpio_handle.config.alt_func << (4U * tmp2);

        ClearRegisterMask(gpio_handle.base_addr->AFR[tmp1], mask);

        SetRegisterValue(gpio_handle.base_addr->AFR[tmp1], userRegVal, mask, 4U * tmp2);
    }
}

void Gpio::SetRegisterBit(volatile std::uint32_t& reg, std::uint8_t bit_pos) {
    std::uint32_t val = reg;
    val |= (1U << bit_pos);
    reg = val;
}

void Gpio::ClearRegisterBit(volatile std::uint32_t& reg, std::uint8_t bit_pos) {
    std::uint32_t val = reg;
    val &= ~(1U << bit_pos);
    reg = val;
}

void Gpio::ClearRegisterMask(volatile std::uint32_t& reg, std::uint32_t mask) {
    std::uint32_t val = reg;
    val &= ~mask;
    reg = val;
}

void Gpio::ToggleRegisterBit(volatile std::uint32_t& reg, std::uint8_t bit_pos) {
    std::uint32_t val = reg;
    val ^= (1U << bit_pos);
    reg = val;
}

void Gpio::SetRegisterValue(volatile std::uint32_t& reg, std::uint32_t value, std::uint32_t mask, std::uint8_t shift) {
    reg = (reg & ~mask) | (value << shift);
}

std::uint8_t Gpio::GpioBaseAddrToCode(GpioRegisterDefinition *x) {
    std::uint8_t retVal;

    if (x == GPIOA)  { retVal = 0U; }
    else if (x == GPIOB) { retVal = 1U; }
    else if (x == GPIOC) { retVal = 2U; }
    else if (x == GPIOD) { retVal = 3U; }
    else if (x == GPIOE) { retVal = 4U; }
    else if (x == GPIOF) { retVal = 5U; }
    else if (x == GPIOG) { retVal = 6U; }
    else if (x == GPIOH) { retVal = 7U; }
    else { retVal = UINT8_MAX; }

    return retVal;
}
