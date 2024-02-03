#ifndef DRIVERS_INC_STM32F446XX_GPIO_DRV_H_
#define DRIVERS_INC_STM32F446XX_GPIO_DRV_H_

#include <cstdint>

#include "stm32f446xx_memory_bus_architecture.h"
#include "stm32f446xx_rcc_drv.h"

/**
 * @brief: Peripheral register definition structure for GPIOx
 *
 * Each general-purpose I/O port has four 32-bit configuration registers (GPIOx_MODER,
 * GPIOx_OTYPER, GPIOx_OSPEEDR and GPIOx_PUPDR), two 32-bit data registers (GPIOx_IDR and GPIOx_ODR),
 * a 32-bit set/reset register (GPIOx_BSRR), a 32-bit locking register (GPIOx_LCKR) and two 32-bit
 * alternate function selection register (GPIOx_AFRH and GPIOx_AFRL).
 */
typedef struct  {
    volatile std::uint32_t MODER;   /** GPIO port mode register - Address offset: 0x00 */
    volatile std::uint32_t OTYPER;  /** GPIO port output type register - Address offset: 0x04 */
    volatile std::uint32_t OSPEEDR; /** GPIO port output speed register - Address offset: 0x08 */
    volatile std::uint32_t PUPDR;   /** GPIO port pull-up/pull-down register - Address offset: 0x0C */
    volatile std::uint32_t IDR;     /** GPIO port input data register - Address offset: 0x10 */
    volatile std::uint32_t ODR;     /** GPIO port output data register - Address offset: 0x14 */
    volatile std::uint32_t BSRR;    /** GPIO port bit set/reset register - Address offset: 0x18 */
    volatile std::uint32_t LCKR;    /** GPIO port configuration lock register - Address offset: 0x1C */
    volatile std::uint32_t AFR[2];  /** GPIO alternate function low and high registers - Address offset: 0x20 - 0x24 */
} GpioRegisterDefinition;

enum class GpioPinList : std::uint8_t {
    PIN_0 				= 0U,
    PIN_1 				= 1U,
    PIN_2 				= 2U,
    PIN_3 				= 3U,
    PIN_4 				= 4U,
    PIN_5 				= 5U,
    PIN_6 				= 6U,
    PIN_7 				= 7U,
    PIN_8 				= 8U,
    PIN_9 				= 9U,
    PIN_10 				= 10U,
    PIN_11 				= 11U,
    PIN_12 				= 12U,
    PIN_13 				= 13U,
    PIN_14 				= 14U,
    PIN_15 				= 15U
};

enum class GpioModeList : std::uint8_t {
    IN 					= 0U,
    OUT 				= 1U,
    ALTERNATE 			= 2U,
    ANALOG 				= 3U,
    IRQ_FALL 			= 4U,
    IRQ_RISING 			= 5U,
    IRQ_RISE_FALL 		= 6U
};

enum class GpioOutputTypeConfigList : std::uint8_t {
    PUSH_PULL 			= 0U, /* reset state */
    OPEN_DRAIN			= 1U
};

enum class GpioSpeedList : std::uint8_t {
    LOW					= 0U,
    MEDIUM				= 1U,
    FAST				= 2U,
    HIGH				= 3U
};

enum class GpioPullConfigList : std::uint8_t {
    NO_PULL_UP_DOWN		= 0U,
    PULL_UP				= 1U,
    PULL_DOWN			= 2U
};

typedef struct {
    GpioPinList pin;
    GpioModeList mode;
    GpioSpeedList speed;
    GpioPullConfigList pull_mode;
    GpioOutputTypeConfigList op_type;
    std::uint8_t alt_func;
} PinConfiguration;

typedef struct {
    GpioRegisterDefinition* base_addr;
    PinConfiguration config;
}GpioHandle;

/**
 * Base addresses of peripherals which are hanging on AHB1 bus
 * For more information's see RM0390 2.2.2 Memory map and register boundary addresses
 */
constexpr std::uint32_t GPIOA_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x0000UL;
constexpr std::uint32_t GPIOB_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x0400UL;
constexpr std::uint32_t GPIOC_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x0800UL;
constexpr std::uint32_t GPIOD_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x0C00UL;
constexpr std::uint32_t GPIOE_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x1000UL;
constexpr std::uint32_t GPIOF_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x1400UL;
constexpr std::uint32_t GPIOG_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x1800UL;
constexpr std::uint32_t GPIOH_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x1C00UL;

/**
 * peripheral definitions (peripheral base addresses typecast to typedefs) for easier use
 */
GpioRegisterDefinition* const GPIOA = reinterpret_cast<GpioRegisterDefinition*>(GPIOA_BASE_ADDR);
GpioRegisterDefinition* const GPIOB = reinterpret_cast<GpioRegisterDefinition*>(GPIOB_BASE_ADDR);
GpioRegisterDefinition* const GPIOC = reinterpret_cast<GpioRegisterDefinition*>(GPIOC_BASE_ADDR);
GpioRegisterDefinition* const GPIOD = reinterpret_cast<GpioRegisterDefinition*>(GPIOD_BASE_ADDR);
GpioRegisterDefinition* const GPIOE = reinterpret_cast<GpioRegisterDefinition*>(GPIOE_BASE_ADDR);
GpioRegisterDefinition* const GPIOF = reinterpret_cast<GpioRegisterDefinition*>(GPIOF_BASE_ADDR);
GpioRegisterDefinition* const GPIOG = reinterpret_cast<GpioRegisterDefinition*>(GPIOG_BASE_ADDR);
GpioRegisterDefinition* const GPIOH = reinterpret_cast<GpioRegisterDefinition*>(GPIOH_BASE_ADDR);


class Gpio {
public:
    static void GpioInit(const GpioHandle& gpio_handle);
    static void GpioDeinit(const GpioHandle& gpio_handle);

    static std::uint8_t GetPinStatus(const GpioHandle& gpio_handle);
    static std::uint16_t GetPortStatus(const GpioHandle& gpio_handle);

    static void SetPinStatus(const GpioHandle& gpio_handle, std::uint8_t status);
    static void SetPortStatus(const GpioHandle& gpio_handle, std::uint16_t status);
    static void TogglePinStatus(const GpioHandle& gpio_handle);

    static void IrqConfig(std::uint8_t irq_number, bool enable);
    static void SetIrqPrio(std::uint8_t irq_number, std::uint32_t prio);
    static void IrqHandle(const GpioHandle& gpio_handle);

private:
    static void PeripheralControl(const GpioHandle& gpio_handle, bool enable);

    static inline void GpioPeripheryClockEnable(const GpioHandle& gpio_handle);
    static inline void GpioPeripheryClockDisable(const GpioHandle& gpio_handle);
    static inline void GpioRegisterReset(const GpioHandle& gpio_handle);

    static void ConfigureGpioMode(const GpioHandle& gpio_handle);
    static void ConfigureInterruptMode(const GpioHandle& gpio_handle);
    static void ConfigureSyscfgExtiIcr(const GpioHandle& gpio_handle);
    static void ConfigureGpioSpeed(const GpioHandle& gpio_handle);
    static void ConfigureGpioPullUpDown(const GpioHandle& gpio_handle);
    static void ConfigureGpioOutputTypes(const GpioHandle& gpio_handle);
    static void ConfigureAlternateFunctions(const GpioHandle& gpio_handle);

    static inline void SetRegisterBit(volatile std::uint32_t& reg, std::uint8_t bit_pos);
    static inline void ClearRegisterBit(volatile std::uint32_t& reg, std::uint8_t bit_pos);
    static inline void ClearRegisterMask(volatile std::uint32_t& reg, std::uint32_t mask);
    static inline void ToggleRegisterBit(volatile std::uint32_t& reg, std::uint8_t bit_pos);
    static inline void SetRegisterValue(volatile std::uint32_t& reg, std::uint32_t value, std::uint32_t mask, std::uint8_t shift);

    static inline std::uint8_t GpioBaseAddrToCode(GpioRegisterDefinition* x);
};


#endif /* DRIVERS_INC_STM32F446XX_GPIO_DRV_H_ */
