#ifndef DRIVERS_INC_STM32F446XX_SYSCFG_H_
#define DRIVERS_INC_STM32F446XX_SYSCFG_H_

#include <cstdint>

#include "stm32f446xx_memory_bus_architecture.h"

/**
 * Peripheral register definition structure for System configuration controller (SYSCFG)
 *
 * @brief: The system configuration controller is mainly used to remap the memory accessible in the code area and to
 * manage the external interrupt line connection to the GPIOs.
 */
struct SyscfgRegisterDefinition {
	volatile std::uint32_t MEMRMP;       /** SYSCFG memory remap register (SYSCFG_MEMRMP) - Address offset: 0x00 */
	volatile std::uint32_t PMC;          /** SYSCFG peripheral mode configuration register (SYSCFG_PMC) - Address offset: 0x04 */
	volatile std::uint32_t  EXTICR[4];   /** SYSCFG external interrupt configuration register 1 to 4 (SYSCFG_EXTICRx) - Address offset: 0x08 - 0x14 */
	std::uint32_t RESERVED1[2];          /** Reserved Register - Address offset: 0x18 - 0x1C */
	volatile uint32_t  CMPCR;            /** Compensation cell control register (SYSCFG_CMPCR) - Address offset: 0x20 */
	std::uint32_t RESERVED2[2];          /** Reserved Register - Address offset: 0x24 - 0x28 */
	volatile uint32_t  CFGR;             /** SYSCFG configuration register (SYSCFG_CFGR) - Address offset: 0x2C */
} drv_syscfg_reg_def_t;

constexpr std::uint32_t SYSCFG_BASE_ADDR = APB2_PERIPH_BASE_ADDR + 0x3800UL;
SyscfgRegisterDefinition* const SYSCFG = reinterpret_cast<SyscfgRegisterDefinition*>(SYSCFG_BASE_ADDR);

/**
 * Clock enable macros for SYSCFG peripherals
 */
inline void SYSCFG_PCLK_EN() {
	std::uint32_t regVal = RCC->APB2ENR;
	std::uint32_t tmp = (1 << 14);
	regVal |= tmp;
	RCC->APB2ENR = regVal;
}

/**
 * Clock disable macros for SYSCFG peripherals
 */
inline void SYSCFG_PCLK_DI() {
	uint32_t regVal = RCC->APB2ENR;
	uint32_t tmp = ~( 1 << 14 );
	regVal &= tmp;
	RCC->APB2ENR = regVal;
}


#endif /* DRIVERS_INC_STM32F446XX_SYSCFG_H_ */
