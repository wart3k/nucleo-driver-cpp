#ifndef DRIVERS_INC_STM32F446XX_RCC_DRV_H_
#define DRIVERS_INC_STM32F446XX_RCC_DRV_H_

#include <cstdint>

#include "stm32f446xx_memory_bus_architecture.h"

/**
 * Peripheral register definition structure for RCC (Reset Clock Control)
 */
struct RccRegisterDefinition {
	volatile std::uint32_t CR;			/** RCC clock control register (RCC_CR) - Address offset: 0x00 */
	volatile std::uint32_t PLL_CFGR;	/** RCC PLL configuration register (RCC_PLLCFGR) - Address offset: 0x04 */
	volatile std::uint32_t CFGR;		/** RCC clock configuration register - Address offset: 0x08 */
	volatile std::uint32_t CIR;			/** RCC clock interrupt register (RCC_CIR) - Address offset: 0x0C */
	volatile std::uint32_t AHB1_RSTR;	/** RCC AHB1 peripheral reset register (RCC_AHB1RSTR) - Address offset: 0x10 */
	volatile std::uint32_t AHB2_RSTR;	/** RCC AHB2 peripheral reset register (RCC_AHB2RSTR) - Address offset: 0x14 */
	volatile std::uint32_t AHB3_RSTR;	/** RCC AHB3 peripheral reset register (RCC_AHB3RSTR) - Address offset: 0x18 */
	std::uint32_t RESERVED1;			/**  Reserved register - Address offset: 0x1C */
	volatile std::uint32_t APB1_RSTR;	/** RCC APB1 peripheral reset register (RCC_APB1RSTR) - Address offset: 0x20 */
	volatile std::uint32_t APB2_RSTR;	/** RCC APB2 peripheral reset register (RCC_APB2RSTR) - Address offset: 0x24 */
	std::uint32_t RESERVED2[2];			/**  Reserved register - Address offset: 0x28 to 0x2C */
	volatile std::uint32_t AHB1ENR;		/** RCC AHB1 peripheral clock enable register (RCC_AHB1ENR) - Address offset: 0x30 */
	volatile std::uint32_t AHB2ENR;		/** RCC AHB2 peripheral clock enable register (RCC_AHB2ENR) - Address offset: 0x34 */
	volatile std::uint32_t AHB3ENR;		/** RCC AHB3 peripheral clock enable register (RCC_AHB3ENR) - Address offset: 0x38 */
	std::uint32_t RESERVED3;			/**  Reserved register - Address offset: 0x3C */
	volatile std::uint32_t APB1ENR;		/** RCC APB1 peripheral clock enable register (RCC_APB1ENR) - Address offset: 0x40 */
	volatile std::uint32_t APB2ENR;		/** RCC APB2 peripheral clock enable register (RCC_APB2ENR) - Address offset: 0x44 */
	std::uint32_t RESERVED4[2];			/**  Reserved register - Address offset: 0x48 to 0x4C */
	volatile std::uint32_t AHB1_LPENR;	/** RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR) -
	 *  Address offset: 0x50 */
	volatile std::uint32_t AHB2_LPENR;	/** RCC AHB2 peripheral clock enable in low power mode register
	 *  (RCC_AHB2LPENR) - Address offset: 0x54 */
	volatile std::int32_t AHB3_LPENR;	/** RCC AHB3 peripheral clock enable in low power mode register
	 *  (RCC_AHB3LPENR) - Address offset: 0x58 */
	std::uint32_t RESERVED5;			/**  Reserved register - Address offset: 0x5C */
	volatile std::uint32_t APB1_LPENR;	/** RCC APB1 peripheral clock enable in low power mode register
	 *  (RCC_APB1LPENR) - Address offset: 0x60 */
	volatile std::uint32_t APB2_LPENR;	/** RCC APB2 peripheral clock enabled in low power mode register
	 *  (RCC_APB2LPENR) - Address offset: 0x64 */
	std::uint32_t RESERVED6[2];			/**  Reserved register - Address offset: 0x68 to 0x6C */
	volatile std::uint32_t BDCR;		/** RCC Backup domain control register (RCC_BDCR) - Address offset: 0x70 */
	volatile std::uint32_t CSR;			/** RCC clock control & status register (RCC_CSR) - Address offset: 0x74 */
	std::uint32_t RESERVED7[2];			/**  Reserved register - Address offset: 0x78 to 0x7C */
	volatile std::uint32_t SS_CGR;		/** RCC spread spectrum clock generation register (RCC_SSCGR) - Address offset: 0x80 */
	volatile std::uint32_t PLLI2_SCFGR;	/** RCC PLLI2S configuration register (RCC_PLLI2SCFGR) - Address offset: 0x84 */
	volatile std::uint32_t PLL_SAI_CFGR;/** RCC PLL configuration register (RCC_PLLSAICFGR) - Address offset: 0x88 */
	volatile std::uint32_t DCK_CFGR;	/** RCC dedicated clock configuration register (RCC_DCKCFGR) - Address offset: 0x8C */
	volatile std::uint32_t CK_GATENR;	/** RCC clocks gated enable register (CKGATENR) - Address offset: 0x90 */
	volatile std::uint32_t DCK_CFGR2;	/** RCC dedicated clocks configuration register 2 (DCKCFGR2) - Address offset: 0x94 */
};

constexpr std::uint32_t RCC_BASE_ADDR = AHB1_PERIPH_BASE_ADDR + 0x3800UL;
RccRegisterDefinition* const RCC = reinterpret_cast<RccRegisterDefinition*>(RCC_BASE_ADDR);

#endif /* DRIVERS_INC_STM32F446XX_RCC_DRV_H_ */
