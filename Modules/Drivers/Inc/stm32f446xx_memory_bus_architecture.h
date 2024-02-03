#ifndef DRIVERS_INC_STM32F446XX_MEMORY_BUS_ARCHITECTURE_H_
#define DRIVERS_INC_STM32F446XX_MEMORY_BUS_ARCHITECTURE_H_

#include <cstdint>

/**
 * base addresses of flash, SRAM and ROM memories
 * For more information's see RM0390 2.2.2 Memory map and register boundary addresses
 */
constexpr std::uint32_t FLASH_BASE_ADDR 		= 0x20000000UL;
constexpr std::uint32_t SRAM1_BASE_ADDR 		= 0x2001C000UL;
constexpr std::uint32_t ROM_BASE_ADDR 			= 0x1FFF0000UL;
constexpr std::uint32_t SRAM 					= SRAM1_BASE_ADDR;

/**
 * AHBx and APBx Bus peripheral base addresses
 * For more informations see RM0390 2.2.2 Memory map and register boundary addresses
 */
constexpr std::uint32_t PERIPH_BASE_ADDR		= 0x40000000UL;
constexpr std::uint32_t APB1_PERIPH_BASE_ADDR 	= PERIPH_BASE_ADDR;
constexpr std::uint32_t APB2_PERIPH_BASE_ADDR 	= 0x40010000UL;
constexpr std::uint32_t AHB1_PERIPH_BASE_ADDR 	= 0x40020000UL;
constexpr std::uint32_t AHB2_PERIPH_BASE_ADDR 	= 0x50000000UL;
constexpr std::uint32_t AHB3_PERIPH_BASE_ADDR 	= 0x60000000UL;

#endif /* DRIVERS_INC_STM32F446XX_MEMORY_BUS_ARCHITECTURE_H_ */
