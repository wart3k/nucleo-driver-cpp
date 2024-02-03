#ifndef DRIVERS_INC_STM32F446XX_INTERRUPT_H_
#define DRIVERS_INC_STM32F446XX_INTERRUPT_H_

#include <cstdint>

#include "stm32f446xx_memory_bus_architecture.h"

/**
 * Peripheral register definition structure for External interrupt/event controller (EXTI)
 *
 * @brief: The external interrupt/event controller consists of up to 23 edge detectors for generating event/interrupt
 * requests. Each input line can be independently configured to select the type (interrupt or event) and the corresponding
 * trigger event (rising or falling or both). Each line can also masked independently. A pending register maintains the
 * status line of the interrupt requests.
 */
struct ExtiRegisterDefinition {
	volatile std::uint32_t IMR;			/** Interrupt mask register (EXTI_IMR) - Address offset: 0x00 */
	volatile std::uint32_t EMR;			/** Event mask register (EXTI_EMR) - Address offset: 0x04 */
	volatile std::uint32_t RTSR;		/** Rising trigger selection register (EXTI_RTSR) - Address offset: 0x08 */
	volatile std::uint32_t FTSR;		/** Falling trigger selection register (EXTI_FTSR) - Address offset: 0x0C */
	volatile std::uint32_t SWIER;		/** Software interrupt event register (EXTI_SWIER) - Address offset: 0x10 */
	volatile std::uint32_t PR;			/** Pending register (EXTI_PR) - Address offset: 0x14 */
};

/**
 * Interrupt numbers
 */
constexpr std::uint8_t IRQ_EXTI0_NUMBER	= 6;
constexpr std::uint8_t IRQ_EXTI2_NUMBER	= 8;
constexpr std::uint8_t IRQ_EXTI1_NUMBER	= 7;
constexpr std::uint8_t IRQ_EXTI3_NUMBER	= 9;
constexpr std::uint8_t IRQ_EXTI4_NUMBER	= 10;
constexpr std::uint8_t IRQ_EXTI9_5_NUMBER = 23;
constexpr std::uint8_t IRQ_EXTI15_10_NUMBER	= 40;

constexpr std::uint32_t EXTI_BASE_ADDR = APB2_PERIPH_BASE_ADDR + 0x3C00UL;
ExtiRegisterDefinition* const EXTI = reinterpret_cast<ExtiRegisterDefinition*>(EXTI_BASE_ADDR);

/**
 * ARM Cortex M4 processor NVIC ISERx register addresses
 */
volatile std::uint32_t* const NVIC_ISER0 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E100UL );
volatile std::uint32_t* const NVIC_ISER1 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E104UL );
volatile std::uint32_t* const NVIC_ISER2 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E108UL );
volatile std::uint32_t* const NVIC_ISER3 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E10CUL );
volatile std::uint32_t* const NVIC_ISER4 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E110UL );
volatile std::uint32_t* const NVIC_ISER5 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E114UL );
volatile std::uint32_t* const NVIC_ISER6 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E118UL );
volatile std::uint32_t* const NVIC_ISER7 = reinterpret_cast<volatile std::uint32_t*>( 0xE000E11CUL );

volatile std::uint32_t* const NVIC_ICER0 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E180UL );
volatile std::uint32_t* const NVIC_ICER1 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E184UL );
volatile std::uint32_t* const NVIC_ICER2 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E188UL );
volatile std::uint32_t* const NVIC_ICER3 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E18CUL );
volatile std::uint32_t* const NVIC_ICER4 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E190UL );
volatile std::uint32_t* const NVIC_ICER5 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E194UL );
volatile std::uint32_t* const NVIC_ICER6 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E198UL );
volatile std::uint32_t* const NVIC_ICER7 = reinterpret_cast<volatile std::uint32_t*> ( 0XE000E19CUL );

constexpr std::uint8_t NO_OF_IMPLEMENTED_PRIORITY_BITS = 4;
uint32_t* const NVIC_PRIORITY_BASE_ADDR = reinterpret_cast<uint32_t*>(0xE000E400UL);

#endif /* DRIVERS_INC_STM32F446XX_INTERRUPT_H_ */
