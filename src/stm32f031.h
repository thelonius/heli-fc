#ifndef STM32F031_H
#define STM32F031_H

#include <stdint.h>

/* ============================================================
 * Memory Map
 * ============================================================ */
#define FLASH_MEM_BASE  0x08000000
#define SRAM_BASE       0x20000000
#define PERIPH_BASE     0x40000000

/* Volatile macro */
#ifndef __IO
#define __IO volatile
#endif

/* ============================================================
 * Cortex-M0 System
 * ============================================================ */
#define SCB_AIRCR       (*(volatile uint32_t *)0xE000ED0C)
#define NVIC_ISER       (*(volatile uint32_t *)0xE000E100)
#define NVIC_ICER       (*(volatile uint32_t *)0xE000E180)
#define NVIC_IPR        (*(volatile uint32_t *)0xE000E400)
#define STK_CSR         (*(volatile uint32_t *)0xE000E010)
#define STK_RVR         (*(volatile uint32_t *)0xE000E014)
#define STK_CVR         (*(volatile uint32_t *)0xE000E018)

#define AIRCR_KEY       0x05FA0000

/* ============================================================
 * RCC (0x40021000)
 * ============================================================ */
#define RCC_BASE        0x40021000
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR        (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CIR         (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_APB2RSTR    (*(volatile uint32_t *)(RCC_BASE + 0x0C))
#define RCC_APB1RSTR    (*(volatile uint32_t *)(RCC_BASE + 0x10))
#define RCC_AHBENR      (*(volatile uint32_t *)(RCC_BASE + 0x14))
#define RCC_APB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define RCC_BDCR        (*(volatile uint32_t *)(RCC_BASE + 0x20))
#define RCC_CSR         (*(volatile uint32_t *)(RCC_BASE + 0x24))
#define RCC_AHBRSTR     (*(volatile uint32_t *)(RCC_BASE + 0x28))
#define RCC_CFGR2       (*(volatile uint32_t *)(RCC_BASE + 0x2C))

/* RCC_AHBENR bits — STM32F031: GPIO at bits 17-22 */
#define RCC_AHBENR_GPIOAEN  (1 << 17)
#define RCC_AHBENR_GPIOBEN  (1 << 18)
#define RCC_AHBENR_DMA1EN   (1 << 0)

/* RCC_APB1ENR bits */
#define RCC_APB1ENR_TIM3EN  (1 << 1)
#define RCC_APB1ENR_TIM14EN (1 << 8)
#define RCC_APB1ENR_I2C1EN  (1 << 21)
#define RCC_APB1ENR_PWREN   (1 << 28)

/* RCC_APB2ENR bits */
#define RCC_APB2ENR_SPI1EN  (1 << 12)
#define RCC_APB2ENR_TIM1EN  (1 << 11)

/* ============================================================
 * GPIO (GPIOA: 0x48000000, GPIOB: 0x48000400)
 * ============================================================ */
#define GPIOA_BASE      0x48000000
#define GPIOB_BASE      0x48000400

#define GPIO_MODER(b)   (*(volatile uint32_t *)((b) + 0x00))
#define GPIO_OTYPER(b)  (*(volatile uint32_t *)((b) + 0x04))
#define GPIO_OSPEEDR(b) (*(volatile uint32_t *)((b) + 0x08))
#define GPIO_PUPDR(b)   (*(volatile uint32_t *)((b) + 0x0C))
#define GPIO_IDR(b)     (*(volatile uint32_t *)((b) + 0x10))
#define GPIO_ODR(b)     (*(volatile uint32_t *)((b) + 0x14))
#define GPIO_BSRR(b)    (*(volatile uint32_t *)((b) + 0x18))
#define GPIO_LCKR(b)    (*(volatile uint32_t *)((b) + 0x1C))
#define GPIO_AFRL(b)    (*(volatile uint32_t *)((b) + 0x20))
#define GPIO_AFRH(b)    (*(volatile uint32_t *)((b) + 0x24))

#define GPIOA_MODER     GPIO_MODER(GPIOA_BASE)
#define GPIOA_OTYPER    GPIO_OTYPER(GPIOA_BASE)
#define GPIOA_OSPEEDR   GPIO_OSPEEDR(GPIOA_BASE)
#define GPIOA_PUPDR     GPIO_PUPDR(GPIOA_BASE)
#define GPIOA_IDR       GPIO_IDR(GPIOA_BASE)
#define GPIOA_ODR       GPIO_ODR(GPIOA_BASE)
#define GPIOA_BSRR      GPIO_BSRR(GPIOA_BASE)
#define GPIOA_AFRL      GPIO_AFRL(GPIOA_BASE)
#define GPIOA_AFRH      GPIO_AFRH(GPIOA_BASE)

/* BRR: STM32F0 has no BRR; BSRR[31:16] resets pins.
 * Use BSRR with (pin << 16) to reset. */
#define GPIO_BRR(port_base, pin)  (*(volatile uint32_t *)((port_base) + 0x18) = (1 << ((pin) + 16)))

#define GPIOB_MODER     GPIO_MODER(GPIOB_BASE)
#define GPIOB_OTYPER    GPIO_OTYPER(GPIOB_BASE)
#define GPIOB_OSPEEDR   GPIO_OSPEEDR(GPIOB_BASE)
#define GPIOB_PUPDR     GPIO_PUPDR(GPIOB_BASE)
#define GPIOB_IDR       GPIO_IDR(GPIOB_BASE)
#define GPIOB_ODR       GPIO_ODR(GPIOB_BASE)
#define GPIOB_BSRR      GPIO_BSRR(GPIOB_BASE)
#define GPIOB_AFRL      GPIO_AFRL(GPIOB_BASE)
#define GPIOB_AFRH      GPIO_AFRH(GPIOB_BASE)
#define GPIOB_BRR       GPIO_BSRR(GPIOB_BASE)

/* ============================================================
 * TIM2 (0x40000000) — Swashplate Servos
 * ============================================================ */
#define TIM2_BASE       0x40000000
#define TIM2_CR1        (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_CR2        (*(volatile uint32_t *)(TIM2_BASE + 0x04))
#define TIM2_SMCR       (*(volatile uint32_t *)(TIM2_BASE + 0x08))
#define TIM2_DIER       (*(volatile uint32_t *)(TIM2_BASE + 0x0C))
#define TIM2_SR         (*(volatile uint32_t *)(TIM2_BASE + 0x10))
#define TIM2_EGR        (*(volatile uint32_t *)(TIM2_BASE + 0x14))
#define TIM2_CCMR1      (*(volatile uint32_t *)(TIM2_BASE + 0x18))
#define TIM2_CCMR2      (*(volatile uint32_t *)(TIM2_BASE + 0x1C))
#define TIM2_CCER       (*(volatile uint32_t *)(TIM2_BASE + 0x20))
#define TIM2_CNT        (*(volatile uint32_t *)(TIM2_BASE + 0x24))
#define TIM2_PSC        (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR        (*(volatile uint32_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCR1       (*(volatile uint32_t *)(TIM2_BASE + 0x34))
#define TIM2_CCR2       (*(volatile uint32_t *)(TIM2_BASE + 0x38))
#define TIM2_CCR3       (*(volatile uint32_t *)(TIM2_BASE + 0x3C))
#define TIM2_CCR4       (*(volatile uint32_t *)(TIM2_BASE + 0x40))

/* ============================================================
 * TIM3 (0x40000400) — Tail Motor
 * ============================================================ */
#define TIM3_BASE       0x40000400
#define TIM3_CR1        (*(volatile uint32_t *)(TIM3_BASE + 0x00))
#define TIM3_CR2        (*(volatile uint32_t *)(TIM3_BASE + 0x04))
#define TIM3_SMCR       (*(volatile uint32_t *)(TIM3_BASE + 0x08))
#define TIM3_DIER       (*(volatile uint32_t *)(TIM3_BASE + 0x0C))
#define TIM3_SR         (*(volatile uint32_t *)(TIM3_BASE + 0x10))
#define TIM3_EGR        (*(volatile uint32_t *)(TIM3_BASE + 0x14))
#define TIM3_CCMR1      (*(volatile uint32_t *)(TIM3_BASE + 0x18))
#define TIM3_CCMR2      (*(volatile uint32_t *)(TIM3_BASE + 0x1C))
#define TIM3_CCER       (*(volatile uint32_t *)(TIM3_BASE + 0x20))
#define TIM3_CNT        (*(volatile uint32_t *)(TIM3_BASE + 0x24))
#define TIM3_PSC        (*(volatile uint32_t *)(TIM3_BASE + 0x28))
#define TIM3_ARR        (*(volatile uint32_t *)(TIM3_BASE + 0x2C))
#define TIM3_CCR1       (*(volatile uint32_t *)(TIM3_BASE + 0x34))
#define TIM3_CCR2       (*(volatile uint32_t *)(TIM3_BASE + 0x38))
#define TIM3_CCR3       (*(volatile uint32_t *)(TIM3_BASE + 0x3C))
#define TIM3_CCR4       (*(volatile uint32_t *)(TIM3_BASE + 0x40))
#define TIM3_BDTR       (*(volatile uint32_t *)(TIM3_BASE + 0x44))

/* ============================================================
 * TIM14 (0x40002000) — Auxiliary Timer
 * ============================================================ */
#define TIM14_BASE      0x40002000
#define TIM14_CR1       (*(volatile uint32_t *)(TIM14_BASE + 0x00))
#define TIM14_DIER      (*(volatile uint32_t *)(TIM14_BASE + 0x0C))
#define TIM14_SR        (*(volatile uint32_t *)(TIM14_BASE + 0x10))
#define TIM14_EGR       (*(volatile uint32_t *)(TIM14_BASE + 0x14))
#define TIM14_CCMR1     (*(volatile uint32_t *)(TIM14_BASE + 0x18))
#define TIM14_CCER      (*(volatile uint32_t *)(TIM14_BASE + 0x20))
#define TIM14_CNT       (*(volatile uint32_t *)(TIM14_BASE + 0x24))
#define TIM14_PSC       (*(volatile uint32_t *)(TIM14_BASE + 0x28))
#define TIM14_ARR       (*(volatile uint32_t *)(TIM14_BASE + 0x2C))
#define TIM14_CCR1      (*(volatile uint32_t *)(TIM14_BASE + 0x34))

/* ============================================================
 * TIM1 (0x40012C00) — Advanced Timer
 * ============================================================ */
#define TIM1_BASE       0x40012C00
#define TIM1_CR1        (*(volatile uint32_t *)(TIM1_BASE + 0x00))
#define TIM1_CR2        (*(volatile uint32_t *)(TIM1_BASE + 0x04))
#define TIM1_SMCR       (*(volatile uint32_t *)(TIM1_BASE + 0x08))
#define TIM1_DIER       (*(volatile uint32_t *)(TIM1_BASE + 0x0C))
#define TIM1_SR         (*(volatile uint32_t *)(TIM1_BASE + 0x10))
#define TIM1_EGR        (*(volatile uint32_t *)(TIM1_BASE + 0x14))
#define TIM1_CCMR1      (*(volatile uint32_t *)(TIM1_BASE + 0x18))
#define TIM1_CCMR2      (*(volatile uint32_t *)(TIM1_BASE + 0x1C))
#define TIM1_CCER       (*(volatile uint32_t *)(TIM1_BASE + 0x20))
#define TIM1_CNT        (*(volatile uint32_t *)(TIM1_BASE + 0x24))
#define TIM1_PSC        (*(volatile uint32_t *)(TIM1_BASE + 0x28))
#define TIM1_ARR        (*(volatile uint32_t *)(TIM1_BASE + 0x2C))
#define TIM1_RCR        (*(volatile uint32_t *)(TIM1_BASE + 0x30))
#define TIM1_CCR1       (*(volatile uint32_t *)(TIM1_BASE + 0x34))
#define TIM1_CCR2       (*(volatile uint32_t *)(TIM1_BASE + 0x38))
#define TIM1_CCR3       (*(volatile uint32_t *)(TIM1_BASE + 0x3C))
#define TIM1_CCR4       (*(volatile uint32_t *)(TIM1_BASE + 0x40))
#define TIM1_BDTR       (*(volatile uint32_t *)(TIM1_BASE + 0x44))

/* ============================================================
 * SPI1 (0x40013000)
 * ============================================================ */
#define SPI1_BASE       0x40013000
#define SPI1_CR1        (*(volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_CR2        (*(volatile uint32_t *)(SPI1_BASE + 0x04))
#define SPI1_SR         (*(volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR         (*(volatile uint32_t *)(SPI1_BASE + 0x0C))

/* ============================================================
 * I2C1 (0x40005400)
 * ============================================================ */
#define I2C1_BASE       0x40005400
#define I2C1_CR1        (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2        (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_OAR1       (*(volatile uint32_t *)(I2C1_BASE + 0x08))
#define I2C1_OAR2       (*(volatile uint32_t *)(I2C1_BASE + 0x0C))
#define I2C1_TIMINGR    (*(volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C1_ISR        (*(volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C1_ICR        (*(volatile uint32_t *)(I2C1_BASE + 0x1C))

/* ============================================================
 * DMA1 (0x40020000)
 * ============================================================ */
#define DMA1_BASE       0x40020000
#define DMA1_ISR        (*(volatile uint32_t *)(DMA1_BASE + 0x00))
#define DMA1_IFCR       (*(volatile uint32_t *)(DMA1_BASE + 0x04))
#define DMA1_CCR1       (*(volatile uint32_t *)(DMA1_BASE + 0x08))
#define DMA1_CNDTR1     (*(volatile uint32_t *)(DMA1_BASE + 0x0C))
#define DMA1_CPAR1      (*(volatile uint32_t *)(DMA1_BASE + 0x10))
#define DMA1_CMAR1      (*(volatile uint32_t *)(DMA1_BASE + 0x14))

/* ============================================================
 * FLASH Registers (0x40022000)
 * ============================================================ */
#define FLASH_REG_BASE  0x40022000
#define FLASH_ACR       (*(volatile uint32_t *)(FLASH_REG_BASE + 0x00))
#define FLASH_KEYR      (*(volatile uint32_t *)(FLASH_REG_BASE + 0x04))
#define FLASH_SR        (*(volatile uint32_t *)(FLASH_REG_BASE + 0x0C))
#define FLASH_CR        (*(volatile uint32_t *)(FLASH_REG_BASE + 0x10))
#define FLASH_AR        (*(volatile uint32_t *)(FLASH_REG_BASE + 0x14))

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

/* ============================================================
 * IWDG (0x40003000)
 * ============================================================ */
#define IWDG_BASE       0x40003000
#define IWDG_KR         (*(volatile uint32_t *)(IWDG_BASE + 0x00))
#define IWDG_PR         (*(volatile uint32_t *)(IWDG_BASE + 0x04))
#define IWDG_RLR        (*(volatile uint32_t *)(IWDG_BASE + 0x08))
#define IWDG_SR         (*(volatile uint32_t *)(IWDG_BASE + 0x0C))

#define IWDG_REFRESH    0x0000AAAA
#define IWDG_WRITE_EN   0x00005555
#define IWDG_UNLOCK     0x0000CCCC

/* ============================================================
 * PWR (0x40007000)
 * ============================================================ */
#define PWR_BASE        0x40007000
#define PWR_CR          (*(volatile uint32_t *)(PWR_BASE + 0x00))

#endif /* STM32F031_H */
