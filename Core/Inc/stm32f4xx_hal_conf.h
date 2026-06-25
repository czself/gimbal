#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_DISABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_CRC_MODULE_DISABLED
#define HAL_CRYP_MODULE_DISABLED
#define HAL_DAC_MODULE_DISABLED
#define HAL_DCMI_MODULE_DISABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_DMA2D_MODULE_DISABLED
#define HAL_ETH_MODULE_DISABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_NAND_MODULE_DISABLED
#define HAL_NOR_MODULE_DISABLED
#define HAL_PCCARD_MODULE_DISABLED
#define HAL_SRAM_MODULE_DISABLED
#define HAL_SDRAM_MODULE_DISABLED
#define HAL_HASH_MODULE_DISABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_DISABLED
#define HAL_I2S_MODULE_DISABLED
#define HAL_IWDG_MODULE_DISABLED
#define HAL_LTDC_MODULE_DISABLED
#define HAL_DSI_MODULE_DISABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_QSPI_MODULE_DISABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_DISABLED
#define HAL_RTC_MODULE_DISABLED
#define HAL_SAI_MODULE_DISABLED
#define HAL_SD_MODULE_DISABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_USART_MODULE_DISABLED
#define HAL_IRDA_MODULE_DISABLED
#define HAL_SMARTCARD_MODULE_DISABLED
#define HAL_WWDG_MODULE_DISABLED
#define HAL_PCD_MODULE_DISABLED
#define HAL_HCD_MODULE_DISABLED
#define HAL_FMPI2C_MODULE_DISABLED
#define HAL_SPDIFRX_MODULE_DISABLED
#define HAL_DFSDM_MODULE_DISABLED
#define HAL_LPTIM_MODULE_DISABLED
#define HAL_MMC_MODULE_DISABLED

#ifndef HSE_VALUE
#define HSE_VALUE        ((uint32_t)12000000U)
#endif
#define HSE_STARTUP_TIMEOUT  ((uint32_t)5000U)
#define HSI_VALUE        ((uint32_t)16000000U)
#define LSE_VALUE        ((uint32_t)32768U)
#define LSE_STARTUP_TIMEOUT  ((uint32_t)5000U)
#define LSI_VALUE        ((uint32_t)32000U)
#define VDD_VALUE        ((uint32_t)3300U)
#define TICK_INT_PRIORITY  ((uint32_t)0U)
#define USE_RTOS         0U
#define PREFETCH_ENABLE  1U
#define INSTRUCTION_CACHE_ENABLE  1U
#define DATA_CACHE_ENABLE  1U

#ifndef STM32F407xx
#define STM32F407xx
#endif

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_can.h"

#define assert_param(expr) ((void)0U)

#ifndef EXTERNAL_CLOCK_VALUE
#define EXTERNAL_CLOCK_VALUE    ((uint32_t)12288000U)
#endif

#endif