/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 geosmall
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+

#define LED_PORT              GPIOE
#define LED_PIN               GPIO_PIN_3
#define LED_STATE_ON          1

//--------------------------------------------------------------------+
// BUTTON
//--------------------------------------------------------------------+

#define BUTTON_PORT           GPIOC
#define BUTTON_PIN            GPIO_PIN_13
#define BUTTON_STATE_ACTIVE   0

//--------------------------------------------------------------------+
// Flash
//--------------------------------------------------------------------+

// H743 has 2MB internal flash (dual-bank, 128KB sectors)
#define BOARD_FLASH_SIZE      (2 * 1024 * 1024)
#define BOARD_FLASH_SECTORS   16  // 8 per bank x 2 banks

// App starts at second 128KB sector
#define BOARD_FLASH_APP_START 0x08020000

//--------------------------------------------------------------------+
// USB UF2
//--------------------------------------------------------------------+

#define USB_VID               0x239A
#define USB_PID               0x006C
#define USB_MANUFACTURER      "MATEK"
#define USB_PRODUCT           "H743-WLITE"

#define UF2_PRODUCT_NAME      USB_MANUFACTURER " " USB_PRODUCT
#define UF2_BOARD_ID          "STM32H743-MATEK-H743"
#define UF2_VOLUME_LABEL      "H743BOOT"
#define UF2_INDEX_URL         "https://github.com/geosmall/tinyuf2"

#define USB_NO_VBUS_PIN       1
#define BOARD_TUD_RHPORT      0

//--------------------------------------------------------------------+
// UART (optional debug)
//--------------------------------------------------------------------+

#define UART_DEV              USART1
#define UART_CLOCK_ENABLE     __HAL_RCC_USART1_CLK_ENABLE
#define UART_CLOCK_DISABLE    __HAL_RCC_USART1_CLK_DISABLE
#define UART_GPIO_PORT        GPIOA
#define UART_GPIO_AF          GPIO_AF7_USART1
#define UART_TX_PIN           GPIO_PIN_9
#define UART_RX_PIN           GPIO_PIN_10

//--------------------------------------------------------------------+
// RCC Clock - 480 MHz from 8 MHz HSE
//--------------------------------------------------------------------+

static inline void clock_init(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  // HAL_Init() equivalent - required for clock config on H7
  HAL_Init();

  // Configure power supply
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  // Configure voltage scaling for max frequency (VOS0)
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  // Macro to configure the PLL clock source (from working Arduino config)
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  // Configure HSE, HSI48 and PLL
  // 8 MHz HSE -> 480 MHz SYSCLK
  // PLLM=1, PLLN=120, PLLP=2 -> 8/1*120/2 = 480 MHz
  // HSI48 required for USB
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1) {}
  }

  // Configure bus clocks (matching Arduino working config order)
  // SYSCLK=480MHz, HCLK=240MHz, APBx=120MHz
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    while (1) {}
  }

  // Set flash programming delay for 240 MHz AXI clock (WRHIGHFREQ = 2)
  // Required: reset default value (3) is INVALID and causes flash programming failures
  // Per RM0433 Table 17: DELAY_2 is for AXI 185-225 MHz range
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);

  // Configure USB clock from HSI48
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    while (1) {}
  }

  // Enable USB voltage detector
  HAL_PWREx_EnableUSBVoltageDetector();
}

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
