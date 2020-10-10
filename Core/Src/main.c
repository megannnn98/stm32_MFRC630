/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Adafruit_MFRC630.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Use the default I2C address */
//MFRC630 rfid = MFRC630();

/* Prints out len bytes of hex data in table format. */
static void print_buf_hex(uint8_t* buf, size_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        PRINT("0x%02x ", buf[i]);
    }
    PRINT("\r\n");
}

/*
 * This more concise loop show the minimim requirements to dump the first 39
 * 4 uint8_t blocks of memory from an NTAG2xx card. No meaningful error-handling
 * or debug output is present here, so this code is intended as a simple
 * starting point for further work.
 */
bool radio_ntag156b_dump_minimal(void)
{
    bool rc;

    /* Put the IC in a known-state. */
    mfrc630_softReset();

    /* Configure the radio for ISO14443A-106. */
    mfrc630_configRadio(mfrc630_RADIOCFG_ISO1443A_106);

    /* Request a tag (activates the near field, etc.). */
    uint16_t atqa = mfrc630_iso14443aRequest();

    /* Looks like we found a tag, move on to selection. */
    if (atqa) {
        /* NTAG has a ATQA of 00 44 (Ultralight does as well!). */
        if (atqa == 0x44) {
            uint8_t uid[10] = { 0 };
            uint8_t uidlen;
            uint8_t sak;

            /* Retrieve the UID and SAK values. */
            uidlen = mfrc630_iso14443aSelect(uid, &sak);
            PRINT("Found a tag with UUID ");
            for (uint8_t i = 0; i < uidlen; i++) {
                PRINT("0x%02x ", uid[i]);
            }

            PRINT("\r\n");
            if (uidlen == 7) {
                /* Try to read the first 42 pages from the card. */
                for (uint8_t i = 0; i < 42; i++) {
                    /* We should be able to read the page contents now. */
                    uint8_t pagebuf[4] = { 0, 0, 0, 0 };
                    uint8_t len = mfrc630_ntagReadPage(i, pagebuf);
                    PRINT("%d: ", i);
                    print_buf_hex(pagebuf, len);
                }
                rc = true;
            }
            else {
                /* Should be 7, not sure what kind of tag we have. */
                PRINT("Unexpected UID length: %d", uidlen);
                rc = false;
            }
        }
        else {
            /* Found a tag, but it isn't NTAG */
            PRINT("Unexpected ATQA value: %02", xatqa);
            rc = false;
        }
    }
    else {
        /* No tag found! */
        rc = false;
    }

    return rc;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  
  /* Try to initialize the IC */
  if (!(mfrc630_begin())) {
    
    PRINT("Unable to initialize the MFRC630. Check wiring?\r\n");
    while(1) {
//      digitalWrite(LED_BUILTIN, HIGH);
      HAL_Delay(50);
//      digitalWrite(LED_BUILTIN, LOW);
      HAL_Delay(50);
    }
  }

  /*
   * This will be INCREDIBLY chatty on the I2C bus, but can be used as a
   * quick test to wait until a card enters the near field.
   */
  PRINT("Waiting for an ISO14443-A compatible card ...\r\n");
  while (!radio_ntag156b_dump_minimal())
  {
    HAL_Delay(50);
  }
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
