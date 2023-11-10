/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "oled.h"
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__IO uint8_t RawKey[3];
uint8_t snake_head_x = 0;
uint8_t snake_head_y = 0;
uint8_t snake_tail_x = 0;
uint8_t snake_tail_y = 0;
uint8_t snake_tail_next[2];
uint16_t score = 0;
uint8_t score_str[8];
__IO uint8_t snake_direction;
// uint8_t map[128*56]={0};
uint8_t block_map[128*6]={0};
struct SNAKE_NODE snake_node[7000];
struct SNAKE_NODE food;
uint16_t snake_len;
__IO uint8_t fSnakeMove;
uint8_t diffcu = 6;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint16_t font_data[8];
__IO uint8_t fRecive_key;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void font_add()
{
  for (uint8_t i = 0; i < 8; i++)
  {
    font_data[i] = font_data[i] + 1;
    //printf_UART_DMA(&huart1, "%X ", font_data[i]);
  }
  printf_UART_DMA(&huart1, "\r\n");
}

void OLED_display_snake_xy(uint8_t x, uint8_t y, uint8_t status) // 128*56
{
  uint8_t block_x = x;
  uint8_t block_y = y / 8;
  if (status == 1) // set pixel
  {
    block_map[block_x + 128 * block_y] |= (0x01 << (y % 8));
  }
  else // clear pixel
  {
    block_map[block_x + 128 * block_y] &= ~(0x01 << (y % 8));
  }
  OLED_Display_block(block_x, block_y + 2, block_map[block_x + 128 * block_y]);
}

static uint8_t check_snake_point_x(uint8_t x)
{
  for(uint8_t i = 0;i<snake_len;i++)
  {
    if(x == snake_node[i].x)
    return 1;
  }
}

static uint8_t check_snake_point_y(uint8_t y)
{

}

static uint8_t check_point_on_snake(uint8_t x,uint8_t y)
{
  for (uint8_t i = 0; i < snake_len; i++)
  {
    if((snake_node[i].x == x)&&(snake_node[i].y == y))
    {
      return 1;
    }
  }
  return 0;
}

static void food_respawn()
{
  while (1)
  {
    food.x = rand()%127;
    food.y = rand()%47;
    if(!check_point_on_snake(food.x,food.y))
    {
      break;
    }
  }
  
  
  if(food.x > 127)
  {
    food.x -= 127;
  }
  if(food.x > 47)
  {
    food.x -= 47;
  }
  
  printf_UART_DMA(&huart1,"food respwan: X:%d Y:%d\r\n",food.x,food.y);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  const uint8_t AMD[] = {0x01, 0x83, 0xC7, 0xEF, 0xFF, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF8, 0xF8, 0x78, 0x38, 0x18, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};
  const uint8_t Intel[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0x70, 0x70, 0x78, 0x38, 0x38, 0x3C, 0x1C, 0x1C, 0x1E, 0x1E, 0x1E, 0x0E, 0x0E, 0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x0F, 0x0F, 0x0F, 0x0F, 0x1F, 0x1E, 0x1E, 0x3E, 0x3C, 0x7C, 0x7C, 0xF8, 0xF8, 0xF0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0x07, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0x78, 0x78, 0x78, 0x31, 0x03, 0x0F, 0x7F, 0xFF, 0xFF, 0xFE, 0xF8, 0x80, 0xE0, 0xF8, 0xFE, 0x7F, 0x0F, 0x07, 0x01, 0x00, 0x00, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x00, 0x00, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x1E, 0x1E, 0x1E, 0x3E, 0xFE, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1E, 0x1E, 0x1E, 0xF0, 0xF8, 0xFC, 0xFE, 0xFE, 0x9E, 0x9E, 0x9E, 0xBE, 0xFE, 0xFE, 0xFC, 0xF8, 0xE0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xC7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0xF3, 0xF3, 0xE3, 0xF3, 0xF3, 0xFB, 0xF3, 0x73, 0x63, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xF8, 0xFC, 0xFE, 0x7F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03, 0x01, 0x00, 0x0F, 0x3F, 0x7F, 0xFF, 0xFE, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x81, 0x81, 0x80, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF8, 0xF8, 0xFC, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x07, 0x0F, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7E, 0x7E, 0x7E, 0x7E, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFE, 0x7E, 0x7E, 0x7E, 0x7F, 0x7F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x1F, 0x1F, 0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t blank[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  for (uint8_t i = 0; i < 8; i++)
  {
    font_data[i] = i;
  }
  uint8_t current_step = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  uint8_t x_pos = 0;
  printf_UART_DMA(&huart1, "init finished\n\r");
  OLED_Display_GB2312_string(32, 2, "ÊäÈëÃû×Ö",0);
  char name[10] = {""};
  uint8_t name_len = 0;
  srand(1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    switch (current_step)
    {
    case 0: // enter name
      while (fRecive_key)
      {
        fRecive_key = 0;
        if (UART1_DMA_RX_buffer[0] != 0x0D) // Enter
        {
          if (name_len < 10)
          {
            strcat(name, &RawKey[0]);
            name_len++;
          }
          OLED_Display_GB2312_string(0, 4, name, 0);
        }
        else
        {
          current_step++;
        }
      }
      break;
    case 1: // init game
      OLED_Clear();
      OLED_address(0, 0);
      for (uint8_t i = 0; i < 88; i++)
      {
        OLED_WR_Byte(0xff, OLED_DATA);
      }
      OLED_address(0, 1);
      for (uint8_t i = 0; i < 88; i++)
      {
        OLED_WR_Byte(0xff, OLED_DATA);
      }
      OLED_Display_GB2312_string(0, 0, name, 1);
      //init snake
      snake_direction = Right;
      snake_head_x = 62;
      snake_head_y = 30;
      snake_tail_x = 32;
      snake_tail_y = 30;
      snake_len = 30;
      for (uint8_t i = 0; i < 30; i++)
      {
        // map[128 * 30 + 32 + i] = 1; // tail
        snake_node[i].x = 32 + i;
        snake_node[i].y = 30;
        OLED_display_snake_xy(32 + i, snake_head_y, 1);
      }
      food.x = 99;
      food.y = snake_head_y;
      OLED_display_snake_xy(food.x,food.y,1);
      current_step++;
      break;
    case 2:
      sprintf(score_str, "%d", score);
      if (fSnakeMove) // change direction
      {
        fSnakeMove = 0;
        switch (UART1_DMA_RX_buffer[0])
        {
        case 0x77: // w
        case 0x57: // W
          if (snake_direction != Down)
          {
            snake_direction = Up;
          }
          break;
        case 0x73:
        case 0x53:
          if (snake_direction != Up)
          {
            snake_direction = Down;
          }
          break;
        case 0x61:
        case 0x41:
          if (snake_direction != Right)
          {
            snake_direction = Left;
          }
          break;
        case 0x64:
        case 0x44:
          if (snake_direction != Left)
          {
            snake_direction = Right;
          }
          break;
        default:
          break;
        }
      // }
      // if (fSnakeMove)
      // {
      //   fSnakeMove = 0;
        switch (snake_direction)
        {
        case Up:
          snake_node[snake_len].x = snake_node[snake_len - 1].x;
          if (snake_node[snake_len - 1].y == 0)
          {
            snake_node[snake_len].y = 48 - 1;
          }
          else
          {
            snake_node[snake_len].y = snake_node[snake_len - 1].y - 1;
          }
          break;
        case Down:
          snake_node[snake_len].x = snake_node[snake_len - 1].x;
          if (snake_node[snake_len - 1].y == 48 - 1)
          {
            snake_node[snake_len].y = 0;
          }
          else
          {
            snake_node[snake_len].y = snake_node[snake_len - 1].y + 1;
          }
          break;
        case Left:
          if (snake_node[snake_len - 1].x == 0)
          {
            snake_node[snake_len].x = 127;
          }
          else
          {
            snake_node[snake_len].x = snake_node[snake_len - 1].x - 1;
          }
          snake_node[snake_len].y = snake_node[snake_len - 1].y;
          break;
        case Right:
          if (snake_node[snake_len - 1].x == 127)
          {
            snake_node[snake_len].x = 0;
          }
          else
          {
            snake_node[snake_len].x = snake_node[snake_len - 1].x + 1;
          }
          snake_node[snake_len].y = snake_node[snake_len - 1].y;
          break;
        default:
          break;
        }
        printf_UART_DMA(&huart1, "head:X:%d Y:%d   tail:X:%d Y:%d  length:%d\n\r", snake_node[snake_len].x, snake_node[snake_len].y, snake_node[0].x, snake_node[0].y,snake_len);
        OLED_display_snake_xy(food.x, food.y, 1);
        OLED_display_snake_xy(snake_node[snake_len].x, snake_node[snake_len].y, 1);
        OLED_display_snake_xy(snake_node[0].x, snake_node[0].y, 0);
        if(check_point_on_snake(snake_node[snake_len].x,snake_node[snake_len].y))
        {
          current_step++;
          break;
        }
        if ((snake_node[snake_len].x == food.x) && (snake_node[snake_len].y == food.y))
        {
          score++;
          snake_len++;
          OLED_Display_GB2312_string(128 - 5 * 8, 0, score_str, 0);
          food_respawn();
        }
        else
        {
          for (uint16_t i = 0; i < snake_len + 1; i++)
          {
            snake_node[i] = snake_node[i + 1];
          }
        }
      }
      break;
      case 3:
      break;
    default:
      break;
    }

    // OLED_Display_16x16(x_pos, 0, AMD);
    // OLED_Display_xy(x_pos, 2, 73, 48, Intel);
    // OLED_Display_GB2312_string(x_pos - 8, 0, " ");
    // OLED_Display_xy(x_pos - 1, 2, 1, 48, blank);
    // x_pos++;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

