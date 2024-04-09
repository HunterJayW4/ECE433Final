/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341_gfx.h"
#include "testimg.h"
#include "pongColor.h"
#include "ili9341_font.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ICACHE_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
//static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ICACHE_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_UCPD1_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  //MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  setClks();
  setupADC();
  LPUART1init();
  char  txt [20];

  ili9341_t *ili9341_display = ili9341_new(
	  &hspi1, // SPI handle
	  GPIOB, GPIO_PIN_0, // Reset pin -> PB0
	  GPIOA, GPIO_PIN_4, // CS (Chip Select) pin -> PA4
	  GPIOB, GPIO_PIN_2, // DC (Data/Command) pin -> PB2
	  isoLandscape, // Screen orientation
	  NULL, 0, // Touch CS pin: Not used
	  NULL, 0, // Touch IRQ pin: Not used
	  itsNotSupported, // Touch support: Not supported
	  itnNONE // Touch normalization: Not applicable
  );

  //ili9341_fill_screen(ili9341_display, ILI9341_BLACK);
  ili9341_draw_bitmap_1b(ili9341_display, ILI9341_WHITE, ILI9341_BLACK, 0, 0, 320, 240, image_data_Pong);
  HAL_Delay(500);


  // Assuming `lcd` is an already initialized ili9341_t instance
  ili9341_color_t color = ILI9341_WHITE;
  uint16_t width = 7;  // Width of the rectangle
  uint16_t height = 40;  // Height of the rectangle
  int16_t playerX = 20;  // X coordinate of the top-left corner of the rectangle
  int16_t playerY = 50;  // Y coordinate of the top-left corner of the rectangle
  int16_t botX = (320 - playerX) - width; // X coordinate of the top-left corner of the bot rectangle
  int16_t botY = (240 - playerY) - height; // Y coordinate of the top-left corner of the bot rectangle
  int8_t botSpeed = 3;
  int16_t direction = 1;  // Initial direction of movement: 1 for moving down, -1 for moving up
  int8_t movement_distance;
  uint8_t tmpCalc;
  int16_t new_y;
  int8_t gameSpeed = 5;
  int16_t ballX = 160;
  int16_t ballY = 120;
  int16_t velocityX = 2;  // Initial velocity in the x-direction
  int16_t velocityY = 2;  // Initial velocity in the y-direction
  uint8_t playerScoreNum = 0;
  uint8_t botScoreNum = 0;


  // Define the text attributes
  ili9341_text_attr_t playerScore;
  playerScore.origin_x = 80;  // X coordinate of the top-left corner of the character
  playerScore.origin_y = 10;  // Y coordinate of the top-left corner of the character
  playerScore.fg_color = ILI9341_GREEN;  // Color of the character
  playerScore.bg_color = ILI9341_BLACK;  // Background color
  playerScore.font = &ili9341_font_16x26;  // Use the desired font from ili9341_font.h

  // Define the text attributes
  ili9341_text_attr_t botScore;
  botScore.origin_x = 240;  // X coordinate of the top-left corner of the character
  botScore.origin_y = 10;  // Y coordinate of the top-left corner of the character
  botScore.fg_color = ILI9341_RED;  // Color of the character
  playerScore.bg_color = ILI9341_BLACK;  // Background color
  botScore.font = &ili9341_font_16x26;  // Use the desired font from ili9341_font.h



  //Values for potentiometer value:
  uint16_t potentiometer_value = 0;

  void readADC2(void) {
  // ADC1 is now enabled and ready for conversion
	bitset(ADC1->CR, 2); // Start Conversion

	// EOC Flag: wait for conversion complete
	while(bitcheck(ADC1->ISR, 2)==0);

	// read conversion result  (Reading DR clear EOC flag)
	potentiometer_value = (ADC1->DR);
  }

  void drawArena() {
	  ili9341_fill_screen(ili9341_display, ILI9341_BLACK);

	  // Draw a vertical line on the left edge of the screen
	  ili9341_fill_rect(ili9341_display, ILI9341_RED, 0, 0, 4, 240);

	  // Draw a vertical line on the right edge of the screen
	  ili9341_fill_rect(ili9341_display, ILI9341_GREEN, 316, 0, 4, 240);

	  // Draw the dotted line
	  for (int16_t yLine = 0; yLine < 240; yLine += 10) {
	      ili9341_fill_rect(ili9341_display, ILI9341_WHITE, 159, yLine, 2, 5);
	  }

	  // Draw the character 'A' at the specified coordinates with the defined attributes
	  ili9341_draw_char(ili9341_display, botScore, botScoreNum);
	  ili9341_draw_char(ili9341_display, playerScore, playerScoreNum);

  }

  void updateArena() {
	  // Draw a vertical line on the left edge of the screen
	  ili9341_fill_rect(ili9341_display, ILI9341_RED, 0, 0, 4, 240);

	  // Draw a vertical line on the right edge of the screen
	  ili9341_fill_rect(ili9341_display, ILI9341_GREEN, 316, 0, 4, 240);

	  // Draw the dotted line
	  for (int16_t yLine = 0; yLine < 240; yLine += 10) {
	      ili9341_fill_rect(ili9341_display, ILI9341_WHITE, 159, yLine, 2, 5);
	  }

	  // Draw the character 'A' at the specified coordinates with the defined attributes
	  ili9341_draw_char(ili9341_display, botScore, (char)botScoreNum);
	  ili9341_draw_char(ili9341_display, playerScore, (char)playerScoreNum);
  }

  void updateBallPosition() {
      // Clear the old ball position
      ili9341_fill_circle(ili9341_display, ILI9341_BLACK, ballX, ballY, 4);
      uint8_t side = 0;
      // Update the position of the ball based on its velocity
      ballX += velocityX;
      ballY += velocityY;

      // Check if the ball hits the sides of the screen
      if (ballX <= 0 || ballX >= 320) {
          // If it hits the left or right side, reverse its x-velocity
          velocityX *= -1;

          if (ballX >= 320)
        	  playerScoreNum++;

          if (ballX <= 0)
        	  botScoreNum++;

      }

      if (ballY <= 0 || ballY >= 240) {
          // If it hits the top or bottom side, reverse its y-velocity
          velocityY *= -1;
      }

      // Check for collision with the player's rectangle
      if (ballX + 4 >= playerX && ballX - 4 <= playerX + width &&
          ballY + 4 >= playerY && ballY - 4 <= playerY + height) {
          // Collision detected with the player's rectangle, reverse the ball's velocity
          // You can adjust the angle of reflection here if needed
          velocityX *= -1;
          velocityY *= -1;
      }

      // Check for collision with the bot's rectangle
      if (ballX + 4 >= botX && ballX - 4 <= botX + width &&
          ballY + 4 >= botY && ballY - 4 <= botY + height) {
          // Collision detected with the bot's rectangle, reverse the ball's velocity
          // You can adjust the angle of reflection here if needed
          velocityX *= -1;
          velocityY *= -1;
      }

      // Draw the ball at the new position
      ili9341_fill_circle(ili9341_display, ILI9341_WHITE, ballX, ballY, 4);
  }



  void calculateOtherRectangle() {

	// Clear the portion of the screen where the rectangle was previously drawn
	if (direction == 1 && botY > 0) {
	// If moving down and not at the top, clear the area above the new rectangle position
	ili9341_fill_rect(ili9341_display, ILI9341_BLACK, botX, botY + 1, width, botSpeed);
	movement_distance = botSpeed;
	} else if (direction == -1 && botY + height < 240) {
	// If moving up and not at the bottom, clear the area below the new rectangle position
	ili9341_fill_rect(ili9341_display, ILI9341_BLACK, botX, botY + height - 1, width, height);
	movement_distance = -botSpeed;
	}


	// Draw the rectangle at the current position
	ili9341_fill_rect(ili9341_display, color, botX, botY + botSpeed, width, height);
	//HAL_Delay(gameSpeed);
	// Update the Y coordinate based on the direction of movement
	botY += direction;


	// Check if the rectangle has reached the top or bottom of the screen
	if (botY <= 0 || botY + height >= 240) {
	   // Change the direction of movement
	   direction *= -1;
	}
  }

  void calculateRectangle() {
  	  // Read the potentiometer value
  	  readADC2();

  	  // Determine the movement direction based on the potentiometer value
  	  int8_t direction = 0;
  	  if (potentiometer_value >= 1650 && potentiometer_value <= 1750) {
  		  direction = 0;
  		  movement_distance = 0;
  	  } else if (potentiometer_value < 1650) {
  		  // Move down if potentiometer is down
  		  movement_distance =  abs(potentiometer_value - 1650) / 325;
  		  direction = 1;
  	  } else if (potentiometer_value > 1800) {
  		  // Move up if potentiometer is up
  		  movement_distance = -abs(1750 - potentiometer_value) / 325;
  		  direction = -1;
  	  }

  	  // Calculate the new position of the rectangle
  	  new_y = playerY + movement_distance;

  	  // Check if the new position is within the bounds of the screen
  	  if (new_y < 0) {
  		  new_y = 0;
  	  } else if (new_y + height > 240) {
  		  new_y = 240 - height;
  	  }


  	// Clear the portion of the screen where the rectangle was previously drawn
  	if (direction == 1 && playerY > 0) {
  	    // If moving down and not at the top, clear the area above the new rectangle position
  	    ili9341_fill_rect(ili9341_display, ILI9341_BLACK, playerX, playerY, width, new_y - playerY);
  	} else if (direction == -1 && playerY + height < 240) {
  	    // If moving up and not at the bottom, clear the area below the new rectangle position
  	    ili9341_fill_rect(ili9341_display, ILI9341_BLACK, playerX, playerY + height, width, height + abs(movement_distance));
  	}


  	  // Draw the rectangle at the new position
  	  ili9341_fill_rect(ili9341_display, color, playerX, new_y, width, height);

  	  // Update the position of the rectangle
  	  playerY = new_y;

  	  HAL_Delay(10);
    }

  //initialize_ili9341();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  drawArena();
  while (1) {
	  calculateRectangle();
	  calculateOtherRectangle();
	  updateBallPosition();
	  updateArena();
  }

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
////
//  /* USER CODE END ADC1_Init 0 */
//
//  ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC1_Init 1 */
////
//  /* USER CODE END ADC1_Init 1 */
//
//  /** Common config
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc1.Init.LowPowerAutoWait = DISABLE;
//  hadc1.Init.ContinuousConvMode = DISABLE;
//  hadc1.Init.NbrOfConversion = 1;
//  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.DMAContinuousRequests = DISABLE;
//  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc1.Init.OversamplingMode = DISABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure the ADC multi-mode
//  */
//  multimode.Mode = ADC_MODE_INDEPENDENT;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */
////
//  /* USER CODE END ADC1_Init 2 */
//
//}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**UCPD1 GPIO Configuration
  PB15   ------> UCPD1_CC2
  PA15 (JTDI)   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2|UCPD_DBN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 LED_RED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 UCPD_DBN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|UCPD_DBN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

    // Initialize GPIO pin : PB0 for Reset
    GPIO_InitStruct.Pin = GPIO_PIN_0; // Reset pin
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize GPIO pin : PA4 for Chip Select (CS)
    GPIO_InitStruct.Pin = GPIO_PIN_4; // CS pin
    // Other parameters remain the same from previous initialization
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize GPIO pin : PB2 for Data/Command (DC)
    GPIO_InitStruct.Pin = GPIO_PIN_2; // DC pin
    // Other parameters remain the same from PB0 initialization
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setupADC() {
	// PC0 is ADC1_IN1  (Check datasheet or slides)
	    RCC->AHB2ENR  |= 0b100;         // Enable GPIOC
	    bitset(GPIOC->MODER, 0);        // Setup PC0 to 0b11 (Analog input)
	    bitset(GPIOC->MODER, 1);

	    // Enable ADC Clock
		bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
		RCC->CCIPR1 |=0x3<<28;     // Route SYSCLK (HCLK) to ADC

	    // Turn on ADC Voltage Regulator
		bitclear(ADC1->CR, 29);  // Get out of deep power down mode
	    bitset(ADC1->CR, 28);

	    // Wait for the voltage regulator to stabilize
		HAL_Delay(10);

		// Set up ADC1
	    ADC1->SQR1 = (1<<6)|(0); 	// L=0 (one channel to read), SQ1=IN1 which is connected to PC0 (Called ADC1_IN1)
	    ADC1->CR |= 1;            	// Enable ADC

		// Wait until ADC is Ready (ADRDY)
		while(bitcheck(ADC1->ISR, 0)==0);
}

void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}

/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0')
    {
    	LPUART1write(msg[idx++]);
    }
}



void LPUART1init(){
	PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
							// Basically power up PORTG

	// LPUART1 TX is connected to Port G pin 7, RX is connected to PG8
	// GPIOG is connected to the AHB2 bus.
	RCC->AHB2ENR |= (1<<6);   // Enable the clock of the GPIOG

	// MCU LPUART1 TX is connected the MCU pin PG7
	// PG7 must be set to use AF (Alternate Function).
	// Note that you need to set AF to 8 to connect the LPUART1 TX to the GPIOG.7
	GPIOG->MODER  &= ~(0x3<<(2*7)); // Clear the two bits for PG7
	GPIOG->MODER  |=  (0x2<<(2*7)); // Set the mode to AF (10--> 0x2)
	// Set the AF=8
	GPIOG->AFR[0] &= ~(0xF<<(4*7)); // Clear the 4 bits for PG7
	GPIOG->AFR[0] |=  (0x8<<(4*7)); // Set the 4 bits to (8)

	// MCU LPUART1 RX can be connected the MCU pin PG8
		// PA3 must be set to use AF (Alternate Function).
		// Note that you need to set AF to 8 to connect the LPUART1 RX to the GPIOG.8
	GPIOG->MODER  &= ~(0x3<<(2*8)); // Clear the two bits for PG8
	GPIOG->MODER  |=  (0x2<<(2*8)); // Set the mode to AF (10--> 0x2)

	GPIOG->AFR[1] &= ~(0xF<<(4*0)); // Clear the 4 bits for PG8
	GPIOG->AFR[1] |=  (0x8<<(4*0)); // Set the 4 bits to (7)

	// Enable the clock for the LPUART1
	// LPUART1 is connected to the APB1 (Advanced Peripheral 1) bus.
		// LPUART1 enabled by setting bit 0

	// LPUART1 CONFIGURATION //
		// We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity, and no hw flow control
	LPUART1-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided

	// Buadrate = (256 X LPUARTtck_pres)/LPUART_BRR
	// LPUART_BRR = 256 * 16MHz / 115200=  35,555.5  ==> 0x8AE3
	LPUART1->BRR = 0x8AE3;  //  (16000000/115200)<<8

	// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
	LPUART1->CR1  = 0x0;  // clear all settings
	LPUART1->CR1 |= 1<<3; // Enable Transmitter
	LPUART1->CR1 |= 1<<2; // Enable Receiver

	// 00: 1 stop bit
	LPUART1->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
	LPUART1->CR3 = 0x0000;    // no flow control and all other features left to default (0)

	// Last thing is to enable the LPUART1 module (remember that we set the clock, configure GPIO, configure LPUART1)
	LPUART1->CR1 |= 1; // Enable LPUART1

}


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
