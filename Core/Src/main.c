/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Leitura MPU6050 com Filtro Complementar e Display OLED
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1)
#define WHO_AM_I_REG  0x75
#define PWR_MGMT_1_REG 0X6B
#define ACCEL_XOUT_H_REG 0x3B
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  uint8_t check;
  uint8_t data;
  char msg[100];
  uint8_t rec_data[14];
  int16_t accel_x_raw, accel_y_raw, accel_z_raw;
  int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
  float Ax, Ay, Az, Gx, Gy, Gz;

  // Vars do filtro complementar
  float roll = 0, pitch = 0, yaw = 0;
  float accel_roll, accel_pitch;
  float alpha = 0.96;
  float dt = 0.0;
  uint32_t tempo_anterior = 0;

  // Verifica se o MPU6050 responde no endereco esperado
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

  if (check == 0x68){
    sprintf(msg,"MPU6050 OK!\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen(msg),100);

    // Tira o MPU do modo sleep
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,PWR_MGMT_1_REG,1,&data,1,1000);
  }
  else{
    sprintf(msg,"ERRO: MPU6050 NAO ENCONTRADO\r\n");
    HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),100);
  }

  ssd1306_Init(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Leitura bruta dos dados de aceleracao e giroscopio
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 14, 1000);

    accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);

    gyro_x_raw = (int16_t)(rec_data[8] << 8 | rec_data[9]);
    gyro_y_raw = (int16_t)(rec_data[10] << 8 | rec_data[11]);
    gyro_z_raw = (int16_t)(rec_data[12] << 8 | rec_data[13]);

    // Conversao para g e graus/s de acordo com o fundo de escala default
    Ax = accel_x_raw / 16384.0;
    Ay = accel_y_raw / 16384.0;
    Az = accel_z_raw / 16384.0;

    Gx = gyro_x_raw / 131.0;
    Gy = gyro_y_raw / 131.0;
    Gz = gyro_z_raw / 131.0;

    // Filtro para ignorar pequenos ruidos no eixo Z
    if (Gz > -3.0 && Gz < 3.0) { Gz = 0.0; }

    // Calcula o dt para a integracao do giroscopio
    uint32_t tempo_atual = HAL_GetTick();
    dt = (tempo_atual - tempo_anterior) / 1000.0;
    tempo_anterior = tempo_atual;

    // Angulos baseados apenas no acelerometro
    accel_roll = atan2(Ay, Az) * 180.0 / 3.14159;
    accel_pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / 3.14159;

    // Aplicacao do filtro complementar
    roll = alpha * (roll + Gx * dt) + (1.0 - alpha) * accel_roll;
    pitch = alpha * (pitch + Gy * dt) + (1.0 - alpha) * accel_pitch;
    yaw = yaw + (Gz * dt);

    // Print de debug via UART
    sprintf(msg, "R: %.1f | P: %.1f | Y: %.1f\r\n", roll, pitch, yaw);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    // --- Atualizacao do Display OLED ---
    ssd1306_Fill(Black);

    // Mostra os valores textuais na esquerda
    char str_roll[20], str_pitch[20], str_yaw[20];
    sprintf(str_roll, "Rol:%.0f", roll);
    sprintf(str_pitch, "Arf:%.0f", pitch);
    sprintf(str_yaw, "Gui:%.0f", yaw);

    ssd1306_SetCursor(0, 5);
    ssd1306_WriteString(str_roll, Font_7x10, White);

    ssd1306_SetCursor(0, 25);
    ssd1306_WriteString(str_pitch, Font_7x10, White);

    ssd1306_SetCursor(0, 45);
    ssd1306_WriteString(str_yaw, Font_7x10, White);

    // Logica da animacao visual na direita
    int cx = 96;
    int cy = 32;
    float ganho = 0.6f;

    int pos_x = cx + (int)(roll * ganho);
    int pos_y = cy - (int)(pitch * ganho);

    // Trava o indicador para nao sobrepor o texto
    if(pos_x < 64) pos_x = 64;
    if(pos_x > 127) pos_x = 127;
    if(pos_y < 0) pos_y = 0;
    if(pos_y > 63) pos_y = 63;

    // Desenha as retas dos eixos
    for(int i = 64; i < 128; i++) { ssd1306_DrawPixel(i, cy, White); }
    for(int i = 0; i < 64; i++) { ssd1306_DrawPixel(cx, i, White); }

    // Desenha o ponto que se move com os angulos
    ssd1306_DrawPixel(pos_x, pos_y, White);
    ssd1306_DrawPixel(pos_x + 1, pos_y, White);
    ssd1306_DrawPixel(pos_x - 1, pos_y, White);
    ssd1306_DrawPixel(pos_x, pos_y + 1, White);
    ssd1306_DrawPixel(pos_x, pos_y - 1, White);

    ssd1306_UpdateScreen(&hi2c1);

    HAL_Delay(10);

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
