/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F411RE IBT-2 actuator limit-switch test, no FreeRTOS
  *
  * Goal:
  *   - Control two 24 V actuators through two IBT-2 motor drivers.
  *   - Only one actuator moves at a time.
  *   - Normally-open limit switches stop OPEN motion.
  *   - GUI/RS-232 is ignored for now.
  *   - The blue USER button is used as a temporary test input.
  *
  * Temporary USER button sequence:
  *   Press 1: Actuator 1 OPEN until limit switch 1 closes or timeout
  *   Press 2: Actuator 1 CLOSE for fixed time
  *   Press 3: Actuator 2 OPEN until limit switch 2 closes or timeout
  *   Press 4: Actuator 2 CLOSE for fixed time
  *   Then repeat.
  *
  * Limit switch wiring:
  *   STM32 input pin ---- normally-open limit switch ---- GND
  *
  * Limit switch GPIO setup:
  *   GPIO_MODE_IT_FALLING with GPIO_PULLUP
  *   Unpressed = HIGH
  *   Pressed/closed = LOW
  *
  * IBT-2 wiring reminder:
  *   24 V DC +  -> IBT-2 B+
  *   24 V DC -  -> IBT-2 B-
  *   Actuator   -> IBT-2 M+ / M-
  *   STM32 GND, IBT-2 GND, and 24 V DC - must be common
  *   Do NOT connect 24 V to STM32 pins or IBT-2 logic VCC
  *
  * Suggested pin plan:
  *   Actuator 1 IBT-2:
  *      PB0 -> A1_RPWM
  *      PB1 -> A1_LPWM
  *      PC0 -> A1_REN
  *      PC1 -> A1_LEN
  *
  *   Actuator 2 IBT-2:
  *      PB6 -> A2_RPWM
  *      PB7 -> A2_LPWM
  *      PC2 -> A2_REN
  *      PC3 -> A2_LEN
  *
  *   Limit switches:
  *      PC10 -> Actuator 1 open limit switch to GND
  *      PC11 -> Actuator 2 open limit switch to GND
  *
  *   Nucleo pins:
  *      PC13 -> Blue USER button
  *      PA5  -> LD2 green LED
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_A1_OPENING,
  MOTOR_STATE_A1_CLOSING,
  MOTOR_STATE_A2_OPENING,
  MOTOR_STATE_A2_CLOSING
} MotorState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Actuator 1 IBT-2 pins */
#define A1_RPWM_GPIO_Port      GPIOB
#define A1_RPWM_Pin            GPIO_PIN_0
#define A1_LPWM_GPIO_Port      GPIOB
#define A1_LPWM_Pin            GPIO_PIN_1
#define A1_REN_GPIO_Port       GPIOC
#define A1_REN_Pin             GPIO_PIN_0
#define A1_LEN_GPIO_Port       GPIOC
#define A1_LEN_Pin             GPIO_PIN_1

/* Actuator 2 IBT-2 pins */
#define A2_RPWM_GPIO_Port      GPIOB
#define A2_RPWM_Pin            GPIO_PIN_6
#define A2_LPWM_GPIO_Port      GPIOB
#define A2_LPWM_Pin            GPIO_PIN_7
#define A2_REN_GPIO_Port       GPIOC
#define A2_REN_Pin             GPIO_PIN_2
#define A2_LEN_GPIO_Port       GPIOC
#define A2_LEN_Pin             GPIO_PIN_3

/* Normally-open open-limit switches, wired from pin to GND */
#define A1_LIMIT_GPIO_Port     GPIOC
#define A1_LIMIT_Pin           GPIO_PIN_10
#define A2_LIMIT_GPIO_Port     GPIOC
#define A2_LIMIT_Pin           GPIO_PIN_11

/*
 * Tune these values for your real actuator.
 * OPEN should normally stop by limit switch. The timeout is only safety backup.
 * CLOSE currently stops by time because we are assuming no close-limit switch yet.
 */
#define MOTOR_OPEN_TIMEOUT_MS  15000U
#define MOTOR_CLOSE_RUN_MS     5000U
#define BUTTON_DEBOUNCE_MS     250U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static volatile MotorState_t g_motor_state = MOTOR_STATE_IDLE;
static volatile uint8_t g_button_event = 0U;
static volatile uint8_t g_limit1_event = 0U;
static volatile uint8_t g_limit2_event = 0U;
static volatile uint32_t g_last_button_tick_ms = 0U;

static uint8_t g_test_step = 0U;
static uint32_t g_state_start_tick_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
static void Motor_Set_State(MotorState_t new_state);
static void Motor_Stop_All(void);

static void Actuator1_Stop(void);
static void Actuator1_Open(void);
static void Actuator1_Close(void);

static void Actuator2_Stop(void);
static void Actuator2_Open(void);
static void Actuator2_Close(void);

static void Motor_Start_Open1(void);
static void Motor_Start_Close1(void);
static void Motor_Start_Open2(void);
static void Motor_Start_Close2(void);

static void Motor_Handle_Button_Test(void);
static void Motor_Handle_Limit1(void);
static void Motor_Handle_Limit2(void);
static void Motor_Check_Timeouts(void);
static void Motor_Task_Poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Motor_Set_State(MotorState_t new_state)
{
  g_motor_state = new_state;

  if (new_state == MOTOR_STATE_IDLE)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }

  g_state_start_tick_ms = HAL_GetTick();
}

static void Actuator1_Stop(void)
{
  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_REN_GPIO_Port,  A1_REN_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_LEN_GPIO_Port,  A1_LEN_Pin,  GPIO_PIN_RESET);
}

static void Actuator1_Open(void)
{
  /*
   * Direction safety:
   * Turn both PWM pins off before enabling and driving one direction.
   */
  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(A1_REN_GPIO_Port, A1_REN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(A1_LEN_GPIO_Port, A1_LEN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_SET);
}

static void Actuator1_Close(void)
{
  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(A1_REN_GPIO_Port, A1_REN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(A1_LEN_GPIO_Port, A1_LEN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_SET);
}

static void Actuator2_Stop(void)
{
  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_REN_GPIO_Port,  A2_REN_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_LEN_GPIO_Port,  A2_LEN_Pin,  GPIO_PIN_RESET);
}

static void Actuator2_Open(void)
{
  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(A2_REN_GPIO_Port, A2_REN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(A2_LEN_GPIO_Port, A2_LEN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_SET);
}

static void Actuator2_Close(void)
{
  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(A2_REN_GPIO_Port, A2_REN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(A2_LEN_GPIO_Port, A2_LEN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_SET);
}

static void Motor_Stop_All(void)
{
  Actuator1_Stop();
  Actuator2_Stop();
  Motor_Set_State(MOTOR_STATE_IDLE);
}

static void Motor_Start_Open1(void)
{
  Motor_Stop_All();

  /*
   * If the open limit is already pressed, do not drive farther open.
   */
  if (HAL_GPIO_ReadPin(A1_LIMIT_GPIO_Port, A1_LIMIT_Pin) == GPIO_PIN_RESET)
  {
    Motor_Set_State(MOTOR_STATE_IDLE);
    return;
  }

  Actuator1_Open();
  Motor_Set_State(MOTOR_STATE_A1_OPENING);
}

static void Motor_Start_Close1(void)
{
  Motor_Stop_All();
  Actuator1_Close();
  Motor_Set_State(MOTOR_STATE_A1_CLOSING);
}

static void Motor_Start_Open2(void)
{
  Motor_Stop_All();

  if (HAL_GPIO_ReadPin(A2_LIMIT_GPIO_Port, A2_LIMIT_Pin) == GPIO_PIN_RESET)
  {
    Motor_Set_State(MOTOR_STATE_IDLE);
    return;
  }

  Actuator2_Open();
  Motor_Set_State(MOTOR_STATE_A2_OPENING);
}

static void Motor_Start_Close2(void)
{
  Motor_Stop_All();
  Actuator2_Close();
  Motor_Set_State(MOTOR_STATE_A2_CLOSING);
}

static void Motor_Handle_Button_Test(void)
{
  switch (g_test_step)
  {
    case 0U:
      Motor_Start_Open1();
      break;

    case 1U:
      Motor_Start_Close1();
      break;

    case 2U:
      Motor_Start_Open2();
      break;

    case 3U:
    default:
      Motor_Start_Close2();
      break;
  }

  g_test_step++;
  if (g_test_step >= 4U)
  {
    g_test_step = 0U;
  }
}

static void Motor_Handle_Limit1(void)
{
  if (g_motor_state == MOTOR_STATE_A1_OPENING)
  {
    Actuator1_Stop();
    Motor_Set_State(MOTOR_STATE_IDLE);
  }
}

static void Motor_Handle_Limit2(void)
{
  if (g_motor_state == MOTOR_STATE_A2_OPENING)
  {
    Actuator2_Stop();
    Motor_Set_State(MOTOR_STATE_IDLE);
  }
}

static void Motor_Check_Timeouts(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed_ms = now_ms - g_state_start_tick_ms;

  switch (g_motor_state)
  {
    case MOTOR_STATE_A1_OPENING:
    case MOTOR_STATE_A2_OPENING:
      if (elapsed_ms >= MOTOR_OPEN_TIMEOUT_MS)
      {
        Motor_Stop_All();
      }
      break;

    case MOTOR_STATE_A1_CLOSING:
    case MOTOR_STATE_A2_CLOSING:
      if (elapsed_ms >= MOTOR_CLOSE_RUN_MS)
      {
        Motor_Stop_All();
      }
      break;

    case MOTOR_STATE_IDLE:
    default:
      break;
  }
}

/*
 * This function is the no-FreeRTOS replacement for a MotorTask.
 * It is called repeatedly from while(1).
 */
static void Motor_Task_Poll(void)
{
  if (g_limit1_event != 0U)
  {
    g_limit1_event = 0U;
    Motor_Handle_Limit1();
  }

  if (g_limit2_event != 0U)
  {
    g_limit2_event = 0U;
    Motor_Handle_Limit2();
  }

  if (g_button_event != 0U)
  {
    g_button_event = 0U;
    Motor_Handle_Button_Test();
  }

  Motor_Check_Timeouts();
}

/**
  * @brief EXTI callback for USER button and limit switches.
  * @param GPIO_Pin Pin that caused the EXTI interrupt.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - g_last_button_tick_ms) >= BUTTON_DEBOUNCE_MS)
    {
      g_last_button_tick_ms = now_ms;
      g_button_event = 1U;
    }
  }
  else if (GPIO_Pin == A1_LIMIT_Pin)
  {
    /*
     * Immediate safety stop on limit event.
     * Main loop will also process g_limit1_event and update state.
     */
    if (g_motor_state == MOTOR_STATE_A1_OPENING)
    {
      Actuator1_Stop();
    }

    g_limit1_event = 1U;
  }
  else if (GPIO_Pin == A2_LIMIT_Pin)
  {
    if (g_motor_state == MOTOR_STATE_A2_OPENING)
    {
      Actuator2_Stop();
    }

    g_limit2_event = 1U;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  Motor_Stop_All();
  /* USER CODE END 2 */

  while (1)
  {
    Motor_Task_Poll();

    /*
     * Small delay keeps the polling loop calm.
     * Limit switch stop still happens immediately in the EXTI callback.
     */
    HAL_Delay(1);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*
   * Put all motor outputs low before configuring pins.
   */
  HAL_GPIO_WritePin(GPIOB,
                    A1_RPWM_Pin | A1_LPWM_Pin | A2_RPWM_Pin | A2_LPWM_Pin,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC,
                    A1_REN_Pin | A1_LEN_Pin | A2_REN_Pin | A2_LEN_Pin,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Configure GPIO pin : B1_Pin, blue USER button */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*
   * USART2 pins are not used yet, but left as generated by Nucleo template.
   * Remove this block if CubeMX did not generate USART_TX_Pin/USART_RX_Pin.
   */
#ifdef USART_TX_Pin
  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

  /* Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* Actuator PWM direction pins on GPIOB */
  GPIO_InitStruct.Pin = A1_RPWM_Pin | A1_LPWM_Pin | A2_RPWM_Pin | A2_LPWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Actuator enable pins on GPIOC */
  GPIO_InitStruct.Pin = A1_REN_Pin | A1_LEN_Pin | A2_REN_Pin | A2_LEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*
   * Limit switches:
   * Normally-open switch from pin to GND.
   * Internal pull-up makes unpressed = HIGH.
   * Closing switch pulls pin LOW and causes falling-edge EXTI.
   */
  GPIO_InitStruct.Pin = A1_LIMIT_Pin | A2_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*
   * PC10, PC11, and PC13 all share EXTI15_10_IRQn.
   */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Reports the source file and source line number.
  * @param file Source file name.
  * @param line Source line number.
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */





////FreeRTOS VERSION
///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : STM32F411RE FreeRTOS IBT-2 actuator limit-switch test
//  *
//  * Hardware intent:
//  *   - Two 24 V actuators.
//  *   - Two IBT-2 / BTS7960 motor drivers, one driver per actuator.
//  *   - Only one actuator is allowed to move at a time.
//  *   - Normally-open limit switches are used for the OPEN end of travel.
//  *   - For now, GUI/RS-232 is ignored.
//  *   - The blue USER button is used as a temporary test command source.
//  *
//  * Temporary USER button command sequence:
//  *   Press 1: Actuator 1 OPEN until limit 1 closes or timeout
//  *   Press 2: Actuator 1 CLOSE for CLOSE_RUN_TIME_MS
//  *   Press 3: Actuator 2 OPEN until limit 2 closes or timeout
//  *   Press 4: Actuator 2 CLOSE for CLOSE_RUN_TIME_MS
//  *   Then repeats.
//  *
//  * Limit switch wiring:
//  *   STM32 limit input pin ---- normally-open switch ---- GND
//  *
//  * Limit switch GPIO setup:
//  *   - Input with internal pull-up
//  *   - Falling-edge EXTI
//  *   - Unpressed reads HIGH
//  *   - Pressed/closed reads LOW
//  *
//  * IBT-2 power wiring:
//  *   - 24 V DC supply + to IBT-2 B+
//  *   - 24 V DC supply - to IBT-2 B-
//  *   - STM32 GND, IBT-2 GND, and 24 V supply - must share common ground
//  *   - Do not connect 24 V to STM32 pins or to IBT-2 logic VCC
//  *
//  * Suggested pin plan in this file:
//  *   Actuator 1 IBT-2:
//  *      PB0 -> A1_RPWM
//  *      PB1 -> A1_LPWM
//  *      PC0 -> A1_REN
//  *      PC1 -> A1_LEN
//  *
//  *   Actuator 2 IBT-2:
//  *      PB6 -> A2_RPWM
//  *      PB7 -> A2_LPWM
//  *      PC2 -> A2_REN
//  *      PC3 -> A2_LEN
//  *
//  *   Limit switches:
//  *      PC10 -> A1 open limit, switch to GND
//  *      PC11 -> A2 open limit, switch to GND
//  *
//  *   Existing Nucleo pins:
//  *      PC13 -> B1 blue USER button
//  *      PA5  -> LD2 green LED
//  *
//  * CubeMX requirements:
//  *   - Board/MCU: NUCLEO-F411RE / STM32F411RETx
//  *   - Middleware: FreeRTOS enabled with CMSIS-RTOS V2
//  *   - SYS Debug: Serial Wire
//  *   - If CubeMX overwrites GPIO, recreate the pin setup below or preserve
//  *     USER CODE blocks.
//  ******************************************************************************
//  */
///* USER CODE END Header */
//
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include <stdbool.h>
//#include <stdint.h>
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//typedef enum
//{
//  MOTOR_STATE_IDLE = 0,
//  MOTOR_STATE_A1_OPENING,
//  MOTOR_STATE_A1_CLOSING,
//  MOTOR_STATE_A2_OPENING,
//  MOTOR_STATE_A2_CLOSING
//} MotorState_t;
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///*
// * =========================
// * IBT-2 pin assignments
// * =========================
// * Change only these defines if you move wires.
// */
//
///* Actuator 1 IBT-2 */
//#define A1_RPWM_GPIO_Port      GPIOB
//#define A1_RPWM_Pin            GPIO_PIN_0
//#define A1_LPWM_GPIO_Port      GPIOB
//#define A1_LPWM_Pin            GPIO_PIN_1
//#define A1_REN_GPIO_Port       GPIOC
//#define A1_REN_Pin             GPIO_PIN_0
//#define A1_LEN_GPIO_Port       GPIOC
//#define A1_LEN_Pin             GPIO_PIN_1
//
///* Actuator 2 IBT-2 */
//#define A2_RPWM_GPIO_Port      GPIOB
//#define A2_RPWM_Pin            GPIO_PIN_6
//#define A2_LPWM_GPIO_Port      GPIOB
//#define A2_LPWM_Pin            GPIO_PIN_7
//#define A2_REN_GPIO_Port       GPIOC
//#define A2_REN_Pin             GPIO_PIN_2
//#define A2_LEN_GPIO_Port       GPIOC
//#define A2_LEN_Pin             GPIO_PIN_3
//
///* Normally-open open-limit switches, wired from pin to GND */
//#define A1_LIMIT_GPIO_Port     GPIOC
//#define A1_LIMIT_Pin           GPIO_PIN_10
//#define A2_LIMIT_GPIO_Port     GPIOC
//#define A2_LIMIT_Pin           GPIO_PIN_11
//
///*
// * Tune these to match the actual actuator travel.
// * OPEN stops on limit switch, but also has a timeout safety.
// * CLOSE has no limit switch yet, so it stops after a fixed time.
// */
//#define MOTOR_OPEN_TIMEOUT_MS  15000U
//#define MOTOR_CLOSE_RUN_MS     5000U
//#define BUTTON_DEBOUNCE_MS     250U
//
///*
// * CMSIS-RTOS thread flags.
// * Do not use bit 31 because CMSIS uses high bits for error returns.
// */
//#define MOTOR_FLAG_CMD_OPEN1   (1UL << 0)
//#define MOTOR_FLAG_CMD_CLOSE1  (1UL << 1)
//#define MOTOR_FLAG_CMD_OPEN2   (1UL << 2)
//#define MOTOR_FLAG_CMD_CLOSE2  (1UL << 3)
//#define MOTOR_FLAG_LIMIT1      (1UL << 4)
//#define MOTOR_FLAG_LIMIT2      (1UL << 5)
//#define MOTOR_FLAG_BUTTON      (1UL << 6)
//
//#define MOTOR_FLAGS_ALL        (MOTOR_FLAG_CMD_OPEN1  | \
//                                MOTOR_FLAG_CMD_CLOSE1 | \
//                                MOTOR_FLAG_CMD_OPEN2  | \
//                                MOTOR_FLAG_CMD_CLOSE2 | \
//                                MOTOR_FLAG_LIMIT1     | \
//                                MOTOR_FLAG_LIMIT2     | \
//                                MOTOR_FLAG_BUTTON)
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
///* USER CODE BEGIN PV */
//osThreadId_t motorTaskHandle;
//
//static const osThreadAttr_t motorTask_attributes =
//{
//  .name = "MotorTask",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t)osPriorityAboveNormal
//};
//
///*
// * g_motor_state is read inside the EXTI callback so the limit ISR only
// * immediately stops a motor if that actuator is opening.
// */
//static volatile MotorState_t g_motor_state = MOTOR_STATE_IDLE;
//static volatile uint32_t g_last_button_tick_ms = 0U;
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//
///* USER CODE BEGIN PFP */
//static void MotorTask(void *argument);
//
//static void Motor_Stop_All(void);
//static void Motor_Set_State(MotorState_t new_state);
//
//static void Actuator1_Stop(void);
//static void Actuator1_Open(void);
//static void Actuator1_Close(void);
//
//static void Actuator2_Stop(void);
//static void Actuator2_Open(void);
//static void Actuator2_Close(void);
//
//static void Motor_Start_Open1(uint32_t now_tick);
//static void Motor_Start_Close1(uint32_t now_tick);
//static void Motor_Start_Open2(uint32_t now_tick);
//static void Motor_Start_Close2(uint32_t now_tick);
//
//static void Motor_Handle_Limit1(void);
//static void Motor_Handle_Limit2(void);
//static void Motor_Handle_Button_Test(uint32_t now_tick);
//static void Motor_Check_Timeouts(uint32_t now_tick, uint32_t state_start_tick);
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//static void Motor_Set_State(MotorState_t new_state)
//{
//  g_motor_state = new_state;
//
//  /*
//   * LD2 is ON whenever an actuator is actively being driven.
//   */
//  if (new_state == MOTOR_STATE_IDLE)
//  {
//    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//  }
//  else
//  {
//    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//  }
//}
//
//static void Actuator1_Stop(void)
//{
//  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_REN_GPIO_Port,  A1_REN_Pin,  GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_LEN_GPIO_Port,  A1_LEN_Pin,  GPIO_PIN_RESET);
//}
//
//static void Actuator1_Open(void)
//{
//  /*
//   * Direction safety:
//   * Set both PWM pins low before enabling and driving one direction.
//   */
//  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(A1_REN_GPIO_Port, A1_REN_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(A1_LEN_GPIO_Port, A1_LEN_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_SET);
//}
//
//static void Actuator1_Close(void)
//{
//  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(A1_REN_GPIO_Port, A1_REN_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(A1_LEN_GPIO_Port, A1_LEN_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(A1_RPWM_GPIO_Port, A1_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A1_LPWM_GPIO_Port, A1_LPWM_Pin, GPIO_PIN_SET);
//}
//
//static void Actuator2_Stop(void)
//{
//  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_REN_GPIO_Port,  A2_REN_Pin,  GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_LEN_GPIO_Port,  A2_LEN_Pin,  GPIO_PIN_RESET);
//}
//
//static void Actuator2_Open(void)
//{
//  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(A2_REN_GPIO_Port, A2_REN_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(A2_LEN_GPIO_Port, A2_LEN_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_SET);
//}
//
//static void Actuator2_Close(void)
//{
//  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(A2_REN_GPIO_Port, A2_REN_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(A2_LEN_GPIO_Port, A2_LEN_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(A2_RPWM_GPIO_Port, A2_RPWM_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(A2_LPWM_GPIO_Port, A2_LPWM_Pin, GPIO_PIN_SET);
//}
//
//static void Motor_Stop_All(void)
//{
//  Actuator1_Stop();
//  Actuator2_Stop();
//  Motor_Set_State(MOTOR_STATE_IDLE);
//}
//
//static void Motor_Start_Open1(uint32_t now_tick)
//{
//  (void)now_tick;
//
//  Motor_Stop_All();
//
//  /*
//   * If the open limit is already pressed, do not drive farther open.
//   */
//  if (HAL_GPIO_ReadPin(A1_LIMIT_GPIO_Port, A1_LIMIT_Pin) == GPIO_PIN_RESET)
//  {
//    Motor_Set_State(MOTOR_STATE_IDLE);
//    return;
//  }
//
//  Actuator1_Open();
//  Motor_Set_State(MOTOR_STATE_A1_OPENING);
//}
//
//static void Motor_Start_Close1(uint32_t now_tick)
//{
//  (void)now_tick;
//
//  Motor_Stop_All();
//  Actuator1_Close();
//  Motor_Set_State(MOTOR_STATE_A1_CLOSING);
//}
//
//static void Motor_Start_Open2(uint32_t now_tick)
//{
//  (void)now_tick;
//
//  Motor_Stop_All();
//
//  if (HAL_GPIO_ReadPin(A2_LIMIT_GPIO_Port, A2_LIMIT_Pin) == GPIO_PIN_RESET)
//  {
//    Motor_Set_State(MOTOR_STATE_IDLE);
//    return;
//  }
//
//  Actuator2_Open();
//  Motor_Set_State(MOTOR_STATE_A2_OPENING);
//}
//
//static void Motor_Start_Close2(uint32_t now_tick)
//{
//  (void)now_tick;
//
//  Motor_Stop_All();
//  Actuator2_Close();
//  Motor_Set_State(MOTOR_STATE_A2_CLOSING);
//}
//
//static void Motor_Handle_Limit1(void)
//{
//  if (g_motor_state == MOTOR_STATE_A1_OPENING)
//  {
//    Actuator1_Stop();
//    Motor_Set_State(MOTOR_STATE_IDLE);
//  }
//}
//
//static void Motor_Handle_Limit2(void)
//{
//  if (g_motor_state == MOTOR_STATE_A2_OPENING)
//  {
//    Actuator2_Stop();
//    Motor_Set_State(MOTOR_STATE_IDLE);
//  }
//}
//
//static void Motor_Handle_Button_Test(uint32_t now_tick)
//{
//  static uint8_t test_step = 0U;
//
//  /*
//   * This is only a temporary stand-in for GUI/RS-232 commands.
//   */
//  switch (test_step)
//  {
//    case 0U:
//      Motor_Start_Open1(now_tick);
//      break;
//
//    case 1U:
//      Motor_Start_Close1(now_tick);
//      break;
//
//    case 2U:
//      Motor_Start_Open2(now_tick);
//      break;
//
//    case 3U:
//    default:
//      Motor_Start_Close2(now_tick);
//      break;
//  }
//
//  test_step++;
//  if (test_step >= 4U)
//  {
//    test_step = 0U;
//  }
//}
//
//static void Motor_Check_Timeouts(uint32_t now_tick, uint32_t state_start_tick)
//{
//  uint32_t elapsed_ms = now_tick - state_start_tick;
//
//  switch (g_motor_state)
//  {
//    case MOTOR_STATE_A1_OPENING:
//    case MOTOR_STATE_A2_OPENING:
//      if (elapsed_ms >= MOTOR_OPEN_TIMEOUT_MS)
//      {
//        Motor_Stop_All();
//      }
//      break;
//
//    case MOTOR_STATE_A1_CLOSING:
//    case MOTOR_STATE_A2_CLOSING:
//      if (elapsed_ms >= MOTOR_CLOSE_RUN_MS)
//      {
//        Motor_Stop_All();
//      }
//      break;
//
//    case MOTOR_STATE_IDLE:
//    default:
//      break;
//  }
//}
//
//static void MotorTask(void *argument)
//{
//  (void)argument;
//
//  uint32_t state_start_tick = 0U;
//
//  /*
//   * Safety default on task start.
//   */
//  Motor_Stop_All();
//
//  for (;;)
//  {
//    uint32_t flags = osThreadFlagsWait(MOTOR_FLAGS_ALL,
//                                       osFlagsWaitAny,
//                                       20U);
//
//    uint32_t now_tick = osKernelGetTickCount();
//
//    if ((flags & osFlagsError) == 0U)
//    {
//      /*
//       * Limit switches get priority over command events.
//       */
//      if ((flags & MOTOR_FLAG_LIMIT1) != 0U)
//      {
//        Motor_Handle_Limit1();
//      }
//
//      if ((flags & MOTOR_FLAG_LIMIT2) != 0U)
//      {
//        Motor_Handle_Limit2();
//      }
//
//      if ((flags & MOTOR_FLAG_BUTTON) != 0U)
//      {
//        Motor_Handle_Button_Test(now_tick);
//        state_start_tick = now_tick;
//      }
//
//      /*
//       * These command flags are ready for the future GUI task.
//       * For now they are not sent by UART, but the motor task already
//       * supports them.
//       */
//      if ((flags & MOTOR_FLAG_CMD_OPEN1) != 0U)
//      {
//        Motor_Start_Open1(now_tick);
//        state_start_tick = now_tick;
//      }
//
//      if ((flags & MOTOR_FLAG_CMD_CLOSE1) != 0U)
//      {
//        Motor_Start_Close1(now_tick);
//        state_start_tick = now_tick;
//      }
//
//      if ((flags & MOTOR_FLAG_CMD_OPEN2) != 0U)
//      {
//        Motor_Start_Open2(now_tick);
//        state_start_tick = now_tick;
//      }
//
//      if ((flags & MOTOR_FLAG_CMD_CLOSE2) != 0U)
//      {
//        Motor_Start_Close2(now_tick);
//        state_start_tick = now_tick;
//      }
//    }
//
//    Motor_Check_Timeouts(now_tick, state_start_tick);
//  }
//}
//
///**
//  * @brief EXTI callback for USER button and limit switches.
//  * @param GPIO_Pin Pin that caused the EXTI interrupt.
//  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if (GPIO_Pin == B1_Pin)
//  {
//    uint32_t now_ms = HAL_GetTick();
//
//    if ((now_ms - g_last_button_tick_ms) >= BUTTON_DEBOUNCE_MS)
//    {
//      g_last_button_tick_ms = now_ms;
//
//      if (motorTaskHandle != NULL)
//      {
//        (void)osThreadFlagsSet(motorTaskHandle, MOTOR_FLAG_BUTTON);
//      }
//    }
//  }
//  else if (GPIO_Pin == A1_LIMIT_Pin)
//  {
//    /*
//     * Immediate safety stop when the active motion is opening actuator 1.
//     * The task also receives the event and updates the state machine.
//     */
//    if (g_motor_state == MOTOR_STATE_A1_OPENING)
//    {
//      Actuator1_Stop();
//    }
//
//    if (motorTaskHandle != NULL)
//    {
//      (void)osThreadFlagsSet(motorTaskHandle, MOTOR_FLAG_LIMIT1);
//    }
//  }
//  else if (GPIO_Pin == A2_LIMIT_Pin)
//  {
//    if (g_motor_state == MOTOR_STATE_A2_OPENING)
//    {
//      Actuator2_Stop();
//    }
//
//    if (motorTaskHandle != NULL)
//    {
//      (void)osThreadFlagsSet(motorTaskHandle, MOTOR_FLAG_LIMIT2);
//    }
//  }
//}
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* MCU Configuration--------------------------------------------------------*/
//
//  HAL_Init();
//
//  SystemClock_Config();
//
//  MX_GPIO_Init();
//
//  /* USER CODE BEGIN 2 */
//  osKernelInitialize();
//
//  motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);
//  if (motorTaskHandle == NULL)
//  {
//    Error_Handler();
//  }
//
//  osKernelStart();
//  /* USER CODE END 2 */
//
//  while (1)
//  {
//    /*
//     * Execution should never get here after osKernelStart().
//     */
//  }
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 16;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
//                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*
//   * Make sure all motor command outputs are low before configuring pins.
//   */
//  HAL_GPIO_WritePin(GPIOB,
//                    A1_RPWM_Pin | A1_LPWM_Pin | A2_RPWM_Pin | A2_LPWM_Pin,
//                    GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(GPIOC,
//                    A1_REN_Pin | A1_LEN_Pin | A2_REN_Pin | A2_LEN_Pin,
//                    GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*
//   * Blue USER button on Nucleo.
//   * Temporary stand-in for GUI commands.
//   */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*
//   * USART2 pins are left configured like the default Nucleo project.
//   * They are not used yet, but this keeps PA2/PA3 ready for later.
//   */
//  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* LD2 debug LED */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
//  /* Actuator PWM direction pins on GPIOB */
//  GPIO_InitStruct.Pin = A1_RPWM_Pin | A1_LPWM_Pin | A2_RPWM_Pin | A2_LPWM_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* Actuator enable pins on GPIOC */
//  GPIO_InitStruct.Pin = A1_REN_Pin | A1_LEN_Pin | A2_REN_Pin | A2_LEN_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*
//   * Limit switches:
//   * Normally-open switch from pin to GND.
//   * Internal pull-up makes unpressed = HIGH.
//   * Closing switch pulls pin LOW and generates falling-edge EXTI.
//   */
//  GPIO_InitStruct.Pin = A1_LIMIT_Pin | A2_LIMIT_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*
//   * PC10, PC11, and PC13 all use EXTI15_10_IRQn.
//   * Priority 5 is commonly safe with FreeRTOS/CMSIS on STM32 projects.
//   */
//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//}
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  __disable_irq();
//
//  while (1)
//  {
//  }
//}
//
//#ifdef USE_FULL_ASSERT
///**
//  * @brief Reports the source file and source line number for assert_param error.
//  * @param file Source file name.
//  * @param line Source line number.
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  (void)file;
//  (void)line;
//}
//#endif /* USE_FULL_ASSERT */
