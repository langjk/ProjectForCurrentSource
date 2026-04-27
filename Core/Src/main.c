/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include <string.h>
#include "flash_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t modeFlag;             // 电流/电压模式�??0=电流�??1=电压
    uint8_t channelFlag;          // 当前选择的�?�道�??0~3表示�??1到第4个�?�道
    uint8_t inSettingsPage;       // 是否在设置页面，0：不在，1：在设置页面
    uint8_t settingFlag;          // 当前高亮设置项，0:电流/电压切换�??1:电流/电压值，2:PWM频率�??3:PWM占空比，4:满量程补�??
		uint8_t isSettingFlag;
} UIState_t;


typedef struct {
    uint8_t mode;                     // 电流/电压模式
    uint16_t current_value;           // 电流�?? (对应A)
    uint16_t voltage_value;           // 电压�??
    uint16_t pwm_frequency;           // PWM频率
    uint8_t pwm_duty_cycle;           // PWM占空�??
    uint16_t full_scale_A; // 满量程补�?? (对应C)
		uint16_t full_scale_V;
    uint16_t zero_offset_A;           // 电流零点偏移
    uint16_t zero_offset_V;           // 电压零点偏移
    uint8_t status;                   // 通道连接状�??
} Channel_t;

extern UIState_t uiState;  // 定义UI状�?�结构体实例
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LONG_PRESS_TIME 10  // 长按判定时间（ms�??
#define REPEAT_RATE     50   // 连续加减的频率（ms�??
#define TURBO_PRESS_TIME 40
#define RX_BUFFER_SIZE 64
#define MIN_CURRENT   0   // �??小电�??
#define MAX_CURRENT   400 // �??大电�??
#define MIN_VOLTAGE   0   // �??小电�??
#define MAX_VOLTAGE   500  // �??大电�??
#define MIN_PWM_FREQ  1   // �??小PWM频率
#define MAX_PWM_FREQ  100 // �??大PWM频率
#define MIN_DUTY       0   // �??小PWM占空�??
#define MAX_DUTY      100  // �??大PWM占空�??
#define MIN_ZERO_OFFSET 0   // 最小零点偏移
#define MAX_ZERO_OFFSET 200 // 最大零点偏移
#define DEFAULT_DEVICE_ID 1 // 默认设备ID
#define MAX_DEVICE_ID 99    // 最大设备ID
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t key_state1 = 0;
volatile uint32_t key_press_time1 = 0;
volatile uint8_t key_state2 = 0;
volatile uint32_t key_press_time2 = 0;
volatile uint8_t key_state3 = 0;
volatile uint32_t key_press_time3 = 0;
volatile uint8_t set_current_flag = 0;  // 标志位，标记是否�??要执行电流设�??

volatile uint8_t uart_cmd_ready = 0;
char uart_cmd_buffer[RX_BUFFER_SIZE];

uint8_t modeFlag = 0;
uint8_t channelFlag = 0;
	
uint8_t repeat_flag1 = 0;  // KEY1 是否进入连续减状�??
uint8_t repeat_flag2 = 0;  // KEY2 是否进入连续加状�??
uint8_t calibrationFlag = 0;
uint16_t A[4] = {440,446,456,454};
uint16_t V[4] = {1005,1005,1005,1005};
uint16_t C[4] = {4000,4000,4000,4000};
uint8_t channel_connected[4] = {0, 0, 0, 0};

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

uint32_t last_key_check = 0;

uint8_t buffer[3];

Channel_t channels[4];  // 四路通道
UIState_t uiState; // UI状态实例
uint8_t deviceID = DEFAULT_DEVICE_ID;  // 设备ID

// 多点校准相关变量
uint8_t inCalibPage = 0;           // 是否在校准页面
uint8_t calibPointIndex = 0;       // 当前校准点索引 (0-8)
uint16_t calibInputValue = 0;      // 用户输入的测量值 (以100为基数)
uint16_t multiCalibData[4][CALIB_POINTS_NUM];  // 4通道9点校准数据
uint8_t multiCalibEnabled[4] = {0, 0, 0, 0};   // 多点校准是否启用
// 校准点目标值 (以100为基数): 0, 50, 100, 150, 200, 250, 300, 350, 400 (对应0~4mA)
const uint16_t calibTargetValues[CALIB_POINTS_NUM] = {0, 50, 100, 150, 200, 250, 300, 350, 400};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void showUI(void);
void Check_I2C_Devices(void);
void Send_System_Params(void);
void ParseCommand(char *cmd);
void KeyScan(void);
void SetGP(void);
void initChannels(void);
// 设置电流
void setCurrent(void);
// 设置电压
void setVoltage(void);
// 设置PWM频率
void setPWMFrequency(void);
// 设置PWM占空�??
void setPWMDutyCycle(void);
// 设置满量程补�??
void setFullScaleCompensation(void);
// 设置电流/电压模式
void setMode(void);
// 设置通道状�?�（设备连接状�?�）
void setChannelStatus(void);
void initUIState(void);
void showSettingsPage(void);
void SaveCalibrationToFlash(void);
void LoadCalibrationFromFlash(void);
// 多点校准相关函数
void showCalibrationPage(void);
void initMultiCalibData(void);
void setCurrentWithCalib(void);  // 使用多点校准的电流输出
void outputCalibPoint(uint8_t pointIndex);  // 输出校准点
static uint16_t CalculateCurrentDacCodeLinear(uint8_t ch, uint16_t targetValue);
static void SanitizeMultiCalibChannel(uint8_t ch);
static uint8_t FindCalibrationSegment(uint8_t ch, uint16_t targetValue);
static uint16_t GetLongPressStep(uint32_t pressTime, uint16_t normalStep, uint16_t turboStep);
static HAL_StatusTypeDef GP8630_WriteFrame(uint8_t ch, const uint8_t *data, uint16_t size);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	initChannels();
	LoadCalibrationFromFlash();  // 从Flash加载校准数据（覆盖默认值）
	initUIState();
	Check_I2C_Devices();
	HAL_Delay(500);
	OLED_Init();
	  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        
        // 为所有通道配置PWM参数，确保PWM正确初始化
        for (uint8_t i = 0; i < 4; i++) {
            uiState.channelFlag = i;
            setPWMFrequency();
            setPWMDutyCycle();
        }
        uiState.channelFlag = 0; // 恢复默认通道
//	Send_System_Params();


	HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		OLED_Update();
		OLED_Clear();
		if (HAL_GetTick() - last_key_check >= 100) { 
            last_key_check = HAL_GetTick();
            
            // 判断当前页面
            if (inCalibPage) {
                showCalibrationPage();  // 显示校准页面
            }
            else if (uiState.inSettingsPage) {
                showSettingsPage();  // 显示设置页面
            }
            else {
                showUI();  // 显示主页面
            }
						
        KeyScan();  // 手动扫描按键状濿
        }
	
        // 按键扫描
        KeyScan();  // 手动扫描按键状�??
				if (uart_cmd_ready) {
						uart_cmd_ready = 0;
						ParseCommand(uart_cmd_buffer);  // 在主循环中执行，安全
				}

        // 根据�??要设置输出参�??
        if (set_current_flag == 1) {
            SetGP();
            set_current_flag = 0;
        }

		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/* USER CODE BEGIN 4 */
static uint16_t CalculateCurrentDacCodeLinear(uint8_t ch, uint16_t targetValue) {
    float fullS = channels[ch].full_scale_A / 10.0f;
    float zeroOffset = channels[ch].zero_offset_A / 10.0f;
    float adjustedValue = targetValue;

    if (fullS <= 0.0f) {
        fullS = 400.0f;
    }

    if (adjustedValue >= zeroOffset) {
        adjustedValue -= zeroOffset;
    } else {
        adjustedValue = 0.0f;
    }

    if (adjustedValue > fullS) {
        adjustedValue = fullS;
    }

    return (uint16_t)((adjustedValue / fullS) * 0xFFFF);
}

static void SanitizeMultiCalibChannel(uint8_t ch) {
    if (ch >= 4) {
        return;
    }

    for (uint8_t i = 1; i < CALIB_POINTS_NUM; i++) {
        if (multiCalibData[ch][i] < multiCalibData[ch][i - 1]) {
            multiCalibData[ch][i] = multiCalibData[ch][i - 1];
        }
    }
}

static uint8_t FindCalibrationSegment(uint8_t ch, uint16_t targetValue) {
    uint8_t bestIdx = 0;
    uint16_t bestDistance = 0xFFFF;

    for (uint8_t i = 0; i < CALIB_POINTS_NUM - 1; i++) {
        uint16_t p1 = multiCalibData[ch][i];
        uint16_t p2 = multiCalibData[ch][i + 1];
        uint16_t low = (p1 < p2) ? p1 : p2;
        uint16_t high = (p1 > p2) ? p1 : p2;
        uint16_t distance;

        if (targetValue >= low && targetValue <= high) {
            return i;
        }

        distance = (targetValue < low) ? (low - targetValue) : (targetValue - high);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIdx = i;
        }
    }

    return bestIdx;
}

static uint16_t GetLongPressStep(uint32_t pressTime, uint16_t normalStep, uint16_t turboStep) {
    if (pressTime >= TURBO_PRESS_TIME) {
        return turboStep;
    }

    return normalStep;
}

void initUIState(void) {
    uiState.modeFlag = 0;            // 默认电流模式
    uiState.channelFlag = 0;         // 默认选择通道1
    uiState.inSettingsPage = 0;      // 默认不在设置页面
    uiState.settingFlag = 0;         // 默认选中电流/电压切换
	  uiState.isSettingFlag = 0;
}
void initChannels(void) {
    for (uint8_t i = 0; i < 4; i++) {
        channels[i].mode = 0;  // 默认模式为电流模�??
        channels[i].current_value = 0;  // 默认电流�??
        channels[i].voltage_value = 0;   // 默认电压�??
        channels[i].pwm_frequency = 100;  // 默认PWM频率
        channels[i].pwm_duty_cycle = 100;   // 默认占空�??
        channels[i].full_scale_A = 4000;  // 默认满量程补偿 (对应4mA)
        channels[i].full_scale_V = 10000; // 默认满量程补偿 (对应芯片10V满量程)
        channels[i].zero_offset_A = 0;    // 默认电流零点偏移
        channels[i].zero_offset_V = 0;    // 默认电压零点偏移
        channels[i].status = 0;  // 初始状�?�未连接
    }
}

static HAL_StatusTypeDef GP8630_WriteFrame(uint8_t ch, const uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;

    for (uint8_t retry = 0; retry < 3; retry++) {
        status = HAL_I2C_Master_Transmit(&hi2c1, (0x58 + ch) << 1, (uint8_t *)data, size, HAL_MAX_DELAY);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(2);
    }

    return status;
}

void setCurrent(void) {
    uint8_t ch = uiState.channelFlag;
    uint16_t targetValue = channels[ch].current_value;  // 目标电流值 (以100为基数)

    {
        uint16_t improvedData;
        uint8_t txBuffer[3];

        if (multiCalibEnabled[ch]) {
            uint8_t idx;
            float t1;
            float t2;
            float m1;
            float m2;
            float dacOutput;

            SanitizeMultiCalibChannel(ch);
            idx = FindCalibrationSegment(ch, targetValue);
            t1 = calibTargetValues[idx];
            t2 = calibTargetValues[idx + 1];
            m1 = multiCalibData[ch][idx];
            m2 = multiCalibData[ch][idx + 1];

            if ((m2 - m1 > 0.001f) || (m1 - m2 > 0.001f)) {
                dacOutput = t1 + (targetValue - m1) * (t2 - t1) / (m2 - m1);
                if (dacOutput < 0.0f) {
                    dacOutput = 0.0f;
                }
                if (dacOutput > 400.0f) {
                    dacOutput = 400.0f;
                }
                improvedData = (uint16_t)((dacOutput / 400.0f) * 0xFFFF);
            } else {
                improvedData = CalculateCurrentDacCodeLinear(ch, targetValue);
            }
        } else {
            improvedData = CalculateCurrentDacCodeLinear(ch, targetValue);
        }

        txBuffer[0] = 0x02;
        txBuffer[1] = improvedData & 0xFF;
        txBuffer[2] = (improvedData >> 8) & 0xFF;
        GP8630_WriteFrame(ch, txBuffer, 3);
        return;
    }

#if 0

    // 如果启用了多点校准，使用插值补偿
    if (multiCalibEnabled[ch]) {
        // 在实测值序列中找到targetValue所在的区间
        uint8_t idx = 0;
        for (uint8_t i = 0; i < CALIB_POINTS_NUM - 1; i++) {
            if (targetValue >= multiCalibData[ch][i] && targetValue <= multiCalibData[ch][i + 1]) {
                idx = i;
                break;
            }
            if (i == CALIB_POINTS_NUM - 2) {
                // 如果超出范围，使用最后一个区间
                if (targetValue > multiCalibData[ch][CALIB_POINTS_NUM - 1]) {
                    idx = CALIB_POINTS_NUM - 2;
                } else if (targetValue < multiCalibData[ch][0]) {
                    idx = 0;
                }
            }
        }

        // 校准数据含义：当DAC输出calibTargetValues[i]时，实测为multiCalibData[ch][i]
        // 现在要输出targetValue（期望实际电流），反向计算需要的DAC值
        float t1 = calibTargetValues[idx];      // 区间起点DAC目标值
        float t2 = calibTargetValues[idx + 1];  // 区间终点DAC目标值
        float m1 = multiCalibData[ch][idx];     // 区间起点实测值
        float m2 = multiCalibData[ch][idx + 1]; // 区间终点实测值

        // 反向插值：从期望实测值targetValue，求对应的DAC输出值
        // (targetValue - m1) / (m2 - m1) = (dacOutput - t1) / (t2 - t1)
        float dacOutput;
        if (m2 != m1) {
            dacOutput = t1 + (targetValue - m1) * (t2 - t1) / (m2 - m1);
        } else {
            dacOutput = t1;
        }

        // 限制范围
        if (dacOutput < 0) dacOutput = 0;
        if (dacOutput > 400) dacOutput = 400;

        // 转换为DAC值 (满量程4mA = 400)
        data = (uint16_t)((dacOutput / 400.0f) * 0xFFFF);
    } else {
        // 未启用多点校准，使用原来的CZ补偿
        float value = targetValue;
        float fullS = channels[ch].full_scale_A / 10.0f;
        float zeroOffset = channels[ch].zero_offset_A / 10.0f;

        float adjustedValue = value;
        if (adjustedValue >= zeroOffset) {
            adjustedValue = adjustedValue - zeroOffset;
        } else {
            adjustedValue = 0;
        }
        data = (uint16_t)((adjustedValue / fullS) * 0xFFFF);
    }

    uint8_t buffer2[3];
    buffer2[0] = 0x02;
    buffer2[1] = data & 0xFF;
    buffer2[2] = (data >> 8) & 0xFF;

    HAL_I2C_Master_Transmit(&hi2c1, (0x58 + ch) << 1, buffer2, 3, HAL_MAX_DELAY);
#endif
}

void setVoltage(void) {
    float value = channels[uiState.channelFlag].voltage_value;
    float fullS = channels[uiState.channelFlag].full_scale_V / 10.0f;  // 转换为与value同基数(100)
    float zeroOffset = channels[uiState.channelFlag].zero_offset_V / 10.0f;  // 转换为与value同基数(100)
    // 应用零点偏移：实际输出 = 设定值 - 零点偏移（补偿正偏移）
    float adjustedValue = value;
    if (adjustedValue >= zeroOffset) {
        adjustedValue = adjustedValue - zeroOffset;
    } else {
        adjustedValue = 0;  // 防止负值
    }
    uint16_t data = (uint16_t)((adjustedValue / fullS) * 0xFFFF);

    // 发�?�第二条指令：带数据部分
    uint8_t buffer2[3];
    buffer2[0] = 0x02;               // 控制命令地址
    buffer2[1] = data & 0xFF;        // DATA Low
    buffer2[2] = (data >> 8) & 0xFF; // DATA High

    GP8630_WriteFrame(uiState.channelFlag, buffer2, 3);

}

void setPWMFrequency(void) {
      uint8_t ch = uiState.channelFlag;

      if (ch > 3) return;  // 防止数组越界

      float duty = channels[ch].pwm_duty_cycle;
      uint32_t freq = channels[ch].pwm_frequency;

      // 限制输入范围
      if (duty < 0.0f) duty = 0.0f;
      if (duty > 100.0f) duty = 100.0f;
      if (freq < 1) freq = 1;
      if (freq > 100) freq = 100;

      // 固定预分频：7200分频 => 定时器时钟为10kHz
      const uint32_t prescaler = 7199;
      const uint32_t timer_clk = 72000000;
      const uint32_t arr = (timer_clk / (prescaler + 1)) / freq - 1;
      const uint32_t ccr = (uint32_t)((arr + 1) * (duty / 100.0f));

      switch (ch)
      {
          case 0:
              htim1.Init.Prescaler = prescaler;
              htim1.Init.Period = arr;
              __HAL_TIM_SET_PRESCALER(&htim1, prescaler);
              __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
              htim1.Instance->EGR = TIM_EGR_UG;
              break;

          case 1:
              htim2.Init.Prescaler = prescaler;
              htim2.Init.Period = arr;
              __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
              __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
              htim2.Instance->EGR = TIM_EGR_UG;
              break;

          case 2:
              htim3.Init.Prescaler = prescaler;
              htim3.Init.Period = arr;
              __HAL_TIM_SET_PRESCALER(&htim3, prescaler);
              __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
              htim3.Instance->EGR = TIM_EGR_UG;
              break;

          case 3:
              htim4.Init.Prescaler = prescaler;
              htim4.Init.Period = arr;
              __HAL_TIM_SET_PRESCALER(&htim4, prescaler);
              __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
              htim4.Instance->EGR = TIM_EGR_UG;
              break;

          default:
              break;
      }
  }

void setPWMDutyCycle(void) {
    uint8_t ch = uiState.channelFlag;
    if (ch > 3) return;  // 防止越界

    float duty = channels[ch].pwm_duty_cycle;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;

    switch (ch) {
        case 0: htim = &htim1; tim_channel = TIM_CHANNEL_1; break;
        case 1: htim = &htim2; tim_channel = TIM_CHANNEL_2; break;
        case 2: htim = &htim3; tim_channel = TIM_CHANNEL_1; break;
        case 3: htim = &htim4; tim_channel = TIM_CHANNEL_1; break;
        default: return;
    }

    uint32_t ccr;
    if (duty < 100.0f) {
        ccr = (uint32_t)(htim->Init.Period * duty / 100.0f);
    } else {
        ccr = htim->Init.Period + 1;  // 超过ARR，确保输出高电平
    }

    __HAL_TIM_SET_COMPARE(htim, tim_channel, ccr);
}
void setFullScaleCompensation(void) {
    uint8_t buffer2[3];
    buffer2[0] = 0x02;               // 控制命令地址
    buffer2[1] = 0xFF;        // DATA Low
    buffer2[2] = 0xFF; // DATA High

    GP8630_WriteFrame(uiState.channelFlag, buffer2, 3);
}

void setMode(void) {
    if(channels[uiState.channelFlag].mode){
					buffer[0] = 0x01;           // 控制命令地址
					buffer[1] = 0x1C;           // 命令高位
					if (GP8630_WriteFrame(uiState.channelFlag, buffer, 2) != HAL_OK) return;

					HAL_Delay(2);
					setVoltage();
		}
		else{
					buffer[0] = 0x01;           // 控制命令地址
					buffer[1] = 0x24;           // 命令高位
					if (GP8630_WriteFrame(uiState.channelFlag, buffer, 2) != HAL_OK) return;

					HAL_Delay(2);
					setCurrent();
		}
}

void setChannelStatus(void) {

}
void showSettingsPage(void) {
    char str[50];
    OLED_Clear();

    uint8_t ch = uiState.channelFlag;  // 获取当前选中的�?�道

    // 第一行：显示当前选择的�?�道数字
    sprintf(str, "Ch%d Settings Back", ch + 1);
    OLED_ShowString(1, 0, str, 6);
		if (uiState.settingFlag == 0){
			OLED_ReverseArea(1, 0, 100, 8); 
		}
    // 第二行：显示电流/电压输出模式，并允许按键切换
    if (uiState.settingFlag == 1) {
        if (channels[ch].mode == 0) {
            sprintf(str, "Mode: Current");
        } else {
            sprintf(str, "Mode: Voltage");
        }
        OLED_ShowString(1, 10, str, 6);
				if(uiState.isSettingFlag)
					OLED_ReverseArea(35, 10, 45, 8);  // 高亮显示当前设置�??
				else
					OLED_ReverseArea(1, 10, 30, 8);  // 高亮显示当前设置�??
    } else {
        if (channels[ch].mode == 0) {
            sprintf(str, "Mode: Current");
        } else {
            sprintf(str, "Mode: Voltage");
        }
        OLED_ShowString(1, 10, str, 6);
    }

    // 第三行：显示当前的电流或电压输出�??
    if (uiState.settingFlag == 2) {
        if (channels[ch].mode == 0) {  // 电流模式
            sprintf(str, "I: %.2fmA", (float)channels[ch].current_value / 100);
        } else {  // 电压模式
            sprintf(str, "V: %.2fV", (float)channels[ch].voltage_value / 100);
        }
        OLED_ShowString(1, 20, str, 6);
				if(uiState.isSettingFlag)
					OLED_ReverseArea(17, 20, 42, 8);  // 高亮显示当前设置�??
				else
					OLED_ReverseArea(1, 20, 15, 8);  // 高亮显示当前设置�??
    } else {
        if (channels[ch].mode == 0) {  // 电流模式
            sprintf(str, "I: %.2fmA", (float)channels[ch].current_value / 100);
        } else {  // 电压模式
            sprintf(str, "V: %.2fV", (float)channels[ch].voltage_value / 100);
        }
        OLED_ShowString(1, 20, str, 6);
    }

    // 第四行：PWM频率设置
    if (uiState.settingFlag == 3) {
        sprintf(str, "PWM Freq: %dHz", channels[ch].pwm_frequency);
        OLED_ShowString(1, 30, str, 6);
			if(uiState.isSettingFlag)
        OLED_ReverseArea(60, 30, 40, 8);  // 高亮显示当前设置�??
			else
        OLED_ReverseArea(1, 30, 55, 8);  // 高亮显示当前设置�??
    } else {
        sprintf(str, "PWM Freq: %dHz", channels[ch].pwm_frequency);
        OLED_ShowString(1, 30, str, 6);
    }

    // 第五行：PWM占空比设�??
    if (uiState.settingFlag == 4) {
        sprintf(str, "PWM Duty: %d%%", channels[ch].pwm_duty_cycle);
        OLED_ShowString(1, 40, str, 6);
				if(uiState.isSettingFlag)
					OLED_ReverseArea(60, 40, 20, 8);  // 高亮显示当前设置�??
				else
					OLED_ReverseArea(1, 40, 55, 8);  // 高亮显示当前设置�??
    } else {
        sprintf(str, "PWM Duty: %d%%", channels[ch].pwm_duty_cycle);
        OLED_ShowString(1, 40, str, 6);
    }

    // 第六行：电流模式显示Calib入口，电压模式显示C和Z
    if(channels[ch].mode == 0) {
        // 电流模式：显示多点校准入口
        sprintf(str, "Calib %s", multiCalibEnabled[ch] ? "[ON]" : "[OFF]");
        OLED_ShowString(1, 50, str, 6);
        if (uiState.settingFlag == 5) {
            OLED_ReverseArea(1, 50, 75, 8);
        }
    } else {
        // 电压模式：显示C和Z
        sprintf(str, "C:%.3f Z:%.3f", (float)channels[ch].full_scale_V / 1000, (float)channels[ch].zero_offset_V / 1000);
        OLED_ShowString(1, 50, str, 6);
        if (uiState.settingFlag == 5) {  // 满量程校准
            if(uiState.isSettingFlag)
                OLED_ReverseArea(13, 50, 30, 8);
            else
                OLED_ReverseArea(1, 50, 10, 8);
        } else if (uiState.settingFlag == 6) {  // 零点偏移
            if(uiState.isSettingFlag)
                OLED_ReverseArea(58, 50, 35, 8);
            else
                OLED_ReverseArea(46, 50, 10, 8);
        }
    }

    OLED_Update();
}

// 初始化多点校准数据为默认值
void initMultiCalibData(void) {
    for (uint8_t ch = 0; ch < 4; ch++) {
        for (uint8_t i = 0; i < CALIB_POINTS_NUM; i++) {
            // 默认值等于目标值（无补偿）
            multiCalibData[ch][i] = calibTargetValues[i];
        }
        multiCalibEnabled[ch] = 0;
    }
}

// 输出校准点（直接输出DAC值，不经过补偿）
void outputCalibPoint(uint8_t pointIndex) {
    if (pointIndex >= CALIB_POINTS_NUM) return;

    // 计算DAC输出值：目标值 / 满量程(4mA=400) * 0xFFFF
    uint16_t targetValue = calibTargetValues[pointIndex];
    uint16_t data = (uint16_t)((float)targetValue / 400.0f * 0xFFFF);

    uint8_t buffer2[3];
    buffer2[0] = 0x02;
    buffer2[1] = data & 0xFF;
    buffer2[2] = (data >> 8) & 0xFF;

    HAL_I2C_Master_Transmit(&hi2c1, (0x58 + uiState.channelFlag) << 1, buffer2, 3, HAL_MAX_DELAY);
}

// 显示多点校准页面
void showCalibrationPage(void) {
    char str[50];
    OLED_Clear();

    uint8_t ch = uiState.channelFlag;

    // 第一行：标题
    sprintf(str, "Ch%d Calibration", ch + 1);
    OLED_ShowString(1, 0, str, 6);

    // 第二行：当前校准点
    sprintf(str, "Point %d/%d", calibPointIndex + 1, CALIB_POINTS_NUM);
    OLED_ShowString(1, 12, str, 6);

    // 第三行：目标输出值
    sprintf(str, "Target: %.2fmA", (float)calibTargetValues[calibPointIndex] / 100.0f);
    OLED_ShowString(1, 24, str, 6);

    // 第四行：用户输入的实际测量值
    sprintf(str, "Actual: %.2fmA", (float)calibInputValue / 100.0f);
    OLED_ShowString(1, 36, str, 6);
    OLED_ReverseArea(43, 36, 50, 8);  // 高亮输入值

    // 第五行：操作提示
    OLED_ShowString(1, 50, "K1+K2:Adj K3:Next", 6);

    OLED_Update();
}

void showUI(void) {
    char str[50];  // 分配足够的存储空间
    OLED_Clear();
    memset(str, 0, sizeof(str));

    // 第一行：显示四个芯片状态和设备ID
    strcpy(str, "C1  C2  C3  C4");
    for (uint8_t i = 0; i < 4; i++) {
        if (channels[i].status) {
            str[2 + i * 4] = 96 + 32;  // 设备连接时显示对勾符号
        } else {
            str[2 + i * 4] = 95 + 32;  // 设备未连接时显示叉号符号
        }
    }
    OLED_ShowString(1, 0, str, 6);
    // 在右侧显示设备ID
    sprintf(str, "#%02d", deviceID);
    OLED_ShowString(110, 0, str, 6);

    // 第二到五行：显示当前通道的电流或电压输出�??
    for (uint8_t i = 0; i < 4; i++) {
        if (channels[i].mode == 0) {  // 电流模式
            sprintf(str, "Ch%d: %.2fmA", i + 1, (float)channels[i].current_value / 100);
        } else {  // 电压模式
            sprintf(str, "Ch%d: %.2fV", i + 1, (float)channels[i].voltage_value / 100);
        }
        OLED_ShowString(1, 10 + (i * 10), str, 6);
    }

    // 高亮显示当前选中的�?�道
    OLED_ReverseArea(1, 10 + (uiState.channelFlag * 10), 67, 8);

    OLED_Update();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        char c = rx_buffer[rx_index];

        if (c == '\n') {
            rx_buffer[rx_index] = '\0';
            if (rx_index > 0 && rx_buffer[rx_index - 1] == '\r') {
                rx_buffer[rx_index - 1] = '\0';
            }

            // 拷贝命令
            strcpy((char *)uart_cmd_buffer, (char *)rx_buffer);
            uart_cmd_ready = 1;  // 设置标志位

            rx_index = 0;
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_index++;
            }
        }

        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
}

void ParseCommand(char *cmd) {
    uint8_t index;
    float value;
    char response[64];

    // 设置电流：I1=12.34
    if ((sscanf(cmd, "I%hhu=%f", &index, &value) == 2) && index >= 1 && index <= 4) {
        uint8_t ch = index - 1;
        uint16_t val = (uint16_t)(value * 100);

        if (value >= 0.0f && value <= (MAX_CURRENT / 100.0f)) {
            A[ch] = val;
            channels[ch].current_value = val;
            channels[ch].mode = 0;  // 电流模式
            uiState.channelFlag = ch;
            setMode();

            setCurrent();  // 立即应用

            snprintf(response, sizeof(response), "Set I%hhu = %.2fmA OK\r\n", index, value);
        } else {
            snprintf(response, sizeof(response), "I%hhu out of range (0~%.2fmA)\r\n", index, MAX_CURRENT / 100.0f);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }

    // 设置电压：V2=5.00
    else if ((sscanf(cmd, "V%hhu=%f", &index, &value) == 2) && index >= 1 && index <= 4) {
        uint8_t ch = index - 1;
        uint16_t val = (uint16_t)(value * 100);

        if (value >= 0.0f && value <= (MAX_VOLTAGE / 100.0f)) {
            V[ch] = val;
            channels[ch].voltage_value = val;
            channels[ch].mode = 1;  // 电压模式
            uiState.channelFlag = ch;
            setMode();
            setVoltage();

            snprintf(response, sizeof(response), "Set V%hhu = %.2fV OK\r\n", index, value);
        } else {
            snprintf(response, sizeof(response), "V%hhu out of range (0~%.2fV)\r\n", index, MAX_VOLTAGE / 100.0f);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }

    // 设置 PWM 频率：f3=500
    else if ((sscanf(cmd, "F%hhu=%f", &index, &value) == 2) && index >= 1 && index <= 4) {
        uint8_t ch = index - 1;
        uint16_t freq = (uint16_t)value;

        if (value >= (float)MIN_PWM_FREQ && value <= (float)MAX_PWM_FREQ) {
            channels[ch].pwm_frequency = freq;
            uiState.channelFlag = ch;

            setPWMFrequency();

            snprintf(response, sizeof(response), "Set f%hhu = %dHz OK\r\n", index, freq);
        } else {
            snprintf(response, sizeof(response), "Freq %dHz out of range (1~100Hz)\r\n", freq);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }

    // 设置 PWM 占空比：d4=80
    else if ((sscanf(cmd, "D%hhu=%f", &index, &value) == 2) && index >= 1 && index <= 4) {
        uint8_t ch = index - 1;
        uint8_t duty = (uint8_t)value;

        if (value >= 0.0f && value <= 100.0f) {
            channels[ch].pwm_duty_cycle = duty;
            uiState.channelFlag = ch;

            setPWMDutyCycle();

            snprintf(response, sizeof(response), "Set d%hhu = %d%% OK\r\n", index, duty);
        } else {
            snprintf(response, sizeof(response), "Duty %d%% out of range (0~100%%)\r\n", duty);
        }

        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }
		
	 else if ((sscanf(cmd, "M%hhu=%f", &index, &value) == 2) && index >= 1 && index <= 4) {
			uint8_t ch = index - 1;
			uint8_t mode = (uint8_t)value;

			if (value == 0.0f || value == 1.0f) {
					channels[ch].mode = mode;
					uiState.channelFlag = ch;

					setMode();

					snprintf(response, sizeof(response), "Set M%hhu = %s OK\r\n", index, mode == 0 ? "Current" : "Voltage");
			} else {
					snprintf(response, sizeof(response), "Invalid mode (use 0=current, 1=voltage)\r\n");
			}

			HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }

    // 设置设备ID：ID=5
    else if (sscanf(cmd, "ID=%hhu", &index) == 1) {
        if (index >= 1 && index <= MAX_DEVICE_ID) {
            deviceID = index;
            SaveCalibrationToFlash();  // 保存到Flash
            snprintf(response, sizeof(response), "Set ID = %d OK\r\n", deviceID);
        } else {
            snprintf(response, sizeof(response), "ID out of range (1~%d)\r\n", MAX_DEVICE_ID);
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }

    // 查询设备ID：ID?
    else if (strcmp(cmd, "ID?") == 0) {
        snprintf(response, sizeof(response), "ID = %d\r\n", deviceID);
        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }

    // 错误命令
    else {
        snprintf(response, sizeof(response), "Invalid command\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
    }
}



void Send_System_Params(void) {
    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer),
        "I1=%.2fmA, I2=%.2fmA, I3=%.2fmA, I4=%.2fmA\r\n",
        A[0] / 100.0f, A[1] / 100.0f, A[2] / 100.0f, A[3] / 100.0f);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

void KeyScan(void) {

    // KEY1 按键处理：增加当前设置项的值
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET) {
        if (key_state1 == 0) {
            key_state1 = 1;
            key_press_time1 = 0;
        } else {
							key_press_time1++;
							// 校准页面长按处理
							if (key_press_time1 >= LONG_PRESS_TIME && inCalibPage) {
								if (calibInputValue + GetLongPressStep(key_press_time1, 1, 5) <= 500) calibInputValue += GetLongPressStep(key_press_time1, 1, 5);  // 最大5mA
								else calibInputValue = 500;
							}
							else if (key_press_time1 >= LONG_PRESS_TIME && uiState.isSettingFlag) {
									if (uiState.settingFlag == 2) {  // 电流/电压值
											if (channels[uiState.channelFlag].mode == 0) {  // 电流模式
													if (channels[uiState.channelFlag].current_value + GetLongPressStep(key_press_time1, 1, 5) > MAX_CURRENT) {
															channels[uiState.channelFlag].current_value = 0;
													}
													else{
															channels[uiState.channelFlag].current_value += GetLongPressStep(key_press_time1, 1, 5);
													}
											} else {  // 电压模式
													if (channels[uiState.channelFlag].voltage_value + GetLongPressStep(key_press_time1, 1, 5) > MAX_VOLTAGE) {
															channels[uiState.channelFlag].voltage_value = 0;
													}
													else{
															channels[uiState.channelFlag].voltage_value += GetLongPressStep(key_press_time1, 1, 5);
													}
											}
									} else if (uiState.settingFlag == 3) {  // PWM频率
											if (channels[uiState.channelFlag].pwm_frequency + GetLongPressStep(key_press_time1, 1, 5) <= MAX_PWM_FREQ) {
													channels[uiState.channelFlag].pwm_frequency += GetLongPressStep(key_press_time1, 1, 5);  // 增加PWM频率
											}
											else
													channels[uiState.channelFlag].pwm_frequency = MIN_PWM_FREQ;  // 增加PWM频率
									} else if (uiState.settingFlag == 4) {  // PWM占空�??
											if (channels[uiState.channelFlag].pwm_duty_cycle + GetLongPressStep(key_press_time1, 1, 5) <= MAX_DUTY) {
													channels[uiState.channelFlag].pwm_duty_cycle += GetLongPressStep(key_press_time1, 1, 5);  // 增加PWM占空�??
											}
											else
													channels[uiState.channelFlag].pwm_duty_cycle = MIN_DUTY;  // 增加PWM占空�??
									} 
									else if (uiState.settingFlag == 5) {
										if(channels[uiState.channelFlag].mode){
													channels[uiState.channelFlag].full_scale_V += GetLongPressStep(key_press_time1, 10, 50);
										}
										else{
													channels[uiState.channelFlag].full_scale_A += GetLongPressStep(key_press_time1, 10, 50);
										}
									}
									else if (uiState.settingFlag == 6) {  // 零点偏移
										if(channels[uiState.channelFlag].mode){
											if (channels[uiState.channelFlag].zero_offset_V + GetLongPressStep(key_press_time1, 10, 50) <= MAX_ZERO_OFFSET)
												channels[uiState.channelFlag].zero_offset_V += GetLongPressStep(key_press_time1, 10, 50);
											else
												channels[uiState.channelFlag].zero_offset_V = MIN_ZERO_OFFSET;
										}
										else{
											if (channels[uiState.channelFlag].zero_offset_A + GetLongPressStep(key_press_time1, 10, 50) <= MAX_ZERO_OFFSET)
												channels[uiState.channelFlag].zero_offset_A += GetLongPressStep(key_press_time1, 10, 50);
											else
												channels[uiState.channelFlag].zero_offset_A = MIN_ZERO_OFFSET;
										}
									}
							}
        }
    } else {
        if (key_state1 == 1) {
            // 按键松开时重置计时和状态
						if (key_press_time1 < LONG_PRESS_TIME) {
							// 校准页面短按处理
							if(inCalibPage) {
								if (calibInputValue < 500) calibInputValue += 1;
							}
							// 根据当前设置项调整值
							else if(uiState.inSettingsPage){
								if(uiState.isSettingFlag){
									if(uiState.settingFlag == 1){
										channels[uiState.channelFlag].mode = !channels[uiState.channelFlag].mode;
									}
									else if (uiState.settingFlag == 2) {
                    if (channels[uiState.channelFlag].mode == 0) {  // 电流模式
                        if (channels[uiState.channelFlag].current_value == MAX_CURRENT) {
                            channels[uiState.channelFlag].current_value = 0;
                        }
												else{
                            channels[uiState.channelFlag].current_value += 1;
												}
                    } else {  // 电压模式
                        if (channels[uiState.channelFlag].voltage_value == MAX_VOLTAGE) {
                            channels[uiState.channelFlag].voltage_value = 0;
                        }
												else{
                            channels[uiState.channelFlag].voltage_value += 1;
												}
                    }
									} 
									else if (uiState.settingFlag == 3) {
											if (channels[uiState.channelFlag].pwm_frequency < MAX_PWM_FREQ) {
													channels[uiState.channelFlag].pwm_frequency += 1;  // 增加PWM频率
											}
											else
													channels[uiState.channelFlag].pwm_frequency = MIN_PWM_FREQ;  // 增加PWM频率
									} 
									else if (uiState.settingFlag == 4) {
											if (channels[uiState.channelFlag].pwm_duty_cycle < MAX_DUTY) {
													channels[uiState.channelFlag].pwm_duty_cycle += 1;  // 增加PWM占空�??
											}
											else
													channels[uiState.channelFlag].pwm_duty_cycle = MIN_DUTY;  // 增加PWM占空�??
									}
									else if (uiState.settingFlag == 5) {
										if(channels[uiState.channelFlag].mode){
													channels[uiState.channelFlag].full_scale_V += 1;
										}
										else{
													channels[uiState.channelFlag].full_scale_A += 1;
										}
									}
									else if (uiState.settingFlag == 6) {  // 零点偏移
										if(channels[uiState.channelFlag].mode){
											if (channels[uiState.channelFlag].zero_offset_V < MAX_ZERO_OFFSET)
												channels[uiState.channelFlag].zero_offset_V += 1;
											else
												channels[uiState.channelFlag].zero_offset_V = MIN_ZERO_OFFSET;
										}
										else{
											if (channels[uiState.channelFlag].zero_offset_A < MAX_ZERO_OFFSET)
												channels[uiState.channelFlag].zero_offset_A += 1;
											else
												channels[uiState.channelFlag].zero_offset_A = MIN_ZERO_OFFSET;
										}
									}
								}
								 else{
									if(uiState.settingFlag == 0)
										uiState.settingFlag = 6;  // 改为6
									else uiState.settingFlag -= 1;
								 }
							}
							else{
								if(uiState.channelFlag == 0)
									uiState.channelFlag = 3;
								else uiState.channelFlag -= 1;
							}
							key_state1 = 0;
						}
            key_state1 = 0;
            key_press_time1 = 0;
        }
    }

    // KEY2 按键处理：减少当前设置项的�??
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
        if (key_state2 == 0) {
            key_state2 = 1;
            key_press_time2 = 0;
        } else {
            key_press_time2++;
						// 校准页面长按处理
						if (key_press_time2 >= LONG_PRESS_TIME && inCalibPage) {
							if (calibInputValue >= GetLongPressStep(key_press_time2, 1, 5)) calibInputValue -= GetLongPressStep(key_press_time2, 1, 5);
							else calibInputValue = 0;
						}
						else if (key_press_time2 >= LONG_PRESS_TIME && uiState.isSettingFlag) {
							if (uiState.settingFlag == 2) {  // 电流/电压值
									if (channels[uiState.channelFlag].mode == 0) {  // 电流模式
											if (channels[uiState.channelFlag].current_value < GetLongPressStep(key_press_time2, 1, 5)) {
													channels[uiState.channelFlag].current_value = MAX_CURRENT;
											}
											else{
													channels[uiState.channelFlag].current_value -= GetLongPressStep(key_press_time2, 1, 5);
											}
									} else {  // 电压模式
											if (channels[uiState.channelFlag].voltage_value < GetLongPressStep(key_press_time2, 1, 5)) {
												channels[uiState.channelFlag].voltage_value = MAX_VOLTAGE;
											}
											else{
													channels[uiState.channelFlag].voltage_value -= GetLongPressStep(key_press_time2, 1, 5);
											}
									}
							} else if (uiState.settingFlag == 3) {  // PWM频率
									if (channels[uiState.channelFlag].pwm_frequency > GetLongPressStep(key_press_time2, 1, 5)) {
											channels[uiState.channelFlag].pwm_frequency -= GetLongPressStep(key_press_time2, 1, 5);  // 减少PWM频率
									}
									else
											channels[uiState.channelFlag].pwm_frequency = MAX_PWM_FREQ;  // 减少PWM频率
							} else if (uiState.settingFlag == 4) {  // PWM占空�??
									if (channels[uiState.channelFlag].pwm_duty_cycle >= GetLongPressStep(key_press_time2, 1, 5)) {
											channels[uiState.channelFlag].pwm_duty_cycle -= GetLongPressStep(key_press_time2, 1, 5);  // 减少PWM占空�??
									}
									else
										channels[uiState.channelFlag].pwm_duty_cycle = MAX_DUTY;
							} else if (uiState.settingFlag == 5) {
										if(channels[uiState.channelFlag].mode){
											if (channels[uiState.channelFlag].full_scale_V >= GetLongPressStep(key_press_time2, 10, 50))
													channels[uiState.channelFlag].full_scale_V -= GetLongPressStep(key_press_time2, 10, 50);
											else
													channels[uiState.channelFlag].full_scale_V = 0;
										}
										else{
											if (channels[uiState.channelFlag].full_scale_A >= GetLongPressStep(key_press_time2, 10, 50))
													channels[uiState.channelFlag].full_scale_A -= GetLongPressStep(key_press_time2, 10, 50);
											else
													channels[uiState.channelFlag].full_scale_A = 0;
										}
									}
									else if (uiState.settingFlag == 6) {  // 零点偏移
										if(channels[uiState.channelFlag].mode){
											if (channels[uiState.channelFlag].zero_offset_V >= GetLongPressStep(key_press_time2, 10, 50))
												channels[uiState.channelFlag].zero_offset_V -= GetLongPressStep(key_press_time2, 10, 50);
											else
												channels[uiState.channelFlag].zero_offset_V = MIN_ZERO_OFFSET;
										}
										else{
											if (channels[uiState.channelFlag].zero_offset_A >= GetLongPressStep(key_press_time2, 10, 50))
												channels[uiState.channelFlag].zero_offset_A -= GetLongPressStep(key_press_time2, 10, 50);
											else
												channels[uiState.channelFlag].zero_offset_A = MIN_ZERO_OFFSET;
										}
									}
            }
        }
    } else {
        if (key_state2 == 1) {
            if (key_press_time2 < LONG_PRESS_TIME) {
							// 校准页面短按处理
							if(inCalibPage) {
								if (calibInputValue > 0) calibInputValue -= 1;
							}
							else if(uiState.inSettingsPage){
								if(uiState.isSettingFlag){
                // 根据当前设置项调整值
									if(uiState.settingFlag == 1){
										channels[uiState.channelFlag].mode = !channels[uiState.channelFlag].mode;
									}
									else if (uiState.settingFlag == 2) {
										if (channels[uiState.channelFlag].mode == 0) {  // 电流模式
												if (channels[uiState.channelFlag].current_value == MIN_CURRENT) {
														channels[uiState.channelFlag].current_value = MAX_CURRENT;
												}
												else{
														channels[uiState.channelFlag].current_value -= 1;
												}
										} else {  // 电压模式
												if (channels[uiState.channelFlag].voltage_value == MIN_VOLTAGE) {
													channels[uiState.channelFlag].voltage_value = MAX_VOLTAGE;
												}
												else{
														channels[uiState.channelFlag].voltage_value -= 1;
												}
										}
									} else if (uiState.settingFlag == 3) {
											if (channels[uiState.channelFlag].pwm_frequency > 1)
													channels[uiState.channelFlag].pwm_frequency -= 1;  // 减少PWM频率
											else
													channels[uiState.channelFlag].pwm_frequency = MAX_PWM_FREQ;  // 减少PWM频率
									} else if (uiState.settingFlag == 4) {
											if (channels[uiState.channelFlag].pwm_duty_cycle > 0)
													channels[uiState.channelFlag].pwm_duty_cycle -= 1;  // 减少PWM占空�??
											else
												channels[uiState.channelFlag].pwm_duty_cycle = MAX_DUTY;
									} else if (uiState.settingFlag == 5) {
										if(channels[uiState.channelFlag].mode){
											if (channels[uiState.channelFlag].full_scale_V > 0)
													channels[uiState.channelFlag].full_scale_V -= 1;
										}
										else{
											if (channels[uiState.channelFlag].full_scale_A > 0)
													channels[uiState.channelFlag].full_scale_A -= 1;
										}
									}
									else if (uiState.settingFlag == 6) {  // 零点偏移
										if(channels[uiState.channelFlag].mode){
											if (channels[uiState.channelFlag].zero_offset_V > MIN_ZERO_OFFSET)
												channels[uiState.channelFlag].zero_offset_V -= 1;
											else
												channels[uiState.channelFlag].zero_offset_V = MAX_ZERO_OFFSET;
										}
										else{
											if (channels[uiState.channelFlag].zero_offset_A > MIN_ZERO_OFFSET)
												channels[uiState.channelFlag].zero_offset_A -= 1;
											else
												channels[uiState.channelFlag].zero_offset_A = MAX_ZERO_OFFSET;
										}
									}
								}
								else{
									uiState.settingFlag += 1;
									if(uiState.settingFlag > 6)  // 改为6
										uiState.settingFlag = 0;
								}
            }
							else{
								uiState.channelFlag += 1;
								if(uiState.channelFlag == 4)
									uiState.channelFlag = 0;
							}
            key_state2 = 0;
					}
            key_state2 = 0;
            key_press_time2 = 0;
        }
    }

    // KEY3 按键处理：切换高亮设置项
    if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_SET) {
        if (key_state3 == 0) {
            key_state3 = 1;
            key_press_time3 = 0;
        } else {
            key_press_time3++;
        }
    } 
		else {
				if (key_state3 == 1) {
							// 校准页面的按键处理
							if(inCalibPage) {
								// 保存当前校准点的输入值
								multiCalibData[uiState.channelFlag][calibPointIndex] = calibInputValue;

								// 进入下一个校准点
								calibPointIndex++;
								if(calibPointIndex >= CALIB_POINTS_NUM) {
									// 校准完成，启用多点校准并保存
									multiCalibEnabled[uiState.channelFlag] = 1;
									SaveCalibrationToFlash();
									setMode();
									setCurrent();
									// 退出校准页面
									inCalibPage = 0;
									calibPointIndex = 0;
								} else {
									// 输出下一个校准点，显示上次记录的actual值
									calibInputValue = multiCalibData[uiState.channelFlag][calibPointIndex];
									outputCalibPoint(calibPointIndex);
								}
							}
							else if(uiState.inSettingsPage){
										if(uiState.settingFlag == 0){
												uiState.isSettingFlag = 0;
												uiState.inSettingsPage = 0;
										}
										else if(uiState.settingFlag == 5 && channels[uiState.channelFlag].mode == 0){
											// 电流模式下进入多点校准页面
											inCalibPage = 1;
											calibPointIndex = 0;
											// 显示上次记录的actual值（如果有），否则显示目标值
											calibInputValue = multiCalibData[uiState.channelFlag][0];
											// 设置为电流模式并输出第一个校准点
											buffer[0] = 0x01;
											buffer[1] = 0x24;
											HAL_I2C_Master_Transmit(&hi2c1, (0x58 + uiState.channelFlag) << 1, buffer, 2, HAL_MAX_DELAY);
											outputCalibPoint(0);
										}
										else if(uiState.settingFlag == 5 && channels[uiState.channelFlag].mode == 1){
											// 电压模式下的满量程校准
											setFullScaleCompensation();
										}
									if(uiState.isSettingFlag){
										if(uiState.settingFlag == 1){
												setMode();
										}
										else if(uiState.settingFlag == 2){
												if(channels[uiState.channelFlag].mode){
													setVoltage();
												}
												else{
													setCurrent();
												}
										}
										else if(uiState.settingFlag == 3){
											setPWMFrequency();
										}
										else if(uiState.settingFlag == 4){
											setPWMDutyCycle();
										}
										else if(uiState.settingFlag == 5){
											if(channels[uiState.channelFlag].mode) {
												setVoltage();
												SaveCalibrationToFlash();
											}
											// 电流模式下settingFlag==5是进入校准页面，不在这里处理
										}
										else if(uiState.settingFlag == 6){  // 零点偏移确认时应用
											if(channels[uiState.channelFlag].mode) {
												setVoltage();
												SaveCalibrationToFlash();
											}
										}
									}
									if(!(uiState.settingFlag == 5 && channels[uiState.channelFlag].mode == 0)) {
										uiState.isSettingFlag = !uiState.isSettingFlag;
									}
							}
							else{
									uiState.inSettingsPage = 1;
									uiState.isSettingFlag = 0;
							}
								key_state3 = 0;
				}
		}
}

void SaveCalibrationToFlash(void) {
    CalibrationData_t calData = {0};
    for (uint8_t i = 0; i < 4; i++) {
        SanitizeMultiCalibChannel(i);
        calData.channels[i].full_scale_A = channels[i].full_scale_A;
        calData.channels[i].full_scale_V = channels[i].full_scale_V;
        calData.channels[i].zero_offset_A = channels[i].zero_offset_A;
        calData.channels[i].zero_offset_V = channels[i].zero_offset_V;
        // 保存多点校准数据
        for (uint8_t j = 0; j < CALIB_POINTS_NUM; j++) {
            calData.multi_calib[i].measured[j] = multiCalibData[i][j];
        }
        calData.multi_calib[i].enabled = multiCalibEnabled[i];
        calData.multi_calib[i].reserved = 0;
    }
    calData.device_id = deviceID;
    calData.reserved = 0;
    Flash_SaveCalibration(&calData);
}

void LoadCalibrationFromFlash(void) {
    CalibrationData_t calData;
    if (Flash_LoadCalibration(&calData) == HAL_OK) {
        // 数据有效，加载到通道结构体
        for (uint8_t i = 0; i < 4; i++) {
            channels[i].full_scale_A = calData.channels[i].full_scale_A;
            channels[i].full_scale_V = calData.channels[i].full_scale_V;
            channels[i].zero_offset_A = calData.channels[i].zero_offset_A;
            channels[i].zero_offset_V = calData.channels[i].zero_offset_V;
            // 加载多点校准数据
        for (uint8_t j = 0; j < CALIB_POINTS_NUM; j++) {
            multiCalibData[i][j] = calData.multi_calib[i].measured[j];
        }
        SanitizeMultiCalibChannel(i);
        multiCalibEnabled[i] = calData.multi_calib[i].enabled;
    }
        // 加载设备ID
        if (calData.device_id > 0 && calData.device_id <= MAX_DEVICE_ID) {
            deviceID = calData.device_id;
        }
    } else {
        // 数据无效，初始化多点校准数据为默认值
        initMultiCalibData();
    }
}

void Check_I2C_Devices(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        channels[i].status = 0;
			if(HAL_I2C_IsDeviceReady(&hi2c1, (0x58 + i) << 1, 2, 10) == HAL_OK) {
        channels[i].status = 1;
				buffer[0] = 0x01;           // 控制命令地址
				buffer[1] = 0x24;           // 命令高位
				HAL_I2C_Master_Transmit(&hi2c1, (0x58 + i) << 1, buffer, 2, HAL_MAX_DELAY);
				buffer[0] = 0x02;           // 控制命令地址
				buffer[1] = 0x00;          
				buffer[2] = 0x00;         
				HAL_I2C_Master_Transmit(&hi2c1, (0x58 + i) << 1, buffer, 3, HAL_MAX_DELAY);
        }
    }
}

void SetGP(void) {
    // 限制�??大�?�，避免 A[channelFlag] 超过 400
    if (A[channelFlag] > 400) {
        A[channelFlag] = 400;
    }
		
    uint16_t data = (uint16_t)(((float)A[channelFlag] / (float)C[channelFlag]) * 0xFFFF);

    // 发�?�第�??条指�??
    uint8_t buffer1[3];
    buffer1[0] = 0x01;           // 控制命令地址
    buffer1[1] = 0x24;           // 命令高位

    HAL_I2C_Master_Transmit(&hi2c1, (0x58 + channelFlag) << 1, buffer1, 2, HAL_MAX_DELAY);

    // 发�?�第二条指令：带数据部分
    uint8_t buffer2[3];
    buffer2[0] = 0x02;               // 控制命令地址
    buffer2[1] = data & 0xFF;        // DATA Low
    buffer2[2] = (data >> 8) & 0xFF; // DATA High

    HAL_I2C_Master_Transmit(&hi2c1, (0x58 + channelFlag) << 1, buffer2, 3, HAL_MAX_DELAY);
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
