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
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <inverse_kinematics.h>
#include <LSPB.h>
#include <pid.h>
#include <pca9685.h>
#include <math.h>
#include <string.h>

#define PWM_FREQUENCY 50
#define MAX_RESOLUTION 4095
#define MIN_PULSE_WIDTH 102.375f   // 1ms
#define MAX_PULSE_WIDTH 511.875f   // 2ms
#define PWM_PERIOD_MS 20.0f    // 20ms
#define DT 0.2
#define PI 3.14159265358979323846
#define BUFFER_SIZE 11


uint8_t rxBuffer[BUFFER_SIZE + 5]; // Receive_buffer
uint8_t frame[BUFFER_SIZE]; // Buffer chứa frame hợp lệ
uint8_t frameIndex = 0; // Chỉ số hiện tại trong frame
uint8_t frameReceived = 0; // Cờ nhận frame hoàn chỉnh

int active = 0; // State of robot
int position_received = 0; // Receive_state
float desired_position[3] = {3.7, 17.7, 2.7};

// Khai báo các handle semaphore và message queue
osSemaphoreId_t positionSemaphoreHandle;
osMessageQueueId_t uartQueueHandle;

// Khai báo các thuộc tính của semaphore và message queue
const osSemaphoreAttr_t positionSemaphore_attributes = {
    .name = "positionSemaphore"
};

const osMessageQueueAttr_t uartQueue_attributes = {
    .name = "uartQueue"
};
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Robot_Task */
osThreadId_t Robot_TaskHandle;
const osThreadAttr_t Robot_Task_attributes = {
  .name = "Robot_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void UART_Task_Init(void *argument);
void Robot_Task_Init(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float rad2deg(float radian) {
    return radian * (180.0f / PI);
}

void convertThetaArrayToDegrees(float arr[5])
{
    for (int b = 0; b < 5; b++)
    {
        arr[b] = rad2deg(arr[b]);
    }
}

float angleToPWM(float angle)
{
    float pwm_value = MIN_PULSE_WIDTH + (angle * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 180.0f;
    return pwm_value;
}

float PWMToAngle(int pwm_value)
{
    float angle = (pwm_value - MIN_PULSE_WIDTH) * 180.0f / (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);
    return angle;
}

void control(float *current_angles, float *target_angles, float *vmax, PID_Controller pid, LSPB_Params *lspb, float Kp, float Ki, float Kd, float dt)
{
	float angles[5];
    // 2. Khởi tạo quỹ đạo LSPB cho từng khớp
    for (int i = 0; i < 5; i++)
    {
        LSPB_Init(&lspb[i], current_angles[i], target_angles[i], vmax[i], 2 * vmax[i], 2.0f);
        current_angles[i] = angleToPWM(current_angles[i]);
    }
//	for (float t = 0; t <= lspb[0].t_total + 1; t += DT)
//	{
//		//3. LSPB để hoạch định quỹ đạo cho từng khớp tại th�?i điểm t
//        float target_angle = LSPB_CalculatePosition(&lspb[3], t);
//        // 4. Chuyển đổi góc đích sang xung PWM
//        target_angle = angleToPWM(target_angle);
//
//        PID_Init(&pid, Kp, Ki, Kd, target_angle, dt);
//
//        // Vòng lặp đi�?u khiển servo để đạt tới góc mục tiêu bằng PID
//        while (fabs(current_angles[3] - target_angle) > 0.1)
//        {
//            // 5. Tính toán PID để cập nhật xung PWM cho servo
//            current_angles[3] += PID_Compute(&pid, current_angles[3]) * dt;
//            // Chuyển đổi từ PWM sang góc
//            angles[3] = PWMToAngle(current_angles[3]);
//			PCA9685_SetServoAngle(6, angles[3]);
//        }
//        osDelay((int)(DT * 10)); // Delay theo mili giây
//	}
	for (int i = 3; i < 5; i++)
	{
		for (float t = 0; t <= lspb[0].t_total + 1; t += DT)
		{
			//3. LSPB để hoạch định quỹ đạo cho từng khớp tại th�?i điểm t
			float target_angle = LSPB_CalculatePosition(&lspb[i], t);
			// 4. Chuyển đổi góc đích sang xung PWM
			target_angle = angleToPWM(target_angle);

			PID_Init(&pid, Kp, Ki, Kd, target_angle, dt);

			// Vòng lặp đi�?u khiển servo để đạt tới góc mục tiêu bằng PID
			while (fabs(current_angles[i] - target_angle) > 0.1)
			{
				// 5. Tính toán PID để cập nhật xung PWM cho servo
				current_angles[i] += PID_Compute(&pid, current_angles[i]) * dt;

				// Chuyển đổi từ PWM sang góc
				angles[i] = PWMToAngle(current_angles[i]);
				if(i == 1)
				{
					PCA9685_SetServoAngle_1(2 * i, angles[i]);
				}
				else
				{
					PCA9685_SetServoAngle(2 * i, angles[i]);
				}
			}
			osDelay((int)(DT * 10)); // Delay theo mili giây
		}
	}
	for (int i = 0; i < 3; i++)
	{
		for (float t = 0; t <= lspb[0].t_total + 1; t += DT)
		{
			//3. LSPB để hoạch định quỹ đạo cho từng khớp tại th�?i điểm t
			float target_angle = LSPB_CalculatePosition(&lspb[i], t);
			// 4. Chuyển đổi góc đích sang xung PWM
			target_angle = angleToPWM(target_angle);

			PID_Init(&pid, Kp, Ki, Kd, target_angle, dt);

			// Vòng lặp đi�?u khiển servo để đạt tới góc mục tiêu bằng PID
			while (fabs(current_angles[i] - target_angle) > 0.1)
			{
				// 5. Tính toán PID để cập nhật xung PWM cho servo
				current_angles[i] += PID_Compute(&pid, current_angles[i]) * dt;

				// Chuyển đổi từ PWM sang góc
				angles[i] = PWMToAngle(current_angles[i]);
				if(i == 1)
				{
					PCA9685_SetServoAngle_1(2 * i, angles[i]);
				}
				else
				{
					PCA9685_SetServoAngle(2 * i, angles[i]);
				}
			}
			osDelay((int)(DT * 10)); // Delay theo mili giây
		}
	}

	active = 0;
}

void control_back(float *current_angles, float *target_angles, float *vmax, PID_Controller pid, LSPB_Params *lspb, float Kp, float Ki, float Kd, float dt)
{
	float angles[5];
    // 2. Khởi tạo quỹ đạo LSPB cho từng khớp
    for (int i = 0; i < 5; i++)
    {
        LSPB_Init(&lspb[i], current_angles[i], target_angles[i], vmax[i], 2 * vmax[i], 2.0f);
        current_angles[i] = angleToPWM(current_angles[i]);
    }
	for (int i = 2; i >= 0; i--)
	{
		for (float t = 0; t <= lspb[0].t_total + 1; t += DT)
		{
			//3. LSPB để hoạch định quỹ đạo cho từng khớp tại th�?i điểm t
			float target_angle = LSPB_CalculatePosition(&lspb[i], t);
			// 4. Chuyển đổi góc đích sang xung PWM
			target_angle = angleToPWM(target_angle);

			PID_Init(&pid, Kp, Ki, Kd, target_angle, dt);

			// Vòng lặp đi�?u khiển servo để đạt tới góc mục tiêu bằng PID
			while (fabs(current_angles[i] - target_angle) > 0.1)
			{
				// 5. Tính toán PID để cập nhật xung PWM cho servo
				current_angles[i] += PID_Compute(&pid, current_angles[i]) * dt;

				// Chuyển đổi từ PWM sang góc
				angles[i] = PWMToAngle(current_angles[i]);
				if(i == 1)
				{
					PCA9685_SetServoAngle_1(2 * i, angles[i]);
				}
				else
				{
					PCA9685_SetServoAngle(2 * i, angles[i]);
				}
			}
			osDelay((int)(DT * 10)); // Delay theo mili giây
		}
	}
	for (int i = 4; i >= 3; i--)
	{
		for (float t = 0; t <= lspb[0].t_total + 1; t += DT)
		{
			//3. LSPB để hoạch định quỹ đạo cho từng khớp tại th�?i điểm t
			float target_angle = LSPB_CalculatePosition(&lspb[i], t);
			// 4. Chuyển đổi góc đích sang xung PWM
			target_angle = angleToPWM(target_angle);

			PID_Init(&pid, Kp, Ki, Kd, target_angle, dt);

			// Vòng lặp đi�?u khiển servo để đạt tới góc mục tiêu bằng PID
			while (fabs(current_angles[i] - target_angle) > 0.1)
			{
				// 5. Tính toán PID để cập nhật xung PWM cho servo
				current_angles[i] += PID_Compute(&pid, current_angles[i]) * dt;

				// Chuyển đổi từ PWM sang góc
				angles[i] = PWMToAngle(current_angles[i]);
				if(i == 1)
				{
					PCA9685_SetServoAngle_1(2 * i, angles[i]);
				}
				else
				{
					PCA9685_SetServoAngle(2 * i, angles[i]);
				}
			}
			osDelay((int)(DT * 10)); // Delay theo mili giây
		}
	}
//	for (float t = 0; t <= lspb[0].t_total + 1; t += DT)
//	{
//		//3. LSPB để hoạch định quỹ đạo cho từng khớp tại th�?i điểm t
//        float target_angle = LSPB_CalculatePosition(&lspb[3], t);
//        // 4. Chuyển đổi góc đích sang xung PWM
//        target_angle = angleToPWM(target_angle);
//
//        PID_Init(&pid, Kp, Ki, Kd, target_angle, dt);
//
//        // Vòng lặp đi�?u khiển servo để đạt tới góc mục tiêu bằng PID
//        while (fabs(current_angles[3] - target_angle) > 0.1)
//        {
//            // 5. Tính toán PID để cập nhật xung PWM cho servo
//            current_angles[3] += PID_Compute(&pid, current_angles[3]) * dt;
//            // Chuyển đổi từ PWM sang góc
//            angles[3] = PWMToAngle(current_angles[3]);
//			PCA9685_SetServoAngle(6, angles[3]);
//        }
//        osDelay((int)(DT * 10)); // Delay theo mili giây
//	}

	active = 0;
}

void calculate_vmax(float *current_angles, float *target_angles, float *vmax)
{
	for (int i = 0; i < 5; i++)
	{
//		vmax[i] = fabs(target_angles[i] - current_angles[i]) / 1.0;
		vmax[i] = 2.0 * fabs(target_angles[i] - current_angles[i]) / 3.0;
	}
}

void swap_bytes(uint8_t *input, uint8_t *result, int end)
{
	for (int cnt = 0; cnt < sizeof(result); cnt++)
	{
		result[cnt] = input[end];
		end--;
	}
}

void SendRequest(UART_HandleTypeDef *huart)
{
    uint8_t frame[3];
    frame[0] = 0x02;       // Byte bắt đầu (ví dụ: 0x02 - STX)
    frame[1] = 0x01;       // Byte tín hiệu (cố định là 0x01)
    frame[2] = 0x03;       // Byte kết thúc (ví dụ: 0x03 - ETX)

    HAL_UART_Transmit(huart, frame, sizeof(frame), HAL_MAX_DELAY);
}

//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
//    // Callback khi nhận nửa buffer
//    processData();
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1)
{
    if (huart1->Instance == USART1)
    {
        // �?ẩy dữ liệu từ rxBuffer vào hàng đợi
        osMessageQueuePut(uartQueueHandle, rxBuffer, 0, 0);

        // Restart DMA để tiếp tục nhận dữ liệu
        HAL_UART_Receive_DMA(&huart1, rxBuffer, BUFFER_SIZE);
    }
//    // Callback khi nhận hoàn tất buffer
//    processData();
}

void processData()
{
    // Dừng DMA để tránh nhận thêm dữ liệu khi đang xử lý
    HAL_UART_DMAStop(&huart1);
    for (uint8_t i = 0; i < BUFFER_SIZE ; i++)
    {
        uint8_t data = rxBuffer[i];
        if (frameIndex == 0)
        {
            // Kiểm tra header
            if (data == 01)
            {
                frame[frameIndex++] = data; // Lưu header
            }
        }
        else
        {
            frame[frameIndex++] = data;
            // Kiểm tra footer
            if (data == 0xff)
            {
                frameReceived = 1; // �?ã nhận frame hoàn chỉnh
                break; // Thoát vòng lặp
            }
            // Nếu nhận đủ dữ liệu (9 bytes) mà không gặp footer
            if (frameIndex >= 11)
            {
                // �?ặt lại chỉ số
                frameIndex = 0; // Reset để nhận frame mới
                break;
            }
        }
    }

    // Nếu frame đã được nhận hoàn chỉnh
    if (frameReceived)
    {
        handleFrame(frame);
        frameIndex = 0; // Reset để nhận frame mới
        frameReceived = 0; // Reset c�?
    }

}

void handleFrame(uint8_t *frame)
{
    // �?�?c t�?a độ x từ frame (byte 1-4)
	float x;
    float y;
    uint8_t temp_1[4];
    uint8_t temp_2[4];

    swap_bytes(frame, temp_2, 8);
    swap_bytes(frame, temp_1, 4);

    // Chuyển đổi 4 byte thành float cho t�?a độ x
    memcpy(&x, &temp_1[0], sizeof(float));

    // Chuyển đổi 4 byte thành float cho t�?a độ y
    memcpy(&y, &temp_2[0], sizeof(float));

    // Tính toán checksum nếu có
    uint8_t checksum = frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^
                       frame[6] ^ frame[7] ^ frame[8];

    // Kiểm tra checksum (nếu sử dụng)
    if (checksum == frame[9])
    {
//    	if(x <= 10.0)
//    	{
//        	if(y <= 3.0)
//        	{
//            	desired_position[1] = y + 7.23 + 0.3 + (fabs(x - 6.0)) * 0.1 / 1.0; // error caused by field of view of camera
//            	desired_position[0] = x - 2.9225 - (x - 7.5) * 0.1;
//        	}
//        	else if(y <= 4.4)
//        	{
//            	desired_position[1] = y + 7.22 + 0.3 + (fabs(x - 8.5)) * 0.1 / 1.0; // error caused by field of view of camera
//            	desired_position[0] = x - 3.2725;
//        	}
//        	else if(y <= 5.5)
//        	{
//            	desired_position[1] = y + 6.83 + 1.0 + (fabs(x - 6.0)) * 0.1 / 1.0; // error caused by field of view of camera
//            	desired_position[0] = x - 3.2725;
//        	}
//        	else if(y <= 7.0)
//        	{
//            	desired_position[1] = y + 7.13 + 0.5 + (fabs(x - 6.0)) * 0.1 / 1.0;
//            	desired_position[0] = x - 3.0725 - 0.5;
//        	}
//        	else
//        	{
//            	desired_position[1] = y + 7.83 + (fabs(x - 6.0)) * 0.1 / 1.0;
//            	desired_position[0] = x - 3.32;
//        	}
//    	}
//    	else
//    	{
//        	if(y <= 3.0)
//        	{
//            	desired_position[1] = y + 7.03 - 0.4 + (fabs(x - 6.0)) * 0.1 / 1.0; // error caused by field of view of camera
//            	desired_position[0] = x - 2.9225 + 1.0 - (x - 7.5) * 0.1;
//        	}
//        	else if(y <= 4.4)
//        	{
//            	desired_position[1] = y + 7.22 + (fabs(x - 6.0)) * 0.1 / 1.5; // error caused by field of view of camera
//            	desired_position[0] = x - 3.2725 + 1.0;
//        	}
//        	else if(y <= 5.5)
//        	{
//            	desired_position[1] = y + 6.83 + 0.8 + (fabs(x - 6.0)) * 0.1 / 1.5; // error caused by field of view of camera
//            	desired_position[0] = x - 3.2725 + 1.0;
//        	}
//        	else if(y <= 7.0)
//        	{
//            	desired_position[1] = y + 7.13 + 0.8 + (fabs(x - 6.0)) * 0.1 / 1.5;
//            	desired_position[0] = x - 3.0725 + 1.0 - 0.5;
//        	}
//        	else
//        	{
//            	desired_position[1] = y + 7.83 + 1.5 + (fabs(x - 6.0)) * 0.1 / 1.5;
//            	desired_position[0] = x - 4.32 + 1.0;
//        	}
//    	}
    	if(y >= 6.0)
    	{
        	desired_position[1] = y + 6.0;
        	desired_position[0] = x - 11;
    	}
    	else if(y >= 5.0)
    	{
        	desired_position[1] = y + 6.5;
        	desired_position[0] = x - 11;
    	}
    	else
    	{
        	desired_position[1] = y + 6.7;
        	desired_position[0] = x - 11;
    	}
    	position_received = 1;
    }
}

void drop_object()
{
	PCA9685_SetServoAngle(10, 35);
}

void grasp_object()
{
	PCA9685_SetServoAngle(10, 80);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(50); // PCA Initial with frequency 50 Hz
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  // Tạo semaphore
  positionSemaphoreHandle = osSemaphoreNew(1, 0, &positionSemaphore_attributes);

  // Tạo message queue
  uartQueueHandle = osMessageQueueNew(10, sizeof(uint8_t), &uartQueue_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(UART_Task_Init, NULL, &UART_Task_attributes);

  /* creation of Robot_Task */
  Robot_TaskHandle = osThreadNew(Robot_Task_Init, NULL, &Robot_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  // Khởi động nhận dữ liệu qua DMA
//  HAL_UART_Receive_DMA(&huart1, rxBuffer, BUFFER_SIZE);
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_UART_Task_Init */
/**
  * @brief  Function implementing the UART_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_UART_Task_Init */
void UART_Task_Init(void *argument)
{
  /* USER CODE BEGIN 5 */
    HAL_UART_Receive_DMA(&huart1, rxBuffer, BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
      if ((osMessageQueueGet(uartQueueHandle, rxBuffer, NULL, osWaitForever) == osOK) && (active == 0)) //Complete receiving data
      {
          processData();
          memset(rxBuffer, 0, BUFFER_SIZE);
          if (position_received)
          {
              osSemaphoreRelease(positionSemaphoreHandle); //Initialize Robot Task
              active = 1;
              HAL_UART_Receive_DMA(&huart1, rxBuffer, BUFFER_SIZE);
          }
      }
      osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Robot_Task_Init */
/**
* @brief Function implementing the Robot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_Task_Init */
void Robot_Task_Init(void *argument)
{
  /* USER CODE BEGIN Robot_Task_Init */
	  // PID Initial
	  PID_Controller pid;
	  float Kp = 1, Ki = 0.7, Kd = 0;
	  float dt = 0.01;

	  //LSPB parameters declaration
	  LSPB_Params lspb[5];

	  InverseKinematics ik;
	  InverseKinematics_Init(&ik, 14.4, 10.5, 13.0, 7.0, 11.0);  // Define parameters of DH table


  /* Infinite loop */
  for(;;)
  {
	  float target_angles[5];
	  float current_angles[5] = {0.0, 90.0, 90.0, 90.0, 90.0};
	  float temp[5] = {0.0, 90.0, 90.0, 90.0, 90.0};
	  float vmax[5];
	  float px = desired_position[0];
	  float py = desired_position[1];
	  float d = sqrt(px * px + py * py);

      //Wait for signal
      if ((osSemaphoreAcquire(positionSemaphoreHandle, osWaitForever) == osOK) && (position_received == 1))
      {
    	  if((d <= 19.0) && (d >= 10.3))
    	  {
              //Calculate angles respected to desired position
              InverseKinematics_Calculate(&ik, desired_position, target_angles);
              convertThetaArrayToDegrees(target_angles);

              //Calculate max velocity
              calculate_vmax(current_angles, target_angles, vmax);
              drop_object();// Open gripper

              //Move to desired position
              control(current_angles, target_angles, vmax, pid, lspb, Kp, Ki, Kd, dt);
              osDelay(500);
              grasp_object();
              osDelay(500);

              //Move back to home position
              control_back(target_angles, temp, vmax, pid, lspb, Kp, Ki, Kd, dt);
              osDelay(500);
              drop_object();
    	  }

          //Update states
          position_received = 0;
          active = 0;
          SendRequest(&huart1);

      }
      osDelay(1000);
  }
  /* USER CODE END Robot_Task_Init */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
