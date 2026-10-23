/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	MODE_STREAMING,
	MODE_SNAPSHOT
} CaptureMode_t;

typedef struct {
	uint32_t freq;
  uint16_t psc;
	uint16_t arr;
} FreqConfig_t;

#pragma pack(push, 1)  // Ép compiler đóng gói chặt 1-byte, không chèn byte rác căn lề
typedef struct {
  uint8_t  magic[2]; // 0xAA, 0x55 (Mốc đồng bộ nhận diện đầu gói)
  uint8_t  pkt_type;
  uint8_t  seq_num;  // Số thứ tự gói (0 - 255)
  uint16_t data_len; // Chiều dài dữ liệu thực tế theo sau (tính bằng byte)
} PacketHeader_t;

typedef enum {
  PKT_STREAM_DATA     = 0x01,
  PKT_SNAPSHOT_DATA   = 0x02,
  PKT_CONFIG_OK       = 0x10,
  PKT_CAPTURE_STARTED = 0x11,
  PKT_CAPTURE_DONE    = 0x12,
  PKT_ERROR           = 0x7F
} PacketType_t;

typedef enum {
  ERR_NOT_CONFIGURED  = 0x01,
  ERR_INVALID_CONFIG  = 0x02,
  ERR_SNAPSHOT_LIMIT  = 0x03,
  ERR_CAPTURE_START   = 0x04,
  ERR_USB_TRANSMIT    = 0x05,
  ERR_DMA_OVERRUN     = 0x06
} PacketError_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE       10240U
#define CMD_BUFFER_SIZE		32U
#define SNAPSHOT_MAX_SAMPLES 10000U
#define STREAMING_MAX_SAMPLES 100000U
#define STREAMING_MAX_BLOCK_SIZE 1024U
#define HALF_BUFFER_SIZE  (BUFFER_SIZE / 2)
#define NUM_FREQS (sizeof(Freq_LUT) / sizeof(Freq_LUT[0]))
#define NUM_ALLOWED_SAMPLES (sizeof(ALLOWED_SAMPLES) / sizeof(ALLOWED_SAMPLES[0]))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */
uint8_t capture_buffer[BUFFER_SIZE];
char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t flag_half_ready = 0;
volatile uint8_t flag_full_ready = 0;
volatile uint8_t flag_snapshot_done = 0;
volatile uint8_t flag_capturing = 0;
uint8_t flag_configured = 0;
volatile uint8_t flag_cmd_ready = 0;
static uint8_t tx_staging_buffer[STREAMING_MAX_BLOCK_SIZE];
volatile uint8_t flag_stream_overrun = 0U;

CaptureMode_t current_mode = MODE_STREAMING;

uint32_t target_samples = 0;
uint32_t samples_captured = 0;
uint16_t stream_block_size = 1U;

const FreqConfig_t Freq_LUT[] = {
	{          1,   1199,   59999 },
  {          2,    599,   59999 },
  {          5,    239,   59999 },
  {         10,    119,   59999 },
  {         20,     59,   59999 },
  {         50,     23,   59999 },
  {        100,     11,   59999 },
  {        200,      5,   59999 },
  {        500,      2,   47999 },
  {       1000,      1,   35999 },
  {       2000,      0,   35999 },
  {       5000,      0,   14399 },
  {      10000,      0,    7199 },
  {      20000,      0,    3599 },
  {      50000,      0,    1439 },
  {     100000,      0,     719 },
  {     200000,      0,     359 },
  {     500000,      0,     143 },
  {    1000000,      0,      71 },
  {    2000000,      0,      35 }
};
const uint32_t ALLOWED_SAMPLES[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000};

// Biến tĩnh dùng chung để gửi Header
static PacketHeader_t tx_header = { {0xAA, 0x55}, 0, 0, 0 };
static uint8_t global_seq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t USB_Transmit_Safe(uint8_t *pData, uint16_t length);
uint8_t Send_Data_Packet(uint8_t type, uint8_t *pData, uint16_t length);

void My_DMA_HalfConvCallback(DMA_HandleTypeDef *hdma);
void My_DMA_FullConvCallback(DMA_HandleTypeDef *hdma);

static uint8_t Copy_Ready_Stream_Block(volatile uint8_t *ready_flag, const uint8_t *source, uint16_t length);

HAL_StatusTypeDef LogicAnalyzer_StartCapture(void);
void LogicAnalyzer_StopCapture(void);

uint8_t Validate_And_Set_Config(uint32_t freq, uint32_t samples);

void Process_Command(const char *cmd);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t USB_Transmit_Safe(uint8_t *pData, uint16_t length) {
  extern USBD_HandleTypeDef hUsbDeviceFS;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  // Chờ CDC rảnh (tối đa 50ms)
  uint32_t tickstart = HAL_GetTick();
  while (CDC_Transmit_FS(pData, length) == USBD_BUSY) {
    if ((HAL_GetTick()  - tickstart) > 50) {
      return 1; //Lỗi USB bận quá lâu
    }
  }

  // Nếu CDC đã rảnh, chờ PHẦN CỨNG gửi gói tin hiện tại ra ngoài rồi mới nạp gói tin mới (50ms)
  tickstart = HAL_GetTick();
  while (hcdc != NULL && hcdc->TxState != 0) {
    if ((HAL_GetTick() - tickstart) > 50) {
        return 2; // Lỗi Phần cứng không hoàn tất truyền;
    }
  }

  return 0; //Truyền thành công
}

uint8_t Send_Data_Packet(uint8_t type, uint8_t *pData, uint16_t length) {
  if (length > STREAMING_MAX_BLOCK_SIZE || length > 0U && pData == NULL) {
    return 3U;
  }
  
  tx_header.pkt_type = type;
  tx_header.seq_num = global_seq++;
  tx_header.data_len = length;

  //Gửi 6 bytes Header
  if (USB_Transmit_Safe((uint8_t*)&tx_header, (uint16_t)sizeof(PacketHeader_t)) != 0U) {
    return 1U; //Truyền Header thất bại
  }
  //Gửi dữ liệu thực tế
  if (length > 0U && USB_Transmit_Safe(pData, length) != 0) {
    return 2U; //Truyền dữ liệu thất bại
  }
  return 0U; //Truyền thành công
}

static uint8_t Send_Status_Packet(uint8_t type) {
  return Send_Data_Packet(type, NULL, 0U);
}
static uint8_t Send_Error_Packet(PacketError_t error) {
  uint8_t error_code = (uint8_t)error;
  return Send_Data_Packet(PKT_ERROR, &error_code, 1U);
}
static uint8_t Send_Capture_Done_Packet(uint32_t total_samples) {
  uint8_t payload[4];

  payload[0] = (uint8_t)(total_samples & 0xFFU);
  payload[1] = (uint8_t)((total_samples >> 8) & 0xFFU);
  payload[2] = (uint8_t)((total_samples >> 16) & 0xFFU);
  payload[3] = (uint8_t)((total_samples >> 24) & 0xFFU);

  return Send_Data_Packet(PKT_CAPTURE_DONE, payload, sizeof(payload));
}

void My_DMA_HalfConvCallback(DMA_HandleTypeDef *hdma) {
  if (current_mode == MODE_STREAMING) {
    if (flag_half_ready != 0U) {
      flag_stream_overrun = 1U;
    }
    else {
      flag_half_ready = 1U;
    }
  }
}

void My_DMA_FullConvCallback(DMA_HandleTypeDef *hdma) {
	if (current_mode == MODE_STREAMING) {
    if (flag_full_ready != 0U) {
      flag_stream_overrun = 1U;
    }
    else {
      flag_full_ready = 1U;
    }
  }
  else {
    flag_snapshot_done = 1U;
  }
}

HAL_StatusTypeDef LogicAnalyzer_StartCapture(void) {
    DMA_HandleTypeDef *hdma = htim1.hdma[TIM_DMA_ID_UPDATE];
    uint32_t dma_length;

    if (hdma == NULL ||
        hdma->State == HAL_DMA_STATE_BUSY ||
        target_samples == 0U) {
        return HAL_ERROR;
    }

    if (current_mode == MODE_SNAPSHOT) {
      if (target_samples > SNAPSHOT_MAX_SAMPLES || target_samples > BUFFER_SIZE) {
        return HAL_ERROR;
      }
      hdma->Init.Mode = DMA_NORMAL;
      dma_length = target_samples;
    }
    else {
      if (target_samples > STREAMING_MAX_SAMPLES ||
          stream_block_size == 0U ||
          stream_block_size > STREAMING_MAX_BLOCK_SIZE) {
        return HAL_ERROR;
      }
      hdma->Init.Mode = DMA_CIRCULAR;
      dma_length = 2U * stream_block_size;
    }

    HAL_TIM_Base_Stop(&htim1);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);

    if (HAL_DMA_Init(hdma) != HAL_OK)
    {
        return HAL_ERROR;
    }

    flag_half_ready = 0U;
    flag_full_ready = 0U;
    flag_snapshot_done = 0U;
    flag_stream_overrun = 0U;

    hdma->XferHalfCpltCallback = My_DMA_HalfConvCallback;
    hdma->XferCpltCallback = My_DMA_FullConvCallback;

    if (HAL_DMA_Start_IT(hdma,
                         (uint32_t)&GPIOB->IDR,
                         (uint32_t)capture_buffer,
                         dma_length) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
        (void)HAL_DMA_Abort(hdma);
        return HAL_ERROR;
    }

    return HAL_OK;
}

void LogicAnalyzer_StopCapture(void) {
	HAL_TIM_Base_Stop(&htim1);
	__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
	HAL_DMA_Abort(htim1.hdma[TIM_DMA_ID_UPDATE]);
}

uint8_t Validate_And_Set_Config(uint32_t freq, uint32_t samples) {
  int freq_index = -1;
	uint8_t flag_sample_valid = 0U;
	//Check tan so so thu may trong bang
	for (uint8_t i = 0U; i < NUM_FREQS; i++) {
		if (freq == Freq_LUT[i].freq) {
			freq_index = (int)i;
			break;
		}
	}
	//Check so mau thu may trong bang
	for (uint8_t i = 0U; i < NUM_ALLOWED_SAMPLES; i++) {
		if (samples == ALLOWED_SAMPLES[i]) {
			flag_sample_valid = 1U;
			break;
		}
	}
	
	if (freq_index == -1 || flag_sample_valid == 0U) {
		return 1U; //loi 1: tham so khong nam trong dai quy dinh
	}

	if (freq >= 500000U) {
    if (samples > SNAPSHOT_MAX_SAMPLES) {
      return 2U; //loi 2: Samples trong snapshot bi vuot qua
    }
		current_mode = MODE_SNAPSHOT;
	}
	else {
    if (samples > STREAMING_MAX_SAMPLES) {
      return 1U; //loi 1: tham so khong nam trong dai quy dinh
    }
		current_mode = MODE_STREAMING;
    uint32_t block_size = (freq + 19U) / 20U;

    if (block_size == 0U) {
      block_size = 1U;
    }
    if (block_size > STREAMING_MAX_BLOCK_SIZE) {
      block_size = STREAMING_MAX_BLOCK_SIZE;
    }
    if (block_size > samples) {
      block_size = samples;
    }

    stream_block_size = (uint16_t)block_size;
	}
	
	target_samples = samples;
	
	htim1.Instance->PSC = Freq_LUT[freq_index].psc;
	htim1.Instance->ARR = Freq_LUT[freq_index].arr;
	htim1.Instance->EGR = TIM_EGR_UG;
	htim1.Instance->SR = ~TIM_SR_UIF;

	flag_configured = 1U;
	return 0U; //Nap tan so va so mau thanh cong
}

static uint8_t Copy_Ready_Stream_Block(volatile uint8_t *ready_flag, const uint8_t *source, uint16_t length) {
  uint32_t primask;
  uint8_t ready;
  uint8_t overrun;

  if (length == 0U || length > STREAMING_MAX_BLOCK_SIZE || source == NULL) {
    return 2U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  ready = *ready_flag;
  if (ready != 0U) {
    *ready_flag = 0U; //hạ cờ để đánh dấu đang xử lý
  }

  __set_PRIMASK(primask);

  if (ready == 0U) {
    return 0U; //chưa có block mới
  }

  memcpy(tx_staging_buffer, source, length);

  primask = __get_PRIMASK();
  __disable_irq();

  overrun = (uint8_t)(flag_stream_overrun || *ready_flag); //check lại xem có thể bị ghi đè khi đang copy hay chưa

  __set_PRIMASK(primask);

  if (overrun != 0U) {
    return 2U; /* Block có thể đã bị DMA ghi đè trong lúc copy */
  }
  return 1U; //copy an toàn sang staging
}

void Process_Command(const char *cmd) {
	//Re nhanh check CMD la START hay CONFIGURE
	if (*cmd == 'S') {
		if (flag_configured == 1U) { // Check da cau hinh hay chua
			samples_captured = 0U;

      if (LogicAnalyzer_StartCapture() == HAL_OK) {
        flag_capturing = 1U;

        if (Send_Status_Packet(PKT_CAPTURE_STARTED) != 0U) {
          LogicAnalyzer_StopCapture();
          flag_capturing = 0U;
          (void)Send_Error_Packet(ERR_USB_TRANSMIT);
        }
      }
      else {
        flag_capturing = 0U;
        (void)Send_Error_Packet(ERR_CAPTURE_START);
      }
		}
		else if (flag_configured == 0) { // Neu chua cau hinh thi in loi
			(void)Send_Error_Packet(ERR_NOT_CONFIGURED);
		}
	}
	//Re nhanh check CMD la START hay CONFIG
	// Example: CONFIGURE 10000 5000
	else if (*cmd == 'C') {
		uint32_t cmd_freq = 0;
		uint32_t cmd_samples = 0;
		cmd += 10; // nhay den so thu nhat
		//Vong lap lay so thu nhat
		while (*cmd >= '0' && *cmd <= '9') {
			cmd_freq = cmd_freq * 10 + (*cmd - '0');
			cmd++;
		}
		cmd++; //Bo qua khoang trang giua tan so va so mau
		//Vong lap lay so thu hai
		while (*cmd >= '0' && *cmd <= '9') {
			cmd_samples = cmd_samples * 10 + (*cmd - '0');
			cmd++;
		}
		
    uint8_t result = Validate_And_Set_Config(cmd_freq, cmd_samples);
    if (result == 0U) {
      if (Send_Status_Packet(PKT_CONFIG_OK) != 0U) {
        (void)Send_Error_Packet(ERR_USB_TRANSMIT);
      }
    }
    else if (result == 1U) {
      (void)Send_Error_Packet(ERR_INVALID_CONFIG);
    }
    else if (result == 2U) {
      (void)Send_Error_Packet(ERR_SNAPSHOT_LIMIT);
    }
	}
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
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (flag_cmd_ready == 1U) {
      flag_cmd_ready = 0U;
      Process_Command(cmd_buffer);
    }

    if (current_mode == MODE_SNAPSHOT) {
      if (flag_snapshot_done == 1U) {
        flag_snapshot_done = 0U;

        HAL_TIM_Base_Stop(&htim1);
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);

        flag_capturing = 0U;
        samples_captured = target_samples;

        uint32_t offset = 0U;
        uint8_t tx_ok = 1U; //1 thi la truyen duoc, 0 thi la truyen loi

        while (offset < target_samples) {
          uint32_t remaining = target_samples - offset;
          uint16_t chunk_length = (remaining < STREAMING_MAX_BLOCK_SIZE) ? (uint16_t)remaining : STREAMING_MAX_BLOCK_SIZE;

          if (Send_Data_Packet(PKT_SNAPSHOT_DATA, &capture_buffer[offset], chunk_length) != 0U) {
            tx_ok = 0U;
            break;
          }

          offset += chunk_length;
        }

        if (tx_ok != 0U) {
          if (Send_Capture_Done_Packet(target_samples) != 0U) {
            (void)Send_Error_Packet(ERR_USB_TRANSMIT);
          }
        }
        else {
          (void)Send_Error_Packet(ERR_USB_TRANSMIT);
        }
      }
    }
    else if (flag_capturing == 1U) { //Dang la mode STREAMING
      uint8_t block_result = 0U;
      uint32_t remaining;
      uint16_t block_length;
      
      if (flag_stream_overrun != 0U) {
        LogicAnalyzer_StopCapture();
        flag_capturing = 0U;
        flag_half_ready = 0U;
        flag_full_ready = 0U;
        (void)Send_Error_Packet(ERR_DMA_OVERRUN);
      }
      else {
        remaining = target_samples - samples_captured;
        block_length = (remaining < stream_block_size) ? (uint16_t)remaining : stream_block_size;

        if (block_length > 0U) {
          block_result = Copy_Ready_Stream_Block(&flag_half_ready, capture_buffer, block_length);
          
          if (block_result == 1U) {
            if (Send_Data_Packet(PKT_STREAM_DATA, tx_staging_buffer, block_length) == 0U) {
              samples_captured += block_length;
            }
            else {
              block_result = 3U;
            }
          }

          if (block_result == 0U && flag_stream_overrun == 0U) {
            block_result = Copy_Ready_Stream_Block(&flag_full_ready, &capture_buffer[stream_block_size], block_length);
            
            if (block_result == 1U) {
              if (Send_Data_Packet(PKT_STREAM_DATA, tx_staging_buffer, block_length) == 0U) {
                samples_captured += block_length;
              }
              else {
                block_result = 3U;
              }
            }
          }
        }

        if (block_result == 2U || flag_stream_overrun != 0U) {
          LogicAnalyzer_StopCapture();
          flag_capturing = 0U;
          flag_half_ready = 0U;
          flag_full_ready = 0U;
          (void)Send_Error_Packet(ERR_DMA_OVERRUN);
        }
        else if (block_result == 3U) {
          LogicAnalyzer_StopCapture();
          flag_capturing = 0U;
          flag_half_ready = 0U;
          flag_full_ready = 0U;
          (void)Send_Error_Packet(ERR_USB_TRANSMIT);
        }
        else if (samples_captured >= target_samples) {
          LogicAnalyzer_StopCapture();
          flag_capturing = 0U;
          flag_half_ready = 0U;
          flag_full_ready = 0U;
          
          if (Send_Capture_Done_Packet(samples_captured) != 0) {
            (void)Send_Error_Packet(ERR_USB_TRANSMIT);
          }
        }
      }
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
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

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
