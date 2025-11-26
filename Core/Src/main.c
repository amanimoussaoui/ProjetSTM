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
#include "liquidcrystal_i2c.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    MODE_NORMAL = 0,
    MODE_DIAGNOSTIC
} SystemMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB

#define LED_VERTE_Pin GPIO_PIN_5
#define LED_ROUGE_Pin GPIO_PIN_1
#define LED_BLEUE_Pin GPIO_PIN_2
#define LED_JAUNE_Pin GPIO_PIN_3

#define POMPE_Pin GPIO_PIN_4
#define POMPE_GPIO_Port GPIOB

#define VENTILATEUR_Pin GPIO_PIN_8
#define VENTILATEUR_GPIO_Port GPIOB

#define LED_GPIO_Port GPIOB

// Seuils
#define TEMP_SEUIL_ALARME 30.0f
#define WATER_SEUIL_BAS 1000
#define HUMIDITE_SOL_SEUIL_BAS 2000
#define LDR_SEUIL_SOMBRE 1000  // Seuil pour détecter l'obscurité

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
volatile SystemMode system_mode = MODE_NORMAL;
volatile uint8_t diagnostic_requested = 0;
volatile uint8_t bouton_urgence_appuye = 0;
uint32_t last_button_press = 0;

// Variables pour stockage des valeurs des capteurs
float current_temperature = 0.0f;
int current_water_level = 0;
int current_soil_moisture = 0;
int current_ldr_value = 0;  // Nouvelle variable pour le LDR

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
float read_temperature_simple(void);
int read_water_level_simple(void);
int read_soil_moisture_simple(void);
int read_ldr_simple(void);  // Nouvelle fonction pour le LDR
void buzzer_beep_short(void);
void buzzer_alarme(void);
void leds_all_off(void);
void diagnostic_complet(void);
void afficher_normal_mode(void);
void gerer_actions_automatiques(float temp, int water_level, int soil_moisture);
void pompe_activer(void);
void pompe_desactiver(void);
void ventilateur_activer(void);
void ventilateur_desactiver(void);
void bouton_urgence_handler(void);
uint32_t read_adc_simple(uint32_t channel);
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
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HD44780_Init(2);

  HD44780_Clear();
  HD44780_PrintStr("Initialisation...");
  HAL_Delay(1000);

  // Lecture initiale des capteurs
  current_temperature = read_temperature_simple();
  current_water_level = read_water_level_simple();
  current_soil_moisture = read_soil_moisture_simple();
  current_ldr_value = read_ldr_simple();  // Lecture initiale du LDR

  HD44780_Clear();
  HD44780_PrintStr("Systeme Pret!");
  HAL_Delay(1000);

  buzzer_beep_short();
  leds_all_off();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_VERTE_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (system_mode == MODE_NORMAL) {
        afficher_normal_mode();
    }
    else if (system_mode == MODE_DIAGNOSTIC) {
        diagnostic_complet();
        system_mode = MODE_NORMAL;
    }

    if (diagnostic_requested) {
        diagnostic_requested = 0;
        system_mode = MODE_DIAGNOSTIC;
    }

    if (bouton_urgence_appuye) {
        bouton_urgence_appuye = 0;
        bouton_urgence_handler();
    }

    HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;  // IMPORTANT: Mode non scan comme dans l'ancien code
  hadc2.Init.ContinuousConvMode = DISABLE;  // IMPORTANT: Conversion unique
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;  // IMPORTANT: Une seule conversion
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;  // Canal par défaut
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8399;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ==============================================
// FONCTIONS SIMPLES
// ==============================================

void leds_all_off(void) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_VERTE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLEUE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_JAUNE_Pin, GPIO_PIN_RESET);
    pompe_desactiver();
    ventilateur_desactiver();
}

void buzzer_beep_short(void) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void buzzer_alarme(void) {
    for(int i=0; i<3; i++) {
        buzzer_beep_short();
        HAL_Delay(300);
    }
}

void pompe_activer(void) {
    HAL_GPIO_WritePin(POMPE_GPIO_Port, POMPE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_JAUNE_Pin, GPIO_PIN_SET);
}

void pompe_desactiver(void) {
    HAL_GPIO_WritePin(POMPE_GPIO_Port, POMPE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_JAUNE_Pin, GPIO_PIN_RESET);
}

void ventilateur_activer(void) {
    HAL_GPIO_WritePin(VENTILATEUR_GPIO_Port, VENTILATEUR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLEUE_Pin, GPIO_PIN_SET);
}

void ventilateur_desactiver(void) {
    HAL_GPIO_WritePin(VENTILATEUR_GPIO_Port, VENTILATEUR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLEUE_Pin, GPIO_PIN_RESET);
}

// ==============================================
// LECTURE ADC SIMPLIFIÉE (comme l'ancien code)
// ==============================================

uint32_t read_adc_simple(uint32_t channel) {
    uint32_t adc_value;

    // Arrêt de l'ADC
    HAL_ADC_Stop(&hadc2);
    HAL_Delay(2);

    // Configuration du canal
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        return 0xFFFFFFFF;
    }

    // Démarrage et lecture
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 100) != HAL_OK) {
        HAL_ADC_Stop(&hadc2);
        return 0xFFFFFFFF;
    }

    adc_value = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    return adc_value;
}

// ==============================================
// CAPTEURS - FONCTIONS SIMPLIFIÉES
// ==============================================

float read_temperature_simple(void) {
    uint32_t adc_value = read_adc_simple(ADC_CHANNEL_5);  // PA5
    if (adc_value == 0xFFFFFFFF || adc_value > 4095) {
        return -999.0f;
    }

    // Conversion simple comme dans l'ancien code
    float voltage = (adc_value * 3.3f) / 4095.0f;
    float temp = voltage * 100.0f;  // Pour capteur type LM35

    // Plages plus larges pour éviter les erreurs
    if (temp < -10.0f || temp > 80.0f) {
        return -999.0f;
    }

    current_temperature = temp;
    return temp;
}

int read_water_level_simple(void) {
    uint32_t adc_value = read_adc_simple(ADC_CHANNEL_2);  // PA2
    if (adc_value == 0xFFFFFFFF || adc_value > 4095) {
        return -999;
    }

    // Pas d'inversion - utilisation directe comme dans l'ancien code
    current_water_level = (int)adc_value;
    return (int)adc_value;
}

int read_soil_moisture_simple(void) {
    uint32_t adc_value = read_adc_simple(ADC_CHANNEL_4);  // PA4
    if (adc_value == 0xFFFFFFFF || adc_value > 4095) {
        return -999;
    }

    current_soil_moisture = (int)adc_value;
    return (int)adc_value;
}

// ==============================================
// NOUVELLE FONCTION POUR LDR
// ==============================================

int read_ldr_simple(void) {
    uint32_t adc_value = read_adc_simple(ADC_CHANNEL_3);  // PA3 - LDR
    if (adc_value == 0xFFFFFFFF || adc_value > 4095) {
        return -999;
    }

    current_ldr_value = (int)adc_value;
    return (int)adc_value;
}

// ==============================================
// FONCTIONNALITES PRINCIPALES
// ==============================================

void gerer_actions_automatiques(float temp, int water_level, int soil_moisture) {
    // Ventilateur si température élevée
    if (temp > TEMP_SEUIL_ALARME && temp != -999.0f) {
        ventilateur_activer();
    } else {
        ventilateur_desactiver();
    }

    // Pompe si niveau d'eau bas ET sol sec
    if (water_level < WATER_SEUIL_BAS && water_level != -999 &&
        soil_moisture > HUMIDITE_SOL_SEUIL_BAS && soil_moisture != -999) {
        pompe_activer();
    } else {
        pompe_desactiver();
    }
}

void afficher_normal_mode(void) {
    static uint32_t last_time = 0;
    static uint8_t display_state = 0;
    char msg[17];

    if (HAL_GetTick() - last_time > 3000) {
        last_time = HAL_GetTick();

        // Lecture des capteurs
        float temp = read_temperature_simple();
        int water = read_water_level_simple();
        int soil = read_soil_moisture_simple();
        int ldr = read_ldr_simple();  // Lecture du LDR

        // Actions automatiques
        gerer_actions_automatiques(temp, water, soil);

        // Affichage
        switch(display_state) {
            case 0:
                if (temp == -999.0f) {
                    sprintf(msg, "Temp: ERREUR");
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_SET);
                } else {
                    sprintf(msg, "Temp: %.1fC", temp);
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_RESET);
                }
                HD44780_Clear();
                HD44780_PrintStr(msg);
                break;

            case 1:
                if (water == -999) {
                    sprintf(msg, "Eau: ERREUR");
                } else {
                    sprintf(msg, "Eau: %d", water);
                }
                HD44780_Clear();
                HD44780_PrintStr(msg);
                HD44780_SetCursor(0,1);
                if (water < 1000) HD44780_PrintStr("Niveau: BAS");
                else if (water < 2500) HD44780_PrintStr("Niveau: MOYEN");
                else HD44780_PrintStr("Niveau: HAUT");
                break;

            case 2:
                if (soil == -999) {
                    sprintf(msg, "Sol: ERREUR");
                } else {
                    sprintf(msg, "Sol: %d", soil);
                }
                HD44780_Clear();
                HD44780_PrintStr(msg);
                HD44780_SetCursor(0,1);
                if (soil > 3000) HD44780_PrintStr("Etat: SEC");
                else if (soil > 1500) HD44780_PrintStr("Etat: HUMIDE");
                else HD44780_PrintStr("Etat: TREMPE");
                break;

            case 3:  // NOUVEL ÉTAT POUR LDR
                if (ldr == -999) {
                    sprintf(msg, "LDR: ERREUR");
                } else {
                    sprintf(msg, "LDR: %d", ldr);
                }
                HD44780_Clear();
                HD44780_PrintStr(msg);
                HD44780_SetCursor(0,1);
                if (ldr < LDR_SEUIL_SOMBRE) HD44780_PrintStr("Lumière: CLAIR");
                else HD44780_PrintStr("Lumière: SOMBRE");
                break;
        }

        display_state = (display_state + 1) % 4;  // Changé à 4 états
    }
}

// DIAGNOSTIC COMPLET AVEC LDR
void diagnostic_complet(void) {
    char msg[17];
    uint8_t errors = 0;
    uint8_t alertes = 0;

    leds_all_off();

    // ÉTAPE 1: Test des capteurs
    HD44780_Clear();
    HD44780_PrintStr("Test CAPTEURS...");
    HAL_Delay(1500);

    float temp = read_temperature_simple();
    int water = read_water_level_simple();
    int soil = read_soil_moisture_simple();
    int ldr = read_ldr_simple();  // Test du LDR

    // Affichage résultats capteurs - PREMIÈRE LIGNE
    HD44780_Clear();
    if (temp == -999.0f) {
        HD44780_PrintStr("T:ERREUR ");
        errors++;
    } else {
        sprintf(msg, "T:%.1fC ", temp);
        HD44780_PrintStr(msg);
    }

    if (water == -999) {
        HD44780_PrintStr("E:ERREUR");
        errors++;
    } else {
        sprintf(msg, "E:%d", water);
        HD44780_PrintStr(msg);
    }

    // DEUXIÈME LIGNE - SOL ET LDR
    HD44780_SetCursor(0,1);
    if (soil == -999) {
        HD44780_PrintStr("S:ERREUR ");
        errors++;
    } else {
        sprintf(msg, "S:%d ", soil);
        HD44780_PrintStr(msg);
    }

    if (ldr == -999) {
        HD44780_PrintStr("L:ERREUR");
        errors++;
    } else {
        sprintf(msg, "L:%d", ldr);
        HD44780_PrintStr(msg);
    }

    HAL_Delay(2000);

    // Affichage des états des capteurs
    HD44780_Clear();
    HD44780_PrintStr("Etats capteurs:");
    HD44780_SetCursor(0,1);

    if (temp != -999.0f && temp > TEMP_SEUIL_ALARME) {
        HD44780_PrintStr("TEMP! ");
        alertes++;
    }
    if (water != -999 && water < WATER_SEUIL_BAS) {
        HD44780_PrintStr("EAU! ");
        alertes++;
    }
    if (ldr != -999 && ldr > LDR_SEUIL_SOMBRE) {
        HD44780_PrintStr("SOMBRE");
        alertes++;
    }

    if (alertes == 0) {
        HD44780_PrintStr("NORMAL");
    }

    HAL_Delay(2000);

    // ÉTAPE 2: Test des actionneurs
    HD44780_Clear();
    HD44780_PrintStr("Test ACTIONNEURS");
    HAL_Delay(1500);

    // Test séquentiel
    HD44780_Clear();
    HD44780_PrintStr("LEDs + Buzzer...");
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_VERTE_Pin, GPIO_PIN_SET);
    buzzer_beep_short();
    HAL_Delay(500);

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_SET);
    buzzer_beep_short();
    HAL_Delay(500);

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_BLEUE_Pin, GPIO_PIN_SET);
    buzzer_beep_short();
    HAL_Delay(500);

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_JAUNE_Pin, GPIO_PIN_SET);
    buzzer_beep_short();
    HAL_Delay(500);

    HD44780_Clear();
    HD44780_PrintStr("Test POMPE...");
    pompe_activer();
    buzzer_beep_short();
    HAL_Delay(1000);
    pompe_desactiver();

    HD44780_Clear();
    HD44780_PrintStr("Test VENTIL...");
    ventilateur_activer();
    buzzer_beep_short();
    HAL_Delay(1000);
    ventilateur_desactiver();

    leds_all_off();

    // RÉSULTAT FINAL
    HD44780_Clear();
    if (errors == 0) {
        HD44780_PrintStr("DIAGNOSTIC OK");
        HD44780_SetCursor(0,1);
        HD44780_PrintStr("Systeme pret!");
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_VERTE_Pin, GPIO_PIN_SET);
    } else {
        HD44780_PrintStr("DIAG: ERREURS");
        HD44780_SetCursor(0,1);
        sprintf(msg, "Capteurs: %d", errors);
        HD44780_PrintStr(msg);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_SET);
        buzzer_alarme();
    }
    HAL_Delay(3000);

    leds_all_off();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_VERTE_Pin, GPIO_PIN_SET);
}

void bouton_urgence_handler(void) {
    int water_level = read_water_level_simple();
    float temp = read_temperature_simple();
    char msg[17];

    HD44780_Clear();
    HD44780_PrintStr("URGENCE!");
    HAL_Delay(500);

    // Action prioritaire: pompe si niveau d'eau bas
    if (water_level < WATER_SEUIL_BAS && water_level != -999) {
        HD44780_Clear();
        HD44780_PrintStr("NIVEAU BAS");
        HD44780_SetCursor(0,1);
        HD44780_PrintStr("POMPE ACTIVE");

        pompe_activer();
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_SET);
        buzzer_alarme();

        HAL_Delay(3000);

        pompe_desactiver();
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_RESET);
    }
    // Action secondaire: ventilateur si température élevée
    else if (temp > TEMP_SEUIL_ALARME && temp != -999.0f) {
        HD44780_Clear();
        HD44780_PrintStr("TEMP ELEVEE");
        HD44780_SetCursor(0,1);
        HD44780_PrintStr("VENTIL ACTIF");

        ventilateur_activer();
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_SET);
        buzzer_alarme();

        HAL_Delay(3000);

        ventilateur_desactiver();
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_ROUGE_Pin, GPIO_PIN_RESET);
    }
    else {
        HD44780_Clear();
        HD44780_PrintStr("NIVEAU NORMAL");
        HD44780_SetCursor(0,1);
        if (water_level != -999 && temp != -999.0f) {
            sprintf(msg, "Eau: %d T:%.1fC", water_level, temp);
        } else {
            sprintf(msg, "Capteurs OK");
        }
        HD44780_PrintStr(msg);

        buzzer_beep_short();
        HAL_Delay(2000);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_time > 300) {
        last_time = now;

        if (GPIO_Pin == GPIO_PIN_0) {
            diagnostic_requested = 1;
        }
        else if (GPIO_Pin == GPIO_PIN_1) {
            bouton_urgence_appuye = 1;
        }
    }
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
