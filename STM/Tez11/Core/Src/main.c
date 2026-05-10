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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h> // Sin, Cos, Atan2 için şart!
#include <stdlib.h> // abs() fonksiyonu için

// --- ROBOT FİZİKSEL ÖLÇÜLERİ (mm) ---
#define D1_BASE_YUKSEKLIK  43.0f
#define A2_OMUZ_UZUNLUK    155.2f
#define A3_DIRSEK_UZUNLUK  128.2f
#define A4_BILEK_UZUNLUK   193.4f

// Matematik Sabitleri
#define PI 3.14159265f
#define RAD2DEG 57.2957795f
#define DEG2RAD 0.01745329f

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Durum Tanımları

typedef struct {
    // --- DONANIM ---
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_ch_ileri;
    uint32_t pwm_ch_geri;
    TIM_HandleTypeDef *enc_timer;

    // --- AYARLAR ---
    float Kp;
    float Ki;
    float Kd;
    int max_hiz;
    int min_hiz;
    int olu_bolge;

    int yon_carpani; // 1: Normal, -1: Ters (Hem encoder hem motoru tersler)

    // --- KONTROL ---
    volatile float hedef_aci;
    volatile float anlik_aci;  // <--- YENİ: ŞU ANKİ AÇI (Debug için)
    volatile int32_t hedef_pulse;
    volatile int32_t anlik_pulse;
    volatile int32_t hata;

    // --- PID HAFIZASI ---
    float toplam_hata;
    float onceki_hata;

} RobotEklemi;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

// Global Değişkenler
RobotEklemi robot[4]; // 0:Taban, 1:Omuz, 2:Dirsek, 3:Bilek

// DİKKAT: Kalibrasyon sonucuna göre bunu değiştireceğiz!
// Şimdilik eski hesap dursun. (5 Derece testi ile güncelleyeceğiz)
float ADIM_KATSAYISI = 20.53f;

// Kinematik için Ofsetler (Gerekirse açılar saparsa buradan düzeltilir)
// Örn: Robot dik dururken encoder 0 ise ama matematik 90 diyorsa, buraya 90 yazarız.
float OFFSET_TABAN = 0.0f;
float OFFSET_OMUZ = 0.0f;
float OFFSET_DIRSEK = 0.0f;
float OFFSET_BILEK = -10.0f;


// 1. Sensör Adresleri (Datasheet'ten aldık)
#define TCS34725_ADDRESS (0x29 << 1)
#define TCS34725_ID      0x12        // Kimlik Register Adresi (Yeni)
#define TCS34725_COMMAND 0x80        // Komut biti (Yeni)
#define TCS34725_ENABLE  0x00
#define TCS34725_ATIME   0x01
#define TCS34725_CDATAL  0x14
// 3. Sensör Kimliği (Bağlantı Testi İçin)
// Eğer bağlantı sağlamsa bu değişken 0x44 (68) olacak.
volatile uint8_t sensor_id = 0;
// Sensör Verileri (Live Watch)
volatile uint16_t renk_clear, renk_kirmizi, renk_yesil, renk_mavi;
volatile int hedef_kutu = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void Motor_Guncelle(RobotEklemi *eklem);
void Robotu_Senkron_Tasi(float aci0, float aci1, float aci2, float aci3);
int Hareket_Tamamlandi_Mi(void);
void IK_Git(float x, float y, float z, float phi_derece);
void Senaryoyu_Baslat(void);
void Hedefe_Yavas_Git(float aci0, float aci1, float aci2, float aci3, int bekleme_hizi_ms);
void IK_ile_Yavas_Git(float x, float y, float z, float phi_derece, int bekleme_hizi_ms);
// --- YENİ KONUM FONKSİYONLARI ---
void Git_Home(void);
void Git_Hazne(void);
void Git_Kutu_1(void);
void Git_Kutu_2(void);
void Git_Kutu_3(void);
void Git_Kutu_4(void);

// --YERİNDE KALMASI İÇİN--
void Aktif_Bekle(int sure_ms);

/* SERVO AYARLARI (MG90S - Sürekli Dönen Tip İçin) */
#define SERVO_TIMER     &htim9         // Senin Servo Timer'ın hangisiyse onu yaz (htim1, htim9 vb.)
#define SERVO_KANAL     TIM_CHANNEL_1  // Bağladığın Kanal (CH1, CH2 vb.)
//Servo
void Gripper_Ac(void);
void Gripper_Kapat(void);
void Servo_Pulse_Ver(uint32_t pulse);
// Arduino'daki değerlerin karşılığı:
#define SERVO_DUR_PULSE   1500  // (90 Derece) - Motoru Kilitler/Durdurur
#define SERVO_AC_PULSE    2000  // (113 Derece) - Sola yavaşça döner
#define SERVO_KAPAT_PULSE 1000  // (72 Derece) - Sağa yavaşça döner

//  Sensör
void TCS34725_Baslat(void);
void TCS34725_Oku(void);
int Sensor_Kutu_Var_Mi(void);
int Rengi_Analiz_Et(void);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  // --- A) DONANIM BAŞLATMA ---
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);


    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);


    // Sensörü Başlat (I2C)
    TCS34725_Baslat();


    // --- B) MOTOR AYARLARI (GÜNCEL & GÜVENLİ) ---

    // MOTOR 0: TABAN
    robot[0].pwm_timer = &htim8; robot[0].pwm_ch_ileri = TIM_CHANNEL_1; robot[0].pwm_ch_geri = TIM_CHANNEL_2;
    robot[0].enc_timer = &htim2;
    robot[0].yon_carpani = 1; // <-- BU DÜZ KALSIN
    robot[0].Kp = 1.5; robot[0].Ki = 0.08; robot[0].Kd = 90.0; // Taban yanal yük taşır, Ki kapalı
    robot[0].max_hiz = 350; robot[0].min_hiz = 0.05; robot[0].olu_bolge = 5;

    // MOTOR 1: OMUZ (Kritik - Ki SIFIRLANDI)
    robot[1].pwm_timer = &htim8; robot[1].pwm_ch_ileri = TIM_CHANNEL_3; robot[1].pwm_ch_geri = TIM_CHANNEL_4;
    robot[1].enc_timer = &htim5;
    robot[1].yon_carpani = -1; // <-- BU TERSLENMELİ
    robot[1].Kp = 3.80; robot[1].Ki = 0.20; robot[1].Kd = 78.0; // Yüksek Kd frenleme yapar
    robot[1].max_hiz = 500; robot[1].min_hiz = 270.0; robot[1].olu_bolge = 4;

    // MOTOR 2: DİRSEK
    robot[2].pwm_timer = &htim1; robot[2].pwm_ch_ileri = TIM_CHANNEL_1; robot[2].pwm_ch_geri = TIM_CHANNEL_2;
    robot[2].enc_timer = &htim3;
    robot[2].yon_carpani = 1; // <-- BU DÜZ KALSIN
    robot[2].Kp = 2.38; robot[2].Ki = 0.08; robot[2].Kd = 52.0;
    robot[2].max_hiz = 350; robot[2].min_hiz = 250; robot[2].olu_bolge = 5;

    // MOTOR 3: BİLEK
    robot[3].pwm_timer = &htim1; robot[3].pwm_ch_ileri = TIM_CHANNEL_3; robot[3].pwm_ch_geri = TIM_CHANNEL_4;
    robot[3].enc_timer = &htim4;
    robot[3].yon_carpani = -1; // <-- BU TERSLENMELİ
    robot[3].Kp = 1.2; robot[3].Ki = 0.03; robot[3].Kd = 30.0;
    robot[3].max_hiz = 300; robot[3].min_hiz = 150; robot[3].olu_bolge = 5;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    // ---------------------------------------------------------
        // 1. AŞAMA: GÜVENLİ AÇILIŞ (Mevcut Konumu Kilitle) 🔒
        // ---------------------------------------------------------
        for(int i=0; i<4; i++) {
            // Tüm encoderları (16-bit/32-bit fark etmez) standart oku
            int32_t anlik_konum;
            if(robot[i].enc_timer->Instance == TIM3 || robot[i].enc_timer->Instance == TIM4)
                 anlik_konum = (int32_t)((int16_t)__HAL_TIM_GET_COUNTER(robot[i].enc_timer));
            else anlik_konum = (int32_t)__HAL_TIM_GET_COUNTER(robot[i].enc_timer);

            // ---> BURASI ÖNEMLİ: Açılışta da konumu yön ayarına göre düzelt <---
                anlik_konum = anlik_konum * robot[i].yon_carpani;

            // Hedefi anlık konuma eşitle (Hata = 0)
            robot[i].anlik_pulse = anlik_konum;
            robot[i].hedef_pulse = anlik_konum;

            // Açıyı güncelle (Yoksa motor kendini 0 sanıp fırlar!)
            robot[i].hedef_aci = (float)anlik_konum / ADIM_KATSAYISI;
            robot[i].hedef_aci = robot[i].anlik_aci;

            robot[i].toplam_hata = 0;
            robot[i].onceki_hata = 0;
        }

        // ---------------------------------------------------------
        // 2. AŞAMA: 2 SANİYE ISINMA (Active Hold) ⏳
        // ---------------------------------------------------------
        // Elini çekmen için süre. Motorlar seni itecek, düşmeyecek.
        uint32_t baslangic = HAL_GetTick();
        while((HAL_GetTick() - baslangic) < 2000) {
            for(int i=0; i<4; i++) Motor_Guncelle(&robot[i]);
            HAL_Delay(10);
        }

        // =========================================================
            // 🚀 3. AŞAMA: GÖREV ALANI (GÜNCELLENDİ)
            // =========================================================

            // ÖRNEK: Home'a git ve 5 saniye orada KİLİTLİ kal.

            // 1. Fonksiyonu çağır (Gidip duracak)
        //    Git_Home();
            // 2. Beklerken SALMAMASI için Aktif_Bekle kullan
      //      Aktif_Bekle(5000); // 5 saniye boyunca PID çalışır, kol düşmez.
    //        Git_Kutu_1();
    //        Aktif_Bekle(5000);
    //        Git_Home();
            // ---------------------------------------------------------
            // ÖRNEK 2: Hazneye git
            // Git_Hazne();
            // Aktif_Bekle(2000);

            // =========================================================
	      Git_Home();
	      Aktif_Bekle(2000);
	      Gripper_Ac();
  while (1)
  {
	  // PID sürekli canlı kalsın (Robot kendini tutsun)
	  //      for(int i=0; i<4; i++)Motor_Guncelle(&robot[i]);
	  //      HAL_Delay(1);
	  //TCS34725_Oku();
	  //Rengi_Analiz_Et();



	      // 2. ADIM: RENK ANALİZİ YAP
	      hedef_kutu = Rengi_Analiz_Et();
	      // --- SENARYO A: KUTU YOK ---
	          if (hedef_kutu == 0) {
	               // Kutu yoksa boşuna yorulma, 1 saniye bekle tekrar bak
	               Aktif_Bekle(1000);
	               // (Döngü başa döner, robot haznede beklemeye devam eder)
	          }
	          // --- SENARYO B: KUTU VAR (RENK BULUNDU) ---
	          else {
	        	  Aktif_Bekle(2000);
                  Git_Hazne();
                  Aktif_Bekle(2000);
                  Gripper_Kapat();
                  Aktif_Bekle(2000);
                  Git_Home();
                  Aktif_Bekle(2000);
                  // 3. Rengine Göre Kutuya Git
                            if (hedef_kutu == 1) {
                                Git_Kutu_1(); // Kırmızı
                                Aktif_Bekle(2000);
                            }
                            else if (hedef_kutu == 2) {
                                Git_Kutu_2(); // Mavi
                                Aktif_Bekle(2000);
                            }
                            else if (hedef_kutu == 3) {
                                Git_Kutu_3(); // Yeşil
                                Aktif_Bekle(2000);
                            }
                            else {
                                Git_Kutu_4(); // Sarı
                                Aktif_Bekle(2000);
                            }
                  Gripper_Ac();
                  Aktif_Bekle(2000);
                  Gripper_Kapat();
                  Aktif_Bekle(1000);
                  Git_Home();
                  Aktif_Bekle(1000);
                  Gripper_Ac();
                  Aktif_Bekle(1000);
	          }





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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 167;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 19999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- 1. MOTOR GÜNCELLEME (BEYİN - İYİLEŞTİRİLMİŞ) ---
void Motor_Guncelle(RobotEklemi *eklem) {
	// 1. ADIM: ENCODER OKUMA VE YÖN DÜZELTME
	// önce 16-bit'e (int16_t) zorluyoruz.
	    // Bu sayede sayaç 0'dan geri gidip 65535 olduğunda, işlemci bunu
	    // "+65535" değil, "-1" olarak algılayacak. Sıçrama bitecek.
	    int16_t ham_16bit = (int16_t)__HAL_TIM_GET_COUNTER(eklem->enc_timer);
	    int32_t ham_konum = (int32_t)ham_16bit;

	    // GÖZÜMÜZÜ TERSLİYORUZ: Eğer çarpan -1 ise, encoder -100 okuduğunda biz +100 göreceğiz.
	    eklem->anlik_pulse = ham_konum * eklem->yon_carpani;

	    // Bunu Live Watch'ta "anlik_aci" olarak göreceksin.
	        eklem->anlik_aci = (float)eklem->anlik_pulse / ADIM_KATSAYISI;

	    // B) HATA HESABI
	    eklem->hata = eklem->hedef_pulse - eklem->anlik_pulse;

	    // C) DEADZONE
	    if (abs(eklem->hata) < eklem->olu_bolge) {
	        eklem->hata = 0;
	       // eklem->toplam_hata = 0;
	    }

	    // D) PID
	    float P = eklem->hata * eklem->Kp;

	    eklem->toplam_hata += eklem->hata;
	    if(eklem->toplam_hata > 2000) eklem->toplam_hata = 2000;
	    if(eklem->toplam_hata < -2000) eklem->toplam_hata = -2000;

	    float I = eklem->toplam_hata * eklem->Ki;
	    float D = (eklem->hata - eklem->onceki_hata) * eklem->Kd;
	    eklem->onceki_hata = eklem->hata;

	    int pwm_cikis = (int)(P + I + D);

	    // E) HIZ SINIRLAMA
	    if (pwm_cikis > eklem->max_hiz) pwm_cikis = eklem->max_hiz;
	    if (pwm_cikis < -eklem->max_hiz) pwm_cikis = -eklem->max_hiz;

	    // 2. ADIM: MOTOR ÇIKIŞINI TERSLEME (ELİMİZİ TERSLİYORUZ)
	    // Eğer çarpan -1 ise, biz ileri (+PWM) gitmek istediğimizde
	    // aslında motorun fiziksel geri kablolarına enerji vermeliyiz.
	    pwm_cikis = pwm_cikis * eklem->yon_carpani;

	    // F) MOTOR SÜRÜŞ
	    if (pwm_cikis > 0) { // İLERİ KANALINI SÜR
	        if (pwm_cikis < eklem->min_hiz && eklem->hata != 0) pwm_cikis = eklem->min_hiz;
	        __HAL_TIM_SET_COMPARE(eklem->pwm_timer, eklem->pwm_ch_ileri, pwm_cikis);
	        __HAL_TIM_SET_COMPARE(eklem->pwm_timer, eklem->pwm_ch_geri, 0);
	    }
	    else if (pwm_cikis < 0) { // GERİ KANALINI SÜR
	        pwm_cikis = abs(pwm_cikis); // Mutlak değer al, PWM negatif olmaz
	        if (pwm_cikis < eklem->min_hiz && eklem->hata != 0) pwm_cikis = eklem->min_hiz;
	        __HAL_TIM_SET_COMPARE(eklem->pwm_timer, eklem->pwm_ch_ileri, 0);
	        __HAL_TIM_SET_COMPARE(eklem->pwm_timer, eklem->pwm_ch_geri, pwm_cikis);
	    }
	    else { // DUR
	        __HAL_TIM_SET_COMPARE(eklem->pwm_timer, eklem->pwm_ch_ileri, 0);
	        __HAL_TIM_SET_COMPARE(eklem->pwm_timer, eklem->pwm_ch_geri, 0);
	    }
	}
// --- YENİ YUMUŞAK HAREKET FONKSİYONU ---
void Hedefe_Yavas_Git(float aci0, float aci1, float aci2, float aci3, int bekleme_hizi_ms) {
    float mevcut_acilar[4];
    float hedef_acilar[4] = {aci0, aci1, aci2, aci3};
    float farklar[4];
    float adimlar[4];
    int en_buyuk_adim_sayisi = 0;

    // 1. Mevcut durum ile hedef arasındaki farkı analiz et
    for(int i=0; i<4; i++) {
        mevcut_acilar[i] = robot[i].anlik_aci;
        farklar[i] = hedef_acilar[i] - mevcut_acilar[i];

        // Hassasiyet ayarı: 0.5 derece parçalara böl
        int adim_sayisi = abs((int)(farklar[i] / 2.0f));
        if(adim_sayisi > en_buyuk_adim_sayisi) en_buyuk_adim_sayisi = adim_sayisi;
    }

    // 2. Her motorun adım büyüklüğünü belirle
    for(int i=0; i<4; i++) {
        if(en_buyuk_adim_sayisi > 0)
            adimlar[i] = farklar[i] / (float)en_buyuk_adim_sayisi;
        else
            adimlar[i] = 0;
    }

    // 3. ADIM ADIM İLERLE (RAMPING)
    for(int k=0; k < en_buyuk_adim_sayisi; k++) {
        for(int i=0; i<4; i++) {
            // Hedefi minik minik kaydır
            robot[i].hedef_aci = mevcut_acilar[i] + (adimlar[i] * (k+1));
            // Pulse değerini güncelle
            robot[i].hedef_pulse = (int32_t)(robot[i].hedef_aci * ADIM_KATSAYISI);

            // PID'yi çalıştır ki motor o konuma gitsin
            Motor_Guncelle(&robot[i]);
        }
        // Buradaki bekleme süresi hareketi yumuşatır (Soft geçiş)
        Aktif_Bekle(bekleme_hizi_ms);
    }

    // 4. SON KİLİTLEME (Küsurat hatasını temizle)
    for(int i=0; i<4; i++) {
        robot[i].hedef_aci = hedef_acilar[i];
        robot[i].hedef_pulse = (int32_t)(hedef_acilar[i] * ADIM_KATSAYISI);
    }

    // Varana kadar son bir bekleme (Emin olmak için)
    Hareketi_Tamamla();
}
// --- YENİ TERS KİNEMATİK SÜRÜCÜSÜ (SOFT HAREKET) ---
void IK_ile_Yavas_Git(float x, float y, float z, float phi_derece, int bekleme_hizi_ms) {

    // 1. MATEMATİK HESAPLARI (Senin eski kodun aynısı)
    float phi_rad = phi_derece * DEG2RAD;
    float theta1_rad = atan2f(y, x);
    float theta1_deg = theta1_rad * RAD2DEG;

    float r_toplam = sqrtf(x*x + y*y);
    float r_wrist = r_toplam - (A4_BILEK_UZUNLUK * cosf(phi_rad));
    float z_wrist = (z - D1_BASE_YUKSEKLIK) - (A4_BILEK_UZUNLUK * sinf(phi_rad));

    float h_kare = (r_wrist * r_wrist) + (z_wrist * z_wrist);
    float h = sqrtf(h_kare);

    // Kosinüs Teoremi
    float pay1 = (A3_DIRSEK_UZUNLUK*A3_DIRSEK_UZUNLUK) - (A2_OMUZ_UZUNLUK*A2_OMUZ_UZUNLUK) - h_kare;
    float payda1 = -2 * A2_OMUZ_UZUNLUK * h;

    // Sıfıra bölme hatası koruması
    if(payda1 == 0) return;

    float cos_beta1 = pay1 / payda1;

    float pay2 = h_kare - (A2_OMUZ_UZUNLUK*A2_OMUZ_UZUNLUK) - (A3_DIRSEK_UZUNLUK*A3_DIRSEK_UZUNLUK);
    float payda2 = -2 * A2_OMUZ_UZUNLUK * A3_DIRSEK_UZUNLUK;
    float cos_beta2 = pay2 / payda2;

    // Kapsama Alanı Kontrolü (Robotun boyu yetiyor mu?)
    if(cos_beta1 < -1.0f || cos_beta1 > 1.0f || cos_beta2 < -1.0f || cos_beta2 > 1.0f) {
        // HATA: Hedef nokta erişilemez!
        // Güvenlik için fonksiyonu burada durduruyoruz.
        return;
    }

    float beta1 = acosf(cos_beta1);
    float beta2 = acosf(cos_beta2);
    float alpha = atan2f(z_wrist, r_wrist);

    float theta2_deg = (alpha + beta1) * RAD2DEG;
    float theta3_deg = -1 * (180.0f - (acosf(cos_beta2) * RAD2DEG));
    float theta4_deg = phi_derece - theta2_deg - theta3_deg;

    // 2. YUMUŞAK HAREKET EMRİ
    // Hesaplanan açıları ve Ofsetleri "Yavaş Git" fonksiyonuna veriyoruz.
    // bekleme_hizi_ms: Hareketin ne kadar "ağır çekim" olacağını belirler.

    Hedefe_Yavas_Git(theta1_deg + OFFSET_TABAN,
                     theta2_deg + OFFSET_OMUZ,
                     theta3_deg + OFFSET_DIRSEK,
                     theta4_deg + OFFSET_BILEK,
                     bekleme_hizi_ms);
}

// --- 2. SENKRON HAREKET ---
void Robotu_Senkron_Tasi(float aci0, float aci1, float aci2, float aci3) {
    float hedefler[4] = {aci0, aci1, aci2, aci3};
    int32_t farklar[4];
    int32_t en_uzun_yol = 0;
    int GENEL_MAX_HIZ = 700; // Güvenli Hız

    for(int i=0; i<4; i++) {
        int32_t hedef_pulse_hesap = (int32_t)(hedefler[i] * ADIM_KATSAYISI);
        farklar[i] = abs(hedef_pulse_hesap - robot[i].anlik_pulse);
        if(farklar[i] > en_uzun_yol) en_uzun_yol = farklar[i];
    }

    for(int i=0; i<4; i++) {
        if(en_uzun_yol > 0) {
            // Hareket Modu
            int yeni_hiz = (farklar[i] * GENEL_MAX_HIZ) / en_uzun_yol;
            if(yeni_hiz < robot[i].min_hiz && farklar[i] > 20) yeni_hiz = robot[i].min_hiz;
            robot[i].max_hiz = yeni_hiz;
        } else {
            // HOLD MODU: Yol bittiyse SALMA! Tam güçle tut.
            robot[i].max_hiz = 900;
        }

        robot[i].hedef_aci = hedefler[i];
        robot[i].hedef_pulse = (int32_t)(hedefler[i] * ADIM_KATSAYISI);
    }
}

// --- 3. VARIŞ KONTROLÜ ---
int Hareket_Tamamlandi_Mi() {
    int varan_sayisi = 0;
    int tolerans = 40;
    for(int i=0; i<4; i++) {
        if(abs(robot[i].hata) < tolerans) varan_sayisi++;
    }
    return (varan_sayisi == 4) ? 1 : 0;
}

// --- 4. TERS KİNEMATİK (IK) ---
void IK_Git(float x, float y, float z, float phi_derece) {
    float phi_rad = phi_derece * DEG2RAD;
    float theta1_rad = atan2f(y, x);
    float theta1_deg = theta1_rad * RAD2DEG;

    float r_toplam = sqrtf(x*x + y*y);
    float r_wrist = r_toplam - (A4_BILEK_UZUNLUK * cosf(phi_rad));
    float z_wrist = (z - D1_BASE_YUKSEKLIK) - (A4_BILEK_UZUNLUK * sinf(phi_rad));

    float h_kare = (r_wrist * r_wrist) + (z_wrist * z_wrist);
    float h = sqrtf(h_kare);

    float cos_beta1 = ( (A3_DIRSEK_UZUNLUK*A3_DIRSEK_UZUNLUK) - (A2_OMUZ_UZUNLUK*A2_OMUZ_UZUNLUK) - h_kare ) / (-2 * A2_OMUZ_UZUNLUK * h);
    float cos_beta2 = ( h_kare - (A2_OMUZ_UZUNLUK*A2_OMUZ_UZUNLUK) - (A3_DIRSEK_UZUNLUK*A3_DIRSEK_UZUNLUK) ) / (-2 * A2_OMUZ_UZUNLUK * A3_DIRSEK_UZUNLUK);

    if(cos_beta1 < -1.0f || cos_beta1 > 1.0f || cos_beta2 < -1.0f || cos_beta2 > 1.0f) return;

    float beta1 = acosf(cos_beta1);
    float beta2 = acosf(cos_beta2);
    float alpha = atan2f(z_wrist, r_wrist);

    float theta2_deg = (alpha + beta1) * RAD2DEG;

    // Dirsek Yön Düzeltmesi (Dış Açı)
    float theta3_deg = -1 * (180.0f - (acosf(cos_beta2) * RAD2DEG));

    float theta4_deg = phi_derece - theta2_deg - theta3_deg;

    // OFSETLERİ EKLEYEREK GÖNDER
    Robotu_Senkron_Tasi(theta1_deg + OFFSET_TABAN,
                        theta2_deg + OFFSET_OMUZ,
                        theta3_deg + OFFSET_DIRSEK,
                        theta4_deg + OFFSET_BILEK);
}

// --- 5. EKSİK OLAN KONUM FONKSİYONLARI (BUNLAR EKLENDİ!) ---

// YARDIMCI: Hareketi başlatır ve bitene kadar sistemi kilitler
void Hareketi_Tamamla(void) {
    // 1. İlk hesaplama tekmesi (Hata güncellensin diye)
    for(int i=0; i<4; i++) Motor_Guncelle(&robot[i]);
    HAL_Delay(10);

    // 2. Varana kadar bekle (Smart Delay)
    while(Hareket_Tamamlandi_Mi() == 0) {
        for(int i=0; i<4; i++) Motor_Guncelle(&robot[i]);
        HAL_Delay(10);
    }
    // 3. Vardıktan sonra azıcık durulması için opsiyonel bekleme
    Aktif_Bekle(500);
}

// 1. HOME (Güvenli Bekleme)
void Git_Home(void) {
    // (X=219, Y=0, Z=55) | Phi: -50
//	IK_Git(257.0f, 0.0f, 105.0f, -45.0f);
  //  Hareketi_Tamamla();
    // X=257, Y=0, Z=105, Phi=-45, Hız=10 (Orta Hız)
     //   IK_ile_Yavas_Git(270.0f, 0.0f, 100.0f, -45.0f, 35);
	IK_ile_Yavas_Git(268.0f, 0.0f, 20.0f, -58.15f, 35);
        Aktif_Bekle(500);
}

// 2. HAZNE (Alma Pozisyonu)
void Git_Hazne(void) {
    // (X=283, Y=0, Z=-65) | Phi: -60
//    IK_Git(177.0f, 0.0f, -106.0f, -114.0f);
//    Hareketi_Tamamla();
    // X=177, Y=0, Z=-106, Phi=-114, Hız=20 (Çok Yavaş ve Güçlü İniş)
        // Hazneye inerken "20" verdim ki yerçekimiyle kapışırken acele etmesin.
  //    IK_ile_Yavas_Git(335.0f, 0.0f, 5.0f, -57.5f, 35);
	IK_ile_Yavas_Git(292.0f, 0.0f, -32.0f, -75.6f, 35);

        // Vardıktan sonra tutması için:
        Aktif_Bekle(500);
}

// 3. KUTU 1 (Uzak Sağ)
void Git_Kutu_1(void) {
    // (X=193, Y=-88, Z=-16) | Phi: -75
  //  IK_Git(193.0f, -88.0f, -16.0f, -75.0f);
 //   Hareketi_Tamamla();

	// 1. ADIM: Önce Sadece Tabanı Döndür (-41.4 Dereceye)
	    // Diğer motorları kilitle (anlik_aci)
	    Hedefe_Yavas_Git(-41.45f, robot[1].anlik_aci, robot[2].anlik_aci, robot[3].anlik_aci, 35);

	    Aktif_Bekle(1000); // Sarsıntı sönümleme

	    // 2. ADIM: Kolu Uzat (XYZ Hesabı)
	    // Bu açılar şu koordinata denk geliyor:
	    // X=210, Y=-185, Z=-62 | Phi=-65.8
	    // (Y değeri taban açısı -41.4 olduğu için X ile orantılı çıkar)

	    // Biz IK fonksiyonuna "r" (uzanma mesafesi) veriyoruz gibi düşünebilirsin.
	    // Hesaplanan Uzanma (R): 280mm
	//    IK_ile_Yavas_Git(210.0f, -185.0f, -62.0f, -65.8f, 20); eski
	    Aktif_Bekle(500);
}

// 4. KUTU 2 (Uzak Sol)
void Git_Kutu_2(void) {
    // (X=193, Y=88, Z=-16) | Phi: -75
//    IK_Git(193.0f, 88.0f, -16.0f, -75.0f);
 //   Hareketi_Tamamla();
	// 1. ADIM: Tabanı Döndür (32.49 Dereceye)
	    Hedefe_Yavas_Git(45.0f, robot[1].anlik_aci, robot[2].anlik_aci, robot[3].anlik_aci, 35);

	    Aktif_Bekle(1000);

	    // 2. ADIM: Kolu Uzat
	    // Kutu 2, 3 ve 4 aynı uzaklıkta (Sadece taban dönüyor).
	    // Bu çok stabil bir hareket sağlar.
	    // Hesaplanan Uzanma (R): 335mm | Z: -42mm | Phi: -65.0

	//    IK_ile_Yavas_Git(282.0f, 180.0f, -42.0f, -65.0f, 20); eski

	    Aktif_Bekle(500);
}

// 5. KUTU 3 (Yakın Sol - Dik İniş)
void Git_Kutu_3(void) {
    // (X=93, Y=74, Z=-16) | Phi: -90
  //  IK_Git(93.0f, 74.0f, -16.0f, -90.0f);
  //  Hareketi_Tamamla();
	// 1. ADIM: Tabanı Döndür (56.06 Dereceye)
	    Hedefe_Yavas_Git(65.0f, robot[1].anlik_aci, robot[2].anlik_aci, robot[3].anlik_aci, 35);

	    Aktif_Bekle(1000);

	    // 2. ADIM: Kolu Uzat
	    // Açıları Kutu 2 ile aynı olduğu için Z ve Phi aynı.
	    // Sadece X ve Y bileşenleri taban açısına göre değişti.
//	    IK_ile_Yavas_Git(203.0f, 266.0f, -42.0f, -65.0f, 20); eski

	    Aktif_Bekle(500);
}

// 6. KUTU 4 (Yakın Sağ - Dik İniş)
void Git_Kutu_4(void) {
    // (X=93, Y=-74, Z=-16) | Phi: -90
  //  IK_Git(93.0f, -74.0f, -16.0f, -90.0f);
  //  Hareketi_Tamamla();
	// 1. ADIM: Tabanı Döndür (-58.69 Dereceye)
	    Hedefe_Yavas_Git(-62.69f, robot[1].anlik_aci, robot[2].anlik_aci, robot[3].anlik_aci, 35);

	    Aktif_Bekle(1000);

	    // 2. ADIM: Kolu Uzat
	    // Yine aynı kol duruşu, sadece taban farklı.
//	    IK_ile_Yavas_Git(191.0f, -275.0f, -42.0f, -65.0f, 20); eski

	    Aktif_Bekle(500);
}

// --- 6. ESKİ SENARYO DÖNGÜSÜ (Gerekirse) ---
void Senaryoyu_Baslat(void) {
    while(1) {
        Git_Home();
        Git_Hazne();
        Git_Home();
        Git_Kutu_1();
    }
}

// YENİ FONKSİYON: Hem bekle hem de PID ile kolu tut!
void Aktif_Bekle(int sure_ms) {
    uint32_t baslangic = HAL_GetTick();

    // Süre dolana kadar döngüde kal
    while((HAL_GetTick() - baslangic) < sure_ms) {
        // Motorları sürekli güncelle (Düşmemesi için)
        for(int i=0; i<4; i++) Motor_Guncelle(&robot[i]);

        HAL_Delay(1); // PID örnekleme hızı
    }
}


// --- DATASHEET UYUMLU BAŞLATMA VE TEST ---
void TCS34725_Baslat(void) {
    uint8_t data[2];

    // 1. ADIM: SENSÖR VAR MI? (ID OKUMA)
    // 0x12 adresindeki veriyi oku. TCS34725 ise 0x44 (68) dönmeli.
    uint8_t cmd = TCS34725_COMMAND | TCS34725_ID;
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, &cmd, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, &sensor_id, 1, 100);

    if(sensor_id != 0x44 && sensor_id != 0x4D) {
        // HATA! Tanınmayan sensör veya bağlantı kopuk.
        return;
    }

    // 2. ADIM: POWER ON (Sadece Güç Ver)
    data[0] = TCS34725_COMMAND | TCS34725_ENABLE;
    data[1] = 0x01; // PON (Power ON) biti 1
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, data, 2, 100);

    // 3. ADIM: BEKLEME (Datasheet: Min 2.4ms bekle)
    HAL_Delay(5);

    // 4. ADIM: ADC ENABLE (Ölçümü Başlat)
    data[0] = TCS34725_COMMAND | TCS34725_ENABLE;
    data[1] = 0x03; // PON (1) + AEN (1) = 0x03
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, data, 2, 100);

    // 5. ADIM: HASSASİYET AYARI (Integration Time)
    // 0xEB = ~60ms okuma süresi (Standart)
    data[0] = TCS34725_COMMAND | TCS34725_ATIME;
    data[1] = 0xEB;
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, data, 2, 100);
}

// --- YARDIMCI FONKSİYONLAR ---
void Servo_Pulse_Ver(uint32_t pulse) {
    __HAL_TIM_SET_COMPARE(SERVO_TIMER, SERVO_KANAL, pulse);
}

// --- 1. KISKACI AÇMA FONKSİYONU ---
void Gripper_Ac(void) {
    // 1. Motoru Başlat (Attach)
    HAL_TIM_PWM_Start(SERVO_TIMER, SERVO_KANAL);

    // 2. Açılma Yönüne Dön
    Servo_Pulse_Ver(SERVO_AC_PULSE);
    Aktif_Bekle(550);   // <--- DOĞRUSU BU (PID çalışmaya devam eder)

    // 3. Durdur (Fren Yap)
    Servo_Pulse_Ver(SERVO_DUR_PULSE);
    Aktif_Bekle(550);   // <--- DOĞRUSU BU (PID çalışmaya devam eder)

    // 4. Enerjiyi Kes (Detach) - Motor soğur, ses yapmaz
    HAL_TIM_PWM_Stop(SERVO_TIMER, SERVO_KANAL);
}

// --- 2. KISKACI KAPATMA FONKSİYONU ---
void Gripper_Kapat(void) {
    // 1. Motoru Başlat
    HAL_TIM_PWM_Start(SERVO_TIMER, SERVO_KANAL);

    // 2. Kapanma Yönüne Dön
    Servo_Pulse_Ver(SERVO_KAPAT_PULSE);
    Aktif_Bekle(1400);   // <--- DOĞRUSU BU (PID çalışmaya devam eder)

    // 3. Durdur
 //   Servo_Pulse_Ver(SERVO_DUR_PULSE);
    Servo_Pulse_Ver(1450);
    Aktif_Bekle(1000);   // <--- DOĞRUSU BU (PID çalışmaya devam eder)

    // 4. Enerjiyi Kes
 //   HAL_TIM_PWM_Stop(SERVO_TIMER, SERVO_KANAL);
}

void TCS34725_Oku(void) {
	uint8_t buffer[8];

	    // Hacım burası kritik: 0x80 yerine 0xA0 kullanıyoruz.
	    // 0xA0 = (CMD Biti | Auto-Increment Biti)
	    // Böylece sensör 0x14, 0x15, 0x16... diye sırayla okur.
	    uint8_t reg_addr = 0xA0 | TCS34725_CDATAL;

	    // Adresi gönder (Auto-Increment komutuyla)
	    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, &reg_addr, 1, 100);

	    // 8 Byte veriyi peş peşe çek
	    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 8, 100);

	    // Verileri Birleştir
	    renk_clear   = (buffer[1] << 8) | buffer[0];
	    renk_kirmizi = (buffer[3] << 8) | buffer[2];
	    renk_yesil   = (buffer[5] << 8) | buffer[4];
	    renk_mavi    = (buffer[7] << 8) | buffer[6];
	}

int Sensor_Kutu_Var_Mi(void) {
    TCS34725_Oku();
    // Eşik değeri 300 (Ortam ışığına göre değiştir)
    return (renk_clear > 300) ? 1 : 0;
}

// --- GELİŞMİŞ RENK AYIRT ETME ---
// Dönüş Değerleri:
// 0: Kutu Yok (Boş)
// 1: KIRMIZI (Kutu 1)
// 2: MAVİ    (Kutu 2)
// 3: YEŞİL   (Kutu 3)
// 4: SARI    (Kutu 4 - Kırmızı ve Yeşilin karışımı)

int Rengi_Analiz_Et(void) {
    // 1. Taze veri oku
    TCS34725_Oku();
    Aktif_Bekle(300);

    // 2. Kutu Var mı Kontrolü (Clear kanalına bakıyoruz)
    // EŞİK AYARI: Ortam ışığında 50-100 okuyorsa, kutu gelince 300+ okur.
    // Bu değeri Live Watch ile test edip güncelleyebilirsin.
    if (renk_clear < 30 || (renk_kirmizi <15 && renk_mavi < 15 && renk_yesil < 15)) {
        return 0; // Kutu Yok
    }

    // 3. Renk Karşılaştırma Mantığı (Dominant Renk Hangisi?)

    // SARI KONTROLÜ (En gıcık renk budur)
    // Sarı = Çok Kırmızı + Çok Yeşil + Az Mavi
    // Eğer Kırmızı ve Yeşil birbirine yakın ve Mavi'den çok büyükse -> SARI
    if (renk_kirmizi > renk_mavi + 100 && renk_yesil > renk_mavi + 100 && renk_clear < 500) {
        return 1; // Sarı (Kutu 4'e gitsin)
    }

    // KIRMIZI KONTROLÜ
    else if (renk_kirmizi > renk_yesil && renk_kirmizi > renk_mavi && renk_clear < 500) {
        return 1; // Kırmızı (Kutu 1)
    }

    // MAVİ KONTROLÜ
    else if (renk_mavi > renk_kirmizi && renk_mavi > renk_yesil && renk_clear < 500) {
        return 2; // Mavi (Kutu 2)
    }

    // YEŞİL KONTROLÜ
    else if (renk_yesil > renk_kirmizi && renk_yesil > renk_mavi && renk_clear < 500) {
        return 3; // Yeşil (Kutu 3)
    }

    // Hiçbiri değilse ama kutu varsa (Bilinmeyen Renk)
    else if(renk_clear > 500){
        return 4; // Diğerleri 4'e gitsin
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
