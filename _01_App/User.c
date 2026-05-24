/****************************
 * Project description: 阻抗参数识别项目
 * 原理：通过ADC采集+FFT分析实现阻抗参数识别RC/RLLC等电路参数计算
 * Author: Mao -> 2026 Mao
 * Creation Date: 2026/04/24
 * Update date: 2026/05/04 (更新)
 * 1-> 增加DAC输出DDS
 * 2-> 增加学习模式校准功能
 * 3-> 增加多档位硬件校准功能
 * ****************************/

/* ***************************** Include & Define Part     	***************************** */
#include "User.h"

/* ***************************** 系统参数配置 ***************************** */
#define MAX_SAMPLE_RATE  512000			/* 最大采样率 512kHz */
#define Sweep_Fre_Buff_Length    300     /* 扫频缓存长度 */

/* 工作模式枚举 C89标准 */
#define MENU_IMPEDANCE  1    /* 菜单1 阻抗模式 + 阻抗计算 */
#define MENU_NETWORK    2    /* 菜单2 网络识别 + 谐振频率 */
#define MENU_ORIGINAL   3    /* 菜单3 原始数据显示(ADC1/ADC2/相位) */

/* 4档电阻档位 */
/* 标准阻值表格 Rref_Table[] 统一供 Resistance_Calibration/Capacitance_Calibration/Inductance_Calibration 使用 */

/* 不同档位阻值边界，预留约10%的档位切换 */
#define GEAR_BOUND_47_820_DOWN      157.0f
#define GEAR_BOUND_47_820_UP        235.0f
#define GEAR_BOUND_820_15K_DOWN     2800.0f
#define GEAR_BOUND_820_15K_UP       4500.0f
#define GEAR_BOUND_15K_270K_DOWN    50900.0f
#define GEAR_BOUND_15K_270K_UP      76400.0f

/* 不同频率对应的档位切换条件 */

/* 扫频档位切换条件，对应扫频点100Hz/500Hz/5kHz/20kHz/50kHz */
#define PHA_COMPENSATE_100HZ    0.0f
#define PHA_COMPENSATE_500HZ    0.0f
#define PHA_COMPENSATE_5KHZ     0.0f
#define PHA_COMPENSATE_20KHZ    0.0f
#define PHA_COMPENSATE_50KHZ   	0.0f

/* ==================== Phase / VOS learning storage layout ==================== */
/* All new fields live inside the existing Proportion[] / Proportion_Tmep[]
 * arrays (Flash sector 10) so no new sector is needed. Sector 10 is 128 KB,
 * 1230 floats = ~4.92 KB, far below the sector size. */
#define PROPORTION_TOTAL    1230   /* extended Proportion array size (was 1203)  */
#define PHA_OFFSET_BASE     1203   /* [1203-1207] phase offsets at Fre_Sweep[0..4] */
#define PHA_LEARNED_FLAG    1208   /* >=0.5 -> use learned, else fall back to consts */
#define VOS_ADC1_IDX        1209   /* short-circuit ADC1 reading (offset)        */
#define VOS_ADC2_IDX        1210   /* short-circuit ADC2 reading (offset)        */
#define VOS_LEARNED_FLAG    1211   /* >=0.5 -> VOS available                     */

/* 不同档位枚举，切换时使用 */
enum User_Gear {
	Gear_47 = 0,    /* 47欧标准档位 */
	Gear_820,      	/* 820欧标准档位 */
	Gear_15k,      	/* 15k欧标准档位 */
	Gear_270k,      /* 270k欧标准档位 */
	Gear_Count = 4  /* 档位总数 */
};

/* 标准阻值表格与枚举一一对应 */
const float Rref_Table[Gear_Count] = { 47.0f, 820.0f, 15000.0f, 270000.0f };

/* ==================== 阻抗校准系数表 ==================== */
/*
 * 每频率 + 每档位对应硬件校准系数
 * 行 = 档位Gear_47/Gear_820/Gear_15k/Gear_270k
 * 列 = 频率序号Fre_Sweep[0]~[4] = 100Hz/500Hz/5kHz/20kHz/50kHz
 *
 * 使用公式：Z = Rref[gear] * V1 / V2 / Calibration[gear][freq_idx]
 * 校准时只需要修改对应表格数值，刷新系统参数
 *
 * 原始值说明：
 *   原始默认5kHz 系数为 2.71267，扫频为 2.8677
 *   实际扫频测试中，通过实际校准得到
 */

/* 电阻校准系数表(默认500Hz) */
float Resistance_Calibration[Gear_Count] = {
    2.7127f,   /* Gear_47   (47欧)   - 500Hz */
    2.7127f,   /* Gear_820  (820欧)  - 500Hz */
    2.7127f,   /* Gear_15k  (15k欧)  - 500Hz */
    2.7127f,   /* Gear_270k (270k欧) - 500Hz */
};

/* 电容校准系数表 */
float Capacitance_Calibration[Gear_Count][5] = {
    /*  300Hz     500Hz      5kHz      20kHz     50kHz  */
    {  3.0695f,  2.8753f,   3.1750f,   2.8804f,   2.7692f  },  /* Gear_47   (47欧)   */
    {  2.4592f,  2.7127f,   2.8677f,   2.7189f,   2.6434f  },  /* Gear_820  (820欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.2469f  },  /* Gear_15k  (15k欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_270k (270k欧) */
};

/* 电感校准系数表 */
float Inductance_Calibration[Gear_Count][5] = {
    /*  300Hz     500Hz      5kHz      20kHz     50kHz  */
    {  2.6490f,  2.7947f,   2.9418f,   2.6999f,   2.6951f  },  /* Gear_47   (47欧)   */
    {  2.4592f,  2.7127f,   2.8677f,   2.8677f,   2.7559f  },  /* Gear_820  (820欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.7680f  },  /* Gear_15k  (15k欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_270k (270k欧) */
};

/* ==================== 双通道ADC采集偏移校准功能 ==================== */
//#define VOS_ADC1  0.025f  /* 通道1偏移电压，短路时ADC1采集值 */
//#define VOS_ADC2  0.018f  /* 通道2偏移电压，短路时ADC2采集值 */

uint8_t Gear_sign = Gear_820; /* 当前档位标志，默认820欧 */

/* ***************************** 全局变量 ***************************** */
/* 外部硬件变量 */
extern uint32_t ADCData[];      /* ADC采集数据缓冲区 */
extern DDSDataStruct dds[2];     /* DDS输出数据结构 */
extern float ADCfre;              /* ADC采样频率 */
extern float ADCfre1;
extern float ADC1VOL;            /* 通道1电压，待测元件两端电压 */
extern float ADC2VOL;            /* 通道2电压，标准电阻两端电压 */
extern float pha;                 /* 原始相位值 */

/* 菜单显示标志 */
uint8_t MenuSign = 0;            /* 当前菜单序号 */
unsigned char mode = 0 ;          /* 工作模式 */

float Sample_rate = MAX_SAMPLE_RATE; /* 当前采样率 */
float PhaArrary[10] = {0};			  /* 10个相位缓存，菜单使用 */
float ADC1VolArrary[10] = {0};		  /* 10个通道1电压数据 */
float ADC2VolArrary[10] = {0};		  /* 10个通道2电压数据 */
float ShowPha;						  /* 菜单最终显示相位 */
GRAPH_Struct GridData;                 /* 绘图数据结构 */
float Z_abs, L_abs, C_abs;             /* 阻抗模值，未校准原始值 */
uint8_t Display_flag = 0;               /* 显示刷新标志 */

/* 扫频数据缓冲区，默认5个频率点：100Hz, 500Hz, 5kHz, 20kHz, 50kHz */
float Pha_Sweep[5] = {0};               /* 5个扫频相位 */
float ADC1_Sweep[5] = {0};              /* 5个通道1电压 */
float ADC2_Sweep[5] = {0};              /* 5个通道2电压 */
float Fre_Sweep[5] = {300, 500, 5000, 20000, 50000}; /* 5个扫频频率 */
float Zabs_Sweep[5] = {0};              /* 5个阻抗模 */
float z_r[5] = {0};                      /* 实时阻抗实部 */

/* 电路网络枚举，识别使用 */
enum Network {
	Nw_Null = 0,  /* 未知网络 */
	RC_S,         /* RC串联 */
	RC_P,         /* RC并联 */
	RL_S,         /* RL串联 */
	RL_P,         /* RL并联 */
	LC_S,         /* LC串联 */
	LC_P,         /* LC并联 */
};

/* ==================== 学习模式校准数据存储地址 ==================== */
uint16_t Show_flag = 0;        /* 元件类型标志 1=电阻，2=电容，3=电感 */
float last_z;                   /* 上一次阻抗值，防止频率抖动 */

/* 学习数据存储结构，1203个float存储在flash */
float Proportion[PROPORTION_TOTAL];        /* [0..2]=R/C/L counts, [3..1202]=R/C/L proportions, [1203..1211]=phase+VOS, [1212..1229]=reserved */
uint32_t Proportion_Tmep[PROPORTION_TOTAL];/* Flash 32-bit storage temp */
float Value[1200];              /* 学习时原始数据值 */
uint32_t Value_Tmep[1200];     /* Flash存储32位转换临时值 */
uint32_t Storage_Bit_z = 0;     /* 电阻学习数据存储指针 0-400 */
uint32_t Storage_Bit_c = 0;     /* 电容学习数据存储指针 0-400 */
uint32_t Storage_Bit_l = 0;     /* 电感学习数据存储指针 0-400 */
uint32_t Storage_mode = 0;      /* 当前学习模式 1=学电阻，2=学电容，3=学电感 */
float data;                      /* 实时数据采集值 */
/* ========================================================== */

/* ***************************** 函数声明 ***************************** */
/* 通用公式计算 */
float User_FixPhase( float pha );        /* 相位修正到-180~180 */
void GPIO_Change_Init();                  /* 切换档位GPIO初始化，控制阻抗档位 */
void SetGear(uint8_t Gear);               /* 设置当前阻抗档位 */
void Get_FFTInformation(float FreSet, uint8_t MenuMode); /* 获取FFT数据信息，不同模式处理 */
void Get_FFTQuick(float FreSet);                          /* 快速FFT计算，用于扫频预处理 */
void Get_Zabs(float Get_ADC1, float Get_ADC2, float NowFre); /* 计算阻抗 */
void Get_ZabsWithType(float Get_ADC1, float Get_ADC2, float NowFre, uint8_t element_type);
uint8_t Get_Network(void);                /* 计算网络类型 */
uint8_t User_GetNetwork( float z_abs[], float pha_dif[] );    /* 精确电路识别 */
uint8_t DetectPrimaryElementType(float pha_dif[], float z_abs[]); /* 检测主要元件类型 1=R 2=C 3=L */
uint8_t change_resistance_gear();     /* 自动阻抗切换 1=档位切换 */
uint8_t GetHigherGear(uint8_t Gear);      /* 获取更高阻值档位 */
uint8_t GetLowerGear(uint8_t Gear);       /* 获取更低阻值档位 */
uint8_t GetBoundaryGear(float zAbs, uint8_t Gear); /* 计算阻抗选档位 */

/* 阻抗计算公式，统一在 Get_Zabs() + 校准系数表中计算 */

/* 电感计算公式 */
float CalculateInductanceSmallZabs(float zAbs, float pha, float fre);
float CalculateInductanceLargeZabs(float zAbs, float pha, float fre);

/* 电容计算公式，分频段 */
float CalculateCapacitanceMidRange(float fre, float zAbs, float pha);
float CalculateCapacitanceSmallRange(float fre, float zAbs, float pha);
float CalculateCapacitanceLargeRange(float fre, float zAbs, float pha);
float CalibrateCapacitance(float c); /* 电容值校准 */

/* RC串并联计算 */
float CalculateRC_SResistance(float zAbs, float pha);
float CalculateRC_SCapacitance(float zAbs, float pha, float fre);
float CalculateRC_PResistance(float zAbs, float pha);
float CalculateRC_PCapacitance(float pha, float r, float fre);

/* RL串并联计算 */
float CalculateRL_SResistance(float zAbs, float pha);
float CalculateRL_SInductance(float zAbs, float pha, float fre);
void SortArray(float arr[], uint8_t len); /* 排序数组取平均值使用 */
float CalculateRL_PResistancePart1(float zAbs, float pha);
float CalculateRL_PInductancePart1(float r, float fre, float pha);
float CalculateRL_PResistancePart2(float zAbs, float pha);
float CalculateRL_PInductancePart2(float r, float fre, float pha);

/* LC串并联计算 */
float CalculateLC_SC(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3);
float CalculateLC_SL(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3);
float CalibrateResistance(float r);
float CalibrateInductance(float l);
void Auto_ShiftGear(float zAbs);

float CalculateLC_PCPart1(float w1, float w2, float zAbs1, float zAbs3);
float CalculateLC_PLPart1(float w1, float w2, float zAbs4, float zAbs2);
float CalculateLC_PCPart2(float w1, float w2, float zAbs1, float zAbs4);
float CalculateLC_PLPart2(float w1, float w2, float zAbs4, float zAbs0);
float CalculateResonantFrequency(float l, float c);
float CalculateResonantFrequencyFromSweep(uint8_t network, float fre[], float zAbs[], float phaDif[]);

/* ==================== 学习模式相关函数 ==================== */
void Correct(uint8_t mode);       /* 数值校准，学习模式修正测量值 */
void Correct_init(void);           /* 初始化学习数据，清空存储 */
float PS2_ReadNum(float num);      /* 获取键盘输入数值，学习模式输入标准值 */
void InFLASH_Read(uint32_t addr, uint32_t *buf, uint32_t len);  /* Flash读取函数 */
void InFLASH_Write(uint32_t addr, uint32_t *buf, uint32_t len); /* Flash写入函数 */
uint8_t InFLASH_Read_Safe(uint32_t addr, uint32_t *buf, uint32_t len);/* 安全Flash读取 */
/* ============================================================== */
float User_AlignPhase(float pha, float ref);
float FindBestFrequency(float z_abs[], uint8_t gear_sweep[], float default_fre, uint8_t *best_idx);

/* Forward declarations for Menu 4 helpers (defined after MenuHaddler_3). */
static float GetPhaseCompensation(uint8_t fre_idx);
static void SaveProportionToFlash(void);
static uint8_t WaitEnterOrBack(void);
static void LearnPhase_Submode(void);
static void LearnVOS_Submode(void);
static void ViewData_Submode(void);
static void LearnRCL_Submode(void);

/* ***************************** Main Part 主函数 ***************************** */
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
void User_main(void) {
	int i = 0;
	
	/* 初始化学习数据，从Flash读取校准参数 */
	Correct_init();
	
	InFLASH_Read(ADDR_FLASH_SECTOR_10, Proportion_Tmep, PROPORTION_TOTAL);
	InFLASH_Read(ADDR_FLASH_SECTOR_11, Value_Tmep, 1200);
	
	/* 数据格式转换 */
	for(i=0; i<PROPORTION_TOTAL; i++) {
		Proportion[i] = *(float *)&Proportion_Tmep[i];
		if(i<1200) {
			Value[i] = *(float *)&Value_Tmep[i];
		}
	}
	
	/* 获取存储的学习数据指针 */
	Storage_Bit_z = Proportion_Tmep[0];
	Storage_Bit_c = Proportion_Tmep[1];
	Storage_Bit_l = Proportion_Tmep[2];
	
	Init_All();    /* 初始化所有硬件LCD/ADC/DAC/GPIO/定时器 */
	Disp_Main();   /* 显示主菜单界面 */

	while (1) {
		switch ( MenuSign ) {
			case 0:
				/* 主菜单界面，按键选择 */
				if ( Ps2KeyValue != KeyValue_Null ) 	
					Change_Menu( Ps2KeyValue );				
				break;
			case 1:
				MenuHaddler_1(); /* 菜单1 定频测量模式 */
				break;
			case 2:
				MenuHaddler_2(); /* 菜单2 扫频网络识别模式 */
				break;
			case 3:
				MenuHaddler_3(); /* 菜单3 原始数据模式 */
				break;
			case 4:
				MenuHaddler_4(); /* 菜单4 学习模式校准功能 */
				break;
			case 5:
				MenuHaddler_5(); /* 菜单5 清除学习数据 */
				break;
			default:
				break;
		}
		delay_ms(10); /* 延时10ms，降低CPU占用 */
	}
}
float CalibrateResistance(float r) {
	return r;
}

float CalibrateInductance(float l) {
	return l;
}

void Auto_ShiftGear(float zAbs) {
	uint8_t targetGear = GetBoundaryGear(zAbs, Gear_sign);
	if (targetGear != Gear_sign) {
		Gear_sign = targetGear;
		SetGear(Gear_sign);
	}
}


/* ***************************** Initialization Part 初始化函数 ***************************** */
/**
  * @brief  初始化所有硬件
  * @param  无
  * @retval 无
  */
void Init_All() {
	TFT_LCD_Init();     /* 初始化LCD */
	LCD_Clear(Black); /* 清屏为黑色 */

	/* 初始化ADC/DMA/GPIO等，在Drive_DMA_DSP_FFT.h中 */
	User_GPIO_doubleInit();
	User_ADC_double_Init();
	User_DMA_doubleInit();
	ADC_TIM3_Init(MAX_SAMPLE_RATE);

	GPIO_Change_Init(); /* 初始化档位切换GPIO PC11/PC12 */
	DDSInit();					/* 初始化DDS信号源 */
	dacInit();          /* 初始化DAC输出 */
	setDDS(1.5, 500, 50, SINWAVE); /* 设置初始信号 2Vpp 500Hz 50%占空比，正弦波 */
}
/**
  * @brief  显示主菜单界面
  * @param  无
  * @retval 无
  */
void Disp_Main() {
	uint8_t count;

	OS_String_Show( 300 - 32 * 2, 16, 32, 0, TitleStr ); /* 显示标题 */

	/* 边框指示 */
	LCD_Appoint_Clear( 0, 64, 800, 64 + 8, White );
	LCD_Appoint_Clear( 0, 480 - 32 - 8, 800, 480 - 32, White );
	LCD_Appoint_Clear( 250, 64 + 8, 250 + 2, 480 - 32 - 8, White );

	/* 显示版本信息 */
	OS_String_Show( 32, 480 - 16 - 8, 16, 0, ModelVerStr );
	OS_String_Show( 632, 480 - 16 - 8, 16, 0, UserVerStr );

	/* 显示菜单选项，5个 */
	for ( count = 1 ; count < MenuChoiceNum + 1 ; count ++ )
		OS_String_Show( 32, 32 + 64 * count, 32, 0, "○" );
	for ( count = 0 ; count < MenuChoiceNum ; count ++ ) {
		switch ( count ) {
			case 0:
				OS_String_Show( 80, 96, 32, 0, Menu1Choice1 ); /* 定频测量 */
				break;
			case 1:
				OS_String_Show( 80, 96 + 64, 32, 0, Menu1Choice2 ); /* 扫频识别 */
				break;
			case 2:
				OS_String_Show( 80, 96 + 64 * 2, 32, 0, Menu1Choice3 ); /* 原始数据 */
				break;
			case 3:
				OS_String_Show( 80, 96 + 64 * 3, 32, 0, Menu1Choice4 ); /* 学习模式 */
				break;
			case 4:
				OS_String_Show( 80, 96 + 64 * 4, 32, 0, Menu1Choice5 ); /* 清除学习数据 */
				break;
			default:
				break;
		}
	}
}

/**
  * @brief  指定位置显示数值
  * @param  location: 位置序号 1-20
  * @param  value: 要显示的数值
  * @param  str: 格式字符串
  * @retval 无
  */
void Show_Val( uint8_t location, float value, char *str ) {
	if ( location > 0 && location <= 10 )
		OS_Num_Show( 250 + 64, 96 + 32 * ( location - 1 ), 32, 1, value, str );
	else if ( location > 10 && location <= 20 )
		OS_Num_Show( 500 + 64, 96 + 32 * ( location - 11 ), 32, 1, value, str );
	else
		OS_String_Show( 250 + 64, 96, 32, 1, "ERROR" );
}

/**
  * @brief  切换菜单
  * @param  menu_sign: 目标菜单序号
  * @retval 无
  */
void Change_Menu( uint8_t menu_sign ) {
	uint8_t count;

	LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black ); /* 清空右侧区域 */

	for ( count = 1 ; count < MenuChoiceNum + 1 ; count ++ )
		OS_String_Show( 32, 32 + 64 * count, 32, 1, "○" ); /* 未选择圆圈 */

	if ( menu_sign > 0 && menu_sign <= MenuChoiceNum )
		OS_String_Show( 32, 32 + 64 * menu_sign, 32, 1, "●" ); /* 显示选择实心圆 */
	else
		menu_sign = 0;

	Ps2KeyValue = KeyValue_Null; /* 清空按键值 */
	MenuSign = menu_sign;         /* 更新当前菜单 */
}

/* ***************************** Menu Handler Part 菜单处理函数 ***************************** */
/**
  * @brief  菜单1 定频测量模式
  * 原理：使用固定频率点测量，支持学习模式校准
  * @param  无
  * @retval 无
  */
void MenuHaddler_1(void)
{
    uint8_t i;
    float Qinductance;          // 品质因数，电感专用
    uint16_t Gear_BH_flag = 0;  // 档位切换标志 0=未切换 1=已切换
    float gear_shifting_frequency;  // 自动切换频率
    uint8_t element_type;       // 元件类型标志 1=电阻，2=电容，3=电感
    uint8_t Gear_sweep[5] = {0};

    /* 初始化5个扫频点 单位Hz */
    Fre_Sweep[0] = 300;    	// 100Hz
    Fre_Sweep[1] = 500;    	// 500Hz
    Fre_Sweep[2] = 5000;   	// 5kHz
    Fre_Sweep[3] = 20000;  	// 20kHz
    Fre_Sweep[4] = 50000; 	// 50kHz
    Ps2KeyValue = KeyValue_Null;  // 清空按键

    /* 初始化档位820欧 */
    Gear_sign = Gear_820;              // 默认档位820欧
    SetGear(Gear_sign);                // 执行档位设置

    /* 循环直到返回 */
    while (Ps2KeyValue != KeyValue_Back)
    {
        /************************ 一次5个扫频预处理，每个频率1次FFT ************************/
		uint8_t si;
		uint8_t best_gear = Gear_sign;
		for (si = 0; si < 5; si++)
		{
			setDDS(2.0, Fre_Sweep[si], 50, SINWAVE);  // 设置对应频率
			OSTimeDly(5);                                 // 信号稳定
			change_resistance_gear();
			Get_FFTQuick(ddsStructData.hz);              // 快速FFT计算
			
			// 存储当前频率点数据
			Pha_Sweep[si]  = ShowPha;
			ADC1_Sweep[si] = ADC1VOL;
			ADC2_Sweep[si] = ADC2VOL;
			Gear_sweep[si] = Gear_sign;
			Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[si]);
			Zabs_Sweep[si] = Z_abs;
		}

        /************************ 检测元件类型 ************************/
        element_type = 0;                  // 初始化元件类型
        gear_shifting_frequency = Fre_Sweep[1];  // 默认切换频率500Hz

        // 判断短路/开路状态，通过ADC电压检测
        // ADC1 = 待测元件电压，ADC2 = 标准电阻两端电压
        // 短路时，待测元件电压接近0，标准电阻接近满量程
        // 开路时，标准电阻为0，待测元件接近信号源电压
        if (ADC2_Sweep[1] > 0.54f && ADC1_Sweep[1] < 0.005f)
        {
            // 短路状态，标准电阻承受全部电压，识别为短路
            element_type = 0;
        }
        // 开路状态，标准电阻为0，待测元件承受全部电压
        else if ((ADC2_Sweep[1] < 0.02f && ADC1_Sweep[1] > 1.60f))
        {
            element_type = 0;  // 开路状态，无效元件
        }
        // 检测为电感，相位为正
        else if (
                 ((Pha_Sweep[1] > 30.0f && Pha_Sweep[1] < 150.0f) ? 1 : 0) +
                 ((Pha_Sweep[2] > 60.0f && Pha_Sweep[2] < 120.0f) ? 1 : 0) +
                 ((Pha_Sweep[3] > 30.0f && Pha_Sweep[3] < 150.0f) ? 1 : 0) >= 2)
        {
            element_type = 3;  // 识别为电感
            float best_diff = 1e10f;
            uint8_t best_idx_l = 2;
            uint8_t fi;
            for (fi = 2; fi < 5; fi++) {
                if (Zabs_Sweep[fi] > 10.0f && Zabs_Sweep[fi] < 1000000.0f) {
                    uint8_t gear_idx = (Gear_sweep[fi] < Gear_Count) ? Gear_sweep[fi] : Gear_sign;
                    float rref = Rref_Table[gear_idx];
                    float diff = fabs(Zabs_Sweep[fi] - rref);
                    if (diff < best_diff) {
                        best_diff = diff;
                        best_idx_l = fi;
                    }
                }
            }
            gear_shifting_frequency = Fre_Sweep[best_idx_l];
            best_gear = Gear_sweep[best_idx_l];
        }
        // 检测为电容，相位为负
        else if (Pha_Sweep[1] < -45.0f && Pha_Sweep[1] > -135.0f)
        {
            element_type = 2;  // 识别为电容
            float best_fre = 0;
            uint8_t best_idx = 1;
            uint8_t fi;
            // 选择最佳测量频率
            gear_shifting_frequency = FindBestFrequency(Zabs_Sweep, Gear_sweep, Fre_Sweep[1], &best_idx);
            best_gear = Gear_sweep[best_idx];
        }
        // 电阻，相位接近0
        else
        {
            element_type = 1;  // 识别为电阻
        }

        /************************ 确定最佳测量频率，自动切换到最佳档位 ************************/
        setDDS(2.0, gear_shifting_frequency, 50, SINWAVE);  // 设置最佳测量频率
        OSTimeDly(5);                                          // 信号稳定

        Gear_sign = best_gear;
        SetGear(Gear_sign);
        OSTimeDly(5);

        // 执行自动切换
        Gear_BH_flag = change_resistance_gear();
        // 档位切换成功，重新扫频
        if (Gear_BH_flag) {
            last_z = Z_abs;
            continue;
        }
        last_z = Z_abs;

        Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);  // 获取准确FFT数据

        /************************ 同步刷新屏幕显示信息 ************************/
        OS_Num_Show(270, 120, 32, 1, ShowPha, "阻抗角: %0.3f°       ");
        OS_Num_Show(270, 160, 32, 1, ADC1VOL, "ADC1   : %0.3fV       ");
        OS_Num_Show(270, 200, 32, 1, ADC2VOL, "ADC2   : %0.3fV       ");

        /************************ 根据元件类型，计算并显示数值 ************************/
        // 短路/开路显示
        if (ADC2VOL > 0.54f && ADC1VOL < 0.005f)
        {
			OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模: %0.3fΩ          ");
            OS_Num_Show(270, 240, 32, 1, 1, "短路状态                       ");
            OS_Num_Show(270, 280, 32, 1, 1, "                               ");
            OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
            OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
        }
        else if (ADC2VOL < 0.02f && ADC1VOL > 1.60f)
        {
   			OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模: %0.3fΩ          ");
			OS_Num_Show(270, 240, 32, 1, 1, "开路状态                       ");
            OS_Num_Show(270, 280, 32, 1, 1, "                               ");
            OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
            OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
        }
        // 有效元件测量
        else
        {
            // 电感测量
            if (element_type == 3)
            {
                Show_flag = 3;
                float measure_freq = gear_shifting_frequency;

                // 准确测量
                Gear_sign = best_gear;
                SetGear(Gear_sign);
                OSTimeDly(5);
                setDDS(2.0, measure_freq, 50, SINWAVE);
                OSTimeDly(5);
                Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);
                Get_Zabs(ADC1VOL, ADC2VOL, measure_freq);

                // 电感计算公式
                if (Z_abs < 10.0f)
                {
                    L_abs = CalculateInductanceSmallZabs(Z_abs, ShowPha, measure_freq);
                }
                else
                {
                    L_abs = CalculateInductanceLargeZabs(Z_abs, ShowPha, measure_freq);
                }

                // 显示电感
				OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模: %0.3fΩ          ");
				OS_Num_Show(270, 240, 32, 1, 1, "                            ");
                OS_Num_Show(270, 320, 32, 1, measure_freq, "使用频率:%.0fHz          ");

                Correct(1);  // 学习校准

                // 单位自动转换 H/mH/μH
                if (L_abs >= 1000000.0f)
                {
                    OS_Num_Show(270, 280, 32, 1, L_abs / 1000000.0f, "电感:%0.3fH            ");
                }
                else if (L_abs >= 1000.0f)
                {
                    OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "电感:%0.3fmH           ");
                }
                else
                {
                    OS_Num_Show(270, 280, 32, 1, L_abs,          "电感:%0.3fμH           ");
                }
            }
            // 电容测量
            else if (element_type == 2)
            {
                Show_flag = 2;
                uint8_t best_idx = 1;
                float best_fre = FindBestFrequency(Zabs_Sweep, Gear_sweep, Fre_Sweep[1], &best_idx);

                // 准确测量
                setDDS(2.0, best_fre, 50, SINWAVE);
                OSTimeDly(5);
                Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);
                Get_Zabs(ADC1VOL, ADC2VOL, best_fre);

                // 分频段电容计算
                if (best_idx == 0)
                {
                    C_abs = CalculateCapacitanceSmallRange(best_fre, Z_abs, ShowPha);
                }
                else if (best_idx == 1)
                {
                    C_abs = CalculateCapacitanceMidRange(best_fre, Z_abs, ShowPha);
                }
                else
                {
                    C_abs = CalculateCapacitanceLargeRange(best_fre, Z_abs, ShowPha);
                }

                // 显示电容
				OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模: %0.3fΩ          ");

                Correct(2);                    // 学习校准
                C_abs = CalibrateCapacitance(C_abs);  // 电容校准

                // 单位自动转换 F/nF/pF
				if(C_abs < 0.03f)
                {
                    OS_Num_Show(270, 240, 32, 1, 1, "短路状态                    ");
					OS_Num_Show(270, 320, 32, 1, 1, "                            ");
                }
                else if (C_abs >= 1000.0f)
                {
					OS_Num_Show(270, 240, 32, 1, 1, "                            ");
                    OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "电容 : %0.3fμF            ");
					OS_Num_Show(270, 320, 32, 1, best_fre, "使用频率:%.0fHz          ");
                }
                else if (C_abs >= 1.0f)
                {
					OS_Num_Show(270, 240, 32, 1, 1, "                            ");
                    OS_Num_Show(270, 280, 32, 1, C_abs,          "电容 : %0.3fnF            ");
					OS_Num_Show(270, 320, 32, 1, best_fre, "使用频率:%.0fHz          ");
                }
                else
                {
					OS_Num_Show(270, 240, 32, 1, 1, "                            ");
                    OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "电容 : %0.3fpF            ");
					OS_Num_Show(270, 320, 32, 1, best_fre, "使用频率:%.0fHz          ");
                }
            }
            // 电阻测量
            else
            {
                Show_flag = 1;
                // 使用500Hz测量电阻
                setDDS(2.0, Fre_Sweep[1], 50, SINWAVE);
                OSTimeDly(5);
                Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);
                Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[1]);

                Correct(1);  // 学习校准

                // 显示电阻
				OS_Num_Show(270, 80,  32, 1, Z_abs, "阻抗模: %0.3fΩ          ");
				OS_Num_Show(270, 240, 32, 1, 1, "                            ");
                OS_Num_Show(270, 280, 32, 1, 1, "                               ");
                OS_Num_Show(270, 320, 32, 1, 500, "使用频率:%.0fHz          ");
                OS_Num_Show(270, 360, 32, 1, 1,"                                        ");
            }
        }

        OSTimeDly(100);  // 100ms刷新屏幕
    }

    Change_Menu(0);  // 返回主菜单
}

/**
  * @brief  菜单2 扫频识别模式
  * 原理：5个频率点扫频，识别串并联类型，计算谐振频率
  * @param  无
  * @retval 无
  */
void MenuHaddler_2()
{
	uint8_t i;
	uint8_t network;
	uint8_t primary_type;
	uint8_t positive_cnt;
	uint8_t negative_cnt;
	uint8_t idx_rc = 2;
	uint8_t idx_rl = 1;
	uint8_t idx_lc_low = 1;
	uint8_t idx_lc_high = 3;
	uint8_t Gear_Sweep[5] = {0};  /* Bug#5: per-frequency gear bookkeeping */
	float w1, w2;
	float ResonantFre;
	float Pha5k;
	float ADC1_5k;
	float ADC2_5k;
	float Zabs5k;

	Fre_Sweep[0] = 300;
	Fre_Sweep[1] = 500;
	Fre_Sweep[2] = 5000;
	Fre_Sweep[3] = 20000;
	Fre_Sweep[4] = 50000;

	setDDS(2.0, 5000, 50, SINWAVE);
	Gear_sign = Gear_820;
	SetGear(Gear_sign);
	Ps2KeyValue = KeyValue_Null;

	while ( Ps2KeyValue != KeyValue_Back )
	{
		setDDS(2.0, 5000, 50, SINWAVE);
		OSTimeDly(10);

		Display_flag = 0;
		Show_flag = 0;  /* avoid stale element-type bias in User_GetNetwork */
		change_resistance_gear();

		Get_FFTInformation(5000, MENU_NETWORK);
		Pha5k = ShowPha;
		ADC1_5k = ADC1VOL;
		ADC2_5k = ADC2VOL;
		Get_ZabsWithType(ADC1VOL, ADC2VOL, 5000, 0);
		Zabs5k = Z_abs;

		/* 阻抗模 与 相位 固定行, 避免被其他行清除 */
		OS_Num_Show(270, 160, 32, 1, Zabs5k, "阻抗模: %10.2fΩ        ");
		OS_Num_Show(270, 200, 32, 1, Pha5k,  "相  位: %+10.2f°        ");
		/* ADC1/ADC2 移至 y=320, 24px 字体, 不与主信息冲突 */
		OS_Num_Show(270, 320, 24, 1, ADC1_5k, "ADC1: %.3fV    ");
		OS_Num_Show(450, 320, 24, 1, ADC2_5k, "ADC2: %.3fV          ");

		if (Zabs5k < 0.10f) {
			OS_String_Show(270, 80,  32, 1, "网络类型: 短路状态           ");
			OS_String_Show(270, 120, 32, 1, "谐振频率: --                  ");
			OS_Num_Show(270, 240, 32, 1, 1, "                                ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                ");
		} else if (ADC2_5k < 0.15f && ADC1_5k > 1.75f) {
			OS_String_Show(270, 80,  32, 1, "网络类型: 开路状态           ");
			OS_String_Show(270, 120, 32, 1, "谐振频率: --                  ");
			OS_Num_Show(270, 240, 32, 1, 1, "                                ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                ");
		} else if (Zabs5k > 300000.0f && fabs(Pha5k) < 12.0f) {
			OS_String_Show(270, 80,  32, 1, "网络类型: 未知网络           ");
			OS_String_Show(270, 120, 32, 1, "谐振频率: N/A                 ");
			OS_String_Show(270, 240, 32, 1, "阻抗值超出测量范围           ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                ");
		} else {
			for (i = 0; i < 5; i++) {
				setDDS(2.0, Fre_Sweep[i], 50, SINWAVE);
				OSTimeDly(15);
				/* Bug#5: pick best gear for THIS frequency before averaging FFT */
				change_resistance_gear();
				Gear_Sweep[i] = Gear_sign;
				Get_FFTInformation(Fre_Sweep[i], MENU_NETWORK);

				Pha_Sweep[i] = ShowPha;
				ADC1_Sweep[i] = ADC1VOL;
				ADC2_Sweep[i] = ADC2VOL;
				Get_ZabsWithType(ADC1VOL, ADC2VOL, Fre_Sweep[i], 0);
				Zabs_Sweep[i] = Z_abs;

				/* Apply learned phase offset if Menu 4 -> Phase cal has been run,
				 * otherwise GetPhaseCompensation() falls back to PHA_COMPENSATE_*. */
				Pha_Sweep[i] += GetPhaseCompensation(i);
				Pha_Sweep[i] = User_FixPhase(Pha_Sweep[i]);
			}

			primary_type = DetectPrimaryElementType(Pha_Sweep, Zabs_Sweep);
			Show_flag = primary_type;
			positive_cnt = 0;
			negative_cnt = 0;

			for (i = 0; i < 5; i++) {
				if (Pha_Sweep[i] > 8.0f) {
					positive_cnt++;
				} else if (Pha_Sweep[i] < -8.0f) {
					negative_cnt++;
				}
			}

			if (primary_type == 2 || primary_type == 3) {
				for (i = 0; i < 5; i++) {
					Gear_sign = Gear_Sweep[i];  /* Bug#5: restore gear matching ADC samples */
					Get_ZabsWithType(ADC1_Sweep[i], ADC2_Sweep[i], Fre_Sweep[i], primary_type);
					Zabs_Sweep[i] = Z_abs;
				}
			}

			network = Get_Network();

			if (network == Nw_Null) {
				if (primary_type == 2 && negative_cnt >= 2) {
					if (Pha_Sweep[4] < Pha_Sweep[0]) {
						network = RC_P;
					} else {
						network = RC_S;
					}
				} else if (primary_type == 3 && positive_cnt >= 2) {
					if (Pha_Sweep[4] > Pha_Sweep[0]) {
						network = RL_S;
					} else {
						network = RL_P;
					}
				}
			}

			if (network == RC_S || network == RC_P) {
				for (i = 0; i < 5; i++) {
					Gear_sign = Gear_Sweep[i];  /* Bug#5 */
					Get_ZabsWithType(ADC1_Sweep[i], ADC2_Sweep[i], Fre_Sweep[i], 2);
					Zabs_Sweep[i] = Z_abs;
				}
				idx_rc = 2;
				for (i = 1; i < 5; i++) {
					if (fabs(Pha_Sweep[i]) > fabs(Pha_Sweep[idx_rc])) {
						idx_rc = i;
					}
				}
			} else if (network == RL_S || network == RL_P) {
				for (i = 0; i < 5; i++) {
					Gear_sign = Gear_Sweep[i];  /* Bug#5 */
					Get_ZabsWithType(ADC1_Sweep[i], ADC2_Sweep[i], Fre_Sweep[i], 3);
					Zabs_Sweep[i] = Z_abs;
				}
				idx_rl = 1;
				for (i = 1; i < 5; i++) {
					if (fabs(Pha_Sweep[i]) > fabs(Pha_Sweep[idx_rl])) {
						idx_rl = i;
					}
				}
			}

			idx_lc_low = 1;
			idx_lc_high = 3;
			Z_abs = 0.0f;
			L_abs = 0.0f;
			C_abs = 0.0f;
			ResonantFre = 0.0f;

			if ( network == RC_S ) {
				Z_abs = CalculateRC_SResistance(Zabs_Sweep[idx_rc], Pha_Sweep[idx_rc]);
				C_abs = CalculateRC_SCapacitance(Zabs_Sweep[idx_rc], Pha_Sweep[idx_rc], Fre_Sweep[idx_rc]);
				if (C_abs < 0.0f) C_abs = -C_abs;
				C_abs = CalibrateCapacitance(C_abs);
				Correct(2);
				if (Z_abs < 0.0f) Z_abs = 0.0f;
			} else if ( network == RC_P ) {
				Z_abs = CalculateRC_PResistance(Zabs_Sweep[idx_rc], Pha_Sweep[idx_rc]);
				C_abs = CalculateRC_PCapacitance(Pha_Sweep[idx_rc], Z_abs, Fre_Sweep[idx_rc]);
				if (C_abs < 0.0f) C_abs = -C_abs;
				C_abs = CalibrateCapacitance(C_abs);
				Correct(2);
				if (Z_abs < 0.0f) Z_abs = 0.0f;
			} else if ( network == RL_S ) {
				Z_abs = CalculateRL_SResistance(Zabs_Sweep[idx_rl], Pha_Sweep[idx_rl]);
				L_abs = CalculateRL_SInductance(Zabs_Sweep[idx_rl], Pha_Sweep[idx_rl], Fre_Sweep[idx_rl]);
				Z_abs = CalibrateResistance(Z_abs);
				Correct(1);
				if (Z_abs < 0.0f) Z_abs = 0.0f;
				if (L_abs < 0.0f) L_abs = -L_abs;
			} else if ( network == RL_P ) {
				Z_abs = CalculateRL_PResistancePart1(Zabs_Sweep[idx_rl], Pha_Sweep[idx_rl]);
				Z_abs = CalibrateResistance(Z_abs);
				L_abs = CalculateRL_PInductancePart1(Z_abs, Fre_Sweep[idx_rl], Pha_Sweep[idx_rl]);
				if (Z_abs > 20000.0f && idx_rl < 4) {
					uint8_t idx_rl_high = (idx_rl < 3) ? 3 : idx_rl;
					Z_abs = CalculateRL_PResistancePart2(Zabs_Sweep[idx_rl_high], Pha_Sweep[idx_rl_high]);
					Z_abs = CalibrateResistance(Z_abs);
					L_abs = CalculateRL_PInductancePart2(Z_abs, Fre_Sweep[idx_rl_high], Pha_Sweep[idx_rl_high]);
				}
				/* Bug#11-C: cross-check L against the low-frequency asymptote.
				 * For RL_P at the lowest sweep freq (300 Hz), omega*L is far smaller
				 * than ANY plausible R, so |Z| ~= omega*L and L ~= |Z[0]| / omega[0].
				 * That estimate is robust because it does NOT divide by tan(phase) --
				 * the phase-based formula L = R/(omega*tan(phase)) blows up when
				 * idx_rl lands on a low-|phase| point or when ADC SNR depresses the
				 * measured phase.  If the phase-based L disagrees with the
				 * asymptotic estimate by more than 2x, prefer the asymptote.
				 * (Current CalculateRL_PInductancePart1 returns L in uH; match it.) */
				{
					float omega_lo = 2.0f * PI * Fre_Sweep[0];
					float L_lo_uH  = (omega_lo > 1.0f)
					                 ? (Zabs_Sweep[0] / omega_lo) * 1.0e6f
					                 : 0.0f;
					if (L_lo_uH > 0.0f) {
						if (L_abs <= 0.0f) {
							L_abs = L_lo_uH;
						} else {
							float ratio = (L_abs > L_lo_uH)
							              ? (L_abs / L_lo_uH)
							              : (L_lo_uH / L_abs);
							if (ratio > 2.0f) {
								L_abs = L_lo_uH;
							}
						}
					}
				}
				Correct(1);
				if (Z_abs < 0.0f) Z_abs = 0.0f;
				if (L_abs < 0.0f) L_abs = -L_abs;
			} else if ( network == LC_S ) {
				w1 = 2 * PI * Fre_Sweep[idx_lc_low];
				w2 = 2 * PI * Fre_Sweep[idx_lc_high];
				C_abs = fabs(CalculateLC_SC(w1, w2, Pha_Sweep[idx_lc_low], Zabs_Sweep[idx_lc_low], Pha_Sweep[idx_lc_high], Zabs_Sweep[idx_lc_high]));
				L_abs = fabs(CalculateLC_SL(w1, w2, Pha_Sweep[idx_lc_low], Zabs_Sweep[idx_lc_low], Pha_Sweep[idx_lc_high], Zabs_Sweep[idx_lc_high]));
				C_abs = CalibrateCapacitance(C_abs);
				L_abs = CalibrateInductance(L_abs);
			} else if ( network == LC_P ) {
				/* LC parallel: Z = -j / (omega*C - 1/(omega*L)).
				 *   Im(Z) is real and signed:
				 *     sub-resonance: Im(Z) > 0 (inductive)
				 *     super-resonance: Im(Z) < 0 (capacitive)
				 *
				 * Previous |Z|-based formula bias source: a parasitic series
				 * resistance R_p (PCB / source / gear resistor finite output
				 * impedance) makes |Z_meas|^2 = R_p^2 + Im(Z_LC)^2 > Im(Z_LC)^2,
				 * inflating L and deflating C.  The measured PHASE also
				 * shrinks from +-90deg toward the value atan(Im/R_p).
				 *
				 * Fix: use Im(Z_meas) = |Z_meas| * sin(pha_meas) directly.
				 * Im(Z_meas) == Im(Z_LC) exactly (no R_p contamination).
				 * Then two-point algebra recovers both L and C cleanly even
				 * when both points are on the SAME side of resonance:
				 *
				 *   1/Im(Z) = 1/(omega*L) - omega*C
				 *
				 *   Let b_a = Im(Z) at omega_a, b_b = Im(Z) at omega_b:
				 *     C = (omega_a/b_a - omega_b/b_b) / (omega_b^2 - omega_a^2)
				 *     1/L = omega_a/b_a + omega_a^2 * C
				 *
				 * Verified algebraically with the user's L=92.31uH C=530nF
				 * case (sub/super straddle) and the L=92.31uH C=89.4nF
				 * case (both sub-resonance). */
				{
					uint8_t z_peak_idx = 0;
					uint8_t i_lo, i_hi;
					uint8_t kk;
					float w_a, w_b;
					float b_a, b_b;
					float denom, c_val, l_inv;

					for (kk = 1; kk < 5; kk++) {
						if (Zabs_Sweep[kk] > Zabs_Sweep[z_peak_idx]) z_peak_idx = kk;
					}
					if (z_peak_idx >= 1 && z_peak_idx <= 3) {
						i_lo = (uint8_t)(z_peak_idx - 1);
						i_hi = (uint8_t)(z_peak_idx + 1);
					} else if (z_peak_idx == 0) {
						i_lo = 0;
						i_hi = 2;
					} else {
						i_lo = 2;
						i_hi = 4;
					}

					w_a = 2.0f * PI * Fre_Sweep[i_lo];
					w_b = 2.0f * PI * Fre_Sweep[i_hi];
					b_a = Zabs_Sweep[i_lo] * sin(Pha_Sweep[i_lo] * PI / 180.0f);
					b_b = Zabs_Sweep[i_hi] * sin(Pha_Sweep[i_hi] * PI / 180.0f);

					denom = (w_b * w_b) - (w_a * w_a);
					if (fabs(b_a) > 1.0e-4f && fabs(b_b) > 1.0e-4f && fabs(denom) > 1.0f) {
						c_val = (w_a / b_a - w_b / b_b) / denom;
						l_inv = w_a / b_a + (w_a * w_a) * c_val;
						if (l_inv > 1.0e-4f) {
							L_abs = 1.0f / l_inv;
						} else {
							L_abs = 0.0f;
						}
						C_abs = c_val;
					} else {
						/* Insufficient signal -- fall back to single-point |Z| form. */
						if (w_a > 1.0f) {
							L_abs = Zabs_Sweep[i_lo] / w_a;
						} else {
							L_abs = 0.0f;
						}
						if (w_b > 1.0f && Zabs_Sweep[i_hi] > 1.0e-3f) {
							C_abs = 1.0f / (w_b * Zabs_Sweep[i_hi]);
						} else {
							C_abs = 0.0f;
						}
					}

					if (L_abs < 0.0f) L_abs = -L_abs;
					if (C_abs < 0.0f) C_abs = -C_abs;
					/* LC_S / LC_P branches display SI units (H and F) and
					 * CalculateResonantFrequency expects the same -- do NOT
					 * pre-scale to uH/nF here. */
					C_abs = CalibrateCapacitance(C_abs);
					L_abs = CalibrateInductance(L_abs);
				}
				/* keep w1,w2 written for any downstream readers / legacy code */
				w1 = 2.0f * PI * Fre_Sweep[idx_lc_low];
				w2 = 2.0f * PI * Fre_Sweep[idx_lc_high];
				(void)w1; (void)w2;
			}

			/* === 显示网络类型 (y=80) 整行写入, 不再用 (420,80) 拼接 === */
			if ( network == RC_S ) {
				OS_String_Show(270, 80, 32, 1, "网络类型: RC串联            ");
			} else if ( network == RC_P ) {
				OS_String_Show(270, 80, 32, 1, "网络类型: RC并联            ");
			} else if ( network == RL_S ) {
				OS_String_Show(270, 80, 32, 1, "网络类型: RL串联            ");
			} else if ( network == RL_P ) {
				OS_String_Show(270, 80, 32, 1, "网络类型: RL并联            ");
			} else if ( network == LC_S ) {
				OS_String_Show(270, 80, 32, 1, "网络类型: LC串联            ");
			} else if ( network == LC_P ) {
				OS_String_Show(270, 80, 32, 1, "网络类型: LC并联            ");
			} else {
				OS_String_Show(270, 80, 32, 1, "网络类型: 未知               ");
			}

			/* === 显示谐振频率 (y=120) === */
			if (network == LC_S || network == LC_P) {
				Auto_ShiftGear(Zabs5k);
				ResonantFre = 0.0f;
				if (L_abs > 0.0f && C_abs > 0.0f) {
					ResonantFre = CalculateResonantFrequency(L_abs, C_abs);
				}
				if (ResonantFre > 0.0f && ResonantFre < 100000.0f) {
					OS_Num_Show(270, 120, 32, 1, ResonantFre, "谐振频率: %0.1fHz       ");
				} else {
					OS_String_Show(270, 120, 32, 1, "谐振频率: N/A                ");
				}
			} else {
				OS_String_Show(270, 120, 32, 1, "谐振频率: N/A                ");
			}

			/* === 显示元件参数 (y=240, y=280) === */
			if ( network == RC_S || network == RC_P ) {
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ                     ");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "C: %.3fμF                  ");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "C: %.3fpF                  ");
				} else {
					OS_Num_Show(270, 280, 32, 1, C_abs, "C: %.3fnF                  ");
				}
				OS_Num_Show(270, 360, 32, 1, 1, "                                ");
			} else if ( network == RL_S || network == RL_P ) {
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ                     ");
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH                  ");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fμH                  ");
				}
				OS_Num_Show(270, 360, 32, 1, 1, "                                ");
			} else if ( network == LC_S || network == LC_P ) {
				if (C_abs > 1.0e-7f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e6f, "C: %.3fμF                  ");
				} else if (C_abs < 1.0e-10f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e12f, "C: %.3fpF                  ");
				} else {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e9f, "C: %.3fnF                  ");
				}
				if (L_abs > 1.0e-4f) {
					OS_Num_Show(270, 280, 32, 1, L_abs * 1.0e3f, "L: %.3fmH                  ");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs * 1.0e6f, "L: %.3fμH                  ");
				}
				OS_Num_Show(270, 360, 32, 1, 1, "                                ");
			} else {
				OS_Num_Show(270, 240, 32, 1, 1, "                                ");
				OS_Num_Show(270, 280, 32, 1, 1, "                                ");
				OS_Num_Show(270, 360, 32, 1, 1, "                                ");
			}
		}
		OSTimeDly(100);
	}
	Change_Menu( 0 );
}

/**
  * @brief  菜单3 - DDS信号源（自定义模式）
  * 频率范围: 10Hz ~ 200kHz, 峰峰值: 2.0V +/- 0.1V, 步进: 10/100/1k/10kHz
  * 使用STM32内置DAC（通过TIM6+DMA实现DDS）
  *
  * 按键映射:
  *   1/2/3/4 : 设置步进为 10Hz / 100Hz / 1kHz / 10kHz
  *   + / -   : 频率增加/减少当前步进值
  *   * / /   : 峰峰值增加/减少0.05V
  *   5       : 重置峰峰值为2.000V（标准中心值）
  *   6/7/8/9 : 快速预设 10Hz / 1kHz / 10kHz / 100kHz
  *   0       : 手动输入频率（PS2数字键盘输入，单位Hz）
  *   .       : 切换数据视图/操作指南
  *   Enter   : 重新应用当前设置
  *   Back    : 退出菜单
  *
  * 显示布局（清除区域: x 252..800, y 72..440）:
  *   数据视图: 标题 + 9行数据 + 底部提示
  *   指南视图: 标题 + 9行指南（24px字体） + 底部提示
  *   两种视图都确保所有文本严格在清除区域内（无溢出）
  */
void MenuHaddler_3(void) {
    /* 信号源状态变量 */
    float    target_fre = 1000.0f;       /* 目标输出频率（Hz）   */
    float    vpp_set    = 2.000f;        /* 目标峰峰值（V）        */
    float    fre_step   = 1000.0f;       /* 当前步进值（Hz）       */
    uint8_t  step_idx   = 2;             /* 0=10Hz 1=100Hz 2=1kHz 3=10kHz  */
    const float step_table[4] = { 10.0f, 100.0f, 1000.0f, 10000.0f };

    /* DDS内部状态回读 */
    uint32_t sample_len   = 128;         /* 每个周期的DDS采样点数 */
    uint32_t timer_period = 1;           /* TIM6自动重装值+1      */
    float    actual_fre   = 1000.0f;     /* 硬件实际输出频率      */
    float    fre_err_pct  = 0.0f;        /* 频率误差（百分比）    */
    float    fre_err_ppm  = 0.0f;        /* 频率误差（百万分比）  */
    float    dac_peak_code= 0.0f;        /* 12位DAC峰值码值       */
    float    update_rate  = 0.0f;        /* DAC更新速率 = 84e6/period  */

    uint8_t  need_apply   = 1;           /* 1 -> 重新计算DDS参数  */
    float    input_fre    = 0.0f;        /* 手动输入频率缓冲区    */

    /* 视图切换状态
     * show_guide = 0 -> 数据视图（默认）; 1 -> 操作指南
     * 按'.'键切换。need_redraw强制全屏刷新 */
    uint8_t  show_guide  = 0;
    uint8_t  need_redraw = 1;

    /* 清除右侧显示区域 */
    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);

    /* 初始输出 */
    setDDS(vpp_set, (uint32_t)target_fre, 50, SINWAVE);
    Ps2KeyValue = KeyValue_Null;

    while (Ps2KeyValue != KeyValue_Back) {
        /* -------------------- 按键处理 -------------------- */
        switch (Ps2KeyValue) {
            /* 步进选择 */
            case KeyValue_1: step_idx = 0; fre_step = step_table[0]; break;
            case KeyValue_2: step_idx = 1; fre_step = step_table[1]; break;
            case KeyValue_3: step_idx = 2; fre_step = step_table[2]; break;
            case KeyValue_4: step_idx = 3; fre_step = step_table[3]; break;

            /* 频率 +/- 当前步进 */
            case KeyValue_Add:
                target_fre += fre_step;
                if (target_fre > 200000.0f) target_fre = 200000.0f;
                need_apply = 1;
                break;
            case KeyValue_Minus:
                target_fre -= fre_step;
                if (target_fre < 10.0f) target_fre = 10.0f;
                need_apply = 1;
                break;

            /* 峰峰值微调 +/- 0.05V */
            case KeyValue_Ride:
                vpp_set += 0.05f;
                if (vpp_set > 3.000f) vpp_set = 3.000f;
                need_apply = 1;
                break;
            case KeyValue_Div:
                vpp_set -= 0.05f;
                if (vpp_set < 0.100f) vpp_set = 0.100f;
                need_apply = 1;
                break;

            /* 一键重置/预设 */
            case KeyValue_5: vpp_set    = 2.000f;   need_apply = 1; break;
            case KeyValue_6: target_fre = 10.0f;    need_apply = 1; break;
            case KeyValue_7: target_fre = 1000.0f;  need_apply = 1; break;
            case KeyValue_8: target_fre = 10000.0f; need_apply = 1; break;
            case KeyValue_9: target_fre = 100000.0f;need_apply = 1; break;

            /* 手动数字输入频率（Hz） */
            case KeyValue_0:
                input_fre = PS2_ReadNum(0);
                if (input_fre >= 10.0f && input_fre <= 200000.0f) {
                    target_fre = input_fre;
                    need_apply = 1;
                }
                /* PS2_ReadNum会在面板上绘制自己的UI；需要重绘 */
                need_redraw = 1;
                break;

            /* 切换数据视图 <-> 操作指南 */
            case KeyValue_Point:
                show_guide  = (uint8_t)(!show_guide);
                need_redraw = 1;
                break;

            case KeyValue_Enter:
                need_apply = 1;            /* 强制重新应用当前参数 */
                break;

            default: break;
        }
        Ps2KeyValue = KeyValue_Null;

        /* -------------------- 应用DDS设置 + 重新计算回读值 -------------------- */
        if (need_apply) {
            setDDS(vpp_set, (uint32_t)target_fre, 50, SINWAVE);

            /* 采样长度选择，与Drive_DAC.c中的setDDS()函数保持一致 */
            if      (target_fre < 500.0f)   sample_len = 512;
            else if (target_fre < 5000.0f)  sample_len = 128;
            else if (target_fre < 50000.0f) sample_len = 64;
            else                            sample_len = 32;

            /* 定时器周期: 84MHz / (频率 * 采样长度) */
            timer_period = (uint32_t)(84000000.0f / (target_fre * (float)sample_len));
            if (timer_period == 0) timer_period = 1;

            update_rate  = 84000000.0f / (float)timer_period;
            actual_fre   = update_rate / (float)sample_len;
            fre_err_pct  = (actual_fre - target_fre) / target_fre * 100.0f;
            fre_err_ppm  = (actual_fre - target_fre) / target_fre * 1.0e6f;
            dac_peak_code= (vpp_set / 3.3f) * 4095.0f;

            need_apply = 0;
        }

        /* -------------------- 全屏重绘（视图切换/手动输入后） -------------------- */
        if (need_redraw) {
            LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);

            if (show_guide) {
                /* 操作指南视图: 24px字体，所有文本在清除区域内（y <= 432） */
                OS_String_Show(270, 80,  32, 1, "操作指南");
                OS_String_Show(270, 128, 24, 1, "1/2/3/4 : 步进 10/100/1k/10kHz");
                OS_String_Show(270, 160, 24, 1, "+ / -   : 频率 +/- 当前步进");
                OS_String_Show(270, 192, 24, 1, "* / /   : 峰峰值 +/- 0.05V");
                OS_String_Show(270, 224, 24, 1, "5       : 重置峰峰值为2.0V");
                OS_String_Show(270, 256, 24, 1, "6/7/8/9 : 频率 10Hz/1k/10k/100k");
                OS_String_Show(270, 288, 24, 1, "0       : 手动输入频率");
                OS_String_Show(270, 320, 24, 1, ".       : 切换数据/指南视图");
                OS_String_Show(270, 352, 24, 1, "Enter   : 重新应用设置");
                OS_String_Show(270, 384, 24, 1, "Back    : 退出菜单");
                OS_String_Show(270, 416, 16, 1, "按 '.' 键查看数据");
            } else {
                /* 数据视图: 标题 + 底部提示，数据行在下方绘制 */
                OS_String_Show(270, 80,  32, 1, "DDS信号源");
                OS_String_Show(270, 416, 16, 1, "按 '.' 键查看帮助    Back退出");
            }

            need_redraw = 0;
        }

        /* -------------------- 刷新数据标签（仅在数据视图中） -------------------- */
        if (!show_guide) {
            OS_Num_Show(270, 120, 32, 1, target_fre,  "目标频率: %10.2f Hz   ");
            OS_Num_Show(270, 152, 32, 1, actual_fre,  "实际频率: %10.4f Hz   ");

            /* 误差绝对值 >= 0.0001% 显示百分比；否则显示ppm */
            if (fabs(fre_err_pct) >= 0.0001f) {
                OS_Num_Show(270, 184, 32, 1, fre_err_pct,
                            (fabs(fre_err_pct) > 0.1f)
                              ? "频率误差: %+.4f %% (偏大)  "
                              : "频率误差: %+.4f %%        ");
            } else {
                OS_Num_Show(270, 184, 32, 1, fre_err_ppm,
                            "频率误差: %+.2f ppm       ");
            }

            OS_Num_Show(270, 216, 32, 1, vpp_set,
                        ((vpp_set >= 1.9f && vpp_set <= 2.1f)
                            ? "峰峰值  : %.3f V (标准)     "
                            : "峰峰值  : %.3f V           "));

            OS_Num_Show(270, 248, 32, 1, fre_step,            "当前步进: %.0f Hz             ");
            OS_Num_Show(270, 280, 32, 1, (float)sample_len,   "采样点数: %.0f                ");
            OS_Num_Show(270, 312, 32, 1, (float)timer_period, "定时器值: %.0f                ");
            OS_Num_Show(270, 344, 32, 1, update_rate,         "更新速率: %.0f Hz             ");
            OS_Num_Show(270, 376, 32, 1, dac_peak_code,       "DAC码值 : %.0f / 4095         ");
        }

        OSTimeDly(50);
    }

    /* 退出: 清除面板，返回主菜单 */
    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    Change_Menu(0);
}

/* ============================================================================
 *                    菜单4 学习模式校准功能
 * ============================================================================ */

/* 根据扫频点序号 0..4对应100/500/5k/20k/50kHz，获取学习校准补偿值
 * 未学习则使用原始 PHA_COMPENSATE_* 常量作为 fallback */
static float GetPhaseCompensation(uint8_t fre_idx)
{
    float v;
    if (fre_idx >= 5) return 0.0f;
    if (Proportion[PHA_LEARNED_FLAG] >= 0.5f) {
        v = Proportion[PHA_OFFSET_BASE + fre_idx];
        /* 数值合法性校验，防止异常状态导致Flash数据异常 */
        if (v == v && v > -180.0f && v < 180.0f) return v;
    }
    switch (fre_idx) {
        case 0:  return PHA_COMPENSATE_100HZ;
        case 1:  return PHA_COMPENSATE_500HZ;
        case 2:  return PHA_COMPENSATE_5KHZ;
        case 3:  return PHA_COMPENSATE_20KHZ;
        case 4:  return PHA_COMPENSATE_50KHZ;
        default: return 0.0f;
    }
}

/* 将 Proportion[] 同步到 Proportion_Tmep[]，然后写入 Flash 扇区 10 */
static void SaveProportionToFlash(void)
{
    uint32_t k;
    for (k = 0; k < PROPORTION_TOTAL; k++) {
        Proportion_Tmep[k] = *(uint32_t *)&Proportion[k];
    }
    InFLASH_Write(ADDR_FLASH_SECTOR_10, Proportion_Tmep, PROPORTION_TOTAL);
    OSTimeDly(200);
}

/* 等待按键确认 1=确认继续 0=取消返回 */
static uint8_t WaitEnterOrBack(void)
{
    Ps2KeyValue = KeyValue_Null;
    while (1) {
        if (Ps2KeyValue == KeyValue_Enter) { Ps2KeyValue = KeyValue_Null; return 1; }
        if (Ps2KeyValue == KeyValue_Back)  { Ps2KeyValue = KeyValue_Null; return 0; }
        OSTimeDly(20);
    }
}

/* --- 相位校准 --------------------------------------------------------
 * 接入纯电阻（建议100R~1k），全频率点相位应为0
 * 扫描 5 个频率点，记录当前相位，存储 -相位 作为偏移补偿
 * 菜单 2 扫频时会自动叠加该补偿 */
static void LearnPhase_Submode(void)
{
    const float pha_fre_list[5] = {300.0f, 500.0f, 5000.0f, 20000.0f, 50000.0f};
    float   new_offsets[5] = {0};
    float   meas_phi[5]    = {0};
    uint8_t step;
    uint8_t rep;
    uint8_t k;
    uint8_t bad_count = 0;
    float   sum_p;

    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    OS_String_Show(270, 80,  32, 1, "相位校准 (全自动)");
    OS_String_Show(270, 140, 24, 1, "请插入 500R-1k 纯电阻");
    OS_String_Show(270, 180, 24, 1, "档位=820R, 5频×3测, 约3秒");
    OS_String_Show(270, 220, 24, 1, "理论相位=0°, 取负为补偿");
    OS_String_Show(270, 260, 24, 1, "测后确认再保存到 Flash");
    OS_String_Show(270, 410, 16, 1, "ENTER 开始     BACK 取消");

    if (!WaitEnterOrBack()) {
        LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
        return;
    }

    Gear_sign = Gear_820;
    SetGear(Gear_sign);

    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    OS_String_Show(270, 80, 32, 1, "相位校准 - 测量中");
    OS_String_Show(270, 410, 16, 1, "请稍候 ...                    ");

    /* Auto sweep, 3 averaged samples per frequency (each = 10-sample median) */
    for (step = 0; step < 5; step++) {
        setDDS(2.0, (uint32_t)pha_fre_list[step], 50, SINWAVE);
        OSTimeDly(120);
        sum_p = 0.0f;
        for (rep = 0; rep < 3; rep++) {
            Get_FFTInformation(pha_fre_list[step], MENU_NETWORK);
            sum_p += ShowPha;
            OSTimeDly(20);
        }
        meas_phi[step]    = sum_p / 3.0f;
        new_offsets[step] = -meas_phi[step];
        if (fabs(meas_phi[step]) > 30.0f) bad_count++;

        OS_Num_Show(270, 120 + step * 32, 24, 1, pha_fre_list[step], "f=%7.0fHz ");
        OS_Num_Show(470, 120 + step * 32, 24, 1, meas_phi[step],     "phi=%+6.2f°");
        OS_Num_Show(640, 120 + step * 32, 24, 1, new_offsets[step],  "->%+6.2f°");
    }

    if (bad_count > 0) {
        OS_String_Show(270, 300, 24, 1, "警告: |phi|>30° 可能不是纯电阻   ");
    } else {
        OS_String_Show(270, 300, 24, 1, "质量 OK, 所有相位偏差<30°       ");
    }

    OS_String_Show(270, 336, 32, 1, "ENTER 保存 / BACK 丢弃        ");
    OS_String_Show(270, 410, 16, 1, "保存后以后上电自动加载                   ");

    if (WaitEnterOrBack()) {
        for (k = 0; k < 5; k++) {
            Proportion[PHA_OFFSET_BASE + k] = new_offsets[k];
        }
        Proportion[PHA_LEARNED_FLAG] = 1.0f;
        OS_String_Show(270, 336, 32, 1, "保存到 Flash ...             ");
        SaveProportionToFlash();
        OS_String_Show(270, 336, 32, 1, "相位校准完成                 ");
    } else {
        OS_String_Show(270, 336, 32, 1, "丢弃, 未保存                 ");
    }

    OS_String_Show(270, 410, 16, 1, "按返回键退出                  ");
    Ps2KeyValue = KeyValue_Null;
    while (Ps2KeyValue != KeyValue_Back) OSTimeDly(20);
    Ps2KeyValue = KeyValue_Null;
    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
}

/* --- ADC 偏移校准 -----------------------------------------------------
 * 探针短路，使V_DUT = 0
 * 2Vpp 5kHz 输出下，ADC1 理论应为 0V，实际采集为偏移，ADC2 采集标准电平
 * 校准后消除硬件零漂误差 */
static void LearnVOS_Submode(void)
{
    float vos1 = 0.0f, vos2 = 0.0f;

    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    OS_String_Show(270, 80,  32, 1, "ADC 偏移校准");
    OS_String_Show(270, 140, 24, 1, "请将探针短路");
    OS_String_Show(270, 180, 24, 1, "V_DUT = 0, ADC1 理论应为 0V");
    OS_String_Show(270, 220, 24, 1, "实际采集值即为系统偏移");
    OS_String_Show(270, 260, 24, 1, "档位 820R, 信号 5kHz @ 2Vpp");
    OS_String_Show(270, 410, 16, 1, "ENTER 开始采集     BACK 取消返回");

    if (!WaitEnterOrBack()) {
        LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
        return;
    }

    Gear_sign = Gear_820;
    SetGear(Gear_sign);
    setDDS(2.0, 5000, 50, SINWAVE);
    OSTimeDly(120);
    Get_FFTInformation(5000, MENU_NETWORK);
    vos1 = ADC1VOL;
    vos2 = ADC2VOL;

    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    OS_String_Show(270, 80, 32, 1, "ADC 偏移 - 采集完成");
    OS_Num_Show(270, 144, 32, 1, vos1, "VOS_ADC1 = %.4f V");
    OS_Num_Show(270, 184, 32, 1, vos2, "VOS_ADC2 = %.4f V");
    OS_String_Show(270, 240, 24, 1, "VOS_ADC1 : 待测端偏移");
    OS_String_Show(270, 272, 24, 1, "VOS_ADC2 : 标准端参考");
    OS_String_Show(270, 304, 24, 1, "测量自动扣除标准电平");
    OS_String_Show(270, 410, 16, 1, "ENTER 保存到Flash    BACK 放弃");

    if (WaitEnterOrBack()) {
        Proportion[VOS_ADC1_IDX]     = vos1;
        Proportion[VOS_ADC2_IDX]     = vos2;
        Proportion[VOS_LEARNED_FLAG] = 1.0f;
        OS_String_Show(270, 360, 32, 1, "保存到 Flash ...   ");
        SaveProportionToFlash();
        OS_String_Show(270, 360, 32, 1, "偏移校准完成。       ");
    } else {
        OS_String_Show(270, 360, 32, 1, "放弃，未保存。   ");
    }

    OS_String_Show(270, 410, 16, 1, "按返回键退出                ");
    Ps2KeyValue = KeyValue_Null;
    while (Ps2KeyValue != KeyValue_Back) OSTimeDly(20);
    Ps2KeyValue = KeyValue_Null;
    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
}

/* --- 校准数据查看分页界面------------------------------------------------- */
static void ViewData_Submode(void)
{
    uint8_t page = 0;
    const uint8_t total_pages = 4;
    uint8_t i;
    uint8_t need_redraw = 1;

    Ps2KeyValue = KeyValue_Null;
    while (1) {
        if (Ps2KeyValue == KeyValue_Back) break;
        if (Ps2KeyValue == KeyValue_Add) {
            page = (uint8_t)((page + 1) % total_pages);
            need_redraw = 1;
            Ps2KeyValue = KeyValue_Null;
        } else if (Ps2KeyValue == KeyValue_Minus) {
            page = (uint8_t)((page == 0) ? (total_pages - 1) : (page - 1));
            need_redraw = 1;
            Ps2KeyValue = KeyValue_Null;
        } else if (Ps2KeyValue != KeyValue_Null) {
            Ps2KeyValue = KeyValue_Null;
        }

        if (need_redraw) {
            LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
            OS_String_Show(270, 420, 16, 1, "+/- 翻页        返回键 退出");

            if (page == 0) {
                OS_String_Show(270, 80, 32, 1, "查看 1/4 电阻校准");
                OS_Num_Show(270, 124, 24, 1, (float)Storage_Bit_z, "数量: %.0f / 400");
                OS_String_Show(270, 156, 24, 1, "#");
                OS_String_Show(330, 156, 24, 1, "阻抗 Ω");
                OS_String_Show(540, 156, 24, 1, "系数");
                for (i = 0; i < 8 && i < Storage_Bit_z; i++) {
                    OS_Num_Show(270, 192 + i * 28, 24, 1, (float)i,          "#%2.0f");
                    OS_Num_Show(330, 192 + i * 28, 24, 1, Value[i],          "%10.3f");
                    OS_Num_Show(540, 192 + i * 28, 24, 1, Proportion[i + 3], "k=%.4f");
                }
            } else if (page == 1) {
                OS_String_Show(270, 80, 32, 1, "查看 2/4 电容校准");
                OS_Num_Show(270, 124, 24, 1, (float)Storage_Bit_c, "数量: %.0f / 400");
                OS_String_Show(270, 156, 24, 1, "#");
                OS_String_Show(330, 156, 24, 1, "容值");
                OS_String_Show(540, 156, 24, 1, "系数");
                for (i = 0; i < 8 && i < Storage_Bit_c; i++) {
                    OS_Num_Show(270, 192 + i * 28, 24, 1, (float)i,                "#%2.0f");
                    OS_Num_Show(330, 192 + i * 28, 24, 1, Value[i + 400],          "%10.4f");
                    OS_Num_Show(540, 192 + i * 28, 24, 1, Proportion[i + 3 + 400], "k=%.4f");
                }
            } else if (page == 2) {
                OS_String_Show(270, 80, 32, 1, "查看 3/4 电感校准");
                OS_Num_Show(270, 124, 24, 1, (float)Storage_Bit_l, "数量: %.0f / 400");
                OS_String_Show(270, 156, 24, 1, "#");
                OS_String_Show(330, 156, 24, 1, "感值 uH");
                OS_String_Show(540, 156, 24, 1, "系数");
                for (i = 0; i < 8 && i < Storage_Bit_l; i++) {
                    OS_Num_Show(270, 192 + i * 28, 24, 1, (float)i,                "#%2.0f");
                    OS_Num_Show(330, 192 + i * 28, 24, 1, Value[i + 800],          "%10.4f");
                    OS_Num_Show(540, 192 + i * 28, 24, 1, Proportion[i + 3 + 800], "k=%.4f");
                }
            } else {
                OS_String_Show(270, 80, 32, 1, "查看 4/4 相位 / 偏移");
                if (Proportion[PHA_LEARNED_FLAG] >= 0.5f) {
                    OS_String_Show(270, 128, 24, 1, "相位补偿: 已学习");
                    OS_Num_Show(290, 160, 24, 1, Proportion[PHA_OFFSET_BASE + 0], "100Hz   = %+7.3f °");
                    OS_Num_Show(290, 190, 24, 1, Proportion[PHA_OFFSET_BASE + 1], "500Hz   = %+7.3f °");
                    OS_Num_Show(290, 220, 24, 1, Proportion[PHA_OFFSET_BASE + 2], "5  kHz  = %+7.3f °");
                    OS_Num_Show(290, 250, 24, 1, Proportion[PHA_OFFSET_BASE + 3], "20 kHz  = %+7.3f °");
                    OS_Num_Show(290, 280, 24, 1, Proportion[PHA_OFFSET_BASE + 4], "50 kHz  = %+7.3f °");
                } else {
                    OS_String_Show(270, 128, 24, 1, "相位补偿: 未学习");
                    OS_String_Show(270, 160, 24, 1, "使用默认补偿值");
                }
                if (Proportion[VOS_LEARNED_FLAG] >= 0.5f) {
                    OS_String_Show(270, 324, 24, 1, "ADC 偏移: 已学习");
                    OS_Num_Show(290, 356, 24, 1, Proportion[VOS_ADC1_IDX], "VOS_ADC1 = %.4f V");
                    OS_Num_Show(290, 386, 24, 1, Proportion[VOS_ADC2_IDX], "VOS_ADC2 = %.4f V");
                } else {
                    OS_String_Show(270, 324, 24, 1, "ADC 偏移: 未学习");
                }
            }
            need_redraw = 0;
        }
        OSTimeDly(50);
    }
    Ps2KeyValue = KeyValue_Null;
    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
}

/* --- 阻/容/感 数值校准系统（通用接口）-------------------------------- */
static void LearnRCL_Submode(void)
{
    uint8_t enter_input_flag = 0;
    float   input_data       = 0.0f;
    float   input_internal   = 0.0f;   /* 折算回内部单位(欧姆 / nF / μH)后的输入值 */
    char   *element_name     = "未知元件";
    char   *unit_str         = "";     /* 当前显示与输入所用的单位字符串 */
    float   unit_factor      = 1.0f;   /* 输入(显示单位) * unit_factor = 内部单位 */
    float   display_value    = 0.0f;   /* 测量值在显示单位下的数值 */
    char    fmt_val[64];               /* 测量值显示格式串 */
    char    fmt_in[64];                /* 输入提示格式串(含单位) */

    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);

    switch (Storage_mode) {
        case 1: element_name = "电阻 (R)";  break;
        case 2: element_name = "电容 (C)"; break;
        case 3: element_name = "电感(L)";  break;
        default: element_name = "(无)";       break;
    }

    if (Storage_mode < 1 || Storage_mode > 3) {
        Show_Val(1, 0, "阻/容/感 学习失败");
        Show_Val(2, 0, "未识别对应元件");
        Show_Val(3, 0, "请执行菜单1测量");
        Show_Val(4, 0, "再进入校准。");
        Show_Val(7, 0, "按返回退出");
        Ps2KeyValue = KeyValue_Null;
        while (Ps2KeyValue != KeyValue_Back) OSTimeDly(20);
        Ps2KeyValue = KeyValue_Null;
        return;
    }

    /* ===== 与菜单1保持一致的自动量程单位选择 =====
     * 内部存储:  Z_abs 单位 欧姆;  C_abs 单位 nF;  L_abs 单位 μH
     * 显示单位:
     *   R -> 欧姆;
     *   C -> C_abs>=1000 显示 μF;  >=1 显示 nF;  否则 pF;
     *   L -> L_abs>=1e6  显示 H;   >=1e3 显示 mH; 否则 μH.
     * unit_factor 把"显示单位下的数值"换算回"内部单位下的数值",
     * 使 比例 = input_internal / measured 在数学上仍然成立. */
    switch (Storage_mode) {
        case 1:
            unit_str = "欧姆";
            unit_factor = 1.0f;
            display_value = Z_abs;
            break;
        case 2:
            if (C_abs >= 1000.0f) {
                unit_str = "μF";
                unit_factor = 1000.0f;          /* 1 μF = 1000 nF */
                display_value = C_abs / 1000.0f;
            } else if (C_abs >= 1.0f) {
                unit_str = "nF";
                unit_factor = 1.0f;
                display_value = C_abs;
            } else {
                unit_str = "pF";
                unit_factor = 0.001f;           /* 1 pF = 0.001 nF */
                display_value = C_abs * 1000.0f;
            }
            break;
        case 3:
            if (L_abs >= 1000000.0f) {
                unit_str = "H";
                unit_factor = 1000000.0f;       /* 1 H  = 1e6 μH */
                display_value = L_abs / 1000000.0f;
            } else if (L_abs >= 1000.0f) {
                unit_str = "mH";
                unit_factor = 1000.0f;          /* 1 mH = 1000 μH */
                display_value = L_abs / 1000.0f;
            } else {
                unit_str = "μH";
                unit_factor = 1.0f;
                display_value = L_abs;
            }
            break;
        default:
            unit_str = "";
            unit_factor = 1.0f;
            display_value = 0.0f;
            break;
    }

    /* 按当前单位拼装显示/提示格式串 */
    sprintf(fmt_val, "测量值 : %%.3f %s          ", unit_str);
    sprintf(fmt_in,  "输入标准值(单位:%s)按确认 ",  unit_str);

    Show_Val(1, 0, "学习校准");
    Show_Val(2, 0, element_name);
    /* 进入即先显示当前测量值, 便于对照输入 */
    Show_Val(4, display_value, fmt_val);
    Show_Val(7, 0, fmt_in);

    Ps2KeyValue = KeyValue_Null;
    while (1) {
        if (Ps2KeyValue == KeyValue_Back) {
            Ps2KeyValue = KeyValue_Null;
            break;
        }
        if (Ps2KeyValue >= KeyValue_0 && Ps2KeyValue <= KeyValue_9 && enter_input_flag == 0) {
            enter_input_flag = 1;
        }
        if (enter_input_flag == 1) {
            Show_Val(7, 0, fmt_in);             /* 输入前重申单位提示 */
            input_data = PS2_ReadNum(0);
            enter_input_flag = 0;
        }
        if (input_data > 0.0f) {
            input_internal = input_data * unit_factor;   /* 折算到内部单位 */
            switch (Storage_mode) {
                case 1:
                    Show_Val(3, (float)Storage_Bit_z, "校准点 #%0.0f");
                    Show_Val(4, display_value,        fmt_val);
                    if (Storage_Bit_z < 400) {
                        Proportion[Storage_Bit_z + 3]      = input_internal / Z_abs;
                        Proportion_Tmep[Storage_Bit_z + 3] = *(uint32_t *)&Proportion[Storage_Bit_z + 3];
                        Value[Storage_Bit_z]      = Z_abs;
                        Value_Tmep[Storage_Bit_z] = *(uint32_t *)&Z_abs;
                        Storage_Bit_z++;
                        Proportion_Tmep[0] = Storage_Bit_z;
                        Show_Val(7, 0, "保存中...");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10, Proportion_Tmep, PROPORTION_TOTAL);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11, Value_Tmep, 1200);
                        OSTimeDly(200);
                        Show_Val(7, 0, "保存成功，继续下一点");
                    } else {
                        Show_Val(7, 0, "电阻点数已满400点）");
                    }
                    break;
                case 2:
                    Show_Val(3, (float)Storage_Bit_c, "校准点 #%0.0f");
                    Show_Val(4, display_value,        fmt_val);
                    if (Storage_Bit_c < 400) {
                        Proportion[Storage_Bit_c + 3 + 400]      = input_internal / C_abs;
                        Proportion_Tmep[Storage_Bit_c + 3 + 400] = *(uint32_t *)&Proportion[Storage_Bit_c + 3 + 400];
                        Value[Storage_Bit_c + 400]      = C_abs;
                        Value_Tmep[Storage_Bit_c + 400] = *(uint32_t *)&C_abs;
                        Storage_Bit_c++;
                        Proportion_Tmep[1] = Storage_Bit_c;
                        Show_Val(7, 0, "保存中...");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10, Proportion_Tmep, PROPORTION_TOTAL);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11, Value_Tmep, 1200);
                        OSTimeDly(200);
                        Show_Val(7, 0, "保存成功，继续下一点");
                    } else {
                        Show_Val(7, 0, "电容点数已满400点）");
                    }
                    break;
                case 3:
                    Show_Val(3, (float)Storage_Bit_l, "校准点 #%0.0f");
                    Show_Val(4, display_value,        fmt_val);
                    if (Storage_Bit_l < 400) {
                        Proportion[Storage_Bit_l + 3 + 800]      = input_internal / L_abs;
                        Proportion_Tmep[Storage_Bit_l + 3 + 800] = *(uint32_t *)&Proportion[Storage_Bit_l + 3 + 800];
                        Value[Storage_Bit_l + 800]      = L_abs;
                        Value_Tmep[Storage_Bit_l + 800] = *(uint32_t *)&L_abs;
                        Storage_Bit_l++;
                        Proportion_Tmep[2] = Storage_Bit_l;
                        Show_Val(7, 0, "保存中...");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10, Proportion_Tmep, PROPORTION_TOTAL);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11, Value_Tmep, 1200);
                        OSTimeDly(200);
                        Show_Val(7, 0, "保存成功，继续下一点");
                    } else {
                        Show_Val(7, 0, "电感点数已满400点）");
                    }
                    break;
                default:
                    Show_Val(2, 0, "校准失败");
                    break;
            }
            input_data = 0.0f;
        }
        OSTimeDly(10);
    }
}

/* ==================== 菜单4 学习模式校准功能 ==================== */
/**
  * @brief  学习模式，校准硬件误差
  * @param  无
  * @retval 无
  */
/* 菜单4 - 学习模式校准功能
 *   1 -> 电阻/电容/电感 数值校准系统（使用上一次测量数据）
 *   2 -> 相位校准，消除硬件相移误差
 *   3 -> ADC偏移校准，探针短路
 *   4 -> 查看学习数据，分页显示
 *   返回 -> 退出菜单 */
void MenuHaddler_4()
{
    int i;
    uint8_t submode;
    char *m;

    /* 进入时从Flash读取一次校准参数 */
    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    OS_String_Show(270, 80, 32, 1, "加载校准数据...");
    InFLASH_Read(ADDR_FLASH_SECTOR_10, Proportion_Tmep, PROPORTION_TOTAL);
    InFLASH_Read(ADDR_FLASH_SECTOR_11, Value_Tmep,      1200);
    for (i = 0; i < PROPORTION_TOTAL; i++) {
        Proportion[i] = *(float *)&Proportion_Tmep[i];
        if (i < 1200) Value[i] = *(float *)&Value_Tmep[i];
    }
    Storage_Bit_z = Proportion_Tmep[0];
    Storage_Bit_c = Proportion_Tmep[1];
    Storage_Bit_l = Proportion_Tmep[2];
    /* 指针越界保护，防止Flash默认值0xFFFFFFFF */
    if (Storage_Bit_z > 400) Storage_Bit_z = 0;
    if (Storage_Bit_c > 400) Storage_Bit_c = 0;
    if (Storage_Bit_l > 400) Storage_Bit_l = 0;
    /* 异常值修正（校准值为0时确保未校准时显示正常） */
    for (i = PHA_OFFSET_BASE; i <= VOS_LEARNED_FLAG; i++) {
        if (!(Proportion[i] == Proportion[i])) Proportion[i] = 0.0f;
    }
    Storage_mode = Show_flag;

    /* 子菜单循环 */
    while (1) {
        LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
        OS_String_Show(270, 80, 32, 1, "学习模式校准");

        /* 菜单选项 (24px) */
        OS_String_Show(270, 128, 24, 1, "1 - 阻/容/感 数值校准");
        OS_String_Show(270, 160, 24, 1, "2 - 相位校准");
        OS_String_Show(270, 192, 24, 1, "3 - ADC 偏移校准");
        OS_String_Show(270, 224, 24, 1, "4 - 查看学习数据");
        OS_String_Show(270, 256, 24, 1, "返回 - 退出菜单");

        /* 学习计数 (24px, 三列分隔) */
        OS_Num_Show(270, 300, 24, 1, (float)Storage_Bit_z, "学习计数 R=%.0f");
        OS_Num_Show(450, 300, 24, 1, (float)Storage_Bit_c, "C=%.0f");
        OS_Num_Show(540, 300, 24, 1, (float)Storage_Bit_l, "L=%.0f");

        /* 校准标志 (24px, x=270 / x=520, 充分分隔避免覆盖) */
        OS_String_Show(270, 336, 24, 1,
                       (Proportion[PHA_LEARNED_FLAG] >= 0.5f) ? "相位校准: 已校准" : "相位校准: 未校准");
        OS_String_Show(520, 336, 24, 1,
                       (Proportion[VOS_LEARNED_FLAG] >= 0.5f) ? "偏移校准: 已校准" : "偏移校准: 未校准");

        if (Storage_mode >= 1 && Storage_mode <= 3) {
            m = (Storage_mode == 1) ? "电阻" : ((Storage_mode == 2) ? "电容" : "电感");
            OS_String_Show(270, 372, 24, 1, "上次测量类型: ");
            OS_String_Show(438, 372, 24, 1, m);
        } else {
            OS_String_Show(270, 372, 24, 1, "上次测量类型: 无");
        }

        OS_String_Show(270, 416, 16, 1, "按 1/2/3/4 选择     按返回键退出");

        /* 等待按键选择 */
        submode = 0;
        Ps2KeyValue = KeyValue_Null;
        while (submode == 0) {
            switch (Ps2KeyValue) {
                case KeyValue_1:    submode = 1;   break;
                case KeyValue_2:    submode = 2;   break;
                case KeyValue_3:    submode = 3;   break;
                case KeyValue_4:    submode = 4;   break;
                case KeyValue_Back: submode = 255; break;
                default: break;
            }
            if (Ps2KeyValue != KeyValue_Null) Ps2KeyValue = KeyValue_Null;
            if (submode == 0) OSTimeDly(20);
        }
        if (submode == 255) break;

        switch (submode) {
            case 1: LearnRCL_Submode();   break;
            case 2: LearnPhase_Submode(); break;
            case 3: LearnVOS_Submode();   break;
            case 4: ViewData_Submode();   break;
            default: break;
        }
    }

    LCD_Appoint_Clear(250 + 2, 64 + 8, 800, 480 - 32 - 8, Black);
    Change_Menu(0);
}

/**
  * @brief  菜单5 清除学习数据
  * @param  无
  * @retval 无
  */
void MenuHaddler_5() {
	Ps2KeyValue = KeyValue_Null;
    uint8_t confirm_flag = 0;

    // 确认提示
    LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black );
    Show_Val(1,0,"确认清除所有学习数据？");
    Show_Val(2,0,"确认=清除  取消=返回");

    // 等待确认
    while(1) {
        if(Ps2KeyValue == KeyValue_Enter) {
            confirm_flag = 1;
            break;
        }
        if(Ps2KeyValue == KeyValue_Back) {
            confirm_flag = 0;
            break;
        }
        OSTimeDly(10);
    }

    // 确认清除
    if(confirm_flag == 1) {
        Correct_init(); /* 初始化学习数据 */
        /* 写入Flash */
        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, PROPORTION_TOTAL);
        OSTimeDly(200);
        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
        OSTimeDly(200);
        Show_Val(1,0,"         清除完成         ");
        OSTimeDly(1000);
    }

	Change_Menu( 0 );
}

/* ***************************** Custom Function Part 自定义函数 ***************************** */
/* ------------------------------ 电感计算公式 ------------------------------ */
float CalculateInductanceSmallZabs(float zAbs, float pha, float fre) {
	float l = zAbs / (2 * PI * fre) * pow(10, 6);
	return 1.00f * l;
}

float CalculateInductanceLargeZabs(float zAbs, float pha, float fre) {
	float l = zAbs / (2 * PI * fre) * pow(10, 6);
	return 1.00f * l;
}

/* ------------------------------ 电容计算公式 ------------------------------ */
float CalculateCapacitanceMidRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if (sin_pha > -0.05f && sin_pha <  0.0f) sin_pha = -0.05f;
	if (sin_pha <  0.05f && sin_pha >= 0.0f) sin_pha =  0.05f;
	float c = -1.0f / (2 * PI * fre * zAbs * sin_pha) * pow(10, 9);
	return fabs(c);
}

float CalculateCapacitanceSmallRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if (sin_pha > -0.05f && sin_pha <  0.0f) sin_pha = -0.05f;
	if (sin_pha <  0.05f && sin_pha >= 0.0f) sin_pha =  0.05f;
	float c = -1.0f / (2 * PI * fre * zAbs * sin_pha) * pow(10, 9);
	return fabs(c);
}

float CalculateCapacitanceLargeRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if (sin_pha > -0.05f && sin_pha <  0.0f) sin_pha = -0.05f;
	if (sin_pha <  0.05f && sin_pha >= 0.0f) sin_pha =  0.05f;
	float c = -1.0f / (2 * PI * fre * zAbs * sin_pha) * pow(10, 9);
	return fabs(c);
}

/**
  * @brief  电容校准
  * @param  c: 原始电容值 nF
  * @retval 校准后电容值 nF
  */
float CalibrateCapacitance(float c) {
    return c;
}

/* ------------------------------ RC电路计算函数 ------------------------------ */
float CalculateRC_SResistance(float zAbs, float pha) {
	float r = zAbs * cos(pha / 180 * PI);
	if (r < 0.0f) r = 0.0f;
	return r;
}

float CalculateRC_SCapacitance(float zAbs, float pha, float fre) {
	return -1 / (zAbs * sin(pha / 180 * PI) * 2 * PI * fre) * pow(10, 9);
}

float CalculateRC_PResistance(float zAbs, float pha) {
	return zAbs * sqrt(1 + pow(tan(pha / 180 * PI), 2));
}

float CalculateRC_PCapacitance(float pha, float r, float fre) {
	return -tan(pha / 180 * PI) / (2 * PI * r * fre) * pow(10, 9);
}

/* ------------------------------ RL电路计算函数 ------------------------------ */
float CalculateRL_SResistance(float zAbs, float pha) {
	float cosp = cos(pha / 180 * PI);
	if (cosp < 0.05f) cosp = 0.05f;
	return zAbs * cosp;
}

float CalculateRL_SInductance(float zAbs, float pha, float fre) {
	return (zAbs * sin(pha / 180 * PI)) / (2 * PI * fre) * pow(10, 6);
}

/**
  * @brief  数组排序，取平均值使用
  * @param  arr: 数组
  * @param  len: 长度
  * @retval 无
  */
void SortArray(float arr[], uint8_t len) {
	uint8_t i, j;
	float t;
	for (i = 0; i < len; i++) {
		for (j = 0; j < len - i - 1; j++) {
			if (arr[j] > arr[j + 1]) {
				t = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = t;
			}
		}
	}
}

float CalculateRL_PResistancePart1(float zAbs, float pha) {
	float tan_pha = tan(pha * PI / 180.0f);
	if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;
	return zAbs * sqrt(1.0f + tan_pha * tan_pha);
}

/**
  * @brief  RL并联电感计算
  */
float CalculateRL_PInductancePart1(float r, float fre, float pha) {
    float tan_pha = tan(pha * PI / 180.0f);
    if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;
    if(fre < 10.0f) fre = 10.0f;
	return r / (2 * PI * fre * tan_pha) * 1e6f;
}

float CalculateRL_PResistancePart2(float zAbs, float pha) {
	return zAbs * sqrt(1 + pow(tan(pha / 180 * PI), 2)) * 1.062f;
}

/**
  * @brief  RL并联电感计算
  */
float CalculateRL_PInductancePart2(float r, float fre, float pha) {
    float tan_pha = tan(pha * PI / 180.0f);
    if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;
    if(fre < 10.0f) fre = 10.0f;
	return r / (2 * PI * fre * tan_pha) * 1e6f;
}

/* ------------------------------ LC电路计算函数 ------------------------------ */
float CalculateLC_SC(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3) {
	float numerator = pow(w1, 2) - pow(w2, 2);
	float denominator = w1 * w2 * (sin(pha1 * PI / 180.0f) * zAbs1 * w2 - sin(pha3 * PI / 180.0f) * zAbs3 * w1);
	if(fabs(denominator) < 1e-10f) return 0.0f;
	return numerator / denominator;
}

float CalculateLC_SL(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3) {
	float numerator = sin(pha1 * PI / 180.0f) * zAbs1 * w1 - sin(pha3 * PI / 180.0f) * zAbs3 * w2;
	float denominator = pow(w1, 2) - pow(w2, 2);
	if(fabs(denominator) < 1e-10f) return 0.0f;
	return numerator / denominator;
}

float CalculateResonantFrequency(float l, float c) {
	if (l <= 0.0f || c <= 0.0f) {
		return 0.0f;
	}
	return 1 / (2 * PI * sqrt(l * c));
}

float CalculateResonantFrequencyFromSweep(uint8_t network, float fre[], float zAbs[], float phaDif[]) {
	uint8_t i;
	uint8_t best_idx = 0;
	float p1, p2;
	float f0;
	
	for (i = 0; i < 4; i++) {
		p1 = phaDif[i];
		p2 = phaDif[i + 1];
		if (fabs(p1) < 0.5f) {
			return fre[i];
		}
		if ((p1 < 0.0f && p2 > 0.0f) || (p1 > 0.0f && p2 < 0.0f)) {
			if (fabs(p2 - p1) < 0.001f) {
				return fre[i];
			}
			f0 = fre[i] + (0.0f - p1) * (fre[i + 1] - fre[i]) / (p2 - p1);
			if (f0 >= fre[i] && f0 <= fre[i + 1]) {
				return f0;
			}
		}
	}
	
	if (network == LC_S) {
		best_idx = 0;
		for (i = 1; i < 5; i++) {
			if (zAbs[i] < zAbs[best_idx]) {
				best_idx = i;
			}
		}
		return fre[best_idx];
	}
	
	if (network == LC_P) {
		best_idx = 0;
		for (i = 1; i < 5; i++) {
			if (zAbs[i] > zAbs[best_idx]) {
				best_idx = i;
			}
		}
		return fre[best_idx];
	}
	
	return 0.0f;
}

float CalculateLC_PCPart1(float w1, float w2, float zAbs1, float zAbs3) {
	float numerator = w1 / zAbs1 - w2 / zAbs3;
	float denominator = pow(w1, 2) - pow(w2, 2);
	if(fabs(denominator) < 1e-10f) return 0.0f;
	return numerator / denominator;
}

float CalculateLC_PLPart1(float w1, float w2, float zAbs4, float zAbs2) {
	float numerator = pow(w1, 2) - pow(w2, 2);
	float denominator = w1 * w2 * (w1 / zAbs4 - w2 / zAbs2);
	if(fabs(denominator) < 1e-10f) return 0.0f;
	return numerator / denominator / 10;
}

float CalculateLC_PCPart2(float w1, float w2, float zAbs1, float zAbs4) {
	float numerator = w1 / zAbs1 - w2 / zAbs4;
	float denominator = pow(w1, 2) - pow(w2, 2);
	if(fabs(denominator) < 1e-10f) return 0.0f;
	return numerator / denominator;
}

float CalculateLC_PLPart2(float w1, float w2, float zAbs4, float zAbs0) {
	float numerator = pow(w1, 2) - pow(w2, 2);
	float denominator = w1 * w2 * (w1 / zAbs4 - w2 / zAbs0);
	if(fabs(denominator) < 1e-10f) return 0.0f;
	return numerator / denominator;
}

/**
  * @brief  获取更高阻值档位
  * @param  Gear: 当前档位
  * @retval 目标档位
  */
uint8_t GetHigherGear(uint8_t Gear) {
	switch (Gear) {
		case Gear_47: return Gear_820;
		case Gear_820: return Gear_15k;
		case Gear_15k: return Gear_270k;
		default: return Gear;
	}
}

/**
  * @brief  获取更低阻值档位
  * @param  Gear: 当前档位
  * @retval 目标档位
  */
uint8_t GetLowerGear(uint8_t Gear) {
	switch (Gear) {
		case Gear_820: return Gear_47;
		case Gear_15k: return Gear_820;
		case Gear_270k: return Gear_15k;
		default: return Gear;
	}
}

/**
  * @brief  根据阻抗选择档位
  * @param  zAbs: 当前阻抗
  * @param  Gear: 当前档位
  * @retval 目标档位
  */
uint8_t GetBoundaryGear(float zAbs, uint8_t Gear) {
	switch (Gear) {
		case Gear_47:
			if (zAbs > GEAR_BOUND_47_820_UP) return Gear_820;
			break;
		case Gear_820:
			if (zAbs < GEAR_BOUND_47_820_DOWN) return Gear_47;
			if (zAbs > GEAR_BOUND_820_15K_UP) return Gear_15k;
			break;
		case Gear_15k:
			if (zAbs < GEAR_BOUND_820_15K_DOWN) return Gear_820;
			if (zAbs > GEAR_BOUND_15K_270K_UP) return Gear_270k;
			break;
		case Gear_270k:
			if (zAbs < GEAR_BOUND_15K_270K_DOWN) return Gear_15k;
			break;
		default:
			break;
	}
	return Gear;
}

/**
  * @brief  自动阻抗档位切换
  * @retval 1=档位切换 0=未切换
  */
uint8_t change_resistance_gear(void) {
	uint8_t targetGear;
    float z_current;
	uint8_t gear_changed = 0;

	Display_flag = 0;

	uint8_t max_attempts = 3;
	while (Display_flag == 0 && max_attempts > 0) {
		targetGear = Gear_sign;
		max_attempts--;

		if (Ps2KeyValue == KeyValue_Back) {
			Display_flag = 1;
			return gear_changed;
		}

		/* FFT计算 */
		FFT_SetTargetFre(ddsStructData.hz);
		FFT_Handle();
		pha = User_FixPhase(pha);
		ShowPha = pha;
        Get_ZabsWithType(ADC1VOL, ADC2VOL, ddsStructData.hz, 0);
        z_current = Z_abs;

		/* 电压判断 */
		if (ADC2VOL > 3.0f) {
			targetGear = GetLowerGear(Gear_sign);
		}
		else if (ADC2VOL < 0.005f) {
			targetGear = GetHigherGear(Gear_sign);
		}
		else {
			targetGear = GetBoundaryGear(z_current, Gear_sign);
		}

		if (targetGear != Gear_sign) {
			Gear_sign = targetGear;
			SetGear(Gear_sign);
            gear_changed = 1;
            OSTimeDly(30);
		} else {
			Display_flag = 1;
		}
		OSTimeDly(5);
	}
	return gear_changed;
}

/**
  * @brief  相位修正
  * @param  pha: 原始相位
  * @retval 修正后相位
  */
float User_FixPhase( float pha ) {
	while ( 1 ) {
		if ( pha < -180 )
			pha = pha + 360;
		else if ( pha > 180 )
			pha = pha - 360;
		else
			return pha;
	}
}

/**
  * @brief  设置电阻档位
  * @param  Gear: 档位枚举
  * @retval 无
  */
void SetGear(uint8_t Gear) {
	switch (Gear) {
		case Gear_47:
			PCout(11) = 0; PCout(12) = 0; /* 档位00 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 47Ω             ");
			break;
		case Gear_820:
			PCout(11) = 1; PCout(12) = 0; /* 档位10 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 820Ω            ");
			break;
		case Gear_15k:
			PCout(11) = 0; PCout(12) = 1; /* 档位01 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 15kΩ            ");
			break;
		case Gear_270k:
			PCout(11) = 1; PCout(12) = 1; /* 档位11 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 270kΩ           ");
			break;
	}
}

/**
  * @brief  档位切换GPIO初始化
  * @param  无
  * @retval 无
  */
void GPIO_Change_Init() 
{ 
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); /* 使能GPIOC时钟 */
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12; /* PC11/PC12 */
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; /* 输出模式 */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* 推挽输出 */
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; /* 无上下拉 */
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		PCout(11)=1;		/* 默认270k欧档位 */
		PCout(12)=1; 
}

/**
  * @brief  获取FFT计算信息
  * @param  FreSet: 目标频率
  * @param  MenuMode: 菜单模式
  * @retval 无
  */
void Get_FFTInformation(float FreSet, uint8_t MenuMode)
{
    uint8_t i;
    uint8_t Pha_Max, Pha_Min, Vol1_Max, Vol1_Min, Vol2_Max, Vol2_Min;
    uint8_t LowFreFlag;
    float Pha_Sum, Vol1_Sum, Vol2_Sum;
    float FreTemp;
	float Pha_Ref = 0;
	float Pha_Avg = 0;
	
    i = 0;
    LowFreFlag = 0;

    /* 低频处理 */
    if (FreSet <= 100)
    {
        LowFreFlag = 1;
    }
    FreTemp = FreSet;

    /************************ 使用频率状态直接计算FFT ************************/
    if (LowFreFlag)
    {
        Sample_rate = 64 * FreTemp;
        Sample_rate = Set_SamplingFre(Sample_rate);
        FFT_SetTargetFre(FreTemp);
        OSTimeDly(5);
        FFT_Handle();
        pha = User_FixPhase(pha);
        LowFreFlag = 0;
        ShowPha = pha;
    }
    else
    {
        /* 动态设置采样率 */
        if (FreTemp < 20000)
        {
            Sample_rate = 32 * FreTemp;
        }
        else if (FreTemp >= 20000 && FreTemp < 50000)
        {
            Sample_rate = 16 * FreTemp;
        }
        else if (FreTemp >= 50000 && FreTemp < 100000)
        {
            Sample_rate = 8 * FreTemp;
        }
        else
        {
            Sample_rate = MAX_SAMPLE_RATE;
        }
        Sample_rate = Set_SamplingFre(Sample_rate);
        FFT_SetTargetFre(FreTemp);
        OSTimeDly(5);
        FFT_Handle();

        /* 10次采样去极值滤波 */
        i = 0;
        while (i < 10)
        {
			FFT_Handle();
			pha = User_FixPhase(pha);

			if (i == 0)
			{
				Pha_Ref = pha;
				PhaArrary[i] = pha;
			}
			else
			{
				PhaArrary[i] = User_AlignPhase(pha, Pha_Ref);
			}
			ADC1VolArrary[i] = ADC1VOL;
			ADC2VolArrary[i] = ADC2VOL;
			i++;
        }
        if (i >= 10)
        {
            Pha_Sum = 0.0f;
            Pha_Max = 0;
            Pha_Min = 0;
            Vol1_Sum = 0.0f;
            Vol2_Sum = 0.0f;
            Vol1_Max = 0;
            Vol1_Min = 0;
            Vol2_Max = 0;
            Vol2_Min = 0;

            /* 寻找极值 */
            for (i = 0; i < 10; i++)
            {
                if (PhaArrary[i] > PhaArrary[Pha_Max]) Pha_Max = i;
                if (PhaArrary[i] < PhaArrary[Pha_Min]) Pha_Min = i;
                if (ADC1VolArrary[i] > ADC1VolArrary[Vol1_Max]) Vol1_Max = i;
                if (ADC1VolArrary[i] < ADC1VolArrary[Vol1_Min]) Vol1_Min = i;
                if (ADC2VolArrary[i] > ADC2VolArrary[Vol2_Max]) Vol2_Max = i;
                if (ADC2VolArrary[i] < ADC2VolArrary[Vol2_Min]) Vol2_Min = i;

                Vol1_Sum += ADC1VolArrary[i];
                Vol2_Sum += ADC2VolArrary[i];
                Pha_Sum += PhaArrary[i];
            }

            /* 计算平均值 */
			Pha_Avg = (Pha_Sum - PhaArrary[Pha_Max] - PhaArrary[Pha_Min]) / 8.0f;
			ShowPha = User_FixPhase(Pha_Avg);
            ADC1VOL = (Vol1_Sum - ADC1VolArrary[Vol1_Max] - ADC1VolArrary[Vol1_Min]) / 8.0f;
            ADC2VOL = (Vol2_Sum - ADC2VolArrary[Vol2_Max] - ADC2VolArrary[Vol2_Min]) / 8.0f;
        }
    }
}

/**
  * @brief  快速FFT计算，用于扫频
  * @param  FreSet: 目标频率
  * @retval 无
  */
void Get_FFTQuick(float FreSet)
{
    float FreTemp = FreSet;

    /* 动态设置采样率 */
    if (FreTemp <= 100)
    {
        Sample_rate = 64 * FreTemp;
    }
    else if (FreTemp < 20000)
    {
        Sample_rate = 32 * FreTemp;
    }
    else if (FreTemp >= 20000 && FreTemp < 50000)
    {
        Sample_rate = 16 * FreTemp;
    }
    else if (FreTemp >= 50000 && FreTemp < 100000)
    {
        Sample_rate = 8 * FreTemp;
    }
    else
    {
        Sample_rate = MAX_SAMPLE_RATE;
    }

    Sample_rate = Set_SamplingFre(Sample_rate);
    FFT_SetTargetFre(FreTemp);
    OSTimeDly(5);

    /* 快速FFT */
    FFT_Handle();
    ShowPha = User_FixPhase(pha);
}

/**
  * @brief  计算阻抗模
  * @param  Get_ADC1: 通道1电压
  * @param  Get_ADC2: 通道2电压
  * @param  NowFre: 当前频率
  * @retval 无
  */
void Get_Zabs(float Get_ADC1, float Get_ADC2, float NowFre) {
    Get_ZabsWithType(Get_ADC1, Get_ADC2, NowFre, (uint8_t)Show_flag);
}

void Get_ZabsWithType(float Get_ADC1, float Get_ADC2, float NowFre, uint8_t element_type) {
    uint8_t freq_idx = 0;
    uint8_t fi;
    uint8_t gear_idx;
    float rref;
    float cal;

    if(Get_ADC2 < 0.005f) {
        Z_abs = 999999999.0f;
        return;
    }

    for (fi = 0; fi < 5; fi++) {
        if (fabs(NowFre - Fre_Sweep[fi]) < (Fre_Sweep[fi] * 0.1f + 1.0f)) {
            freq_idx = fi;
            break;
        }
    }

    gear_idx = (Gear_sign < Gear_Count) ? Gear_sign : 1;
    rref = Rref_Table[gear_idx];

    switch(element_type) {
        case 1:
            cal = Resistance_Calibration[gear_idx];
            break;
        case 2:
            cal = Capacitance_Calibration[gear_idx][freq_idx];
            break;
        case 3:
            cal = Inductance_Calibration[gear_idx][freq_idx];
            break;
        case 0:
        default:
            cal = (Capacitance_Calibration[gear_idx][freq_idx] + Inductance_Calibration[gear_idx][freq_idx]) * 0.5f;
            break;
    }

    if (fabs(cal) < 0.0001f) {
        cal = 1.0f;
    }

    Z_abs = rref * Get_ADC1 / Get_ADC2 / cal;

    if(Z_abs < 0.0f) Z_abs = 0.0f;
}
/**
  * @brief  获取电路网络类型
  * @retval 电路网络类型
  */
uint8_t Get_Network(void) {
	return User_GetNetwork(Zabs_Sweep, Pha_Sweep);
}

/**
  * @brief  判断扫频得到的主要元件类型
  * @param  pha_dif[]: 相位差值
  * @param  z_abs[]: 阻抗值
  * @retval 1=电阻 2=电容 3=电感
  */
uint8_t DetectPrimaryElementType(float pha_dif[], float z_abs[]) {
	uint8_t i;
	uint8_t positive_cnt = 0;
	uint8_t negative_cnt = 0;
	float pha_sum = 0.0f;
	float z_sum = 0.0f;
	float z_avg = 0.0f;

	for (i = 0; i < 5; i++) {
		pha_sum += pha_dif[i];
		z_sum += z_abs[i];
		if (pha_dif[i] > 12.0f) {
			positive_cnt++;
		} else if (pha_dif[i] < -12.0f) {
			negative_cnt++;
		}
	}

	z_avg = z_sum / 5.0f;

	if (negative_cnt >= 2 && pha_sum < -20.0f) {
		return 2;
	}

	if (positive_cnt >= 2 && pha_sum > 20.0f) {
		return 3;
	}

	if (pha_dif[2] < -25.0f && z_avg > 1.0f) {
		return 2;
	}

	if (pha_dif[2] > 25.0f && z_avg > 1.0f) {
		return 3;
	}

	return 1;
}

/**
  * @brief  电路网络识别算法
  * @param  z_abs[]: 阻抗值数组
  * @param  pha_dif[]: 相位值数组
  * @retval 识别到的电路网络类型
  */
uint8_t User_GetNetwork( float z_abs[], float pha_dif[] ) {
	uint8_t network = Nw_Null;
    uint8_t i;
    uint8_t z_min_idx = 0;
    uint8_t z_max_idx = 0;
    uint8_t z_rise_cnt = 0;
    uint8_t z_fall_cnt = 0;
    uint8_t pha_rise_cnt = 0;
    uint8_t pha_fall_cnt = 0;
    float pha_min;
    float pha_max;
    float pha_span;
    float lc_ratio;
    float left_z;
    float right_z;
    float edge_phase_delta;

    for(i = 0; i < 5; i++) {
        if(z_abs[i] < z_abs[z_min_idx]) z_min_idx = i;
        if(z_abs[i] > z_abs[z_max_idx]) z_max_idx = i;
    }

    pha_min = pha_dif[0];
    pha_max = pha_dif[0];
    for(i = 0; i < 5; i++) {
        if(pha_dif[i] < pha_min) pha_min = pha_dif[i];
        if(pha_dif[i] > pha_max) pha_max = pha_dif[i];
    }
    pha_span = pha_max - pha_min;

    for(i = 0; i < 4; i++) {
        if(z_abs[i + 1] > z_abs[i] * 1.05f) {
            z_rise_cnt++;
        } else if(z_abs[i + 1] < z_abs[i] * 0.95f) {
            z_fall_cnt++;
        }

        if(pha_dif[i + 1] > pha_dif[i] + 2.0f) {
            pha_rise_cnt++;
        } else if(pha_dif[i + 1] < pha_dif[i] - 2.0f) {
            pha_fall_cnt++;
        }
    }

    if(pha_min < -15.0f && pha_max > 15.0f && pha_span > 40.0f) {
        if(z_min_idx > 0 && z_min_idx < 4) {
            left_z = z_abs[z_min_idx - 1];
            right_z = z_abs[z_min_idx + 1];
            if(z_abs[z_min_idx] > 0.1f) {
                lc_ratio = ((left_z < right_z) ? left_z : right_z) / z_abs[z_min_idx];
                if(lc_ratio > 1.25f) {
                    return LC_S;
                }
            }
        }

        if(z_max_idx > 0 && z_max_idx < 4) {
            left_z = z_abs[z_max_idx - 1];
            right_z = z_abs[z_max_idx + 1];
            if(left_z > 0.1f || right_z > 0.1f) {
                lc_ratio = z_abs[z_max_idx] / (((left_z > right_z) ? left_z : right_z) + 0.1f);
                if(lc_ratio > 1.25f) {
                    return LC_P;
                }
            }
        }
    }

    /* Bug#8: Relaxed LC fallback - resonance just outside sweep range.
       If |Z| shows a strong V-dip or inverted-V peak in the MIDDLE of
       the sweep AND phase span is moderate, declare LC even when phase
       does not cross zero in our 300Hz..50kHz window. */
    if(pha_span > 25.0f) {
        if(z_min_idx >= 1 && z_min_idx <= 3 && z_abs[z_min_idx] > 0.1f) {
            left_z  = z_abs[z_min_idx - 1];
            right_z = z_abs[z_min_idx + 1];
            lc_ratio = ((left_z < right_z) ? left_z : right_z) / z_abs[z_min_idx];
            if(lc_ratio > 1.5f) return LC_S;
        }
        if(z_max_idx >= 1 && z_max_idx <= 3) {
            left_z  = z_abs[z_max_idx - 1];
            right_z = z_abs[z_max_idx + 1];
            if(left_z > 0.1f || right_z > 0.1f) {
                lc_ratio = z_abs[z_max_idx] / (((left_z > right_z) ? left_z : right_z) + 0.1f);
                if(lc_ratio > 1.5f) return LC_P;
            }
        }
    }

    edge_phase_delta = pha_dif[4] - pha_dif[0];

    /* ============================================================
     * Bug#11-A: LC resonance detectors covering the gaps in the legacy
     * "peak/dip in middle of sweep" rule.
     *
     * Two subcases the legacy rule misses:
     *   (i)  Resonance just OUTSIDE the sweep window -> |Z| diverges but
     *        the peak/dip index sits at an endpoint (idx 0 or 4).
     *   (ii) Resonance BETWEEN two adjacent sweep points -> the phase
     *        flips sign across that pair but no single sample sits at
     *        the extremum.
     *
     * Detector (i): super-linear |Z| growth/drop relative to pure L/C.
     *   Pure L:  |Z[i+1]/Z[i]| == omega-ratio   (linear)
     *   Pure C:  |Z[i]/Z[i+1]| == omega-ratio   (linear in 1/omega)
     *   LC near resonance:  |Z| ratio >> omega-ratio.
     * RL_S/RL_P/RC_S/RC_P are bounded by R, so they never exceed the
     * pure-L/pure-C linear rate. >1.5x super-linear -> LC + resonance nearby.
     *
     * Detector (ii): phase sign reversal between adjacent samples.
     *   LC_P crosses resonance:  pha goes from positive to negative
     *                            (L branch wins below, C branch wins above).
     *   LC_S crosses resonance:  pha goes from negative to positive.
     *
     * Both detectors run BEFORE the impedance-trend block, because
     * they have specific signatures that the generic trend logic
     * (z_lo vs z_hi ratio) cannot represent.
     * ============================================================ */
    {
        uint8_t k;
        /* (ii) phase sign reversal across adjacent samples (run first --
         * cheapest and most decisive). */
        for (k = 0; k < 4; k++) {
            if (pha_dif[k] > 8.0f && pha_dif[k + 1] < -8.0f) {
                /* +90 -> -90  : LC parallel (L below, C above resonance) */
                return LC_P;
            }
            if (pha_dif[k] < -8.0f && pha_dif[k + 1] > 8.0f) {
                /* -90 -> +90  : LC series (C below, L above resonance) */
                return LC_S;
            }
        }
        /* (i) super-linear |Z| growth/drop on high-frequency pairs. */
        for (k = 2; k < 4; k++) {
            float w_ratio = Fre_Sweep[k + 1] / Fre_Sweep[k];
            if (z_abs[k] > 0.1f && z_abs[k + 1] > 0.1f) {
                float z_growth = z_abs[k + 1] / z_abs[k];
                float z_drop   = z_abs[k]     / z_abs[k + 1];
                if (z_growth > w_ratio * 1.5f && pha_max >  20.0f && pha_min > -10.0f) {
                    /* Inductive everywhere + super-linear growth -> LC_P sub-resonance */
                    return LC_P;
                }
                if (z_drop   > w_ratio * 1.5f && pha_min < -20.0f && pha_max <  10.0f) {
                    /* Capacitive everywhere + super-linear drop -> LC_S sub-resonance */
                    return LC_S;
                }
            }
        }
    }

    /* Impedance-trend S-vs-P discriminator (robust against phase noise).
     *   RC_S: |Z| drops sharply at low freq (cap dominant), flat at high freq (~R)
     *   RC_P: |Z| flat at low freq (~R),     drops sharply at high freq (cap branch)
     *   RL_S: |Z| flat at low freq (~R),     rises sharply at high freq (L)
     *   RL_P: |Z| rises sharply at low freq (L), flat at high freq (~R)
     * z_lo = Z[2]/Z[0] low-decade ratio; z_hi = Z[4]/Z[2] high-decade ratio.
     * This block runs BEFORE the legacy phase-only checks so we catch
     * the noisy/marginal cases that the phase logic was missing. */
    {
        float z_lo = (z_abs[0] > 1.0e-3f) ? (z_abs[2] / z_abs[0]) : 1.0f;
        float z_hi = (z_abs[2] > 1.0e-3f) ? (z_abs[4] / z_abs[2]) : 1.0f;
        float pha_mean = (pha_dif[0] + pha_dif[1] + pha_dif[2] + pha_dif[3] + pha_dif[4]) * 0.2f;
        uint8_t cap_hint = (pha_mean < -3.0f) || (pha_max < -5.0f)
                         || ((z_lo < 0.9f || z_hi < 0.9f) && pha_mean <  2.0f);
        uint8_t ind_hint = (pha_mean >  3.0f) || (pha_min >  5.0f)
                         || ((z_lo > 1.1f || z_hi > 1.1f) && pha_mean > -2.0f);

        /* Bug#11-B: High-phase-magnitude parallel detector.
         * For high-R parallel networks the parasitic series R distorts the
         * z_lo/z_hi ratios so the legacy comparison can flip sides.  But the
         * AVERAGE phase magnitude stays near the parallel-element side
         * (~+80 for RL_P, ~-80 for RC_P).  Important distinction vs the
         * SERIES topology with the same dominant element:
         *   RL_S: phase RISES with freq (atan(wL/R) monotone in w)
         *   RL_P: phase FALLS with freq (atan(R/(wL)) monotone-down in w)
         *   RC_S: phase RISES toward 0 with freq (less negative at high f)
         *   RC_P: phase FALLS away from 0 with freq (more negative at high f)
         * Require both elevated mean phase AND the parallel-side direction
         * of phase change so we do not steal cases from low-R RL_S/RC_S. */
        if (pha_mean < -40.0f && pha_min < -50.0f
            && pha_dif[4] <= pha_dif[2] + 5.0f) {
            /* Capacitive on average + phase NOT becoming less negative at
             * high freq -> RC_P (RC_S would see |pha| shrink as wRC grows). */
            return RC_P;
        }
        if (pha_mean >  40.0f && pha_max >  50.0f
            && pha_dif[4] <= pha_dif[2] - 5.0f) {
            /* Inductive on average + phase falling at high freq -> RL_P
             * (RL_S phase rises monotonically). */
            return RL_P;
        }

        if (cap_hint && !ind_hint) {
            if (z_hi < z_lo * 0.85f) return RC_P;       /* high-end drops more */
            if (z_lo < z_hi * 0.85f) return RC_S;       /* low-end drops more */
            if (edge_phase_delta < -3.0f) return RC_P;
            if (edge_phase_delta >  3.0f) return RC_S;
            return (pha_dif[4] < pha_dif[0]) ? RC_P : RC_S;
        }

        if (ind_hint && !cap_hint) {
            if (z_lo > z_hi * 1.18f) return RL_P;       /* low-end rises more */
            if (z_hi > z_lo * 1.18f) return RL_S;       /* high-end rises more */
            if (edge_phase_delta < -3.0f) return RL_P;
            if (edge_phase_delta >  3.0f) return RL_S;
            return (pha_dif[0] > pha_dif[4]) ? RL_P : RL_S;
        }
    }

    if(pha_max < -8.0f || (Show_flag == 2 && pha_max < 8.0f && pha_min < -2.0f)) {
        /* Bug#10: require margin >=2 OR edge_delta>=6 to flip to P (noise-robust at high |Z|) */
        if(pha_fall_cnt > pha_rise_cnt + 1 || edge_phase_delta < -6.0f) {
            network = RC_P;
        } else {
            network = RC_S;
        }
        return network;
    }

    if(pha_min > 8.0f || (Show_flag == 3 && pha_min > -8.0f && pha_max > 2.0f)) {
        /* Bug#10: require margin >=2 OR edge_delta>=6 to flip to P */
        if(pha_rise_cnt > pha_fall_cnt + 1 || edge_phase_delta > 6.0f) {
            network = RL_S;
        } else {
            network = RL_P;
        }
        return network;
    }

    if(Show_flag == 2 && z_fall_cnt >= 2) {
        /* Bug#10: margin >=2 OR edge_delta>=3deg even in fallback */
        if(pha_fall_cnt > pha_rise_cnt + 1 || edge_phase_delta < -3.0f) {
            return RC_P;
        }
        return RC_S;
    }

    if(Show_flag == 3 && z_rise_cnt >= 2) {
        /* Bug#10: margin >=2 OR edge_delta>=3deg even in fallback */
        if(pha_rise_cnt > pha_fall_cnt + 1 || edge_phase_delta > 3.0f) {
            return RL_S;
        }
        return RL_P;
    }

	return Nw_Null;
}

/* ==================== 学习模式校准功能 ==================== */
/**
  * @brief  数值校准
  * @param  mode: 元件校准模式
  * @retval 无
  */
void Correct(uint8_t mode) {
	u32 i=0;
	u32 Correct_flag=0;
	switch(mode) {
		case 1: /* 电阻校准 */
			for(i=0; i<Storage_Bit_z && i<400; i++) {
				if(Z_abs>=0.95f*Value[i] && Z_abs<=1.05f*Value[i]) {
					Z_abs*=Proportion[i+3];
					Correct_flag=1;
				}
				if(Correct_flag==1) break;
			}
			break;
		case 2: /* 电容校准 */
			for(i=0; i<Storage_Bit_c && i<400; i++) {
				if(C_abs>=0.95f*Value[i+400] && C_abs<=1.05f*Value[i+400]) {
					C_abs*=Proportion[i+3+400];
					Correct_flag=1;
				}
				if(Correct_flag==1) break;
			}
			break;
		case 3: /* 电感校准 */
			for(i=0; i<Storage_Bit_l && i<400; i++) {
				if(L_abs>=0.95f*Value[i+800] && L_abs<=1.05f*Value[i+800]) {
					L_abs*=Proportion[i+3+800];
					Correct_flag=1;
				}
				if(Correct_flag==1) break;
			}
			break;
	}
}

/**
  * @brief  初始化学习校准
  * @param  无
  * @retval 无
  */
void Correct_init(void) {
	int i;
	for(i=0;i<PROPORTION_TOTAL;i++) {
		Proportion[i]=0;
		Proportion_Tmep[i]=0;
		if(i<1200) {
			Value[i]=0;
			Value_Tmep[i]=0;
		}
	}
	Storage_Bit_z = 0;
	Storage_Bit_c = 0;
	Storage_Bit_l = 0;
	Proportion_Tmep[0] = 0;
	Proportion_Tmep[1] = 0;
	Proportion_Tmep[2] = 0;
}

/**
  * @brief  获取键盘输入数值
  * @param  num: 默认值
  * @retval 输入的数值
  */
float PS2_ReadNum(float num) {
	uint8_t count = 0;
	uint8_t dec_sign = 0;      /* 小数点标志位 */
	float temp_num = 0;
	uint32_t timeout_cnt = 0;  /* 超时计数 */
	
	/* 清空输入区域 */
	LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
	OS_Rect_Draw( 332 , 96 + 64 * 4 , 750 , 96 + 64 * 5 , 1 , White );
	OS_String_Show( 332 + 16 , 96 + 64 * 4 + 16 , 32, 1, "-> ");
	
	Ps2KeyValue = KeyValue_Null;

	while (1) {
		/* 超时退出 */
		if(timeout_cnt > 2000) { 
			LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
			return -1.0f;
		}
		
		/* 按键处理 */
		if (Ps2KeyValue != KeyValue_Null) {
			timeout_cnt = 0;
			
			/* 数字键0-9 */
			if (Ps2KeyValue >= KeyValue_0 && Ps2KeyValue <= KeyValue_9) {
				if (dec_sign == 0) {
					temp_num = temp_num * 10 + Ps2KeyValue;
				} else {
					temp_num = temp_num + (float)Ps2KeyValue / pow(10, count);
					count++;
				}
				OS_Num_Show( 332 + 80 , 96 + 64 * 4 + 16 , 32 , 1, temp_num , "%.6f      ");
			}
			
			/* 小数点 */
			else if (Ps2KeyValue == KeyValue_Point) {
				if(dec_sign == 0) {
					dec_sign = 1;
					count = 1;
				}
			}
			
			/* 确认键 */
			else if (Ps2KeyValue == KeyValue_Enter) {
				LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
				Ps2KeyValue = KeyValue_Null;
				return temp_num;
			}
			
			/* 返回键 */
			else if (Ps2KeyValue == KeyValue_Back) {
				LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
				Ps2KeyValue = KeyValue_Null;
				return -1.0f;
			}
			
			Ps2KeyValue = KeyValue_Null;
		}
		
		delay_ms(5);
		timeout_cnt++;
	}
}

/* ==================== 安全Flash读写 ==================== */
/**
  * @brief  安全Flash读取
  * @param  addr: 起始地址
  * @param  buf: 数据存储缓冲区
  * @param  len: 数据长度
  * @retval 1=成功 0=失败
  */
uint8_t InFLASH_Read_Safe(uint32_t addr, uint32_t *buf, uint32_t len) {
    /* 地址范围验证 */
    if (addr < ADDR_FLASH_SECTOR_10 || (addr + len * 4) > (ADDR_FLASH_SECTOR_11 + 1200 * 4)) {
        return 0;
    }
    InFLASH_Read(addr, buf, len);
    return 1;
}

/**
  * @brief  相位校准，将相位约束在180度范围内
  * @param  pha: 待校准相位
  * @param  ref: 参考相位
  * @retval 校准后相位值
  */
float User_AlignPhase(float pha, float pha_ref)
{
	while ((pha - pha_ref) > 180.0f)
	{
		pha -= 360.0f;
	}

	while ((pha - pha_ref) < -180.0f)
	{
		pha += 360.0f;
	}

	return pha;
}

/**
  * @brief  寻找阻抗最接近参考电阻的频率点
  * @param  z_abs[]: 5点扫频阻抗值
  * @param  gear_sweep[]: 扫频档位数组
  * @param  default_fre: 默认测量频率
  * @param  best_idx: 最优频率点索引，可为NULL
  * @retval 最优频率值
  */
float FindBestFrequency(float z_abs[], uint8_t gear_sweep[], float default_fre, uint8_t *best_idx) {
    float min_diff = 1e10f;
    float best_fre = default_fre;
    uint8_t fi;
    uint8_t selected_idx = 1;

    for (fi = 0; fi < 5; fi++) {
        if (z_abs[fi] > 10.0f && z_abs[fi] < 1000000.0f) {
            uint8_t gear_idx = (gear_sweep && gear_sweep[fi] < Gear_Count) ? gear_sweep[fi] : ((Gear_sign < Gear_Count) ? Gear_sign : 1);
            float rref = Rref_Table[gear_idx];
            float diff = fabs(z_abs[fi] - rref);
            if (diff < min_diff) {
                min_diff = diff;
                best_fre = Fre_Sweep[fi];
                selected_idx = fi;
            }
        }
    }

    if (best_idx) {
        *best_idx = selected_idx;
    }
    return best_fre;
}

/* ***************************** END *****************************/