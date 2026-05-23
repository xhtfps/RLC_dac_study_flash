/****************************
 * Project description: 阻抗测量识别项目
 * 原理：通过ADC采集+FFT分析实现阻抗测量识别RC/RLLC等电路网络的测量与分析
 * Author: Mao -> 2026 Mao
 * Creation Date: 2026/04/24
 * Update date: 2026/05/04 (优化)
 * 1-> 增加DAC输出DDS
 * 2-> 增加学习模式校准功能
 * 3-> 增加多档位硬件校准功能
 * ****************************/

/* ***************************** Include & Define Part     	***************************** */
#include "User.h"

/* ***************************** 系统参数配置 ***************************** */
#define MAX_SAMPLE_RATE  512000			/* 最大采样率 512kHz */
#define Sweep_Fre_Buff_Length    300     /* 扫频缓存长度 */

/* 测量模式枚举 C89标准 */
#define MENU_IMPEDANCE  1    /* 菜单1 阻抗模式 + 阻抗测量 */
#define MENU_NETWORK    2    /* 菜单2 网络识别 + 谐振频率 */
#define MENU_ORIGINAL   3    /* 菜单3 原始数据显示(ADC1/ADC2/相位) */

/* 4档测量档位 */
/* 标准参考值表格 Rref_Table[] 统一供 Resistance_Calibration/Capacitance_Calibration/Inductance_Calibration 使用 */

/* 不同档位参考值边界，预留约10%的档位切换 */
#define GEAR_BOUND_47_820_DOWN      157.0f
#define GEAR_BOUND_47_820_UP        235.0f
#define GEAR_BOUND_820_15K_DOWN     2800.0f
#define GEAR_BOUND_820_15K_UP       4500.0f
#define GEAR_BOUND_15K_270K_DOWN    50900.0f
#define GEAR_BOUND_15K_270K_UP      76400.0f

/* 不同频率对应的档位相位补偿 */

/* 扫频档位相位补偿，对应扫频点100Hz/500Hz/5kHz/20kHz/50kHz */
#define PHA_COMPENSATE_100HZ    0.0f
#define PHA_COMPENSATE_500HZ    0.0f
#define PHA_COMPENSATE_5KHZ     0.0f
#define PHA_COMPENSATE_20KHZ    0.0f
#define PHA_COMPENSATE_50KHZ   	5.0f

/* 不同档位枚举，切换时使用 */
enum User_Gear {
	Gear_47 = 0,    /* 47欧标准档位 */
	Gear_820,      	/* 820欧标准档位 */
	Gear_15k,      	/* 15k欧标准档位 */
	Gear_270k,      /* 270k欧标准档位 */
	Gear_Count = 4  /* 档位总数 */
};

/* 标准参考值表格，与枚举一一对应 */
const float Rref_Table[Gear_Count] = { 47.0f, 820.0f, 15000.0f, 270000.0f };

/* ==================== 阻抗校准系数表 ==================== */
/*
 * 每频率 + 每档位对应硬件校准系数
 * 行 = 档位Gear_47/Gear_820/Gear_15k/Gear_270k
 * 列 = 频率索引Fre_Sweep[0]~[4] = 100Hz/500Hz/5kHz/20kHz/50kHz
 *
 * 使用公式：Z = Rref[gear] * V1 / V2 / Calibration[gear][freq_idx]
 * 校准时只需要修改对应表格数值，刷新系统即可
 *
 * 原始值说明：
 *   原始数据5kHz 系数为 2.71267，扫频为 2.8677
 *   实际扫频测试中，通过实际校准测试
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
    /*  100Hz     500Hz      5kHz      20kHz     50kHz  */
    {  2.88625f,  2.8753f,  2.840f,    2.8804f,   3.2571f  },  /* Gear_47   (47欧)   */
    {  2.4592f,  2.7127f,   2.8677f,   2.7189f,   2.6434f  },  /* Gear_820  (820欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.2469f  },  /* Gear_15k  (15k欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_270k (270k欧) */
};

/* 电感校准系数表 */
float Inductance_Calibration[Gear_Count][5] = {
    /*  100Hz     500Hz      5kHz      20kHz     50kHz  */
    {  2.6490f,  2.7947f,   2.9418f,   2.8607f,   2.6951f  },  /* Gear_47   (47欧)   */
    {  2.4592f,  2.7127f,   2.8677f,   2.8677f,   2.7559f  },  /* Gear_820  (820欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.7680f  },  /* Gear_15k  (15k欧)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_270k (270k欧) */
};

/* ==================== 双通道ADC采集偏移校准参数 ==================== */
//#define VOS_ADC1  0.025f  /* 通道1偏移电压，短路时ADC1采集值 */
//#define VOS_ADC2  0.018f  /* 通道2偏移电压，短路时ADC2采集值 */

uint8_t Gear_sign = Gear_820; /* 当前档位标志，默认820欧 */

/* ***************************** 全局变量 ***************************** */
/* 外部文件变量 */
extern uint32_t ADCData[];      /* ADC采集数据缓冲区 */
extern DDSDataStruct dds[2];     /* DDS输出数据结构 */
extern float ADCfre;              /* ADC采样频率 */
extern float ADCfre1;
extern float ADC1VOL;            /* 通道1电压，待测元件两端电压 */
extern float ADC2VOL;            /* 通道2电压，标准电阻两端电压 */
extern float pha;                 /* 原始相位值 */

/* 菜单显示标志 */
uint8_t MenuSign = 0;            /* 当前菜单索引 */
unsigned char mode = 0 ;          /* 测量模式 */

float Sample_rate = MAX_SAMPLE_RATE; /* 当前采样率 */
float PhaArrary[10] = {0};			  /* 10个相位缓存，滤波使用 */
float ADC1VolArrary[10] = {0};		  /* 10个通道1电压数据 */
float ADC2VolArrary[10] = {0};		  /* 10个通道2电压数据 */
float ShowPha;						  /* 菜单最终显示相位 */
GRAPH_Struct GridData;                 /* 绘图数据结构 */
float Z_abs, L_abs, C_abs;             /* 阻抗模值，未校准原始值 */
uint8_t Display_flag = 0;               /* 显示刷新标志 */

/* 扫频数据缓冲区，固定5个频率点：100Hz, 500Hz, 5kHz, 20kHz, 50kHz */
float Pha_Sweep[5] = {0};               /* 5个扫频相位 */
float ADC1_Sweep[5] = {0};              /* 5个通道1电压 */
float ADC2_Sweep[5] = {0};              /* 5个通道2电压 */
float Fre_Sweep[5] = {100, 500, 5000, 20000, 50000}; /* 5个扫频频率 */
float Zabs_Sweep[5] = {0};              /* 5个阻抗模 */
float z_r[5] = {0};                      /* 实时阻抗真实值 */

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
uint16_t Show_flag = 0;        /* 元件类型标志 1=电阻，2=电容，3=电感*/
float last_z;                   /* 上一个阻抗值，防止频率抖动 */

/* 学习数据存储结构，1203个float存储在flash */
float Proportion[1203];        /* 比例系数表[0]=电阻总数,[1]=电容总数, [2]=电感总数[3-402]=电阻系数, [403-802]=电容系数, [803-1202]=电感系数*/
uint32_t Proportion_Tmep[1203];/* Flash存储32位转换临时值 */
float Value[1200];              /* 学习时原始数据值 */
uint32_t Value_Tmep[1200];     /* Flash存储32位转换临时值 */
uint32_t Storage_Bit_z = 0;     /* 电阻学习数据存储指针 0-400 */
uint32_t Storage_Bit_c = 0;     /* 电容学习数据存储指针 0-400 */
uint32_t Storage_Bit_l = 0;     /* 电感学习数据存储指针 0-400 */
uint32_t Storage_mode = 0;      /* 当前学习模式 1=学电阻，2=学电容，3=学电感*/
float data;                      /* 实时数据计算值 */
/* ========================================================== */

/* ***************************** 函数声明 ***************************** */
/* 通用公式函数 */
float User_FixPhase( float pha );        /* 相位修正到-180~180 */
void GPIO_Change_Init();                  /* 切换档位GPIO初始化，用于阻抗档位 */
void SetGear(uint8_t Gear);               /* 设置当前阻抗档位 */
void Get_FFTInformation(float FreSet, uint8_t MenuMode); /* 获取FFT分析信息，不同模式处理 */
void Get_FFTQuick(float FreSet);                          /* 快速FFT分析，用于扫频预处理 */
void Get_Zabs(float Get_ADC1, float Get_ADC2, float NowFre); /* 计算阻抗 */
void Get_ZabsWithType(float Get_ADC1, float Get_ADC2, float NowFre, uint8_t element_type);
uint8_t Get_Network(void);                /* 计算电路识别 */
uint8_t User_GetNetwork( float z_abs[], float pha_dif[] );    /* 精确电路识别 */
uint8_t DetectPrimaryElementType(float pha_dif[], float z_abs[]); /* 检测主要元件类型 1=R 2=C 3=L */
uint8_t change_resistance_gear(void);     /* 自动阻抗切换 1=档位切换 */
uint8_t GetHigherGear(uint8_t Gear);      /* 获取更高阻值档位 */
uint8_t GetLowerGear(uint8_t Gear);       /* 获取更低阻值档位 */
uint8_t GetBoundaryGear(float zAbs, uint8_t Gear); /* 计算阻抗选档位 */

/* 阻抗计算公式，统一在 Get_Zabs() + 校准系数表中调用 */

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
void SortArray(float arr[], uint8_t len); /* 排序数组取均值滤波使用 */
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
float PS2_ReadNum(float num);      /* 获取键盘输入数值，学习模式输入标准值*/
void InFLASH_Read(uint32_t addr, uint32_t *buf, uint32_t len);  /* Flash读取数据 */
void InFLASH_Write(uint32_t addr, uint32_t *buf, uint32_t len); /* Flash写入数据 */
uint8_t InFLASH_Read_Safe(uint32_t addr, uint32_t *buf, uint32_t len);/* 安全Flash读取 */
/* ============================================================== */
float User_AlignPhase(float pha, float ref);
float FindBestFrequency(float z_abs[], uint8_t gear_sweep[], float default_fre, uint8_t *best_idx);

/* ***************************** Main Part 主函数 ***************************** */
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
void User_main(void) {
	int i = 0;
	
	/* 初始化学习数据，从Flash读取校准数据 */
	Correct_init();
	
	InFLASH_Read(ADDR_FLASH_SECTOR_10, Proportion_Tmep, 1203);
	InFLASH_Read(ADDR_FLASH_SECTOR_11, Value_Tmep, 1200);
	
	/* 数据格式转换 */
	for(i=0; i<1203; i++) {
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
  * @param  location: 位置编号 1-20
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
  * @param  menu_sign: 目标菜单编号
  * @retval 无
  */
void Change_Menu( uint8_t menu_sign ) {
	uint8_t count;

	LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black ); /* 清空右侧区域 */

	for ( count = 1 ; count < MenuChoiceNum + 1 ; count ++ )
		OS_String_Show( 32, 32 + 64 * count, 32, 1, "○" ); /* 未选中圆圈 */

	if ( menu_sign > 0 && menu_sign <= MenuChoiceNum )
		OS_String_Show( 32, 32 + 64 * menu_sign, 32, 1, "●" ); /* 显示选中实心圆 */
	else
		menu_sign = 0;

	Ps2KeyValue = KeyValue_Null; /* 清空按键值 */
	MenuSign = menu_sign;         /* 更新当前菜单 */
}

/* ***************************** Menu Handler Part 菜单处理函数 ***************************** */
/**
  * @brief  菜单1 定频测量模式
  * 原理：固定频率点的测量，通过远近点选择，学习模式校准
  * @param  无
  * @retval 无
  */
void MenuHaddler_1(void)
{
    uint8_t i;
    float Qinductance;          // 预估电感品质因数
    uint16_t Gear_BH_flag = 0;  // 档位切换标志 0=未切换 1=已切换
    float gear_shifting_frequency;  // 自动切换频率
    uint8_t element_type;       // 元件类型标志 1=电阻，2=电容，3=电感
    uint8_t Gear_sweep[5] = {0};

    /* 初始化5个扫频点 单位Hz */
    Fre_Sweep[0] = 100;    	// 100Hz
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
        /************************ 一次5点扫频预处理，每个频率1次FFT ************************/
		uint8_t si;
		uint8_t best_gear = Gear_sign;
		for (si = 0; si < 5; si++)
		{
			setDDS(2.0, Fre_Sweep[si], 50, SINWAVE);  // 设置对应频率
			OSTimeDly(5);                                 // 信号稳定
			change_resistance_gear();
			Get_FFTQuick(ddsStructData.hz);              // 快速FFT分析
			
			// 存储当前频率的测量数据
			Pha_Sweep[si]  = ShowPha;
			ADC1_Sweep[si] = ADC1VOL;
			ADC2_Sweep[si] = ADC2VOL;
			Gear_sweep[si] = Gear_sign;
			Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[si]);
			Zabs_Sweep[si] = Z_abs;
		}

        /************************ 计算检测元件类型 ************************/
        element_type = 0;                  // 初始化元件类型
        gear_shifting_frequency = Fre_Sweep[1];  // 默认切换频率500Hz

        // 判断短路/开路状态，通过ADC电压判断
        // ADC1 = 待测元件电压，ADC2 = 标准电阻两端电压
        // 短路时待测元件电压接近0，标准电阻接近满量程
        // 开路时标准电阻为0，待测元件接近满量程电压
        if (ADC2_Sweep[1] > 0.59f && ADC1_Sweep[1] < 0.005f && Z_abs < 0.1f)
        {
            // 短路状态，标准电阻满量程，判断为短路
            element_type = 0;
        }
        // 开路状态，标准电阻为0，待测元件满量程
        else if ((ADC2_Sweep[1] < 0.03f && ADC1_Sweep[1] > 1.76f))
        {
            element_type = 0;  // 开路状态，无效元件
        }
        // 判断为电感，相位为正
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
        // 判断为电容，相位为负
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

        /************************ 确定测量频率，自动切换档位确认 ************************/
        setDDS(2.0, gear_shifting_frequency, 50, SINWAVE);  // 设置最佳测量频率
        OSTimeDly(5);                                          // 信号稳定

        Gear_sign = best_gear;
        SetGear(Gear_sign);
        OSTimeDly(5);

        // 执行自动切换
        Gear_BH_flag = change_resistance_gear();
        // 档位切换后重新扫频
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

        /************************ 同步元件类型，计算最终参数值 ************************/
        // 短路/开路显示
        if (ADC2VOL > 0.585f && ADC1VOL < 0.01f)
        {
			OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模: %0.3fΩ          ");
            OS_Num_Show(270, 240, 32, 1, 1, "短路状态                       ");
            OS_Num_Show(270, 280, 32, 1, 1, "                               ");
            OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
            OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
        }
        else if (ADC2VOL < 0.1f && ADC1VOL > 1.755f)
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

                // 电感阻抗大小选择计算公式
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
                // 固定500Hz测量电阻
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
  * 原理：5个频率扫频，识别串并联，计算谐振频率
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
	float w1, w2;
	float ResonantFre;
	float Pha5k;
	float ADC1_5k;
	float ADC2_5k;
	float Zabs5k;

	Fre_Sweep[0] = 100;
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
		change_resistance_gear();

		Get_FFTInformation(5000, MENU_NETWORK);
		Pha5k = ShowPha;
		ADC1_5k = ADC1VOL;
		ADC2_5k = ADC2VOL;
		Get_ZabsWithType(ADC1VOL, ADC2VOL, 5000, 0);
		Zabs5k = Z_abs;

		OS_Num_Show(270, 160, 32, 1, ADC1_5k, "ADC1VOL:%.3f v                       ");
		OS_Num_Show(550, 160, 32, 1, ADC2_5k, "ADC2VOL:%.3f v                               ");
		OS_Num_Show(550, 200, 32, 1, Pha5k, "相位:%.3f°                               ");
		if (Zabs5k < 0.10f) {
			OS_String_Show(270, 80, 32, 1, "网络类型--         ");
			OS_String_Show(270, 120, 32, 1, "谐振频率 : --         ");
			OS_Num_Show(270, 200, 32, 1, 1, "短路状态                       ");
			OS_Num_Show(270, 240, 32, 1, 1, "                               ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
		} else if (ADC2_5k < 0.15f && ADC1_5k > 1.75f) {
			OS_String_Show(270, 80, 32, 1, "网络类型--         ");
			OS_String_Show(270, 120, 32, 1, "谐振频率 : --         ");
			OS_Num_Show(270, 200, 32, 1, 1, "开路状态                       ");
			OS_Num_Show(270, 240, 32, 1, 1, "                               ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
		} else if (Zabs5k > 300000.0f && fabs(Pha5k) < 12.0f) {
			OS_String_Show(270, 80, 32, 1, "未知网络");
			OS_String_Show(270, 120, 32, 1, "谐振频率 : N/A         ");
			OS_String_Show(270, 200, 32, 1, "阻抗值超出测量范围");
			OS_Num_Show(270, 240, 32, 1, Zabs5k, "|Z|: %.1fΩ                    ");
			OS_Num_Show(270, 280, 32, 1, Pha5k, "相位: %.2f°                    ");
			OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
		} else {
			for (i = 0; i < 5; i++) {
				setDDS(2.0, Fre_Sweep[i], 50, SINWAVE);
				OSTimeDly(10);
				Get_FFTInformation(Fre_Sweep[i], MENU_NETWORK);

				Pha_Sweep[i] = ShowPha;
				ADC1_Sweep[i] = ADC1VOL;
				ADC2_Sweep[i] = ADC2VOL;
				Get_ZabsWithType(ADC1VOL, ADC2VOL, Fre_Sweep[i], 0);
				Zabs_Sweep[i] = Z_abs;

				switch(i) {
					case 0: Pha_Sweep[i] += PHA_COMPENSATE_100HZ; break;
					case 1: Pha_Sweep[i] += PHA_COMPENSATE_500HZ; break;
					case 2: Pha_Sweep[i] += PHA_COMPENSATE_5KHZ; break;
					case 3: Pha_Sweep[i] += PHA_COMPENSATE_20KHZ; break;
					case 4: Pha_Sweep[i] += PHA_COMPENSATE_50KHZ; break;
					default: break;
				}
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
				C_abs = CalibrateCapacitance(C_abs);
				Correct(2);
			} else if ( network == RC_P ) {
				Z_abs = CalculateRC_PResistance(Zabs_Sweep[idx_rc], Pha_Sweep[idx_rc]);
				C_abs = CalculateRC_PCapacitance(Pha_Sweep[idx_rc], Z_abs, Fre_Sweep[idx_rc]);
				C_abs = CalibrateCapacitance(C_abs);
				Correct(2);
			} else if ( network == RL_S ) {
				Z_abs = CalculateRL_SResistance(Zabs_Sweep[idx_rl], Pha_Sweep[idx_rl]);
				L_abs = CalculateRL_SInductance(Zabs_Sweep[idx_rl], Pha_Sweep[idx_rl], Fre_Sweep[idx_rl]);
				Z_abs = CalibrateResistance(Z_abs);
				Correct(1);
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
				Correct(1);
			} else if ( network == LC_S ) {
				w1 = 2 * PI * Fre_Sweep[idx_lc_low];
				w2 = 2 * PI * Fre_Sweep[idx_lc_high];
				C_abs = fabs(CalculateLC_SC(w1, w2, Pha_Sweep[idx_lc_low], Zabs_Sweep[idx_lc_low], Pha_Sweep[idx_lc_high], Zabs_Sweep[idx_lc_high]));
				L_abs = fabs(CalculateLC_SL(w1, w2, Pha_Sweep[idx_lc_low], Zabs_Sweep[idx_lc_low], Pha_Sweep[idx_lc_high], Zabs_Sweep[idx_lc_high]));
				C_abs = CalibrateCapacitance(C_abs);
				L_abs = CalibrateInductance(L_abs);
			} else if ( network == LC_P ) {
				w1 = 2 * PI * Fre_Sweep[idx_lc_low];
				w2 = 2 * PI * Fre_Sweep[idx_lc_high];
				C_abs = CalculateLC_PCPart1(w1, w2, Zabs_Sweep[idx_lc_low], Zabs_Sweep[idx_lc_high]);
				L_abs = CalculateLC_PLPart1(w1, w2, Zabs_Sweep[4], Zabs_Sweep[2]);
				if (L_abs * pow(10, 6) > 1000.0f) {
					w1 = 2 * PI * Fre_Sweep[idx_lc_low];
					w2 = 2 * PI * Fre_Sweep[4];
					C_abs = CalculateLC_PCPart2(w1, w2, Zabs_Sweep[idx_lc_low], Zabs_Sweep[4]);
					L_abs = CalculateLC_PLPart2(w1, w2, Zabs_Sweep[4], Zabs_Sweep[0]);
				}
				C_abs = fabs(C_abs);
				L_abs = fabs(L_abs);
				C_abs = CalibrateCapacitance(C_abs);
				L_abs = CalibrateInductance(L_abs);
			}

			if (network == Nw_Null) {
				OS_String_Show(270, 80, 32, 1, "网络类型: 未知     ");
			} else {
				OS_Num_Show(270, 80, 32, 1, network, "网络类型:          ");
			}
			OS_Num_Show(270, 200, 32, 1, 1, "                       ");

			if (network == LC_S || network == LC_P) {
				Auto_ShiftGear(Zabs5k);
				ResonantFre = 0.0f;
				if (L_abs > 0.0f && C_abs > 0.0f) {
					ResonantFre = CalculateResonantFrequency(L_abs, C_abs);
				}
				if (ResonantFre > 0.0f && ResonantFre < 100000.0f) {
					OS_Num_Show(270, 120, 32, 1, ResonantFre, "谐振频率 : %0.1fHz    ");
				} else {
					OS_String_Show(270, 120, 32, 1, "谐振频率 : N/A         ");
				}
			} else {
				OS_String_Show(270, 120, 32, 1, "谐振频率 : N/A         ");
			}

			if ( network == RC_S ) {
				OS_String_Show(420, 80, 32, 1, "RC串联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ                    ");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "C: %.3fμF                    ");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "C: %.3fpF                    ");
				} else {
					OS_Num_Show(270, 280, 32, 1, C_abs, "C: %.3fnF                    ");
				}
			} else if ( network == RC_P ) {
				OS_String_Show(420, 80, 32, 1, "RC并联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ                    ");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "C: %.3fμF                    ");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "C: %.3fpF                    ");
				} else {
					OS_Num_Show(270, 280, 32, 1, C_abs, "C: %.3fnF                    ");
				}
			} else if ( network == RL_S ) {
				OS_String_Show(420, 80, 32, 1, "RL串联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ                    ");
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH                    ");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fμH                    ");
				}
			} else if ( network == RL_P ) {
				OS_String_Show(420, 80, 32, 1, "RL并联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ                    ");
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH                    ");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fμH                    ");
				}
			} else if ( network == LC_S ) {
				OS_String_Show(420, 80, 32, 1, "LC串联");
				if (C_abs > 1.0e-7f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e6f, "C: %.3fμF                    ");
				} else if (C_abs < 1.0e-10f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e12f, "C: %.3fpF                    ");
				} else {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e9f, "C: %.3fnF                    ");
				}
				if (L_abs > 1.0e-4f) {
					OS_Num_Show(270, 280, 32, 1, L_abs * 1.0e3f, "L: %.3fmH                    ");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs * 1.0e6f, "L: %.3fμH                    ");
				}
			} else if ( network == LC_P ) {
				OS_String_Show(420, 80, 32, 1, "LC并联");
				if (C_abs > 1.0e-7f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e6f, "C: %.3fμF                    ");
				} else if (C_abs < 1.0e-10f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e12f, "C: %.3fpF                    ");
				} else {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1.0e9f, "C: %.3fnF                    ");
				}
				if (L_abs > 1.0e-4f) {
					OS_Num_Show(270, 280, 32, 1, L_abs * 1.0e3f, "L: %.3fmH                    ");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs * 1.0e6f, "L: %.3fμH                    ");
				}
			} else {
				OS_String_Show(270, 80, 32, 1, "未知网络");
			}
		}
		OSTimeDly(100);
	}
	Change_Menu( 0 );
}

/**
  * @brief  菜单3 原始数据模式
  * 原理：手动选择频率/档位，显示校准前原始数据
  * @param  无
  * @retval 无
  */
void MenuHaddler_3() {
	uint8_t targetGear;
	float current_fre = 500.0f; /* 当前频率，默认500Hz */

	/* 初始化 */
	setDDS(2.0, current_fre, 50, SINWAVE);
	OSTimeDly(10);
	Gear_sign = Gear_820;
	SetGear(Gear_sign);
	Ps2KeyValue = KeyValue_Null;

	while ( Ps2KeyValue != KeyValue_Back ) {
		/* 设置当前测量频率 */
		setDDS(2.0, current_fre, 50, SINWAVE);
		OSTimeDly(10);

		/* 手动切换档位和频率 */
		switch (Ps2KeyValue) {
			/* 档位切换 1-4 */
			case KeyValue_1: Gear_sign = Gear_47; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_2: Gear_sign = Gear_820; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_3: Gear_sign = Gear_15k; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_4: Gear_sign = Gear_270k; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			
			/* 固定频率 5-9 */
			case KeyValue_5: current_fre = 100;   Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_6: current_fre = 500;   Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_7: current_fre = 5000;  Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_8: current_fre = 20000; Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_9: current_fre = 50000; Ps2KeyValue = KeyValue_Null; break;
			
			/* 频率加减 */
			case KeyValue_Add: 
				current_fre += 10000; /* +10kHz */
				if(current_fre > 200000) current_fre = 200000; /* 上限200kHz */
				Ps2KeyValue = KeyValue_Null; 
				break;
				
			case KeyValue_Minus: 
				current_fre -= 10000; /* -10kHz */
				if(current_fre < 100) current_fre = 100; /* 下限100Hz */
				Ps2KeyValue = KeyValue_Null; 
				break;
				
			default: break;
		}
		
		/* 显示当前频率 */
		OS_Num_Show(270, 360, 32, 1, current_fre, "当前频率:%0.2fHz                         ");

		Display_flag = 0;
		/* 自动切换 */
		while (Display_flag == 0) {
			targetGear = Gear_sign;
			Get_FFTInformation(current_fre, MENU_ORIGINAL);
			/************************ 显示原始数据 ************************/
			OS_Num_Show(270, 80, 32, 1, ADC1VOL, "ADC1VOL: %0.3f       ");
			OS_Num_Show(270, 120, 32, 1, ADC2VOL, "ADC2VOL: %0.3f       ");
			OS_Num_Show(270, 160, 32, 1, ShowPha, "相位值: %0.3f°       ");

			if (ADC2VOL < 0.10f) {
				/* 电压过小，升档 */
				targetGear = GetHigherGear(Gear_sign);
			}
			else if (ADC2VOL > (ADC1VOL * 10) || ADC2VOL > 3.0f) {
				/* 电压过大，降档 */
				targetGear = GetLowerGear(Gear_sign);
			} else {
				Get_Zabs(ADC1VOL, ADC2VOL, current_fre);
				targetGear = GetBoundaryGear(Z_abs, Gear_sign);
			}

			if (targetGear != Gear_sign) {
				Gear_sign = targetGear;
				SetGear(Gear_sign);
			} else {
				Display_flag = 1;
			}
			OSTimeDly(10);
		}

		/* 计算并显示阻抗 */
		Get_FFTInformation(current_fre, MENU_ORIGINAL);
		/************************ 阻抗计算显示 ************************/
		OS_Num_Show(270, 80, 32, 1, ADC1VOL, "ADC1VOL: %0.3f       ");
		OS_Num_Show(270, 120, 32, 1, ADC2VOL, "ADC2VOL: %0.3f       ");
		OS_Num_Show(270, 160, 32, 1, ShowPha, "相位值: %0.3f°       ");

        /* 1. 未校准原始阻抗 */
        float Rref = Rref_Table[(Gear_sign < Gear_Count) ? Gear_sign : 1];
        if(ADC2VOL > 0.005f) {
            float Z_abs_raw = Rref * ADC1VOL / ADC2VOL;
            OS_Num_Show(270,220,32,1,Z_abs_raw,"未校准阻抗:%.3f欧                          ");
        } else {
            OS_String_Show(270,220,32,1,"未校准阻抗: 短路          ");
        }

        // 2. 校准后阻抗
        Get_Zabs(ADC1VOL,ADC2VOL,current_fre);
        OS_Num_Show(270,260,32,1,Z_abs,"校准后阻抗:%.3f欧                          ");
	}
	
	Change_Menu( 0 ); /* 返回主菜单 */
}

/* ==================== 菜单4 学习模式校准功能 ==================== */
/**
  * @brief  学习模式，输入标准值修正测量值
  * @param  无
  * @retval 无
  */
void MenuHaddler_4()
{
	int i = 0;
    uint8_t enter_input_flag = 0; /* 输入模式标志 */
    float input_data = 0.0f;      /* 输入标准值 */
    char *element_name = "UNKNOWN";   /* 元件类型名称 */

	/* 从Flash读取学习数据 */
    Show_Val(7,0,"   Loading data...            ");
	InFLASH_Read(ADDR_FLASH_SECTOR_10,Proportion_Tmep,1203);
	InFLASH_Read(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
    Show_Val(7,0,"   Load done                 ");

	/* 格式转换 */
	for(i=0;i<1203;i++)
	{
		Proportion[i]=*(float *)&Proportion_Tmep[i];
		if(i<1200)
		{
			Value[i]=*(float *)&Value_Tmep[i];
		}
	}
	
	/* 获取存储指针 */
	Storage_Bit_z=Proportion_Tmep[0];
	Storage_Bit_c=Proportion_Tmep[1];
	Storage_Bit_l=Proportion_Tmep[2];
	
	Storage_mode=Show_flag; /* 获取当前元件类型 */
	Ps2KeyValue = KeyValue_Null; /* 清空按键 */

    /* 清空右侧区域 */
    LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black );

    /************************ 元件类型确认 ************************/
    switch(Storage_mode) {
        case 1: element_name = "RES"; break;
        case 2: element_name = "CAP"; break;
        case 3: element_name = "IND"; break;
        default: element_name = "UNKNOWN"; break;
    }

    // 无效学习提示
    if(Storage_mode < 1 || Storage_mode > 3) {
        Show_Val(1,0,"Learn failed.");
        Show_Val(2,0,"Measure part first.");
        Show_Val(3,0,"Type is unknown.");
        Show_Val(7,0,"Press Back.");
        while(Ps2KeyValue != KeyValue_Back) {
            OSTimeDly(10);
        }
        Ps2KeyValue = KeyValue_Null;
        Change_Menu(0);
        return;
    }

    // 有效学习提示
    Show_Val(1,0,"Learning item:");
    Show_Val(2,0,element_name);
    Show_Val(7,0,"Input ref value. Back=exit");

	/* 主循环 */
	while(1)
	{	
        /* 返回按键 */
        if(Ps2KeyValue == KeyValue_Back)
        {
            Ps2KeyValue = KeyValue_Null;
            break;
        }

        /* 按下数字键进入输入模式 */
        if(Ps2KeyValue >= KeyValue_0 && Ps2KeyValue <= KeyValue_9 && enter_input_flag == 0)
        {
            enter_input_flag = 1;
        }

        /* 输入模式处理 */
        if(enter_input_flag == 1)
        {
            Show_Val(7,0,"Input ref value.");
            input_data = PS2_ReadNum(0); /* 获取键盘输入数值 */
            enter_input_flag = 0;
        }

        /* 输入有效值执行校准 */
        if(input_data > 0)
        {
            switch(Storage_mode)
            {
                case 1: /* 电阻校准 */
                    Show_Val(3,Storage_Bit_z,"Cal #%0.0f                  ");
                    Show_Val(4,Z_abs,"Measured: %.3f ohm         ");

                    if(Storage_Bit_z < 400)
                    {
                        /* 电阻校准系数 */
                        Proportion[Storage_Bit_z + 3] = input_data / Z_abs;
                        Proportion_Tmep[Storage_Bit_z + 3] = *(uint32_t *)&Proportion[Storage_Bit_z + 3];
                        /* 电阻原始值 */
                        Value[Storage_Bit_z] = Z_abs;
                        Value_Tmep[Storage_Bit_z] = *(uint32_t *)&Z_abs;
                        /* 指针自增 */
                        Storage_Bit_z++;
                        Proportion_Tmep[0] = Storage_Bit_z;
                        
                        /* 写入Flash */
                        Show_Val(7,0,"   Saving...                 ");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
                        OSTimeDly(200);
                        Show_Val(7,0,"   Saved. Press Back.       ");
                    }
                    else
                    {
                        Show_Val(7,0,"   Storage full (400).      ");
                    }
                    break;
                    
                case 2: /* 电容校准 */
                    Show_Val(3,Storage_Bit_c,"Cal #%0.0f                  ");
                    Show_Val(4,C_abs,"Measured: %.6f nF          ");

                    if(Storage_Bit_c < 400)
                    {
                        Proportion[Storage_Bit_c+3+400] = input_data / C_abs;
                        Proportion_Tmep[Storage_Bit_c+3+400] = *(uint32_t *)&Proportion[Storage_Bit_c+3+400];
                        Value[Storage_Bit_c+400] = C_abs;
                        Value_Tmep[Storage_Bit_c+400] = *(uint32_t *)&C_abs;
                        Storage_Bit_c++;
                        Proportion_Tmep[1] = Storage_Bit_c;
                        
                        Show_Val(7,0,"   Saving...                ");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
                        OSTimeDly(200);
                        Show_Val(7,0,"   Saved. Press Back.      ");
                    }
                    else
                    {
                        Show_Val(7,0,"   Storage full (400).     ");
                    }
                    break;
                    
                case 3: /* 电感 */
                    Show_Val(3,Storage_Bit_l,"Cal #%0.0f                  ");
                    Show_Val(4,L_abs,"Measured: %.6f uH          ");

                    if(Storage_Bit_l < 400)
                    {
                        Proportion[Storage_Bit_l+3+800] = input_data / L_abs;
                        Proportion_Tmep[Storage_Bit_l+3+800] = *(uint32_t *)&Proportion[Storage_Bit_l+3+800];
                        Value[Storage_Bit_l+800] = L_abs;
                        Value_Tmep[Storage_Bit_l+800] = *(uint32_t *)&L_abs;
                        Storage_Bit_l++;
                        Proportion_Tmep[2] = Storage_Bit_l;
                        
                        Show_Val(7,0,"   Saving...                ");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
                        OSTimeDly(200);
                        Show_Val(7,0,"   Saved. Press Back.      ");
                    }
                    else
                    {
                        Show_Val(7,0,"   Storage full (400).     ");
                    }
                    break;
                    
                default:
                    Show_Val(2,0,"        Cal failed!!!            ");
                    break;
            }
            input_data = 0.0f; /* 清空输入 */
        }

        OSTimeDly(10);
	}
	
	/* 返回主菜单 */
    LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black );
	Change_Menu( 0 );
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
    Show_Val(1,0,"Clear all learning data?");
    Show_Val(2,0,"Enter=yes  Back=no");

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
        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
        OSTimeDly(200);
        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
        OSTimeDly(200);
        Show_Val(1,0,"         Clear done         ");
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
	if(fabs(sin_pha) < 0.001f) sin_pha = 0.001f;
	float c = 1 / (2 * PI * fre * zAbs) * pow(10, 9);
	return c * 1.0f;
}

float CalculateCapacitanceSmallRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if(fabs(sin_pha) < 0.001f) sin_pha = 0.001f;
	float c = 1 / (2 * PI * fre * zAbs) * pow(10, 9);
	return c * 1.0f;
}

float CalculateCapacitanceLargeRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if(fabs(sin_pha) < 0.001f) sin_pha = 0.001f;
	float c = 1 / (2 * PI * fre * zAbs) * pow(10, 9);
	return c * 1.0f;
}

/**
  * @brief  电容校准
  * @param  c: 原始测量值 nF
  * @retval 校准后电容值 nF
  */
float CalibrateCapacitance(float c) {
    return c;
}

/* ------------------------------ RC串并联计算 ------------------------------ */
float CalculateRC_SResistance(float zAbs, float pha) {
	float r = zAbs * cos(pha / 180 * PI);
	return r * 0.846f;
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

/* ------------------------------ RL串并联计算 ------------------------------ */
float CalculateRL_SResistance(float zAbs, float pha) {
	return zAbs * cos(pha / 180 * PI);
}

float CalculateRL_SInductance(float zAbs, float pha, float fre) {
	return (zAbs * sin(pha / 180 * PI)) / (2 * PI * fre) * pow(10, 6);
}

/**
  * @brief  排序数组取均值滤波使用
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
  * @brief  RL并联计算
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
  * @brief  RL并联计算
  */
float CalculateRL_PInductancePart2(float r, float fre, float pha) {
    float tan_pha = tan(pha * PI / 180.0f);
    if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;
    if(fre < 10.0f) fre = 10.0f;
	return r / (2 * PI * fre * tan_pha) * 1e6f;
}

/* ------------------------------ LC串并联计算 ------------------------------ */
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
  * @brief  计算阻抗选档位
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
  * @brief  自动阻抗切换
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

		/* FFT分析 */
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
  * @brief  设置阻抗档位
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
  * @brief  获取FFT分析信息
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

    /************************ 计算频率状态动态调整，多次FFT ************************/
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
        /* 动态调整采样率 */
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
  * @brief  快速FFT分析，用于扫频
  * @param  FreSet: 目标频率
  * @retval 无
  */
void Get_FFTQuick(float FreSet)
{
    float FreTemp = FreSet;

    /* 动态调整采样率 */
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

    /* 单次FFT */
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
  * @brief  计算电路识别
  * @retval 电路类型编号
  */
uint8_t Get_Network(void) {
	return User_GetNetwork(Zabs_Sweep, Pha_Sweep);
}

/**
  * @brief  计算扫频数据检测主要元件类型
  * @param  pha_dif[]: 相位数组
  * @param  z_abs[]: 阻抗数组
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
  * @brief  精确识别算法
  * @param  z_abs[]: 阻抗数组
  * @param  pha_dif[]: 相位数组
  * @retval 电路类型编号
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

    edge_phase_delta = pha_dif[4] - pha_dif[0];

    if(pha_max < -8.0f || (Show_flag == 2 && pha_max < 8.0f && pha_min < -2.0f)) {
        if(pha_fall_cnt > pha_rise_cnt || edge_phase_delta < -4.0f) {
            network = RC_P;
        } else {
            network = RC_S;
        }
        return network;
    }

    if(pha_min > 8.0f || (Show_flag == 3 && pha_min > -8.0f && pha_max > 2.0f)) {
        if(pha_rise_cnt > pha_fall_cnt || edge_phase_delta > 4.0f) {
            network = RL_S;
        } else {
            network = RL_P;
        }
        return network;
    }

    if(Show_flag == 2 && z_fall_cnt >= 2) {
        if(pha_fall_cnt > pha_rise_cnt || edge_phase_delta < 0.0f) {
            return RC_P;
        }
        return RC_S;
    }

    if(Show_flag == 3 && z_rise_cnt >= 2) {
        if(pha_rise_cnt > pha_fall_cnt || edge_phase_delta > 0.0f) {
            return RL_S;
        }
        return RL_P;
    }

	return Nw_Null;
}

/* ==================== 学习模式校准功能 ==================== */
/**
  * @brief  数值校准
  * @param  mode: 元件类型
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
		case 3: /* 电感 */
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
  * @brief  初始化学习数据
  * @param  无
  * @retval 无
  */
void Correct_init(void) {
	int i;
	for(i=0;i<1203;i++) {
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
  * @brief  获取键盘输入的标准值
  * @param  num: 默认值
  * @retval 输入标准值
  */
float PS2_ReadNum(float num) {
	uint8_t count = 0;
	uint8_t dec_sign = 0;      /* 小数点标志 */
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
		
		/* 按键输入 */
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
  * @param  buf: 数据缓冲区
  * @param  len: 长度
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

float User_AlignPhase(float pha, float ref)
{
	while ((pha - ref) > 180.0f)
	{
		pha -= 360.0f;
	}

	while ((pha - ref) < -180.0f)
	{
		pha += 360.0f;
	}

	return pha;
}

/**
  * @brief  Find the best sweep frequency where |Z| is closest to Rref
  * @param  z_abs[]: impedance array from 5-point sweep
  * @param  default_fre: fallback frequency
  * @param  best_idx: output index (can be NULL)
  * @retval best frequency
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