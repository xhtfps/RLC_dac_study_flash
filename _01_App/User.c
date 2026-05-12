/****************************
 * Project description: 阻抗测量与元件参数分析项目
 * 功能：通过ADC采样、FFT分析测量阻抗，自动识别RC/RL/LC网络结构并计算元件参数
 * Author: 创新基地 -> 2026 Mao
 * Creation Date: 2026/04/24
 * Update date: 2026/05/04 (修复版)
 * 1-> 加入DAC驱动替代DDS
 * 2-> 新增学习模式与查表修正
 * 3-> 修复计算错误、逻辑缺陷与鲁棒性问题
 * ****************************/

/* ***************************** Include & Define Part     	***************************** */
#include "User.h"

/* ***************************** 宏定义区（可根据硬件修改） ***************************** */
#define MAX_SAMPLE_RATE  512000			/* 最大采样率 512kHz */
#define Sweep_Fre_Buff_Length    300     /* 扫频数据缓冲区长度 */

/* 菜单模式定义 C89标准 */
#define MENU_IMPEDANCE  1    /* 菜单1：阻抗模 + 阻抗角 */
#define MENU_NETWORK    2    /* 菜单2：网络结构 + 谐振点 */
#define MENU_ORIGINAL   3    /* 菜单3：原始显示(ADC1/ADC2/相位) */

/* 4个基准电阻值 */
/* 基准电阻值由 Rref_Table[] 数组统一管理，见 Gear_Calibration 定义处 */

/* 按相邻基准电阻几何平均值设置边界，并加入约20%滞回防止反复换挡 */
#define GEAR_BOUND_47_820_DOWN      157.0f
#define GEAR_BOUND_47_820_UP        235.0f
#define GEAR_BOUND_820_15K_DOWN     2800.0f
#define GEAR_BOUND_820_15K_UP       4200.0f
#define GEAR_BOUND_15K_270K_DOWN    50900.0f
#define GEAR_BOUND_15K_270K_UP      76400.0f

/* 最大阻抗不再限制，允许测量开路 */

/* 高频相位补偿系数（和扫频点一一对应：100Hz/500Hz/5kHz/20kHz/100kHz） */
#define PHA_COMPENSATE_100HZ    0.0f
#define PHA_COMPENSATE_500HZ    0.0f
#define PHA_COMPENSATE_5KHZ     0.0f
#define PHA_COMPENSATE_20KHZ    0.0f
#define PHA_COMPENSATE_100KHZ   5.0f

/* 电阻档位枚举（方便代码阅读） */
enum User_Gear {
	Gear_47 = 0,    /* 47Ω基准电阻 */
	Gear_820,      	/* 820Ω基准电阻 */
	Gear_15k,      	/* 15kΩ基准电阻 */
	Gear_270k,      /* 270kΩ基准电阻 */
	Gear_Count = 4  /* 档位总数 */
};

/* 基准电阻值数组（和枚举一一对应） */
const float Rref_Table[Gear_Count] = { 47.0f, 820.0f, 15000.0f, 270000.0f };

/* ==================== 阻抗校准系数表 ==================== */
/*
 * 每个频率 + 每个档位都有独立的硬件校准系数
 * 行 = 档位（Gear_47/Gear_820/Gear_15k/Gear_270k）
 * 列 = 频率索引（Fre_Sweep[0]~[4] = 100Hz/500Hz/5kHz/20kHz/100kHz）
 *
 * 使用方法：Z = Rref[gear] × V1 / V2 / Calibration[gear][freq_idx]
 * 调试时只需修改此表中的数值，无需改动任何函数逻辑
 *
 * 初始值说明：
 *   原代码中 1kHz 系数为 2.71267，其他频率为 2.8677
 *   这里按频率分配初始值，请根据实测标准电阻逐个校准
 */
float Gear_Calibration[Gear_Count][5] = {
    /*  100Hz     500Hz      5kHz      20kHz     100kHz  */
    {  2.7127f,  2.7127f,   2.9677f,   3.0477f,   2.8280f  },  /* Gear_47   (47Ω)   */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_820  (820Ω)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_15k  (15kΩ)  */
    {  2.7127f,  2.7127f,   2.8677f,   2.8677f,   2.8677f  },  /* Gear_270k (270kΩ) */
};

/* ==================== 双通道零点失调校准值（实测得到） ==================== */
//#define VOS_ADC1  0.025f  /* 通道1失调电压：短接被测端测得的ADC1基波幅值 */
//#define VOS_ADC2  0.018f  /* 通道2失调电压：断路被测端测得的ADC2基波幅值 */

uint8_t Gear_sign = Gear_820; /* 当前档位标志，默认820Ω */

/* ***************************** 全局变量区 ***************************** */
/* 外部变量（来自其他文件） */
extern uint32_t ADCData[];      /* ADC采样数据缓冲区 */
extern DDSDataStruct dds[2];     /* DDS波形数据结构 */
extern float ADCfre;              /* ADC采样频率 */
extern float ADCfre1;
extern float ADC1VOL;            /* 通道1电压（被测件） */
extern float ADC2VOL;            /* 通道2电压（基准电阻） */
extern float pha;                 /* 原始相位值 */

/* 菜单与显示控制 */
uint8_t MenuSign = 0;            /* 当前菜单编号 */
unsigned char mode = 0 ;          /* 工作模式 */

float Sample_rate = MAX_SAMPLE_RATE; /* 当前采样率 */
float PhaArrary[10] = {0};			  /* 10次相位采样数组（用于滤波） */
float ADC1VolArrary[10] = {0};		  /* 10次通道1电压采样数组 */
float ADC2VolArrary[10] = {0};		  /* 10次通道2电压采样数组 */
float ShowPha;						  /* 滤波后的最终相位值 */
GRAPH_Struct GridData;                 /* 绘图网格数据 */
float Z_abs, L_abs, C_abs;             /* 计算结果：阻抗、电感、电容 */
uint8_t Display_flag = 0;               /* 显示完成标志 */

/* 扫频测量数据（5个频率点：100Hz, 500Hz, 5kHz, 20kHz, 100kHz） */
float Pha_Sweep[5] = {0};               /* 5个点的相位 */
float ADC1_Sweep[5] = {0};              /* 5个点的通道1电压 */
float ADC2_Sweep[5] = {0};              /* 5个点的通道2电压 */
float Fre_Sweep[5] = {100, 500, 5000, 20000, 100000}; /* 5个扫频频率 */
float Zabs_Sweep[5] = {0};              /* 5个点的阻抗 */
float z_r[5] = {0};                      /* 临时数组（用于排序） */

/* 网络类型枚举（自动识别结果） */
enum Network {
	Nw_Null = 0,  /* 未知网络 */
	RC_S,         /* RC串联 */
	RC_P,         /* RC并联 */
	RL_S,         /* RL串联 */
	RL_P,         /* RL并联 */
	LC_S,         /* LC串联 */
	LC_P,         /* LC并联 */
};

/* ==================== 学习模式新增变量（核心功能） ==================== */
uint16_t Show_flag = 0;        /* 元件类型标志：1=电阻，2=电容，3=电感 */
float last_z;                   /* 保存上一次阻抗值（用于换挡判断） */

/* 学习数据存储区（共1203个float，掉电保存到Flash） */
float Proportion[1203];        /* 修正比例数组：[0]=电阻个数, [1]=电容个数, [2]=电感个数, [3-402]=电阻数据, [403-802]=电容数据, [803-1202]=电感数据 */
uint32_t Proportion_Tmep[1203];/* 用于Flash存储的32位转换数组 */
float Value[1200];              /* 学习时的原始测量值 */
uint32_t Value_Tmep[1200];     /* 用于Flash存储的32位转换数组 */
uint32_t Storage_Bit_z = 0;     /* 电阻已学习数据个数（最多400个） */
uint32_t Storage_Bit_c = 0;     /* 电容已学习数据个数（最多400个） */
uint32_t Storage_Bit_l = 0;     /* 电感已学习数据个数（最多400个） */
uint32_t Storage_mode = 0;      /* 当前学习模式：1=学电阻，2=学电容，3=学电感 */
float data;                      /* 临时变量（用于键盘输入） */
/* ========================================================== */

/* ***************************** 函数声明区（按功能分类） ***************************** */
/* 基础功能函数 */
float User_FixPhase( float pha );        /* 相位修正（限制在-180°~180°） */
void GPIO_Change_Init();                  /* 继电器GPIO初始化（切换电阻档位） */
void SetGear(uint8_t Gear);               /* 设置电阻档位 */
void Get_FFTInformation(float FreSet, uint8_t MenuMode); /* 获取FFT分析结果（电压、相位） */
void Get_FFTQuick(float FreSet);                          /* 快速FFT：单次采样，用于扫频预测量（仅1次FFT） */
void Get_Zabs(float Get_ADC1, float Get_ADC2, float NowFre); /* 计算阻抗 */
uint8_t Get_Network(void);				  /* 简单网络识别 */
uint8_t User_GetNetwork( float z_abs[], float pha_dif[] );	  /* 复杂网络识别 */
uint8_t change_resistance_gear(void);		  /* 自动换挡逻辑，返回1=发生了换挡 */
uint8_t GetHigherGear(uint8_t Gear);      /* 获取相邻高阻值档位 */
uint8_t GetLowerGear(uint8_t Gear);       /* 获取相邻低阻值档位 */
uint8_t GetBoundaryGear(float zAbs, uint8_t Gear); /* 根据阻值边界选择档位 */

/* 阻抗计算已统一由 Get_Zabs() + Gear_Calibration[] 系数表处理，无需单独函数 */

/* 电感计算函数（分大小阻抗优化精度） */
float CalculateInductanceSmallZabs(float zAbs, float pha, float fre);
float CalculateInductanceLargeZabs(float zAbs, float pha, float fre);

/* 电容计算函数（分频段优化精度） */
float CalculateCapacitanceMidRange(float fre, float zAbs, float pha);
float CalculateCapacitanceSmallRange(float fre, float zAbs, float pha);
float CalculateCapacitanceLargeRange(float fre, float zAbs, float pha);
float CalibrateCapacitance(float c); /* 电容二次校准 */

/* RC串并联计算 */
float CalculateRC_SResistance(float zAbs, float pha);
float CalculateRC_SCapacitance(float zAbs, float pha, float fre);
float CalculateRC_PResistance(float zAbs, float pha);
float CalculateRC_PCapacitance(float pha, float r, float fre);

/* RL串并联计算 */
float CalculateRL_SResistance(float zAbs, float pha);
float CalculateRL_SInductance(float zAbs, float pha, float fre);
void SortArray(float arr[], uint8_t len); /* 数组排序（用于中值滤波） */
float CalculateRL_PResistancePart1(float zAbs, float pha);
float CalculateRL_PInductancePart1(float r, float fre, float pha);
float CalculateRL_PResistancePart2(float zAbs, float pha);
float CalculateRL_PInductancePart2(float r, float fre, float pha);

/* LC串并联计算 */
float CalculateLC_SC(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3);
float CalculateLC_SL(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3);
float CalculateResonantFrequency(float l, float c); /* 计算谐振频率 */
float CalculateLC_PCPart1(float w1, float w2, float zAbs1, float zAbs3);
float CalculateLC_PLPart1(float w1, float w2, float zAbs4, float zAbs2);
float CalculateLC_PCPart2(float w1, float w2, float zAbs1, float zAbs4);
float CalculateLC_PLPart2(float w1, float w2, float zAbs4, float zAbs0);

/* ==================== 学习模式核心函数声明 ==================== */
void Correct(uint8_t mode);       /* 修正函数：根据学习数据修正测量值 */
void Correct_init(void);           /* 初始化学习数据（清空） */
float PS2_ReadNum(float num);      /* 键盘输入数字（用于学习模式输入标准值） */
void InFLASH_Read(uint32_t addr, uint32_t *buf, uint32_t len);  /* 从Flash读取数据 */
void InFLASH_Write(uint32_t addr, uint32_t *buf, uint32_t len); /* 向Flash写入数据 */
uint8_t InFLASH_Read_Safe(uint32_t addr, uint32_t *buf, uint32_t len);/* 安全的Flash读取函数声明 */
/* ============================================================== */
float User_AlignPhase(float pha, float ref);

/* ***************************** Main Part（主程序入口） ***************************** */
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
void User_main(void) {
	int i = 0;
	
	/* 先清空学习数据，确保处于安全状态，防止Flash里的脏数据导致数组越界 */
	Correct_init();
	
	InFLASH_Read(ADDR_FLASH_SECTOR_10, Proportion_Tmep, 1203);
	InFLASH_Read(ADDR_FLASH_SECTOR_11, Value_Tmep, 1200);
	
	/* 浮点数据转换 */
	for(i=0; i<1203; i++) {
		Proportion[i] = *(float *)&Proportion_Tmep[i];
		if(i<1200) {
			Value[i] = *(float *)&Value_Tmep[i];
		}
	}
	
	/* 读取已存储的学习数据指针 */
	Storage_Bit_z = Proportion_Tmep[0];
	Storage_Bit_c = Proportion_Tmep[1];
	Storage_Bit_l = Proportion_Tmep[2];
	
	Init_All();    /* 初始化所有硬件：LCD、ADC、DAC、GPIO、定时器 */
	Disp_Main();   /* 显示主菜单界面 */

	while (1) {
		switch ( MenuSign ) {
			case 0:
				/* 主菜单：等待按键选择 */
				if ( Ps2KeyValue != KeyValue_Null ) 	
					Change_Menu( Ps2KeyValue );				
				break;
			case 1:
				MenuHaddler_1(); /* 菜单1：单频测量模式 */
				break;
			case 2:
				MenuHaddler_2(); /* 菜单2：扫频测量模式 */
				break;
			case 3:
				MenuHaddler_3(); /* 菜单3：自动测量模式 */
				break;
			case 4:
				MenuHaddler_4(); /* 菜单4：学习模式（核心功能） */
				break;
			case 5:
				MenuHaddler_5(); /* 菜单5：清除学习数据 */
				break;
			default:
				break;
		}
		delay_ms(10); /* 延时10ms，防止CPU占用率过高 */
	}
}

/* ***************************** Initialization Part（初始化函数） ***************************** */
/**
  * @brief  初始化所有硬件外设
  * @param  无
  * @retval 无
  */
void Init_All() {
	TFT_LCD_Init();     /* 初始化LCD */
	LCD_Clear(Black); /* 清屏，黑色背景 */

	/* 初始化ADC、DMA、GPIO（来自Drive_DMA_DSP_FFT.h） */
	User_GPIO_doubleInit();
	User_ADC_double_Init();
	User_DMA_doubleInit();
	ADC_TIM3_Init(MAX_SAMPLE_RATE);

	GPIO_Change_Init(); /* 初始化继电器控制GPIO（PC11/PC12） */
	DDSInit();					/* 初始化DDS（生成正弦波） */
	dacInit();          /* 初始化DAC（生成正弦波） */
	setDDS(2.0, 500, 50, SINWAVE); /* 设置初始波形：2Vpp，500Hz，50%占空比，正弦波 */
}

/**
  * @brief  显示主菜单界面
  * @param  无
  * @retval 无
  */
void Disp_Main() {
	uint8_t count;

	OS_String_Show( 300 - 32 * 2, 16, 32, 0, TitleStr ); /* 显示标题 */

	/* 清屏部分区域 */
	LCD_Appoint_Clear( 0, 64, 800, 64 + 8, White );
	LCD_Appoint_Clear( 0, 480 - 32 - 8, 800, 480 - 32, White );
	LCD_Appoint_Clear( 250, 64 + 8, 250 + 2, 480 - 32 - 8, White );

	/* 显示版本信息 */
	OS_String_Show( 32, 480 - 16 - 8, 16, 0, ModelVerStr );
	OS_String_Show( 632, 480 - 16 - 8, 16, 0, UserVerStr );

	/* 显示菜单选项（5个） */
	for ( count = 1 ; count < MenuChoiceNum + 1 ; count ++ )
		OS_String_Show( 32, 32 + 64 * count, 32, 0, "—" );
	for ( count = 0 ; count < MenuChoiceNum ; count ++ ) {
		switch ( count ) {
			case 0:
				OS_String_Show( 80, 96, 32, 0, Menu1Choice1 ); /* 单频测量 */
				break;
			case 1:
				OS_String_Show( 80, 96 + 64, 32, 0, Menu1Choice2 ); /* 扫频测量 */
				break;
			case 2:
				OS_String_Show( 80, 96 + 64 * 2, 32, 0, Menu1Choice3 ); /* 自动测量 */
				break;
			case 3:
				OS_String_Show( 80, 96 + 64 * 3, 32, 0, Menu1Choice4 ); /* 学习模式 */
				break;
			case 4:
				OS_String_Show( 80, 96 + 64 * 4, 32, 0, Menu1Choice5 ); /* 清除数据 */
				break;
			default:
				break;
		}
	}
}

/**
  * @brief  在指定位置显示数值
  * @param  location: 位置编号(1-20)
  * @param  value: 要显示的数值
  * @param  str: 格式化字符串
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

	LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black ); /* 清屏右侧区域 */

	for ( count = 1 ; count < MenuChoiceNum + 1 ; count ++ )
		OS_String_Show( 32, 32 + 64 * count, 32, 1, "—" ); /* 清除旧光标 */

	if ( menu_sign > 0 && menu_sign <= MenuChoiceNum )
		OS_String_Show( 32, 32 + 64 * menu_sign, 32, 1, "◆" ); /* 显示新光标 */
	else
		menu_sign = 0;

	Ps2KeyValue = KeyValue_Null; /* 清空按键值 */
	MenuSign = menu_sign;         /* 更新当前菜单 */
}

/* ***************************** Menu Handler Part（菜单处理函数） ***************************** */
/**
  * @brief  菜单1：单频测量模式（核心功能）
  * @功能：  测量单个频率下的阻抗，自动识别元件类型，调用学习模式修正
  * @param  无
  * @retval 无
  */
void MenuHaddler_1(void)
{
    uint8_t i;
    float Qinductance;          // 预留：电感品质因数（当前代码未使用）
    uint16_t Gear_BH_flag = 0;  // 档位变化标志：0=未变化，1=已变化（用于保存last_z）
    float gear_shifting_frequency;  // 用于自动换档的测试频率
    uint8_t element_type;       // 元件类型标志：1=电阻，2=电容，3=电感

    /* 初始化5个扫频频率点（单位：Hz） */
    Fre_Sweep[0] = 100;    // 100Hz
    Fre_Sweep[1] = 500;    // 500Hz
    Fre_Sweep[2] = 5000;   // 5kHz
    Fre_Sweep[3] = 20000;  // 20kHz
    Fre_Sweep[4] = 100000; // 100kHz
    Ps2KeyValue = KeyValue_Null;  // 按键值初始化为空

    /* 初始配置：500Hz正弦波，820Ω档位 */
    setDDS(2.0, 500, 50, SINWAVE);  // DDS输出：2.0V幅度，500Hz频率，50%占空比，正弦波
    Gear_sign = Gear_820;              // 初始档位：820Ω档
    SetGear(Gear_sign);                // 执行档位切换

    /* 主循环：直到按下返回键退出 */
    while (Ps2KeyValue != KeyValue_Back)
    {
        /************************ 第一步：5点扫频预测量（快速模式，每点仅1次FFT） ************************/
		uint8_t si;
		for (si = 0; si < 5; si++)
		{
			setDDS(2.0, Fre_Sweep[si], 50, SINWAVE);  // 切换到对应频率
			OSTimeDly(5);                                 // 等待信号稳定
			Get_FFTQuick(ddsStructData.hz);              // 快速FFT测量
			
			// 保存当前频率点的测量结果
			Pha_Sweep[si]  = ShowPha;
			ADC1_Sweep[si] = ADC1VOL;
			ADC2_Sweep[si] = ADC2VOL;
			Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[si]);
			Zabs_Sweep[si] = Z_abs;
		}

        /************************ 第二步：元件类型识别 ************************/
        element_type = 0;                  // 初始化元件类型
        gear_shifting_frequency = Fre_Sweep[1];  // 默认换档频率：500Hz

        // 判断短路或开路（直接根据ADC电压判断，不依赖Z值计算）
        // ADC1 = 待测元件电压，ADC2 = 基准电阻电压
        // 短路时：待测元件压降极小（ADC1≈0），基准电阻分到大部分电压（ADC2大）
        // 开路时：无电流流过，基准电阻无压降（ADC2≈0），待测端电压≈信号源电压（ADC1大）
        if (ADC2_Sweep[1] > 0.7f && ADC1_Sweep[1] < 0.01f)
        {
            // 短路：基准电压正常但待测电压接近0
            element_type = 0;
        }
        // 判断开路：基准电压极小（无电流），待测电压正常
        else if (ADC2_Sweep[1] < 0.005f && ADC1_Sweep[1] > 2.2f)
        {
            element_type = 0;  // 开路，不进行后续计算
        }
        // 判断电感：多频点综合判断，避免单频点噪声导致漏判
        else if (/* 多频点综合判断电感 */
                 ((Pha_Sweep[1] > 30.0f && Pha_Sweep[1] < 150.0f) ? 1 : 0) +
                 ((Pha_Sweep[2] > 60.0f && Pha_Sweep[2] < 120.0f) ? 1 : 0) +
                 ((Pha_Sweep[3] > 30.0f && Pha_Sweep[3] < 150.0f) ? 1 : 0) >= 2)
        {
            element_type = 3;  // 标记为电感
            float best_fre_l = 0;
            uint8_t fi;
            // 选择最佳测试频率：选阻抗在有效范围内的最高频率（频率越高，小电感越准确）
            for (fi = 0; fi < 5; fi++) {
                if (Zabs_Sweep[fi] > 1.0f && Zabs_Sweep[fi] < 8000000.0f) {
                    best_fre_l = Fre_Sweep[fi];
                    // 不break，继续找更高的频率
                }
            }
            if (best_fre_l == 0) { best_fre_l = Fre_Sweep[2]; }  // 兜底
            gear_shifting_frequency = best_fre_l;
        }
        // 判断电容：相位滞后（<-45°）
        else if (Pha_Sweep[1] < -45.0f && Pha_Sweep[1] > -135.0f)
        {
            element_type = 2;  // 标记为电容
            float best_fre = 0;
            uint8_t best_idx = 1;
            uint8_t fi;
            // 选择最佳测试频率：阻抗在10Ω~8MΩ范围内的最高频率（频率越高信号越强，pF小电容更准确）
            for (fi = 0; fi < 5; fi++) {
                if (Zabs_Sweep[fi] > 10.0f && Zabs_Sweep[fi] < 8000000.0f) {
                    best_fre = Fre_Sweep[fi];
                    best_idx = fi;
                    // 不break，继续找更高的频率
                }
            }
            if (best_fre == 0) { best_fre = Fre_Sweep[1]; best_idx = 1; }  // 兜底
            gear_shifting_frequency = best_fre;
        }
        // 剩余情况：电阻（相位接近0°）
        else
        {
            element_type = 1;  // 标记为电阻
        }

        /************************ 第三步：自动换挡与精确测量 ************************/
        setDDS(2.0, gear_shifting_frequency, 50, SINWAVE);  // 切换到选定的测试频率
        OSTimeDly(5);                                          // 等待信号稳定（从20ms缩短到5ms）

        // 执行自动换挡（返回是否发生了换挡）
        Gear_BH_flag = change_resistance_gear();
        // 换挡后基准电阻变了，之前的扫频数据全部失效，必须重新扫频
        if (Gear_BH_flag) {
            last_z = Z_abs;
            continue;  // 跳过本次显示，回到while循环开头重新扫频
        }
        last_z = Z_abs;

        Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);  // 获取精确测量的FFT结果

        /************************ 第四步：屏幕显示（基础信息） ************************/
        OS_Num_Show(270, 120, 32, 1, ShowPha, "阻抗角 : %0.3f°       ");
        OS_Num_Show(270, 160, 32, 1, ADC1VOL, "ADC1   : %0.3fV       ");
        OS_Num_Show(270, 200, 32, 1, ADC2VOL, "ADC2   : %0.3fV       ");

        /************************ 第五步：分元件类型处理与显示 ************************/
        // 短路/开路处理（直接根据ADC电压判断）
        // 短路：ADC2（基准）正常，ADC1（待测）接近0
        if (ADC2VOL > 0.5f && ADC1VOL < 0.005f)
        {
            OS_Num_Show(270, 240, 32, 1, 1, "短路检测                        ");
            OS_Num_Show(270, 280, 32, 1, 1, "                               ");
            OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
            OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
        }
        // 开路：ADC2（基准）接近0，ADC1（待测）正常
        else if (ADC2VOL < 0.005f && ADC1VOL > 2.0f)
        {
            OS_Num_Show(270, 240, 32, 1, 1, "开路检测                        ");
            OS_Num_Show(270, 280, 32, 1, 1, "                               ");
            OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
            OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
        }
        // 正常元件处理
        else
        {
            // 电感测量与显示
            if (element_type == 3)
            {
                Show_flag = 3;
                float measure_freq;
                float best_fre_l = 0;
                uint8_t fi;
                // 选择最佳测量频率：阻抗在有效范围内的最高频率
                for (fi = 0; fi < 5; fi++) {
                    if (Zabs_Sweep[fi] > 1.0f && Zabs_Sweep[fi] < 8000000.0f) {
                        best_fre_l = Fre_Sweep[fi];
                    }
                }
                if (best_fre_l == 0) { best_fre_l = Fre_Sweep[2]; }
                measure_freq = best_fre_l;

                // 重新切换到最佳频率并测量
                setDDS(2.0, measure_freq, 50, SINWAVE);
                OSTimeDly(5);
                Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);
                Get_Zabs(ADC1VOL, ADC2VOL, measure_freq);

                // 根据阻抗大小选择电感计算公式
                if (Z_abs < 10.0f)
                {
                    L_abs = CalculateInductanceSmallZabs(Z_abs, ShowPha, measure_freq);  // 小阻抗专用公式
                }
                else
                {
                    L_abs = CalculateInductanceLargeZabs(Z_abs, ShowPha, measure_freq);  // 大阻抗专用公式
                }

                // 显示电感基础信息
				OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模 : %0.3fΩ       ");
                OS_Num_Show(270, 320, 32, 1, measure_freq, "使用测试频率:%.0fHz          ");

                Correct(3);  // 电感误差校正

                // 电感值单位自动转换（H/mH/μH）
                // L_abs 单位为 μH
                if (L_abs >= 1000000.0f)
                {
                    OS_Num_Show(270, 280, 32, 1, L_abs / 1000000.0f, "电感 : %0.3fH            ");
                }
                else if (L_abs >= 1000.0f)
                {
                    OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "电感 : %0.3fmH           ");
                }
                else
                {
                    OS_Num_Show(270, 280, 32, 1, L_abs,          "电感 : %0.3fuH           ");
                }
            }
            // 电容测量与显示
            else if (element_type == 2)
            {
                Show_flag = 2;
                float best_fre = 0;
                uint8_t best_idx = 1;
                uint8_t fi;
                // 选择最佳频率：取阻抗在有效范围内的最高频率（频率越高信号越强）
                for (fi = 0; fi < 5; fi++) {
                    if (Zabs_Sweep[fi] > 10.0f && Zabs_Sweep[fi] < 8000000.0f) {
                        best_fre = Fre_Sweep[fi];
                        best_idx = fi;
                        // 不break，继续找更高的频率
                    }
                }
                if (best_fre == 0) { best_fre = Fre_Sweep[1]; best_idx = 1; }

                // 重新切换到最佳频率并测量
                setDDS(2.0, best_fre, 50, SINWAVE);
                OSTimeDly(5);
                Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);
                Get_Zabs(ADC1VOL, ADC2VOL, best_fre);

                // 根据频率范围选择电容计算公式
                if (best_idx == 0)
                {
                    C_abs = CalculateCapacitanceSmallRange(best_fre, Z_abs, ShowPha);  // 低频（100Hz）
                }
                else if (best_idx == 1)
                {
                    C_abs = CalculateCapacitanceMidRange(best_fre, Z_abs, ShowPha);  // 中频（500Hz）
                }
                else
                {
                    C_abs = CalculateCapacitanceLargeRange(best_fre, Z_abs, ShowPha);  // 高频（5kHz+）
                }

                // 显示电容基础信息
				OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模 : %0.3fΩ       ");
                OS_Num_Show(270, 320, 32, 1, best_fre, "使用测试频率:%.0fHz          ");

                Correct(2);                    // 电容误差校正
                C_abs = CalibrateCapacitance(C_abs);  // 电容二次校准

                // 电容值单位自动转换（μF/nF/pF）
                // C_abs 单位为 nF
                if (C_abs >= 1000.0f)
                {
                    OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "电容 : %0.3fuF            ");
                }
                else if (C_abs >= 1.0f)
                {
                    OS_Num_Show(270, 280, 32, 1, C_abs,          "电容 : %0.3fnF            ");
                }
                else
                {
                    OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "电容 : %0.3fpF            ");
                }
            }
            // 电阻测量与显示
            else
            {
                Show_flag = 1;
                // 切换到500Hz进行电阻测量
                setDDS(2.0, Fre_Sweep[1], 50, SINWAVE);
                OSTimeDly(5);
                Get_FFTInformation(ddsStructData.hz, MENU_IMPEDANCE);
                Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[1]);

                Correct(1);  // 电阻误差校正

                // 显示电阻信息
				OS_Num_Show(270, 80,  32, 1, Z_abs,   "阻抗模 : %0.3fΩ       ");
                OS_Num_Show(270, 280, 32, 1, 1,     "                               ");
                OS_Num_Show(270, 320, 32, 1, 500,  "使用测试频率:%.0fHz          ");
                OS_Num_Show(270, 360, 32, 1, 1,     "                                        ");
            }
        }

        OSTimeDly(100);  // 延时100ms，刷新显示周期
    }

    Change_Menu(0);  // 退出菜单1，返回主菜单
}

/**
  * @brief  菜单二扫描测量模式
  * @功能: 对5个频率点进行扫描测量，识别网络类型并显示参数
  * @param  无
  * @retval 无
  */
void MenuHaddler_2() 
{
	uint8_t i;
	uint8_t network;
	float w1, w2, ResonantFre = 0;
	
	/* 扫描频率点（统一使用：100Hz/500Hz/5kHz/20kHz/100kHz） */
	Fre_Sweep[0] = 100;
	Fre_Sweep[1] = 500;
	Fre_Sweep[2] = 5000;
	Fre_Sweep[3] = 20000;
	Fre_Sweep[4] = 100000;
	
	/* 初始化DDS为5kHz */
	setDDS(2.0, 5000, 50, SINWAVE);
	Gear_sign = Gear_820;
	SetGear(Gear_sign);
	Ps2KeyValue = KeyValue_Null;

	while ( Ps2KeyValue != KeyValue_Back ) 
	{
		/* 设置当前频率为5kHz */
		setDDS(2.0, 5000, 50, SINWAVE);
		OSTimeDly(10);
		
		Display_flag = 0;
		change_resistance_gear(); /* 自动换挡 */
		
		/************************ 第一步：先获取5kHz下的初始数据 ************************/
		Get_FFTInformation(5000, MENU_NETWORK);
		Pha_Sweep[1] = ShowPha;
		ADC1_Sweep[1] = ADC1VOL;
		ADC2_Sweep[1] = ADC2VOL;
		Get_Zabs(ADC1VOL, ADC2VOL, 5000);
		Zabs_Sweep[1] = Z_abs;

		/************************ 第二步：检查短路/断路 ************************/
		if (Z_abs < 0.10f) {
			// 短路时清空显示区域
			OS_String_Show(270, 80, 32, 1, "网络结构: --         ");
			OS_String_Show(270, 120, 32, 1, "谐振点  : --         ");
			OS_Num_Show(270, 200, 32, 1, 1, "电路短路                        ");
			OS_Num_Show(270, 240, 32, 1, 1, "                               ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
		} 
		else if (Z_abs > 20000000 && ADC1VOL > 3.00f) {
			// 断路时清空显示区域
			OS_String_Show(270, 80, 32, 1, "网络结构: --         ");
			OS_String_Show(270, 120, 32, 1, "谐振点  : --         ");
			OS_Num_Show(270, 200, 32, 1, 1, "电路断路                        ");
			OS_Num_Show(270, 240, 32, 1, 1, "                               ");
			OS_Num_Show(270, 280, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 320, 32, 1, 1, "                                        ");
			OS_Num_Show(270, 360, 32, 1, 1, "                                        ");
		} 
		else {
			/************************ 第三步：对5个频率点进行完整扫描测量 ************************/
			for (i = 0; i < 5; i++) {
				setDDS(2.0, Fre_Sweep[i], 50, SINWAVE);
				OSTimeDly(10);
				Get_FFTInformation(Fre_Sweep[i], MENU_NETWORK);
				
				Pha_Sweep[i] = ShowPha;
				ADC1_Sweep[i] = ADC1VOL;
				ADC2_Sweep[i] = ADC2VOL;
				Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[i]);
				Zabs_Sweep[i] = Z_abs;
				
				/* 高频相位补偿：和扫频点一一对应（100Hz/500Hz/5kHz/20kHz/100kHz） */
				switch(i) {
					case 0: Pha_Sweep[i] += PHA_COMPENSATE_100HZ; break;
					case 1: Pha_Sweep[i] += PHA_COMPENSATE_500HZ; break;
					case 2: Pha_Sweep[i] += PHA_COMPENSATE_5KHZ; break;
					case 3: Pha_Sweep[i] += PHA_COMPENSATE_20KHZ; break;
					case 4: Pha_Sweep[i] += PHA_COMPENSATE_100KHZ; break;
					default: break;
				}
				// 补偿后相位修正到-180~180°
				Pha_Sweep[i] = User_FixPhase(Pha_Sweep[i]);
			}

			/************************ 第四步：识别网络并计算元件参数 ************************/
			network = Get_Network(); 
			L_abs = 0; // 先清零
			C_abs = 0;
			ResonantFre = 0;

			/* 根据网络类型计算 L 和 C 的值 */
			if ( network == RC_S ) {
				Z_abs = CalculateRC_SResistance(Zabs_Sweep[2], Pha_Sweep[2]);
				C_abs = CalculateRC_SCapacitance(Zabs_Sweep[2], Pha_Sweep[2], Fre_Sweep[2]);
				C_abs = CalibrateCapacitance(C_abs);
			} else if ( network == RC_P ) {
				Z_abs = CalculateRC_PResistance(Zabs_Sweep[1], Pha_Sweep[1]);
				C_abs = CalculateRC_PCapacitance(Pha_Sweep[2], Z_abs, Fre_Sweep[2]);
				C_abs = CalibrateCapacitance(C_abs);
			} else if ( network == RL_S ) {
				Z_abs = CalculateRL_SResistance(Zabs_Sweep[1], Pha_Sweep[1]);
				L_abs = CalculateRL_SInductance(Zabs_Sweep[1], Pha_Sweep[1], Fre_Sweep[1]);
			} else if ( network == RL_P ) {
				/* RL并联计算：对10kHz频点进行5次独立测量，真正中值滤波 */
				for (i = 0; i < 5; i++) {
					Get_FFTInformation(Fre_Sweep[2], MENU_NETWORK);
					Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[2]);
					z_r[i] = CalculateRL_PResistancePart1(Z_abs, ShowPha);
				}
				SortArray(z_r, 5); Z_abs = z_r[2];
				L_abs = CalculateRL_PInductancePart1(Z_abs, Fre_Sweep[2], Pha_Sweep[2]);
				if (Z_abs > 20000) {
					for (i = 0; i < 5; i++) {
						Get_FFTInformation(Fre_Sweep[3], MENU_NETWORK);
						Get_Zabs(ADC1VOL, ADC2VOL, Fre_Sweep[3]);
						z_r[i] = CalculateRL_PResistancePart2(Z_abs, ShowPha);
					}
					SortArray(z_r, 5); Z_abs = z_r[2];
					L_abs = CalculateRL_PInductancePart2(Z_abs, Fre_Sweep[3], Pha_Sweep[3]);
				}
			} else if ( network == LC_S ) {
				w1 = 2 * PI * Fre_Sweep[1]; w2 = 2 * PI * Fre_Sweep[3];
				C_abs = CalculateLC_SC(w1, w2, Pha_Sweep[1], Zabs_Sweep[1], Pha_Sweep[3], Zabs_Sweep[3]);
				L_abs = CalculateLC_SL(w1, w2, Pha_Sweep[1], Zabs_Sweep[1], Pha_Sweep[3], Zabs_Sweep[3]);
			} else if ( network == LC_P ) {
				w1 = 2 * PI * Fre_Sweep[1]; w2 = 2 * PI * Fre_Sweep[3];
				C_abs = CalculateLC_PCPart1(w1, w2, Zabs_Sweep[1], Zabs_Sweep[3]);
				L_abs = CalculateLC_PLPart1(w1, w2, Zabs_Sweep[4], Zabs_Sweep[2]);
				if (L_abs * pow(10, 6) > 1000) { 
					w1 = 2 * PI * Fre_Sweep[1]; w2 = 2 * PI * Fre_Sweep[4];
					C_abs = CalculateLC_PCPart2(w1, w2, Zabs_Sweep[1], Zabs_Sweep[4]);
					L_abs = CalculateLC_PLPart2(w1, w2, Zabs_Sweep[4], Zabs_Sweep[0]);
				}
			}

			/************************ 第五步：统一对齐排版显示 ************************/
			/* 左侧顶部 */
			OS_Num_Show(270, 80, 32, 1, network, "网络结构:           ");
			if(L_abs > 0 && C_abs > 0) {
				ResonantFre = CalculateResonantFrequency(L_abs, C_abs);
				if(ResonantFre < 1000000.0f) { 
					OS_Num_Show(270, 120, 32, 1, ResonantFre, "谐振点  : %0.1fHz    ");
				} else {
					OS_String_Show(270, 120, 32, 1, "谐振点  : N/A         ");
				}
			} else {
				OS_String_Show(270, 120, 32, 1, "谐振点  : N/A         ");
			}

			/* 右侧全部统一Y坐标对齐：200 / 240 / 280 等间距严格对齐 */
			if ( network == RC_S ) {
				OS_String_Show(420, 80, 32, 1, "RC串联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "C: %.3fuF");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "C: %.3fpF");
				} else {
					OS_Num_Show(270, 280, 32, 1, C_abs, "C: %.3fnF");
				}
			} else if ( network == RC_P ) {
				OS_String_Show(420, 80, 32, 1, "RC并联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, C_abs / 1000.0f, "C: %.3fuF");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 280, 32, 1, C_abs * 1000.0f, "C: %.3fpF");
				} else {
					OS_Num_Show(270, 280, 32, 1, C_abs, "C: %.3fnF");
				}
			} else if ( network == RL_S ) {
				OS_String_Show(420, 80, 32, 1, "RL串联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ");
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fuH");
				}
			} else if ( network == RL_P ) {
				OS_String_Show(420, 80, 32, 1, "RL并联");
				OS_Num_Show(270, 240, 32, 1, Z_abs, "R: %.3fΩ");
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fuH");
				}
			} else if ( network == LC_S ) {
				OS_String_Show(420, 80, 32, 1, "LC串联");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 240, 32, 1, C_abs / 1000.0f, "C: %.3fuF");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1000.0f, "C: %.3fpF");
				} else {
					OS_Num_Show(270, 240, 32, 1, C_abs, "C: %.3fnF");
				}
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fuH");
				}
			} else if ( network == LC_P ) {
				OS_String_Show(420, 80, 32, 1, "LC并联");
				if (C_abs > 100.0f) {
					OS_Num_Show(270, 240, 32, 1, C_abs / 1000.0f, "C: %.3fuF");
				} else if (C_abs < 0.1f) {
					OS_Num_Show(270, 240, 32, 1, C_abs * 1000.0f, "C: %.3fpF");
				} else {
					OS_Num_Show(270, 240, 32, 1, C_abs, "C: %.3fnF");
				}
				if (L_abs > 100.0f) {
					OS_Num_Show(270, 280, 32, 1, L_abs / 1000.0f, "L: %.3fmH");
				} else {
					OS_Num_Show(270, 280, 32, 1, L_abs, "L: %.3fuH");
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
  * @brief  菜单3：自动测量模式
  * @功能：  手动选择频率/步进调频率、手动换挡，显示校准前后的阻抗
  * @param  无
  * @retval 无
  */
void MenuHaddler_3() {
	uint8_t targetGear;
	float current_fre = 500.0f; /* 当前频率变量，彻底解决循环还原问题，初始500Hz */

	/* 初始配置：用current_fre初始化，不再固定写死 */
	setDDS(2.0, current_fre, 50, SINWAVE);
	OSTimeDly(10);
	Gear_sign = Gear_820;
	SetGear(Gear_sign);
	Ps2KeyValue = KeyValue_Null;

	while ( Ps2KeyValue != KeyValue_Back ) {
		/* 每次循环用current_fre设置频率，不再强制重置 */
		setDDS(2.0, current_fre, 50, SINWAVE);
		OSTimeDly(10);

		/* 按键处理：保留原有换挡+固定频率，新增±10kHz步进 */
		switch (Ps2KeyValue) {
			/* 原有手动换挡按键 1-4 */
			case KeyValue_1: Gear_sign = Gear_47; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_2: Gear_sign = Gear_820; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_3: Gear_sign = Gear_15k; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_4: Gear_sign = Gear_270k; SetGear(Gear_sign); Ps2KeyValue = KeyValue_Null; break;
			
			/* 原有固定频率快捷键 5-9 */
			case KeyValue_5: current_fre = 100;   Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_6: current_fre = 1000;  Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_7: current_fre = 10000; Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_8: current_fre = 100000;Ps2KeyValue = KeyValue_Null; break;
			case KeyValue_9: current_fre = 150000;Ps2KeyValue = KeyValue_Null; break;
			
			/* 频率步进功能，加安全上下限 */
			case KeyValue_Add: 
				current_fre += 10000; /* +10kHz */
				if(current_fre > 200000) current_fre = 200000; /* 上限200kHz，不超硬件能力 */
				Ps2KeyValue = KeyValue_Null; 
				break;
				
			case KeyValue_Minus: 
				current_fre -= 10000; /* -10kHz */
				if(current_fre < 100) current_fre = 100; /* 下限100Hz，避免低频异常 */
				Ps2KeyValue = KeyValue_Null; 
				break;
				
			default: break;
		}
		
		/* 显示当前频率，用current_fre保证显示和实际一致 */
		OS_Num_Show(270, 360, 32, 1, current_fre, "当前频率：%0.2fHz                         ");

		Display_flag = 0;
		/* 自动换挡逻辑：菜单3保留原始显示，同时使用阻值边界换挡 */
		while (Display_flag == 0) {
			targetGear = Gear_sign;
			Get_FFTInformation(current_fre, MENU_ORIGINAL);
			/************************ 菜单3显示：原始ADC + 相位 ************************/
			OS_Num_Show(270, 80, 32, 1, ADC1VOL, "ADC1VOL: %0.3f       ");
			OS_Num_Show(270, 120, 32, 1, ADC2VOL, "ADC2VOL: %0.3f       ");
			OS_Num_Show(270, 160, 32, 1, ShowPha, "相位值 : %0.3f       ");

			if (ADC2VOL < 0.10f) {
				/* 电压保护：ADC2过小，升高基准电阻 */
				targetGear = GetHigherGear(Gear_sign);
			}
			else if (ADC2VOL > (ADC1VOL * 10) || ADC2VOL > 3.0f) {
				/* 电压保护：ADC2过大，降低基准电阻 */
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
		/************************ 菜单3最终显示 ************************/
		OS_Num_Show(270, 80, 32, 1, ADC1VOL, "ADC1VOL: %0.3f       ");
		OS_Num_Show(270, 120, 32, 1, ADC2VOL, "ADC2VOL: %0.3f       ");
		OS_Num_Show(270, 160, 32, 1, ShowPha, "相位值 : %0.3f       ");

        /* 1. 计算未校准原始阻抗：纯欧姆定律，无硬件校准系数 */
        float Rref = Rref_Table[(Gear_sign < Gear_Count) ? Gear_sign : 1];
        // 防止ADC2电压为0导致除以零
        if(ADC2VOL > 0.005f) {
            float Z_abs_raw = Rref * ADC1VOL / ADC2VOL;
            OS_Num_Show(270,220,32,1,Z_abs_raw,"未校准阻抗:%.3fΩ                          ");
        } else {
            OS_String_Show(270,220,32,1,"未校准阻抗: 超量程            ");
        }

        // 2. 计算校准后阻抗：传入正确的当前频率
        Get_Zabs(ADC1VOL,ADC2VOL,current_fre);
        OS_Num_Show(270,260,32,1,Z_abs,"校准后阻抗:%.3fΩ                          ");
	}
	
	Change_Menu( 0 ); /* 返回主菜单 */
}

/* ==================== 菜单4：学习模式（核心创新点，已加安全加固） ==================== */
/**
  * @brief  学习模式：保存标准元件的测量值和修正比例
  * @安全加固：1. 限制Storage_Bit不超过399；2. 检查指针对齐；3. 限制循环次数
  * @param  无
  * @retval 无
  */
void MenuHaddler_4()
{
	int i = 0;
    uint8_t enter_input_flag = 0; /* 进入输入模式的标志 */
    float input_data = 0.0f;      /* 输入的标准值 */
    char *element_name = "未知";   /* 元件类型名称，用于显示 */

	/* 上电先读取Flash里的学习数据 */
	Show_Val(7,0,"   读取数据中...                 ");
	InFLASH_Read(ADDR_FLASH_SECTOR_10,Proportion_Tmep,1203);
	InFLASH_Read(ADDR_FLASH_SECTOR_11,Value_Tmep,1200);
    Show_Val(7,0,"   读取成功                  ");

	/* 数据类型转换 */
	for(i=0;i<1203;i++)
	{
		Proportion[i]=*(float *)&Proportion_Tmep[i];
		if(i<1200)
		{
			Value[i]=*(float *)&Value_Tmep[i];
		}
	}
	
	/* 读取已存储的数据个数 */
	Storage_Bit_z=Proportion_Tmep[0];
	Storage_Bit_c=Proportion_Tmep[1];
	Storage_Bit_l=Proportion_Tmep[2];
	
	Storage_mode=Show_flag; /* 从单频测量模式获取当前元件类型 */
	Ps2KeyValue = KeyValue_Null; /* 清空按键缓存 */

    /* 先清屏，显示基础界面 */
    LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black );

    /************************ 新增：元件类型有效性检查+引导 ************************/
    switch(Storage_mode) {
        case 1: element_name = "电阻"; break;
        case 2: element_name = "电容"; break;
        case 3: element_name = "电感"; break;
        default: element_name = "未知"; break;
    }

    // 无效元件类型，提示用户操作步骤
    if(Storage_mode < 1 || Storage_mode > 3) {
        Show_Val(1,0,"请先进入【单频测量模式】");
        Show_Val(2,0,"识别元件后再进入学习模式");
        Show_Val(3,0,"当前元件：未知");
        Show_Val(7,0,"按Back键返回主菜单");
        // 等待用户按返回键退出
        while(Ps2KeyValue != KeyValue_Back) {
            OSTimeDly(10);
        }
        Ps2KeyValue = KeyValue_Null;
        Change_Menu(0);
        return; // 直接退出，不执行后续逻辑
    }

    // 有效元件类型，显示正常界面
    Show_Val(1,0,"当前学习元件：");
    Show_Val(2,0,element_name);
    Show_Val(7,0,"按数字键输入标准值，按Back退出");

	/* 外层循环：先检测按键，再处理逻辑，随时可退出 */
	while(1)
	{	
        /* 1. 优先检测Back键：按下直接退出菜单，返回主界面 */
        if(Ps2KeyValue == KeyValue_Back)
        {
            Ps2KeyValue = KeyValue_Null;
            break; /* 直接跳出循环，执行后面的Change_Menu(0) */
        }

        /* 2. 检测数字键：按下进入输入模式 */
        if(Ps2KeyValue >= KeyValue_0 && Ps2KeyValue <= KeyValue_9 && enter_input_flag == 0)
        {
            enter_input_flag = 1; /* 标记进入输入模式 */
        }

        /* 3. 进入输入模式：调用输入函数 */
        if(enter_input_flag == 1)
        {
            Show_Val(7,0,"请输入标准数值... ");
            input_data = PS2_ReadNum(0); /* 调用输入函数 */
            enter_input_flag = 0; /* 输入完成，退出输入模式 */
        }

        /* 4. 处理输入结果：输入有效（>0）才执行保存 */
        if(input_data > 0)
        {
            switch(Storage_mode)
            {
                case 1: /* 电阻学习 */
                    Show_Val(3,Storage_Bit_z,"第%.0f位数据                 ");
                    Show_Val(4,Z_abs,"测量值: %.3f Ω            ");

                    if(Storage_Bit_z < 400)
                    {
                        /* 计算修正比例 */
                        Proportion[Storage_Bit_z + 3] = input_data / Z_abs;
                        Proportion_Tmep[Storage_Bit_z + 3] = *(uint32_t *)&Proportion[Storage_Bit_z + 3];
                        /* 保存原始测量值 */
                        Value[Storage_Bit_z] = Z_abs;
                        Value_Tmep[Storage_Bit_z] = *(uint32_t *)&Z_abs;
                        /* 更新计数 */
                        Storage_Bit_z++;
                        Proportion_Tmep[0] = Storage_Bit_z;
                        
                        /* 写入Flash */
                        Show_Val(7,0,"   保存中...                  ");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
                        OSTimeDly(200); /* 用OSTimeDly替代delay_ms，不阻塞任务调度 */
                        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep, 1200);
                        OSTimeDly(200);
                        Show_Val(7,0,"   保存成功，按Back退出       ");
                    }
                    else
                    {
                        Show_Val(7,0,"   数据已存满(≤399)         ");
                    }
                    break;
                    
                case 2: /* 电容学习 */
                    Show_Val(3,Storage_Bit_c,"第%.0f位数据                 ");
                    Show_Val(4,C_abs,"测量值: %.6f nF            ");

                    if(Storage_Bit_c < 400)
                    {
                        Proportion[Storage_Bit_c+3+400] = input_data / C_abs;
                        Proportion_Tmep[Storage_Bit_c+3+400] = *(uint32_t *)&Proportion[Storage_Bit_c+3+400];
                        Value[Storage_Bit_c+400] = C_abs;
                        Value_Tmep[Storage_Bit_c+400] = *(uint32_t *)&C_abs;
                        Storage_Bit_c++;
                        Proportion_Tmep[1] = Storage_Bit_c;
                        
                        Show_Val(7,0,"   保存中...                 ");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep, 1200);
                        OSTimeDly(200);
                        Show_Val(7,0,"   保存成功，按Back退出      ");
                    }
                    else
                    {
                        Show_Val(7,0,"   数据已存满(≤399)         ");
                    }
                    break;
                    
                case 3: /* 电感学习 */
                    Show_Val(3,Storage_Bit_l,"第%.0f位数据                 ");
                    Show_Val(4,L_abs,"测量值: %.6f uH            ");

                    if(Storage_Bit_l < 400)
                    {
                        Proportion[Storage_Bit_l+3+800] = input_data / L_abs;
                        Proportion_Tmep[Storage_Bit_l+3+800] = *(uint32_t *)&Proportion[Storage_Bit_l+3+800];
                        Value[Storage_Bit_l+800] = L_abs;
                        Value_Tmep[Storage_Bit_l+800] = *(uint32_t *)&L_abs;
                        Storage_Bit_l++;
                        Proportion_Tmep[2] = Storage_Bit_l;
                        
                        Show_Val(7,0,"   保存中...                 ");
                        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
                        OSTimeDly(200);
                        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep, 1200);
                        OSTimeDly(200);
                        Show_Val(7,0,"   保存成功，按Back退出      ");
                    }
                    else
                    {
                        Show_Val(7,0,"   数据已存满(≤399)         ");
                    }
                    break;
                    
                default:
                    Show_Val(2,0,"        无法学习!!!               ");
                    break;
            }
            input_data = 0.0f; /* 处理完清空输入值，避免重复执行 */
        }

        /* 5. 循环延时，释放CPU */
        OSTimeDly(10);
	}
	
	/* 跳出循环后，一定会执行这里，返回主菜单 */
    LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black );
	Change_Menu( 0 );
}

/**
  * @brief  菜单5：清除学习数据
  * @功能：  清空Flash中的所有学习数据，恢复出厂状态
  * @param  无
  * @retval 无
  */
void MenuHaddler_5() {
	Ps2KeyValue = KeyValue_Null;
    uint8_t confirm_flag = 0;

    // 增加确认环节，避免误触清除
    LCD_Appoint_Clear( 250 + 2, 64 + 8, 800, 480 - 32 - 8, Black );
    Show_Val(1,0,"确定要清除所有学习数据？");
    Show_Val(2,0,"按Enter确认，按Back取消");

    // 等待用户确认
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
        Correct_init(); /* 清空内存中的学习数据 */
        /* 写入Flash */
        InFLASH_Write(ADDR_FLASH_SECTOR_10,Proportion_Tmep, 1203);
        OSTimeDly(200);
        InFLASH_Write(ADDR_FLASH_SECTOR_11,Value_Tmep, 1200);
        OSTimeDly(200);
        Show_Val(1,0,"         恢复成功           ");
        OSTimeDly(1000);
    }

    // 无论是否清除，都返回主菜单
	Change_Menu( 0 );
}

/* ***************************** Custom Function Part（核心算法函数） ***************************** */
/* ------------------------------ 电感计算函数 ------------------------------ */
float CalculateInductanceSmallZabs(float zAbs, float pha, float fre) {
	float l = zAbs * sin(pha * PI / 180.0f) / (2 * PI * fre) * pow(10, 6);
	return 0.95f * l; /* 小电感校准系数 */
}

float CalculateInductanceLargeZabs(float zAbs, float pha, float fre) {
	float l = zAbs * sin(pha * PI / 180.0f) / (2 * PI * fre) * pow(10, 6);
	return 1.00f * l; /* 大电感校准系数 */
}

/* ------------------------------ 电容计算函数（分频段校准优化） ------------------------------ */
float CalculateCapacitanceMidRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if(fabs(sin_pha) < 0.001f) sin_pha = 0.001f;  /* 防除零 */
	float c = 1 / (2 * PI * fre * zAbs) * pow(10, 9);
	return c * 1.0f; /* 中频（500Hz）校准系数，需根据实测修改 */
}

float CalculateCapacitanceSmallRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if(fabs(sin_pha) < 0.001f) sin_pha = 0.001f;
	float c = 1 / (2 * PI * fre * zAbs) * pow(10, 9);
	return c * 1.0f; /* 低频（100Hz）校准系数，需根据实测修改 */
}

float CalculateCapacitanceLargeRange(float fre, float zAbs, float pha) {
	float sin_pha = sin(pha * PI / 180.0f);
	if(fabs(sin_pha) < 0.001f) sin_pha = 0.001f;
	float c = 1 / (2 * PI * fre * zAbs) * pow(10, 9);
	return c * 1.0f; /* 高频（5kHz+）校准系数，需根据实测修改 */
}

/**
  * @brief  电容二次校准
  * @功能：  分段校准，避免小电容负值失真
  * @param  c: 原始计算电容值，单位nF
  * @retval 校准后的电容值，单位nF
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
  * @brief  冒泡排序（用于中值滤波）
  * @param  arr: 待排序数组
  * @param  len: 数组长度
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
	if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;  /* 防除零 */
	return zAbs * sqrt(1.0f + tan_pha * tan_pha);  /* 正确公式：tan2(φ) */
}

/**
  * @brief  RL并联电感计算Part1
  */
float CalculateRL_PInductancePart1(float r, float fre, float pha) {
    float tan_pha = tan(pha * PI / 180.0f);
    // 边界保护：相位接近0°时，避免除以零
    if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;
    if(fre < 10.0f) fre = 10.0f;
	return r / (2 * PI * fre * tan_pha) * 1e6f; // 修正单位转换：H转uH是1e6
}

float CalculateRL_PResistancePart2(float zAbs, float pha) {
	return zAbs * sqrt(1 + pow(tan(pha / 180 * PI), 2)) * 1.062f;
}

/**
  * @brief  RL并联电感计算Part2
  */
float CalculateRL_PInductancePart2(float r, float fre, float pha) {
    float tan_pha = tan(pha * PI / 180.0f);
    if(fabs(tan_pha) < 0.001f) tan_pha = 0.001f;
    if(fre < 10.0f) fre = 10.0f;
	return r / (2 * PI * fre * tan_pha) * 1e6f; // 统一单位转换
}

/* ------------------------------ LC串并联计算 ------------------------------ */
float CalculateLC_SC(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3) {
	float numerator = pow(w1, 2) - pow(w2, 2);
	float denominator = w1 * w2 * (sin(pha1 * PI / 180.0f) * zAbs1 * w2 - sin(pha3 * PI / 180.0f) * zAbs3 * w1);
	if(fabs(denominator) < 1e-10f) return 0.0f;  /* 防除零 */
	return numerator / denominator;
}

float CalculateLC_SL(float w1, float w2, float pha1, float zAbs1, float pha3, float zAbs3) {
	float numerator = sin(pha1 * PI / 180.0f) * zAbs1 * w1 - sin(pha3 * PI / 180.0f) * zAbs3 * w2;
	float denominator = pow(w1, 2) - pow(w2, 2);
	if(fabs(denominator) < 1e-10f) return 0.0f;  /* 防除零 */
	return numerator / denominator;
}

float CalculateResonantFrequency(float l, float c) {
	return 1 / (2 * PI * sqrt(fabs(l * c)));
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
  * @brief  获取相邻高阻值档位
  * @param  Gear: 当前档位
  * @retval 相邻高阻值档位，到最高档后保持不变
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
  * @brief  获取相邻低阻值档位
  * @param  Gear: 当前档位
  * @retval 相邻低阻值档位，到最低档后保持不变
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
  * @brief  根据阻值边界选择档位
  * @功能：  47Ω/820Ω/15kΩ/270kΩ四档按几何平均边界换挡，并加入滞回
  * @param  zAbs: 当前档位估算出的阻抗值
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
  * @brief  自动换挡逻辑
  * @功能：  电压条件只做保护，正常情况下按阻值边界自动切换档位，带滞回防止频繁切换
  * @param  无
  * @retval 1=发生了换挡，0=未换挡
  */
uint8_t change_resistance_gear(void) {
	uint8_t targetGear;
    float z_current;
	uint8_t gear_changed = 0;  /* 记录是否发生了换挡 */

	Display_flag = 0;

	/* 最多执行3次换挡尝试，防止死循环 */
	uint8_t max_attempts = 3;
	while (Display_flag == 0 && max_attempts > 0) {
		targetGear = Gear_sign;
		max_attempts--;

		/* 检查是否按下Back键 */
		if (Ps2KeyValue == KeyValue_Back) {
			Display_flag = 1;
			return gear_changed;
		}

		/* 换挡阶段用单次FFT快速判断，不做10次平均 */
		FFT_Handle();
		pha = User_FixPhase(pha);
		ShowPha = pha;
        Get_Zabs(ADC1VOL, ADC2VOL, ddsStructData.hz);
        z_current = Z_abs;

		/* 电压保护：ADC2过小先升高基准电阻，避免小信号比值失真 */
		if (ADC2VOL < 0.10f) {
			targetGear = GetHigherGear(Gear_sign);
		}
		/* 电压保护：ADC2过大先降低基准电阻，避免饱和或参考端压降过大 */
		else if (ADC2VOL > (ADC1VOL * 10) || ADC2VOL > 3.0f) {
			targetGear = GetLowerGear(Gear_sign);
		}
		else {
			/* 带滞回的档位选择：用last_z和当前值的平均值判断，避免边界抖动 */
            float z_for_gear = (z_current + last_z) * 0.5f;
			targetGear = GetBoundaryGear(z_for_gear, Gear_sign);
		}

		if (targetGear != Gear_sign) {
			Gear_sign = targetGear;
			SetGear(Gear_sign);
            gear_changed = 1;  /* 标记发生了换挡 */
            OSTimeDly(30); // 换挡后等待继电器稳定
            /* 换挡后直接退出，让主循环重新扫频（换挡后扫频数据已失效） */
            Display_flag = 1;
		} else {
			Display_flag = 1;  /* 档位合适，退出循环 */
		}
		OSTimeDly(5);  /* 从10ms减到5ms，加快换挡速度 */
	}
	return gear_changed;
}

/**
  * @brief  相位修正
  * @功能：  将相位限制在 -180° ~ 180° 之间
  * @param  pha: 原始相位
  * @retval 修正后的相位
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
  * @功能：  通过控制继电器（PC11/PC12）切换4个基准电阻
  * @param  Gear: 档位编号（Gear_100/Gear_10k/Gear_100k/Gear_500k，对应47Ω/820Ω/15kΩ/270kΩ）
  * @retval 无
  */
void SetGear(uint8_t Gear) {
	switch (Gear) {
		case Gear_47:
			PCout(11) = 0; PCout(12) = 0; /* 继电器组合：00 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 47Ω             ");
			break;
		case Gear_820:
			PCout(11) = 1; PCout(12) = 0; /* 继电器组合：10 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 820Ω            ");
			break;
		case Gear_15k:
			PCout(11) = 0; PCout(12) = 1; /* 继电器组合：01 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 15kΩ            ");
			break;
		case Gear_270k:
			PCout(11) = 1; PCout(12) = 1; /* 继电器组合：11 */
			OS_Num_Show(270, 400, 32, 1, 1, "当前档位: 270kΩ           ");
			break;
	}
}

/**
  * @brief  继电器控制GPIO初始化
  * @功能：  初始化PC11和PC12为推挽输出，用于控制继电器切换档位
  * @param  无
  * @retval 无
  */
void GPIO_Change_Init() 
{ 
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); /* 使能GPIOC时钟 */
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12; /* PC11和PC12 */
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; /* 输出模式 */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* 推挽输出 */
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; /* 无上下拉 */
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		PCout(11)=1;		/* 初始状态：270kΩ档（继电器11） */
		PCout(12)=1; 
}

/**
  * @brief  获取FFT分析结果 菜单切换显示
  * @功能：  调用FFT库，计算电压幅值和相位，并进行滤波
  * @param  FreSet: 设置的频率
  * @param  MenuMode: 菜单模式 1-阻抗 2-网络 3-原始
  * @retval 无（结果保存在全局变量中）
  */
void Get_FFTInformation(float FreSet, uint8_t MenuMode)
{
    /* 所有变量必须在函数最开头集中声明 */
    uint8_t i;
    uint8_t Pha_Max, Pha_Min, Vol1_Max, Vol1_Min, Vol2_Max, Vol2_Min;
    uint8_t LowFreFlag;
    float Pha_Sum, Vol1_Sum, Vol2_Sum;
    float FreTemp;
	float Pha_Ref = 0;
	float Pha_Avg = 0;
	
    /* 变量初始化 */
    i = 0;
    LowFreFlag = 0;

    /* 低频判断 */
    if (FreSet <= 100)
    {
        LowFreFlag = 1;        /* 低频特殊处理 */
    }
    FreTemp = FreSet;

    /************************ 核心采样/FFT逻辑（完全不变） ************************/
    if (LowFreFlag)
    {
        /* 低频：降低采样率，提高精度，单次采样 */
        Sample_rate = 64 * FreTemp;
        Set_SamplingFre(Sample_rate);
        OSTimeDly(5);
        FFT_Handle();
        pha = User_FixPhase(pha);
        LowFreFlag = 0;
        ShowPha = pha;
    }
    else
    {
        /* 中高频：根据频率调整采样率 */
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
        Set_SamplingFre(Sample_rate);
        OSTimeDly(5);
        FFT_Handle();

        /* 连续采样10次，去掉最大最小值取平均 */
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

            /* 查找极值索引 */
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
  * @brief  快速FFT分析（单次采样，用于扫频预测量）
  * @功能：  仅执行1次FFT采样，不做多次平均，用于扫频预测量阶段
  *         相比Get_FFTInformation的11次FFT（1+10），本函数仅1次FFT
  * @param  FreSet: 设置的频率
  * @retval 无（结果保存在全局变量ADC1VOL、ADC2VOL、ShowPha中）
  */
void Get_FFTQuick(float FreSet)
{
    float FreTemp = FreSet;

    /* 根据频率调整采样率（与Get_FFTInformation保持一致） */
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

    Set_SamplingFre(Sample_rate);
    OSTimeDly(5);

    /* 仅执行1次FFT，不做多次平均 */
    FFT_Handle();
    ShowPha = User_FixPhase(pha);
}

/**
  * @brief  计算阻抗
  * @功能：  根据当前档位和频率，选择对应的计算公式
  * @param  Get_ADC1: 通道1电压（被测件）
  * @param  Get_ADC2: 通道2电压（基准电阻）
  * @param  NowFre: 当前频率
  * @retval 无（结果保存在Z_abs中）
  */
void Get_Zabs(float Get_ADC1, float Get_ADC2, float NowFre) {
    // 防止除以零崩溃
    if(Get_ADC2 < 0.005f) {
        Z_abs = 999999999.0f;  // 无穷大（开路）
        return;
    }

    // 查找当前频率对应的索引
    uint8_t freq_idx = 0;
    uint8_t fi;
    for (fi = 0; fi < 5; fi++) {
        if (fabs(NowFre - Fre_Sweep[fi]) < (Fre_Sweep[fi] * 0.1f + 1.0f)) {
            freq_idx = fi;
            break;
        }
    }

    // 查表获取基准电阻和校准系数
    uint8_t gear_idx = (Gear_sign < Gear_Count) ? Gear_sign : 1;  // 防越界
    float rref = Rref_Table[gear_idx];
    float cal  = Gear_Calibration[gear_idx][freq_idx];

    // 统一公式：Z = Rref × V1 / V2 / 校准系数
    Z_abs = rref * Get_ADC1 / Get_ADC2 / cal;

    // 阻抗不能为负
    if(Z_abs < 0.0f) Z_abs = 0.0f;
    // 不再限制最大阻抗值，允许测量开路
}

/**
  * @brief  网络识别入口函数
  * @功能：  调用核心识别函数，统一入口
  * @param  无
  * @retval 网络类型编号
  */
uint8_t Get_Network(void) {
	return User_GetNetwork(Zabs_Sweep, Pha_Sweep);
}

/**
  * @brief  核心网络识别函数
  * @param  z_abs[]: 5个频率点的阻抗数组（100Hz/500Hz/5kHz/20kHz/100kHz）
  * @param  pha_dif[]: 5个频率点的相位数组
  * @retval 网络类型编号
  */
uint8_t User_GetNetwork( float z_abs[], float pha_dif[] ) {
	uint8_t network = Nw_Null;
    uint8_t i;
    float pha_min, pha_max;
    uint8_t pha_cross_zero = 0; // 相位是否过零（LC网络核心特征）
    uint8_t z_min_idx = 0; // 阻抗最小值对应的频点索引
    uint8_t z_max_idx = 0; // 阻抗最大值对应的频点索引

    /************************ 第一步：提取核心特征 ************************/
    // 1. 找阻抗极值点（LC网络核心特征）
    for(i=0; i<5; i++) {
        if(z_abs[i] < z_abs[z_min_idx]) z_min_idx = i;
        if(z_abs[i] > z_abs[z_max_idx]) z_max_idx = i;
    }

    // 2. 判断相位是否过零（正负都有，LC网络核心特征）
    pha_min = pha_dif[0];
    pha_max = pha_dif[0];
    for(i=0; i<5; i++) {
        if(pha_dif[i] < pha_min) pha_min = pha_dif[i];
        if(pha_dif[i] > pha_max) pha_max = pha_dif[i];
    }
    // 相位同时有正有负，判定为过零
    if(pha_min < -5.0f && pha_max > 5.0f) {
        pha_cross_zero = 1;
    }

    /************************ 第二步：优先识别LC网络（最容易误判） ************************/
    if(pha_cross_zero == 1) {
        // LC串联：阻抗先减后增，中间有最小值（谐振点），相位从负变正
        if(z_min_idx > 0 && z_min_idx < 4) {
            network = LC_S;
        }
        // LC并联：阻抗先增后减，中间有最大值（谐振点），相位从正变负
        else if(z_max_idx > 0 && z_max_idx < 4) {
            network = LC_P;
        }
        return network;
    }

    /************************ 第三步：识别RC网络（相位始终为负） ************************/
    if(pha_max < -5.0f) { // 所有频点相位都为负，纯容性网络
        // 相位随频率升高越来越负（接近-90°），RC并联
        if(pha_dif[4] < pha_dif[2] && pha_dif[2] < pha_dif[1]) {
            network = RC_P;
        }
        // 阻抗随频率升高单调减小，RC串联
        else if(z_abs[4] < z_abs[3] && z_abs[3] < z_abs[2] && z_abs[2] < z_abs[1]) {
            network = RC_S;
        }
        return network;
    }

    /************************ 第四步：识别RL网络（相位始终为正） ************************/
    if(pha_min > 5.0f) { // 所有频点相位都为正，纯感性网络
        // 相位随频率升高越来越正（接近+90°），RL串联
        if(pha_dif[4] > pha_dif[2] && pha_dif[2] > pha_dif[1]) {
            network = RL_S;
        }
        // 阻抗随频率升高单调增大，RL并联
        else if(z_abs[4] > z_abs[3] && z_abs[3] > z_abs[2] && z_abs[2] > z_abs[1]) {
            network = RL_P;
        }
        return network;
    }

    /************************ 第五步：无法识别的网络 ************************/
	return Nw_Null;
}

/* ==================== 学习模式核心函数 ==================== */
/**
  * @brief  修正函数
  * @安全加固：限制循环次数i不超过Storage_Bit，且不超过399
  * @param  mode: 元件类型（1=电阻，2=电容，3=电感）
  * @retval 无
  */
void Correct(uint8_t mode) {
	u32 i=0;
	u32 Correct_flag=0;
	switch(mode) {
		case 1: /* 修正电阻 */
			/* 循环条件加上 i<400，双重保险 */
			for(i=0; i<Storage_Bit_z && i<400; i++) {
				if(Z_abs>=0.95f*Value[i] && Z_abs<=1.05f*Value[i]) {
					Z_abs*=Proportion[i+3];
					Correct_flag=1;
				}
				if(Correct_flag==1) break;
			}
			break;
		case 2: /* 修正电容 */
			/* 循环条件加上 i<400，双重保险 */
			for(i=0; i<Storage_Bit_c && i<400; i++) {
				if(C_abs>=0.95f*Value[i+400] && C_abs<=1.05f*Value[i+400]) {
					C_abs*=Proportion[i+3+400];
					Correct_flag=1;
				}
				if(Correct_flag==1) break;
			}
			break;
		case 3: /* 修正电感 */
			/* 循环条件加上 i<400，双重保险 */
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
  * @功能：  清空内存中的所有学习数据
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
  * @brief  键盘输入数字 (移植自旧版稳定代码)
  * @功能：  通过PS2键盘输入一个浮点数，支持小数点
  * @param  num: 默认值
  * @retval 输入的数值
  */
float PS2_ReadNum(float num) {
	uint8_t count = 0;
	uint8_t dec_sign = 0;      /* 小数点标志 */
	float temp_num = 0;
	uint32_t timeout_cnt = 0;  /* 超时计数 */
	
	/* 清屏并绘制输入框 */
	LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
	OS_Rect_Draw( 332 , 96 + 64 * 4 , 750 , 96 + 64 * 5 , 1 , White );
	OS_String_Show( 332 + 16 , 96 + 64 * 4 + 16 , 32 , 1, "-> "); /* 提示符 */
	
	Ps2KeyValue = KeyValue_Null; /* 清空按键缓存 */

	while (1) {
		/* 1. 超时检测 (大约10秒无操作自动退出) */
		if(timeout_cnt > 2000) { 
			LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
			return -1.0f; /* 返回负数表示取消 */
		}
		
		/* 2. 按键处理 */
		if (Ps2KeyValue != KeyValue_Null) {
			timeout_cnt = 0; /* 有按键，重置超时 */
			
			/* --- 情况A：按下数字键 0-9 --- */
			if (Ps2KeyValue >= KeyValue_0 && Ps2KeyValue <= KeyValue_9) {
				if (dec_sign == 0) {
					/* 整数部分 */
					temp_num = temp_num * 10 + Ps2KeyValue;
				} else {
					/* 小数部分 */
					temp_num = temp_num + (float)Ps2KeyValue / pow(10, count);
					count++;
				}
				/* 实时显示 */
				OS_Num_Show( 332 + 80 , 96 + 64 * 4 + 16 , 32 , 1, temp_num , "%.6f      ");
			}
			
			/* --- 情况B：按下小数点 --- */
			else if (Ps2KeyValue == KeyValue_Point) {
				if(dec_sign == 0) { /* 只允许按一次小数点 */
					dec_sign = 1;
					count = 1;
				}
			}
			
			/* --- 情况C：按下回车 (Enter) -> 确认输入 --- */
			else if (Ps2KeyValue == KeyValue_Enter) {
				LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
				Ps2KeyValue = KeyValue_Null;
				return temp_num; /* 返回输入的正数 */
			}
			
			/* --- 情况D：按下 Backspace -> 取消输入 --- */
			else if (Ps2KeyValue == KeyValue_Back) {
				LCD_Appoint_Clear( 332 , 96 + 64 * 4 , 750 + 1 , 480 - 32 - 8 , Black );
				Ps2KeyValue = KeyValue_Null;
				return -1.0f; /* 返回负数表示取消 */
			}
			
			Ps2KeyValue = KeyValue_Null; /* 处理完按键清空 */
		}
		
		delay_ms(5); /* 注意：此处在独立输入循环中，阻塞式延时可接受 */
		timeout_cnt++;
	}
}

/* ==================== 安全加固新增函数 ==================== */
/**
  * @brief  安全的Flash读取函数
  * @功能：  检查Flash地址范围是否合法，防止读取越界
  * @param  addr: 起始地址
  * @param  buf:  数据缓冲区
  * @param  len:  数据长度（字为单位）
  * @retval 1=成功，0=失败（地址无效）
  */
uint8_t InFLASH_Read_Safe(uint32_t addr, uint32_t *buf, uint32_t len) {
    /* 检查地址和长度是否有效（仅允许访问SECTOR_10和SECTOR_11） */
    if (addr < ADDR_FLASH_SECTOR_10 || (addr + len * 4) > (ADDR_FLASH_SECTOR_11 + 1200 * 4)) {
        return 0; /* 无效地址，返回失败 */
    }
    /* 地址合法，执行读取 */
    InFLASH_Read(addr, buf, len);
    return 1; /* 成功 */
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

/* ***************************** 						END 	   	     	*****************************/