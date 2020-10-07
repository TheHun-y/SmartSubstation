// 每周波采样点数
#define POINTS 48
// 数据窗总长(包括记忆量) 4个周波
#define WINDOW 4*POINTS
#define PI 3.1415926
#define MAXSIZE 655350
#define RECORD_LENGTH 10*POINTS
#define QUEUE_LENGTH 1*POINTS
#define MAX_VALUE 65535
#define STRING_LENGTH 200
#define RECLOSE_COUNT 10*POINTS

// 配置文件读取
#define PARAM_COUNT 200
#define MAX_BUF_LEN 1024
#define MAX_KEY_LEN 64
#define MAX_VAL_LEN 256

// 母线保护最大出线数
#define MAX_LINE_NUM_220kV 24
#define MAX_BRK_NUM_500kV 36

// 测控装置
#define MAX_VOLTAGE_ARRAYSIZE 12
#define MAX_CURRENT_ARRAYSIZE 8
#define MAX_BREAKER_ARRAYSIZE 36
#define MAX_PHASORSIZE 12
#define MAX_MEA_SAMPLESIZE 24

// 端口信号数量
#define MEA_PORTSIZE_1 10
#define MEA_PORTSIZE_2 9

// 装置类型
#define ERRORTYPE -1
#define LINETYPE 1
#define BUSTYPE 2
#define BUSBRKTYPE_SIDE 3
#define BUSBRKTYPE_MID 4


// ** PT下标索引 ** 
// *** 线路测控 、母线测控-500kV边开关、母线测控-500kV中开关 ***  
#define LOCAL_VOLTAGE_A 0
#define LOCAL_VOLTAGE_B 1
#define LOCAL_VOLTAGE_C 2
#define REMOTE_VOLTAGE 3
#define LOCAL_VOLTAGE_ZERO 4
// *** 母线测控 ***  
#define LINE1_VOLTAGE_A 0
#define LINE1_VOLTAGE_B 1
#define LINE1_VOLTAGE_C 2
#define LINE2_VOLTAGE_A 3
#define LINE2_VOLTAGE_B 4
#define LINE2_VOLTAGE_C 5
#define LINE3_VOLTAGE_A 6
#define LINE3_VOLTAGE_B 7
#define LINE3_VOLTAGE_C 8
#define LINE4_VOLTAGE_A 9
#define LINE4_VOLTAGE_B 10
#define LINE4_VOLTAGE_C 11

// ** CT下标索引 ** 
// *** 线路测控 *** 
#define LINE_LOCAL_CURRENT_A 0
#define LINE_LOCAL_CURRENT_B 1
#define LINE_LOCAL_CURRENT_C 2
#define LINE_LOCAL_CURRENT_ZERO 3

// *** 母线测控-500kV边开关 、母线测控-500kV中开关 *** 
#define UPPER_CURRENT_A 0
#define UPPER_CURRENT_B 1
#define UPPER_CURRENT_C 2
#define LOWER_CURRENT_A 3
#define LOWER_CURRENT_B 4
#define LOWER_CURRENT_C 5

// ** 相量行号索引 **
#define VOLTAGE_ROW 0
#define CURRENT_ROW 1

// ** 变比索引 ** 
// *** 双侧PT/CT ***
#define RATEINDEX_LOCAL_PT 0
#define RATEINDEX_REMOTE_PT 1
#define RATEINDEX_ZEROSEQ_PT 2
#define RATEINDEX_LOCAL_CT 0
#define RATEINDEX_REMOTE_CT 1
#define RATEINDEX_ZEROSEQ_CT 2
// *** 多侧PT/CT ***
#define RATEINDEX_LINE1_PT 0
#define RATEINDEX_LINE2_PT 1
#define RATEINDEX_LINE3_PT 2
#define RATEINDEX_LINE4_PT 3

// *** 同期电压类型 ***
#define SYNVOLT_TYPE_A 0
#define SYNVOLT_TYPE_B 1
#define SYNVOLT_TYPE_C 2
#define SYNVOLT_TYPE_AB 3
#define SYNVOLT_TYPE_BC 4
#define SYNVOLT_TYPE_CA 5

// *** 母线测控装置出线数及每线相数 ***
#define MEA_BUSTYPE_PHASE_NUM 3
#define MEA_BUSTYPE_LINE_NUM 4

// 相量
typedef struct Phasor {
    double real;
    double img;
} Phasor;

typedef struct DataPackage {
    double delayTime;
    double frame[10]; // 一个port的9维数据
} DataPackage;


/**
 * 保护装置通用的数据结构--全局变量
 * sample, 本次12通道采样值
 * instVma, instVmb等数组: 瞬时值数组
 * phasor 本次相量计算结果
 * setValue数组: 通用整定值数组,具体每一个元素所代表的的整定值含义见各个保护的说明
 * relayTime数组, 通用延时数组
 * relayFlag数组, 通用跳闸标记
 */

typedef struct Device {
    // 装置是否启用
    int deviceEnable;

    char globalFileName[STRING_LENGTH];
    char deviceFileName[STRING_LENGTH];
    char deviceName[STRING_LENGTH];
    // 采样计数器, 用于仿真10次, 保护装置跑一次
    int sampleCount1; // 保护装置
    int sampleCount2; // 交换机


    double time;
    // 母线电压(3) + 线路电压电流(6+6)+零序电流1
    double sample[16];
    // 断路器状态采样,合位为1,断开为0
    int sampleBrkStatus[6];
    int sampleManBrkStatus[3];
    int brkStatus[POINTS][6];
    int manBrkStatus[POINTS][3];

    // 交换机延时范围(最大延时)
    double switch1DelayMin;
    double switch1DelayMax;
    double switch2DelayMin;
    double switch2DelayMax;

    DataPackage switchQueue1[QUEUE_LENGTH];
    DataPackage switchQueue2[QUEUE_LENGTH];

    int queueLength1;
    int queueLength2;

    double switchPort1[10];
    double switchPort2[9];

    // 具体长度根据录波长度确定
    double instTime[RECORD_LENGTH];
    double instVma[RECORD_LENGTH];
    double instVmb[RECORD_LENGTH];
    double instVmc[RECORD_LENGTH];
    double instIma[RECORD_LENGTH];
    double instImb[RECORD_LENGTH];
    double instImc[RECORD_LENGTH];
    double instVna[RECORD_LENGTH];
    double instVnb[RECORD_LENGTH];
    double instVnc[RECORD_LENGTH];
    double instIna[RECORD_LENGTH];
    double instInb[RECORD_LENGTH];
    double instInc[RECORD_LENGTH];

    double instVmaBus[RECORD_LENGTH];
    double instVmbBus[RECORD_LENGTH];
    double instVmcBus[RECORD_LENGTH];

    double diffFiltVma[WINDOW];
    double diffFiltVmb[WINDOW];
    double diffFiltVmc[WINDOW];
    double diffFiltIma[WINDOW];
    double diffFiltImb[WINDOW];
    double diffFiltImc[WINDOW];
    double diffFiltVna[WINDOW];
    double diffFiltVnb[WINDOW];
    double diffFiltVnc[WINDOW];
    double diffFiltIna[WINDOW];
    double diffFiltInb[WINDOW];
    double diffFiltInc[WINDOW];
    
    double filterVma[WINDOW];
    double filterVmb[WINDOW];
    double filterVmc[WINDOW];
    double filterIma[WINDOW];
    double filterImb[WINDOW];
    double filterImc[WINDOW];
    double filterVna[WINDOW];
    double filterVnb[WINDOW];
    double filterVnc[WINDOW];
    double filterIna[WINDOW];
    double filterInb[WINDOW];
    double filterInc[WINDOW];

    double filterVmaBus[WINDOW];
    double filterVmbBus[WINDOW];
    double filterVmcBus[WINDOW];

    // 记忆量
    Phasor memoryVma[POINTS], memoryVmb[POINTS], memoryVmc[POINTS];
    Phasor memoryIma[POINTS], memoryImb[POINTS], memoryImc[POINTS];
    Phasor memoryVna[POINTS], memoryVnb[POINTS], memoryVnc[POINTS];
    Phasor memoryIna[POINTS], memoryInb[POINTS], memoryInc[POINTS];

    // 运行参数
    double ratedSideIVoltage, ratedSideICurrent; // 一次侧额定线电压, 相电流？
    double ratedSideIIVoltage, ratedSideIICurrent; // 二次侧额定线电压, 相电流？
    //double ratedBetweenVoltage, ratedBetweenCurrent; // 额定相间电压, 线电流
    double capacityCurrent; // 电容电流

    // 同期合闸相关
    Phasor prevBus;
    Phasor prevLine;
    double prevTime;
    // 计算合闸相角
    double closeAngle;

    // 线路参数
    double lineZ1; // 正序阻抗
    double lineZ1Angle; // 线路正序灵敏角
    double lineZ2; // 负序阻抗
    double lineZ0; // 零序阻抗
    double lineZ0Angle; // 线路零序灵敏角
    double lineC1; // 正序容抗
    double lineC0; // 零序容抗
    double lineLength; // 线路长度
    double localReactor; // 本侧电抗器阻抗定值
    double localSmallReactor; // 本侧小电抗器阻抗定值
    double offsideReactor; // 对侧电抗器阻抗定值
    double offsideSmallReactor; // 对侧小电抗器阻抗定值
    double KZ; // 零序补偿系数

    // 线路保护控制字：1-投入，0-退出
    int currentDiffEnable;	            //纵联差动保护
    int CTbreakBlockDiffEnable;	        //CT断线闭锁差动
    int voltFromLinePTEnable;	        //电压取线路PT电压
    int powerSwingBlockEnable;	        //振荡闭锁元件
    int distanceIEnable;	            //距离保护I段
    int distanceIIEnable;	            //距离保护II段
    int distanceIIIEnable;          	//距离保护III段
    int zeroSequenceEnable;	            //零序电流保护
    int directionZeroSequenceIIIEnable;	//零序过流III段经方向
    int brkThreePhaseMode;	            //三相跳闸方式
    int currentCompensationEnable;	    //电流补偿
    int loadLimitDistanceEnable;	    //负荷限制距离
    int spRecloseMode;	                //单相重合闸
    int tpRecloseMode;	                //三相重合闸
    int banRecloseMode;	                //禁止重合闸
    int stopRecloseMode;                //停用重合闸
    int relayIIBlockRecloseEnable;      //II段保护闭锁重合闸
    int mpFaultBlockRecloseEnable;	    //多相故障闭锁重合闸
    int checkSynMode;	                //重合闸检同期方式
    int checkVoltMode;	                //重合闸检无压方式
    int speedupDistanceIIEnable;	    //三重加速距离保护II段
    int speedupDistanceIIIEnable;	    //三重加速距离保护III段
    int remoteTripControlEnable;        //远跳经本侧控制	
	int spTWJRecloseEnable;             //单相TWJ启动重合闸
	int tpTWJRecloseEnable;             //三相TWJ启动重合闸
    int deltaDistanceEnable;            //工频变化量距离投运
    int comTripleBlockEnable;           //沟三闭重开入
    int brkChargeStatus;                //断路器充电状态：0-已满，1-未满


    // 线路保护整定值列表
    double lineStartSetValue[3]; // 线路启动元件整定值: 0:电流突变量整定值  1:零序电流整定值  2:差动动作电流定值

    double p2gDistanceSetValue[3]; // 接地距离定值
    double p2gDistanceTimeSetValue[3];	// 接地距离时间  注：下标 1-II段时间，2-III段时间，**0未使用**
    double p2gDistanceDevAngle; // 接地距离偏移角

    double p2pDistanceSetValue[3]; // 相间距离定值
    double p2pDistanceTimeSetValue[3];	// 相间距离时间  注：下标 1-II段时间，2-III段时间，**0未使用**
    double p2pDistanceDevAngle; //	相间距离偏移角

    double loadLimitSetValue; //负荷限制电阻定值

    double zeroSequenceSetValue[3];	// 零序过流定值  注：0-II段，1-III段，2-加速段
    double zeroSequenceTimeSetValue[2];	// 零序过流时间  注：0-II段，1-III段

    double deltaImpedance; // 工频变化量阻抗 

    double psbCurrentSetValue; // 振荡闭锁过流定值

    double ptBreakOverCurrentSetValue; // PT断线相过流定值
    double ptBreakZeroSeqSetValue; // PT断线零序过流定值
    double ptBreakOcTimeSetValue; // PT断线过流时间

    double ctBreakDiffCurrentSetValue; // CT断线差流定值

    // 其它定值
    double psbOpenPhaseSetValue[2]; // 非全相振闭开放元件定值：0-无流门槛值；1-工频变化量电流的门槛值
    double returnSetTime; // 复归时间
    double PTBreakJudgeTimeSetValue[2];
    double CTBreakJudgeTimeSetValue[2];

    
    // 保护启动标志
    int startFlag;
    // 保护启动时间
    double startTime;

    //相量上一个采样点的计算值记忆
    Phasor prePhasor[12];

    // 相量实时计算值
    Phasor phasor[12];
    Phasor busPhasor[3];

    // 电流差动保护
    double currentDiffTimeSetValue[2];
    int currentDiffTripFlag[4]; //综合判位，3-零差保护
    int currentDiffSelfTripFlag[3];//自判位
    int currentDiffUnionFlag[3];// 对侧差动信号

    // 工频变化量距离
    int deltaDistanceTripFlag[3];
    // 工作电压记忆量
    double operateVoltageP2PMemory[3][POINTS];
    double operateVoltageP2GMemory[3][POINTS];
    // 半波积分值门槛
    double deltaDistanceP2GSetValue[3];
    double deltaDistanceP2PSetValue[3];
    // 半波积分累加
    double halfWaveP2PSum[3];
    double halfWaveP2GSum[3];
    double halfWaveP2PSumCnt[3];
    double halfWaveP2GSumCnt[3];

    // 负荷限制
    int loadLimitTripFlag[3];

    // 距离保护
    int distanceITripFlag[3];
    int distanceIITripFlag[3];
    int distanceIIITripFlag[3];

    // 零序电流保护
    int zeroSeqStartFlag;
    int zeroSequenceTripFlag[3]; // 0-II段动作；1-III段动作；2-加速段动作；

    // PT断线保护
    int PTBreakPreFlag; // 预标志
    double PTBreakTime;
    double PTBreakOcTime;
    int PTBreakFlag;
    double PTBreakReturnTime;
    int PTBreakTripFlag[2];

    // CT断线
    int CTBreakPreFlag[2];
    double CTBreakTime[2];
    int CTBreakFlag;

    // 外接零序
    Phasor zeroSeqCurrentOUT;
    double instZeroSeqI[RECORD_LENGTH], filterZeroSeqI[WINDOW], diffFiltZeroSeqI[WINDOW];

    // 过电流保护
    int overCurrentEnable;
    double overCurrentSetValue[3];
    double overCurrentTimeSetValue[4];
    int overCurrentTripFlag[3];

    // 与母线断路器保护的交互：电流变化量（只交互标志位）
    double halfWaveDeltaCurrentValue; // 门槛值
    double memoryCurrent[3][POINTS]; // 瞬时记忆量
    double deltaCurrentMemory[3][24]; // 滑动窗记忆半周波的变化量电流
    int deltaCurrentFlag; // 三相合一，取或

    int busSynFlag; // 1-与母线同步，0-滞后母线一个步长

    // 选相标志
    int tripPhase[3];
    // 选相无效标记及时间
    int invalidChooseFlag; // 0-有效，1-无效
    double invalidChooseStartTime; // 记下选相无效累计时间

    // 单跳失败
    int singleTripFailFlag;
    double singleTripFailStartTime;

    // 单跳标志
    double singleNotReturnTimeSetValue;
    int singleTripFlag;
    double singleTripTime;

    // 非全相运行
    int openPhasePreFlag[3];
    double openPhaseTime[3];
    int openPhaseFlag[3];

    // 沟三跳
    int comTripleTripFlag;

    // 保护跳闸动作标志
    double tripFlag[3];
    // 跳闸信号（监测上升沿，持续发出）
    double tripStartTime;
    int tripSignalOutput[3];
    // 结合断路器状态/手动状态/保护状态 --> 出口
    double outRes[3];

    // 手动合闸标志位
    int manCloseFlag[3];
    // 手动跳闸标志位
    int manTripFlag[3];
    // 手动合闸时间
    double manCloseTime;
    int manCloseOnFault;

    // 加速II段标志, 0不加速, 1加速
    int fastII;
    // 跳闸状态标志
    int tripState;
    // 跳闸次数
    int tripTimes;
    // 重合闸闭锁标志位, 0不闭锁, 1闭锁
    int reCloseBlocked;
    // 重合闸计数
    int reCloseCount;
    // 重合闸次数
    int reCloseTimes;

    double test[3];







    // hash散列数组
    int notYetFlag[MAXSIZE];



    /**
     * 以下为母线保护用到的数据结构
     *
     struct Device* busRange[20];
     // 标志位, 实际挂到该母线上的线路
     // 母线上实际出线数量
     int busNum;

     // 母线电流差动保护整定值
     double busCurrentDiffSetValue;
     double busCurrentDiffTimeSetValue;

     // 线路-母线关联矩阵(初始化一次)
     int busTopo[24][4];
     // 运行方式识别矩阵(每次调用时更新)
     int busModeStatus[24][4];
     // 母联开关状态
     int busTieBrkStatus[2];
     // 分段开关状态
     int busSecBrkStatus[2];
     // 旁路隔离开关状态
     int bypassBusStatus[4];
     // 旁母-线路隔离开关状态
     int bypassSwitchStatus[24];
    


     // 母线接线方式
     int busMode;
     int bypassBusMode;*/

     // 作500kV母线保护采样装置时的数据结构
     double memory3I0; // 记忆稳态的3I0
     int memoryI0Flag;
     int sampleBrkFailureFlag; // 失灵保护就地判据
     double memoryI0StartTime;
     double sampleInstDeltaSum;
     double instDeltaStartTime;



   
} Device;

typedef struct BusDevice {
    // 装置是否启用
    int deviceEnable;

    char globalFileName[STRING_LENGTH];
    char deviceFileName[STRING_LENGTH];
    char deviceName[STRING_LENGTH];
    // 采样计数器, 用于仿真10次, 保护装置跑一次
    int sampleCount1; // 保护装置
    int sampleCount2; // 交换机


    double time;

    // 交换机延时范围(最大延时)
    double switch1DelayMin;
    double switch1DelayMax;
    double switch2DelayMin;
    double switch2DelayMax;

    DataPackage switchQueue1[QUEUE_LENGTH];
    DataPackage switchQueue2[QUEUE_LENGTH];

    int queueLength1;
    int queueLength2;

    double switchPort1[9];
    double switchPort2[9];


    // 跳闸状态标志
    int tripState;
    // 跳闸次数
    int tripTimes;
    // 重合闸计数
    int reCloseCount;
    // 重合闸次数
    int reCloseTimes;





    // hash散列数组
    int notYetFlag[MAXSIZE];



    /**
     * 以下为母线保护用到的数据结构
     
     struct Device* busRange[20];
     // 标志位, 实际挂到该母线上的线路
     // 母线上实际出线数量
     int busNum;

     // 母线电流差动保护整定值
     double busCurrentDiffSetValue;
     double busCurrentDiffTimeSetValue;
     */

    // 控制字列表
    // 220kV
     int busDiffEnable; // 差动保护投入
     int busIndependentOpEnable[4]; // 母联/分段分列运行压板
     int busBranchFailureEnable; // 支路断路器失灵保护控制字
     int LbranchStartFailureEnable[80]; // 线路支路X启动失灵开入（每个支路4维）： 0-三相启动失灵开入；1/2/3-A/B/C相启动失灵开入****
     int TbranchStartFailureEnable[4]; // 变压器支路X启动失灵开入（每个支路1维）： 0-三相启动失灵开入****
     int LbranchUnlockFailureEnable[29]; // 线路支路X解除复压闭锁控制字（28-ML1带路运行带旁母)
     int bypassBusMode; // 0-旁母未投运，1-双母运行I母带旁路，2-双母运行II母带旁路
     int busSecManualClose[2]; // 分段手合开入
     int busIncompletePhaseEnable[4]; // 非全相保护投入
     int busBrkChargeOcIEnable[4]; // 充电过流I段
     int busBrkChargeOcIIEnable[4]; // 充电过流I段
     int busBrkChargeOcZeroEnable[4]; // 充电过流零序
     int brkFailureTripEnable; //500kV失灵经母差跳闸控制字

    // 定值列表
     double side1VoltSetValue; // 一次侧额定电压
     double side2VoltSetValue; // 二次侧额定电压
     double side1CurrSetValue; // 一次侧额定电流
     double side2CurrSetValue; // 二次侧额定电流
     double busDiffSetValue; // 母线差动启动电流定值
     double busCTBreakBlockSetValue; // CT断线闭锁定值
     double busCTBreakWarnSetValue; // CT断线告警定值
     double busBrkFailureSetValue[4]; // 母联分段失灵保护电流定值
     double busBrkFailureSetTime[4];
     // 断路器失灵保护部分定值
     double failLowVoltBlockSetValue;
     double failZeroVoltBlockSetValue;
     double failNegVoltBlockSetValue;
     double failPhaseCurrentSetValue;
     double failZeroCurrentSetValue;
     double failNegCurrentSetValue;
     double failTimeSetI;
     double failTimeSetII;

     double chargeOcISetValue;
     double chargeOcISetTime;
     double chargeOcIISetValue;
     double chargeOcIISetTime;
     double chargeZeroCurrentSetValue;

     double openPhaseZeroCurrSetValue;
     double openPhaseNegCurrSetValue;
     double openPhaseSetTime;



     // 线路-母线关联矩阵(初始化一次)
     int busTopo[36][4]; // 最后一行（29,下标28）代表旁路运行时的母联电流极性
     int busTopo_500kV[36][2]; // 500kV母线接线方式
     // 运行方式识别矩阵(每次调用时更新)
     int busModeStatus[29][4];
     int writeInFlag;
     // 母联开关状态0/1，分段开关状态2/3
     int busBrkStatus[12];
     int busBrkStatusMemory[12];
     // 母联合闸开入标志位
     int busBrkManualFlag[4]; // 尚未增加一次侧接口（2020-05-20） // 已增加一次侧接口，安排在port7/8的后两维（2020-06-24）
     int busBrkManualFlagMemory[4];
     // 旁路隔离开关状态
     int bypassBusStatus[4];
     // 旁母-线路隔离开关状态
     int bypassSwitchStatus[24];

     // 母线接线方式
     int busMode;
     
     // 各母线投运状态
     int busRunStatus[4];

     // 母线相关的互感器电量: m/n 代表两个断路器
     // 采样值
     double busSample[24];
     // 瞬时值
     double busInstTime[RECORD_LENGTH];
     double busTieInstVma[RECORD_LENGTH]; double busTieInstVmb[RECORD_LENGTH]; double busTieInstVmc[RECORD_LENGTH];
     double busTieInstIma[RECORD_LENGTH]; double busTieInstImb[RECORD_LENGTH]; double busTieInstImc[RECORD_LENGTH];
     double busTieInstVna[RECORD_LENGTH]; double busTieInstVnb[RECORD_LENGTH]; double busTieInstVnc[RECORD_LENGTH];
     double busTieInstIna[RECORD_LENGTH]; double busTieInstInb[RECORD_LENGTH]; double busTieInstInc[RECORD_LENGTH];

     double busSecInstVma[RECORD_LENGTH]; double busSecInstVmb[RECORD_LENGTH]; double busSecInstVmc[RECORD_LENGTH];
     double busSecInstIma[RECORD_LENGTH]; double busSecInstImb[RECORD_LENGTH]; double busSecInstImc[RECORD_LENGTH];
     double busSecInstVna[RECORD_LENGTH]; double busSecInstVnb[RECORD_LENGTH]; double busSecInstVnc[RECORD_LENGTH];
     double busSecInstIna[RECORD_LENGTH]; double busSecInstInb[RECORD_LENGTH]; double busSecInstInc[RECORD_LENGTH];

     double busTieDiffFiltVma[WINDOW]; double busTieDiffFiltVmb[WINDOW]; double busTieDiffFiltVmc[WINDOW];
     double busTieDiffFiltIma[WINDOW]; double busTieDiffFiltImb[WINDOW]; double busTieDiffFiltImc[WINDOW];
     double busTieDiffFiltVna[WINDOW]; double busTieDiffFiltVnb[WINDOW]; double busTieDiffFiltVnc[WINDOW];
     double busTieDiffFiltIna[WINDOW]; double busTieDiffFiltInb[WINDOW]; double busTieDiffFiltInc[WINDOW];

     double busSecDiffFiltVma[WINDOW]; double busSecDiffFiltVmb[WINDOW]; double busSecDiffFiltVmc[WINDOW];
     double busSecDiffFiltIma[WINDOW]; double busSecDiffFiltImb[WINDOW]; double busSecDiffFiltImc[WINDOW];
     double busSecDiffFiltVna[WINDOW]; double busSecDiffFiltVnb[WINDOW]; double busSecDiffFiltVnc[WINDOW];
     double busSecDiffFiltIna[WINDOW]; double busSecDiffFiltInb[WINDOW]; double busSecDiffFiltInc[WINDOW];
     
     double busTieFiltVma[WINDOW]; double busTieFiltVmb[WINDOW]; double busTieFiltVmc[WINDOW];
     double busTieFiltIma[WINDOW]; double busTieFiltImb[WINDOW]; double busTieFiltImc[WINDOW];
     double busTieFiltVna[WINDOW]; double busTieFiltVnb[WINDOW]; double busTieFiltVnc[WINDOW];
     double busTieFiltIna[WINDOW]; double busTieFiltInb[WINDOW]; double busTieFiltInc[WINDOW];

     double busSecFiltVma[WINDOW]; double busSecFiltVmb[WINDOW]; double busSecFiltVmc[WINDOW];
     double busSecFiltIma[WINDOW]; double busSecFiltImb[WINDOW]; double busSecFiltImc[WINDOW];
     double busSecFiltVna[WINDOW]; double busSecFiltVnb[WINDOW]; double busSecFiltVnc[WINDOW];
     double busSecFiltIna[WINDOW]; double busSecFiltInb[WINDOW]; double busSecFiltInc[WINDOW];

     // 记忆量
     int memoryRecordFlag;

     Phasor busTieMemVma[POINTS], busTieMemVmb[POINTS], busTieMemVmc[POINTS];
     Phasor busTieMemVna[POINTS], busTieMemVnb[POINTS], busTieMemVnc[POINTS];
     Phasor busSecMemVma[POINTS], busSecMemVmb[POINTS], busSecMemVmc[POINTS];
     Phasor busSecMemVna[POINTS], busSecMemVnb[POINTS], busSecMemVnc[POINTS];
     Phasor busTieMemIma[POINTS], busTieMemImb[POINTS], busTieMemImc[POINTS];
     Phasor busTieMemIna[POINTS], busTieMemInb[POINTS], busTieMemInc[POINTS];
     Phasor busSecMemIma[POINTS], busSecMemImb[POINTS], busSecMemImc[POINTS];
     Phasor busSecMemIna[POINTS], busSecMemInb[POINTS], busSecMemInc[POINTS];

     // 相量值
     Phasor busPhasor[48]; //后24位是前24位乘-1（反向）
     Phasor busPrePhasor[48];

     // 启动元件标志位 :0 作为总启动标志位，1/2/3分别作为电压、电流工频变化量及差流启动标志位
     int busStartFlag[4];
     int busStartTimeCount[3];
     double busStartTime[4];

     // 母联/分段跳闸标志位
     int busBrkTripFlag[4];
     double busBrkTripTime[4]; // CT断线使用
     // 母线跳闸标志位
     int busTripFlag[4];

     // 保护使用的采样点时间
     double relayTime;
     int busDiffDelayFlag;

     // 母线常规差动保护定值及标志位
     //double busDiffSetValue[10];
     //double busDiffTimeSetValue[10];
     double busDiffDcTripTimeCount;
     // 跳闸标志
     int busDiffTripFlag[5]; // 0:大差
     double busDiffTripTime[5]; // 0:大差

     int busDiffPhaseTripFlag[5][3]; // 0:大差


     // 母线工频变化量差动保护及标志位
     double busDeltaDiffSetValue[10];
     double busDeltaDiffTimeSetValue[10];
     // 跳闸标志
     int busDeltaDiffTripFlag[4];
     //int busDeltaDiffBrkTripFlag[4];
     double busDeltaDiffTripTime[4];

     int busDeltaDiffPhaseTripFlag[4][3];
     
     // 抗饱和1时间整定值
     double antiSat1TimeSetValue;
     double blcdStartTime[5];
     // 1-饱和；0-开放
     int antiSat1Flag[5]; // 0:dc; 1/2/3/4:xc
     int antiSat1FinalOutTag[5]; // 1-代表已完成抗饱和1检测，0-未完成

     // 抗饱和2整定值
     double antiSat2SetValue;
     int antiSat2Flag[5];

     // 电压闭锁标志位
     int voltBlockFlag[4];

     // 充电闭锁标志位
     int chargeBlockFlag[4];
     double chargeBlockTime[4];
     // TWJ1-0返回时间
     double TWJReturnTime[4];
     // 母联合闸开入正翻转时间
     double manualFlagChangeTime[4];

     // 后备保护标志位 I/II段
     int busDiffBackupTripFlag[2];

     // 死区保护
     double busCloseDeadZoneTime[4]; // 只用了前两个（母联）
     double busOpenDeadZoneTime[4];
     int busBrkDeadZoneFlag[4]; // 1-退出计算

     // 母联失灵标志
     int busTieFailureFlag[4]; // 母联I失灵跳1/2母，母联II失灵跳1/2母
     // 分段失灵标志
     int busSecFailureStartFlag[4]; // 启动
     int busSecFailureFlag[4];

	 // 母联充电过流标志
     int busTieOverCurrentStartFlag[6];//启动    
	 int busTieOverCurrentFlag[6];    
	 // 分段充电过流标志
     int busSecOverCurrentStartFlag[6];
	 int busSecOverCurrentFlag[6];     

	 int busOpenphaseEnable[8]; // 母联分段非全相控制字+投入软压板：01对应母联I的控制字+软压板；23对应母联II；45对应分段I；67对应分段II    
	 int busTieOpenphaseFlag[2];    
	 int busSecOpenphaseFlag[2];  

     double busTieFailueStartTime[4];//母联失灵起始时间，用于延时动作时进行比较          /////FZY
	 double busSecFailueStartTime[4];//分段失灵起始时间，用于延时动作时进行比较          /////FZY
	 double busTieOverCurrentStartTime[2];//母联过流起始时间，用于延时动作时进行比较     /////FZY
	 double busSecOverCurrentStartTime[2];//分段过流起始时间，用于延时动作时进行比较     /////FZY
	 double busTieOpenphaseStartTime[2];//母联非全相起始时间，用于延时动作时进行比较     /////FZY
	 double busSecOpenphaseStartTime[2];//分段非全相起始时间，用于延时动作时进行比较      /////FZY

     // 断路器失灵保护：线路变化量差流元件动作标志/时间（共20维-最大）
     int LbranchDeltaFlag[20];
     double LbranchDeltaTime[20];

     int busBranchFailureTripFlagI[4]; // 时限1，标记连接的母线，通过最终跳闸处理成对应母联/分段的跳闸位
     int busBranchFailureTripFlagII[4]; // 时限2
     double busBranchFailureTripTime[24]; // 每个支路使用一维

     // PT断线标志位
     int busPTBreakFlag[4];
     double busPTBreakStartTime[4][3];
     double busPTBreakReturnTime[4];

     // CT断线标志位
     int busCTBreakFlag[4][3];
     int busCTBreakWarnFlag[4][3];
     double busCTBreakStartTime[4][2][3]; // 分别标记两个标志位变化时间
     double busCTBreakWarnStartTime[4][2][3];

     // 500kV失灵保护启动时间
     double brkFailureStartTime_500kV[36];

     double test;
     double testDC;
     double testPhasor[2];
     double test_2, test_3;

   
} BusDevice;




typedef struct tranDataPackage{
    double delayTime;
    double frame[12]; // 一个port的9维数据
} tranDataPackage;


typedef struct tranDevice{
    // 装置是否启用
    int deviceEnable;

    char globalFileName[STRING_LENGTH];
    char deviceFileName[STRING_LENGTH];
    char deviceName[STRING_LENGTH];
    // 采样计数器, 用于仿真10次, 保护装置跑一次
    int sampleCount1; // 保护装置
    int sampleCount2; // 交换机


    double time;
    double tranSample[24];

    double Idt;// 差流
    double Idta, Idtb, Idtc;


    // 交换机延时范围(最大延时)
    double switch1DelayMin;
    double switch1DelayMax;
    double switch2DelayMin;
    double switch2DelayMax;

    tranDataPackage switchQueue1[QUEUE_LENGTH];
    tranDataPackage switchQueue2[QUEUE_LENGTH];

    int queueLength1;
    int queueLength2;

    double switchPort1[12];
    double switchPort2[12];

    // 具体长度根据录波长度确定
    double instTime[RECORD_LENGTH];

    double instVa1[RECORD_LENGTH];
    double instVb1[RECORD_LENGTH];
    double instVc1[RECORD_LENGTH];
    double instIa1[RECORD_LENGTH];
    double instIb1[RECORD_LENGTH];
    double instIc1[RECORD_LENGTH];
    double instVa2[RECORD_LENGTH];
    double instVb2[RECORD_LENGTH];
    double instVc2[RECORD_LENGTH];
    double instIa2[RECORD_LENGTH];
    double instIb2[RECORD_LENGTH];
    double instIc2[RECORD_LENGTH];
    double instVa3[RECORD_LENGTH];
    double instVb3[RECORD_LENGTH];
    double instVc3[RECORD_LENGTH];
    double instIa3[RECORD_LENGTH];
    double instIb3[RECORD_LENGTH];
    double instIc3[RECORD_LENGTH];
    double instVa4[RECORD_LENGTH];
    double instVb4[RECORD_LENGTH];
    double instVc4[RECORD_LENGTH];
    double instIa4[RECORD_LENGTH];
    double instIb4[RECORD_LENGTH];
    double instIc4[RECORD_LENGTH];

    double filterVa1[WINDOW];
    double filterVb1[WINDOW];
    double filterVc1[WINDOW];
    double filterIa1[WINDOW];
    double filterIb1[WINDOW];
    double filterIc1[WINDOW];
    double filterVa2[WINDOW];
    double filterVb2[WINDOW];
    double filterVc2[WINDOW];
    double filterIa2[WINDOW];
    double filterIb2[WINDOW];
    double filterIc2[WINDOW];
    double filterVa3[WINDOW];
    double filterVb3[WINDOW];
    double filterVc3[WINDOW];
    double filterIa3[WINDOW];
    double filterIb3[WINDOW];
    double filterIc3[WINDOW];
    double filterVa4[WINDOW];
    double filterVb4[WINDOW];
    double filterVc4[WINDOW];
    double filterIa4[WINDOW];
    double filterIb4[WINDOW];
    double filterIc4[WINDOW];

    double filterVa1m[WINDOW];
    double filterVb1m[WINDOW];
    double filterVc1m[WINDOW];
    double filterIa1m[WINDOW];
    double filterIb1m[WINDOW];
    double filterIc1m[WINDOW];
    double filterVa2m[WINDOW];
    double filterVb2m[WINDOW];
    double filterVc2m[WINDOW];
    double filterIa2m[WINDOW];
    double filterIb2m[WINDOW];
    double filterIc2m[WINDOW];
    double filterVa3m[WINDOW];
    double filterVb3m[WINDOW];
    double filterVc3m[WINDOW];
    double filterIa3m[WINDOW];
    double filterIb3m[WINDOW];
    double filterIc3m[WINDOW];
    double filterVa4m[WINDOW];
    double filterVb4m[WINDOW];
    double filterVc4m[WINDOW];
    double filterIa4m[WINDOW];
    double filterIb4m[WINDOW];
    double filterIc4m[WINDOW];


    // 记忆量
    Phasor memoryVa1[POINTS], memoryVb1[POINTS], memoryVc1[POINTS];
    Phasor memoryIa1[POINTS], memoryIb1[POINTS], memoryIc1[POINTS];
    Phasor memoryVa2[POINTS], memoryVb2[POINTS], memoryVc2[POINTS];
    Phasor memoryIa2[POINTS], memoryIb2[POINTS], memoryIc2[POINTS];
    Phasor memoryVa3[POINTS], memoryVb3[POINTS], memoryVc3[POINTS];
    Phasor memoryIa3[POINTS], memoryIb3[POINTS], memoryIc3[POINTS];
    Phasor memoryVa4[POINTS], memoryVb4[POINTS], memoryVc4[POINTS];
    Phasor memoryIa4[POINTS], memoryIb4[POINTS], memoryIc4[POINTS];

    Phasor halfmemoryVa1[POINTS], halfmemoryVb1[POINTS], halfmemoryVc1[POINTS];
    Phasor halfmemoryIa1[POINTS], halfmemoryIb1[POINTS], halfmemoryIc1[POINTS];
    Phasor halfmemoryVa2[POINTS], halfmemoryVb2[POINTS], halfmemoryVc2[POINTS];
    Phasor halfmemoryIa2[POINTS], halfmemoryIb2[POINTS], halfmemoryIc2[POINTS];
    Phasor halfmemoryVa3[POINTS], halfmemoryVb3[POINTS], halfmemoryVc3[POINTS];
    Phasor halfmemoryIa3[POINTS], halfmemoryIb3[POINTS], halfmemoryIc3[POINTS];
    Phasor halfmemoryVa4[POINTS], halfmemoryVb4[POINTS], halfmemoryVc4[POINTS];
    Phasor halfmemoryIa4[POINTS], halfmemoryIb4[POINTS], halfmemoryIc4[POINTS];

    Phasor freVa1[POINTS], freVa2[POINTS], freVa3[POINTS]; //频率计算记忆量
    double fre1, fre2, fre3; //频率

    double n1, n2, n3; //过励磁倍数


//    // 运行参数

    double ratedV10, ratedV20, ratedV30;
    double ratedI10, ratedI20, ratedI30;
    double ratedV1, ratedV2, ratedV3;
    double ratedI1, ratedI2, ratedI3;
    double pt1, pt2, pt3;
    double ct1, ct2, ct3;
    double zph, znh, zpv, znv;
    double kh, kv;

    double Ida_base, Idb_base, Idc_base;
    double Ira_base, Irb_base, Irc_base;


    //保护控制字
    double startEnable[20];//主保护控制字
    double backEnable1[20];//高后备控制字
    double backEnable2[20];//中后备控制字
    double backEnable3[20];//低后备控制字
//    int startEnable[20];//主保护控制字
//    int backEnable1[20];//高后备控制字
//    int backEnable2[20];//中后备控制字
//    int backEnable3[20];//低后备控制字

    // 保护启动标志
    int lonDiffStartFlag;
    int deltaDiffStartFlag;
    int zeroDiffStartFlag;
    int splitDiffStartFlag;
    int lowDiffStartFlag;
    int impStartFlag_h;
    int impStartFlag_v;
    int splitStartFlag;
    int flagh, flagv;
    int zeroCurStartFlagh, zeroCurStartFlagv, zeroCurStartFlagt;
    int phaCurStartFlagh, phaCurStartFlagv, phaCurStartFlagl1, phaCurStartFlagl2;


    double test[10];

    // 保护启动时间
    double startTime[20];
    double backTime[20];
    double impTimeh, impTimev;
    double time_seth, time_setv;
    double compSet1[20];
    double compSet2[20];

    // 相量实时计算值
    Phasor phasor[24];
    Phasor halfphasor[24];
    Phasor basePhasor[24];
    Phasor secPhasor[24];
    Phasor thrPhasor[24];
    Phasor fifPhasor[24];



    double halfIa1[POINTS], halfIb1[POINTS], halfIc1[POINTS];
    double halfIa2[POINTS], halfIb2[POINTS], halfIc2[POINTS];
    double halfIa3[POINTS], halfIb3[POINTS], halfIc3[POINTS];
    double halfIa1m[POINTS], halfIb1m[POINTS], halfIc1m[POINTS];
    double halfIa2m[POINTS], halfIb2m[POINTS], halfIc2m[POINTS];
    double halfIa3m[POINTS], halfIb3m[POINTS], halfIc3m[POINTS];


    // 启动元件整定值: 0:电流突变量整定值  1:零序电流整定值  ...
    double tranStartSetValue[20];
    double zeroCur1[10], zeroCur2[10], zeroTime1[10], zeroTime2[10];
    double zeroTrip;

    double overCur3, overCur4;
    double lowVolt3, lowVolt4;
    double overCur_set3, overCur_set4;
    double lowVol_set3, lowVol_set4;
    double lowTimeSet3, lowTimeSet4;



    // 保护跳闸动作标志
    int mainFlag;
    double tranProRes[12];
    double tranTripSignal[12];
    // 结合断路器状态/手动状态/保护状态 --> 出口
    double tranoutRes[12];

    // hash散列数组
    int tranNotYetFlag[MAXSIZE];


}tranDevice;

typedef struct meaDevice {
    // 使能位
    int deviceEnable;
    // 装置类型
    int deviceType;
    // 电压、电流信号维数
    int voltSize;
    int currSize;
    
    // 装置信息
    char globalFileName[STRING_LENGTH];
    char deviceFileName[STRING_LENGTH];
    char deviceName[STRING_LENGTH];

    // 采样计数器, 用于仿真10次, 保护装置跑一次
    int sampleCount1; // 保护装置
    int sampleCount2; // 交换机

    // 时戳
    double time;

    // 延时模块
    // 交换机延时范围(最大延时)
    double switch1DelayMin;
    double switch1DelayMax;
    double switch2DelayMin;
    double switch2DelayMax;

    DataPackage switchQueue1[QUEUE_LENGTH];
    DataPackage switchQueue2[QUEUE_LENGTH];

    int queueLength1;
    int queueLength2;

    double switchPort1[MEA_PORTSIZE_1];
    double switchPort2[MEA_PORTSIZE_2];

    // 整定值
    double priPTSetValue[MAX_VOLTAGE_ARRAYSIZE];
    double secPTSetValue[MAX_VOLTAGE_ARRAYSIZE];
    double priCTSetValue[MAX_CURRENT_ARRAYSIZE];
    double secCTSetValue[MAX_CURRENT_ARRAYSIZE];


    // 输入信号
    // 采样值
    double sample[MAX_MEA_SAMPLESIZE];

    // 瞬时值
    double instTime[RECORD_LENGTH];
    double priMeaInstVolt[MAX_VOLTAGE_ARRAYSIZE][RECORD_LENGTH];
    double priMeaInstCurr[MAX_CURRENT_ARRAYSIZE][RECORD_LENGTH];
    double secMeaInstVolt[MAX_VOLTAGE_ARRAYSIZE][RECORD_LENGTH];
    double secMeaInstCurr[MAX_CURRENT_ARRAYSIZE][RECORD_LENGTH];

    // 差分滤波
    double priMeaDiffVolt[MAX_VOLTAGE_ARRAYSIZE][WINDOW];
    double priMeaDiffCurr[MAX_CURRENT_ARRAYSIZE][WINDOW];
    double secMeaDiffVolt[MAX_VOLTAGE_ARRAYSIZE][WINDOW];
    double secMeaDiffCurr[MAX_CURRENT_ARRAYSIZE][WINDOW];

    // 低通滤波
    double priMeaFiltVolt[MAX_VOLTAGE_ARRAYSIZE][WINDOW];
    double priMeaFiltCurr[MAX_CURRENT_ARRAYSIZE][WINDOW];
    double secMeaFiltVolt[MAX_VOLTAGE_ARRAYSIZE][WINDOW];
    double secMeaFiltCurr[MAX_CURRENT_ARRAYSIZE][WINDOW];

    // 相量值
    Phasor priPhasor[2][MAX_PHASORSIZE];
    Phasor secPhasor[2][MAX_PHASORSIZE];

    // 断路器状态
    int brkNum; // 断路器信号总维数
    int brkStatus[MAX_BREAKER_ARRAYSIZE]; // 断路器状态

    // 遥控信号
    int remoteControlSignal;

    // 整定值部分
    // 同期侧电压选择
    int remoteVoltMode;
    // CT/PT变比整定值
    double CTSetValue[MAX_CURRENT_ARRAYSIZE];
    double PTSetValue[MAX_VOLTAGE_ARRAYSIZE];

    // 计算量
    double priPhaseVoltRMS[MAX_VOLTAGE_ARRAYSIZE]; // 一次侧相电压
    double priPhaseCurrRMS[MAX_CURRENT_ARRAYSIZE]; // 一次侧相电流
    double priP2PVoltRMS[MAX_VOLTAGE_ARRAYSIZE]; // 一次侧相间电压
    double secPhaseVoltRMS[MAX_VOLTAGE_ARRAYSIZE]; // 二次侧相电压
    double secPhaseCurrRMS[MAX_CURRENT_ARRAYSIZE]; // 二次侧相电流
    double secP2PVoltRMS[MAX_VOLTAGE_ARRAYSIZE]; // 二次侧相间电压

    double priActivePower; // 一次侧有功
    double secActivePower; // 二次侧有功
    double priReactivePower; // 一次侧无功
    double secReactivePower; // 二次侧无功
    double priFrequency; // 一次侧频率
    double secFrequency; // 二次侧频率
    double priPowerFactor; // 一次侧功率因数
    double secPowerFactor; // 二次侧功率因数
    double priZeroSeqVolt; // 一次侧零序电压
    double secZeroSeqVolt; // 二次侧零序电压
    double priZeroSeqCurr; // 一次侧零序电流
    double secZeroSeqCurr; // 二次侧零序电流

    Phasor localSideVolt; // 测量侧电压
    Phasor remoteSideVolt; // 抽取侧电压
    double localSideVoltAmp; // 幅值
    double localSideVoltAngle; // 幅值
    double remoteSideVoltAmp; // 幅值
    double remoteSideVoltAngle; // 幅值
    double localSideFrequency; // 测量侧频率
    double remoteSideFrequency; // 抽取侧频率

    double voltDifference; // 压差
    double freqDifference; // 频差
    double angleDifference; // 角差
    double slipDifference; // 滑差（角频率之差）

    // 文件写入相关量
    double recordCycle; // 测控装置输出文件写入周期
    int recordSpacePoints; // 间隔数
    int recordCount; // 计数器

    int fileExistFlag[3]; // 标识是否是首次创建文件，0--尚未创建，1--已创建；对应三个文件：0--一次侧；1--二次侧；2--同期信息

    // 输出量
    int tripSignals; // 三相合一

    // 其他
    int notYetFlag[MAXSIZE]; // HASH散列数组标志

} meaDevice;

