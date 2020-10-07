#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
//#include "../dataStruct.h"
//#include "..\\dataStruct.h"
//#include "..\\common.h"


extern char logDirName[STRING_LENGTH];
extern int globalInitFlag;
/**
 * 测控装置接口函数
 * --内含测控装置逻辑实现
 * @param device
 * @param deviceName
 * @param time
 * @param deviceEnable
 * @param port1
 * @param port2
 * @param port_remote
 * @param tripSignals
 */
void meaLinkSimulation(meaDevice* device, char* deviceName, double time, int deviceEnable, double* port1, double* port2, int* port_remote, int* tripSignals) {
    if (meaNotYet(device, "设置测控装置名及保护定值")) {
        meaDeviceInit(device, deviceName, deviceEnable);
    }
    if (deviceEnable == 0) {
        return;
    }

    meaSwitchRelay(device, time, device->switchPort1, device->switchPort2);

    if (meaUpTo10A(device)) {
        meaSample(device, time, port1, port2, port_remote);
        meaSample2Inst(device);
        meaDataFilter(device);
        meaToPhasor(device);
        if (meaRecordCounter(device) != 0) {
            meaTelemetry(device); // 遥测
            meaRecord(device);
        }
        meaTelecontrol(device);
    }

    tripSignals[0] = device->tripSignals;
    tripSignals[1] = device->tripSignals;
    tripSignals[2] = device->tripSignals;

}
/**
 * 测控装置通信延时模块
 * @param device
 * @param timeD
 * @param port1
 * @param port2
 */
void meaSwitchRelay(meaDevice* device, double timeD, const double* port1, const double* port2) {

    double min1 = device->switch1DelayMin;
    double max1 = device->switch1DelayMax;
    double min2 = device->switch2DelayMin;
    double max2 = device->switch2DelayMax;

    if (max1 - min1 < -0.0001) {
        meaWriteErrorLog(device, "交换机A延时参数错误!");
    }
    if (max2 - min2 < -0.001) {
        meaWriteErrorLog(device , "交换机B延时参数错误");
    }

    double random1 = min1 + rand()%200000 / 200000.0 * (max1-min1);

    // double random1 = 0.1;
    double random2 = 0; // 固定延时为3个仿真步长左右
    int i;

    int q1 = device->queueLength1;
    int q2 = device->queueLength2;

    // 维护指示队列长度的计数器qLength
    // 新到达的数据包放到switchQueue[qLength]位置
    // 仿真10次, 执行一次采样函数, 更新交换机队列
    if (meaUpTo5(device)) {
        if (q1 < QUEUE_LENGTH) {
            // 当队列排满时, 采样值不能进入队列, 模拟丢包
            device->switchQueue1[q1].delayTime = timeD + random1;
            for (i = 0; i < 10; ++i) {
                device->switchQueue1[q1].frame[i] = port1[i];
            }
            if (q1 <= QUEUE_LENGTH-2) {
                device->queueLength1++;
            }
        } else {
            meaWriteLog(device, "信道阻塞, 交换机A丢包");
        }

        if (q2 < QUEUE_LENGTH) {
            device->switchQueue2[q2].delayTime = timeD + random2;
            for (i = 0; i < 9; ++i) {
                device->switchQueue2[q2].frame[i] = port2[i];
            }
            if (q2 <= QUEUE_LENGTH-2) {
                device->queueLength2++;
            }
        } else {
            meaWriteLog(device, "信道阻塞, 交换机B丢包");
        }
    }

    int port1Size = 10;
    int port2Size = 9;
    // 每次仿真都执行
    // 如果判定需要输出, 从队头(0索引位置)取数, 更新队列, 更新交换机端口switchPort
    if (device->queueLength1 > 0 && device->switchQueue1[0].delayTime <= timeD) {
        // 延时到达, 可以输出
        for (i = 0; i < port1Size; ++i) {
            device->switchPort1[i] = device->switchQueue1[0].frame[i];
        }
        // 更新队列
        for (i = 0; i < device->queueLength1; i++) {
            device->switchQueue1[i] = device->switchQueue1[i+1];
        }
        device->switchQueue1[device->queueLength1].delayTime = MAX_VALUE;
        device->queueLength1--;
    }

    if (device->queueLength2 > 0 && device->switchQueue2[0].delayTime <= timeD) {
        // 延时到达, 可以输出
        for (i = 0; i < port2Size; ++i) {
            device->switchPort2[i] = device->switchQueue2[0].frame[i];
        }
        // 更新队列
        for (i = 0; i < device->queueLength2; i++) {
            device->switchQueue2[i] = device->switchQueue2[i+1];
        }
        device->switchQueue2[device->queueLength2].delayTime = MAX_VALUE;
        device->queueLength2--;
    }
}
/**
 * 测控装置初始化函数
 * @param device
 * @param deviceName
 * @param deviceEnable
 */
void meaDeviceInit(meaDevice* device, char* deviceName, int deviceEnable) {
    int i = 0;
    // 设置装置名
    if (deviceEnable == 0) {
        // 装置不启用
        device->deviceEnable = 0;
    } else {
        device->deviceEnable = deviceEnable;

        strcpy(device->deviceName, deviceName);

        // 设置globalFileName和deviceFileName
        sprintf(device->globalFileName, "%s/log.txt", logDirName); // 不同装置共用log.txt
        sprintf(device->deviceFileName, "%s/%s", logDirName, deviceName); // 不同装置录波文件分别存放, 按装置名分开
        // 读取母线保护配置文件, 设置整定值
        meaReadConfiguration(device);

        // 母线保护模块初始化完毕, 设置整定值
        meaWriteLog(device, "测控装置初始化完毕");

    }
}
/**
 * 测控装置配置文件读取函数
 * @param device
 * @return
 */
int meaReadConfiguration(meaDevice* device) {
    char paramName[PARAM_COUNT][STRING_LENGTH];
    double paramValue[PARAM_COUNT];
    char fileName[STRING_LENGTH];

    int count = 0;
    int i, j;
    int flag = 0;
    // 根据device中的deviceName打开对应的配置文件并进行赋值
    sprintf(fileName, "%s%s%s", "..\\config\\", device->deviceName, ".conf");

    FILE *file = fopen(fileName, "r");
    if (file == NULL)
    {
        meaWriteErrorLog(device, "配置文件未找到！");
        return -1;
    }

    char buf[MAX_BUF_LEN];
    int text_comment = 0;
    while (fgets(buf, MAX_BUF_LEN, file) != NULL)
    {
        Trim(buf);
        // to skip text comment with flags /* ... */
        if (buf[0] != '#' && (buf[0] != '/' || buf[1] != '/'))
        {
            if (strstr(buf, "/*") != NULL)
            {
                text_comment = 1;
                continue;
            }
            else if (strstr(buf, "*/") != NULL)
            {
                text_comment = 0;
                continue;
            }
        }
        if (text_comment == 1)
            continue;

        int buf_len = strlen(buf);
        // ignore and skip the line with first chracter '#', '=' or '/'
        if (buf_len <= 1 || buf[0] == '#' || buf[0] == '=' || buf[0] == '/')
            continue;
        buf[buf_len - 1] = '\0';

        char _paramk[MAX_KEY_LEN] = {0}, _paramv[MAX_VAL_LEN] = {0};
        int _kv = 0, _klen = 0, _vlen = 0, _type = 0;
        int i = 0;
        for (i = 0; i < buf_len; ++i)
        {
            if (buf[i] == ' ')
                continue;
            // scan param key name
            if (_kv == 0 && _type == 0 && buf[i] != '=' && buf[i] != '@')
            {
                if (_klen >= MAX_KEY_LEN)
                    break;
                _paramk[_klen++] = buf[i];
                continue;
            }
            else if (buf[i] == '=')
            {
                _kv = 1;
                continue;
            } else if (buf[i] == '@'){
                _type = 1;
                continue;
            }
            // scan param key value
            if (_vlen >= MAX_VAL_LEN || buf[i] == '#')
                break;
            _paramv[_vlen++] = buf[i];
        }
        if (flag == 0 && _type == 1){
            if (strcmp(_paramv, "500kV线路测控装置") == 0) {
                device->deviceType = LINETYPE;
            } else if (strcmp(_paramv, "500kV母线测控装置") == 0) {
                device->deviceType = BUSTYPE;
            } else if (strcmp(_paramv, "500kV母线边开关测控装置") == 0) {
                device->deviceType = BUSBRKTYPE_SIDE;
            } else if (strcmp(_paramv, "500kV母线中开关测控装置") == 0) {
                device->deviceType = BUSBRKTYPE_MID;
            } else {
                device->deviceType = ERRORTYPE;
            }
            flag = 1;
            continue;
        }
        if (strcmp(_paramk, "") == 0 || strcmp(_paramv, "") == 0) {
            continue;
        }
        sprintf(paramName[count], _paramk);
        paramValue[count] = atof(_paramv);
        count++;
        // printf("%s=%s\n", _paramk, _paramv);
    }
    /**
     * 
     * 整定值设置部分
     * 
     */
    device->priPTSetValue[RATEINDEX_LOCAL_PT] = meaFindSetValueIndex("PT一次侧额定值（kV）", paramName, paramValue, count, device);
    device->secPTSetValue[RATEINDEX_LOCAL_PT] = meaFindSetValueIndex("PT二次侧额定值（V）", paramName, paramValue, count, device);
    device->priPTSetValue[RATEINDEX_REMOTE_PT] = meaFindSetValueIndex("同期侧PT一次侧额定值（kV）", paramName, paramValue, count, device);
    device->secPTSetValue[RATEINDEX_REMOTE_PT] = meaFindSetValueIndex("同期侧PT二次侧额定值（V）", paramName, paramValue, count, device);
    device->priPTSetValue[RATEINDEX_ZEROSEQ_PT] = meaFindSetValueIndex("零序PT一次侧额定值（kV）", paramName, paramValue, count, device);
    device->secPTSetValue[RATEINDEX_ZEROSEQ_PT] = meaFindSetValueIndex("零序PT二次侧额定值（V）", paramName, paramValue, count, device);
    device->priCTSetValue[RATEINDEX_LOCAL_CT] = meaFindSetValueIndex("CT一次侧额定值（kA）", paramName, paramValue, count, device);
    device->secCTSetValue[RATEINDEX_LOCAL_CT] = meaFindSetValueIndex("CT二次侧额定值（A）", paramName, paramValue, count, device);
    device->priCTSetValue[RATEINDEX_ZEROSEQ_CT] = meaFindSetValueIndex("零序CT一次侧额定值（kA）", paramName, paramValue, count, device);
    device->secCTSetValue[RATEINDEX_ZEROSEQ_CT] = meaFindSetValueIndex("零序CT二次侧额定值（A）", paramName, paramValue, count, device);
    device->remoteVoltMode = meaFindSetValueIndex("同期侧电压类型", paramName, paramValue, count, device);

    device->recordCycle = 0.02 / POINTS * 1;
    device->recordSpacePoints = (int)(device->recordCycle / (0.02 / POINTS));

    meaWriteLog(device, "读取测控装置整定值完毕");
    return 0;
}
/**
 * 测控装置配置文件字段识别函数
 * @param target
 * @param paramName
 * @param paramValue
 * @param n
 * @param device
 * @return
 */
double meaFindSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, meaDevice* device) {
    int i = 0;
    char errorLog[STRING_LENGTH];

    for (i = 0; i < n; i++) {
        if (strcmp(target, *(paramName+i)) == 0) {
            return paramValue[i];
        }
    }
    // 如果没有找到对应的整定值, 将错误信息记录到日志中
    sprintf(errorLog, "%s%s", target, "参数未定义!");
    meaWriteErrorLog(device, errorLog);
    return 0;
}
/**
 * 测控装置唯一性函数（重载）
 * @param device
 * @param str
 * @return
 */
int meaNotYet(meaDevice* device, char* str) {
    unsigned int hashCode;

    hashCode = SDBMHash(str, MAXSIZE);
    if (device->notYetFlag[hashCode] == 0) {
        device->notYetFlag[hashCode] = 1;
        return 1;
    } 
    return 0;    
}
/**
 * 测控装置计数器（10）
 * @param device
 * @return
 */
int meaUpTo10A(meaDevice* device) {
    device->sampleCount1++;
    if (device->sampleCount1 == 10) {
        device->sampleCount1 = 0;
        return 1;
    } else {
        return 0;
    }
}
/**
 * 测控装置计数器（5）
 * @param device
 * @return
 */
int meaUpTo5(meaDevice* device) {
    device->sampleCount2++;
    if (device->sampleCount2 == 5) {
        device->sampleCount2 = 0;
        return 1;
    } else {
        return 0;
    }
}
/**
 * 测控装置端口数据采样函数
 * @param device
 * @param time
 * @param port1
 * @param port2
 * @param port_remote
 */
void meaSample(meaDevice* device, double time, double* port1, double* port2, int* port_remote) {
    int port1_length = 0, port2_length = 0;

    switch (device->deviceType) {
        case 0 :
            meaWriteErrorLog(device, "测控装置未初始化");
            return;
        case ERRORTYPE :
            meaWriteErrorLog(device, "测控装置初始化错误");
            return;
        case LINETYPE :
            port1_length = 9;
            port2_length = 3;
            break;
        case BUSBRKTYPE_SIDE :
            port1_length = 11;
            port2_length = 3;
            break;
        case BUSBRKTYPE_MID :
            port1_length = 11;
            port2_length = 3;
            break;
        case BUSTYPE : // 待确认
            port1_length = 12;
            port2_length = 12;
            break;
    }
    int i = 0;

    device->time = time;

    for (i = 0; i < port1_length; i++) {
        device->sample[i] = port1[i];
    }
    for (i = 0; i < port2_length; i++) {
        device->brkNum = port2_length;
        device->brkStatus[i] = port2[i];
    }
    device->remoteControlSignal = port_remote[0];
}
/**
 * 测控装置数据写入函数
 * @param device
 */
void meaSample2Inst(meaDevice* device) {
    int i = 0, j = 0, k = 0;
    
    double kPT[3] = {1.0, 1.0, 1.0};
    double kCT[2] = {1.0, 1.0};

    if (device->voltSize == 0 && device->currSize == 0) {
        switch (device->deviceType) {
            case LINETYPE :
                device->voltSize = 5;
                device->currSize = 4;
                for (i = 0; i < 3; i++) {
                    kPT[i] = device->priPTSetValue[i] / device->secPTSetValue[i];
                }
                kCT[0] = device->priCTSetValue[0] / device->secCTSetValue[0];
                kCT[1] = device->priCTSetValue[2] / device->secCTSetValue[2]; // 零序
                break;
            case BUSBRKTYPE_SIDE :
                device->voltSize = 5;
                device->currSize = 6;
                for (i = 0; i < 3; i++) {
                    kPT[i] = device->priPTSetValue[i] / device->secPTSetValue[i];
                    if (i < 2) {
                        kCT[i] = device->priCTSetValue[i] / device->secCTSetValue[i];
                    }
                }
                break;
            case BUSBRKTYPE_MID :
                device->voltSize = 5;
                device->currSize = 6;
                for (i = 0; i < 3; i++) {
                    kPT[i] = device->priPTSetValue[i] / device->secPTSetValue[i];
                    if (i < 2) {
                        kCT[i] = device->priCTSetValue[i] / device->secCTSetValue[i];
                    }
                }
                break;
            case BUSTYPE:
                device->voltSize = 12;
                device->currSize = 0;
                kPT[0] = device->priPTSetValue[0] / device->secPTSetValue[0];
                for (j = RECORD_LENGTH-1; j >= 1; j--) {
                    device->instTime[j] = device->instTime[j-1];
                    for (i = 0; i < device->voltSize; i++) {
                        device->priMeaInstVolt[i][j] = device->priMeaInstVolt[i][j-1];
                        device->secMeaInstVolt[i][j] = device->secMeaInstVolt[i][j-1];
                    }
                }
                device->instTime[0] = device->time;
                for (i = 0; i < device->voltSize; i++) {
                    device->priMeaInstVolt[i][0] = device->sample[i] * kPT[0];
                    device->secMeaInstVolt[i][0] = device->sample[i];
                }
                return;
            default :
                return;
        }
    }

    for (j = RECORD_LENGTH-1; j >= 1; j--) {
        device->instTime[j] = device->instTime[j-1];
        for (i = 0; i < device->voltSize; i++) {
            device->priMeaInstVolt[i][j] = device->priMeaInstVolt[i][j-1];
            device->secMeaInstVolt[i][j] = device->secMeaInstVolt[i][j-1];
        }
        for (k = 0; k < device->currSize; k++) {
            device->priMeaInstCurr[k][j] = device->priMeaInstCurr[k][j-1];
            device->secMeaInstCurr[k][j] = device->secMeaInstCurr[k][j-1];
        }
    }

    device->instTime[0] = device->time;


    for (i = 0; i < device->voltSize; i++) {
        device->secMeaInstVolt[i][0] = device->sample[i];
        if (i < 3) {
            device->priMeaInstVolt[i][0] = device->sample[i] * kPT[0];
        } else if (i < 4) {
            device->priMeaInstVolt[i][0] = device->sample[i] * kPT[1];
        } else if (i < 5) {
            device->priMeaInstVolt[i][0] = device->sample[i] * kPT[2];
        }
    }
    for (i = 0; i < device->currSize; i++) {
        device->secMeaInstCurr[i][0] = device->sample[i + device->voltSize];
        if (i < 3) {
            device->priMeaInstCurr[i][0] = device->sample[i + device->voltSize] * kCT[0];
        } else {
            device->priMeaInstCurr[i][0] = device->sample[i + device->voltSize] * kCT[1];
        }
    }
}

void meaDataFilter(meaDevice* device) {
    int i = 0, j = 0, k = 0;

    switch (device->deviceType) {
        case LINETYPE :
        case BUSBRKTYPE_SIDE :
        case BUSBRKTYPE_MID :
        case BUSTYPE:
            break;
        default :
            return;
    }

    for (j = RECORD_LENGTH-1; j >= 1; j--) {
        for (i = 0; i < device->voltSize; i++) {
            device->priMeaFiltVolt[i][j] = device->priMeaFiltVolt[i][j-1];
            device->secMeaFiltVolt[i][j] = device->secMeaFiltVolt[i][j-1];
        }
        for (k = 0; k < device->currSize; k++) {
            device->priMeaFiltCurr[k][j] = device->priMeaFiltCurr[k][j-1];
            device->secMeaFiltCurr[k][j] = device->secMeaFiltCurr[k][j-1];
        }
    }
    for (i = 0; i < device->voltSize; i++) {
        differenceFilter(device->priMeaDiffVolt[i], device->priMeaInstVolt[i]);
        lowPassFilter(device->priMeaFiltVolt[i], device->priMeaDiffVolt[i]);

        differenceFilter(device->secMeaDiffVolt[i], device->secMeaInstVolt[i]);
        lowPassFilter(device->secMeaFiltVolt[i], device->secMeaDiffVolt[i]);
    }
    for (k = 0; k < device->currSize; k++) {
        differenceFilter(device->priMeaDiffCurr[i], device->priMeaInstCurr[i]);
        lowPassFilter(device->priMeaFiltCurr[i], device->priMeaDiffCurr[i]);
        differenceFilter(device->secMeaDiffCurr[i], device->secMeaInstCurr[i]);
        lowPassFilter(device->secMeaFiltCurr[i], device->secMeaDiffCurr[i]);
    }

}

void meaToPhasor(meaDevice* device) {
    int i = 0;
    for (i = 0; i < device->voltSize; i++) {
        inst2phasor(device->priMeaFiltVolt[i], 0, &device->priPhasor[0][i]);
        inst2phasor(device->secMeaFiltVolt[i], 0, &device->secPhasor[0][i]);
    }
    for (i = 0; i < device->currSize; i++) {
        inst2phasor(device->priMeaFiltCurr[i], 0, &device->priPhasor[1][i]);
        inst2phasor(device->secMeaFiltCurr[i], 0, &device->secPhasor[1][i]);
    }
}

/**
 * 有效值计算函数
 *
 * @param inst 瞬时值序列
 * @param startIdx 数据窗起点对应的索引
 * @return 返回当前数据窗的有效值
 */
double rmsCalculate(double* inst, int startIdx) {
    int i = 0;
    double ret = 0;
    for (i = 0; i < POINTS; i++) {
        ret += (inst[startIdx + i]*inst[startIdx + i]);
    }
    ret *= (1.0/POINTS);
    return sqrt(ret);
}

/**
 * 功率计算函数
 *
 * @param voltA A相电压瞬时值序列
 * @param currA A相电流瞬时值序列
 * @param voltB B相电压瞬时值序列
 * @param currB B相电流瞬时值序列
 * @param voltC C相电压瞬时值序列
 * @param currC C相电流瞬时值序列
 * @param startIdx 数据窗起点索引
 * @param type 功率类型：'P'--有功，'Q'--无功
 * @return 返回当前数据窗的功率值
 */
double powerCalculate(double* voltA, double* currA, double* voltB, double* currB, double* voltC, double* currC, int startIdx, char type) {
    int i = 0;
    double ret = 0;
    int offset = (int)(0.75 * POINTS);
    switch (type) {
        case 'P' :
            for (i = 0; i < POINTS; i++) {
                ret += (voltA[startIdx + i] * currA[startIdx + i] + voltB[startIdx + i] * currB[startIdx + i] + voltC[startIdx + i] * currC[startIdx + i]);
            }
            break;
        case 'Q' :
            for (i = 0; i < POINTS; i++) {
                ret += (voltA[startIdx + i] * currA[startIdx + i + offset] + voltB[startIdx + i] * currB[startIdx + i + offset] + voltC[startIdx + i] * currC[startIdx + i + offset]);
            }
            break;
        default :
            return 0.0;
    }
    ret *= (1.0/POINTS);

    return ret;
}
/**
 * 功率因数计算函数
 *
 * @param P 有功功率
 * @param Q 无功功率
 * @return 返回当前点的功率因数
 */
double powerFactorCalculate(double P, double Q) {
    return (P / sqrt(P * P + Q * Q));
}

/**
 * 频率计算函数（高频时无效，需提高采样率）
 *
 * @param inst 待测频序列
 * @param pt 每周波采样点数
 * @param startIdx 数据窗起点索引
 * @return 返回当前数据窗算得的频率
 */
double freqCalculate(double* inst, int pt, int startIdx) {
    if (pt == 0) {
        return 0.0;
    }
    int offset = (int)(POINTS / pt);
    int total = (int)(POINTS / offset);
    double numerator = 0.0, denominator = 0.0;
    double tempNum = 0.0, tempDen = 0.0;
    int i = 0, j = 0, k = 0;

    for (i = 0; i < total; i++) {
        k = i % 4;
        switch (k) {
            case 0 :
                tempNum += inst[startIdx + offset * i];
                break;
            case 1 :
                tempDen += inst[startIdx + offset * i];
                break;
            case 2 :
                tempDen += inst[startIdx + offset * i];
                denominator += (fabs(tempDen));
                tempDen = 0.0;
                break;
            case 3 :
                tempNum += inst[startIdx + offset * i];
                numerator += (fabs(tempNum));
                tempNum = 0.0;
                break;
        }
    }

    double x = numerator / denominator;
    double T = 0.02 / pt; // 采样周期
    double ret = asin(0.5*sqrt(1-x)) / (PI * T);

    return ret;
}
/**
 * 压差计算
 *
 * @param Ug 机侧电压
 * @param Ux 网侧电压
 * @return 返回电压幅值差（Ug - Ux）
 */
double voltDiffCalculate(Phasor Ug, Phasor Ux) {
    return (phasorAbs(Ug) - phasorAbs(Ux));
}

/**
 * 角差计算
 *
 * @param Ug 机侧电压
 * @param Ux 网侧电压
 * @return 返回电压相角差（Ug - Ux）
 */
double angleDiffCalculate(Phasor Ug, Phasor Ux) {
    double diff = phasorAngleDiff(Ug, Ux);
    return diff > 180.01 ? (diff - 360.0) : diff;
}

/**
 * 频差计算
 *
 * @param fg 机侧频率
 * @param fx 网侧频率
 * @return 返回频率差（Ug - Ux）
 */
double freqDiffCalculate(double fg, double fx) {
    return (fg - fx);
}

/**
 * 滑差计算（wg - wx）
 *
 * @param fg 机侧频率
 * @param fx 网侧频率
 * @return 返回滑差（Ug - Ux）
 */
double slipDiffCalculate(double fg, double fx) {
    return (2 * PI * freqDiffCalculate(fg, fx));
}

void meaWriteLog(meaDevice* device, char* content) {
    if (content != NULL && notYet(device, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            { 
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ", device->time, device->deviceName);
        
                fprintf(fp, content);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;  
            }
        }
    }
}
/**
 * 测控装置计数器
 *
 * @param device 测控装置
 * @return 是否到达测控装置输出点
 */
int meaRecordCounter(meaDevice* device) {
    device->recordCount++;
    if (device->recordCount == device->recordSpacePoints) {
        device->recordCount = 0;
        return 1;
    } else {
        return 0;
    }
}

/**
 * 写入错误日志信息
 * @param device
 * @param content
 */
void meaWriteErrorLog(meaDevice* device, char* content) {
    if (content != NULL && notYet(device, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            {
                fprintf(fp, "[错误] Simulation Time: %fs [%s]: ", device->time, device->deviceName);

                fprintf(fp, content);
                fprintf(fp, "\n");
                fclose(fp);
                fp = NULL;
            }
        }
    }
}


/**
 * 上面日志函数的重载形式, 主要用于相别信息
*/
void meaWriteLogWithPhase(meaDevice* device, char* content, int phase) {
    char formatContent[128];
    char charPhase;

    // 将相别数字转换为字母
    charPhase = (char)('A'+phase);

    // 格式化字符串
    sprintf(formatContent, content, charPhase);
    
    if (formatContent != NULL && notYet(device, formatContent)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            { 
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ",device->time, device->deviceName);
        
                fprintf(fp, formatContent);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;

            }
        }
    }
}