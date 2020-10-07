#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

#include <stdio.h>
#include <math.h>


extern void lineStarter(Device *device, int phase);
extern void deltaDistanceRelay(Device *device, int phase);
extern void distanceRelay(Device *device, int phase);
extern void overCurrentRelay(Device *device, int phase);
extern void currentDiffRelay(Device *device, int phase);
extern void zeroSeqCurrentRelay(Device *device, int phase);
extern void lineTripGenerate(Device* device);
extern void PTbreakRecognize(Device* device);
extern void CTBreakRelay(Device* device);


extern void reClose1(Device *device);
extern void reClose3(Device *device);
extern void reSetAllTripFlag(Device *device, int phase);
extern void manSwitchSig(Device *device);

void openPhaseOperation(Device* device);
void recordPhasor(Device* device);
int deltaCurrentRelay(Device* device, int phase);


void line(Device *device) {

    int phase = 0, count = 0, notAllTrip = (device->sampleBrkStatus[0] == 1 || device->sampleBrkStatus[1] == 1 || device->sampleBrkStatus[2] == 1);
    double beforeTripFlag[3];
    int allRight = ((device->tripSignalOutput[0] != device->sampleBrkStatus[0]) && (device->tripSignalOutput[0] != device->sampleBrkStatus[0]) && (device->tripSignalOutput[0] != device->sampleBrkStatus[0]));

    int openPhase = 0;

    for (phase = 0; phase < 3; phase++) {
        // 写入前一点的跳闸信号
        beforeTripFlag[phase] = device->tripFlag[phase];
    }

    Phasor ua, ub, uc;
    Phasor ua1, ub1, uc1;

    // 将采样值存入瞬时值数组
    sample2inst(device);

    // 瞬时值滤波后存入并更新滤波后数组
    dataFilter(device);

    //记忆前一个采样点的相量计算值（10*0.02/48）
    recordPhasor(device);

    // 利用滤波后数据计算12通道相量,存入phasor数组
    toPhasor(device);

    if (device->time < 0.2) {
        // 等待仿真进入稳定状态
        return;
    }

    // 检测手动开关位置
    manSwitchSig(device);

    // 启动判据
    // 只有保护没有启动才进入判别, 只要有一种启动判据动作置位, 就不再进行启动判别
    for (phase = 0; phase < 3; phase++) {
        if (device->startFlag == 0 && notAllTrip == 1) {
            lineStarter(device, phase);
        }
    }
    
    openPhaseOperation(device);

    for (phase = 0; phase < 3; phase++) {
        if (device->openPhaseFlag[phase] == 1) {
            openPhase = 1;
            break;
        }
    }

    int deltaCurrentFlag = 0;
    // 与母线交互的电流变化量
    if (device->startFlag == 1 && device->deltaCurrentFlag == 0) {
        for (phase = 0; phase < 3; phase++) {
            deltaCurrentFlag = deltaCurrentRelay(device, phase);
            if (deltaCurrentFlag == 1) {
                device->deltaCurrentFlag = 1;
                break;
            }
        }
    }


    // PT断线
    if (openPhase == 0) {
        PTbreakRecognize(device);
        CTBreakRelay(device);
    }
    

    // 零序电流（先于差动调用）
    if (device->zeroSequenceEnable == 1 && notAllTrip == 1) {
        if (device->startFlag == 1) {
            zeroSeqCurrentRelay(device, phase);
        }
    }


    // 差动    
    if ((device->CTbreakBlockDiffEnable == 0 || device->CTBreakFlag == 0) && device->currentDiffEnable == 1 && notAllTrip == 1) {
        for (phase = 0; phase < 3; phase++) {
            if (device->startFlag == 1) {
                currentDiffRelay(device, phase);
            }
        }
    }

    // 工频变化量距离
    if (device->deltaDistanceEnable == 1 && notAllTrip == 1 && device->CTBreakFlag == 0 && device->PTBreakFlag == 0) {
        for (phase = 0; phase < 3; phase++) {
            if (device->startFlag == 1) {
                deltaDistanceRelay(device, phase);
            }
        }
    }

    // 距离保护
    if (device->startFlag == 1 && device->PTBreakFlag == 0 && notAllTrip == 1) {
        distanceRelay(device, phase);
    }


    // 过电流
    /*if (device->overCurrentEnable == 1) {
        for (phase = 0; phase < 3; phase++) {
            if (device->startFlag == 1) {
                overCurrentRelay(device, phase);
            }
        }
    }*/

    // 跳闸判断
    if (device->startFlag == 1 && notAllTrip == 1) {
        lineTripGenerate(device);
    }

    // 跳闸信号输出
    for (phase = 0; phase < 3; phase++) {
        if (device->tripFlag[phase] == 1 && beforeTripFlag[phase] == 0) {
            device->tripSignalOutput[phase] = 1;
            writeLogWithPhase(device, "%c相最终判断跳闸", phase);
            if (device->tripStartTime == 0) {
                device->tripStartTime = device->time;
            }
        }
        // 复归
        if (device->startFlag == 1 && device->tripStartTime != 0 && device->time - device->tripStartTime >= device->returnSetTime && allRight == 1) {
            reSetAllTripFlag(device, phase);
            writeLog(device, "保护动作信号复归");
        }
    }
    

    // 录波模块 启动后7个周波后, 将instIma一次写入
    if ((device->startFlag == 1) && ((device->time - device->startTime) / 0.02 > 7) && notYet(device, "数据录波")) {
        writeLog(device, "装置数据录波");
        recordData(device);
    }



    // 跳闸 1--单跳; 3--三跳
    //decideTrip_3(device);

    // 重合闸 1--单重; 3--三重
    //reClose3(device);




}

int deltaCurrentRelay(Device* device, int phase) {

    double *curr, *currm;
    curr = findInstCurrentPtr(device, phase);
    currm = device->memoryCurrent[phase];
    int i = 0;
    double sum = 0;
    double Iset = 0.0; // 门槛值设置接口

    // 门槛值写入（1次）
    if (device->halfWaveDeltaCurrentValue == 0) {
        for (i = 0; i < 24; i++) {
            sum += fabs(currm[i]);
        }
        device->halfWaveDeltaCurrentValue = sum;
    }

    double delta = fabs(curr[0]-currm[memoryIndex(device)]);
    sum = 0;
    
    for (i = 23; i > 0; i--) {
        device->memoryCurrent[phase][i] = device->memoryCurrent[phase][i-1]; // 22->23...0->1
    }
    device->memoryCurrent[phase][0] = delta;

    for (i = 0; i < 24; i++) {
        sum += device->memoryCurrent[phase][i];
    }

    if (sum > device->halfWaveDeltaCurrentValue + Iset) {
        return 1;
    } else {
        return 0;
    }

}

void recordPhasor(Device* device) {
    int i;
    for (i = 0; i < 12; i++) {
        device->prePhasor[i] = device->phasor[i];
    }
}

void openPhaseOperation(Device* device) {
    int trip[3], TWJ[3], i = 0, currentTag = 0;
    double I[3], In = device->ratedSideICurrent;

    for (i = 0; i < 3; i++) {
        trip[i] = device->tripFlag[i];
        TWJ[i] = device->sampleBrkStatus[i];
        I[i] = phasorAbs(device->phasor[i+3]);
        if (device->openPhaseFlag[i] == 0) {
            if ((trip[i] == 1 || TWJ[i] == 0) && I[i] < 0.02*In) {
                device->openPhasePreFlag[i] = 1;
                if (device->openPhaseTime[i] == 0) {
                    device->openPhaseTime[i] = device->time;
                }
                if (device->time - device->openPhaseTime[i] >= 0.03) {
                    device->openPhaseFlag[i] = 1;
                    writeLogWithPhase(device, "非全相运行状态，%c相退出运行",i);
                    device->openPhasePreFlag[i] = 0;
                    device->openPhaseTime[i] == 0;
                }
            }
        }
        if (device->openPhaseFlag[i] == 1) {
            if (TWJ[i] == 1) {
                device->openPhaseFlag[i] = 0;
                writeLogWithPhase(device, "%c相恢复运行",i);
            }
        }
    }

}



void decideTrip_1(Device *device) {
    // 2020-3-4
    int i = 0;

    for (i = 0; i < 3; i++) {
        // 如果该相已经动作, 不再判定
        if (device->tripFlag[i] == 1) continue;

        /*if (device->currentDiffTripFlag[i] == 1 || device->deltaDistanceTripFlag[i] == 1 ||
            device->distanceTripFlag[i] == 1 || device->zeroSequenceTripFlag[i] == 1 ||
            device->overCurrentTripFlag[i] == 1) {

            device->tripFlag[i] = 1;
        }*/
    }
}


void decideTrip_3(Device *device) {
    int i = 0;
    // 如果已经动作, 不再进入逻辑
    if (device->tripState == 1) return;

    for (i = 0; i < 3; i++) {
        if (device->tripFlag[i] == 1 /*|| device->deltaDistanceTripFlag[i] == 1 ||
            device->distanceTripFlag[i] == 1 || device->zeroSequenceTripFlag[i] == 1 ||
            device->overCurrentTripFlag[i] == 1*/) {

            // 判断是否是手合于故障
            if (device->time - device->manCloseTime < 0.1) {
                device->reCloseBlocked = 1;
            }

            device->tripFlag[0] = 1;
            device->tripFlag[1] = 1;
            device->tripFlag[2] = 1;

            // 其实意义不大, 因为动作后所有保护肯定还满足, 需要在重合闸之前再重置一次
            reSetAllTripFlag(device, 0);
            reSetAllTripFlag(device, 1);
            reSetAllTripFlag(device, 2);
            if (device->fastII == 1) {
                writeLog(device, "三相加速跳闸");
            } else {
                writeLog(device, "三相故障跳闸");
            }

            device->tripTimes++;

            // 置加速II段标志位
            device->fastII = 1;

            device->tripState = 1;
            // break使得只要有一相动作, 就三跳, 不再判断其他项
            break;
        }
    }
}


void reClose1(Device *device) {

}


void reClose3(Device *device) {
    // 如果没有进行过跳闸, 不进入重合闸逻辑
    if (device->tripState == 0) return;
    // 如果已经闭锁, 直接返回
    if (device->reCloseBlocked == 1) return;
    // 如果重合后再次跳闸 不再重合
    if (device->tripTimes >= 2) {
        // 置闭锁标志位
        device->reCloseBlocked = 1;
        return;
    }

    if (device->tripState == 1) {
        device->reCloseCount++;
    }

    if (device->reCloseCount > RECLOSE_COUNT ) {
        // 重置所有保护动作位
        reSetAllTripFlag(device, 0);
        reSetAllTripFlag(device, 1);
        reSetAllTripFlag(device, 2);

        device->tripState = 0;
        device->reCloseCount = 0;

        // 重合

        device->tripFlag[0] = 0;
        device->tripFlag[1] = 0;
        device->tripFlag[2] = 0;
        writeLog(device, "三相重合闸");

        // 重合闸计数
        device->reCloseTimes++;

    }
}


void reSetAllTripFlag(Device *device, int phase) {
    device->startFlag = 0;
    device->startTime = 1000;
    device->currentDiffTripFlag[phase] = 0;
    device->currentDiffSelfTripFlag[phase] = 0;
    device->currentDiffUnionFlag[phase] = 0;
    device->deltaDistanceTripFlag[phase] = 0;
    device->distanceITripFlag[phase] = 0;
    device->distanceIITripFlag[phase] = 0;
    device->distanceIIITripFlag[phase] = 0;
    device->zeroSequenceTripFlag[phase] = 0;
    device->overCurrentTripFlag[phase] = 0;
    device->halfWaveP2GSum[phase] = 0.0;
    device->halfWaveP2PSum[phase] = 0.0;
}

/**
 * 跳变检测, 检测到手跳, 即0 -> 1, 置 manTrip=1
 * 检测到手合, 即1 -> 0, 置 manClose=1, 记录合闸时间
 */
void manSwitchSig(Device *device) {
    int i = 0;

    /* 手跳检测 */
    for (i = 0; i < 3; i++) {
        if (device->manBrkStatus[0][i] == 1 && device->manBrkStatus[5][i] == 0) {
            // 当前为分闸状态, 且5个点前为合闸
            device->manTripFlag[i] = 1;
            writeLogWithPhase(device, "%c相手动跳闸", i);
        }
    }
    /* 手合检测 */
    for (i = 0; i < 3; i++) {
        if (device->manBrkStatus[0][i] == 0 && device->manBrkStatus[5][i] == 1) {
            device->manCloseFlag[i] = 1;
            if (notYet(device, "记录手合时间")) {
                device->manCloseTime = device->time;
            }
            writeLogWithPhase(device, "%c相手动合闸", i);

            // 短时置加速位100ms
            if (device->time - device->manCloseTime < 0.1) {
                device->fastII = 1;
                writeLogWithPhase(device, "%c相短时加速重合闸", i);
            } else {
                if (notYet(device, "恢复加速位")) {
                    device->fastII = 0;
                }
            }
        }
    }
}


/**
 * 同期合闸逻辑
 * @param device
 */
void synchronizedClosed(Device *device) {
    // 使用母线电压A相和线路A相电压
    Phasor curBus = device->busPhasor[0];
    Phasor curLine = device->phasor[0];
    Phasor prevBus = device->prevBus;
    Phasor prevLine = device->prevLine;
    // 相角差
    double angleDiff;
    Phasor angleBusDiff;
    Phasor angleLineDiff;
    // 角速度
    double angleSpeed;
    // 角速度整定值
    double angleSpeedSet = 0.1;  // rad/s
    // 断路器合闸时间整定值
    double switchTime = 100;  // ms

    angleBusDiff = phasorSub(curBus, prevBus);
    angleLineDiff = phasorSub(curLine, prevLine);
    angleDiff = phasorAngleDiff(angleBusDiff, angleLineDiff);
    angleSpeed = angleDiff / (device->time - device->prevTime);

    if (angleSpeed < angleSpeedSet) {
        device->closeAngle = angleSpeed * switchTime / 1000;
    }

}





