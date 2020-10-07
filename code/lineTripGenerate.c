#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

/**
 * 
 * 选相、跳闸逻辑实现
 * 
 * 整合所有保护的跳闸信息，综合判断
 * 
**/

void faultPhaseChoose(Device* device);
void deltaVoltPhaseChoose(Device* device, int* phaseChooseResult);
void currentDiffPhaseChoose(Device* device, int* phaseChooseResult);
void negZeroSeqPhaseChoose(Device* device, int* phaseChooseResult);
double workVoltPhaseCalculate(Device* device, int phase);
double workVoltP2PCalculate(Device* device, int phase);

void lineTripGenerate(Device* device) {
    // 控制字写入
    int relayIIBlock = device->relayIIBlockRecloseEnable; // II段保护闭锁重合闸
    int mpBlock = device->mpFaultBlockRecloseEnable; // 多相故障闭锁重合闸

    // 选相元件
    faultPhaseChoose(device);

    // 动作标志汇总：三相中任一相动作视为该保护动作
    int M1, currentDiffFlag = 0, deltaDistanceFlag = 0, distanceIFlag = 0, distanceIIFlag = 0, zeroSeqIIFlag = device->zeroSequenceTripFlag[0];
    int currentCheckFlag[3], currentCheckAnyPhaseFlag = 0, chooseSinglePhaseFlag[3], chooseMultiPhaseFlag = 0; // 各相有流（有流-1），任一相有流；选单相，选多相
    int MI[3], MII[3], MIII[3]; // MI对应M2/M6/M10，MII对应M3/M7/M11, MIII对应M4/M8/M12
    int beforeTripFlag[3]; // 前一时刻的跳闸信号
    int tripFlag[3]; // 各相跳闸出口,对应M5/M9/M13
    int invalidChooseFlag = device->invalidChooseFlag;
    double  invalidChooseTime = 0.0; // 选相无效
    int M14, M15, M15delay = 0, M16, M17 = 0;
    int M18, distanceIIIFlag = 0, zeroSeqIIIFlag = device->zeroSequenceTripFlag[1], manualCloseSpeedupFlag = 0, recloseSpeedupFlag = 0; // 距离III段，零序III段，手合加速，重合加速
    int M19, remoteTripFlag = 0, singleNoReturnFlag = 0, singleRunTripleFlag = 0, PTBreakOcFlag = (device->PTBreakTripFlag[0] || device->PTBreakTripFlag[1]); // 单相运行三跳涉及非全相运行对有流的判断，尚未实现，见说明书ZL_XLBH5114.0905 PDF-28页
    int M20, M21; // 三跳
    int M22; // II段闭重
    int M23, comTripleTripFlag = device->comTripleTripFlag, reFaultFlag = 0;
    int M24; // 闭锁重合闸
    int M25, comTripleCloseFlag = 0, PTBreakFlag = device->PTBreakFlag;
    int M26, M27; // 启动TJ、TJabc继电器
    int M28 = device->singleTripFailFlag, M28delay = 0;
    int singleTripFailFlag = device->singleTripFailFlag;
    double singleTripFailTime = 0.0;
    int M29, M30, tripleTripMode = device->brkThreePhaseMode; // 三相跳闸-控制字
    int M31, M32, chargeNotFinishFlag = 0, recloseEnable, tpRecloseFlag = device->tpRecloseMode; // 充电未满，重合闸投运-控制字，三相重合闸-控制字
    
    double I[3], In = device->ratedSideICurrent;
    int i, j = 0;

    // 标志判断写入及初始化
    if (device->reCloseTimes > 0) {
        recloseSpeedupFlag = 1;
    }
    if ((device->stopRecloseMode == 0) && (device->banRecloseMode == 0)) {
        recloseEnable = 1;
    } else {
        recloseEnable = 0;
    }
    for (i = 0; i < 4; i++) {
        if (device->currentDiffTripFlag[i] == 1) {
            currentDiffFlag = 1;
        }
    }
    for (i = 0; i < 3; i++) {
        if (device->deltaDistanceTripFlag[i] == 1) {
            deltaDistanceFlag = 1;
        }
        if (device->distanceIITripFlag[i] == 1) {
            distanceIIFlag = 1;
        }
        if (device->distanceITripFlag[i] == 1) {
            distanceIFlag = 1;
        }
        if (device->distanceIIITripFlag[i] == 1) {
            distanceIIIFlag = 1;
        }
        /*if (device->manCloseFlag[i] == 1) {
            manualCloseSpeedupFlag = 1;
        }*/

        tripFlag[i] = 0;
        beforeTripFlag[i] = device->tripFlag[i];

        // 有流判断
        I[i] = phasorAbs(device->phasor[i+3]);
        if (I[i] > 0.06*In) {
            currentCheckFlag[i] = 1;
            currentCheckAnyPhaseFlag = 1;
        } else {
            currentCheckFlag[i] = 0;
        }
        if (device->tripPhase[i] == 1) {
            chooseSinglePhaseFlag[i] = 1;
            j++;
        } else {
            chooseSinglePhaseFlag[i] = 0;
        }
        MI[i] = 0; MII[i] = 0; MIII[i] = 0; //device->tripFlag[i];
    }

    if (j > 1) { 
        chooseMultiPhaseFlag = 1;
        writeLog(device, "选多相");
        j = 0;
    } else { 
        chooseMultiPhaseFlag = 0; 
    }

    // 非全相再故障
    for (i = 0; i < 3; i++) {
        if (device->sampleBrkStatus[i] == 0) {
            for (j = 0; (j < 3 && j != i); j++) {
                if (device->tripFlag[j] == 1) {
                    reFaultFlag = 1;
                    break;
                }
            }
        }
        if (reFaultFlag == 1) { break; }
    }

    // 跳闸判断
    M1 = (currentDiffFlag == 1 || deltaDistanceFlag == 1 || distanceIIFlag == 1 || distanceIFlag == 1 || zeroSeqIIFlag == 1);
    for (i = 0; i < 3; i++) {
        MI[i] = (M1 && chooseSinglePhaseFlag[i] == 1);
        MII[i] = (MI[i] || MIII[i]);
        MIII[i] = (currentCheckFlag[i] == 1 && MII[i]);
    }
    M14 = (M1 && chooseMultiPhaseFlag == 1);
    M15 = (M1 && !chooseSinglePhaseFlag[0] && !chooseSinglePhaseFlag[1] && !chooseSinglePhaseFlag[2] && !chooseMultiPhaseFlag);
    
    
    if (M15 == 1) {
        if (invalidChooseFlag == 0) {
            device->invalidChooseStartTime = device->time;
        }
        device->invalidChooseFlag = 1;
    } else {
        device->invalidChooseFlag = 0;
        device->invalidChooseStartTime = 0;
    }


    if (device->invalidChooseStartTime != 0 && (device->time - device->invalidChooseStartTime > 0.2) && M15) {
        M15delay = 1;
        writeLog(device, "选相无效");
    }

    M18 = (manualCloseSpeedupFlag || recloseSpeedupFlag || distanceIIIFlag || zeroSeqIIIFlag);
    M19 = (remoteTripFlag || singleNoReturnFlag || singleRunTripleFlag || PTBreakOcFlag);
    M22 = (distanceIIFlag || zeroSeqIIFlag);
    M20 = (M18 || M19 || (M22 && relayIIBlock));
    M23 = (comTripleTripFlag || reFaultFlag);
    M21 = (M20 || M23);

    M16 = (M17 || M14 || M15delay || M21);
    M17 = (M16 && currentCheckAnyPhaseFlag);

    // 单相跳闸出口
    for (i = 0; i < 3; i++) {
        tripFlag[i] = (MIII[i] || M17);
    }

    M24 = ((reFaultFlag && mpBlock) || M20);
    M25 = (M24 || comTripleCloseFlag || PTBreakFlag);

    M26 = ((beforeTripFlag[0] && device->sampleBrkStatus[0]) || (beforeTripFlag[1] && device->sampleBrkStatus[1]) || (beforeTripFlag[2] && device->sampleBrkStatus[2])); // 这里模拟单跳发信号而断路器未断开的时候，需要综合断路器状态，如果断路器状态不符合跳闸信号的状态才判为单跳失败
    M27 = (beforeTripFlag[0] && beforeTripFlag[1] && beforeTripFlag[2]);

    M28 = (M26 && !M27);

    // 闭锁重合闸
    if (M24 == 1) {
        device->reCloseBlocked = 1;
    }

    // 闭锁重合放电
    if (M25 == 1) {
        device->reCloseBlocked = 1;
    }

    // 启动TJ、TJabc继电器
    if (M26 == 1) {

    }  
    if (M27 == 1) {

    }

    // 单跳失败发三跳
    if (M28 == 1) {
        if (singleTripFailFlag == 0) {
            device->singleTripFailStartTime = device->time;
        }
        device->singleTripFailFlag = 1;
    } else {
        device->singleTripFailFlag = 0;
        device->singleTripFailStartTime = 0;
    }

    if (device->singleTripFailStartTime != 0 && (device->time - device->singleTripFailStartTime > 0.15) && M28) {
        M28delay = 1;
        tripFlag[0] = 1;
        tripFlag[1] = 1;
        tripFlag[2] = 1;
        writeLog(device, "单跳失败发三跳");
        // 单跳失败标志复位
        device->singleTripFailFlag = 0;
        device->singleTripFailStartTime = 0;
    }

    // 沟三跳
    M31 = (chargeNotFinishFlag && recloseEnable);
    M32 = (recloseEnable && tripleTripMode);
    M30 = (tripleTripMode || comTripleCloseFlag || M31 || M32);
    M29 = (M26 && M30);
    if (M29 == 1) {
        device->comTripleTripFlag = 1;
        writeLog(device, "沟三跳");
    } else {
        device->comTripleTripFlag = 0;
    }
    
    // 最终跳令
    for (i = 0; i < 3; i++) {
        if (device->sampleBrkStatus[i] == 1) { // 若检测到断路器状态为合位才进行最终的跳闸信号写入
            device->tripFlag[i] = tripFlag[i];
        }
    }
    


}

void faultPhaseChoose(Device* device) {
    int phaseChooseResultI[3], phaseChooseResultII[3], phaseChooseResultIII[3], finalResult[3];
    int i, count = 0, check = 0;

    // 初始化
    for (i = 0; i < 3; i++) {
        phaseChooseResultI[i] = 0;
        phaseChooseResultII[i] = 0;
        phaseChooseResultIII[i] = 0;
        finalResult[i] = 0;
    }

    currentDiffPhaseChoose(device, phaseChooseResultI);
    deltaVoltPhaseChoose(device, phaseChooseResultII);
    negZeroSeqPhaseChoose(device, phaseChooseResultIII); // 选区结果

    // 工作电压-不对称电流选相结果综合
    for (i = 0; i < 3; i++) {
        if (phaseChooseResultII[i] == 1) {
            count++;
        }
        if (phaseChooseResultI[i] == 1) {
            check++;
        }
    }
    if (count > 1) {
        for (i = 0; i < 3; i++) {
            if (phaseChooseResultIII[i] == 1) {
                switch (i) {
                    case 0: phaseChooseResultIII[0] = 0; phaseChooseResultIII[1] = 1; phaseChooseResultIII[2] = 1; break; // A区两相接地，判为BC
                    case 1: phaseChooseResultIII[0] = 1; phaseChooseResultIII[1] = 0; phaseChooseResultIII[2] = 1; break; // A区两相接地，判为BC
                    case 2: phaseChooseResultIII[0] = 1; phaseChooseResultIII[1] = 1; phaseChooseResultIII[2] = 0; break; // A区两相接地，判为BC
                }
                break;
            }
        }
    }

    // 最终选相写入
    //优先级：差流选相 > 工作电压 > 不对称电流综合选相
    if (check > 0) {
        for (i = 0; i < 3; i++) {
            finalResult[i] = phaseChooseResultI[i];
            if (finalResult[i] == 1) {
              //  writeLogWithPhase(device, "%c相差动选相成功", i);
            }
        }   
    } else {
        if (device->time - device->startTime < 0.006) {
            return;
        }
        if (count > 0) {
            for (i = 0; i < 3; i++) {
                finalResult[i] = phaseChooseResultII[i]; 
                 if (finalResult[i] == 1) {
                   //  writeLogWithPhase(device, "%c相电压变化量选相成功", i);
                }
            }
        } else {
             for (i = 0; i < 3; i++) {
                finalResult[i] = phaseChooseResultIII[i]; //取了或，任一元件选到则最终判为选到
                if (finalResult[i] == 1) {
                   // writeLogWithPhase(device, "%c相不对称电流选相成功", i);
                }
            }  
        }
    }

    count = 0;

    for (i = 0; i < 3; i++) {
        if (finalResult[i] == 1) {
            device->tripPhase[i] = finalResult[i];
            writeLogWithPhase(device, "选相元件动作，选%c相", i);
            count++;
        }
    }

    if (count == 0) {
        device->invalidChooseFlag = 1;
        writeLog(device, "选相无效");
    }
}

void deltaVoltPhaseChoose(Device* device, int* phaseChooseResultI) {

    double deltaUa, deltaUb, deltaUc, deltaUab, deltaUbc, deltaUca;
    double maxUp, maxUpp;
    double k = 0.2;
    int i, maxPhase, A = 0, B = 0, C = 0;

    deltaUa = workVoltPhaseCalculate(device, 0);
    deltaUb = workVoltPhaseCalculate(device, 1);
    deltaUc = workVoltPhaseCalculate(device, 2);

    deltaUab = workVoltP2PCalculate(device, 0);
    deltaUbc = workVoltP2PCalculate(device, 1);
    deltaUca = workVoltP2PCalculate(device, 2);

    // 变化量相电压最大值
    if (deltaUa >= deltaUb) {
        // A相最大
        maxUp = deltaUa;
        maxPhase = 0;
    } else {
        // B相最大
        maxUp = deltaUb;
        maxPhase = 1;
    }

    if (deltaUc > maxUp) {
        // C相最大
        maxUp = deltaUc;
        maxPhase = 2;
    }

    // 相判别
    switch (maxPhase) {
        case 0: if (maxUp > k*deltaUbc) {
                  A = 1;
                } else {
                    if (deltaUab >= deltaUbc) {
                        maxUpp = deltaUab;
                        A = 1; B = 1; C = 0;
                    } else {
                        maxUpp = deltaUbc;
                        A = 0; B = 1; C = 1;
                    }
                    if (deltaUca > maxUpp) {
                        maxUpp = deltaUca;
                        A = 1; B = 0; C = 1;
                    }
                }
                break;
        case 1: if (maxUp > k*deltaUca) {
                  B = 1;
                } else {
                    if (deltaUab >= deltaUbc) {
                        maxUpp = deltaUab;
                        A = 1; B = 1; C = 0;
                    } else {
                        maxUpp = deltaUbc;
                        A = 0; B = 1; C = 1;
                    }
                    if (deltaUca > maxUpp) {
                        maxUpp = deltaUca;
                        A = 1; B = 0; C = 1;
                    }
                }
                break;
        case 2: if (maxUp > k*deltaUab) {
                  C = 1;
                } else {
                    if (deltaUab >= deltaUbc) {
                        maxUpp = deltaUab;
                        A = 1; B = 1; C = 0;
                    } else {
                        maxUpp = deltaUbc;
                        A = 0; B = 1; C = 1;
                    }
                    if (deltaUca > maxUpp) {
                        maxUpp = deltaUca;
                        A = 1; B = 0; C = 1;
                    }
                }
                break;
    }

    // 写入结果
    *(phaseChooseResultI) = A;
    *(phaseChooseResultI + 1) = B;
    *(phaseChooseResultI + 2) = C;
    

}

double workVoltP2PCalculate(Device* device, int phase){

    Phasor Up1, Ip1, Up2, Ip2, Upp, Ipp, Uop, Upm1, Ipm1, Upm2, Ipm2, Uppm, Ippm, Uopm, deltaUop;

    // 整定值
    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);

    // 变量本地化
    Up1 = device->phasor[phase];
    Ip1 = device->phasor[phase + 3];
    Up2 = device->phasor[phaseMatch(phase)];
    Ip2 = device->phasor[phaseMatch(phase) + 3];

    // 寻找记忆量
    if (phase == 0){
        Upm1 = memoryPhasorValue(device, device->memoryVma);
        Ipm1 = memoryPhasorValue(device, device->memoryIma);
        Upm2 = memoryPhasorValue(device, device->memoryVmb);
        Ipm2 = memoryPhasorValue(device, device->memoryImb);
    }
    else if (phase == 1){
        Upm1 = memoryPhasorValue(device, device->memoryVmb);
        Ipm1 = memoryPhasorValue(device, device->memoryImb);
        Upm2 = memoryPhasorValue(device, device->memoryVmc);
        Ipm2 = memoryPhasorValue(device, device->memoryImc);
    }
    else if (phase == 2){
        Upm1 = memoryPhasorValue(device, device->memoryVmc);
        Ipm1 = memoryPhasorValue(device, device->memoryImc);
        Upm2 = memoryPhasorValue(device, device->memoryVma);
        Ipm2 = memoryPhasorValue(device, device->memoryIma);
    }

    // 相间量计算
    Upp = phasorSub(Up1, Up2);
    Ipp = phasorSub(Ip1, Ip2);
    Uppm = phasorSub(Upm1, Upm2);
    Ippm = phasorSub(Ipm1, Ipm2);

    // 极化电压计算
    Uop = phasorSub(Upp, phasorMulti(Zzd, Ipp));
    Uopm = phasorSub(Uppm, phasorMulti(Zzd, Ippm));

    // 变化量计算及判据实现
    deltaUop = phasorSub(Uop, Uopm);

    return phasorAbs(deltaUop);

}

double workVoltPhaseCalculate(Device* device, int phase){

    Phasor Up, Ip, Ik, I0, Uop, Upm, Ipm, Ikm, I0m, Uopm, deltaUop;

    // 整定值
    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);
    // 零序补偿系数
    double k = device->KZ;

    // 变量本地化
    Up = device->phasor[phase];
    Ip = device->phasor[phase + 3];
    I0 = phasorSeq(device->phasor[3], device->phasor[4], device->phasor[5], 0);
    Ik = phasorAdd(Ip, phasorNumMulti(3*k, I0));

    if (phase == 0){
        Upm = memoryPhasorValue(device, device->memoryVma);
        Ipm = memoryPhasorValue(device, device->memoryIma);
    }
    else if (phase == 1){
        Upm = memoryPhasorValue(device, device->memoryVmb);
        Ipm = memoryPhasorValue(device, device->memoryImb);
    }
    else if (phase == 2){
        Upm = memoryPhasorValue(device, device->memoryVmc);
        Ipm = memoryPhasorValue(device, device->memoryImc);
    }
    I0m = phasorSeq(memoryPhasorValue(device, device->memoryIma), memoryPhasorValue(device, device->memoryImb), memoryPhasorValue(device, device->memoryImc), 0);
    Ikm = phasorAdd(Ipm, phasorNumMulti(3*k, I0m));

    // 极化电压计算
    Uop = phasorSub(Up, phasorMulti(Zzd, Ik));
    Uopm = phasorSub(Upm, phasorMulti(Zzd, Ikm));

    // 变化量计算及判据实现
    deltaUop = phasorSub(Uop, Uopm);

    return phasorAbs(deltaUop);
}

void currentDiffPhaseChoose(Device* device, int* phaseChooseResultII) {
    int i;

    for (i = 0; i < 3; i++) {
        *(phaseChooseResultII + i) = device->currentDiffTripFlag[i];
    }
    
}

void negZeroSeqPhaseChoose(Device* device, int* phaseChooseResult) {

    Phasor Ia, Ib, Ic, I0, I2A;
    double angle;

    Ia = device->phasor[3];
    Ib = device->phasor[4];
    Ic = device->phasor[5];

    I0 = phasorSeq(Ia, Ib, Ic, 0);
    I2A = phasorSeq(Ia, Ib, Ic, 2);

    angle = phasorAngle(I0) - phasorAngle(I2A);

    // 转换至0-360
    if (angle < 0) {
        angle = angle + 360;
    }

    // 选区
    if (60 <= angle < 180) {
        *(phaseChooseResult + 1) = 1; //B
    } else if (180 <= angle <= 300) {
        *(phaseChooseResult + 2) = 1; //C
    } else {
        *(phaseChooseResult) = 1; //A
    }
}



