/**
 * 
 * 母线保护-断路器失灵保护 （双母双分段接线220kV)
 * 
 **/
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

void lineBrkFailureRelay(BusDevice* bus, Device* line, Device* sampleLine, int lineNo);
void findLineConnectBus(BusDevice* bus, int lineNo, int* result);
void transBrkFailureRelay(BusDevice* bus, Device* line, /*trans*/Device* transRelay, int transNo);


void busBranchFailureRelay(BusDevice* bus, Device* line, Device* lineRelay,/*trans*/Device* transRelay) { 
    int i = 0, j = 0;
    for (i = 0; i < 20; i++) {
        for (j = 0; j < 4; j++) {
            if (bus->busModeStatus[i][j] != 0) {
                lineBrkFailureRelay(bus, line, lineRelay, i);
                break;
            }
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            if (bus->busModeStatus[i][j] != 0) {
                transBrkFailureRelay(bus, line, transRelay, i+20);
                break;
            }
        }
    }
    if (bus->bypassBusMode != 0) {
        lineBrkFailureRelay(bus, line, lineRelay, 28);
    }
    return; 
}

void lineBrkFailureRelay(BusDevice* bus, Device* line, Device* lineRelay, int lineNo) {
    int enable = bus->busBranchFailureEnable;
    int unlockFlag = bus->LbranchUnlockFailureEnable[lineNo];

    int threeStart = bus->LbranchStartFailureEnable[lineNo*4]; //三相跳闸接点：对应线路的三相跳闸出口
    // int threeStart = lineRelay[lineNo]->???
    int singleStart[3];
    int i;
    // if (threeStart == 0){  }
    for (i = 1; i < 4; i++) {
        singleStart[i] = bus->LbranchStartFailureEnable[lineNo*4+i]; //分相跳闸接点：对应线路的分相跳闸出口
        // singleStart[i] = lineRelay[lineNo]->???
    }

    int side = 0;
    for (i = 0; i < 4; i++) {
        if (bus->busModeStatus[lineNo][i] != 0) {
            side = bus->busModeStatus[lineNo][i];
            break;
        }
    }
    if (side == 0) {
        return;
    }

    int unlock = bus->LbranchUnlockFailureEnable[lineNo];
    double In = 0.0;
    Phasor Ip[3];
    double I0, I2;
    double I[3], Idiff[3];
    double Iset = 0.0, IdeltaSet = 0.0; // 固定门槛，浮动门槛
    double I0set = 0.0, I2set = 0.0; // 失灵零序（3I0）、负序定值
    double t1set = 0.5, t2set = 1.0; //时间定值1/2

    Phasor *temp; // *temp2, *mem, *mem2;
    //Phasor Imside;
    //Phasor Inside;

    for (i = 0; i < 3; i++) {
        temp = findLineRelayTimePhasor(line[lineNo].busSynFlag, &line[lineNo], side, i);
        //mem = findLineMemory(bus, line, lineNo, i, side);
        // 线路变化量差动
        //temp2 = findLineRelayTimePhasor(bus->busDiffDelayFlag, &line[lineNo], side*(-1), i);
        //mem2 = findLineMemory(bus, line, lineNo, i, side*(-1));
        // 各相电流幅值
        Ip[i] = *temp;
        I[i] = phasorAbs(*temp);
        // 各相变化量差流绝对值
        //Imside = phasorSub(*temp, *mem);
        //Inside = phasorSub(*temp2, *mem2);
        //Idiff[i] = phasorAbs(phasorAdd(Imside, Inside));
    }

    I0 = 3.0*phasorAbs(phasorSeq(Ip[0], Ip[1], Ip[2], 0));
    I2 = phasorAbs(phasorSeq(Ip[0], Ip[1], Ip[2], 2));

    if (bus->LbranchDeltaTime[lineNo] != 0 && bus->relayTime - bus->LbranchDeltaTime[lineNo] > 5) {
        bus->LbranchDeltaFlag[lineNo] = 0;
        bus->LbranchDeltaTime[lineNo] = 0;
    }

    int deltaFlag = lineRelay[lineNo].deltaCurrentFlag;
    int M1, M2[3], M2sum;

    if (bus->LbranchDeltaFlag[lineNo] == 0 && deltaFlag == 1) {
        bus->LbranchDeltaFlag[lineNo] = 1;
        if (bus->LbranchDeltaTime[lineNo] == 0) {
            bus->LbranchDeltaTime[lineNo] = bus->relayTime;
        }
    }

    M1 = I[0] > 0.1*In && I[1] > 0.1*In && I[2] > 0.1*In && threeStart && bus->LbranchDeltaFlag[lineNo];
    for (i = 0; i < 3; i++) {
        M2[i] = (I[i] > 0.04*In) && (threeStart || singleStart[i]);
    }
    M2sum = M2[0] || M2[1] || M2[2];
    int M3 = M2sum && (I0 > I0set || I2 > I2set);
    int M4 = (M3 || M1) && enable;

    int connectBus[2] = {0, 0};
    findLineConnectBus(bus, lineNo, connectBus);

    int blockFlag[2], lineQs[2]; // 电压闭锁、刀闸位置
    for (i = 0; i < 2; i++) {
        blockFlag[i] = bus->voltBlockFlag[connectBus[i]];
        lineQs[i] = bus->busModeStatus[lineNo][connectBus[i]];
    }

    int C1 = lineQs[0] && (unlockFlag || (lineQs[0] && blockFlag[0]));
    int C2 = lineQs[1] && (unlockFlag || (lineQs[1] && blockFlag[1]));
    // 假定同一条线路不会同时连接至两条母线上
    if (C1) {
        if (M4 && C1) {
            if (bus->busBranchFailureTripTime[lineNo] == 0) {
                bus->busBranchFailureTripTime[lineNo] = bus->relayTime;
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t1set) {
                bus->busBranchFailureTripFlagI[connectBus[0]] = 1;
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t2set) {
                bus->busBranchFailureTripFlagII[connectBus[0]] = 1;
            }
        } else {
            bus->busBranchFailureTripTime[lineNo] = 0;
            bus->busBranchFailureTripFlagI[connectBus[0]] = 0;
            bus->busBranchFailureTripFlagII[connectBus[0]] = 0;
        }
    } else if (C2) {
        if (M4 && C2) {
            if (bus->busBranchFailureTripTime[lineNo] == 0) {
                bus->busBranchFailureTripTime[lineNo] = bus->relayTime;
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t1set) {
                bus->busBranchFailureTripFlagI[connectBus[1]] = 1;          
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t2set) {
                bus->busBranchFailureTripFlagII[connectBus[1]] = 1;
            }
        } else {
            bus->busBranchFailureTripTime[lineNo] = 0;
            bus->busBranchFailureTripFlagI[connectBus[1]] = 0;
            bus->busBranchFailureTripFlagII[connectBus[1]] = 0;
        }
    }

}

void transBrkFailureRelay(BusDevice* bus, Device* line, Device* transRelay, int lineNo) {

    int enable = bus->busBranchFailureEnable;
    int busDiffTripTrans = line[lineNo].tripFlag[0]; // 母差跳主变，三跳任选一个标志位

    int threeStart = bus->LbranchStartFailureEnable[lineNo*4]; //三相跳闸接点：对应线路的三相跳闸出口
    // int threeStart = transRelay[lineNo-20]->???
    int i;

    int side = 0;
    for (i = 0; i < 4; i++) {
        if (bus->busModeStatus[lineNo][i] != 0) {
            side = bus->busModeStatus[lineNo][i];
            break;
        }
    }
    if (side == 0) {
        return;
    }

    //int unlock = bus->LbranchUnlockFailureEnable[lineNo];
    double In = 0.0;
    Phasor Ip[3];
    double I0, I2;
    double I[3];
    //double Iset = 0.0, IdeltaSet = 0.0; // 固定门槛，浮动门槛
    double I0set = 0.0, I2set = 0.0; // 失灵零序（3I0）、负序定值
    double t1set = 0.5, t2set = 1.0; //时间定值1/2

    Phasor *temp; // *temp2, *mem, *mem2;
    //Phasor Imside;
    //Phasor Inside;

    for (i = 0; i < 3; i++) {
        temp = findLineRelayTimePhasor(line[lineNo].busSynFlag, &line[lineNo], side, i);
        //mem = findLineMemory(bus, line, lineNo, i, side);
        // 线路变化量差动
        //temp2 = findLineRelayTimePhasor(bus->busDiffDelayFlag, &line[lineNo], side*(-1), i);
        //mem2 = findLineMemory(bus, line, lineNo, i, side*(-1));
        // 各相电流幅值
        Ip[i] = *temp;
        I[i] = phasorAbs(*temp);
        // 各相变化量差流绝对值
        //Imside = phasorSub(*temp, *mem);
        //Inside = phasorSub(*temp2, *mem2);
        //Idiff[i] = phasorAbs(phasorAdd(Imside, Inside));
    }

    I0 = 3.0*phasorAbs(phasorSeq(Ip[0], Ip[1], Ip[2], 0));
    I2 = phasorAbs(phasorSeq(Ip[0], Ip[1], Ip[2], 2));

    int start = busDiffTripTrans || threeStart;

    int M1, M2;

    M1 = I[0] > 0.1*In || I[1] > 0.1*In || I[2] > 0.1*In && start;

    M2 = M1 || (start && I0 > I0set) || (start && I2 > I2set);

    int M3 = M2 && enable;

    int connectBus[2] = {0, 0};
    findLineConnectBus(bus, lineNo, connectBus);

    int lineQs[2]; // 电压闭锁、刀闸位置
    for (i = 0; i < 2; i++) {
        lineQs[i] = bus->busModeStatus[lineNo][connectBus[i]];
    }
    // 假定同一条线路不会同时连接至两条母线上
    if (lineQs[0]) {
        if (M3 && lineQs[0]) {
            if (bus->busBranchFailureTripTime[lineNo] == 0) {
                bus->busBranchFailureTripTime[lineNo] = bus->relayTime;
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t1set) {
                bus->busBranchFailureTripFlagI[connectBus[0]] = 1;
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t2set) {
                bus->busBranchFailureTripFlagII[connectBus[0]] = 1;
            }
        } else {
            bus->busBranchFailureTripTime[lineNo] = 0;
            bus->busBranchFailureTripFlagI[connectBus[0]] = 0;
            bus->busBranchFailureTripFlagII[connectBus[0]] = 0;
        }
    } else if (lineQs[1]) {
        if (M3 && lineQs[1]) {
            if (bus->busBranchFailureTripTime[lineNo] == 0) {
                bus->busBranchFailureTripTime[lineNo] = bus->relayTime;
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t1set) {
                bus->busBranchFailureTripFlagI[connectBus[1]] = 1;          
            }
            if (bus->busBranchFailureTripTime[lineNo] != 0 && bus->relayTime - bus->busBranchFailureTripTime[lineNo] >= t2set) {
                bus->busBranchFailureTripFlagII[connectBus[1]] = 1;
            }
        } else {
            bus->busBranchFailureTripTime[lineNo] = 0;
            bus->busBranchFailureTripFlagI[connectBus[1]] = 0;
            bus->busBranchFailureTripFlagII[connectBus[1]] = 0;
        }
    }

}



