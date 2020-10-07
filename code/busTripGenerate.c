
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

void busBackupTripGenerate(BusDevice* bus, Device line[], int secNum);
void busBrkTripGenerate(BusDevice* bus, int brkNum);
void busTripGenerateForOne(BusDevice* bus, Device line[], int busNum);
void lineTripGenerateForBus(BusDevice* bus, Device line[], int busNum);
void findConnectBrk(int busNum, int* result);
void findBrkConnectBus(int brkNum, int* result);

void busTripGenerate(BusDevice* bus, Device line[]) {
    
    int i, j;
    int brkBlockFlag[4];

    for (i = 0; i < 4; i++) {
        if ((bus->chargeBlockFlag[i] == 0) || (bus->relayTime - bus->chargeBlockTime[i] > 0.3)) {
            brkBlockFlag[i] = 0;
        } else {
            brkBlockFlag[i] = 1;
        }
    }
    int connectBus[2];
    
     // 跳母联
    for (i = 0; i < 4; i++) {
        findBrkConnectBus(i, connectBus);
        if (bus->busBrkTripFlag[i] == 0 && (i % 2 == 1 || bus->voltBlockFlag[connectBus[0]] == 0 || bus->voltBlockFlag[connectBus[1]] == 0) && (bus->antiSat2Flag[0] == 0) && (bus->busDiffTripFlag[0] == 1)) {
            // 跳分段不经电压闭锁
            busBrkTripGenerate(bus, i);
        }
    }
    // 断路器失灵一时限跳母联
    int connectBrk[2];
    for (i = 0; i < 4; i++) {
        if (bus->busBranchFailureTripFlagI[i] == 1) {
            findConnectBrk(i, connectBrk);
            for (j = 0; j < 2; j++) {
                bus->busBrkTripFlag[connectBrk[j]] = 1;
                busWriteLogWithPhase(bus,"断路器失灵保护动作，母联%c跳闸", connectBrk[j]);
            }
        }
    }

    // CT断线标志位整理: 1 - 可以跳母线
    /*int CTBreak[4];
    int busCTBreakFlag[4][3]; 
    int busCTFinal[4]; // 0:可以跳闸，1：闭锁跳闸
    int phase;
    for (i = 0; i < 4; i++) {
        for (phase = 0; phase < 3; phase++) {
            busCTBreakFlag[i][phase] = (bus->busCTBreakFlag[i][phase]) || !(bus->busDiffPhaseTripFlag[i+1][phase] || bus->busDeltaDiffPhaseTripFlag[i][phase]);  
        }
        busCTFinal[i] = busCTBreakFlag[i][0] && busCTBreakFlag[i][1] && busCTBreakFlag[i][2];
    }
    
    
    CTBreak[0] = 1;//(!busCTFinal[0]) || (bus->busBrkTripTime[0] != 0 && bus->relayTime - bus->busBrkTripTime[0] > 0.01);
    CTBreak[1] = 1;//(!busCTFinal[1]) || (bus->busBrkTripTime[1] != 0 && bus->relayTime - bus->busBrkTripTime[1] > 0.01);
    CTBreak[2] = 1;//(!busCTFinal[2]) || (bus->busBrkTripTime[2] != 0 && bus->relayTime - bus->busBrkTripTime[2] > 0.01);
    CTBreak[3] = 1;//(!busCTFinal[3]) || (bus->busBrkTripTime[3] != 0 && bus->relayTime - bus->busBrkTripTime[3] > 0.01);

    // 跳母线
    if ((brkBlockFlag[0] == 0) && (brkBlockFlag[2] == 0)) {
        if (bus->voltBlockFlag[0] == 0 && (CTBreak[0] && CTBreak[2])) {
            busTripGenerateForOne(bus, line, 0);
        }
    }
    if ((brkBlockFlag[0] == 0) && (brkBlockFlag[3] == 0)) {
        if (bus->voltBlockFlag[1] == 0 && (CTBreak[0] && CTBreak[3])) {
            busTripGenerateForOne(bus, line, 1);
        }
    }
    if ((brkBlockFlag[1] == 0) && (brkBlockFlag[2] == 0)) {
        if (bus->voltBlockFlag[2] == 0 && (CTBreak[1] && CTBreak[2])) {
            busTripGenerateForOne(bus, line, 2);
        }
    }
    if ((brkBlockFlag[1] == 0) && (brkBlockFlag[3] == 0)) {
        if (bus->voltBlockFlag[3] == 0 && (CTBreak[1] && CTBreak[3])) {
            busTripGenerateForOne(bus, line, 3);
        }
    }*/

    // 跳母线
    if ((brkBlockFlag[0] == 0) && (brkBlockFlag[2] == 0)) {
        if (bus->voltBlockFlag[0] == 0) {
            busTripGenerateForOne(bus, line, 0);
        }
    }
    if ((brkBlockFlag[0] == 0) && (brkBlockFlag[3] == 0)) {
        if (bus->voltBlockFlag[1] == 0) {
            busTripGenerateForOne(bus, line, 1);
        }
    }
    if ((brkBlockFlag[1] == 0) && (brkBlockFlag[2] == 0)) {
        if (bus->voltBlockFlag[2] == 0) {
            busTripGenerateForOne(bus, line, 2);
        }
    }
    if ((brkBlockFlag[1] == 0) && (brkBlockFlag[3] == 0)) {
        if (bus->voltBlockFlag[3] == 0) {
            busTripGenerateForOne(bus, line, 3);
        }
    }

    int busNum;
    //int connectBrk[2];
    // 母联/分段失灵保护跳母线
    for (i = 0; i < 4; i++) { 
        switch (i) {
            case 1: busNum = 2; break;
            case 2: busNum = 1; break;
            default: busNum = i; break;
        }
        if (bus->busTieFailureFlag[i] == 1) {
            bus->busTripFlag[busNum] = 1;
            lineTripGenerateForBus(bus, line, busNum);
            findConnectBrk(busNum, connectBrk);
            bus->busBrkTripFlag[connectBrk[0]] = 1;
            bus->busBrkTripFlag[connectBrk[1]] = 1;
            busWriteLogWithPhase(bus, "母联失灵保护动作，%c号母联/分段跳闸", connectBrk[0]);
            busWriteLogWithPhase(bus, "母联失灵保护动作，%c号母联/分段跳闸", connectBrk[1]);
        }
        if (bus->busSecFailureFlag[i] == 1) {
            bus->busTripFlag[i] = 1;
            lineTripGenerateForBus(bus, line, i);
            findConnectBrk(i, connectBrk);
            bus->busBrkTripFlag[connectBrk[0]] = 1;
            bus->busBrkTripFlag[connectBrk[1]] = 1;
            busWriteLogWithPhase(bus, "分段失灵保护动作，%c号母联/分段跳闸", connectBrk[0]);
            busWriteLogWithPhase(bus, "分段失灵保护动作，%c号母联/分段跳闸", connectBrk[1]);
        }
    }

    // 跳后备
    for (i = 0; i < 2; i++) {
        if (bus->busDiffBackupTripFlag[i] == 1){
            busBackupTripGenerate(bus, line, i);
            busWriteLog(bus, "母差后备动作");
        }
    }
}

void busBackupTripGenerate(BusDevice* bus, Device line[], int secNum) {
    
    int i, j, busTopo[24][4];
    double In = 2.0;

    switch (secNum) {
        case 0: for (i = 0; i < 24; i++) {
                   for (j = 0; j < 4; j++) {
                        busTopo[i][j] = bus->busModeStatus[i][j];
                        // 切除有流且无刀闸位置开入的支路
                    }
                }
                for (i = 0; i < 4; i++) {
                    if (bus->voltBlockFlag[i] == 0) {
                        bus->busBrkTripFlag[i] = 1;
                    }
                }
                break;
        case 1: for (i = 0; i < 24; i++) {
                   for (j = 0; j < 4; j++) {
                        busTopo[i][j] = bus->busModeStatus[i][j];
                        // 切除所有I>2In支路
                        if (bus->voltBlockFlag[j] == 0) {
                            if (busTopo[i][j] == 1) {
                               if ((phasorAbs(line[i].phasor[9]) > 2*In) || (phasorAbs(line[i].phasor[10]) > 2*In) || (phasorAbs(line[i].phasor[11]) > 2*In)) {
                                    line[i].tripFlag[0] = 1;
                                    line[i].tripFlag[1] = 1;
                                    line[i].tripFlag[2] = 1;
                                }
                            } else if (busTopo[i][j] == -1) {
                                if ((phasorAbs(line[i].phasor[3]) > 2*In) || (phasorAbs(line[i].phasor[4]) > 2*In) || (phasorAbs(line[i].phasor[5]) > 2*In)) {
                                    line[i].tripFlag[0] = 1;
                                    line[i].tripFlag[1] = 1;
                                    line[i].tripFlag[2] = 1;
                                }
                            }
                        }    
                    }
                }
                break;
    }
}

void busBrkTripGenerate(BusDevice* bus, int brkNum) {
    switch (brkNum) {
        case 0: 
        if (bus->bypassBusMode == 1 || bus->bypassBusMode == 2) {
            return;
        }
        if ((bus->busDiffTripFlag[1] == 1) || (bus->busDiffTripFlag[3] == 1)) {
            bus->busBrkTripFlag[0] = 1;
            bus->busBrkTripTime[0] = bus->relayTime;
            busWriteLog(bus,"母差动作，母联1跳闸");
        }
        break;
        case 1: if ((bus->busDiffTripFlag[2] == 1) || (bus->busDiffTripFlag[4] == 1)) {
            bus->busBrkTripFlag[1] = 1;
            bus->busBrkTripTime[1] = bus->relayTime;
            busWriteLog(bus,"母差动作，母联2跳闸");
        }
        break;
        case 2: if ((bus->busDiffTripFlag[1] == 1) || (bus->busDiffTripFlag[2] == 1)) {
            bus->busBrkTripFlag[2] = 1;
            bus->busBrkTripTime[2] = bus->relayTime;
            busWriteLog(bus,"母差动作，分段1跳闸");
        }
        break;
        case 3: if ((bus->busDiffTripFlag[3] == 1) || (bus->busDiffTripFlag[4] == 1)) {
            bus->busBrkTripFlag[3] = 1;
            bus->busBrkTripTime[3] = bus->relayTime;
            busWriteLog(bus,"母差动作，分段2跳闸");
        }
        break;
    }
}

void busTripGenerateForOne(BusDevice* bus, Device line[], int busNum) {

    int antiSat2DiffFlag = bus->antiSat2Flag[busNum+1];
    int busDiffFlag = bus->busDiffTripFlag[busNum+1];
    int busDeltaDiffFlag = bus->busDeltaDiffTripFlag[busNum];
    int i = 0;
    int connectBrk[2] = {-1, -1};
    int CTBreakPhaseFlag[3] = {0}; // 0-不准跳闸
    int CTBreakFlag = 0;

    findConnectBrk(busNum, connectBrk);


    for (i = 0; i < 3; i++) {
        if ((bus->busDeltaDiffPhaseTripFlag[busNum][i] || bus->busDiffPhaseTripFlag[busNum+1][i]) && !(bus->busCTBreakFlag[connectBrk[0]][i] || bus->busCTBreakFlag[connectBrk[1]][i])) {
            CTBreakPhaseFlag[i] = 1;
        }
        // CT断线闭锁后，母联跳开100ms可以跳母线
        if ((bus->busBrkTripTime[connectBrk[0]] != 0 && bus->relayTime - bus->busBrkTripTime[connectBrk[0]] > 0.1) || (bus->busBrkTripTime[connectBrk[1]] != 0 && bus->relayTime - bus->busBrkTripTime[connectBrk[1]] > 0.1)) {
            CTBreakPhaseFlag[i] = 1;
        }
    }

    CTBreakFlag = CTBreakPhaseFlag[0] || CTBreakPhaseFlag[1] || CTBreakPhaseFlag[2]; 
    
    if (CTBreakFlag && (((antiSat2DiffFlag == 0) && (busDiffFlag == 1)) || (bus->busDeltaDiffTripFlag[busNum] == 1 && bus->antiSat1Flag[busNum+1] == 0))) {  //|| antiSat2DiffFlag == 0
        bus->busTripFlag[busNum] = 1;
        lineTripGenerateForBus(bus, line, busNum);
        busWriteLogWithPhase(bus, "%c段母线差动保护动作", busNum);
    }
    if (bus->busBranchFailureTripFlagII[busNum] == 1){
        bus->busTripFlag[busNum] = 1;
        lineTripGenerateForBus(bus, line, busNum);
        busWriteLogWithPhase(bus, "断路器失灵保护动作，母线%c段切除", busNum);
    }
}

void lineTripGenerateForBus(BusDevice* bus, Device line[], int busNum) { //这里还需要进一步改进如何区别是该跳哪一侧的断路器（取不同侧的线路装置的地址）

    int busTopo[24];
    int i;

    for (i = 0; i < 24; i++) {
        busTopo[i] = bus->busModeStatus[i][busNum];
        if (busTopo[i] != 0) {
            line[i].tripFlag[0] = 1;
            line[i].tripFlag[1] = 1;
            line[i].tripFlag[2] = 1;
            writeLog(&line[i], "经母差跳闸");
        }
    }
    if (busNum == 0 && bus->bypassBusMode == 1 || busNum == 2 && bus->bypassBusMode == 2) {
        bus->busBrkTripFlag[0] = 1; //母联1带路，当作支路开关处理
        busWriteLog(bus, "母联1作旁母断路器经母差跳闸");
    }
    
}

void findConnectBrk(int busNum, int* result) {
    switch (busNum) {
        case 0: result[0] = 0; result[1] = 2; break; // ML1/FD1
        case 1: result[0] = 1; result[1] = 2; break; // ML2/FD1
        case 2: result[0] = 0; result[1] = 3; break; // ML1/FD2
        case 3: result[0] = 1; result[1] = 3; break; // ML2/FD2
    }
}

void findBrkConnectBus(int brkNum, int* result) {
    switch (brkNum) {
        case 0: result[0] = 0; result[1] = 2; break; // ML1/FD1
        case 1: result[0] = 1; result[1] = 3; break; // ML1/FD2
        case 2: result[0] = 0; result[1] = 1; break; // ML2/FD1
        case 3: result[0] = 2; result[1] = 3; break; // ML2/FD2
    }
}


