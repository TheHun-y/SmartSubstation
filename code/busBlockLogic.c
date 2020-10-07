
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

int voltageBlockJudge(BusDevice* bus, int busNo) {
    double Ubs, U0bs, U2bs;
    int flag = 0;
    Ubs = 0.7*220; // 固定0.7Un
    U0bs = 0.06*220; U2bs = 0.04*220;
    double Up[3], U0, U2;
    Phasor *temp;
    if (bus->busDiffDelayFlag) {
        temp = bus->busPrePhasor;
    } else {
        temp = bus->busPhasor;
    }
    int i = 0;
    for ( ; i < 3; i++) {
        Up[i] = phasorAbs(temp[busNo*6+i]);
        if (Up[i] <= Ubs) {
            flag = 1;
            break;
        }
    }
    U0 = phasorAbs(phasorSeq(temp[busNo*6], temp[busNo*6+1], temp[busNo*6+2], 0));
    U2 = phasorAbs(phasorSeq(temp[busNo*6], temp[busNo*6+1], temp[busNo*6+2], 2));

    if (flag || (3*U0 >= U0bs) || (3*U2 >= U2bs)) {
        return 0;
    }
    return 1;
}

void chargeBlockJudge(BusDevice* bus) {
    // 充电闭锁模块：只闭锁差动跳母线，跳母联不经任何充电闭锁环节

    int busStatus[4], busStatusAux[4];
    int TWJ[4], TWJmem[4], manualFlag[4], manualFlagMem[4];
    int i, j, phase = 0;
    double In = 2.0;

    for (i = 0; i < 4; i++) {
        TWJ[i] = 1 - bus->busBrkStatus[i];
        manualFlag[i] = bus->busBrkManualFlag[i];
        TWJmem[i] = bus->busBrkStatusMemory[i];
        manualFlagMem[i] = bus->busBrkManualFlagMemory[i];
        busStatus[i] = bus->busRunStatus[i];
        busStatusAux[i] = busStatusAuxJudge(busStatus, i);
    }
    
    for (i = 0; i < 4; i++) {
        // 充电预备状态：TWJ为1且两母线未全在运行状态
        if ((TWJ[i] == 1) && (busStatusAux[i] == 1)){
        // 检测到母联合闸开入由0到1
            if ((manualFlag[i] == 1) && (manualFlagMem[i] == 0)) {
                bus->chargeBlockFlag[i] = 1;
                bus->chargeBlockTime[i] = bus->time;
                busWriteLogWithPhase(bus,"%c号母联合闸开入",i);
            }
        }
    }
    // 母联分列运行或母联有流则立即解除延时，即置充电闭锁位0
    for(i = 0; i < 4; i++) {
        if (manualFlag[i] == 0) {
            bus->chargeBlockFlag[i] = 0;
        }
        for (phase = 0; phase < 3; phase++) {
            if (phasorAbs(bus->busPhasor[6*i+3+phase]) > 0.04*In) {
                bus->chargeBlockFlag[i] = 0;
                busWriteLogWithPhase(bus,"%c号母联有流，解除充电闭锁延时",i);
                break;
            }
        }
    }
    // 翻转计时
    for (i = 0; i < 4; i++) {
        if ((TWJ[i] == 0) && (TWJmem[i] == 1)) {
            bus->TWJReturnTime[i] = bus->time;
        }
        if ((manualFlag[i] == 1) && (manualFlagMem[i] == 0)) {
            bus->manualFlagChangeTime[i] = bus->time;
        }
    }
    // 解除延时判断
    for (i = 0; i < 4; i++) {
        if (bus->chargeBlockFlag[i] == 1) {
            if ((bus->time - bus->TWJReturnTime[i] > 0.5) || (bus->time - bus->manualFlagChangeTime[i] > 1)) {
                bus->chargeBlockFlag[i] = 0;
                busWriteLogWithPhase(bus,"%c号母联充电闭锁时间到，解除延时",i);
            }
        }
    }
   
    
}

void busBrkStatusMemory(BusDevice* bus) {
    
    int i;
    for (i = 0; i < 4; i++) {
        if (bus->busBrkStatus[i] == 0 && bus->busBrkStatusMemory[i] == 1) {
            bus->busBrkTripTime[i] = bus->relayTime;
        }
        bus->busBrkStatusMemory[i] = bus->busBrkStatus[i];
        bus->busBrkManualFlagMemory[i] = bus->busBrkManualFlag[i];
    }
}

int busStatusAuxJudge(int busStatus[], int i) {
    
    int flag = 0;
    switch (i) {
        case 0: if (((busStatus[0] == 1)&&(busStatus[1] == 0)) || ((busStatus[0] == 0)&&(busStatus[1] == 1))) {
                    flag = 1;
                }  break;
        case 1: if (((busStatus[2] == 1)&&(busStatus[3] == 0)) || ((busStatus[2] == 0)&&(busStatus[3] == 1))) {
                    flag = 1;
                }  break;
        case 2: if (((busStatus[0] == 1)&&(busStatus[2] == 0)) || ((busStatus[0] == 0)&&(busStatus[2] == 1))) {
                    flag = 1;
                }  break;
        case 3: if (((busStatus[1] == 1)&&(busStatus[3] == 0)) || ((busStatus[1] == 0)&&(busStatus[3] == 1))) {
                    flag = 1;
                }  break;        
    }

    return flag;
}
