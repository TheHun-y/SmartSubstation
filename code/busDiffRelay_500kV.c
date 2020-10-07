
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

extern void findLineConnectBus(BusDevice* bus, int lineNo, int* result);


void busDiffRelay_500kV(BusDevice* bus, Device line[], int phase) {

    int modeMatrix[36][2];
    int i, j;

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            modeMatrix[i][j] = bus->busTopo[i][j];
        }
    }
    
    // 比率制动系数
    double k = 0.5;
    // 差动启动整定值
    double Icdzd = bus->busDiffSetValue;
    // CT断线定值
    double busCTbreakSetValue = bus->busCTBreakBlockSetValue;
    double busCTwarnSetValue = bus->busCTBreakWarnSetValue;


    // 大差、小差的差动电流及制动电流
    Phasor Idiff[2];
    double Ires[2];
    // 判据标志
    int judgeFlag[2] = {0};

    Phasor* temp;

    Idiff[0].real = 0.0; Idiff[0].img = 0.0;
    Idiff[1].real = 0.0; Idiff[1].img = 0.0;
    Ires[0] = 0.0; Ires[1] = 0.0;

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            if (modeMatrix[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][j], phase);
                Idiff[j] = phasorAdd(phasorNumMulti( modeMatrix[i][j], *temp), Idiff[j]);
                Ires[j] = Ires[j] + phasorAbs(*temp);
            }
        }
    }

    double CTTimeSet = 0.5;
    char s[100];
    // CT断线
    for (i = 0; i < 2; i++) {
        if (phasorAbs(Idiff[i]) > busCTbreakSetValue) {
            if (bus->busCTBreakStartTime[i][0][phase] == 0) {
                bus->busCTBreakStartTime[i][0][phase] = bus->relayTime;
            }
            if (bus->relayTime - bus->busCTBreakStartTime[i][0][phase] > CTTimeSet) {
                bus->busCTBreakFlag[i][phase] = 1;
                sprintf(s, "%c号母联%c相CT断线闭锁", '1'+i, 'A'+phase);
                busWriteLog(bus, s);
                bus->busCTBreakStartTime[i][0][phase] = 0;
            }
        }
        if (phasorAbs(Idiff[i]) > busCTwarnSetValue) {
            if (bus->busCTBreakWarnStartTime[i][1][phase] == 0) {
                bus->busCTBreakWarnStartTime[i][1][phase] = bus->relayTime;
            }
            if (bus->relayTime - bus->busCTBreakWarnStartTime[i][1][phase] > CTTimeSet) {
                bus->busCTBreakWarnFlag[i][phase] = 1;
                sprintf(s, "%c号母联%c相CT断线告警", '1'+i, 'A'+phase);
                busWriteLog(bus, s);
                bus->busCTBreakWarnStartTime[i][1][phase] = 0;
            }
        }
    }

    if (bus->busDiffEnable == 0 || bus->busStartFlag[0] == 0) {
        return;
    }
    // char test[100];
    // sprintf(test, "%d = %lf", 0 , phasorAbs(Idiff[0]));
    // if (bus->time > 0.405) {
    //     busWriteLog(bus, test);
    // }
    for (i = 0; i < 2; i++) {
        if (phasorAbs(Idiff[i]) > Icdzd && phasorAbs(Idiff[i]) > k*Ires[i]) {
            bus->busDiffPhaseTripFlag[i][phase] = 1;
            busWriteLogWithPhase(bus, "%c母线常规差动元件动作", i);
        }
    }
   

}

void busDeltaDiffRelay_500kV(BusDevice* bus, Device line[], int phase) {
    // 工频变化量差动部分整定值
    double kdelta,deltaIcdzd, k;
    kdelta = 0.0; deltaIcdzd = 0.5;
    k = 0.5;

    Phasor* mem, *temp;
    int i, j;
    int modeMatrix[36][2];
    Phasor Idiff[2], IdiffMem[2];
    double ampIdiff[2], Ires[2];
    // 判据标志
    int judgeFlag[2];
    int judgeFlagAux[2];

    Idiff[0].real = 0.0; Idiff[0].img = 0.0; IdiffMem[0].real = 0.0; IdiffMem[0].img = 0.0; 
    Idiff[1].real = 0.0; Idiff[1].img = 0.0; IdiffMem[1].real = 0.0; IdiffMem[1].img = 0.0;
    Ires[0] = 0.0; Ires[1] = 0.0;

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            modeMatrix[i][j] = bus->busTopo[i][j];
        }
    }

    for (i = 0; i < 2; i++) {
        judgeFlagAux[i] = busDeltaDiffAuxJudge_500kV(bus, line, phase, i);
    }

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            if (modeMatrix[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][j], phase);
                Idiff[j] = phasorAdd(phasorNumMulti(modeMatrix[i][j],*temp), Idiff[j]);
                Ires[j] += phasorAbs(phasorSub(*temp, *findLineMemory(bus, line, i, phase, modeMatrix[i][j])));
                IdiffMem[j] = phasorAdd(phasorNumMulti(modeMatrix[i][j],*findLineMemory(bus, line, i, phase, modeMatrix[i][j])), IdiffMem[j]);
            }
        }
    }
    
    for (i = 0; i < 2; i++) {
        ampIdiff[i] = phasorAbs(phasorSub(Idiff[i], IdiffMem[i]));
    }

    for (i = 0; i < 2; i++) {
        if ((ampIdiff[i] > deltaIcdzd + kdelta*ampIdiff[i]) && (ampIdiff[i] > k*Ires[i])) {
            judgeFlag[i] = 1;
        }        
    }
    
    for (i = 0; i < 2; i++) {
        if ((judgeFlagAux[i] == 1) && (judgeFlag[i] == 1)) {
            bus->busDeltaDiffPhaseTripFlag[i][phase] = 1;
            // bus->busDeltaDiffTripTime[i] = bus->relayTime;
            busWriteLogWithPhase(bus, "%c母线变化量差动元件动作（未经抗饱和）",i);
        }
    }
   
}

int busDeltaDiffAuxJudge_500kV(BusDevice* bus, Device line[], int phase, int busNo) {
    // 配合工频变化量差动的0.2常规比率差动整定值
    double k = 0.2;
    // 变量本地化
    int modeMatrix[36][2];
    int i, j;
    double Icdzd = bus->busDiffSetValue;

    // 大差、小差的差动电流及制动电流
    Phasor Idiff, *temp;
    double Ires;

    Idiff.real = 0.0; Idiff.img = 0.0;
    Ires = 0.0;

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            modeMatrix[i][j] = bus->busTopo[i][j];
        }
    }

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            if (modeMatrix[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][j], phase);
                Idiff = phasorAdd(phasorNumMulti(modeMatrix[i][j],*temp), Idiff);
                Ires += phasorAbs(*temp);
            }
        }
    }

    if ((phasorAbs(Idiff) > Icdzd) && (phasorAbs(Idiff) > k*Ires)) {
        return 1;
    }

    return 0;
}

