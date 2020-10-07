
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

extern void busStarter_500kV(BusDevice* bus, Device line[]);
extern void busDiffRelay_500kV(BusDevice* bus, Device line[], int phase);
extern void antiSat1Judge_500kV(BusDevice* bus, Device line[]);
extern void antiSat2Judge_500kV(BusDevice* bus, Device line[]);
extern void busBrkFailureRelay_500kV(BusDevice* bus, Device* line, Device* lineRelay);
extern void busDeltaDiffRelay_500kV(BusDevice* bus, Device line[], int phase);
void busDiffTripJudge_500kV(BusDevice* bus);

void busTripGenerate_500kV(BusDevice* bus, Device* line);


void busDiffTripJudge(BusDevice* bus);

void bus_500kV(BusDevice* bus, Device* line, Device* brkRelay) {

    int busDiffEnable = bus->busDiffEnable;
    int i;

    if (bus->time >= 0.02) {
        busLineSynCheck_500kV(bus, line);
    }
    if (bus->time < 0.2) {
        // 等待仿真进入稳定状态
        for (i = 0; i < 5; i++) {
            bus->antiSat2Flag[i] = 1;
            bus->antiSat1Flag[i] = 1;
        }
        return;
    }
    busStarter_500kV(bus, line);
    // char s[100];
    // sprintf(s, "%lf", bus->time);
    // busWriteLog(bus, s);

    if (bus->busStartFlag[0] == 1 && busDiffEnable == 1) {
        if (bus->relayTime - bus->busStartTime[0] < 0.002) {
            antiSat1Judge_500kV(bus, line);
        } else {
            antiSat2Judge_500kV(bus, line);
        }
    }
    // 这里的差动保护中包含CT断线的判断，因此在内部通过判断装置启动标志位来决定差动是否出口，不在调用时判断启动标志
    for (i = 0; i < 3; i++) {
        busDiffRelay_500kV(bus, line, i);
        busDeltaDiffRelay_500kV(bus, line, i);
    }

    // 整合标志位及动作时间
    for (i = 0; i < 2; i++) {
        bus->busDiffTripFlag[i] = (bus->busDiffPhaseTripFlag[i][0] || bus->busDiffPhaseTripFlag[i][1] || bus->busDiffPhaseTripFlag[i][2]);
        if (bus->busDiffTripFlag[i] && bus->busDiffTripTime[i] != 0.0) {
            bus->busDiffTripTime[i] = bus->relayTime;
        }
        if (i) {
            bus->busDeltaDiffTripFlag[i] = (bus->busDeltaDiffPhaseTripFlag[i][0] || bus->busDeltaDiffPhaseTripFlag[i][1] || bus->busDeltaDiffPhaseTripFlag[i][2]);
            if (bus->busDeltaDiffTripFlag[i] && bus->busDeltaDiffTripTime[i] != 0.0) {
                bus->busDeltaDiffTripTime[i] = bus->relayTime;
            }
        }
    }

    if (bus->busStartFlag[0] == 1 && busDiffEnable == 1) {
        busDiffTripJudge_500kV(bus);
    }

    //busBrkFailureRelay_500kV(bus, line, brkRelay);

    busTripGenerate_500kV(bus, line);


}

void busDiffTripJudge_500kV(BusDevice* bus) {
    int enable = bus->busDiffEnable;

    if (!enable) {
        return;
    }

    int* sat1 = bus->antiSat1Flag; //对500kV母线，0/1就代表1/2母的抗饱和标志位
    int* sat2 = bus->antiSat2Flag;

    int i, phase = 0;
    int M1[2][3], M2[2][3];
    int CTBreak[2][phase];


    for (i = 0; i < 2; i++) {
        for (phase = 0; phase < 3; phase++) {
            M1[i][phase] = !sat1[i] && bus->busDeltaDiffPhaseTripFlag[i][phase];
            M2[i][phase] = !sat2[i] && bus->busDiffPhaseTripFlag[i][phase];
            CTBreak[i][phase] = bus->busCTBreakFlag[i][phase];

            if (M1[i][phase] || M2[i][phase] && !CTBreak[i][phase]) {
                bus->busTripFlag[i] = 1;
                busWriteLogWithPhase(bus, "%c母差动保护动作", i);
                return;
            }
        }
    }
    return;
    

}

void busTripGenerate_500kV(BusDevice* bus, Device* line) {

    int trip[2] = {bus->busTripFlag[0], bus->busTripFlag[1]};
    int i, j;
    int mode[36][2];

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            mode[i][j] = bus->busTopo[i][j];
            if (mode[i][j] != 0 && trip[j]) {
                line[i].tripFlag[0] = 1;
                line[i].tripFlag[1] = 1;
                line[i].tripFlag[2] = 1;
            }
        }
    }

}


