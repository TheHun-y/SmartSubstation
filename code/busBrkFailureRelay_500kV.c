
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

void busBrkFailureRelay_500kV(BusDevice* bus, Device* line, Device* lineRelay) {
    int enable = bus->brkFailureTripEnable;
    if (!enable) {
        return;
    }
    int brkStart[36];
    int brkLocalJudge[36];
    int mode[36][2];

    int i = 0, j = 0, k = 0;

    for (i = 0; i < 36; i++) {
        brkStart[i] = lineRelay[i].sampleBrkFailureFlag; //此处待断路器保护完成后应该修改为对应的失灵保护出口标志
        brkLocalJudge[i] = line[i].sampleBrkFailureFlag;
        for (j = 0; j < 2; j++) {
            mode[i][j] = bus->busTopo[i][j];
        }
    }

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            if (mode[i][j] != 0) {
                if (bus->brkFailureStartTime_500kV[i] != 0) {
                    if (bus->relayTime - bus->brkFailureStartTime_500kV[i] >= 0.05) {
                        bus->busTripFlag[j] = 1;
                        for (k = 0; k < 36; k++) {
                            if (mode[k][j] != 0) {
                                bus->brkFailureStartTime_500kV[k] = 0;
                            }
                        }
                    }
                } else {
                    if (brkLocalJudge[i] && brkStart[i]) {
                        bus->brkFailureStartTime_500kV[i] = bus->relayTime;
                    } else {
                        bus->brkFailureStartTime_500kV[i] = 0;
                    }
                } 
            }
        }
    }
}
