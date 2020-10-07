
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>


void backupDiffRelay(BusDevice* bus) {
    // 电压闭锁不在本函数内判断，在对应母线的跳闸最终判断时再经电压闭锁
    int busTripFlag[4], antiSatFlag = 1;
    double dcTripTime;
    int i;

    dcTripTime = bus->busDiffTripTime[0];
    if ((bus->antiSat1Flag[0] == 0) && (bus->antiSat2Flag[0] == 0)) {
        antiSatFlag = 0;
    } else {
        return;
    }
    for (i = 0; i < 5; i++) {
        busTripFlag[i] = bus->busDiffTripFlag[i];
        if (i != 0) {
            if (busTripFlag[i] == 1) {
                // 若已经有任一母线的小差动作，则直接退出后备保护判断
                return;
            }
        }
    }
    // 大差抗饱和持续动作且无母线跳闸
    if ((antiSatFlag == 0) && (busTripFlag[0] == 1)) {
        if (bus->relayTime - dcTripTime >= 0.24) {
            bus->busDiffBackupTripFlag[0] = 1;
            busWriteLog(bus, "母线差动保护后备I段动作");
        }
        if (bus->relayTime - dcTripTime >= 0.48) {
            bus->busDiffBackupTripFlag[1] = 1;
            busWriteLog(bus, "母线差动保护后备II段动作");
        }
    }
}

