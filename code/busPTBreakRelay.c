
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

void busPTBreakRelay1(BusDevice* bus, Device* line, int busNo);
void findBusConnectBrk(BusDevice* bus, int busNo, int* result);

void busPTBreakRelay(BusDevice* bus, Device* line) {
    
    int i;

    if (bus->busStartFlag[1] == 0 && bus->busStartFlag[2] == 0) {
        for (i = 0; i < 4; i++) {
            busPTBreakRelay1(bus, line, i);
        }
    }
}

void busPTBreakRelay1(BusDevice* bus, Device* line, int busNo) {

    double Un = 220.0;
    double In = 1.0;
    double U2;
    Phasor U[3];

    int i, j;
    int flag1 = 0, flag2 = 0, flag3 = 0;
    for (i = 0; i < 3; i++) {
        U[i] = bus->busPhasor[6*busNo+i];
    }
    U2 = phasorAbs(phasorSeq(U[0], U[1], U[2], 2));
    
    if (3.0*U2 > 0.2*Un) {
        flag1 = 1;
        if (bus->busPTBreakStartTime[busNo][0] == 0) {
            bus->busPTBreakStartTime[busNo][0] = bus->relayTime;
        } else if (bus->relayTime - bus->busPTBreakStartTime[busNo][0] >= 0.4) {
            bus->busPTBreakFlag[busNo] = 1;
            bus->busPTBreakStartTime[busNo][0] = 0.0;
            busWriteLogWithPhase(bus, "%c段母线PT断线告警！", busNo);
        }
    } else {
        bus->busPTBreakStartTime[busNo][0] = 0.0;
    }
    

    double Uabs[3];
    Uabs[0] = phasorAbs(U[0]); Uabs[1] = phasorAbs(U[1]); Uabs[2] = phasorAbs(U[2]);
    double sum = Uabs[0] + Uabs[1] + Uabs[2];
    int currFlag = 0;
    int connectBrk[2];
    ///////////////////////////
    // if (busNo == 0) bus->testDC = sum;

    findBusConnectBrk(bus, busNo, connectBrk);

    for (i = 0; i < 2; i++) {
        if (phasorAbs(bus->busPhasor[connectBrk[i]]) > 0.04*In) {
            currFlag = 1;
            break;
        }
    }

    if (currFlag == 0) {
        for (i = 0; i < 24; i++) {
            for (j = 0; j < 3; j++) {
                if (phasorAbs(line[i].phasor[j]) > 0.04*In) {
                    currFlag = 1;
                    break;
                }
            }
            if (currFlag == 1) break;
        }
    }

    if (sum < Un && (Uabs[0] > 0.7*Un || Uabs[1] > 0.7*Un || Uabs[2] > 0.7*Un || currFlag == 1)) {
        flag2 = 1;
        if (bus->busPTBreakStartTime[busNo][1] == 0) {
            bus->busPTBreakStartTime[busNo][1] = bus->relayTime;
        } else if (bus->relayTime - bus->busPTBreakStartTime[busNo][1] >= 0.4) {
            bus->busPTBreakFlag[busNo] = 1;
            bus->busPTBreakStartTime[busNo][1] = 0.0;
            busWriteLogWithPhase(bus, "%c段母线PT断线告警！", busNo);
        }
    } else {
        bus->busPTBreakStartTime[busNo][1] = 0.0;
    }

    if (bus->voltBlockFlag[busNo] == 0) {
        flag3 = 1;
        if (bus->busPTBreakStartTime[busNo][2] == 0) {
            bus->busPTBreakStartTime[busNo][2] = bus->relayTime;
        } else if (bus->relayTime - bus->busPTBreakStartTime[busNo][2] >= 3) {
            bus->busPTBreakFlag[busNo] = 1;
            bus->busPTBreakStartTime[busNo][2] = 0.0;
            busWriteLogWithPhase(bus, "%c段母线PT断线告警！", busNo);
        }
    } else {
        bus->busPTBreakStartTime[busNo][2] = 0.0;
    }

    if (!flag1 && !flag2 && !flag3) {
        if (bus->busPTBreakReturnTime[busNo] == 0) {
            bus->busPTBreakReturnTime[busNo] = bus->relayTime;
        } else if (bus->relayTime - bus->busPTBreakReturnTime[busNo] >= 10) {
            bus->busPTBreakFlag[busNo] = 0;
            bus->busPTBreakReturnTime[busNo] = 0.0;
            busWriteLogWithPhase(bus, "%c段母线PT恢复正常运行！", busNo);
        }
    } else {
        bus->busPTBreakReturnTime[busNo] = 0.0;
    }
    
}

void findBusConnectBrk(BusDevice* bus, int busNo, int* result) {
    int i, k = 0;
    for (i = 0; i < 4; i++) {
        if (bus->busTopo[i+24][busNo] != 0 && k < 2) {
            result[k] = i;
            k++;
        }
    }
}

