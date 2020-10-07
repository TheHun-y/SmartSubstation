
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>


void findConnectBus(BusDevice* bus, int brkNo, int* result);
int busTieCloseDeadZoneRelay(BusDevice* bus, int brkNo);
int busTieOpenDeadZoneRelay(BusDevice* bus, int brkNo);;
int busSecOpenDeadZoneRelay(BusDevice* bus, int brkNo);

void busDeadZoneRelay(BusDevice* bus) {
    int openFlag[4], closeFlag[2]; // 分段无合位保护
    int i;

    for (i = 0; i < 2; i++) {
        openFlag[i] = busTieOpenDeadZoneRelay(bus, i);
        closeFlag[i] = busTieCloseDeadZoneRelay(bus, i);
        openFlag[i+2] = busSecOpenDeadZoneRelay(bus, i+2);
        if (openFlag[i]) {
            busWriteLogWithPhase(bus,"分位母联死区保护动作，%c号母联退出差动计算", i);
        }
        if (closeFlag[i]) {
            busWriteLogWithPhase(bus,"合位母联死区保护动作，%c号母联退出差动计算", i);
        }
        if (openFlag[i+2]) {
            busWriteLogWithPhase(bus,"分位分段死区保护动作，%c号分段退出差动计算", i);
        }
    }

    for (i = 0; i < 2; i++) {
        if (bus->busBrkDeadZoneFlag[i] || (bus->busBrkDeadZoneFlag[i] = openFlag[i] || closeFlag[i])) {
            busWriteLogWithPhase(bus,"母联死区保护动作，%c号母联退出差动计算", i);
        }
        if (bus->busBrkDeadZoneFlag[i+2] || (bus->busBrkDeadZoneFlag[i+2] = openFlag[i+2])) {
            busWriteLogWithPhase(bus,"分段死区保护动作，%c号分段退出差动计算", i);
        }
    }

}


int busSecOpenDeadZoneRelay(BusDevice* bus, int brkNo) {

    int ch = 0;
    for (ch = 0; ch < 4; ch++) {
        if (bus->busModeStatus[24+brkNo][ch] != 0) {
            break;
        }
    }
    if (ch == 4) {
        // 未计入差动的开关不必进行死区判断
        return 0;
    }
    

    int secManual = bus->busBrkManualFlag[brkNo]; // 分段手合开入
    int TWJ = !(bus->busBrkStatus[brkNo*3] || bus->busBrkStatus[brkNo*3+1] || bus->busBrkStatus[brkNo*3+2]); // 断路器状态：1-闭合，0-断开，与TWJ相反
    int IndeOperate = bus->busIndependentOpEnable[brkNo];
    double In = 1.0; // 额定电流整定值
    double Ibrk[3];
    int currentFlag = 0;
    int connectBus[2]; // 连接的母线编号：0-3

     int i = 0, k = 0;

    findConnectBus(bus, brkNo, connectBus);

    for (i = 0; i < 3; i++) {
        Ibrk[i] = phasorAbs(bus->busPhasor[brkNo*6 + 3 + i]);
        if (Ibrk[i] > 0.04*In) {
            currentFlag = 1;
            break;
        }
    }

    int M1 = TWJ && IndeOperate;

    if (!currentFlag) {
        if (bus->busOpenDeadZoneTime[brkNo] == 0) {
            bus->busOpenDeadZoneTime[brkNo] = bus->relayTime;
        }
        if (bus->relayTime - bus->busOpenDeadZoneTime[brkNo] >= 0.4 && !secManual && M1) {
            bus->busOpenDeadZoneTime[brkNo] = 0;
            return 1;
        } else {
            return 0;
        }
    } else {
        bus->busOpenDeadZoneTime[brkNo] = 0;
        return 0;
    }
}

int busTieOpenDeadZoneRelay(BusDevice* bus, int brkNo) {
    int ch = 0;
    for (ch = 0; ch < 4; ch++) {
        if (bus->busModeStatus[24+brkNo][ch] != 0) {
            break;
        }
    }
    if (ch == 4) {
        // 未计入差动的开关不必进行死区判断
        return 0;
    }

    int TWJ = !(bus->busBrkStatus[brkNo*3] || bus->busBrkStatus[brkNo*3+1] || bus->busBrkStatus[brkNo*3+2]); // 断路器状态：1-闭合，0-断开，与TWJ相反
    int IndeOperate = bus->busIndependentOpEnable[brkNo];
    double In = 1.0; // 额定电流整定值
    double Ibrk[3];
    int currentFlag = 0;
    int connectBus[2]; // 连接的母线编号：0-3

    int busRun[2];
    int i = 0, k = 0;

    findConnectBus(bus, brkNo, connectBus);
    for (i = 0; i < 3; i++) {
        Ibrk[i] = phasorAbs(bus->busPhasor[brkNo*6 + 3 + i]);
        if (Ibrk[i] > 0.04*In) {
            currentFlag = 1;
            break;
        }
    }
    busRun[0] = bus->busRunStatus[connectBus[0]];
    busRun[1] = bus->busRunStatus[connectBus[1]];

    int M1 = busRun[0] && busRun[1] && !currentFlag;
    int M2 = TWJ && IndeOperate;
    if (M1 == 1) {
        if (bus->busOpenDeadZoneTime[brkNo] == 0) {
            bus->busOpenDeadZoneTime[brkNo] = bus->relayTime;
        }
        if (bus->relayTime - bus->busOpenDeadZoneTime[brkNo] >= 0.4 && M2) {
            bus->busOpenDeadZoneTime[brkNo] = 0;
            return 1;
        } else {
            return 0;
        }
    } else {
        bus->busOpenDeadZoneTime[brkNo] = 0;
        return 0;
    }
}

int busTieCloseDeadZoneRelay(BusDevice* bus, int brkNo) {
    
    if (bus->busBrkDeadZoneFlag[brkNo] == 1) {
        return 0;
    }
    int TWJ = !(bus->busBrkStatus[brkNo*3] || bus->busBrkStatus[brkNo*3+1] || bus->busBrkStatus[brkNo*3+2]); // 断路器状态：1-闭合，0-断开，与TWJ相反
    double In = 1.0; // 额定电流整定值
    double Ibrk[3];
    int currentFlag = 0;
    int connectBus[2] = {0, 0}; // 连接的母线编号：0-3
    int dcFlag, xcFlag[2], brkTripFlag;

    int i = 0, k = 0;

    findConnectBus(bus, brkNo, connectBus);

    dcFlag = bus->busDiffTripFlag[0];
    xcFlag[0] = bus->busDiffTripFlag[connectBus[0]+1];
    xcFlag[1] = bus->busDiffTripFlag[connectBus[1]+1];
    brkTripFlag = bus->busBrkTripFlag[brkNo];

    for (i = 0; i < 3; i++) {
        Ibrk[i] = phasorAbs(bus->busPhasor[brkNo*6 + 3 + i]);
        if (Ibrk[i] > 0.1*In) {
            currentFlag = 1;
            break;
        }
    }
    int M1 = TWJ && currentFlag;
    int M2 = dcFlag && (xcFlag[0] || xcFlag[1]) && brkTripFlag;
    if (M1 && M2) {
        if (bus->busCloseDeadZoneTime[brkNo] == 0) {
            bus->busCloseDeadZoneTime[brkNo] = bus->relayTime;
        }
        if (bus->relayTime - bus->busCloseDeadZoneTime[brkNo] >= 0.15) {
            bus->busCloseDeadZoneTime[brkNo] = 0;
            return 1;
        } else {
            return 0;
        }
    } else {
        bus->busCloseDeadZoneTime[brkNo] = 0;
        return 0;
    }

}

void findConnectBus(BusDevice* bus, int brkNo, int* result) {
    int i, k = 0;
    for (i = 0; i < 4; i++) {
        if (bus->busTopo[brkNo+24][i] != 0 && k < 2) {
            result[k] = i;
            k++;
        }
    }
}
