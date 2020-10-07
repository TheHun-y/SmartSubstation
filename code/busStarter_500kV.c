//
// 母线保护启动元件
//  2.电流工频变化量 3.差动电流
//
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>

void busStartJudge_500kV(BusDevice* bus, Device line[]);
int deltaIStart_500kV(BusDevice* bus, Device line[]);
int deltaIStartJudge_500kV(BusDevice* bus, Device line[], double k, double In);
int diffCurrentStart_500kV(BusDevice* bus, Device line[]);
int phasediffCurrentStart_500kV(BusDevice* bus, Device line[], int phase, double Icdzd);

void busStarter_500kV(BusDevice* bus, Device line[]){
    
    // 电流启动元件和差流启动元件动作后展宽500ms
    // 500kV母线无电压变化量启动，stratFlag/Time[1]空置，仍以2-电流，3-差流，0-总启动
    if (bus->busStartFlag[0] == 1) {
        if (bus->busStartFlag[2] == 1 || bus->busStartFlag[3] == 1) {
            if (bus->relayTime - bus->busStartTime[0] < 0.5) {
                bus->busStartFlag[0] = 1;
            } else {
                bus->busStartFlag[0] = 0;
                busStartJudge_500kV(bus, line);
            }
        } else {
            busStartJudge_500kV(bus, line);
        }
    } else {
        busStartJudge_500kV(bus, line);
    }
      
}

void busStartJudge_500kV(BusDevice* bus, Device line[]) {

    int flag2 = deltaIStart_500kV(bus, line);
    int flag3 = diffCurrentStart_500kV(bus, line);
    int i, j;

    if (flag2 == 1) {
        if (bus->busStartTimeCount[1] == 0) {
            bus->busStartTime[2] = bus->relayTime;
        }
        bus->busStartTimeCount[1]++;
        if (bus->busStartTimeCount[1] == 3 && bus->busStartFlag[2] == 0) {
            bus->busStartFlag[2] = 1; 
            bus->busStartFlag[0] = 1;
            bus->busStartTime[0] = bus->busStartTime[2];
            busWriteLog(bus,"母线保护电流工频变化量启动元件动作"); 
        }       
    } else {
        bus->busStartTimeCount[1] = 0;
    }

    if (flag3 == 1) {
        if (bus->busStartTimeCount[2] == 0) {
            bus->busStartTime[3] = bus->relayTime;
        }
        bus->busStartTimeCount[2]++;
        if (bus->busStartTimeCount[2] == 3 && bus->busStartFlag[3] == 0) {
            bus->busStartFlag[3] = 1; 
            bus->busStartFlag[0] = 1;
            bus->busStartTime[0] = bus->busStartTime[3];
            busWriteLog(bus,"母线保护差流启动元件动作"); 
        }       
    } else {
        bus->busStartTimeCount[2] = 0;
    }

    if (bus->busStartFlag[0] == 1) {
        for (i = 0; i < 3; i++) {
            bus->busStartTimeCount[i] = 0;
        }
    }

}

int deltaIStart_500kV(BusDevice* bus, Device line[]) {
    // 浮动门槛的比例系数
    double k = 0.0; 
    // 额定电压的0.5倍作为固定门槛
    double In = bus->side2CurrSetValue;
    int i, flag = 0;

    flag = deltaIStartJudge_500kV(bus, line, k, In);
    if (flag == 1) {
        return 1;
    }
    return 0;
}

int deltaIStartJudge_500kV(BusDevice* bus, Device line[], double k, double In) {
    // 变量本地化
    int i, j;
    double IresA = 0, memIresA = 0, deltaIresA = 0;
    double IresB = 0, memIresB = 0, deltaIresB = 0;
    double IresC = 0, memIresC = 0, deltaIresC = 0;
    int mode[36][2];
    for  (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            mode[i][j] = bus->busTopo[i][j];
        }
    }

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            if (mode[i][j] != 0) {
            // 以线路流入母线电流为正向（1），线路上电流从线路的 m 侧互感器流向线路的 n 侧
                IresA += fabs(line[i].filterIma[0]);
                IresB += fabs(line[i].filterImb[0]);
                IresC += fabs(line[i].filterImc[0]);

                memIresA += fabs(line[i].filterIma[POINTS]);
                memIresB += fabs(line[i].filterImb[POINTS]);
                memIresC += fabs(line[i].filterImc[POINTS]);
            }
        }
    }

    deltaIresA = IresA - memIresA;
    deltaIresB = IresB - memIresB;
    deltaIresC = IresC - memIresC;

    if (deltaIresA > (k*deltaIresA + 0.5*In)) {
        return 1;
    } else if (deltaIresB > (k*deltaIresB + 0.5*In)) {
        return 1;
    } else if (deltaIresC > (k*deltaIresC + 0.5*In)) {
        return 1;
    } else {
        return 0;
    }
}

int diffCurrentStart_500kV(BusDevice* bus, Device line[]) {
    // 差动电流启动整定值
    double Icdzd = bus->busDiffSetValue;
    int i, flag = 0;

    for (i = 0; i < 3; i++) {
        flag = phasediffCurrentStart_500kV(bus, line, i, Icdzd);
        if (flag == 1) {
            return 1;
        }
    }
    return 0;

}

int phasediffCurrentStart_500kV(BusDevice* bus, Device line[], int phase, double Icdzd) {
    // 变量本地化
    int i, j;
    int mode[36][2];

    Phasor Idiff, *temp;
    Idiff.real = 0.0; Idiff.img = 0.0;

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            mode[i][j] = bus->busTopo[i][j];
            if (mode[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], 1, phase);
                Idiff = phasorAdd(phasorNumMulti(mode[i][j], *temp), Idiff);
            }
        }
    }
    
    if (phasorAbs(Idiff) > Icdzd) {
        return 1;
    } else {
        return 0;
    }

}
