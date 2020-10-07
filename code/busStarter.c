//
// 母线保护启动元件
// 1.电压工频变化量 2.电流工频变化量 3.差动电流
//
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>

void busRecordMemoryUI(BusDevice* device);
void busStartJudge(BusDevice* bus, Device line[]);
int deltaUStart(BusDevice* bus);
int phaseDeltaUStart(BusDevice* device, int phase, int brkNum, double k, double Un);
int deltaIStart(BusDevice* bus, Device line[]);
int deltaIStartJudge(BusDevice* bus, Device line[], double k, double In);
int diffCurrentStart(BusDevice* bus, Device line[]);
int phaseDiffCurrentStart(BusDevice* bus, Device line[], int phase, double Icdzd);

void busStarter(BusDevice* bus, Device line[]){


    int i;
    
    // 电压启动元件和差流启动元件动作后展宽500ms
    if (bus->busStartFlag[0] == 1) {
        if (bus->busStartFlag[1] == 1 || bus->busStartFlag[3] == 1) {
            if (bus->relayTime - bus->busStartTime[0] < 0.5) {
                busStartJudge(bus, line);
                bus->busStartFlag[0] = 1;
            } else {
                bus->busStartFlag[0] = 0;
                busStartJudge(bus, line);
            }
        } else {
            busStartJudge(bus, line);
        }
    } else {
        busStartJudge(bus, line);
    }

    if (bus->busStartFlag[0] == 1) {
        if (bus->memoryRecordFlag == 0) {
            busRecordMemoryUI(bus);
            bus->memoryRecordFlag = 1;
        }
        for (i= 0; i < 24; i++) {
            line[i].startFlag = 1;
            line[i].startTime = bus->relayTime;
        }
    }
      
}

void busStartJudge(BusDevice* bus, Device line[]) {


    int flag1 = deltaUStart(bus);
    int flag2 = deltaIStart(bus, line);
    int flag3 = diffCurrentStart(bus, line);
    int i, j;

    if (flag1 == 1) {
        if (bus->busStartTimeCount[0] == 0) {
            bus->busStartTime[1] = bus->relayTime;
        }
        bus->busStartTimeCount[0]++;
        if (bus->busStartTimeCount[0] == 3 && bus->busStartFlag[1] == 0) {
            bus->busStartFlag[1] = 1; 
            bus->busStartFlag[0] = 1;
            bus->busStartTime[0] = bus->busStartTime[1];
            busWriteLog(bus,"母线保护电压工频变化量启动元件动作"); 
        }       
    } else {
        bus->busStartTimeCount[0] = 0;
    }

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

int deltaUStart(BusDevice* device) {
    // 浮动门槛的比例系数
    double k = 0.0;
    // 额定电压的0.05倍作为固定门槛
    double Un = 220.0;
    int i, j, flag = 0;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {

            flag = !device->busPTBreakFlag[j] && phaseDeltaUStart(device, i, j, k, Un);
            if (flag == 1) {
                return 1;
            }
        }
    }
    device->busStartFlag[1] = 0;
    return 0;

}

int phaseDeltaUStart(BusDevice* device, int phase, int brkNum, double k, double Un) {

    double* inst, * memory;
    double deltaU;
    switch(phase){
        case 0: switch(brkNum) {
            case 0: inst = device->busTieFiltVma; memory = &device->busTieFiltVma[POINTS]; break;
            case 1: inst = device->busTieFiltVna; memory = &device->busTieFiltVna[POINTS]; break;
            case 2: inst = device->busSecFiltVma; memory = &device->busSecFiltVma[POINTS]; break;
            case 3: inst = device->busSecFiltVna; memory = &device->busSecFiltVna[POINTS]; break;
        }
        case 1: switch(brkNum) {
            case 0: inst = device->busTieFiltVmb; memory = &device->busTieFiltVmb[POINTS]; break;
            case 1: inst = device->busTieFiltVnb; memory = &device->busTieFiltVnb[POINTS]; break;
            case 2: inst = device->busSecFiltVmb; memory = &device->busSecFiltVmb[POINTS]; break;
            case 3: inst = device->busSecFiltVnb; memory = &device->busSecFiltVnb[POINTS]; break;
        }
        case 2: switch(brkNum) {
            case 0: inst = device->busTieFiltVmc; memory = &device->busTieFiltVmc[POINTS]; break;
            case 1: inst = device->busTieFiltVnc; memory = &device->busTieFiltVnc[POINTS]; break;
            case 2: inst = device->busSecFiltVmc; memory = &device->busSecFiltVmc[POINTS]; break;
            case 3: inst = device->busSecFiltVnc; memory = &device->busSecFiltVnc[POINTS]; break;
        }
    }
    
    deltaU = *inst - *memory;

    if (deltaU > (k*deltaU + 0.05*Un)) {
        return 1;
    }

    return 0;
}

int deltaIStart(BusDevice* bus, Device line[]) {
    // 浮动门槛的比例系数
    double k = 0.0; 
    // 额定电压的0.5倍作为固定门槛
    double In = 1.0; //bus->side2CurrSetValue;
    int i, flag = 0;

    flag = deltaIStartJudge(bus, line, k, In);
    if (flag == 1) {
        return 1;
    }
    return 0;
}

int deltaIStartJudge(BusDevice* bus, Device line[], double k, double In) {
    // 变量本地化
    int mode[29][4];
    int i, j;
    double IresA = 0, memIresA = 0, deltaIresA = 0;
    double IresB = 0, memIresB = 0, deltaIresB = 0;
    double IresC = 0, memIresC = 0, deltaIresC = 0;

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            mode[i][j] = bus->busModeStatus[i][j];
        }
    }

    for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            // 以线路流入母线电流为正向（1），线路上电流从线路的 m 侧互感器流向线路的 n 侧
            if (mode[i][j] != 0) {
                IresA += fabs(line[i].filterIma[0]);
                IresB += fabs(line[i].filterImb[0]);
                IresC += fabs(line[i].filterImc[0]);

                memIresA += fabs(line[i].filterIma[POINTS]);
                memIresB += fabs(line[i].filterImb[POINTS]);
                memIresC += fabs(line[i].filterImc[POINTS]);
            }
        }
    }

    if (bus->bypassBusMode != 0) {
        IresA += fabs(bus->busTieFiltIma[0]);
        IresB += fabs(bus->busTieFiltImb[0]);
        IresC += fabs(bus->busTieFiltImc[0]);

        memIresA += fabs(bus->busTieFiltIma[POINTS]);
        memIresB += fabs(bus->busTieFiltImb[POINTS]);
        memIresC += fabs(bus->busTieFiltImc[POINTS]);
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

int diffCurrentStart(BusDevice* bus, Device line[]) {
    
    // 差动电流启动整定值
    double Icdzd = bus->busDiffSetValue;
    int i, flag = 0;

    for (i = 0; i < 3; i++) {
        flag = phaseDiffCurrentStart(bus, line, i, Icdzd);
        if (flag == 1) {
            return 1;
        }
    }
    return 0;

}

int phaseDiffCurrentStart(BusDevice* bus, Device line[], int phase, double Icdzd) {
    // 变量本地化
    int mode[29][4];
    int i, j;

    Phasor Idiff, *temp;
    Idiff.real = 0.0; Idiff.img = 0.0;
    
    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            mode[i][j] = bus->busModeStatus[i][j];
        }
    }

   /* for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            if (mode[i][j] == 1) {
                Idiff = phasorAdd(line[i].phasor[phase + 3], Idiff);
            } else if (mode[i][j] == -1) {
                Idiff = phasorSub(Idiff, line[i].phasor[phase + 3]);
            }
        }
    }*/

    for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            if (mode[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], mode[i][j], phase);
                Idiff = phasorAdd(phasorNumMulti( mode[i][j], *temp), Idiff);
            }
        }
    }

    if (bus->bypassBusMode != 0) {
        Idiff = phasorAdd(Idiff, phasorNumMulti(bus->busModeStatus[28][bus->bypassBusMode*2-2], bus->busPhasor[3+phase]));
    }
    // double tmp = bus->testDC;
    // bus->testDC = tmp > phasorAbs(Idiff) ? tmp:phasorAbs(Idiff);

    if (phasorAbs(Idiff) > Icdzd) {
        return 1;
    } else {
        return 0;
    }

}

void busRecordMemoryUI(BusDevice* device) {
    // 记录当前时刻
    int i;
    for (i = 3*POINTS; i > 2*POINTS; i--){
        inst2phasor(device->busTieFiltVma, i, &device->busTieMemVma[3*POINTS-i]);  inst2phasor(device->busTieFiltIma, i, &device->busTieMemIma[3*POINTS-i]);
        inst2phasor(device->busTieFiltVmb, i, &device->busTieMemVmb[3*POINTS-i]);  inst2phasor(device->busTieFiltImb, i, &device->busTieMemImb[3*POINTS-i]);
        inst2phasor(device->busTieFiltVmc, i, &device->busTieMemVmc[3*POINTS-i]);  inst2phasor(device->busTieFiltImc, i, &device->busTieMemImc[3*POINTS-i]);
        inst2phasor(device->busTieFiltVna, i, &device->busTieMemVna[3*POINTS-i]);  inst2phasor(device->busTieFiltIna, i, &device->busTieMemIna[3*POINTS-i]);
        inst2phasor(device->busTieFiltVnb, i, &device->busTieMemVnb[3*POINTS-i]);  inst2phasor(device->busTieFiltInb, i, &device->busTieMemInb[3*POINTS-i]);
        inst2phasor(device->busTieFiltVnc, i, &device->busTieMemVnc[3*POINTS-i]);  inst2phasor(device->busTieFiltInc, i, &device->busTieMemInc[3*POINTS-i]);

        inst2phasor(device->busSecFiltVma, i, &device->busSecMemVma[3*POINTS-i]);  inst2phasor(device->busSecFiltIma, i, &device->busSecMemIma[3*POINTS-i]);
        inst2phasor(device->busSecFiltVmb, i, &device->busSecMemVmb[3*POINTS-i]);  inst2phasor(device->busSecFiltImb, i, &device->busSecMemImb[3*POINTS-i]);
        inst2phasor(device->busSecFiltVmc, i, &device->busSecMemVmc[3*POINTS-i]);  inst2phasor(device->busSecFiltImc, i, &device->busSecMemImc[3*POINTS-i]);
        inst2phasor(device->busSecFiltVna, i, &device->busSecMemVna[3*POINTS-i]);  inst2phasor(device->busSecFiltIna, i, &device->busSecMemIna[3*POINTS-i]);
        inst2phasor(device->busSecFiltVnb, i, &device->busSecMemVnb[3*POINTS-i]);  inst2phasor(device->busSecFiltInb, i, &device->busSecMemInb[3*POINTS-i]);
        inst2phasor(device->busSecFiltVnc, i, &device->busSecMemVnc[3*POINTS-i]);  inst2phasor(device->busSecFiltInc, i, &device->busSecMemInc[3*POINTS-i]);

    }
    device->memoryRecordFlag = 1;

}
