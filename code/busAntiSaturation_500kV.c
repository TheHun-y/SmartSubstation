
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>
#include <math.h>

int blcdJudge_500kV(BusDevice* bus, Device line[], int busNum);
int phaseBlcdJudge_500kV(BusDevice* bus, Device line[], int busNum, int phase);
void antiSat2Judge_500kV(BusDevice* bus, Device line[]);
void antiSat2JudgeXc_500kV(BusDevice* bus, Device line[], int busNum, int phase);

void antiSat1Judge_500kV(BusDevice* bus, Device line[]) {

    double deltaStartTime = 0.0;
    int blcdStartFlag[5];
    int i;
    // 取两个元件中先动作的元件的启动时间
    deltaStartTime = bus->busStartTime[2];

    for (i= 0; i < 2; i++) {
        blcdStartFlag[i] = blcdJudge(bus, line, i);
        if (bus->antiSat1FinalOutTag[i] == 0) { // 标志抗饱和是否已经判断出结果
            if (blcdStartFlag[i] == 1) {
                if (deltaStartTime != 0) {
                    if (bus->blcdStartTime[i] - deltaStartTime >= 0.003) {
                        // 饱和
                        bus->antiSat1Flag[i] = 1;
                        //busWriteLogWithPhase(bus,"%c段母线时序检测饱和元件闭锁", i-1);
                        bus->antiSat1FinalOutTag[i] = 1;
                    } else {
                        // 不饱和
                        bus->antiSat1Flag[i] = 0;
                        busWriteLogWithPhase(bus,"%c段母线时序检测饱和元件开放", i-1);
                        bus->antiSat1FinalOutTag[i] = 1;
                    }
                } else {
                    // 不饱和
                    bus->antiSat1Flag[i] = 0;
                    busWriteLogWithPhase(bus,"%c段母线时序检测饱和元件开放", i-1);
                    bus->antiSat1FinalOutTag[i] = 1;
                }
            } else {
                // 当抗饱和检测中的工频变化量差流瞬时值元件未动作而启动元件已经动作时，默认饱和，阻止差流保护跳闸
                if (deltaStartTime != 0 && bus->relayTime - deltaStartTime >= 0.003) {
                    bus->antiSat1Flag[i] = 1;
                    //busWriteLogWithPhase(bus,"%c段母线时序检测饱和元件闭锁", i-1);
                    bus->antiSat1FinalOutTag[i] = 1;
                }
            }
        }
    }
   
   
    return;
}

int blcdJudge_500kV(BusDevice* bus, Device line[], int busNum) {
    int phase = 0;
    int flag = 0;
    for (phase = 0; phase < 3; phase++) {
        flag = phaseBlcdJudge_500kV(bus, line, busNum, phase);
        if (flag == 1) {
            bus->blcdStartTime[busNum] = bus->relayTime;
            return 1;
        }
    } 
    return 0;
}

int phaseBlcdJudge_500kV(BusDevice* bus, Device line[], int busNum, int phase) {
    
    double kdelta,deltaIcdzd, kxc;
    kdelta = 0.0; deltaIcdzd = 0.1;
    kxc = 0.65;

    int i, j;
    int modeMatrix[36][2];
    double IdiffXc = 0;
    double IresXc = 0;
    double* inst;

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            modeMatrix[i][j] = bus->busTopo[i][j];
        }
    }

    for (i = 0; i < 36; i++) {
        inst = lineInstSearchSideM(line, phase, i)+(line[i].busSynFlag && bus->busDiffDelayFlag);
        IdiffXc = IdiffXc + (modeMatrix[i][busNum]*(inst[0]-inst[2*POINTS]));
        IresXc = IresXc + fabs((double)modeMatrix[i][busNum]*(inst[0]-inst[2*POINTS]));
    }

    IdiffXc = fabs(IdiffXc);
    
    if ((IdiffXc > kdelta*IdiffXc + deltaIcdzd) && (IdiffXc > kxc*IresXc)) {
        return 1;
    }
    return 0;
}

void antiSat2Judge_500kV(BusDevice* bus, Device line[]) {
    
    int i, j;
    
    for (i = 0; i < 2; i++) {
        bus->antiSat2Flag[i] = 1;
        for (j = 0; j < 3; j++) {
            antiSat2JudgeXc_500kV(bus, line, i, j);
            if (bus->antiSat2Flag[i] == 0) {
                //busWriteLogWithPhase(bus, "%c段母线小差谐波制动闭锁", i-1);
                break;
            }
        }
        if (j < 3) {
            busWriteLogWithPhase(bus, "%c段母线小差谐波制动开放保护动作", i);
        }
    }
        
}

void antiSat2JudgeXc_500kV(BusDevice* bus, Device line[], int busNum, int phase) {

    double k2 = 0.4, k3 = 0.4;
    double IdiffAmp[3];
    Phasor Idiff[3];
    double* inst;
    int i, j, k;
    int mode[36][2];
    // 初始化
    for (i = 0; i < 3; i++) {
        Idiff[i].real = 0.0;
        Idiff[i].img = 0.0;
        IdiffAmp[i] = 0.0;
    }

    for (i = 0; i < 36; i++) {
        for (j = 0; j < 2; j++) {
            mode[i][j] = bus->busTopo[i][j];
        }
    }

    for (i = 0; i < 36; i++) {
        if (mode[i][busNum] == 1) {
            switch (phase) {
                case 0: inst = line[i].instIma; break;
                case 1: inst = line[i].instImb; break;
                case 2: inst = line[i].instImc; break;
            }
            inst += (bus->busDiffDelayFlag && line[i].busSynFlag);
            for (k = 0; k < 3; k++) {
                Idiff[k] = phasorAdd(Idiff[k], phasorForHarmonic(inst, 0, k+1));
            }
        } else if (mode[i][busNum] == -1) {
            switch (phase) {
                case 0: inst = line[i].instIma; break;
                case 1: inst = line[i].instImb; break;
                case 2: inst = line[i].instImc; break;
            }
            inst += (bus->busDiffDelayFlag && line[i].busSynFlag);
            for (k = 0; k < 3; k++) {
                Idiff[k] = phasorSub(Idiff[k], phasorForHarmonic(inst, 0, k+1));
            }
        }
    }
    
    for (i = 0; i < 3; i++) {
        IdiffAmp[i] = phasorAbs(Idiff[i]);
    }
    // char test[100];
    // sprintf(test, "%d = %lf",busNum, IdiffAmp[0]);
    // if (bus->time > 0.410) {
    //     busWriteLog(bus, test);
    // }
    
    if (IdiffAmp[0] < 0.1) { // 差流要到达一定水平才进行判断
        return;
    }

    if ((IdiffAmp[1] >= IdiffAmp[0]*k2) || (IdiffAmp[2] >= IdiffAmp[0]*k3)) {
        bus->antiSat2Flag[busNum] = 1;
    } else {
        bus->antiSat2Flag[busNum] = 0;
    }
}

