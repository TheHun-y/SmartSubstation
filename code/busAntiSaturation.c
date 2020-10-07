
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>
#include <math.h>

int blcdJudge(BusDevice* bus, Device line[], int busNum);
int phaseDcBlcdJudge(BusDevice* bus, Device line[], int phase);

int phaseXcBlcdJudge(BusDevice* bus, Device line[], int busNum, int phase);
void antiSat2Judge(BusDevice* bus, Device line[]);
void antiSat2JudgeXc(BusDevice* bus, Device line[], int busNum, int phase);
void antiSat2JudgeDc(BusDevice* bus, Device line[], int phase);

void antiSat1Judge(BusDevice* bus, Device line[]) {

    double deltaStartTime = 0.0;
    int blcdStartFlag[5];
    int i;
    // 取两个元件中先动作的元件的启动时间
    deltaStartTime = bus->busStartTime[1] > bus->busStartTime[2] ? bus->busStartTime[1] : bus->busStartTime[2];


    for (i= 0; i < 5; i++) {
        blcdStartFlag[i] = blcdJudge(bus, line, i);
        if (bus->antiSat1FinalOutTag[i] == 0) {
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

int blcdJudge(BusDevice* bus, Device line[], int busNum) {
    int phase = 0;
    int flag = 0;

    if (busNum == 0) {
        for (phase = 0; phase < 3; phase++) {
            flag = phaseDcBlcdJudge(bus, line, phase);
            if (flag == 1) {
                bus->blcdStartTime[busNum] = bus->relayTime;
                return 1; 
            }
        }
        return 0;
    } else {
        for (phase = 0; phase < 3; phase++) {
            flag = phaseXcBlcdJudge(bus, line, busNum, phase);
            if (flag == 1) {
                bus->blcdStartTime[busNum] = bus->relayTime;
                return 1;
            }
        } 
        return 0;
    }
    
}

int phaseDcBlcdJudge(BusDevice* bus, Device line[], int phase) {

    double kdelta,deltaIcdzd, kdc;
    kdelta = 0.0; deltaIcdzd = 0.5;
    kdc = 0.65;

    double mem;
    int i, j;
    int modeMatrix[29][4];
    double IdiffDc = 0;
    double IresDc = 0;
    double* inst;

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            modeMatrix[i][j] = bus->busModeStatus[i][j];
        }
    }
    int busNum = 0;
    for (i = 0; i < 24; i++) {
        for (busNum = 0; busNum < 4; busNum++) {
            inst = lineInstSearchSideM(line, phase, i)+(line[i].busSynFlag && bus->busDiffDelayFlag);
            IdiffDc = IdiffDc + (modeMatrix[i][busNum]*(inst[0]-inst[2*POINTS]));
            IresDc = IresDc + fabs((double)modeMatrix[i][busNum]*(inst[0]-inst[2*POINTS]));  
        }   
    }

    if (bus->bypassBusMode != 0) {
        for (i = 0; i < 4; i++) {
            if (modeMatrix[28][i] != 0) break;
        }
        inst = busInstSearch(bus, phase, 0)+bus->busDiffDelayFlag;
        IdiffDc = IdiffDc + (modeMatrix[28][i]*(inst[0]-inst[2*POINTS]));
        IresDc = IresDc + fabs((double)modeMatrix[28][i]*(inst[0]-inst[2*POINTS])); 
    }

    IdiffDc = fabs(IdiffDc);
    
    if ((IdiffDc > kdelta*IdiffDc + deltaIcdzd) && (IdiffDc > kdc*IresDc)) {
        return 1;
    }
    return 0;
}

int phaseXcBlcdJudge(BusDevice* bus, Device line[], int busNum, int phase) {
    
    double kdelta,deltaIcdzd, kxc;
    kdelta = 0.0; deltaIcdzd = 0.5;
    kxc = 0.65;

    double mem;
    int i, j;
    int modeMatrix[29][4];
    double IdiffXc = 0;
    double IresXc = 0;
    double* inst;

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            modeMatrix[i][j] = bus->busModeStatus[i][j];
        }
    }

    for (i = 0; i < 24; i++) {
        inst = lineInstSearchSideM(line, phase, i)+(line[i].busSynFlag && bus->busDiffDelayFlag);
        IdiffXc = IdiffXc + (modeMatrix[i][busNum-1]*(inst[0]-inst[2*POINTS]));
        IresXc = IresXc + fabs((double)modeMatrix[i][busNum-1]*(inst[0]-inst[2*POINTS]));
        
    }

    for (i = 0; i < 4; i++) {
        if (i == 0) {
            if (bus->bypassBusMode == 0){
                inst = busInstSearch(bus, phase, i)+bus->busDiffDelayFlag;
                IdiffXc = IdiffXc + (modeMatrix[i+24][busNum-1]*(inst[0]-inst[2*POINTS]));
                IresXc = IresXc + fabs((double)modeMatrix[i+24][busNum-1]*(inst[0]-inst[2*POINTS]));
            } else {
                inst = busInstSearch(bus, phase, i)+bus->busDiffDelayFlag;
                IdiffXc = IdiffXc + (modeMatrix[28][busNum-1]*(inst[0]-inst[2*POINTS]));
                IresXc = IresXc + fabs((double)modeMatrix[28][busNum-1]*(inst[0]-inst[2*POINTS]));
            }
        } else {
            inst = busInstSearch(bus, phase, i)+bus->busDiffDelayFlag;
            IdiffXc = IdiffXc + (modeMatrix[i+24][busNum-1]*(inst[0]-inst[2*POINTS]));
            IresXc = IresXc + fabs((double)modeMatrix[i+24][busNum-1]*(inst[0]-inst[2*POINTS]));
        }
    }

    IdiffXc = fabs(IdiffXc);
    
    if ((IdiffXc > kdelta*IdiffXc + deltaIcdzd) && (IdiffXc > kxc*IresXc)) {
        return 1;
    }
    return 0;
}


void antiSat2Judge(BusDevice* bus, Device line[]) {
    
    int i, j;
    
    for (i = 0; i < 5; i++) {
        bus->antiSat1Flag[i] = 1;
        bus->antiSat2Flag[i] = 1;
        if (i == 0) {
            for (j = 0; j < 3; j++) {
                antiSat2JudgeDc(bus, line, j);
                if (bus->antiSat2Flag[i] == 0) {
                    //busWriteLog(bus, "母线大差谐波制动闭锁");
                    break;
                }
            }
            if (j < 3) {
                busWriteLog(bus, "母线大差谐波制动开放保护动作");
            }
        } else {
            for (j = 0; j < 3; j++) {
                antiSat2JudgeXc(bus, line, i, j);
                if (bus->antiSat2Flag[i] == 0) {
                    //busWriteLogWithPhase(bus, "%c段母线小差谐波制动闭锁", i-1);
                    break;
                }
            }
            if (j < 3) {
                busWriteLogWithPhase(bus, "%c段母线小差谐波制动开放保护动作", i-1);
            }
        }
        
    }

}

void antiSat2JudgeXc(BusDevice* bus, Device line[], int busNum, int phase) {

    double k2 = 0.4, k3 = 0.4;
    double IdiffAmp[3];
    Phasor Idiff[3];
    double* inst;
    int i, j, k;
    int mode[29][4];
    // 初始化
    for (i = 0; i < 3; i++) {
        Idiff[i].real = 0.0;
        Idiff[i].img = 0.0;
        IdiffAmp[i] = 0.0;
    }

    int busBrkExitCalculateEnable[4];
    for (i = 0; i < 4; i++) {
        busBrkExitCalculateEnable[i] = bus->busBrkDeadZoneFlag[i];
    } 

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            mode[i][j] = bus->busModeStatus[i][j];
        }
    }

    for (i = 0; i < 24; i++) {
        if (mode[i][busNum-1] == 1) {
            switch (phase) {
                case 0: inst = line[i].instIma; break;
                case 1: inst = line[i].instImb; break;
                case 2: inst = line[i].instImc; break;
            }
            inst += (bus->busDiffDelayFlag && line[i].busSynFlag);
            for (k = 0; k < 3; k++) {
                Idiff[k] = phasorAdd(Idiff[k], phasorForHarmonic(inst, 0, k+1));
            }
        } else if (mode[i][busNum-1] == -1) {
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

    for (i = 0; i < 4; i++) {
        if (mode[i+24][busNum-1] == 1 && busBrkExitCalculateEnable[i] == 0) {
            inst = busInstSearch(bus, phase, i);
            inst += bus->busDiffDelayFlag;
            for (k = 0; k < 3; k++) {
                Idiff[k] = phasorAdd(Idiff[k], phasorForHarmonic(inst, 0, k+1));
            }
        } else if (mode[i+24][busNum-1] == -1 && busBrkExitCalculateEnable[i] == 0) {
            inst = busInstSearch(bus, phase, i);
            inst += bus->busDiffDelayFlag;
            for (k = 0; k < 3; k++) {
                Idiff[k] = phasorSub(Idiff[k], phasorForHarmonic(inst, 0, k+1));
            }
        }
    }

    if (busNum % 2 == 0 && bus->bypassBusMode == busNum/2+1) {
        inst = busInstSearch(bus, phase, 0);
        inst += bus->busDiffDelayFlag;
        for (k = 0; k < 3; k++) {
            Idiff[k] = phasorAdd(Idiff[k], phasorNumMulti(mode[28][busNum] ,phasorForHarmonic(inst, 0, k+1)));
        }
    }

    for (i = 0; i < 3; i++) {
        IdiffAmp[i] = phasorAbs(Idiff[i]);
    }
    /*if (busNum == 1 && phase == 1) {
        bus->testDC = IdiffAmp[1];
        bus->testPhasor[0] = IdiffAmp[2];
        bus->testPhasor[1] = IdiffAmp[0];
    }*/
    //double tmp = bus->testDC;
    //bus->testDC = tmp > IdiffAmp[1] ? tmp:IdiffAmp[1];
    //tmp = bus->testPhasor[0];
    //bus->testPhasor[0] = tmp > IdiffAmp[2] ? tmp:IdiffAmp[2];
    // { 
    //     if (phase == 1 && busNum == 1) {
    //         bus->testDC = IdiffAmp[0];
    //         bus->testPhasor[0] = IdiffAmp[1];
    //     }

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

void antiSat2JudgeDc(BusDevice* bus, Device line[], int phase) {

    double k2 = 0.4, k3 = 0.4;
    double IdiffAmp[3];
    Phasor Idiff[3];
    double* inst;
    int i, j, k;
    int mode[29][4];
    // 初始化
    for (i = 0; i < 3; i++) {
        Idiff[i].real = 0.0;
        Idiff[i].img = 0.0;
        IdiffAmp[i] = 0.0;
    }

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            mode[i][j] = bus->busModeStatus[i][j];
        }
    }

    for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            if (mode[i][j] == 1) {
                switch (phase) {
                    case 0: inst = line[i].instIma; break;
                    case 1: inst = line[i].instImb; break;
                    case 2: inst = line[i].instImc; break;
                }
                inst += (bus->busDiffDelayFlag && line[i].busSynFlag);
                for (k = 0; k < 3; k++) {
                    Idiff[k] = phasorAdd(Idiff[k], phasorForHarmonic(inst, 0, k+1));
                }
            } else if (mode[i][j] == -1) {
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
    }

    if (bus->bypassBusMode != 0) {
        for (i = 0; i < 4; i++) {
            if (mode[28][i] != 0) {
                inst = busInstSearch(bus, phase, 0);
                inst += bus->busDiffDelayFlag;
                for (k = 0; k < 3; k++) {
                    Idiff[k] = phasorAdd(Idiff[k], phasorNumMulti(mode[28][i] ,phasorForHarmonic(inst, 0, k+1)));
                }
                break;
            }
        }
    }

    for (i = 0; i < 3; i++) {
        IdiffAmp[i] = phasorAbs(Idiff[i]);
    }

    if (IdiffAmp[0] < 0.1) { // 差流要到达一定水平才进行判断
        return;
    }

    

    if ((IdiffAmp[1] >= IdiffAmp[0]*k2) || (IdiffAmp[2] >= IdiffAmp[0]*k3)) {
        bus->antiSat2Flag[0] = 1;
    } else {
        bus->antiSat2Flag[0] = 0;
    }
}


