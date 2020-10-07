#include "..\\code\\dataStruct.h"
#include "..\\code\common.h"

/**
 * 
 * CT断线判别及保护
 * 
 **/

void CTBreakRelay(Device* device) {

    Phasor I[3], I0, Ua, Ub, Uc, U0;
    double I0self, I0out, In, Un;
    int i, judgeTag[2] = {0, 0}, currentTag = 0;  

    Ua = device->phasor[0];
    Ub = device->phasor[1];
    Uc = device->phasor[2];
    I[0] = device->phasor[3];
    I[1] = device->phasor[4];
    I[2] = device->phasor[5];
    I0 = device->zeroSeqCurrentOUT;

    In = device->ratedSideICurrent;
    Un = device->ratedSideIVoltage;

    for (i = 0; i < 3; i++) {
        if (phasorAbs(I[i]) < 0.02*In) {
            currentTag = 1;
            break;
        }
    }

    I0out = phasorAbs(I0); //device->instZeroSeqI[0];
    I0self = 0.3*3.0*phasorAbs(phasorSeq(I[0], I[1], I[2], 0));

    /*char test[100];
    char test1[100];
    sprintf(test, "外接 = %lf", phasorAbs(I0));
    sprintf(test1, "自产 = %lf", I0self);
    if (device->time > 0.45 && device->time < 0.46) {
        writeLog(device, test);
        writeLog(device, test1);
    }*/

    U0 = phasorSeq(Ua, Ub, Uc, 0);

    if (device->CTBreakFlag == 0) {
        if (I0self < 0.75*I0out || I0out < 0.75*I0self) {
            device->CTBreakPreFlag[0] = 1;
            judgeTag[0] = 1;
            if (device->CTBreakTime[0] == 0) {
                device->CTBreakTime[0] = device->time;
            }
            if (device->CTBreakTime[0] != 0 && device->time - device->CTBreakTime[0] >= device->CTBreakJudgeTimeSetValue[0]) {
                device->CTBreakFlag = 1;
                device->CTBreakPreFlag[0] = 0;
                device->CTBreakPreFlag[1] = 0;
                device->CTBreakTime[0] = 0;
                device->CTBreakTime[1] = 0;
                writeLog(device, "CT断线异常告警！"); 
            }
        }

        if (I0self > 0.02*In && phasorAbs(U0) < 0.002*Un && currentTag == 1) {
            device->CTBreakPreFlag[1] = 1;
            judgeTag[1] = 1;
            if (device->CTBreakTime[1] == 0) {
                device->CTBreakTime[1] = device->time;
            }
            if (device->CTBreakTime[1] != 0 && device->time - device->CTBreakTime[1] >= device->CTBreakJudgeTimeSetValue[1]) {
                device->CTBreakFlag = 1;
                device->CTBreakPreFlag[0] = 0;
                device->CTBreakPreFlag[1] = 0;
                device->CTBreakTime[0] = 0;
                device->CTBreakTime[1] = 0;
                writeLog(device, "CT断线异常告警！"); 
            }
        }
        
    }
    
    for (i = 0; i < 2; i++) {
        if (judgeTag[i] == 0) {
            device->CTBreakPreFlag[i] = 0;
            device->CTBreakTime[i] = 0;
        }

    }
    

}
