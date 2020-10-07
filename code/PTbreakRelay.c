#include "..\\code\\dataStruct.h"
#include "..\\code\common.h"

/**
 * 
 * PT断线判别及保护
 * 
 **/

void PTbreakRecognize(Device* device) {

    Phasor Ua, Ub, Uc, I[3];
    double Usum, U1, Un, In, IsetP, Iset0, tSet;
    int start, PTtag, currentTag, brkTag[3], TWJtag, i, judgeTag = 0, allTrip;

    Ua = device->phasor[0];
    Ub = device->phasor[1];
    Uc = device->phasor[2];
    I[0] = device->phasor[3];
    I[1] = device->phasor[4];
    I[2] = device->phasor[5];

    Usum = phasorAbs(phasorAdd(Ua, phasorAdd(Ub, Uc)));
    U1 = phasorAbs(phasorSeq(Ua, Ub, Uc, 1));
    Un = device->ratedSideIVoltage*0.4; // 差分滤波的系数
    In = device->ratedSideICurrent;

    IsetP = device->ptBreakOverCurrentSetValue;
    Iset0 = device->ptBreakZeroSeqSetValue;

    tSet = device->ptBreakOcTimeSetValue;

    start = device->startFlag;
    allTrip = (device->tripFlag[0] && device->tripFlag[1] && device->tripFlag[2]);
    PTtag = device->voltFromLinePTEnable; // 0-母线PT；1-线路PT

    for (i = 0; i < 3; i++) {
        if (phasorAbs(I[i]) > 0.02*In) {
            currentTag = 1;
        }
        brkTag[i] = device->sampleBrkStatus[i];
        if (brkTag[i] == 0) {
            TWJtag = 1;
        }
    }

    if (device->PTBreakFlag == 0) {
        if (Usum > 0.08*Un && start == 0 && allTrip == 0) {
            device->PTBreakPreFlag = 1;
            judgeTag = 1;
            if (device->PTBreakTime == 0){
                device->PTBreakTime = device->time;
            }
            if (device->PTBreakTime != 0 && device->time - device->PTBreakTime >= device->PTBreakJudgeTimeSetValue[0])  {
                device->PTBreakFlag = 1;
                device->PTBreakPreFlag = 0;
                device->PTBreakTime = 0;
                if (device->PTBreakOcTime == 0) {
                    device->PTBreakOcTime = device->time;
                }
                writeLog(device, "PT断线异常告警-TYPE1！");
            }   
        }

        if (Usum <= 0.08*Un && U1 < Un/3.0 && allTrip == 0) {
            if (PTtag == 0) {
                device->PTBreakPreFlag = 1;
                judgeTag = 1;
                if (device->PTBreakTime == 0) {
                    device->PTBreakTime = device->time;
                } 
            } else {
                if (currentTag == 1 || TWJtag == 1) {
                    device->PTBreakPreFlag = 1;
                    judgeTag = 1;
                    if (device->PTBreakTime == 0) {
                        device->PTBreakTime = device->time;
                    }
                }
            }

            if (device->PTBreakTime != 0 && device->time - device->PTBreakTime >= device->PTBreakJudgeTimeSetValue[0])  {
                device->PTBreakFlag = 1;
                device->PTBreakPreFlag = 0;
                device->PTBreakTime = 0;
                if (device->PTBreakOcTime == 0) {
                    device->PTBreakOcTime = device->time;
                }
                writeLog(device, "PT断线异常告警-TYPE2！");
            }


        }

    }

    if (device->PTBreakFlag == 1) {
        // 相过流
        if (phasorAbs(I[0]) > IsetP || phasorAbs(I[1]) > IsetP || phasorAbs(I[2]) > IsetP && (device->time - device->PTBreakOcTime >= tSet)) {
            device->PTBreakTripFlag[0] = 1;
            device->PTBreakOcTime = 0;
            writeLog(device, "PT断线相过流元件动作");
        }
        // 零序过流
        if (phasorAbs(phasorSeq(I[0], I[1], I[2], 0)) > Iset0 && (device->time - device->PTBreakOcTime >= tSet)) {
            device->PTBreakTripFlag[1] = 1;
            device->PTBreakOcTime = 0;
            writeLog(device, "PT断线零序过流元件动作");
        }

        if (judgeTag == 0) {
            if (device->PTBreakReturnTime == 0) {
                device->PTBreakReturnTime = device->time;
            }
            if (device->PTBreakReturnTime != 0 && device->time - device->PTBreakReturnTime >= device->PTBreakJudgeTimeSetValue[1]) {
                device->PTBreakReturnTime = 0;
                device->PTBreakFlag = 0;
                writeLog(device, "PT断线信号复归");
            }     
        } else {
            device->PTBreakReturnTime = 0;
        }
    }

    if (judgeTag == 0) {
        device->PTBreakPreFlag = 0;
        device->PTBreakTime = 0;
    }
    
}
