#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

/**
 * 零序电流继电器
 * 由line函数按相调用，对于相间故障phase=0，代表AB相间，以此类推
 */

void zeroSequenceStart(Device* device);

void zeroSeqCurrentRelay(Device* device, int phase) {

    zeroSequenceStart(device);

    double startTime = device->startTime;
    double time = device->time;

    int zeroDirect = device->directionZeroSequenceIIIEnable;//III段方向投入
    int zeroStart = device->startFlag;
    //整定值
    double t1set = device->zeroSequenceTimeSetValue[0];//II段时延
    double t2set = device->zeroSequenceTimeSetValue[1];//III段时延

    //double I0set1 = device->zeroSequenceEnable[0];
    //double I0set2 = device->zeroSequenceEnable[1];
    double I0set1 = device->zeroSequenceSetValue[0]; // II
    double I0set2 = device->zeroSequenceSetValue[1]; // III
    double I0set3 = device->zeroSequenceSetValue[2]; // 加速段
    double I0set4 = device->ptBreakOverCurrentSetValue; // PT断线相过流
    double I0set5 = device->ptBreakZeroSeqSetValue; // PT断线零序过流

    double time_pt = device->ptBreakOcTimeSetValue; // PT断线时间定值

    Phasor Uma, Umb, Umc, Una, Unb, Unc;
    Phasor Ima, Imb, Imc, Ina, Inb, Inc;
    Phasor Um0, Im0, Un0, In0 ;
    double phasem0, phasen0;

    int blockII = 0, blockIII = 0, i;
    for (i = 0; i < 3; i++) {
        if (device->openPhaseFlag[i] == 1) {
            blockII = 1;
            break;
        }
    }
    if (device->PTBreakFlag == 1) {
        zeroDirect = 0;
        blockII = 1;
    }
    if (device->CTBreakFlag == 1) {
        blockIII = 1;
    }

    

    // 参数本地化
    Uma = device->phasor[0];
    Umb = device->phasor[1];
    Umc = device->phasor[2];
    Ima = device->phasor[3];
    Imb = device->phasor[4];
    Imc = device->phasor[5];
    Una = device->phasor[6];
    Unb = device->phasor[7];
    Unc = device->phasor[8];
    Ina = device->phasor[9];
    Inb = device->phasor[10];
    Inc = device->phasor[11];

    Um0 = phasorSeq(Uma, Umb, Umc, 0);
    Im0 = phasorSeq(Ima, Imb, Imc, 0);
    Un0 = phasorSeq(Una, Unb, Unc, 0);
    In0 = phasorSeq(Ina, Inb, Inc, 0);

    phasem0 = phasorAngleDiff(Um0, Im0);
    phasen0 = phasorAngleDiff(Un0, In0);
    /*if (phasem0 < 0.0001){
        phasem0 = phasem0 + 360;
    }
    if (phasen0 < 0.0001){
        phasen0 = phasen0 + 360;
    }*/

    // II段
    if ((time-startTime) > t1set && phasem0 < 344 && phasem0 >164 && (3.0*phasorAbs(Im0)) > I0set1 && blockII == 0 && zeroStart == 1 && device->CTBreakFlag == 0){
        device->zeroSequenceTripFlag[0] = 1;
        writeLog(device, "线路零序过电流保护投入II段动作");
    }
    /*if ((time-startTime) > t1set && phasen0 < 344 && phasen0 >164 && phasorAbs(In0) > I0set1 && blockII == 0){
        device->zeroSequenceTripFlag[0] = 1;
        writeLog(device, "线路零序过电流保护投入II段动作");
    }*/

    // III段
    if ((time-startTime) > t2set && (phasem0 >= 344 || phasem0 <= 164 || zeroDirect == 0) && (3.0*phasorAbs(Im0)) > I0set2 && blockIII == 0 && zeroStart == 1&& device->CTBreakFlag == 0){
        device->zeroSequenceTripFlag[1] = 1;
        writeLog(device, "线路零序过电流保护投入III段动作");
    }
    /*if ((time-startTime) > t2set && (phasen0 >= 290 || phasen0 <= 110 || zeroDirect == 0) && phasorAbs(In0) > I0set2 && blockIII == 0){
        device->zeroSequenceTripFlag[1] = 1;
        writeLog(device, "线路零序过电流保护投入III段动作");
    }*/

    //加速.需要重合闸信号接入
    int CHZsignalm[3], CHZsignaln[3];//重合闸信号；
    CHZsignalm[0] = 0;
    CHZsignalm[1] = 0;
    CHZsignalm[2] = 0;
    CHZsignaln[0] = 0;
    CHZsignaln[1] = 0;
    CHZsignaln[2] = 0;

    if (CHZsignalm[0]+CHZsignalm[1]+CHZsignalm[2] == 1 && (time-startTime) > 0.06 && (3.0*phasorAbs(Im0)) > I0set3 && zeroStart == 1){
        device->zeroSequenceTripFlag[2] = 1;
        writeLog(device, "线路零序加速保护动作");
    }
    /*if (CHZsignaln[0]+CHZsignaln[1]+CHZsignaln[2] == 1 && (time-startTime) > 0.06 && phasorAbs(In0) > I0set3){
        device->zeroSequenceTripFlag[2] = 1;
        writeLog(device, "线路零序加速保护动作");
    }*/

    if (CHZsignalm[0]+CHZsignalm[1]+CHZsignalm[2] == 3 && (time-startTime) > 0.1 && (3.0*phasorAbs(Im0)) > I0set3 && zeroStart == 1){
        device->zeroSequenceTripFlag[2] = 1;
        writeLog(device, "线路零序加速保护动作");
    }
   /* if (CHZsignaln[0]+CHZsignaln[1]+CHZsignaln[2] == 3 && (time-startTime) > 0.1 && phasorAbs(Im0) > I0set3){
        device->zeroSequenceTripFlag[2] = 1;
        writeLog(device, "线路零序加速保护动作");
    }*/


}

void zeroSequenceStart(Device* device){

    Phasor I0, temp;
    double abs;

    temp = phasorAdd(device->phasor[3], device->phasor[4]);
    I0 = phasorAdd(temp, device->phasor[5]);
    abs = phasorAbs(I0); //使用3I0

    if (abs > device->lineStartSetValue[1] && phasorAbs(device->zeroSeqCurrentOUT) > device->lineStartSetValue[1]){
        device->zeroSeqStartFlag = 1;
    } else {
        device->zeroSeqStartFlag = 0;
    }
}

