#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>

void zeroCurrh(tranDevice* trandevice){
    Phasor Ua, Ub, Uc;
    Phasor Ia, Ib, Ic;
    Phasor U0, I0;
    double angle;
    double Iset1, Iset2, Iset3;
    double time1, time2, time3;
    double time;
    double startTime;
    time = trandevice->time;
    startTime = trandevice->backTime[0];

    Iset1 = trandevice->zeroCur1[0];
    Iset2 = trandevice->zeroCur1[1];
    Iset3 = trandevice->zeroCur1[2];
    time1 = trandevice->zeroTime1[0];
    time2 = trandevice->zeroTime1[1];
    time3 = trandevice->zeroTime1[2];

    Ua = trandevice->phasor[0];
    Ub = trandevice->phasor[1];
    Uc = trandevice->phasor[2];

    Ia = trandevice->phasor[3];
    Ib = trandevice->phasor[4];
    Ic = trandevice->phasor[5];

    U0 = phasorSeq(Ua, Ub, Uc, 0);
    I0 = phasorSeq(Ia, Ib, Ic, 0);
    angle = phasorAngleDiff(U0, I0);
    int direct;

    if(angle> 90  && angle <270){
        direct = 0; //指向系统
    }
    else{
        direct = 1; //指向变压器
    }
    int directFlag;
    if(((direct == 0) && (trandevice->backEnable1[5] > 0)) || (trandevice->backEnable1[4] < 1)){
        directFlag = 1;
    }
    if(((direct == 1) && (trandevice->backEnable1[5] < 1)) || (trandevice->backEnable1[7] < 1)){
        directFlag = 1;
    }


    //I
    if((phasorAbs(I0) > Iset1) && ((time - startTime) > (0.02 + time1)) && (directFlag == 1)){

        trandevice->tranProRes[0] = 1;

        tranWriteLog(trandevice, "高零序过流I段动作");
    }
    if((phasorAbs(I0) > Iset2) && ((time - startTime) > (0.02 + time2)) && (directFlag == 1)){

        trandevice->tranProRes[0] = 1;

        tranWriteLog(trandevice, "高零序过流II段动作");
    }
    if((phasorAbs(I0) > Iset3) && ((time - startTime) > (0.02 + time3))){

        trandevice->tranProRes[0] = 1;

        tranWriteLog(trandevice, "高零序过流III段动作");
    }




}

void zeroCurrv(tranDevice* trandevice){
    Phasor Ua, Ub, Uc;
    Phasor Ia, Ib, Ic;
    Phasor U0, I0;
    double angle;
    double Iset1, Iset2, Iset3;
    double time1, time2, time3;
    double startTime;
    double time;
    time = trandevice->time;
    startTime = trandevice->backTime[1];


    Iset1 = trandevice->zeroCur2[0];
    Iset2 = trandevice->zeroCur2[1];
    Iset3 = trandevice->zeroCur2[2];
    time1 = trandevice->zeroTime2[0];
    time2 = trandevice->zeroTime2[1];
    time3 = trandevice->zeroTime2[2];

    Ua = trandevice->phasor[6];
    Ub = trandevice->phasor[7];
    Uc = trandevice->phasor[8];

    Ia = trandevice->phasor[9];
    Ib = trandevice->phasor[10];
    Ic = trandevice->phasor[11];

    U0 = phasorSeq(Ua, Ub, Uc, 0);
    I0 = phasorSeq(Ia, Ib, Ic, 0);
    angle = phasorAngleDiff(U0, I0);
    int direct;

    if(angle> 90  && angle <270){
        direct = 0; //指向系统
    }
    else{
        direct = 1; //指向变压器
    }
    int directFlag;
    if(((direct == 0) && (trandevice->backEnable2[5] > 0)) || (trandevice->backEnable2[4] < 1)){
        directFlag = 1;
    }
    if(((direct == 1) && (trandevice->backEnable2[5] < 1)) || (trandevice->backEnable2[7] < 1)){
        directFlag = 1;
    }


    //I
    if((phasorAbs(I0) > Iset1) && ((time - startTime) > (0.02 + time1)) && (directFlag == 1)){

        trandevice->tranProRes[1] = 1;

        tranWriteLog(trandevice, "中零序过流I段动作");
    }
    if((phasorAbs(I0) > Iset2) && ((time - startTime) > (0.02 + time2)) && (directFlag == 1)){

        trandevice->tranProRes[1] = 1;

        tranWriteLog(trandevice, "中零序过流II段动作");
    }
    if((phasorAbs(I0) > Iset3) && ((time - startTime) > (0.02 + time3))){

        trandevice->tranProRes[1] = 1;

        tranWriteLog(trandevice, "中零序过流III段动作");
    }

}

void zeroCurrt(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    Phasor I0;
    double angle;
    double Iset1;
    double time1;
    double startTime;
    double time;
    time = trandevice->time;
    startTime = trandevice->backTime[2];


    Iset1 = trandevice->zeroCur2[3];
    time1 = trandevice->zeroTime2[3];

    Ia = trandevice->phasor[21];
    Ib = trandevice->phasor[22];
    Ic = trandevice->phasor[23];


    I0 = phasorSeq(Ia, Ib, Ic, 0);

    if((phasorAbs(I0) > Iset1) && ((time - startTime) > (0.02 + time1))){

        trandevice->tranProRes[1] = 1;
        trandevice->tranProRes[2] = 1;
        trandevice->tranProRes[3] = 1;

        tranWriteLog(trandevice, "公共绕组零序过流动作");
    }


}

void compCurrh(tranDevice* trandevice){
    Phasor Ua, Ub, Uc;
    Phasor Ia, Ib, Ic;
    Phasor U2;



    double Iset, Uset1, Uset2;
    double time1;
    double time;
    double startTime;
    time = trandevice->time;
    startTime = trandevice->backTime[3];

    Uset1 = trandevice->compSet1[0];
    Uset2 = trandevice->compSet1[1];
    Iset = trandevice->compSet1[2];
    time1 = trandevice->compSet1[3];


}