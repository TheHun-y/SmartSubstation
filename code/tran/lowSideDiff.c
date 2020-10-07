#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>
//
void lowDiffRelay(tranDevice* trandevice){

    double Ixcd = trandevice->tranStartSetValue[5];
    double Ie = 1.414*(trandevice->ratedI3)*1.732;
    double Ifcd = trandevice->tranStartSetValue[2];
    int flag;
    double time, startTime;
    time = trandevice->time;
    startTime = trandevice->startTime[3];

    Phasor Iaw, Ibw, Icw;
    Phasor Ial, Ibl, Icl;
    Phasor Iad, Ibd, Icd;
    double Ira, Irb, Irc;
    double Idmax;

    Ial = trandevice->phasor[15];
    Ibl = trandevice->phasor[16];
    Icl = trandevice->phasor[17];

    Iaw = trandevice->phasor[18];
    Ibw = trandevice->phasor[19];
    Icw = trandevice->phasor[20];

    Iad = phasorAdd(Ial, Iaw);
    Iad = phasorSub(Iad, Icw);
    Ibd = phasorAdd(Ibl, Ibw);
    Ibd = phasorSub(Ibd, Iaw);
    Icd = phasorAdd(Icl, Icw);
    Icd = phasorSub(Icd, Ibw);

    Idmax = phasorAbs(Iad);
    if(Idmax<phasorAbs(Ibd)){
        Idmax = phasorAbs(Ibd);
    }
    if(Idmax<phasorAbs(Icd)){
        Idmax = phasorAbs(Icd);
    }

    if(phasorAbs(Ial)>phasorAbs(Iaw)){
        Ira = phasorAbs(Ial);
    }
    else{
        Ira = phasorAbs(Iaw);
    }
    if(phasorAbs(Ibl)>phasorAbs(Ibw)){
        Irb = phasorAbs(Ibl);
    }
    else{
        Irb = phasorAbs(Ibw);
    }
    if(phasorAbs(Icl)>phasorAbs(Icw)){
        Irc = phasorAbs(Icl);
    }
    else{
        Irc = phasorAbs(Icw);
    }

    if(phasorAbs(Iad)> Ixcd && Ira<=0.5*Ie){
        flag = 1;
    }
    if(phasorAbs(Iad)> 0.5*(Ira-0.5*Ie)+Ifcd && Ira>0.5*Ie){
        flag = 1;
    }

    if(phasorAbs(Ibd)> Ixcd && Irb<=0.5*Ie){
        flag = 1;
    }
    if(phasorAbs(Ibd)> 0.5*(Irb-0.5*Ie)+Ifcd && Irb>0.5*Ie){
        flag = 1;
    }
    if(phasorAbs(Icd)> Ixcd && Irc<=0.5*Ie){
        flag = 1;
    }
    if(phasorAbs(Icd)> 0.5*(Irc-0.5*Ie)+Ifcd && Irc>0.5*Ie){
        flag = 1;
    }

    if(flag ==1 && time - startTime > 0.02){
        trandevice->mainFlag = 1;
        int i = 0;
        for(i=0; i<3 ; i++){
            trandevice->tranProRes[i] = 1;
        }

        tranWriteLog(trandevice, "低压侧小区差动保护动作");
    }


}
