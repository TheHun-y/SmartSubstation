#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>


void deltaDiffRelay(tranDevice* trandevice) {
    double Ie = 1.414*1.732*(trandevice->ratedI1);
    double Idtset = trandevice->tranStartSetValue[2];
    int flag = 0;

    double startTime = trandevice->startTime[1];
    double time = trandevice->time;

    double Ida, Idb, Idc, Idt;
    Ida = trandevice->Ida_base;
    Idb = trandevice->Idb_base;
    Idc = trandevice->Idc_base;
    Idt = trandevice->Idt;

    double voltageH = trandevice->ratedV1;
    double voltageV = trandevice->ratedV2;
    double voltageL = trandevice->ratedV3;

    double Ira, Irb, Irc;
    double Irmax;

    Ira = trandevice->Ira_base;
    Irb = trandevice->Irb_base;
    Irc = trandevice->Irc_base;


    Irmax = Ira;
    if(Irmax < Irb){
        Irmax = Irb;
    }
    if(Irmax < Irc){
        Irmax = Irc;
    }

    int flaga, flagb, flagc;
    flaga = 0;
    flagb = 0;
    flagc = 0;

    Phasor Ia_1, Ib_1, Ic_1;
    Phasor Ia_2, Ib_2, Ic_2;
    Phasor Ia_3, Ib_3, Ic_3;

    Ia_1 = trandevice->basePhasor[3];
    Ia_2 = trandevice->secPhasor[3];
    Ia_3 = trandevice->thrPhasor[3];
    Ib_1 = trandevice->basePhasor[4];
    Ib_2 = trandevice->secPhasor[4];
    Ib_3 = trandevice->thrPhasor[4];
    Ic_1 = trandevice->basePhasor[5];
    Ic_2 = trandevice->secPhasor[5];
    Ic_3 = trandevice->thrPhasor[5];

    if(phasorAbs(Ia_2) > 0.15*phasorAbs(Ia_1) && phasorAbs(Ia_3) > 0.2*(phasorAbs(Ia_1) && (time-startTime)>0.02)){
        flaga = 1;
    }
    if(phasorAbs(Ib_2) > 0.15*phasorAbs(Ib_1) && phasorAbs(Ib_3) > 0.2*(phasorAbs(Ib_1) && (time-startTime)>0.02)){
        flagb = 1;
    }
    if(phasorAbs(Ic_2) > 0.15*phasorAbs(Ic_1) && phasorAbs(Ic_3) > 0.2*(phasorAbs(Ic_1) && (time-startTime)>0.02)){
        flagc = 1;
    }



    if(Ida > (1.25*Idtset) && Ida > (0.6*Irmax) && Irmax < 2*Ie && flaga == 0){
        flag = 1;
    }
    if(Ida > (1.25*Idtset) && Ida > (0.75*Irmax - 0.3*Ie ) && Irmax > 2*Ie && flaga == 0){
        flag = 1;
    }

    if(Idb > (1.25*Idtset) && Idb > (0.6*Irmax) && Irmax < 2*Ie && flagb == 0 ){
        flag = 1;
    }
    if(Idb > (1.25*Idtset) && Idb > (0.75*Irmax - 0.3*Ie ) && Irmax > 2*Ie && flagb == 0){
        flag = 1;
    }

    if(Idc > (1.25*Idtset) && Idc > (0.6*Irmax) && Irmax < 2*Ie && flagc == 0){
        flag = 1;
    }
    if(Idc > (1.25*Idtset) && Idc > (0.75*Irmax - 0.3*Ie ) && Irmax > 2*Ie && flagc == 0){
        flag = 1;
    }

    int i = 0;
    if(flag == 1 && (time-startTime > 0.02 && time-startTime < 0.52) ){
        trandevice->mainFlag = 1;
        for(i=0; i<3 ; i++){
            trandevice->tranProRes[i] = 1;
        }
        tranWriteLog(trandevice, "工频变化量比率差动保护动作");
    }


}

