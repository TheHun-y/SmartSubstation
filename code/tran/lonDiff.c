#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>

void diffTrip(tranDevice* trandevice){
    double time = trandevice->time;
    double startTime = trandevice->startTime[0];

    double Ie = trandevice->tranStartSetValue[1];

    double voltageH = trandevice->ratedV1;
    double voltageV = trandevice->ratedV2;
    double voltageL = trandevice->ratedV3;

    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Ial, Ibl, Icl;
    Phasor Ida, Idb, Idc;
    double Idmax;


    Iah = phasorSub(trandevice->halfphasor[3],trandevice->halfphasor[5]);
    Ibh = phasorSub(trandevice->halfphasor[4],trandevice->halfphasor[3]);
    Ich = phasorSub(trandevice->halfphasor[5],trandevice->halfphasor[4]);

    Iav = phasorSub(trandevice->halfphasor[9],trandevice->halfphasor[11]);
    Ibv = phasorSub(trandevice->halfphasor[10],trandevice->halfphasor[9]);
    Icv = phasorSub(trandevice->halfphasor[11],trandevice->halfphasor[10]);

    Iav = phasorNumMulti((voltageV / voltageH), Iav);
    Ibv = phasorNumMulti((voltageV / voltageH), Ibv);
    Icv = phasorNumMulti((voltageV / voltageH), Icv);

    Ial = phasorNumMulti((voltageL / voltageH), trandevice->halfphasor[15]);
    Ibl = phasorNumMulti((voltageL / voltageH), trandevice->halfphasor[16]);
    Icl = phasorNumMulti((voltageL / voltageH), trandevice->halfphasor[17]);

    Ida = phasorAdd(Iah, Iav);
    Ida = phasorAdd(Ida, Ial);
    Idb = phasorAdd(Ibh, Ibv);
    Idb = phasorAdd(Idb, Ibl);
    Idc = phasorAdd(Ich, Icv);
    Idc = phasorAdd(Idc, Icl);

    Idmax = phasorAbs(Ida);
    if (phasorAbs(Idb) > Idmax) {
        Idmax = phasorAbs(Idb);
    }
    if (phasorAbs(Idc) > Idmax) {
        Idmax = phasorAbs(Idc);
    }
    trandevice->tranProRes[11] = Idmax;


    if(Idmax > Ie && time - startTime > 0.01 ){
        trandevice->mainFlag = 1;
        int i = 0;
        for(i=0; i<9 ; i++){
            trandevice->tranProRes[i] = 1;
        }
        tranWriteLog(trandevice, "差动速断保护动作");
    }
}


void lonDiffRelay(tranDevice* trandevice){
    double Ie = 1.414*1.732*(trandevice->ratedI1);
    double Icdqd = trandevice->tranStartSetValue[0];

    int flag1 = 0;
    int flag2 = 0;
    int flaga, flagb, flagc;
    flaga = 0;
    flagb = 0;
    flagc = 0;

    double startTime = trandevice->startTime[0];
    double time = trandevice->time;

    double Ida, Idb, Idc, Idt;
    Ida = trandevice->Idta;
    Idb = trandevice->Idtb;
    Idc = trandevice->Idtc;
    Idt = trandevice->Idt;

    double voltageH = trandevice->ratedV1;
    double voltageV = trandevice->ratedV2;
    double voltageL = trandevice->ratedV3;

    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Ial, Ibl, Icl;
    double Ira, Irb, Irc;

    Iah = phasorSub(trandevice->phasor[3],trandevice->phasor[5]);
    Ibh = phasorSub(trandevice->phasor[4],trandevice->phasor[3]);
    Ich = phasorSub(trandevice->phasor[5],trandevice->phasor[4]);

    Iav = phasorSub(trandevice->phasor[9],trandevice->phasor[11]);
    Ibv = phasorSub(trandevice->phasor[10],trandevice->phasor[9]);
    Icv = phasorSub(trandevice->phasor[11],trandevice->phasor[10]);

    Iav = phasorNumMulti((voltageV / voltageH), Iav);
    Ibv = phasorNumMulti((voltageV / voltageH), Ibv);
    Icv = phasorNumMulti((voltageV / voltageH), Icv);

    Ial = phasorNumMulti((voltageL / voltageH), trandevice->phasor[15]);
    Ibl = phasorNumMulti((voltageL / voltageH), trandevice->phasor[16]);
    Icl = phasorNumMulti((voltageL / voltageH), trandevice->phasor[17]);

    Ira = (phasorAbs(Iah) + phasorAbs(Iav) + phasorAbs(Ial)) * 0.5;
    Irb = (phasorAbs(Ibh) + phasorAbs(Ibv) + phasorAbs(Ibl)) * 0.5;
    Irc = (phasorAbs(Ich) + phasorAbs(Icv) + phasorAbs(Icl)) * 0.5;


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


    if(Ida > (0.2*Ira + Icdqd) && Ira <= (0.5*Ie) && flaga ==0){
        flag1 = 1;
    }
    else if(Ida >(0.5*Ira - 0.15 * Ie + Icdqd) && Ira > (0.5*Ie) && Ira<= (6*Ie) && flaga ==0){
        flag1 = 1;
    }
    else if (Ida > (0.75*Ira - 1.65*Ie + Icdqd ) && Ira > (6*Ie) && flaga ==0){
        flag1= 1;
    }

    if(Ida > 0.6*(Ira-0.8*Ie)+1.2*Ie && Ira>(0.8*Ie) && flaga ==0){
        flag2 = 1;
    }

    if(Idb > (0.2*Irb + Icdqd) && Irb <= (0.5*Ie) && flagb ==0){
        flag1 = 1;
    }
    else if(Idb >(0.5*Irb - 0.15 * Ie + Icdqd) && Irb > (0.5*Ie) && Irb<= (6*Ie) && flagb ==0){
        flag1 = 1;
    }
    else if (Idb > (0.75*Irb - 1.65*Ie + Icdqd ) && Irb > (6*Ie) && flagb ==0){
        flag1= 1;
    }

    if(Idb > 0.6*(Irb-0.8*Ie)+1.2*Ie && Irb>(0.8*Ie) && flagb ==0){
        flag2 = 1;
    }

    if(Idc > (0.2*Irc + Icdqd) && Irc <= (0.5*Ie) && flagc ==0){
        flag1 = 1;
    }
    else if(Idc >(0.5*Irc - 0.15 * Ie + Icdqd) && Irc > (0.5*Ie) && Irc<= (6*Ie) && flagc ==0){
        flag1 = 1;
    }
    else if (Idc > (0.75*Irc - 1.65*Ie + Icdqd ) && Irc > (6*Ie) && flagc ==0){
        flag1= 1;
    }

    if(Idc > 0.6*(Irc-0.8*Ie)+1.2*Ie && Irc>(0.8*Ie) && flagc ==0){
        flag2 = 1;
    }


    int i = 0;
    if((flag1 == 1 || flag2 ==1)  && ((time-startTime)>0.02) ){
        trandevice->mainFlag = 1;
        for(i=0; i<3 ; i++){
            trandevice->tranProRes[i] = 1;
        }

        tranWriteLog(trandevice, "纵差稳态比率差动保护动作");
    }



}
