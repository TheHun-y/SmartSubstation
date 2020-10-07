#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>


void splitDiffRelay(tranDevice* trandevice){
    double Ifcdqd = trandevice->tranStartSetValue[2];
    double Ie = trandevice->ratedI2 * 1.414;
    double time, startTime;

    time = trandevice->time;
    startTime = trandevice->startTime[3];

    int flag = 0;

    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Iaw, Ibw, Icw;
    Phasor Ida, Idb, Idc;

    double Ifda, Ifdb, Ifdc;
    double Iramax, Irbmax, Ircmax;


    Iah = trandevice->phasor[3];
    Ibh = trandevice->phasor[4];
    Ich = trandevice->phasor[5];

    Iav = trandevice->phasor[9];
    Ibv = trandevice->phasor[10];
    Icv = trandevice->phasor[11];

    Iaw = trandevice->phasor[21];
    Ibw = trandevice->phasor[22];
    Icw = trandevice->phasor[23];

    Ida = phasorAdd(Iah, Iav);
    Ida = phasorAdd(Ida, Iaw);
    Idb = phasorAdd(Ibh, Ibv);
    Idb = phasorAdd(Idb, Ibw);
    Idc = phasorAdd(Ich, Icv);
    Idc = phasorAdd(Idc, Icw);

    Ifda = phasorAbs(Ida);
    Ifdb = phasorAbs(Idb);
    Ifdc = phasorAbs(Idc);

    Iramax = phasorAbs(Iah);
    if(Iramax < phasorAbs(Iav)){
        Iramax = phasorAbs(Iav);
    }
    if(Iramax < phasorAbs(Iaw)){
        Iramax = phasorAbs(Iaw);
    }
    Irbmax = phasorAbs(Ibh);
    if(Irbmax < phasorAbs(Ibv)){
        Irbmax = phasorAbs(Ibv);
    }
    if(Irbmax < phasorAbs(Ibw)){
        Irbmax = phasorAbs(Ibw);
    }
    Ircmax = phasorAbs(Ich);
    if(Ircmax < phasorAbs(Icv)){
        Ircmax = phasorAbs(Icv);
    }
    if(Ircmax < phasorAbs(Icw)){
        Ircmax = phasorAbs(Icw);
    }

    if(Ifda> 0.5*Iramax -0.25*Ie +Ifcdqd ){
        flag =1;
    }
    if(Ifdb> 0.5*Irbmax -0.25*Ie +Ifcdqd ){
        flag =1;
    }
    if(Ifdc> 0.5*Ircmax -0.25*Ie +Ifcdqd ){
        flag =1;
    }


    if(flag == 1 && time - startTime >0.02){
        trandevice->mainFlag = 1;
        int i = 0;
        for(i=0; i<3 ; i++){
            trandevice->tranProRes[i] = 1;
        }

        tranWriteLog(trandevice, "分侧差动动作");

    }

}

