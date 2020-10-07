#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>

void zeroDiffRelay(tranDevice* trandevice){
    double voltageV = trandevice->ratedV2;
    double voltageH = trandevice->ratedV1;
    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Iaw, Ibw, Icw;
    Phasor Ih0, Iv0, Iw0;
    Phasor I0dtemp;
    double I0d, I0r;
    double I0cdqd = trandevice->tranStartSetValue[4];
    double Ie = 1.414*(trandevice->ratedI2);
    double startTime, time;
    int flag = 0;


    startTime = trandevice->startTime[2];
    time = trandevice->time;

    Iah = trandevice->phasor[3];
    Ibh = trandevice->phasor[4];
    Ich = trandevice->phasor[5];

    Iav = trandevice->phasor[9];
    Ibv = trandevice->phasor[10];
    Icv = trandevice->phasor[11];

    Iaw = trandevice->phasor[21];
    Ibw = trandevice->phasor[22];
    Icw = trandevice->phasor[23];

    Ih0 = phasorAdd(Iah, Ibh);
    Ih0 = phasorAdd(Ih0, Ich);
    Iv0 = phasorAdd(Iav, Ibv);
    Iv0 = phasorAdd(Iv0, Icv);
    Iw0 = phasorAdd(Iaw, Ibw);
    Iw0 = phasorAdd(Iw0, Icw);

    I0dtemp = phasorAdd(Ih0, Iv0);
    I0dtemp = phasorAdd(I0dtemp, Iw0);
    I0d = phasorAbs(I0dtemp);

    I0r = phasorAbs(Ih0);
    if(I0r < phasorAbs(Iv0)){
        I0r = phasorAbs(Iv0);
    }
    if(I0r < phasorAbs(Iw0)){
        I0r = phasorAbs(Iw0);
    }

    if((I0d > 0.5 * I0r - 0.25 * Ie + I0cdqd) && (I0r > 0.5 * Ie) && ((time - startTime) > 0.02)){
        flag = 1;
    }

    if((I0d > I0cdqd) && (I0r < 0.5*Ie) && ((time - startTime) > 0.02)){
        flag = 1;
    }

    if(flag == 1){
        trandevice->mainFlag = 1;

        int i = 0;
        for(i=0; i<3 ; i++){
            trandevice->tranProRes[i] = 1;
        }
        tranWriteLog(trandevice, "零序比率差动动作");
    }

}
