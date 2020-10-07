#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>

/**
 * 电流差动继电器
 * 由line函数按相调用，对于相间故障phase=0，代表AB相间，以此类推
 */

double maxValue(double a, double b);

void currentDiffRelay(Device* device, int phase) {
    
    int capacityEnable = device->currentCompensationEnable;
    double startTime = device->startTime;
    double time = device->time;
    int zeroStart = device->zeroSeqStartFlag;

    double Un = device->ratedSideIVoltage;
    double Xc1 = device->lineC1;
    double IdiffSet = device->lineStartSetValue[2];
    double Ic = 0.5*Un/Xc1, Iseth, Isetm; // 实测电容电流Ic待处理
    Iseth = 1.5*Un/Xc1;
    Isetm = 1.25*Un/Xc1;
    double Ih, Im, Il;
    // 变化量、稳态I段
    if (capacityEnable == 1) {
        Ih = maxValue(4*Ic, 1.5*IdiffSet);
    } else {
        Ih = maxValue(maxValue(Iseth, 4*Ic), 1.5*IdiffSet);
    }
    // 零差
    Il = maxValue(IdiffSet, 1.25*Ic);
    // 稳态II段
    if (capacityEnable == 1) {
        Im = maxValue(1.5*Ic, IdiffSet);
    } else {
        Im = maxValue(maxValue(Isetm, 1.5*Ic), IdiffSet);
    }
    

    double t1set = device->currentDiffTimeSetValue[0];
    double t2set = device->currentDiffTimeSetValue[1];

    Phasor Ima_memory, Imb_memory, Imc_memory, Ina_memory, Inb_memory, Inc_memory, Im_memory0, In_memory0;
    Phasor Ima, Imb, Imc, Ina, Inb, Inc, Im0, In0;
    Phasor Ima_delta, Imb_delta, Imc_delta, Ina_delta, Inb_delta, Inc_delta, Im_delta0, In_delta0;
    double Iacd, Ibcd, Iccd, Iar, Ibr, Icr, Icd0, Ir0;
    double Iac, Ibc, Icc, Iarc, Ibrc, Icrc;
    double Iar_d, Ibr_d, Icr_d;

    // 参数本地化
    Ima = device->phasor[3];
    Imb = device->phasor[4];
    Imc = device->phasor[5];
    Ina = device->phasor[9];
    Inb = device->phasor[10];
    Inc = device->phasor[11];
    Im0 = phasorSeq(Ima, Imb, Imc, 0);
    In0 = phasorSeq(Ina, Inb, Inc, 0);


    Ima_memory = memoryPhasorValue(device, device->memoryIma);
    Imb_memory = memoryPhasorValue(device, device->memoryImb);
    Imc_memory = memoryPhasorValue(device, device->memoryImc);
    Ina_memory = memoryPhasorValue(device, device->memoryIna);
    Inb_memory = memoryPhasorValue(device, device->memoryInb);
    Inc_memory = memoryPhasorValue(device, device->memoryInc);
    Im_memory0 = phasorAdd(Ima_memory, Imb_memory);
    Im_memory0 = phasorAdd(Im_memory0, Imc_memory);
    In_memory0 = phasorAdd(Ina_memory, Inb_memory);
    In_memory0 = phasorAdd(In_memory0, Inc_memory);

    Ima_delta = phasorSub(Ima, Ima_memory);
    Imb_delta = phasorSub(Imb, Imb_memory);
    Imc_delta = phasorSub(Imc, Imc_memory);
    Ina_delta = phasorSub(Ina, Ina_memory);
    Inb_delta = phasorSub(Inb, Inb_memory);
    Inc_delta = phasorSub(Inc, Inc_memory);
    Im_delta0 = phasorSub(Im0, Im_memory0);
    In_delta0 = phasorSub(In0, In_memory0);

    Iacd = phasorAbs( phasorAdd(Ima_delta, Ina_delta));
    Ibcd = phasorAbs( phasorAdd(Imb_delta, Inb_delta));
    Iccd = phasorAbs( phasorAdd(Imc_delta, Inc_delta));
    Iar = phasorAbs( phasorSub(Ima_delta, Ina_delta));
    Ibr = phasorAbs( phasorSub(Imb_delta, Inb_delta));
    Icr = phasorAbs( phasorSub(Imc_delta, Inc_delta));

    Iar_d = phasorAbs(Ima_delta) + phasorAbs(Ina_delta);
    Ibr_d = phasorAbs(Imb_delta) + phasorAbs(Inb_delta);
    Icr_d = phasorAbs(Imc_delta) + phasorAbs(Inc_delta);
    //稳态量
    //Icd0 = phasorAbs( phasorAdd(Im_delta0, In_delta0));
    Icd0 = 3.0*phasorAbs( phasorAdd(Im0, In0));
    //Ir0 = phasorAbs( phasorSub(Im_delta0, In_delta0));
    Ir0 = 3.0*phasorAbs( phasorSub(Im0, In0));
    Iac = phasorAbs( phasorAdd(Ima, Ina));
    Ibc = phasorAbs( phasorAdd(Imb, Inb));
    Icc = phasorAbs( phasorAdd(Imc, Inc));
    //Iac = abs(phasorAbs( phasorAdd(Ima, Ina))-2*Ic);
   // Ibc = abs(phasorAbs( phasorAdd(Imb, Inb))-2*Ic);
   // Icc = abs(phasorAbs( phasorAdd(Imc, Inc))-2*Ic);
    Iarc = phasorAbs( phasorSub(Ima, Ina));
    Ibrc = phasorAbs( phasorSub(Imb, Inb));
    Icrc = phasorAbs( phasorSub(Imc, Inc));
    

    //变化量相差动
    if ((time-startTime) > 0.0 && Iacd > (0.75*Iar_d) && Iacd > Ih) {
        device->currentDiffSelfTripFlag[0] = 1;
        if (device->currentDiffSelfTripFlag[0] == 1 && device->currentDiffUnionFlag[0] == 1) {
            device->currentDiffTripFlag[0] = 1;
            writeLog(device, "A相变化量相差动保护动作");
        }
    }

    if ((time-startTime) > 0.0 && Ibcd > (0.75*Ibr_d) && Ibcd > Ih) {
        device->currentDiffSelfTripFlag[1] = 1;
        if (device->currentDiffSelfTripFlag[1] == 1 && device->currentDiffUnionFlag[1] == 1) {
            device->currentDiffTripFlag[1] = 1;
            writeLog(device, "B相变化量相差动保护动作");
        }
    }

    if ((time-startTime) > 0.0 && Iccd > (0.75*Icr_d) && Iccd > Ih) {
        device->currentDiffSelfTripFlag[2] = 1;
        if (device->currentDiffSelfTripFlag[2] == 1 && device->currentDiffUnionFlag[2] == 1) {
            device->currentDiffTripFlag[2] = 1;
            writeLog(device, "C相变化量相差动保护动作");
        }
    }

    //稳态I段相差动
    if ((time-startTime) > 0.0 && Iac > (0.6*Iarc) && Iac > Ih) {
        device->currentDiffSelfTripFlag[0] = 1;
        if (device->currentDiffSelfTripFlag[0] == 1 && device->currentDiffUnionFlag[0] == 1) {
            device->currentDiffTripFlag[0] = 1;
            writeLog(device, "A相稳态量I段相差动保护动作");
        }
    }

    if ((time-startTime) > 0.0 && Ibc > (0.6*Ibrc) && Ibc > Ih) {
        device->currentDiffSelfTripFlag[1] = 1;
        if (device->currentDiffSelfTripFlag[1] == 1 && device->currentDiffUnionFlag[1] == 1) {
            device->currentDiffTripFlag[1] = 1;
            writeLog(device, "B相稳态量I段相差动保护动作");
        }
    }

    if ((time-startTime) > 0.0 && Icc > (0.6*Icrc) && Icc > Ih) {
        device->currentDiffSelfTripFlag[2] = 1;
        if (device->currentDiffSelfTripFlag[2] == 1 && device->currentDiffUnionFlag[2] == 1) {
            device->currentDiffTripFlag[2] = 1;
            writeLog(device, "C相稳态量I段相差动保护动作");
        }
    }


    //稳态II段相差动
    if ((time-startTime) > t2set && Iac > (0.6*Iarc) && Iac > Im) {
        device->currentDiffSelfTripFlag[0] = 1;
        if (device->currentDiffSelfTripFlag[0] == 1 && device->currentDiffUnionFlag[0] == 1) {
            device->currentDiffTripFlag[0] = 1;
            writeLog(device, "A相稳态量II段相差动保护动作");
        }
    }

    if ((time-startTime) > t2set && Ibc > (0.6*Ibrc) && Ibc > Im) {
        device->currentDiffSelfTripFlag[1] = 1;
        if (device->currentDiffSelfTripFlag[1] == 1 && device->currentDiffUnionFlag[1] == 1) {
            device->currentDiffTripFlag[1] = 1;
            writeLog(device, "B相稳态量II段相差动保护动作");
        }
    }

    if ((time-startTime) > t2set && Icc > (0.6*Icrc) && Icc > Im) {
        device->currentDiffSelfTripFlag[2] = 1;
        if (device->currentDiffSelfTripFlag[2] == 1 && device->currentDiffUnionFlag[2] == 1) {
            device->currentDiffTripFlag[2] = 1;
            writeLog(device, "C相稳态量II段相差动保护动作");
        }
    }

    int CTUnionFlag = device->currentDiffUnionFlag[0] || device->currentDiffUnionFlag[1] || device->currentDiffUnionFlag[2];
    //零序差动 
    if ((!CTUnionFlag) && (time-startTime) > t1set && (Iac > (0.15*Iarc) && Iac > Il || Ibc > (0.15*Ibrc) && Ibc > Il || Icc > (0.15*Icrc) && Icc > Il) && Icd0 > (0.75*Ir0) && Icd0 > Il && zeroStart == 1) {
        device->currentDiffTripFlag[3] = 1;
        writeLog(device, "零序差动保护动作");
    }
}//CT断线需加在保护起动判据；

double maxValue(double a, double b) {

    if (a >= b) {
        return a;
    } else {
        return b;
    }
}
