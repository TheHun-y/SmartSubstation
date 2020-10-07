#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>

/**
 * 工频变化量距离继电器
 * 由line函数按相调用，对于相间故障phase=0，代表AB相间，以此类推
 *  
 */
void phaseDeltaDistanceRelay(Device* device, int phase);
void groundDeltaDistanceRelay(Device* device, int phase);

void halfPhaseDeltaDistanceRelay(Device* device, int phase);
void halfGroundDeltaDistanceRelay(Device* device, int phase);

void deltaDistanceRelay(Device* device, int phase) {
    double startTime = 0.02/48*4.0;
    double timeSet = 0.01+0.02/48.0*4.0    +1;
    
    if (device->openPhaseFlag[phase] == 0) {
        if (device->time - device->startTime >= timeSet) {
            groundDeltaDistanceRelay(device, phase);
        } else if (device->time - device->startTime < startTime) {
        
        } else if (device->time - device->startTime < timeSet) {
            halfGroundDeltaDistanceRelay(device, phase);
        } else {

        }
    }
    if (device->openPhaseFlag[phase] == 0 && device->openPhaseFlag[phaseMatch(phase)] == 0) {
        if (device->time - device->startTime >= timeSet) {
            phaseDeltaDistanceRelay(device, phase);
       } else if (device->time - device->startTime < startTime) {
        
        } else if (device->time - device->startTime < timeSet) {
            halfPhaseDeltaDistanceRelay(device, phase);
        } else {

        }
    }

    // device->test[0] = device->halfWaveP2GSum[1];
    // device->test[1] = device->deltaDistanceP2GSetValue[1];

}

void phaseDeltaDistanceRelay(Device* device, int phase){

    Phasor Up1, Ip1, Up2, Ip2, Upp, Ipp, Uop, Upm1, Ipm1, Upm2, Ipm2, Uppm, Ippm, Uopm, deltaUop;

    // 整定值
    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);
    // 故障前工作电压的记忆量
    double Uz;

    // 变量本地化
    Up1 = device->phasor[phase];
    Ip1 = device->phasor[phase + 3];
    Up2 = device->phasor[phaseMatch(phase)];
    Ip2 = device->phasor[phaseMatch(phase) + 3];

    // 寻找记忆量
    if (phase == 0){
        Upm1 = memoryPhasorValue(device, device->memoryVma);
        Ipm1 = memoryPhasorValue(device, device->memoryIma);
        Upm2 = memoryPhasorValue(device, device->memoryVmb);
        Ipm2 = memoryPhasorValue(device, device->memoryImb);
    }
    else if (phase == 1){
        Upm1 = memoryPhasorValue(device, device->memoryVmb);
        Ipm1 = memoryPhasorValue(device, device->memoryImb);
        Upm2 = memoryPhasorValue(device, device->memoryVmc);
        Ipm2 = memoryPhasorValue(device, device->memoryImc);
    }
    else if (phase == 2){
        Upm1 = memoryPhasorValue(device, device->memoryVmc);
        Ipm1 = memoryPhasorValue(device, device->memoryImc);
        Upm2 = memoryPhasorValue(device, device->memoryVma);
        Ipm2 = memoryPhasorValue(device, device->memoryIma);
    }

    // 相间量计算
    Upp = phasorSub(Up1, Up2);
    Ipp = phasorSub(Ip1, Ip2);
    Uppm = phasorSub(Upm1, Upm2);
    Ippm = phasorSub(Ipm1, Ipm2);

    // 极化电压计算
    Uop = phasorSub(Upp, phasorMulti(Zzd, Ipp));
    Uopm = phasorSub(Uppm, phasorMulti(Zzd, Ippm));

    /*if (device->PTBreakFlag == 0) {
        Uz = device->deltaDistanceP2PSetValue[phase];
    } else {
        Uz = 1.5*device->deltaDistanceP2PSetValue[phase];
    }*/

    if (device->PTBreakFlag == 0) {
        Uz = phasorAbs(Uopm);
    } else {
        Uz = 1.5*phasorAbs(Uopm);
    }

    // 变化量计算及判据实现
    deltaUop = phasorSub(Uop, Uopm);

    if (phasorAbs(deltaUop) > Uz && device->tripPhase[phase] == 1 && device->tripPhase[phaseMatch(phase)] == 1){
        device->deltaDistanceTripFlag[phase] = 1;
        device->deltaDistanceTripFlag[phaseMatch(phase)] = 1;
        writeLogWithPhase(device, "%c相工频变化量相间距离保护动作", phase);
        writeLogWithPhase(device, "%c相工频变化量相间距离保护动作", phaseMatch(phase));
    }

}

void groundDeltaDistanceRelay(Device* device, int phase){

    Phasor Up, Ip, Ik, I0, Uop, Upm, Ipm, Ikm, I0m, Uopm, deltaUop;

    // 整定值
    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);

    // 故障前工作电压的记忆量
    double Uz;
    // 零序补偿系数
    double k = device->KZ;

    // 变量本地化
    Up = device->phasor[phase];
    Ip = device->phasor[phase + 3];
    I0 = phasorSeq(device->phasor[3], device->phasor[4], device->phasor[5], 0);
    Ik = phasorAdd(Ip, phasorNumMulti(3*k, I0));

    if (phase == 0){
        Upm = memoryPhasorValue(device, device->memoryVma);
        Ipm = memoryPhasorValue(device, device->memoryIma);
    }
    else if (phase == 1){
        Upm = memoryPhasorValue(device, device->memoryVmb);
        Ipm = memoryPhasorValue(device, device->memoryImb);
    }
    else if (phase == 2){
        Upm = memoryPhasorValue(device, device->memoryVmc);
        Ipm = memoryPhasorValue(device, device->memoryImc);
    }
    I0m = phasorSeq(memoryPhasorValue(device, device->memoryIma), memoryPhasorValue(device, device->memoryImb), memoryPhasorValue(device, device->memoryImc), 0);
    Ikm = phasorAdd(Ipm, phasorNumMulti(3*k, I0m));

    // 极化电压计算
    Uop = phasorSub(Up, phasorMulti(Zzd, Ik));
    Uopm = phasorSub(Upm, phasorMulti(Zzd, Ikm));

    if (device->PTBreakFlag == 0) {
        Uz = device->deltaDistanceP2GSetValue[phase]/24.0;
    } else {
        Uz = 1.5*device->deltaDistanceP2GSetValue[phase]/24.0;
    }

    // if (device->PTBreakFlag == 0) {
    //     Uz = phasorAbs(Uopm);
    // } else {
    //     Uz = 1.5*phasorAbs(Uopm);
    // }



    // 变化量计算及判据实现
    deltaUop = phasorSub(Uop, Uopm);

    if (phasorAbs(deltaUop) > Uz && device->tripPhase[phase] == 1){

        device->deltaDistanceTripFlag[phase] = 1;
        writeLogWithPhase(device, "%c相工频变化量接地距离保护动作", phase);

    }
    /*if (phase == 1) {
        device->test[phase] = phasorAbs(I0);//device->deltaDistanceP2GSetValue[2];//device->operateVoltageP2GMemory[2][memoryIndex(device)];//device->deltaDistanceP2GSetValue[0];
        device->test[2] = phasorAbs(Uop);
    } else {
        //device->test[phase] = phasorAbs(deltaUop);
    }*/

}

/**
 * 工频变化量距离继电器
 * 由line函数按相调用，对于相间故障phase=0，代表AB相间，以此类推
 *  
 */

void halfPhaseDeltaDistanceRelay(Device* device, int phase){

    double deltaUop = 0.0;

    double uop1 = 0.0, uop2 = 0.0, uopm = 0.0;
    double* instV1, *instV2, *instI1, *instI2;
    double result = 0.0;

    // 整定值
    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);
    double R =Zzd.real;
    double L = Zzd.img/(2.0*50*PI);
    // 故障前工作电压的记忆量
    double Uz;

    switch (phase) {
        case 0: 
            instV1 = device->filterVma ;
            instV2 = device->filterVmb ;
            instI1 = device->filterIma ;
            instI2 = device->filterImb ;
        break;
            case 1:
            instV1 = device->filterVmb ;
            instV2 = device->filterVmc ;
            instI1 = device->filterImb ;
            instI2 = device->filterImc ;
            break;
        case 2:
            instV1 = device->filterVmc ;
            instV2 = device->filterVma ;
            instI1 = device->filterImc ;
            instI2 = device->filterIma ;
            break;
    }


    uop1 = instV1[0] - (R*instI1[0] + L*(instI1[0]-instI1[1])/(1.0*0.02/48.0));
    uop2 = instV2[0] - (R*instI2[0] + L*(instI2[0]-instI2[1])/(1.0*0.02/48.0));
    uopm = device->operateVoltageP2PMemory[phase][memoryIndex(device)];
    result = fabs(uop1 - uop2 - uopm);
    
    device->halfWaveP2PSum[phase] += result;
    device->halfWaveP2PSumCnt[phase]++;

     // 变化量计算及判据实现
    deltaUop = device->halfWaveP2PSum[phase];

    if (device->halfWaveP2PSumCnt[phase] == 23) {
        device->halfWaveP2PSumCnt[phase] = 0;
        device->halfWaveP2PSum[phase] = 0;
    }

    if (device->PTBreakFlag == 0) {
        Uz = device->deltaDistanceP2PSetValue[phase];
    } else {
        Uz = 1.5*device->deltaDistanceP2PSetValue[phase];
    }

    if (deltaUop > Uz && device->tripPhase[phase] == 1 && device->tripPhase[phaseMatch(phase)] == 1){
        device->deltaDistanceTripFlag[phase] = 1;
        device->deltaDistanceTripFlag[phaseMatch(phase)] = 1;
        writeLogWithPhase(device, "%c相工频变化量相间距离保护动作", phase);
        writeLogWithPhase(device, "%c相工频变化量相间距离保护动作", phaseMatch(phase));
    }

}

void halfGroundDeltaDistanceRelay(Device* device, int phase){

    double deltaUop = 0.0;

    double uop1 = 0.0, uopm = 0.0;
    double* instV1, *instI1, instI0[3];
    double result = 0.0;

    // 整定值
    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);
    double R = Zzd.real;
    double L = Zzd.img/(2.0*50*PI);
    double K = device->KZ;
    // 故障前工作电压的记忆量
    double Uz;

    int i = 0;

    switch (phase) {
        case 0: 
            instV1 = device->filterVma ;
            instI1 = device->filterIma ;
        break;
            case 1:
            instV1 = device->filterVmb ;
            instI1 = device->filterImb ;
            break;
        case 2:
            instV1 = device->filterVmc ;
            instI1 = device->filterImc ;
            break;
    }
   
    for (i = 0; i < 3; i++) {
        instI0[i] = device->filterIma[i] + device->filterImb[i] + device->filterImc[i];
    }


    uop1  = instV1[0] - (R*instI1[0] + L*(instI1[0]-instI1[1])/(1.0*0.02/48.0)) - K*(R*instI0[0] + L*(instI0[0]-instI0[1])/(1.0*0.02/48.0));
    uopm = device->operateVoltageP2GMemory[phase][memoryIndex(device)];
    result = fabs(uop1 - uopm);
    
    device->halfWaveP2GSum[phase] += result;
    device->halfWaveP2GSumCnt[phase]++;

     // 变化量计算及判据实现
    deltaUop = device->halfWaveP2GSum[phase];

    if (device->halfWaveP2GSumCnt[phase] == 23) {
        device->halfWaveP2GSumCnt[phase] = 0;
        device->halfWaveP2GSum[phase] = 0;
    }


    if (device->PTBreakFlag == 0) {
        Uz = device->deltaDistanceP2GSetValue[phase];
    } else {
        Uz = 1.5*device->deltaDistanceP2GSetValue[phase];
    }


    if (deltaUop > Uz && device->tripPhase[phase] == 1){

        device->deltaDistanceTripFlag[phase] = 1;
        writeLogWithPhase(device, "%c相工频变化量接地距离保护动作", phase);

    }


    if (phase == 1) {
        device->test[phase] = instI1[0];//(instI1[0]-instI1[1])/(1.0*0.02/48.0);//device->deltaDistanceP2GSetValue[2];//device->operateVoltageP2GMemory[2][memoryIndex(device)];//device->deltaDistanceP2GSetValue[0];//
        device->test[2] = instI0[0];
        device->test[0] = uop1;
    } else {
        
    }
    

}

