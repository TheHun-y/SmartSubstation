#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>

double* chooseCurrentPhase(Device* device, int phase);
double* chooseVoltagePhase(Device* device, int phase);
int overCurrentStart(Device* device, int phase);
int singlePhaseStart(Device* device, double* inst);
int zeroSequenceCurrentStart(Device* device,int phase);
void recordMemoryUI(Device* device); // 未实现
void recordOperateVoltP2P(Device* device, int phase);
void recordOperateVoltP2G(Device* device, int phase);
void ratedParam(Device* device);


// 线路启动判据
void lineStarter(Device* device, int phase) {
    int flag1 = overCurrentStart(device, phase);
    int flag2 = (zeroSequenceCurrentStart(device, phase) && !device->CTBreakFlag);

    if (flag1 == 1 || flag2 == 1) {
        device->startTime = device->time; // 虽然与各自的置位操作重复，但暂时还是保留。
        // 存储记忆量
        recordMemoryUI(device);

    } else {
        device->startTime = 1000;
    }
}



/**
 * 过电流启动判据
 * 突变量整定值为0.5kA
 */
int overCurrentStart(Device* device, int phase) {
    double* instIm;
    instIm = chooseCurrentPhase(device, phase);
    
    if (singlePhaseStart(device, instIm) == 1) {
        device->startFlag = 1;
        device->startTime = device->time;

        writeLogWithPhase(device, "%c相电流突变量启动元件动作", phase);
        return 1;
    } 
    return 0;
}

int singlePhaseStart(Device* device, double* inst) {
    Phasor phasorNow, phasorBefore, phasorDelta;
    double amp;

    inst2phasor(inst, 0, &phasorNow);
    inst2phasor(inst, 1*POINTS, &phasorBefore);
    
    phasorDelta = phasorSub(phasorNow, phasorBefore);

    amp = phasorAbs(phasorDelta);

    // 突变量整定值为0.5kA
    if (amp > device->lineStartSetValue[0]) {
        return 1;
    } else {
        return 0;
    }   
}

double* chooseCurrentPhase(Device* device, int phase) {
    double* instIm;
    switch (phase) {
        case 0:
            /* code */
            instIm = device->filterIma;
            break;
        case 1:
            instIm = device->filterImb;
            break;
        case 2:
            instIm = device->filterImc;
            break;
    }
    return instIm;
}

double* chooseVoltagePhase(Device* device, int phase) {
    double* instVm;
    switch (phase) {
        case 0:
            /* code */
            instVm = device->filterVma;
            break;
        case 1:
            instVm = device->filterVmb;
            break;
        case 2:
            instVm = device->filterVmc;
            break;
    }
    return instVm;
}

/**
 * 零序过电流起动判据
 * 整定值为0.1kA
 */
int zeroSequenceCurrentStart(Device* device,int phase){

    Phasor I0, temp;
    double abs;

    temp = phasorAdd(device->phasor[3], device->phasor[4]);
    I0 = phasorAdd(temp, device->phasor[5]);
    abs = phasorAbs(I0); //使用3I0

    if (abs > device->lineStartSetValue[1] && phasorAbs(device->zeroSeqCurrentOUT) > device->lineStartSetValue[1]){
        device->startFlag = 1;
        
        device->startTime = device->time;

        writeLog(device, "零序过电流启动元件动作");
        return 1;
    }
    return 0;
}


/**
 * 启动后对进行记忆
 * 由总启动元件lineStarter调用
 */
void recordMemoryUI(Device* device) {
    // 记录当前时刻
    int i, phase;
    for (i = 3*POINTS; i > 2*POINTS; i--){
        inst2phasor(device->filterVma, i, &device->memoryVma[3*POINTS-i]);  inst2phasor(device->filterIma, i, &device->memoryIma[3*POINTS-i]);
        inst2phasor(device->filterVmb, i, &device->memoryVmb[3*POINTS-i]);  inst2phasor(device->filterImb, i, &device->memoryImb[3*POINTS-i]);
        inst2phasor(device->filterVmc, i, &device->memoryVmc[3*POINTS-i]);  inst2phasor(device->filterImc, i, &device->memoryImc[3*POINTS-i]);
        inst2phasor(device->filterVna, i, &device->memoryVna[3*POINTS-i]);  inst2phasor(device->filterIna, i, &device->memoryIna[3*POINTS-i]);
        inst2phasor(device->filterVnb, i, &device->memoryVnb[3*POINTS-i]);  inst2phasor(device->filterInb, i, &device->memoryInb[3*POINTS-i]);
        inst2phasor(device->filterVnc, i, &device->memoryVnc[3*POINTS-i]);  inst2phasor(device->filterInc, i, &device->memoryInc[3*POINTS-i]);
    }
    // 线路保护-工频变化量距离工作电压记忆
    for (phase = 0; phase < 3; phase++) {
        recordOperateVoltP2P(device, phase);
        recordOperateVoltP2G(device, phase);
    }

    // 母线保护-断路器失灵-电流工频变化量瞬时值记忆
    double* ptr;
    for (phase = 0; phase < 3; phase++) {
        for (i = 3*POINTS; i > 2*POINTS; i--) {
            ptr = findInstCurrentPtr(device, phase);
            device->memoryCurrent[phase][3*POINTS-i] = ptr[i];
        }
    }
}


void recordOperateVoltP2P(Device* device, int phase) {

    double R, L;
    Phasor Z;
    double instV1[POINTS+2], instV2[POINTS+2], instI1[POINTS+2], instI2[POINTS+2], uop1[POINTS], uop2[POINTS];
    double* result = device->operateVoltageP2PMemory[phase];

    double temp = 0.0;
    Z = ampAngle2phasor(device->deltaImpedance, device->lineZ1Angle);
    R = Z.real; 
    L = Z.real/(2.0*PI*50);

    int i = 0;
    for (i = 0; i < POINTS+2; i++) {
        switch (phase) {
            case 0: 
            instV1[i] = device->filterVma[3*POINTS+1-i];
            instV2[i] = device->filterVmb[3*POINTS+1-i];
            instI1[i] = device->filterIma[3*POINTS+1-i];
            instI2[i] = device->filterImb[3*POINTS+1-i];
            break;
            case 1:
            instV1[i] = device->filterVmb[3*POINTS+1-i];
            instV2[i] = device->filterVmc[3*POINTS+1-i];
            instI1[i] = device->filterImb[3*POINTS+1-i];
            instI2[i] = device->filterImc[3*POINTS+1-i];
            break;
            case 2:
            instV1[i] = device->filterVmc[3*POINTS+1-i];
            instV2[i] = device->filterVma[3*POINTS+1-i];
            instI1[i] = device->filterImc[3*POINTS+1-i];
            instI2[i] = device->filterIma[3*POINTS+1-i];
            break;
        }
    }
    // uop的0对应inst的1
    for (i = 0; i < POINTS; i++) {
        uop1[i] = instV1[i+1] - (R*instI1[i+1] + L*(instI1[i+2]-instI1[i])/(2.0*0.02/48.0));
        uop2[i] = instV2[i+1] - (R*instI2[i+1] + L*(instI2[i+2]-instI2[i])/(2.0*0.02/48.0));
        result[i] = uop1[i] - uop2[i];
    }

    // 门槛写入
    for (i = 0; i < POINTS/2; i++) {
        temp += fabs(result[i]);
    }

    device->deltaDistanceP2PSetValue[phase] = 1.25*temp;
}

void recordOperateVoltP2G(Device* device, int phase) {

    double R, L;
    Phasor Z;
    double K = device->KZ;
    double instV1[POINTS+2], instI1[POINTS+2], instI0[POINTS+2];
    double* result = device->operateVoltageP2GMemory[phase];
    double temp = 0.0;
    Z = ampAngle2phasor(device->deltaImpedance, device->lineZ1Angle);
    R = Z.real; 
    L = Z.real/(2.0*PI*50);

    int i = 0;
    for (i = 0; i < POINTS+2; i++) {
        switch (phase) {
            case 0: 
            instV1[i] = device->filterVma[3*POINTS+1-i];
            instI1[i] = device->filterIma[3*POINTS+1-i];
            break;
            case 1:
            instV1[i] = device->filterVmb[3*POINTS+1-i];
            instI1[i] = device->filterImb[3*POINTS+1-i];
            break;
            case 2:
            instV1[i] = device->filterVmc[3*POINTS+1-i];
            instI1[i] = device->filterImc[3*POINTS+1-i];
            break;
        }
        instI0[i] = device->filterIma[3*POINTS+1-i] + device->filterImb[3*POINTS+1-i] + device->filterImc[3*POINTS+1-i];
    }
    // uop的0对应inst的1
    for (i = 0; i < POINTS; i++) {
        result[i] = instV1[i+1] - (R*instI1[i+1] + L*(instI1[i+1]-instI1[i])/(1.0*0.02/48.0)) - K*(3*R*instI0[i+1] + 3*L*(instI0[i+1]-instI0[i])/(1.0*0.02/48.0));
    }

    // 门槛写入
    for (i = 0; i < POINTS/2; i++) {
        temp += fabs(result[i]);
    }

    device->deltaDistanceP2GSetValue[phase] = 1.25*temp;
}


/**
 * 计算额定参数
 */
/**void ratedParam(Device* device) {
    device->ratedVoltage = phasorAbs(device->memoryVma[0]) / 1.4142;
    device->ratedCurrent = phasorAbs(device->memoryIma[0]) / 1.4142;
    device->ratedBetweenVoltage = phasorAbs(phasorSub(device->memoryVmb[0], device->memoryVma[0])) / 1.4142;
    device->ratedBetweenCurrent = phasorAbs(phasorSub(device->memoryImb[0], device->memoryIma[0])) / 1.4142;
}**/
