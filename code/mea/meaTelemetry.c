//
// Created by yl on 2020/9/11.
//
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

void meaTelemetry(meaDevice* device) {

    int i = 0, j = 0;
    int start = 0;

    //一次侧
    double priP2PVolt[12][POINTS];
    double secP2PVolt[12][POINTS];
    switch (device->deviceType) {
        case LINETYPE :
            // 一、二次侧外接零序电流
            device->priZeroSeqCurr = device->priMeaInstCurr[LINE_LOCAL_CURRENT_ZERO][start];
            device->secZeroSeqCurr = device->secMeaInstCurr[LINE_LOCAL_CURRENT_ZERO][start];
            break;
        case BUSBRKTYPE_SIDE :
        case BUSBRKTYPE_MID :
            // 一、二次侧自产零序电流
            device->priZeroSeqCurr = device->priMeaInstCurr[LINE_LOCAL_CURRENT_A][start] + device->priMeaInstCurr[LINE_LOCAL_CURRENT_B][start] + device->priMeaInstCurr[LINE_LOCAL_CURRENT_C][start];
            device->secZeroSeqCurr = device->secMeaInstCurr[LINE_LOCAL_CURRENT_A][start] + device->secMeaInstCurr[LINE_LOCAL_CURRENT_B][start] + device->secMeaInstCurr[LINE_LOCAL_CURRENT_C][start];
            break;
        case BUSTYPE :
            // v1.0 : 按12PT = 4线路3相配置
            // 一、二次侧自产零序电压
            device->priZeroSeqVolt = device->priMeaInstVolt[LOCAL_VOLTAGE_A][start] + device->priMeaInstVolt[LOCAL_VOLTAGE_B][start] + device->priMeaInstVolt[LOCAL_VOLTAGE_C][start];
            device->secZeroSeqVolt = device->secMeaInstVolt[LOCAL_VOLTAGE_A][start] + device->secMeaInstVolt[LOCAL_VOLTAGE_B][start] + device->secMeaInstVolt[LOCAL_VOLTAGE_C][start];
            // 一、二次侧相电压有效值
            for (i = 0; i < device->voltSize; i++) {
                // 一次侧
                device->priPhaseVoltRMS[i] = rmsCalculate(device->priMeaInstVolt[i], start);
                // 二次侧
                device->secPhaseVoltRMS[i] = rmsCalculate(device->secMeaInstVolt[i], start);
            }
            // 测量侧一、二次侧相间电压有效值
            for (i = 0; i < POINTS; i++) {
                for (j = 0; j < MEA_BUSTYPE_LINE_NUM; j++) {
                    // 一次侧
                    priP2PVolt[MEA_BUSTYPE_PHASE_NUM * j][i] = device->priMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j][start + i] - device->priMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 1][start + i];
                    priP2PVolt[MEA_BUSTYPE_PHASE_NUM * j + 1][i] = device->priMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 1][start + i] - device->priMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 2][start + i];
                    priP2PVolt[MEA_BUSTYPE_PHASE_NUM * j + 2][i] = device->priMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 2][start + i] - device->priMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j][start + i];
                    // 二次侧
                    secP2PVolt[MEA_BUSTYPE_PHASE_NUM * j][i] = device->secMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j][start + i] - device->secMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 1][start + i];
                    secP2PVolt[MEA_BUSTYPE_PHASE_NUM * j + 1][i] = device->secMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 1][start + i] - device->secMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 2][start + i];
                    secP2PVolt[MEA_BUSTYPE_PHASE_NUM * j + 2][i] = device->secMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j + 2][start + i] - device->secMeaInstVolt[MEA_BUSTYPE_PHASE_NUM * j][start + i];
                }
            }
            for (i = 0; i < device->voltSize; i++) {
                device->priP2PVoltRMS[i] = rmsCalculate(priP2PVolt[i], 0);
                device->secP2PVoltRMS[i] = rmsCalculate(secP2PVolt[i], 0);
            }
            return;
        default :
            return;
    }

    // 测量侧一、二次侧相间电压有效值
    for (i = 0; i < POINTS; i++) {
        // 一次侧
        priP2PVolt[0][i] = device->priMeaInstVolt[LOCAL_VOLTAGE_A][start + i] - device->priMeaInstVolt[LOCAL_VOLTAGE_B][start + i];
        priP2PVolt[1][i] = device->priMeaInstVolt[LOCAL_VOLTAGE_B][start + i] - device->priMeaInstVolt[LOCAL_VOLTAGE_C][start + i];
        priP2PVolt[2][i] = device->priMeaInstVolt[LOCAL_VOLTAGE_C][start + i] - device->priMeaInstVolt[LOCAL_VOLTAGE_A][start + i];
        // 二次侧
        secP2PVolt[0][i] = device->secMeaInstVolt[LOCAL_VOLTAGE_A][start + i] - device->secMeaInstVolt[LOCAL_VOLTAGE_B][start + i];
        secP2PVolt[1][i] = device->secMeaInstVolt[LOCAL_VOLTAGE_B][start + i] - device->secMeaInstVolt[LOCAL_VOLTAGE_C][start + i];
        secP2PVolt[2][i] = device->secMeaInstVolt[LOCAL_VOLTAGE_C][start + i] - device->secMeaInstVolt[LOCAL_VOLTAGE_A][start + i];
    }
    for (i = 0; i < 3; i++) {
        device->priP2PVoltRMS[i] = rmsCalculate(priP2PVolt[i], 0);
        device->secP2PVoltRMS[i] = rmsCalculate(secP2PVolt[i], 0);
    }

    // 测量侧一、二次侧相电压有效值
    for (i = 0; i < 3; i++) {
        // 一次侧
        device->priPhaseVoltRMS[i] = rmsCalculate(device->priMeaInstVolt[i], start);
        // 二次侧
        device->secPhaseVoltRMS[i] = rmsCalculate(device->secMeaInstVolt[i], start);
    }
    
    // 测量侧一、二次侧电流有效值
    for (i = 0; i < 3; i++) {
        // 一次侧
        device->priPhaseCurrRMS[i] = rmsCalculate(device->priMeaInstCurr[i], start);
        // 二次侧
        device->secPhaseCurrRMS[i] = rmsCalculate(device->secMeaInstCurr[i], start);
    }

    // 一、二次侧外接零序电压
    device->priZeroSeqVolt = device->priMeaInstVolt[LOCAL_VOLTAGE_ZERO][start];
    device->secZeroSeqVolt = device->secMeaInstVolt[LOCAL_VOLTAGE_ZERO][start];
    
    // 一、二次侧功率
    device->priActivePower = powerCalculate(device->priMeaInstVolt[LOCAL_VOLTAGE_A], device->priMeaInstVolt[LINE_LOCAL_CURRENT_A], 
                                            device->priMeaInstVolt[LOCAL_VOLTAGE_B], device->priMeaInstVolt[LINE_LOCAL_CURRENT_B], 
                                            device->priMeaInstVolt[LOCAL_VOLTAGE_C], device->priMeaInstVolt[LINE_LOCAL_CURRENT_C], 
                                                  start, 'P');
    device->secActivePower = powerCalculate(device->secMeaInstVolt[LOCAL_VOLTAGE_A], device->secMeaInstVolt[LINE_LOCAL_CURRENT_A], 
                                            device->secMeaInstVolt[LOCAL_VOLTAGE_B], device->secMeaInstVolt[LINE_LOCAL_CURRENT_B], 
                                            device->secMeaInstVolt[LOCAL_VOLTAGE_C], device->secMeaInstVolt[LINE_LOCAL_CURRENT_C], 
                                                  start, 'P');
    device->priReactivePower = powerCalculate(device->priMeaInstVolt[LOCAL_VOLTAGE_A], device->priMeaInstVolt[LINE_LOCAL_CURRENT_A],
                                            device->priMeaInstVolt[LOCAL_VOLTAGE_B], device->priMeaInstVolt[LINE_LOCAL_CURRENT_B],
                                            device->priMeaInstVolt[LOCAL_VOLTAGE_C], device->priMeaInstVolt[LINE_LOCAL_CURRENT_C],
                                                  start, 'Q');
    device->secReactivePower = powerCalculate(device->secMeaInstVolt[LOCAL_VOLTAGE_A], device->secMeaInstVolt[LINE_LOCAL_CURRENT_A],
                                            device->secMeaInstVolt[LOCAL_VOLTAGE_B], device->secMeaInstVolt[LINE_LOCAL_CURRENT_B],
                                            device->secMeaInstVolt[LOCAL_VOLTAGE_C], device->secMeaInstVolt[LINE_LOCAL_CURRENT_C],
                                                  start, 'Q');

    int calculatePoints = 48;
    // 一、二次侧频率
    device->priFrequency = freqCalculate(device->priMeaInstVolt[LOCAL_VOLTAGE_A], calculatePoints, start);
    device->secFrequency = freqCalculate(device->secMeaInstVolt[LOCAL_VOLTAGE_A], calculatePoints, start);

    // 一、二次侧功率因数
    device->priPowerFactor = powerFactorCalculate(device->priActivePower, device->priReactivePower);
    device->secPowerFactor = powerFactorCalculate(device->secActivePower, device->secReactivePower);

    // 同期信息
    // 测量侧电压、频率
    switch (device->remoteVoltMode) {
        case SYNVOLT_TYPE_A :
        case SYNVOLT_TYPE_B :
        case SYNVOLT_TYPE_C :
            inst2phasor(device->priMeaInstVolt[device->remoteVoltMode], start, &device->localSideVolt);
            device->localSideFrequency = freqCalculate(device->priMeaInstVolt[device->remoteVoltMode], calculatePoints, start);
            break;
        case SYNVOLT_TYPE_CA :
        case SYNVOLT_TYPE_BC :
        case SYNVOLT_TYPE_AB :
            inst2phasor(priP2PVolt[device->remoteVoltMode - 3], 0, &device->localSideVolt);
            device->localSideFrequency = freqCalculate(priP2PVolt[device->remoteVoltMode - 3], calculatePoints, 0); // start在计算相间电压时已经考虑过
            break;

    }
    // 抽取侧电压、频率
    inst2phasor(device->priMeaInstVolt[REMOTE_VOLTAGE], start, &device->remoteSideVolt);
    device->remoteSideFrequency = freqCalculate(device->priMeaInstVolt[REMOTE_VOLTAGE], 12, start);
    // 幅值、相角计算
    device->localSideVoltAmp = phasorAbs(device->localSideVolt);
    device->remoteSideVoltAmp = phasorAbs(device->remoteSideVolt);
    device->localSideVoltAngle = phasorAngle(device->localSideVolt);
    device->remoteSideVoltAngle = phasorAngle(device->remoteSideVolt);
    // 四差计算
    device->voltDifference = voltDiffCalculate(device->localSideVolt, device->remoteSideVolt); // 压差
    device->angleDifference = angleDiffCalculate(device->localSideVolt, device->remoteSideVolt); // 角差
    device->freqDifference = freqDiffCalculate(device->localSideFrequency, device->remoteSideFrequency); // 频差
    device->slipDifference = slipDiffCalculate(device->localSideFrequency, device->remoteSideFrequency); // 滑差
}
