
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>

void busBrkFailureJudgeAux_500kV(Device* device) {

    if (device->memoryI0StartTime != 0 && device->time - device->memoryI0StartTime <= 5) {
        device->sampleBrkFailureFlag = 1;
        return;
    } else {
        device->memoryI0StartTime = 0;
    }

    int flag_phase = 0;
    int flag_I0_1 = 0;
    int flag_I0_2 = 0;
    int flag_I2 = 0;
    int flag_inst = 0;

    double In = 1.0;
    double I0 = 3*phasorAbs(phasorSeq(device->phasor[3], device->phasor[4], device->phasor[5], 0));
    double I2 = 3*phasorAbs(phasorSeq(device->phasor[3], device->phasor[4], device->phasor[5], 2));
    double memoryI0 = 3*phasorAbs(phasorSeq(device->memoryIma[0], device->memoryImb[0], device->memoryImc[0], 0));
    double memoryI2 = 3*phasorAbs(phasorSeq(device->memoryIma[0], device->memoryImb[0], device->memoryImc[0], 2));

    int i = 0;

    for (i = 0; i < 3; i++) {
        if (phasorAbs(device->phasor[i+3]) > 1.1*In) {
            flag_phase = 1;
            break;
        }
    }

    if (fabs(I0 - device->memory3I0) > 0.03*In) {
        flag_I0_1 = 1;
        device->memoryI0StartTime = device->time;
    }
    if (memoryI0 < 0.08*In && I0 > 0.1*In) {
        flag_I0_2 = 1;
    }
    if (memoryI2 < 0.08*In && I2 > 0.1*In) {
        flag_I2 = 1;
    }

    if (device->instDeltaStartTime == 0) {
        int index = (memoryIndex(device)+1) % POINTS;
        device->sampleInstDeltaSum += fabs(device->filterIma[0] - device->memoryCurrent[0][index]);

        if (device->sampleInstDeltaSum > 0.2*In) {
            device->sampleInstDeltaSum = 0;
            device->instDeltaStartTime = device->time;
            flag_inst = 1;
        }
    } else {
        if (device->time - device->instDeltaStartTime < 5) {
            flag_inst = 1;
        } else {
            device->instDeltaStartTime = 0;

            int index = (memoryIndex(device)+1) % POINTS;
            device->sampleInstDeltaSum += fabs(device->filterIma[0] - device->memoryCurrent[0][index]);

            if (device->sampleInstDeltaSum > 0.2*In) {
                device->sampleInstDeltaSum = 0;
                device->instDeltaStartTime = device->time;
                flag_inst = 1;
            }
        }
    }

    if (flag_I0_1 || flag_I0_2 || flag_I2 || flag_inst || flag_phase) {
        device->sampleBrkFailureFlag = 1;
    }

}