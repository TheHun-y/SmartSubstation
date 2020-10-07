//
// Created by yl on 2020/9/12.
//
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
//#include "../dataStruct.h"

void meaRecord(meaDevice* device) {
    FILE* fp = NULL;
    char recordFileName[STRING_LENGTH];
    char* fileType[3];
    fileType[0] = "PrimarySide";
    fileType[1] = "SecondarySide";
    fileType[2] = "SynchronousInfo";

    int fileNums = 0;
    if (device->deviceType == BUSTYPE) {
        fileNums = 2;
    } else {
        fileNums = 3;
    }
    int i = 0;
    char* synType;
    switch (device->remoteVoltMode) {
        case SYNVOLT_TYPE_A :
            synType = "A相-相电压";
            break;
        case SYNVOLT_TYPE_B :
            synType = "B相-相电压";
            break;
        case SYNVOLT_TYPE_C :
            synType = "C相-相电压";
            break;
        case SYNVOLT_TYPE_AB :
            synType = "AB-相间电压";
            break;
        case SYNVOLT_TYPE_BC :
            synType = "BC-相间电压";
            break;
        case SYNVOLT_TYPE_CA :
            synType = "CA-相间电压";
            break;
    }


    for (i = 0; i < fileNums; i++) {
        sprintf(recordFileName, "%s-%s-record.txt", device->deviceFileName, fileType[i]);
        fp = fopen(recordFileName, "at+");

        if (fp != NULL) {
            // 标题
            switch (i) {
                case 0 :
                    if (device->fileExistFlag[0] == 0) {
                        fprintf(fp, "[%s]一次侧数据---kV/kA\n", device->deviceName);
                        fprintf(fp,
                                "\t\tTIME\t\tVa\t\tVb\t\tVc\t\tIa\t\tIb\t\tIc\t\tVab\t\tVbc\t\tVca\t\tfreq\t\tP\t\tQ\t\tpf\t\t3I0\t\t3V0\n");
                        device->fileExistFlag[0] = 1;
                    }
                    fprintf(fp, "%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f\n",
                                device->instTime[i],
                                device->priPhaseVoltRMS[LOCAL_VOLTAGE_A], device->priPhaseVoltRMS[LOCAL_VOLTAGE_B], device->priPhaseVoltRMS[LOCAL_VOLTAGE_C],
                                device->priPhaseCurrRMS[LINE_LOCAL_CURRENT_A], device->priPhaseCurrRMS[LINE_LOCAL_CURRENT_B], device->priPhaseCurrRMS[LINE_LOCAL_CURRENT_C],
                                device->priP2PVoltRMS[0], device->priP2PVoltRMS[1], device->priP2PVoltRMS[2],
                                device->priFrequency, device->priActivePower, device->priReactivePower,
                                device->priPowerFactor, device->priZeroSeqCurr, device->priZeroSeqVolt
                            );
                    break;
                case 1 :
                    if (device->fileExistFlag[1] == 0) {
                        fprintf(fp, "[%s]二次侧数据---kV/kA\n", device->deviceName);
                        fprintf(fp,
                                "\t\tTIME\t\tVa\t\tVb\t\tVc\t\tIa\t\tIb\t\tIc\t\tVab\t\tVbc\t\tVca\t\tfreq\t\tP\t\tQ\t\tpf\t\t3I0\t\t3V0\n");
                        device->fileExistFlag[1] = 1;
                    }
                    fprintf(fp, "%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f\n",
                            device->instTime[i],
                            device->secPhaseVoltRMS[LOCAL_VOLTAGE_A], device->secPhaseVoltRMS[LOCAL_VOLTAGE_B], device->secPhaseVoltRMS[LOCAL_VOLTAGE_C],
                            device->secPhaseCurrRMS[LINE_LOCAL_CURRENT_A], device->secPhaseCurrRMS[LINE_LOCAL_CURRENT_B], device->secPhaseCurrRMS[LINE_LOCAL_CURRENT_C],
                            device->secP2PVoltRMS[0], device->secP2PVoltRMS[1], device->secP2PVoltRMS[2],
                            device->secFrequency, device->secActivePower, device->secReactivePower,
                            device->secPowerFactor, device->secZeroSeqCurr, device->secZeroSeqVolt
                    );
                    break;
                case 2 :
                    if (device->fileExistFlag[2] == 0) {
                        fprintf(fp, "[%s]同期数据---kV/kA\t\t", device->deviceName);
                        fprintf(fp, "同期电压类型：%s\n", synType);
                        fprintf(fp,
                                "\tTIME\t\tV-local\t\tP-local\t\tP-remote\t\tV-local\t\tf-local\t\tf-remote\t\tDelta-V\t\tDelta-a\t\tDelta-f\t\tDelta-w\n");
                        device->fileExistFlag[2] = 1;
                    }
                    fprintf(fp, "%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f,\t%12.6f\n",
                            device->instTime[i],
                            device->localSideVoltAmp, device->localSideVoltAngle, device->remoteSideVoltAmp, device->remoteSideVoltAngle,
                            device->localSideFrequency, device->remoteSideFrequency,
                            device->voltDifference, device->angleDifference, device->freqDifference, device->slipDifference
                    );
                    break;
            }
            fclose(fp);
            fp = NULL;
        }
    }

}

