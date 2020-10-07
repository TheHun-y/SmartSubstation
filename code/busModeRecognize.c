
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>


void busModeRecognize(BusDevice* device) {

    int ML[2], FD[2], QS[24][4], PL[4];
    int i, j, k, m = 0;

    for (i = 0; i < 2; i++){
        ML[i] = device->busBrkStatus[i*3] || device->busBrkStatus[i*3+1] || device->busBrkStatus[i*3+2];
        FD[i] = device->busBrkStatus[i*3+6] || device->busBrkStatus[i*3+7] || device->busBrkStatus[i*3+8];
    }

    for (i = 0; i < 24; i++){
        for (j = 0; j < 4; j++){

            QS[i][j] = device->busModeStatus[i][j];

        }
    }
    // 各母线投运状态识别
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 24; j++) {
            if (QS[j][i] != 0) {
                device->busRunStatus[i] = 1;
                break;
            }
        }
    }

    for (i = 0; i < 4; i++){

        PL[i] = device->bypassBusStatus[i];
    }
    // 初始化旁母状态
    /*device->bypassBusMode = 0;

    // 旁路母线运行状态判别
    if (ML[0] == 1){
        if (PL[0] == 1 && PL[2] == 1){
            device->bypassBusMode = 1;
        } else {
            if (PL[0] == 1 && PL[3] == 1){
                if (PL[1] == 1){
                    device->bypassBusMode = 3;
                } else {
                device->bypassBusMode = 2;
            } else if (PL[1] == 1 && PL[3] == 1 && PL[0] != 1) {
                device->bypassBusMode = 3;
            }
 
        }
        
    }*/

    switch(device->bypassBusMode){
        case 0: busWriteLog(device, "旁母未投运"); break;
        case 1: busWriteLog(device, "I母带旁母运行"); break;
        case 2: busWriteLog(device, "II母带旁母运行"); break;
        //case 3: busWriteLog(device, "ML1仅作母联运行"); break;
    }

    // 校准母联兼旁路断路器是否作为母联断路器运行:默认只有ML1作为母联兼旁路断路器
    /*if (ML[0] == 1 && PL[1] == 1 && PL[3] == 1){
        ML[0] = 1;
    } else {
        ML[0] = 0;
    }*/
    if (device->bypassBusMode != 0) {
        ML[0] = 0;
    }

    // 双母并列：9
    if (ML[0] == 1 || ML[1] == 1){

        device->busMode = 9;

    } else {
    // 单母：1/2 与 双母单分段：6/7
        if (FD[0] == 1 && FD[1] == 0){

            for (i = 0; i < 24; i++){
                if ( QS[i][2] != 0 || QS[i][3] != 0){
                
                    m = m+1;
                }
            }

            if (m != 0){
                device->busMode = 7;
                m = 0;
            } else {
                device->busMode = 1;
            }
        }

        if (FD[0] == 0 && FD[1] == 1){

            for (i = 0; i < 24; i++){
                if ( QS[i][0] != 0 || QS[i][1] != 0){

                    k = k+1;
                } 
            }

            if (k != 0){
                device->busMode = 6;
                k = 0;
            } else {
                device->busMode = 2;
            }
        }
        
    // 单母分段：3/4 与 双母双分段：8 与 全线停运：0
        if (FD[0] == 0 && FD[1] == 0){

             for (i = 0; i < 24; i++){

                if ( QS[i][0] != 0 || QS[i][1] != 0){

                    k = k+1;
                } 

                if ( QS[i][2] != 0 || QS[i][3] != 0){
                
                    m = m+1;
                }
            }

            if (k != 0){
                if (m != 0){
                    device->busMode = 8;
                    m = 0;
                } else {
                    device->busMode = 3;
                }
                k = 0;
            } else {
                if (m != 0){
                   device->busMode = 4;
                   m = 0;
                } else {
                   device->busMode = 0;
                }
            }
           
        }
    // 单母：1/2 与 双母：5 与 全线停运：0
        if (FD[0] == 1 && FD[1] == 1){

            for (i = 0; i < 24; i++){

                if ( QS[i][0] != 0 || QS[i][1] != 0){

                    k = k+1;
                } 

                if ( QS[i][2] != 0 || QS[i][3] != 0){
                
                    m = m+1;
                }
            }

            if (k != 0){
                if (m != 0){
                    device->busMode = 5;
                    m = 0;
                } else {
                    device->busMode = 1;
                }
                k = 0;
            } else {
                if (m != 0){
                   device->busMode = 2;
                   m = 0;
                } else {
                   device->busMode = 0;
                }
            }
        }
        
    }
    // 主母线全线停运判断：0
    if (device->busMode != 0){
        for (i = 0; i < 24; i++){
             for (j = 0; j < 4; j++){
                 if (QS[i][j] != 0){
                     k = k+1;
                     break;
                }
            }
        }

        if (k == 0){
        device->busMode = 0;
        }
        k = 0;
    }
   
    switch(device->busMode){
        case 0: busWriteLog(device, "主母线全线停运"); break;
        case 1: busWriteLog(device, "I母单母线运行"); break;
        case 2: busWriteLog(device, "II母单母线运行"); break;
        case 3: busWriteLog(device, "I母单母线分段运行"); break;
        case 4: busWriteLog(device, "II母单母线分段运行"); break;
        case 5: busWriteLog(device, "双母线运行"); break;
        case 6: busWriteLog(device, "双母线I母分段运行"); break;
        case 7: busWriteLog(device, "双母线II母分段运行"); break;
        case 8: busWriteLog(device, "双母线双分段运行"); break;
        case 9: busWriteLog(device, "双母线并列运行"); break;
    }

   
}
