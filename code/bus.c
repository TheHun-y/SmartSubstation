// 母线保护主文件
// Created by Nazure on 2020/2/14.
//
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>
// 母线保护算法较为简单, 保护逻辑集中在该文件

extern void busDiffRelay(BusDevice* bus, Device line[], int phase);
extern void busModeRecognize(BusDevice* device);
extern void busTripGenerate(BusDevice* bus, Device line[]);
extern void busDeltaDiffRelay(BusDevice* bus, Device line[], int phase);
extern void busStarter(BusDevice* bus, Device line[]);
extern void busBrkStatusMemory(BusDevice* bus);
extern int voltageBlockJudge(BusDevice* bus, int i);
extern void chargeBlockJudge(BusDevice* bus);
extern void antiSat1Judge(BusDevice* bus, Device line[]);
extern void antiSat2Judge(BusDevice* bus, Device line[]);
extern void backupDiffRelay(BusDevice* bus);

extern void busDeadZoneRelay(BusDevice* bus);
extern void busBrkFailureRelay(BusDevice* bus);
extern void busOverCurrentRelay(BusDevice* bus);
extern void busOpenphaseRelay(BusDevice* bus);
void recordBusPhasor(BusDevice* device);
void busDiffDcTripTimeCalculate(BusDevice* bus);
void busBranchFailureRelay(BusDevice* bus, Device* line, Device* lineRelay/*,transDevice* trans*/);
extern void busPTBreakRelay(BusDevice* bus, Device* line);

void bus(BusDevice* bus, Device line[], Device lineRelay[]) {

    int phase = 0;
    int i, j;
    int busDiffEnable = bus->busDiffEnable; // 差动保护控制字
    // 电压闭锁元件标志-A相: 0表示开放，1表示闭锁
    int busDiffDcFlag = bus->busDiffTripFlag[0];

    // 饱和闭锁标志：0-未饱和，开放动作；1-饱和，闭锁保护
    int antiSat1Flag = 0;
    int antiSat2Flag = 0;

    // 将采样值存入瞬时值数组
    busSample2inst(bus);

    // 瞬时值滤波后存入并更新滤波后数组
    busDataFilter(bus);

    //记忆前一个采样点的相量计算值（10*0.02/48）
    recordBusPhasor(bus);

    // 利用滤波后数据计算12通道相量,存入phasor数组
    toPhasorForBus(bus);
    //bus->testDC = phasorAbs(bus->busPhasor[15]);

    if (bus->time > 0.02) {
        busLineSynCheck(bus, line);
    }
    //bus->testPhasor[0] = phasorAbs(bus->busPhasor[15]);
    // 识别母线运行方式
    busModeRecognize(bus);
    
    if (bus->time < 0.2) {
        // 等待仿真进入稳定状态
        for (i = 0; i < 5; i++) {
            bus->antiSat2Flag[i] = 1;
            bus->antiSat1Flag[i] = 1;
        }
        return;
    }

    
    //antiSat1Judge(bus, line);
    // 启动判断
    busStarter(bus, line);
    //bus->testDC = phasorAbs(bus->busPhasor[15]);
    busDeadZoneRelay(bus);
    busPTBreakRelay(bus, line);

    if (bus->bypassBusMode == 0 || bus->bypassBusMode == 3) {
        //busDeadZoneRelay(bus);
        //busFailureRelay(bus);
    }
   

    if ((bus->busStartFlag[0] == 1) && (bus->relayTime > 0.02)) {
        // 充电闭锁判断：第一次调用时不判断
        if (bus->relayTime > 0.02/(POINTS)*10) {
            chargeBlockJudge(bus);
            
        }

        // 电压闭锁判断
        for (i = 0; i < 4; i++) {
            bus->voltBlockFlag[i] = voltageBlockJudge(bus, i);
            switch (bus->voltBlockFlag[i]) {
                case 0: busWriteLogWithPhase(bus, "%c段母线电压闭锁开放保护动作",i); break;
                case 1: busWriteLogWithPhase(bus, "%c段母线电压闭锁保护动作",i); break;
            }
        }
        
        busDiffDcTripTimeCalculate(bus);
    
        // 抗饱和判据:当常规比率差动连续动作500ms不返回时，不再进行抗饱和判断
        if (bus->busDiffDcTripTimeCount < 0.5) {
           
            if (bus->relayTime - bus->busStartTime[0] < 0.002) {
                antiSat1Judge(bus, line);
            } else {
                antiSat2Judge(bus, line);
            }
        } else {
            for (i = 0; i < 5; i++) {
                bus->antiSat1Flag[i] = 0;
                bus->antiSat2Flag[i] = 0;
                // if(i == 0) {
                // busWriteLog(bus, "母线大差抗饱和退出");
                // } else {
                    // busWriteLogWithPhase(bus, "%c段母线抗饱和退出", i-1);
                // }
            }
            busWriteLog(bus, "母差抗饱和退出");
            bus->busDiffDcTripTimeCount = 0;
        }
        
        // 记录抗饱和大差动作时间:条件是两个抗饱和元件必须都开放，且大差动作标志位是上升沿
        if (/*(bus->antiSat1Flag[0] == 0) &&*/ (bus->antiSat2Flag[0] == 0) && (bus->busDiffTripFlag[0] == 1) && (busDiffDcFlag == 0)) {
            bus->busDiffTripTime[0] = bus->relayTime; 
            busWriteLog(bus, "母线大差抗饱和动作");
        }

        // 后备保护
        backupDiffRelay(bus); 

        // 母联/分段失灵保护
        busBrkFailureRelay(bus);


    }
    // 这里的差动保护中包含CT断线的判断，因此在内部通过判断装置启动标志位来决定差动是否出口，不在调用时判断启动标志
    // if (bus->relayTime - bus->busStartTime[0] > 0.02 && bus->busStartTime[0] != 0) {
        for (phase = 0; phase < 3; phase++) {
            busDiffRelay(bus, line, phase);
            busDeltaDiffRelay(bus, line, phase);
        }
        // 整合标志位及动作时间
        for (i = 0; i < 5; i++) {
            bus->busDiffTripFlag[i] = (bus->busDiffPhaseTripFlag[i][0] || bus->busDiffPhaseTripFlag[i][1] || bus->busDiffPhaseTripFlag[i][2]);
            if (bus->busDiffTripFlag[i] && bus->busDiffTripTime[i] == 0.0) {
                bus->busDiffTripTime[i] = bus->relayTime;
            }
            if (i) {
                bus->busDeltaDiffTripFlag[i-1] = (bus->busDeltaDiffPhaseTripFlag[i-1][0] || bus->busDeltaDiffPhaseTripFlag[i-1][1] || bus->busDeltaDiffPhaseTripFlag[i-1][2]);
                if (bus->busDeltaDiffTripFlag[i-1] && bus->busDeltaDiffTripTime[i-1] == 0.0) {
                    bus->busDeltaDiffTripTime[i-1] = bus->relayTime;
                }
            }
        }
    // }
    bus->testDC = bus->busDiffTripFlag[0];


    // 断路器失灵保护
    //busBranchFailureRelay(bus, line, lineRelay/*, transRelay*/);
    
    
    // 跳闸判断
    busTripGenerate(bus, line);

    // 断路器状态记忆
    busBrkStatusMemory(bus);

   // 非全相保护
    busOpenphaseRelay(bus);

    // 充电过流保护
    busOverCurrentRelay(bus);   

}

void busDiffDcTripTimeCalculate(BusDevice* bus) {
    // 实现功能：记录大差比率元件**连续**动作的时间
    if (bus->busDiffTripFlag[0] == 1) {
        bus->busDiffDcTripTimeCount = bus->relayTime - bus->busDiffTripTime[0];
    } else {
        bus->busDiffDcTripTimeCount = 0;
    }

}

void recordBusPhasor(BusDevice* device) {
    int i;
    for (i = 0; i < 48; i++) {
        device->busPrePhasor[i] = device->busPhasor[i];
    }
}

void resetAllBusFlag(BusDevice* bus) {
    int i = 0;
    for (i = 0; i < 5; i++) {
        bus->busDiffTripFlag[i] = 0;
    }
}








