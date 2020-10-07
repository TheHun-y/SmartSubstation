
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

void busDeltaDiffRelay(BusDevice* bus, Device line[], int phase);
int busDeltaDiffAuxJudge(BusDevice* bus, Device line[], int phase, int busNo);
void busCTBreakRelay(BusDevice* bus, double Idc, double Ixc1, double Ixc2, int busNo, int phase);
extern void findLineConnectBus(BusDevice* bus, int lineNo, int* result);


void busDiffRelay(BusDevice* bus, Device line[], int phase) {

   
    // 变量本地化
    int modeMatrix[29][4];
    int i, j;
    // 比率制动系数：k1（高值），k2（低值）
    double kdc1, kdc2, kxc1, kxc2;
    kdc1 = 0.5; kdc2 = 0.3; kxc1 = 0.6; kxc2 = 0.5;
    // 差动启动整定值
    double Icdzd = bus->busDiffSetValue;

    // 大差、小差的差动电流及制动电流
    Phasor IdiffDc, IdiffXc[4];
    double IresDc, IresXc[4];
    // 判据标志
    int judgeFlagDc[2] = {0};
    int judgeFlagXc[8] = {0};

    // 死区保护标志
    int busBrkExitCalculateEnable[4];
    for (i = 0; i < 4; i++) {
        busBrkExitCalculateEnable[i] = bus->busBrkDeadZoneFlag[i];
    } 

    // 母联带路标志
    int bypassMode = bus->bypassBusMode;

    Phasor* temp ;
    //bus->testDC = phasorAbs(*temp);//tmp > phasorAbs(bus->busPhasor[21]) ? tmp:phasorAbs(bus->busPhasor[21]);


    IdiffDc.real = 0.0; 
    IdiffDc.img = 0.0;
    IresDc = 0.0;
    for (i = 0; i < 4; i++) {
        IdiffXc[i].real = 0.0; 
        IdiffXc[i].img = 0.0;
        IresXc[i] = 0.0;
    }

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            modeMatrix[i][j] = bus->busModeStatus[i][j];
        }
    }
    int cnt[4] = {0};
// bus->testDC = bus->busModeStatus[27][3];
    double tmp = bus->testDC;
    for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            if (modeMatrix[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][j], phase);
                // if (i == 1 && phase == 0 && j == 2) {
                //     bus->test_3 = phasorAbs(*temp);//line[1].phasor[3]);
                // }
                IdiffDc = phasorAdd(phasorNumMulti( modeMatrix[i][j], *temp), IdiffDc);
                IdiffXc[j] = phasorAdd(phasorNumMulti( modeMatrix[i][j], *temp), IdiffXc[j]);
                IresDc = IresDc + phasorAbs(*temp);
                IresXc[j] = IresXc[j] + phasorAbs(*temp);
                
                /*if (i == 1) {
                    bus->testDC = tmp > phasorAbs(*temp)? tmp: phasorAbs(*temp);
                }*/
                cnt[j]++;
            }
        }
    }
    
    //phasorAbs(*temp);

    //bus->testDC = modeMatrix[26][0];
    for (i = 24; i < 28; i++) {
        for (j = 0; j < 4; j++) {
            if (i == 24) { // 母联带路检查
                if ((bypassMode == 3 || bypassMode == 0) && modeMatrix[i][j] != 0 && busBrkExitCalculateEnable[i-24] == 0) { // 仅作母联
                    temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i-24, modeMatrix[i][j], phase);
                    // if (phase == 0 && j == 0) {
                    //     bus->test_3 = phasorAbs(*temp);//line[1].phasor[3]);
                    // }
                    IdiffXc[j] = phasorAdd(*temp, IdiffXc[j]);
                    IresXc[j] = IresXc[j] + phasorAbs(*temp);
                    cnt[j]++;
                } else if (j%2 == 0 && bypassMode == j/2+1) { // 作I母/II母旁路
                    temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i-24, modeMatrix[28][j], phase);
                    IdiffDc = phasorAdd(*temp, IdiffDc);
                    IdiffXc[j] = phasorAdd(*temp, IdiffXc[j]);
                    IresXc[j] = IresXc[j] + phasorAbs(*temp);
                }
            } else {
                if (modeMatrix[i][j] != 0 && busBrkExitCalculateEnable[i-24] == 0) {
        
                    temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i-24, modeMatrix[i][j], phase);
                    
                    IdiffXc[j] = phasorAdd(*temp, IdiffXc[j]);
                    IresXc[j] = IresXc[j] + phasorAbs(*temp); 
                    cnt[j]++;
                }
            }
        }        
    }


    // if (phase == 0) {
    //     bus->testDC = phasorAbs(IdiffXc[0]);//bus->busBrkDeadZoneFlag[0];//phasorAbs(IdiffXc[2]);//cnt[2];////

    //     //bus->testDC = bus->busDiffTripFlag[0];//phasorAbs(IdiffXc[2]);//cnt[2];////
    //     /*4*/bus->testPhasor[0] = phasorAbs(phasorSub(phasorAdd(line[21].phasor[3],phasorAdd(line[1].phasor[3], line[3].phasor[3])), bus->busPhasor[3]));//line[1].filterIma[0]+line[3].filterIma[0]+line[21].filterIma[0]-bus->busTieFiltIma[0];//line[1].filterIma[0];//phasorAbs(bus->busPhasor[3]);
    //     // /*3*/bus->testPhasor[1] = cnt[2];//phasorAbs(line[1].phasor[3]);//(bus->busPhasor[3]);//*findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, 0, modeMatrix[24][2], phase));
    //     // bus->testDC = IresDc;
    //     Phasor tmp = {1,0};
    //     //bus->testDC = phasorAngleDiff(line[1].phasor[3], tmp);//phasorAngleDiff(line[1].phasor[3], tmp);//cnt[2];////
    //     ///*4*/bus->testPhasor[0] = phasorAbs(bus->busPhasor[3]);//phasorAbs(line[3].phasor[3]);//phasorAbs(phasorSub(phasorAdd(line[21].phasor[3],phasorAdd(line[1].phasor[3], line[3].phasor[3])), bus->busPhasor[3]));//line[1].filterIma[0]+line[3].filterIma[0]+line[21].filterIma[0]-bus->busTieFiltIma[0];//line[1].filterIma[0];//phasorAbs(bus->busPhasor[3]);
    //     /*3*/bus->testPhasor[1] = busBrkExitCalculateEnable[0];//bus->busDiffDcTripTimeCount;//[2];//phasorAbs(line[1].phasor[3]);//cnt[2];//phasorAngleDiff(bus->busPhasor[3], tmp);//phasorAbs(line[1].phasor[3]);//(bus->busPhasor[3]);//*findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, 0, modeMatrix[24][2], phase));

        

    // }
    //tmp = bus->testPhasor[0];
    


    int connectBus[2];

    //CT断线
    for (i = 0; i < 4; i++) {
        if (bus->busBrkStatus[i*3] == 1 || bus->busBrkStatus[i*3+1] == 1 || bus->busBrkStatus[i*3+2] == 1) {
            findLineConnectBus(bus, i+24, connectBus);
            busCTBreakRelay(bus, phasorAbs(IdiffDc), phasorAbs(IdiffXc[connectBus[0]]), phasorAbs(IdiffXc[connectBus[1]]), i, phase);
        }
    }
    
    if (bus->busStartFlag[0] == 0 || bus->busDiffEnable == 0) {
        return;
    } 

    if ((phasorAbs(IdiffDc) > Icdzd) && (phasorAbs(IdiffDc) > kdc1*IresDc)) {
        // 大差高值
        judgeFlagDc[0] = 1;
        // busWriteLog(bus, "DC GZ");
    }
    if ((phasorAbs(IdiffDc) > Icdzd) && (phasorAbs(IdiffDc) > kdc2*IresDc)) {
        // 大差低值
        judgeFlagDc[1] = 1;
        // busWriteLog(bus, "DC DZ");
    }

    for (i = 0; i < 4; i++) {
        if ((phasorAbs(IdiffXc[i]) > Icdzd) && (phasorAbs(IdiffXc[i]) > kxc1*IresXc[i])) {
            // 小差高值
            judgeFlagXc[2*i] = 1;
            // busWriteLogWithPhase(bus, "%c段 XC GZ", i);
        }        
        if ((phasorAbs(IdiffXc[i]) > Icdzd) && (phasorAbs(IdiffXc[i]) > kxc2*IresXc[i])) {
            // 小差低值
            judgeFlagXc[2*i+1] = 1;
            // busWriteLogWithPhase(bus, "%c段 XC DZ", i);
        }
    }
    // 大差任一值动作记大差动作
    if ((judgeFlagDc[0] == 1) || (judgeFlagDc[1] == 1)) {
        bus->busDiffPhaseTripFlag[0][phase] = 1;
        // bus->busDiffTripTime[0] = bus->relayTime;
        // busWriteLog(bus, "母线常规大差动作（未经抗饱和）");
    } else {
        bus->busDiffPhaseTripFlag[0][phase] = 0;
    }
    // 大差高值和小差低值/大差低值和小差高值同时动作时记相应母线小差动作
    for (i = 0; i < 4; i++) {
        if ((judgeFlagDc[0] == 1) && (judgeFlagXc[i*2+1] == 1)) {          
            bus->busDiffPhaseTripFlag[i+1][phase] = 1;
            // busWriteLogWithPhase(bus, "%c段母线常规小差动作（未经抗饱和）",i);    
        }
        if ((judgeFlagDc[1] == 1) && (judgeFlagXc[i*2] == 1)) {
            bus->busDiffPhaseTripFlag[i+1][phase] = 1;
            // busWriteLogWithPhase(bus, "%c段母线常规小差动作（未经抗饱和）",i);
        }
    }
   

}

void busCTBreakRelay(BusDevice* bus, double Idc, double Ixc1, double Ixc2, int busNo, int phase) {

    // if (bus->voltBlockFlag[busNo] == 1) {
    //     return;
    // }
    double CTBreakSetTime = 0.3;
    char s[100];

    // double mem = bus->testPhasor[1];
    // if (busNo == 0) bus->testPhasor[1] = mem > Ixc1 ? mem : Ixc1;
    if (Idc > bus->busCTBreakBlockSetValue) {
        if (bus->busCTBreakStartTime[busNo][0][phase] == 0) {
            bus->busCTBreakStartTime[busNo][0][phase] = bus->relayTime;
        } else if (bus->relayTime - bus->busCTBreakStartTime[busNo][0][phase] > CTBreakSetTime) {
            bus->busCTBreakFlag[busNo][phase] = 1;
            bus->busCTBreakStartTime[busNo][0][phase] = 0;
            sprintf(s, "母线大差CT断线闭锁");
            busWriteLog(bus, s);
        }
    } else if (Ixc1 > bus->busCTBreakBlockSetValue && Ixc2 > bus->busCTBreakBlockSetValue) {
        if (bus->busCTBreakStartTime[busNo][1][phase] == 0) {
            bus->busCTBreakStartTime[busNo][1][phase] = bus->relayTime;
        } else if (bus->relayTime - bus->busCTBreakStartTime[busNo][1][phase] > CTBreakSetTime) {
            bus->busCTBreakFlag[busNo][phase] = 1;
            bus->busCTBreakStartTime[busNo][1][phase] = 0;
            sprintf(s, "%c号母联%c相CT断线闭锁", '1'+busNo, 'A'+phase);
            busWriteLog(bus, s);
        }
    } else if (Idc > bus->busCTBreakWarnSetValue) {
        bus->busCTBreakStartTime[busNo][0][phase] = 0;
        bus->busCTBreakStartTime[busNo][1][phase] = 0;
        if (bus->busCTBreakWarnStartTime[busNo][0][phase] == 0) {
            bus->busCTBreakWarnStartTime[busNo][0][phase] = bus->relayTime;
        } else if (bus->relayTime - bus->busCTBreakWarnStartTime[busNo][0][phase] > CTBreakSetTime) {
            bus->busCTBreakWarnFlag[busNo][phase] = 1;
            bus->busCTBreakWarnStartTime[busNo][0][phase] = 0;
            sprintf(s, "%c号母联%c相CT异常", '1'+busNo, 'A'+phase);
            busWriteLog(bus, s);
        }
    } else if (Ixc1 > bus->busCTBreakWarnSetValue && Ixc2 > bus->busCTBreakWarnSetValue) {
        bus->busCTBreakStartTime[busNo][0][phase] = 0;
        bus->busCTBreakStartTime[busNo][1][phase] = 0;
        if (bus->busCTBreakWarnStartTime[busNo][1][phase] == 0) {
            bus->busCTBreakWarnStartTime[busNo][1][phase] = bus->relayTime;
        } else if (bus->relayTime - bus->busCTBreakWarnStartTime[busNo][1][phase] > CTBreakSetTime) {
            bus->busCTBreakWarnFlag[busNo][phase] = 1;
            bus->busCTBreakWarnStartTime[busNo][1][phase] = 0;
            sprintf(s, "%c号母联%c相CT异常", '1'+busNo, 'A'+phase);
            busWriteLog(bus, s);
        }
    } else {
        bus->busCTBreakWarnStartTime[busNo][0][phase] = 0;
        bus->busCTBreakWarnStartTime[busNo][1][phase] = 0;
        bus->busCTBreakStartTime[busNo][0][phase] = 0;
        bus->busCTBreakStartTime[busNo][1][phase] = 0;
    }
    // bus->testPhasor[1] = bus->busCTBreakStartTime[0][0][2];
}


void busDeltaDiffRelay(BusDevice* bus, Device line[], int phase) {
    // 工频变化量差动部分整定值
    double kdelta,deltaIcdzd, kdc1, kdc2, kxc1, kxc2;
    kdelta = 0.9; deltaIcdzd = 0.5;
    kdc1 = 0.65; kxc1 = 0.65; kdc2 = 0.3; kxc2 = 0.5;
    // 母联带路标志
    int bypassMode = bus->bypassBusMode;

    Phasor* mem, *temp;
    int i, j;
    int modeMatrix[29][4];
    Phasor IdiffDc, IdiffDcMem, IdiffXc[4], IdiffXcMem[4];
    double ampDc, ampXc[4], IresDc, IresXc[4];
    double max = 0;
    // 判据标志
    int judgeFlagDc[2];
    int judgeFlagXc[8];
    int judgeFlagAux[4];

    // 死区保护标志
    int busBrkExitCalculateEnable[4];
    for (i = 0; i < 4; i++) {
        busBrkExitCalculateEnable[i] = bus->busBrkDeadZoneFlag[i];
    }

    IdiffDc.real = 0.0; IdiffDc.img = 0.0; IdiffDcMem.real = 0.0; IdiffDcMem.img = 0.0; 
    IresDc = 0.0;

    for (i = 0; i < 4; i++) {
        IdiffXc[i].real = 0.0; 
        IdiffXc[i].img = 0.0;
        IdiffXcMem[i].real = 0.0; 
        IdiffXcMem[i].img = 0.0;
        IresXc[i] = 0.0;
    }

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            modeMatrix[i][j] = bus->busModeStatus[i][j];
        }
    }

    for (i = 0; i < 4; i++) {
        judgeFlagAux[i] = busDeltaDiffAuxJudge(bus, line, phase, i);
    }

    for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            if (modeMatrix[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][j], phase);
                IdiffDc = phasorAdd(phasorNumMulti(modeMatrix[i][j],*temp), IdiffDc);
                IdiffXc[j] = phasorAdd(phasorNumMulti(modeMatrix[i][j],*temp), IdiffXc[j]);
                IresDc = IresDc + phasorAbs(phasorSub(*temp, *findLineMemory(bus, line, i, phase, modeMatrix[i][j])));
                IresXc[j] = IresXc[j] + phasorAbs(phasorSub(*temp, *findLineMemory(bus, line, i, phase, modeMatrix[i][j])));
                IdiffDcMem = phasorAdd(phasorNumMulti(modeMatrix[i][j],*findLineMemory(bus, line, i, phase, modeMatrix[i][j])), IdiffDcMem);
                IdiffXcMem[j] = phasorAdd(phasorNumMulti(modeMatrix[i][j],*findLineMemory(bus, line, i, phase, modeMatrix[i][j])), IdiffXcMem[j]);
                /*{
                        if (phase == 1 && j == 1 && i == 3) {
                            double tmp = bus->testDC;
                            //bus->testDC = tmp > phasorAbs(phasorNumMulti(modeMatrix[28][0], *findBusMemory(bus, phase, 0)))? tmp:phasorAbs(phasorNumMulti(modeMatrix[28][0], *findBusMemory(bus, phase, 0)));
                            bus->testDC = tmp > phasorAbs(phasorNumMulti(modeMatrix[i][j],*findLineMemory(bus, line, i, phase, modeMatrix[i][j]))) ? tmp : phasorAbs(phasorNumMulti(modeMatrix[i][j],*findLineMemory(bus, line, i, phase, modeMatrix[i][j])));
                        }
                }*/
            }
        }
    }

   

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            if (i == 0) {
                    if ((bypassMode == 3 || bypassMode == 0) && busBrkExitCalculateEnable[i] == 0) { // 仅作母联
                        if(modeMatrix[i+24][j] != 0) {
                            temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i, modeMatrix[i+24][j], phase);
                            IdiffXc[j] = phasorAdd(*temp, IdiffXc[j]);
                            IdiffXcMem[j] = phasorAdd(phasorNumMulti(modeMatrix[i+24][j], *findBusMemory(bus, phase, i)), IdiffXcMem[j]);
                            IresXc[j] = IresXc[j] + phasorAbs(phasorSub(*temp, phasorNumMulti(modeMatrix[i+24][j], *findBusMemory(bus, phase, i))));
                        }
                     } else if (j%2 == 0 && bypassMode == j/2+1 && modeMatrix[28][j] != 0) { // 作I母/II母旁路
                        temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i, modeMatrix[28][j], phase);
                        IdiffXc[j] = phasorAdd(*temp, IdiffXc[j]);
                        IdiffXcMem[j] = phasorAdd(phasorNumMulti(modeMatrix[28][j], *findBusMemory(bus, phase, i)), IdiffXcMem[j]);
                        IresXc[j] = IresXc[j] + phasorAbs(phasorSub(*temp, phasorNumMulti(modeMatrix[28][j], *findBusMemory(bus, phase, i))));
                     }
            }  else {
                if (modeMatrix[i+24][j] != 0 && busBrkExitCalculateEnable[i] == 0) {
                    temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i, modeMatrix[i+24][j], phase);
                    /*{
                        if (phase == 1 && j == 1 && i == 2) {
                            double tmp = bus->testPhasor[0];
                            //bus->testDC = tmp > phasorAbs(phasorNumMulti(modeMatrix[28][0], *findBusMemory(bus, phase, 0)))? tmp:phasorAbs(phasorNumMulti(modeMatrix[28][0], *findBusMemory(bus, phase, 0)));
                            bus->testPhasor[0] = tmp > phasorAbs(phasorNumMulti(modeMatrix[i+24][j], *findBusMemory(bus, phase, i))) ? tmp : phasorAbs(phasorNumMulti(modeMatrix[i+24][j], *findBusMemory(bus, phase, i)));
                        }
                    }*/
                    IdiffXc[j] = phasorAdd(*temp, IdiffXc[j]);
                    IdiffXcMem[j] = phasorAdd(phasorNumMulti(modeMatrix[i+24][j], *findBusMemory(bus, phase, i)), IdiffXcMem[j]);
                    IresXc[j] = IresXc[j] + phasorAbs(phasorSub(*temp, phasorNumMulti(modeMatrix[i+24][j], *findBusMemory(bus, phase, i))));
                }
            }
        }        
    }
    
    
    ampDc = phasorAbs(phasorSub(IdiffDc, IdiffDcMem));
    
    for (i = 0; i < 4; i++) {
        ampXc[i] = phasorAbs(phasorSub(IdiffXc[i], IdiffXcMem[i]));
    }

    
    /*{
         if (phase == 1) {
             double tmp = bus->testDC;
            //bus->testDC = tmp > phasorAbs(phasorNumMulti(modeMatrix[28][0], *findBusMemory(bus, phase, 0)))? tmp:phasorAbs(phasorNumMulti(modeMatrix[28][0], *findBusMemory(bus, phase, 0)));
            bus->testDC = tmp > phasorAbs(IdiffXc[1]) ? tmp : phasorAbs(IdiffXc[1]);
            int k = 0;
            for (k = 0; k < 29; k++) {
                if (modeMatrix[k][1] != 0) {
                    tmp++;
                }
            }
            bus->testDC = tmp;

            tmp = bus->testPhasor[0];
            //bus->testPhasor[0] = tmp > phasorAbs(phasorNumMulti(1, *findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, 0, modeMatrix[28][0], phase)))? tmp:phasorAbs(phasorNumMulti(1, *findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, 0, modeMatrix[28][0], phase)));
            bus->testPhasor[0] = tmp > phasorAbs(IdiffXcMem[1]) ? tmp : phasorAbs(IdiffXcMem[1]);
            //bus->testPhasor[0] = tmp > ampXc[1] ? tmp : ampXc[1];
            //bus->testPhasor[0] = modeMatrix[3][1];

            tmp = bus->testPhasor[1];
            bus->testPhasor[1] = tmp > IresXc[1]? tmp:IresXc[1];
            //bus->testPhasor[1] = modeMatrix[26][1];

         }
    }*/
    
    if (bus->busStartFlag[0] == 0 || bus->busDiffEnable == 0) {
        return;
    } 

    if ((ampDc > deltaIcdzd + kdelta*ampDc) && (ampDc > kdc2*IresDc)) {
        // 大差高值
        judgeFlagDc[0] = 1;
        //busWriteLogWithPhase(bus, "%c段 DELTA DC GZ",-1);
    }
    if ((ampDc > deltaIcdzd + kdelta*ampDc) && (ampDc > kdc1*IresDc)) {
        // 大差低值
        judgeFlagDc[1] = 1;
        //busWriteLogWithPhase(bus, "%c段 DELTA DC DZ",-1);
    }

    for (i = 0; i < 4; i++) {
        if ((ampXc[i] > deltaIcdzd + kdelta*ampXc[i]) && (ampXc[i] > kxc2*IresXc[i])) {
            // 小差高值
            judgeFlagXc[2*i] = 1;
            //busWriteLogWithPhase(bus, "%c段 DELTA XC GZ",i);
        }        
        if ((ampXc[i] > deltaIcdzd + kdelta*ampXc[i]) && (ampXc[i] > kxc1*IresXc[i])) {
            // 小差低值
            judgeFlagXc[2*i+1] = 1;
            //busWriteLogWithPhase(bus, "%c段 DELTA XC DZ",i);
        }
    }
    // 大差高值和小差低值/大差低值和小差高值同时动作
    for (i = 0; i < 4; i++) {
        if ((judgeFlagAux[i] == 1) && (judgeFlagDc[0] == 1) && (judgeFlagXc[i*2+1] == 1)) {
            bus->busDeltaDiffPhaseTripFlag[i][phase] = 1;
            // bus->busDeltaDiffTripTime[i] = bus->relayTime;
            //busWriteLogWithPhase(bus, "%c段母线变化量小差动作（未经抗饱和）",i);
        }
        if ((judgeFlagAux[i] == 1) && (judgeFlagDc[1] == 1) && (judgeFlagXc[i*2] == 1)) {
            bus->busDeltaDiffPhaseTripFlag[i][phase] = 1;
            // bus->busDeltaDiffTripTime[i] = bus->relayTime;
            //busWriteLogWithPhase(bus, "%c段母线变化量小差动作（未经抗饱和）",i);
        }
    }
   
}

int busDeltaDiffAuxJudge(BusDevice* bus, Device line[], int phase, int busNo) {
    // 配合工频变化量差动的0.2常规比率差动整定值
    double k = 0.2;
    // 变量本地化
    int modeMatrix[29][4];
    int i, j;
    double Icdzd = bus->busDiffSetValue;

    // 大差、小差的差动电流及制动电流
    Phasor IdiffDc, IdiffXc, *temp;
    double IresDc, IresXc;

    // 母联带路标志
    int bypassMode = bus->bypassBusMode;

    // 死区保护标志
    int busBrkExitCalculateEnable[4];
    for (i = 0; i < 4; i++) {
        busBrkExitCalculateEnable[i] = bus->busBrkDeadZoneFlag[i];
    }

    IdiffDc.real = 0.0; IdiffDc.img = 0.0; IdiffXc.real = 0.0; IdiffXc.img = 0.0;
    IresDc = 0.0; IresXc = 0.0;

    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++) {
            modeMatrix[i][j] = bus->busModeStatus[i][j];
        }
    }

    for (i = 0; i < 24; i++) {
        for (j = 0; j < 4; j++) {
            if (modeMatrix[i][j] != 0) {
                temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][j], phase);
                IdiffDc = phasorAdd(phasorNumMulti(modeMatrix[i][j],*temp), IdiffDc);
                IresDc = IresDc + phasorAbs(*temp);
            }
        }
    }

    for (i = 0; i < 24; i++) {
        if (modeMatrix[i][busNo] != 0) {
            temp = findLineRelayTimePhasor(line[i].busSynFlag, &line[i], modeMatrix[i][busNo], phase);
            IdiffXc = phasorAdd(phasorNumMulti(modeMatrix[i][busNo],*temp), IdiffXc);
            IresXc = IresXc + phasorAbs(*temp);
        }
    }


    for (i = 24; i < 28; i++) {
        if (i == 24) { // 母联带路检查
                     if ((bypassMode == 3 || bypassMode == 0) && busBrkExitCalculateEnable[i-24] == 0 && modeMatrix[i][busNo] != 0) { // 仅作母联
                        temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i-24, modeMatrix[i][busNo], phase);
                        IdiffXc = phasorAdd(*temp, IdiffXc);
                        IresXc = IresXc + phasorAbs(*temp);
                     } else if (busNo%2 == 0 && bypassMode == busNo/2+1) { // 作I母/II母旁路
                        temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i-24, modeMatrix[28][busNo], phase);
                        IdiffXc = phasorAdd(*temp, IdiffXc);
                        IresXc = IresXc + phasorAbs(*temp);
                     }
                    
        } else {
            if (modeMatrix[i][busNo] != 0 && busBrkExitCalculateEnable[i-24] == 0) {
                
                    temp = findBusRelayTimePhasor(bus->busDiffDelayFlag, bus, i-24, modeMatrix[i][busNo], phase);
                    IdiffXc = phasorAdd(*temp, IdiffXc);
                    IresXc = IresXc + phasorAbs(*temp);
                
            }       
        }
                
    }

    if ((phasorAbs(IdiffDc) > Icdzd) && (phasorAbs(IdiffDc) > k*IresDc) && (phasorAbs(IdiffXc) > Icdzd) && (phasorAbs(IdiffXc) > k*IresXc)) {
        // busWriteLogWithPhase(bus, "%c段母线变化量辅助判据动作",busNo);
        return 1;
    }

    return 0;
}

