#include "..\\code\\dataStruct.h"
#include "..\\code\common.h"

/**
 * RelayMain.c
 * 所有保护装置的入口函数
 */

#define busNum_500kV 3

Device linesM[10];
Device linesN[10];
Device Bus2_sampleLines[24]; //采样装置

tranDevice trans[1];

BusDevice buses[5];
Device buses_500kV_sampleBrk[36];
Device buses_500kV_relayBrk[36];
BusDevice buses_500kV[busNum_500kV];
meaDevice mea_500kV[10];

int globalInitFlag = 0;// 全局初始化标志
char logDirName[STRING_LENGTH];

// 声明各类保护函数
extern void line(Device*); // 线路保护
extern void bus(BusDevice* bus, Device line[], Device lineRelay[]);
extern void tran(tranDevice*);
// int _globalBusflag = 0;


// 注意采用GFORTRAN, C函数名需要多一个下划线

void line1m_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesM[0], &linesN[0], "line1M", *time, *deviceEnable, port1, port2, tripSignal);
}

void line1n_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesN[0], &linesM[0], "line1N", *time, *deviceEnable, port1, port2, tripSignal);
}


void line2m_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesM[1], &linesN[1],"line2M", *time, *deviceEnable, port1, port2, tripSignal);
}

void line2n_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesN[1], &linesM[1],"line2N", *time, *deviceEnable, port1, port2, tripSignal);
}

void line3m_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesM[2], &linesN[2],"line3M", *time, *deviceEnable, port1, port2, tripSignal);
}

void line4m_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesM[3], &linesN[3],"line4M", *time, *deviceEnable, port1, port2, tripSignal);
}

void line5m_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&linesM[4], &linesN[4],"line5M", *time, *deviceEnable, port1, port2, tripSignal);
}

/*void sampleLine1_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&sampleLines[0], &linesN[0],"line1M", *time, *deviceEnable, port1, port2, tripSignal);
}*/


void sample_220kv_line1_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[0], &linesN[4],"sample_220_brk1", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line2_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[1], &linesN[4],"sample_220_brk2", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line3_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[2], &linesN[4],"sample_220_brk3", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line4_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[3], &linesN[4],"sample_220_brk4", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line5_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[4], &linesN[4],"sample_220_brk5", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line6_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[5], &linesN[4],"sample_220_brk6", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line7_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[6], &linesN[4],"sample_220_brk7", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line8_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[7], &linesN[4],"sample_220_brk8", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_line9_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[8], &linesN[4],"sample_220_brk9", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_trans1_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[20], &linesN[4],"sample_220_t1", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_trans2_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[21], &linesN[4],"sample_220_t2", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_220kv_trans3_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&Bus2_sampleLines[22], &linesN[4],"sample_220_t3", *time, *deviceEnable, port1, port2, tripSignal);
}

void bus_220kv_1_(double* time, int* deviceEnable,double* port1, double* port2,double* port3, double* port4, double* port5,double* port6, double* port7, double* port8, double* port9, double* port10, double* port11, double* port12, double* port13, double* port14, double* port15, double* tripSignal) {
    globalInit();
    busLinkSimulation(&buses[0], Bus2_sampleLines, linesM, "220kV_bus1", *time, *deviceEnable, port1, port2, port3, port4, port5, port6, port7, port8, port9, port10, port11, port12, port13, port14, port15, tripSignal);
}


// 500kV bus

void sample_500kv_brk1_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[0], &linesN[4],"sample_500_brk1", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk2_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[1], &linesN[4],"sample_500_brk2", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk3_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[2], &linesN[4],"sample_500_brk3", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk4_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[3], &linesN[4],"sample_500_brk4", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk5_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[4], &linesN[4],"sample_500_brk5", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk6_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[5], &linesN[4],"sample_500_brk6", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk7_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[6], &linesN[4],"sample_500_brk7", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk8_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[7], &linesN[4],"sample_500_brk8", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk9_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[8], &linesN[4],"sample_500_brk9", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk10_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[9], &linesN[4],"sample_500_brk10", *time, *deviceEnable, port1, port2, tripSignal);
}
void sample_500kv_brk11_(double* time, int* deviceEnable, double* port1, double* port2, double* tripSignal) {
    globalInit();
    lineLinkSimulation(&buses_500kV_sampleBrk[10], &linesN[4],"sample_500_brk11", *time, *deviceEnable, port1, port2, tripSignal);
}



void bus1_500kv_(double* time, int* deviceEnable) {
    globalInit();
    busLinkSimulation_500kV(&buses_500kV[0], buses_500kV_sampleBrk, buses_500kV_relayBrk, "500kV_bus1", *time, *deviceEnable);
}


void tran1_(double* time, int* deviceEnable, double* tport1, double* tport2, double* tranTripSignal){
    globalInit();
    tranLinkSimulation(&trans[0], "tran1", *time, *deviceEnable, tport1, tport2, tranTripSignal);

}

// 测控
void mea_500kv_1_(double* time, int* deviceEnable, double* port1, double* port2, int* remoteControl, int* tripSignals) {
    globalInit();
    meaLinkSimulation(&mea_500kV[0], "mea_500kV_1", *time, *deviceEnable, port1, port2, remoteControl, tripSignals);
}











