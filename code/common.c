#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <direct.h>
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

// 声明各类保护函数
extern Device linesM[10];
extern void line(Device*); // 线路保护
extern void recordPhasor(Device* device);
extern void recordMemoryUI(Device* device);
extern void bus(BusDevice*, Device line[], Device lineRelay[]); // 母线保护
extern void bus_500kV(BusDevice* device, Device* sampleBrk, Device* brkRelay);
extern void busBrkFailureJudgeAux_500kV(Device* device);
extern char logDirName[STRING_LENGTH];
extern int globalInitFlag;

extern void tran(tranDevice*);
extern tranDevice trans[1];



/**
 * 全局初始化函数
 * 主要负责创建文件系统
 */
void globalInit() {
    if (globalInitFlag == 0) {
        // 根据系统时间确定日志文件名
        time_t t;
        struct tm * lt;
        time (&t);
        lt = localtime (&t);//转为时间结构

        sprintf(logDirName, "..\\\\log\\\\%04d-%02d-%02d_%02d-%02d",
                (lt->tm_year+1900), lt->tm_mon+1, lt->tm_mday,
                lt->tm_hour, lt->tm_min);

        // 根据时间, 在log下创建文件夹
        mkdir(logDirName);
        globalInitFlag = 1;
    }
}

/**
 * 交换机排队及延时
 */
 void switchRelay(Device* device, double timeD, const double* port1, const double* port2) {

    double min1 = device->switch1DelayMin;
    double max1 = device->switch1DelayMax;
    double min2 = device->switch2DelayMin;
    double max2 = device->switch2DelayMax;

    if (max1 - min1 < -0.0001) {
        writeErrorLog(device, "交换机A延时参数错误!");
    }
    if (max2 - min2 < -0.001) {
        writeErrorLog(device , "交换机B延时参数错误");
    }

    double random1 = min1 + rand()%200000 / 200000.0 * (max1-min1);

    // double random1 = 0.1;
    double random2 = 0; // 固定延时为3个仿真步长左右
    int i;

    int q1 = device->queueLength1;
    int q2 = device->queueLength2;

    // 维护指示队列长度的计数器qLength
    // 新到达的数据包放到switchQueue[qLength]位置
    // 仿真10次, 执行一次采样函数, 更新交换机队列
    if (upTo5(device)) {
        if (q1 < QUEUE_LENGTH) {
            // 当队列排满时, 采样值不能进入队列, 模拟丢包
            device->switchQueue1[q1].delayTime = timeD + random1;
            for (i = 0; i < 10; ++i) {
                device->switchQueue1[q1].frame[i] = port1[i];
            }
            if (q1 <= QUEUE_LENGTH-2) {
                device->queueLength1++;
            }
        } else {
            writeLog(device, "信道阻塞, 交换机A丢包");
        }

        if (q2 < QUEUE_LENGTH) {
            device->switchQueue2[q2].delayTime = timeD + random2;
            for (i = 0; i < 9; ++i) {
                device->switchQueue2[q2].frame[i] = port2[i];
            }
            if (q2 <= QUEUE_LENGTH-2) {
                device->queueLength2++;
            }
        } else {
            writeLog(device, "信道阻塞, 交换机B丢包");
        }
    }


    // 每次仿真都执行
    // 如果判定需要输出, 从队头(0索引位置)取数, 更新队列, 更新交换机端口switchPort
    if (device->queueLength1 > 0 && device->switchQueue1[0].delayTime <= timeD) {
        // 延时到达, 可以输出
        for (i = 0; i < 10; ++i) {
            device->switchPort1[i] = device->switchQueue1[0].frame[i];
        }
        // 更新队列
        for (i = 0; i < device->queueLength1; i++) {
            device->switchQueue1[i] = device->switchQueue1[i+1];
        }
        device->switchQueue1[device->queueLength1].delayTime = MAX_VALUE;
        device->queueLength1--;
    }

    if (device->queueLength2 > 0 && device->switchQueue2[0].delayTime <= timeD) {
        // 延时到达, 可以输出
        for (i = 0; i < 9; ++i) {
            device->switchPort2[i] = device->switchQueue2[0].frame[i];
        }
        // 更新队列
        for (i = 0; i < device->queueLength2; i++) {
            device->switchQueue2[i] = device->switchQueue2[i+1];
        }
        device->switchQueue2[device->queueLength2].delayTime = MAX_VALUE;
        device->queueLength2--;
    }
 }

/**
 * 仿真链接函数
 * 综合以下功能
 * 初始化/仿真步长设置/跳闸指令
 */
void lineLinkSimulation(Device* device, Device* deviceN, char* deviceName, double time, int deviceEnable, double* port1, double* port2, double* tripSignal) {
    // 设置整定值
    if (notYet(device, "设置保护装置名及保护定值")) {
        deviceInit(device, deviceName, deviceEnable, 'L');
    }

    switchRelay(device, time, port1, port2);

    // 只有装置启用情况下才进行计算
    // 仿真程序跑10次, 进行一次采样和保护计算
    if (deviceEnable == 1 && upTo10A(device) == 1) {
        sample(device, time, port1, port2);
        line(device);
        device->currentDiffUnionFlag[0] = deviceN->currentDiffSelfTripFlag[0] && !(deviceN->CTbreakBlockDiffEnable && deviceN->CTBreakFlag);
        device->currentDiffUnionFlag[1] = deviceN->currentDiffSelfTripFlag[1] && !(deviceN->CTbreakBlockDiffEnable && deviceN->CTBreakFlag);
        device->currentDiffUnionFlag[2] = deviceN->currentDiffSelfTripFlag[2] && !(deviceN->CTbreakBlockDiffEnable && deviceN->CTBreakFlag);

        /*tripSignal[0] = device->instImb[0];//device->tripFlag[0];
        //tripSignal[0] = device->CTBreakTime[0];
        tripSignal[1] = device->filterImb[0];//device->tripFlag[1];
    
        tripSignal[2] = device->instVmb[0];//device->tripFlag[2];
        return;*/
    } else if (deviceEnable == 220 && upTo10A(device) == 1) {
        // 采样装置
        sample(device, time, port1, port2);//device->switchPort1, device->switchPort2);
        // 将采样值存入瞬时值数组
        sample2inst(device);

        // 瞬时值滤波后存入并更新滤波后数组
        dataFilter(device);
        // Phasor g = {0,1};
        // device->test[0] = (device->phasor[3]).real;
        // device->test[1] = phasorAngleDiff(device->phasor[3],g);

        //记忆前一个采样点的相量计算值（10*0.02/48）
        recordPhasor(device);

        // 利用滤波后数据计算12通道相量,存入phasor数组
        toPhasor(device);

        
        
        if (device->startFlag == 0) {
            // 未收到母线保护启动的指令时一直记忆，启动后停在启动前三个周波起的记忆量
            recordMemoryUI(device);
        } else if (device->busSynFlag != 1) {
            recordMemoryUI(device);
        }
    } else if (deviceEnable == 500 && upTo10A(device) == 1) {
        sample(device, time, port1, port2);
        // 将采样值存入瞬时值数组
        sample2inst(device);

        // 瞬时值滤波后存入并更新滤波后数组
        dataFilter(device);

        //记忆前一个采样点的相量计算值（10*0.02/48）
        recordPhasor(device);

        // 利用滤波后数据计算12通道相量,存入phasor数组
        toPhasor(device);
        if (device->startFlag == 0) {
            // 未收到母线保护启动的指令时一直记忆，启动后停在启动前三个周波起的记忆量
            recordMemoryUI(device);
        } else if (device->busSynFlag != 1) {
            recordMemoryUI(device);
        }

        if (device->time < 0.22) {

        } else {
            if (device->memoryI0Flag == 0){
            device->memoryI0Flag = 1;
            device->memory3I0 = phasorAbs(phasorSeq(device->phasor[3], device->phasor[4], device->phasor[5], 0));
            }
            // busBrkFailureJudgeAux_500kV(device);
        }
    }

    // 结合手动与断路器状态进行出口置位
    //finalOutput(device);

    // 结果输出
    //tripSignal[0] = device->tripSignalOutput[0];
    tripSignal[0] = device->tripFlag[0];//test[0];//filterVmb[0];//test[0];//filterImc[0];//tripFlag[0];
    //tripSignal[0] = device->CTBreakTime[0];
    tripSignal[1] = device->tripFlag[1];//test[1];//tripFlag[1];
    
    tripSignal[2] = device->tripFlag[2];//test[2];//
    //tripSignal[2] = device->test[1];//tripFlag[2];
    //tripSignal[2] = device->test[1];

}


/**
 * 母线链接函数
 */
int _globalBusFlag = 0;
void busLinkSimulation(BusDevice* device, Device line[], Device lineRelay[], char* deviceName, double time, int deviceEnable, double* port1, double* port2,double* port3, double* port4, double* port5,double* port6, double* port7, double* port8, double* port9, double* port10, double* port11, double* port12, double* port13, double* port14, double* port15, double* tripSignal) {
    // 设置整定值
   if (busNotYet(device, "设置保护装置名及保护定值")) {
        busDeviceInit(device, deviceName, deviceEnable, 'B');
    }


    // busWriteLog(device, "测试");
    // 只有装置启用情况下才进行计算
    // 仿真程序跑10次, 进行一次采样和保护计算
    if (device->deviceEnable == 1 && busUpTo10A(device) == 1) {
        busStatusRenew(device, time, port1, port2, port3, port4, port5, port6, port7, port8, port9, port10, port11);
        busSample(device, port12, port13, port14, port15);
        bus(device, line, lineRelay);
    }
    /*char fileName[100];
    sprintf(fileName, "%s%s%s", "..\\test\\", deviceName, ".txt");

    FILE *fi = fopen(fileName, "at+");
    if (fi == NULL) {

    } else if (device->time < 0.002){
        fprintf(fi, "SIDE 1 VOLTAGE SET = %lf\n", device->side1VoltSetValue);
        fprintf(fi, "SIDE 2 VOLTAGE SET = %lf\n", device->side2VoltSetValue);
        int ccc = 0, in = 0;
        for (ccc = 0; ccc < 29; ccc++) {
            fprintf(fi, "%d = ", ccc+1);
            for (in = 0; in < 4; in++) {
                fprintf(fi, "%d ", device->busTopo[ccc][in]);
            }
            fprintf(fi, "&&\n");
        }
        
        _globalBusFlag = 1;
    }
    fclose(fi);
    fi = NULL;

    if (fi == NULL) {

    } else if (device->time > 0.219 && device->time < 0.25){
        fprintf(fi, "busStatus ==================== %lf\n", device->time);
       // fprintf(fi, "SIDE 2 VOLTAGE SET = %lf\n", device->side2VoltSetValue);
        int ccc = 0, in = 0;
        for (ccc = 0; ccc < 29; ccc++) {
            fprintf(fi, "%d = ", ccc+1);
            for (in = 0; in < 4; in++) {
                fprintf(fi, "%d ", device->busModeStatus[ccc][in]);
            }
            fprintf(fi, "&&\n");
        }
        
        _globalBusFlag = 1;
    }
    fclose(fi);
    fi = NULL;*/
    // 结果输出
    //device->testDC = device->busTieFiltVmc[0];

    tripSignal[0] = device->busBrkTripFlag[0];//test_3;//device->busSecFiltIma[0];//
    tripSignal[1] = device->busBrkTripFlag[1];//testPhasor[0];//testDC;//device->busTieFiltIna[0];//device->
    tripSignal[2] = device->busBrkTripFlag[2];//device->device->
    tripSignal[3] = device->busBrkTripFlag[3];//testPhasor[1];//busBrkTripFlag[3];

}

void busLinkSimulation_500kV(BusDevice* device, Device line[], Device lineRelay[], char* deviceName, double time, int deviceEnable) {
    // 设置整定值
   if (busNotYet(device, "设置保护装置名及保护定值")) {
        busDeviceInit(device, deviceName, deviceEnable, 'B');
    }
    device->time = time;
    /*char fileName[100];
    sprintf(fileName, "%s%s%s", "..\\test\\", deviceName, ".txt");

    FILE *fi = fopen(fileName, "at+");
    if (fi == NULL) {

    } else if (device->time > 0.5){
        fprintf(fi, "SIDE 1 VOLTAGE SET = %lf\n", device->side1VoltSetValue);
        fprintf(fi, "SIDE 2 VOLTAGE SET = %lf\n", device->side2VoltSetValue);
        int ccc = 0, in = 0;
        for (ccc = 0; ccc < 10; ccc++) {
            fprintf(fi, "%d = ", ccc+1);
            for (in = 0; in < 4; in++) {
                fprintf(fi, "%d ", device->busTopo[ccc][in]);
            }
            fprintf(fi, "&&\n");
        }
        
        _globalBusFlag = 1;
    }
    fclose(fi);
    fi = NULL;*/
    // busWriteLog(device, "测试");
    // 只有装置启用情况下才进行计算
    // 仿真程序跑10次, 进行一次采样和保护计算
    if (device->deviceEnable && busUpTo10A(device) == 1) {
        bus_500kV(device, line, lineRelay);
    }

}

void busStatusRenew(BusDevice* device, double time,double* port1, double* port2,double* port3, double* port4, double* port5,double* port6, double* port7, double* port8, double* port9, double* port10, double* port11) {
    // 1-3 :线路断路器状态；4-6：线路隔离开关控制字； 7/8：母联、分段断路器状态；/**删除**9：旁母隔离开关信息；**/10-12：线路与旁母的连接状态；13-16：母联、分段开关电压电流
    int i = 0, j = 0, k = 0;
    // 断路器状态qf判断线路是否投运，隔离开关控制字qs判断投运在哪一条母线(采进来的是对两个隔离开关的控制信号，1代表连接于上方母线/编号小的母线，0代表连接于下方母线/编号大的母线)
    int qf[24], qs[24];

    // 母联断路器、分段断路器状态写入
    for (i = 0; i < 3; i++) {
        device->busBrkStatus[i] = (int)((2-port7[i])/2);
        device->busBrkStatus[i+3] = (int)((2-port7[i+3])/2);

        device->busBrkStatus[i+6] = (int)((2-port8[i])/2);
        device->busBrkStatus[i+9] = (int)((2-port8[i+3])/2);
    }

       /* device->busTopo[0][0] = 1;
        device->busTopo[0][1] = 0;
        device->busTopo[0][2] = 1;
        device->busTopo[0][3] = 0;

        device->busTopo[1][0] = 0;
        device->busTopo[1][1] = 1;
        device->busTopo[1][2] = 0;
        device->busTopo[1][3] = 1;

        for (i = 2; i < 3; i++){
        device->busTopo[i][0] = 1;
        device->busTopo[i][1] = 0;
        device->busTopo[i][2] = 1;
        device->busTopo[i][3] = 0;
        }
        for (i = 3; i < 4; i++){
        device->busTopo[i][0] = 0;
        device->busTopo[i][1] = 1;
        device->busTopo[i][2] = 0;
        device->busTopo[i][3] = 1;
        }
        for (i = 4; i < 5; i++){
        device->busTopo[i][0] = 0;
        device->busTopo[i][1] = 0;
        device->busTopo[i][2] = 0;
        device->busTopo[i][3] = 0;
        }
        for (i = 5; i < 20; i++){
        device->busTopo[i][0] = 0;
        device->busTopo[i][1] = 0;
        device->busTopo[i][2] = 0;
        device->busTopo[i][3] = 0;
            }
        for (i = 20; i < 24; i++){
        device->busTopo[i][0] = 0;
        device->busTopo[i][1] = 0;
        device->busTopo[i][2] = 0;
        device->busTopo[i][3] = 0;
        }
        // 24-ML1
        device->busTopo[24][0] = 1;
        device->busTopo[24][1] = 0;
        device->busTopo[24][2] = -1;
        device->busTopo[24][3] = 0;
        // 25-ML2
        device->busTopo[25][0] = 0;
        device->busTopo[25][1] = 1;
        device->busTopo[25][2] = 0;
        device->busTopo[25][3] = -1;
        // 26-FD1
        device->busTopo[26][0] = -1;
        device->busTopo[26][1] = 1;
        device->busTopo[26][2] = 0;
        device->busTopo[26][3] = 0;
        // 27-FD2
        device->busTopo[27][0] = 0;
        device->busTopo[27][1] = 0;
        device->busTopo[27][2] = -1;
        device->busTopo[27][3] = 1;
        // 28-ML1 BYPASS
        device->busTopo[28][0] = -1;
        device->busTopo[28][1] = 0;
        device->busTopo[28][2] = -1;
        device->busTopo[28][3] = 0;*/
    // 更新装置时间
    device->time = time;

    // if (device->writeInFlag == 1) {
    //     return;
    // }

    // 初始化（置0）
    for (i = 0; i < 29; i++) {
        for (j = 0; j < 4; j++){
            device->busModeStatus[i][j] = 0;
        }
    }

    // 整合隔离开关状态/断路器状态
    for (i = 0; i < 10; i++){

            qf[i] = (int) ((2-port1[i])/2);
            qf[i+10] = (int) ((2-port2[i])/2);

            qs[i] = port4[i];
            qs[i+10] = port5[i];

            device->bypassSwitchStatus[i] = port9[i];
            device->bypassSwitchStatus[i+10] = port10[i];
    }
    
    for (i = 0; i < 4; i++){

            qf[i+20] = (int) ((2-port3[i])/2);

            qs[i+20] = port6[i];

            device->bypassSwitchStatus[i+20] = port11[i];
    }

    int cnt;
    
    // 隔离开关状态写入运行方式识别矩阵：采样值合位为0, 开位为1 --转换为--> 合位状态为1,开位状态为0
    for (i = 0; i < 24; i++) {
        cnt = 0;
        if (qf[i] == 0) {

            for (j = 0; j < 4; j++) {
                device->busModeStatus[i][j] = 0;
            }

        } else {

            for (j = 0; j < 4; j++) {
                if (device->busTopo[i][j] != 0) {
                    cnt++;
                }
            }
            
            if (cnt == 2) {
                for (j = 0; j < 4; j++){
                    // 关联矩阵中为1/-1的元素更新为对应的隔离开关状态
                    if (k == 0) {
                        if (device->busTopo[i][j] == 1){
                            device->busModeStatus[i][j] = (int) qs[i];
                            k = k+1; 
                        }
                        if (device->busTopo[i][j] == -1){
                            device->busModeStatus[i][j] = -(int) qs[i];
                            k = k+1;               
                        }
                    } else if (k == 1) {
                        if (device->busTopo[i][j] == 1){
                            device->busModeStatus[i][j] = (int)((2-qs[i])/2);
                            k = k+1; 
                        }
                        if (device->busTopo[i][j] == -1){
                            device->busModeStatus[i][j] = -(int)((2-qs[i])/2);
                            k = k+1;               
                        }
                        if (k == 2){
                            k = 0;
                            break;
                        }
                    }
                }   
            } else if (cnt == 1) {
                
                for (j = 0; j < 4; j++) {
                    if (device->busTopo[i][j] == 1){
                        device->busModeStatus[i][j] = 1;
                        break; 
                    }
                    if (device->busTopo[i][j] == -1){
                        device->busModeStatus[i][j] = -1;
                        break;               
                    }
                }
            }
            
        }
        
    }

    for (i = 0; i < 2; i++) {
        for (j = 0; j < 4; j++){
            if (device->busTopo[i+24][j] == 1){

                device->busModeStatus[i+24][j] = 1;//device->busBrkStatus[3*i] || device->busBrkStatus[3*i+1] || device->busBrkStatus[3*i+2];//
                
            }
            if (device->busTopo[i+24][j] == -1){

                device->busModeStatus[i+24][j] = -1;//-(device->busBrkStatus[3*i] || device->busBrkStatus[3*i+1] || device->busBrkStatus[3*i+2]);//
                
            }
            if (device->busTopo[i+26][j] == 1){

                device->busModeStatus[i+26][j] = 1;//device->busBrkStatus[3*i+6] || device->busBrkStatus[3*i+7] || device->busBrkStatus[3*i+8]; //
                
            }
            if (device->busTopo[i+26][j] == -1){

                device->busModeStatus[i+26][j] = -1;//-(device->busBrkStatus[3*i+6] || device->busBrkStatus[3*i+7] || device->busBrkStatus[3*i+8]);//
                
            }
        }
    }

    
   // 旁母隔离开关状态写入
    /*for (i = 0; i < 4; i++){
       device->bypassBusStatus[i] = (int)((2-port9[i])/2);
    }*/
    if (device->bypassBusMode != 0) {
        switch (device->bypassBusMode) {
            case 1: device->busModeStatus[28][0] = device->busTopo[28][0]; break;
            case 2: device->busModeStatus[28][2] = device->busTopo[28][2]; break;
        }
        device->busModeStatus[24][0] = 0;
        device->busModeStatus[24][2] = 0;
    }

    device->writeInFlag = 1;
    return;

}
void deviceInit(Device* device, char* deviceName, int deviceEnable, char type){
    // 设置装置名
    if (deviceEnable == 0) {
        // 装置不启用
        device->deviceEnable = 0;
    } else /*if (deviceEnable == 1)*/ {
        device->deviceEnable = deviceEnable;

        strcpy(device->deviceName, deviceName);

        // 设置globalFileName和deviceFileName
        sprintf(device->globalFileName, "%s/log.txt", logDirName); // 不同装置共用log.txt
        sprintf(device->deviceFileName, "%s/%s", logDirName, deviceName); // 不同装置录波文件分别存放, 按装置名分开

        if (type == 'L') {
            // 读取配置文件, 设置整定值
            readConfiguration(device, 'L');

            // 初始化交换机队列时间延时默认值MAX_VALUE
            initSwitchQueueTime(device);

            // 初始化完毕,记录日志
            if (deviceEnable == 1) {
                writeLog(device, "线路保护装置初始化完毕");
            }
        } else if (type == 'B') {
            // 读取母线保护配置文件, 设置整定值
            readConfiguration(device, 'B');

            // 母线保护模块初始化完毕, 设置整定值
            writeLog(device, "母线保护装置初始化完毕");
        }

    } /*else {
        device->deviceEnable = 0;
    }*/

}

void busDeviceInit(BusDevice* device, char* deviceName, int deviceEnable, char type){
    int i = 0;
    // 设置装置名
    if (deviceEnable == 0) {
        // 装置不启用
        device->deviceEnable = 0;
    } else {
        device->deviceEnable = 1;

        strcpy(device->deviceName, deviceName);

        // 设置globalFileName和deviceFileName
        sprintf(device->globalFileName, "%s/log.txt", logDirName); // 不同装置共用log.txt
        sprintf(device->deviceFileName, "%s/%s", logDirName, deviceName); // 不同装置录波文件分别存放, 按装置名分开
        // 读取母线保护配置文件, 设置整定值
        busReadConfiguration(device, 'B');
        /*device->bypassBusMode = 0; // 0-旁母未投运，1-双母运行I母带旁路，2-双母运行II母带旁路
        device->busDiffEnable = 1; // 差动保护投入

        for (i = 0; i < 4; i++) {
            device->busIndependentOpEnable[i] = 1; // 母联/分段分列运行压板
            device->busIncompletePhaseEnable[i] = 1; // 非全相保护投入
            device->busBrkChargeOcIEnable[i] = 1; // 充电过流I段
            device->busBrkChargeOcIIEnable[i] = 1; // 充电过流I段
            device->busBrkChargeOcZeroEnable[i] = 1; // 充电过流零序
            device->busBrkFailureSetValue[i] = 0.1; // 母联分段失灵保护电流定值
            device->busBrkFailureSetTime[i] = 0.2;
        }

        device->busBranchFailureEnable = 0; // 支路断路器失灵保护控制字

        for (i = 0; i < 29; i++) {
            device->LbranchUnlockFailureEnable[i] = 0; // 线路支路X解除复压闭锁控制字（28-ML1带路运行带旁母)
        }

        // 定值列表
        device->side1CurrSetValue = 1.0;
        device->side2CurrSetValue = 1.0;
        //device->side1VoltSetValue = 220;
        //device->side2VoltSetValue = 220;
        device->busDiffSetValue = 2.0; // 母线差动启动电流定值
        device->busCTBreakBlockSetValue = 0.13; // CT断线闭锁定值
        device->busCTBreakWarnSetValue = 0.12; // CT断线告警定值
        device->busBrkFailureSetValue[4]; // 母联分段失灵保护电流定值
        device->busBrkFailureSetTime[4];*/

        // 母线保护模块初始化完毕, 设置整定值
        busWriteLog(device, "母线保护装置初始化完毕");

    }

}

int Trim(char s[])
{
    int n;
    for (n = strlen(s) - 1; n >= 0; n--)
    {
        if (s[n] != ' ' && s[n] != '\t' && s[n] != '\n')
            break;
        s[n + 1] = '\0';
    }
    return n;
}

int readConfiguration(Device* device, char elementType) {

    // 双数组 参数名-参数对应
    char paramName[PARAM_COUNT][STRING_LENGTH];
    double paramValue[PARAM_COUNT];
    char fileName[STRING_LENGTH];

    int count = 0;
    int i, j;

    // 根据device中的deviceName打开对应的配置文件并进行赋值
    sprintf(fileName, "%s%s%s", "..\\config\\", device->deviceName, ".conf");

    FILE *file = fopen(fileName, "r");
    if (file == NULL)
    {
        writeErrorLog(device, "未找到配置文件!");
        return -1;
    }

    char buf[MAX_BUF_LEN];
    int text_comment = 0;
    while (fgets(buf, MAX_BUF_LEN, file) != NULL)
    {
        Trim(buf);
        // to skip text comment with flags /* ... */
        if (buf[0] != '#' && (buf[0] != '/' || buf[1] != '/'))
        {
            if (strstr(buf, "/*") != NULL)
            {
                text_comment = 1;
                continue;
            }
            else if (strstr(buf, "*/") != NULL)
            {
                text_comment = 0;
                continue;
            }
        }
        if (text_comment == 1)
            continue;

        int buf_len = strlen(buf);
        // ignore and skip the line with first chracter '#', '=' or '/'
        if (buf_len <= 1 || buf[0] == '#' || buf[0] == '=' || buf[0] == '/')
            continue;
        buf[buf_len - 1] = '\0';

        char _paramk[MAX_KEY_LEN] = {0}, _paramv[MAX_VAL_LEN] = {0};
        int _kv = 0, _klen = 0, _vlen = 0;
        int i = 0;
        for (i = 0; i < buf_len; ++i)
        {
            if (buf[i] == ' ')
                continue;
            // scan param key name
            if (_kv == 0 && buf[i] != '=')
            {
                if (_klen >= MAX_KEY_LEN)
                    break;
                _paramk[_klen++] = buf[i];
                continue;
            }
            else if (buf[i] == '=')
            {
                _kv = 1;
                continue;
            }
            // scan param key value
            if (_vlen >= MAX_VAL_LEN || buf[i] == '#')
                break;
            _paramv[_vlen++] = buf[i];
        }
        if (strcmp(_paramk, "") == 0 || strcmp(_paramv, "") == 0)
            continue;

        sprintf(paramName[count], _paramk);
        paramValue[count] = atof(_paramv);
        count++;
        // printf("%s=%s\n", _paramk, _paramv);
    }

    if (device->deviceEnable != 1 && device->deviceEnable != 0) {
        device->ratedSideIVoltage = findSetValueIndex("一次侧额定电压（kV）", paramName, paramValue, count, device); // 线电压
        device->ratedSideIIVoltage = findSetValueIndex("二次侧额定电压（V）", paramName, paramValue, count, device); // 线电压
        device->ratedSideICurrent = findSetValueIndex("一次侧额定电流（kA）", paramName, paramValue, count, device);
        device->ratedSideIICurrent = findSetValueIndex("二次侧额定电流（A）", paramName, paramValue, count, device);
        writeLog(device, "采样装置准备就绪");
        return 0;
    }

    if (elementType == 'L') {

        // 设备参数
        device->ratedSideIVoltage = findSetValueIndex("一次侧额定电压（kV）", paramName, paramValue, count, device); // 线电压
        device->ratedSideIIVoltage = findSetValueIndex("二次侧额定电压（V）", paramName, paramValue, count, device); // 线电压
        device->ratedSideICurrent = findSetValueIndex("一次侧额定电流（kA）", paramName, paramValue, count, device);
        device->ratedSideIICurrent = findSetValueIndex("二次侧额定电流（A）", paramName, paramValue, count, device);

        // 通信延时定值
        device->switch1DelayMin = findSetValueIndex("交换机A最小延时", paramName, paramValue, count, device);
        device->switch1DelayMax = findSetValueIndex("交换机A最大延时", paramName, paramValue, count, device);
        device->switch2DelayMin = findSetValueIndex("交换机B最小延时", paramName, paramValue, count, device);
        device->switch2DelayMax = findSetValueIndex("交换机B最大延时", paramName, paramValue, count, device);
        device->lineZ1 = findSetValueIndex("线路全长正序阻抗定值（ohm）", paramName, paramValue, count, device); // 正序阻抗
        device->lineZ1Angle = findSetValueIndex("线路正序灵敏角（deg）", paramName, paramValue, count, device); // 线路正序灵敏角
        device->lineZ0 = findSetValueIndex("线路全长零序阻抗定值（ohm）", paramName, paramValue, count, device); // 零序阻抗
        device->lineZ0Angle = findSetValueIndex("线路零序灵敏角（deg）", paramName, paramValue, count, device); // 线路零序灵敏角
        device->lineC1 = findSetValueIndex("线路全长正序容抗定值（ohm）", paramName, paramValue, count, device); // 正序容抗
        device->lineC0 = findSetValueIndex("线路全长零序容抗定值（ohm）", paramName, paramValue, count, device); // 零序容抗
        device->lineLength = findSetValueIndex("线路长度（km）", paramName, paramValue, count, device); // 线路长度
        device->localReactor = findSetValueIndex("本侧电抗器阻抗定值（ohm）", paramName, paramValue, count, device); // 本侧电抗器阻抗定值
        device->localSmallReactor = findSetValueIndex("本侧小电抗器阻抗定值（ohm）", paramName, paramValue, count, device); // 本侧小电抗器阻抗定值
        device->offsideReactor = findSetValueIndex("对侧电抗器阻抗定值（ohm）", paramName, paramValue, count, device); // 对侧电抗器阻抗定值
        device->offsideSmallReactor = findSetValueIndex("对侧小电抗器阻抗定值（ohm）", paramName, paramValue, count, device); // 对侧小电抗器阻抗定值
        device->KZ = findSetValueIndex("零序补偿系数", paramName, paramValue, count, device); // 零序补偿系数

        // 线路保护控制字列表
        device->currentDiffEnable = (int) findSetValueIndex("纵联差动保护", paramName, paramValue, count, device);	 //纵联差动保护
        device->CTbreakBlockDiffEnable = (int) findSetValueIndex("CT断线闭锁差动", paramName, paramValue, count, device);	 //CT断线闭锁差动
        device->voltFromLinePTEnable = (int) findSetValueIndex("电压取线路PT电压", paramName, paramValue, count, device);	        //电压取线路PT电压
        device->powerSwingBlockEnable = (int) findSetValueIndex("振荡闭锁元件", paramName, paramValue, count, device);	        //振荡闭锁元件
        device->distanceIEnable = (int) findSetValueIndex("距离保护I段", paramName, paramValue, count, device);	            //距离保护I段
        device->distanceIIEnable = (int) findSetValueIndex("距离保护II段", paramName, paramValue, count, device);	            //距离保护II段
        device->distanceIIIEnable = (int) findSetValueIndex("距离保护III段", paramName, paramValue, count, device);          	//距离保护III段
        device->zeroSequenceEnable = (int) findSetValueIndex("零序电流保护", paramName, paramValue, count, device);	            //零序电流保护
        device->directionZeroSequenceIIIEnable = (int) findSetValueIndex("零序过流III段经方向", paramName, paramValue, count, device);	//零序过流III段经方向
        device->brkThreePhaseMode = (int) findSetValueIndex("三相跳闸方式", paramName, paramValue, count, device);	            //三相跳闸方式
        device->currentCompensationEnable = (int) findSetValueIndex("电流补偿", paramName, paramValue, count, device);	    //电流补偿
        device->loadLimitDistanceEnable = (int) findSetValueIndex("负荷限制距离", paramName, paramValue, count, device);	    //负荷限制距离
        device->spRecloseMode = (int) findSetValueIndex("单相重合闸", paramName, paramValue, count, device);	                //单相重合闸
        device->tpRecloseMode = (int) findSetValueIndex("三相重合闸", paramName, paramValue, count, device);	                //三相重合闸
        device->banRecloseMode = (int) findSetValueIndex("禁止重合闸", paramName, paramValue, count, device);	                //禁止重合闸
        device->stopRecloseMode = (int) findSetValueIndex("停用重合闸", paramName, paramValue, count, device);                //停用重合闸
        device->relayIIBlockRecloseEnable = (int) findSetValueIndex("II段保护闭锁重合闸", paramName, paramValue, count, device);                //II段保护闭锁重合闸
        device->mpFaultBlockRecloseEnable = (int) findSetValueIndex("多相故障闭锁重合闸", paramName, paramValue, count, device);	    //多相故障闭锁重合闸
        // device->checkSynMode = (int) findSetValueIndex("重合闸检同期方式", paramName, paramValue, count, device);	                //重合闸检同期方式
        // device->checkVoltMode = (int) findSetValueIndex("重合闸检无压方式", paramName, paramValue, count, device);	                //重合闸检无压方式
        device->speedupDistanceIIEnable = (int) findSetValueIndex("三重加速距离保护II段", paramName, paramValue, count, device);	    //三重加速距离保护II段
        device->speedupDistanceIIIEnable = (int) findSetValueIndex("三重加速距离保护III段", paramName, paramValue, count, device);	    //三重加速距离保护III段
        device->remoteTripControlEnable = (int) findSetValueIndex("远跳经本侧控制", paramName, paramValue, count, device);        //远跳经本侧控制	
     	// device->spTWJRecloseEnable = (int) findSetValueIndex("单相TWJ启动重合闸", paramName, paramValue, count, device);             //单相TWJ启动重合闸
    	// device->tpTWJRecloseEnable = (int) findSetValueIndex("三相TWJ启动重合闸", paramName, paramValue, count, device);             //三相TWJ启动重合闸
        device->deltaDistanceEnable = (int) findSetValueIndex("工频变化量阻抗投运", paramName, paramValue, count, device);           // 工频变化量距离元件
        device->comTripleBlockEnable = (int) findSetValueIndex("沟三闭重开入", paramName, paramValue, count, device);           // 沟三闭重开入
        // device->brkChargeStatus = (int) findSetValueIndex("断路器充电状态", paramName, paramValue, count, device);           // 断路器充电状态

        // 线路保护整定值列表
        // 1.启动元件定值
        device->lineStartSetValue[0] = findSetValueIndex("变化量启动电流定值（kA）", paramName, paramValue, count, device); // 电流突变量启动
        device->lineStartSetValue[1] = findSetValueIndex("零序启动电流定值（kA）", paramName, paramValue, count, device); // 零序电流启动
        device->lineStartSetValue[2] = findSetValueIndex("差动动作电流定值（kA）", paramName, paramValue, count, device); // 差动动作电流-启动
        // 2.接地距离定值
        device->p2gDistanceSetValue[0] = findSetValueIndex("接地距离I段定值（ohm）", paramName, paramValue, count, device);
        device->p2gDistanceSetValue[1] = findSetValueIndex("接地距离II段定值（ohm）", paramName, paramValue, count, device); 
        device->p2gDistanceSetValue[2] = findSetValueIndex("接地距离III段定值（ohm）", paramName, paramValue, count, device);

        device->p2gDistanceTimeSetValue[1] = findSetValueIndex("接地距离II段时间（s）", paramName, paramValue, count, device);	// 接地距离时间  注：下标 1-II段时间，2-III段时间，**0未使用**
        device->p2gDistanceTimeSetValue[2] = findSetValueIndex("接地距离III段时间（s）", paramName, paramValue, count, device);

        device->p2gDistanceDevAngle = findSetValueIndex("接地距离偏移角（deg）", paramName, paramValue, count, device); // 接地距离偏移角
        // 3.相间距离定值
        device->p2pDistanceSetValue[0] = findSetValueIndex("相间距离I段定值（ohm）", paramName, paramValue, count, device); // 差动动作电流-启动;
        device->p2pDistanceSetValue[1] = findSetValueIndex("相间距离II段定值（ohm）", paramName, paramValue, count, device); 
        device->p2pDistanceSetValue[2] = findSetValueIndex("相间距离III段定值（ohm）", paramName, paramValue, count, device);

        device->p2pDistanceTimeSetValue[1] = findSetValueIndex("相间距离II段时间（s）", paramName, paramValue, count, device);	// 接地距离时间  注：下标 1-II段时间，2-III段时间，**0未使用**
        device->p2pDistanceTimeSetValue[2] = findSetValueIndex("相间距离III段时间（s）", paramName, paramValue, count, device);

        device->p2pDistanceDevAngle = findSetValueIndex("相间距离偏移角（deg）", paramName, paramValue, count, device); // 接地距离偏移角
        // 4.负荷限制定值
        device->loadLimitSetValue = findSetValueIndex("负荷限制电阻定值（ohm）", paramName, paramValue, count, device); //负荷限制电阻定值
        // 5.零序过流定值  注：0-II段，1-III段，2-加速段
        device->zeroSequenceSetValue[0] = findSetValueIndex("零序过流II段定值（kA）", paramName, paramValue, count, device);
        device->zeroSequenceSetValue[1] = findSetValueIndex("零序过流III段定值（kA）", paramName, paramValue, count, device);
        device->zeroSequenceSetValue[2] = findSetValueIndex("零序过流加速段定值（kA）", paramName, paramValue, count, device);

        device->zeroSequenceTimeSetValue[0] = findSetValueIndex("零序过流II段时间（s）", paramName, paramValue, count, device);
        device->zeroSequenceTimeSetValue[1] = findSetValueIndex("零序过流III段时间（s）", paramName, paramValue, count, device);
        // 6.工频变化量距离定值
        device->deltaImpedance = findSetValueIndex("工频变化量阻抗（ohm）", paramName, paramValue, count, device); // 工频变化量阻抗 
        // 7.振荡闭锁过流定值
        device->psbCurrentSetValue = findSetValueIndex("振荡闭锁过流定值（kA）", paramName, paramValue, count, device); // 振荡闭锁过流定值
        // 8.PT断线定值
        device->ptBreakOverCurrentSetValue = findSetValueIndex("PT断线相过流定值（kA）", paramName, paramValue, count, device); // PT断线相过流定值
        device->ptBreakZeroSeqSetValue = findSetValueIndex("PT断线零序过流定值（kA）", paramName, paramValue, count, device); // PT断线零序过流定值
        device->ptBreakOcTimeSetValue = findSetValueIndex("PT断线过流时间（s）", paramName, paramValue, count, device); // PT断线过流时间
        // 9.CT断线定值
        device->ctBreakDiffCurrentSetValue = findSetValueIndex("CT断线差流定值（kA）", paramName, paramValue, count, device); // CT断线差流定值

        // 其他不需要用户整定的控制字及定值
        // 非全相振闭开放元件
        device->psbOpenPhaseSetValue[0] = 100;
        device->psbOpenPhaseSetValue[1] = device->lineStartSetValue[0];

        // 差动元件时间定值
        device->currentDiffTimeSetValue[0] = 0.04; // 零差40ms
        device->currentDiffTimeSetValue[1] = 0.025; // 稳态II段25ms

        // 单跳不返回三跳时间定值
        device->singleNotReturnTimeSetValue = 0.02;

        // 复归时间定值
        device->returnSetTime = 10.0;

        // PT断线判断时间定值
        device->PTBreakJudgeTimeSetValue[0] = 1.25;
        device->PTBreakJudgeTimeSetValue[1] = 10;
        // CT断线判断时间定值
        device->CTBreakJudgeTimeSetValue[0] = 0.2;
        device->CTBreakJudgeTimeSetValue[1] = 10;

        writeLog(device, "读取线路保护定值");

    } else if (elementType == 'B') {
       /* for (i = 0; i < count; i++) {
            if (strcmp(paramName[i], "参数分隔标志位") == 0) {
                break;
            }
            if (paramValue[i] == 1) {
                device->busRange[device->busNum] = &linesM[i];
                device->busNum++;
            }
        }

        // 整定值赋值
        device->busCurrentDiffSetValue = findSetValueIndex("母线电流差动保护", paramName, paramValue, count, device);
        device->busCurrentDiffTimeSetValue = findSetValueIndex("母线电流差动保护时间", paramName, paramValue, count, device);

        writeLog(device, "读取母线保护定值");
    } else {
*/
    }
    return 0;
}

int busReadConfiguration(BusDevice* device, char elementType) {
    // 双数组 参数名-参数对应
    char paramName[PARAM_COUNT][STRING_LENGTH];
    double paramValue[PARAM_COUNT];
    char fileName[STRING_LENGTH];

    int count = 0;
    int i, j;

    // 根据device中的deviceName打开对应的配置文件并进行赋值
    sprintf(fileName, "%s%s%s", "..\\config\\", device->deviceName, ".conf");

    FILE *file = fopen(fileName, "r");
    if (file == NULL)
    {
        busWriteErrorLog(device, "未找到配置文件!");
        return -1;
    }

    char buf[MAX_BUF_LEN];
    int text_comment = 0;
    while (fgets(buf, MAX_BUF_LEN, file) != NULL)
    {
        Trim(buf);
        // to skip text comment with flags /* ... */
        if (buf[0] != '#' && (buf[0] != '/' || buf[1] != '/'))
        {
            if (strstr(buf, "/*") != NULL)
            {
                text_comment = 1;
                continue;
            }
            else if (strstr(buf, "*/") != NULL)
            {
                text_comment = 0;
                continue;
            }
        }
        if (text_comment == 1)
            continue;

        int buf_len = strlen(buf);
        // ignore and skip the line with first chracter '#', '=' or '/'
        if (buf_len <= 1 || buf[0] == '#' || buf[0] == '=' || buf[0] == '/')
            continue;
        buf[buf_len - 1] = '\0';

        char _paramk[MAX_KEY_LEN] = {0}, _paramv[MAX_VAL_LEN] = {0};
        int _kv = 0, _klen = 0, _vlen = 0;
        int i = 0;
        for (i = 0; i < buf_len; ++i)
        {
            if (buf[i] == ' ')
                continue;
            // scan param key name
            if (_kv == 0 && buf[i] != '=')
            {
                if (_klen >= MAX_KEY_LEN)
                    break;
                _paramk[_klen++] = buf[i];
                continue;
            }
            else if (buf[i] == '=')
            {
                _kv = 1;
                continue;
            }
            // scan param key value
            if (_vlen >= MAX_VAL_LEN || buf[i] == '#')
                break;
            _paramv[_vlen++] = buf[i];
        }
        if (strcmp(_paramk, "") == 0 || strcmp(_paramv, "") == 0)
            continue;

        sprintf(paramName[count], _paramk);

        int voltWriteFlag = 0;
        if (device->side1VoltSetValue != 0.0) {
            voltWriteFlag = 1;
        }
        if (voltWriteFlag == 1) {
            int busCnt = 0;
            if (fabs(device->side1VoltSetValue - 220.0) < 0.5) {
                busCnt = 4;
            } else if (fabs(device->side1VoltSetValue - 500.0) < 0.5) {
                busCnt = 2;
            }
            char busNum[4][5] = {"I", "II","III","IV"};
            char busText_topo[30] = "段母线连接拓扑";
            char busText_polar[30] = "段母线电流极性";
            char *temp;
            int cnt[2] = {0,0};
            for (cnt[0] = 0; cnt[0] < busCnt; cnt[0]++) {
                temp = strcat_self(busNum[cnt[0]], busText_topo);
                if (strcmp(temp, paramName[count]) == 0) {
                    if (hexToBinMatrix(_paramv, device, cnt[0], 't')) {
                        busWriteLogWithPhase(device, "%c段母线连接拓扑整定完成", cnt[0]);
                    } else {
                        busWriteErrorLog(device, "母线连接拓扑整定有误！");
                    }
                    break;
                }
                free(temp);
            }
            for (cnt[1] = 0; cnt[1] < busCnt; cnt[1]++) {
                temp = strcat_self(busNum[cnt[1]], busText_polar);
                if (strcmp(temp, paramName[count]) == 0) {
                    if (hexToBinMatrix(_paramv, device, cnt[1], 'p')) {
                        busWriteLogWithPhase(device, "%c段母线电流极性整定完成", cnt[1]);
                    } else {
                        busWriteErrorLog(device, "母线电流极性整定有误！");
                    }
                    break;
                }
                free(temp);
            }
        }

        paramValue[count] = atof(_paramv);
        if (device->side1VoltSetValue < 0.05) {
            int findCnt = 0;
            for (findCnt = 0; findCnt < count; findCnt++) {
                if (strcmp("一次侧额定电压（kV）", *(paramName+findCnt)) == 0) {
                    device->side1VoltSetValue = paramValue[findCnt];
                    break;
                }
            }
        }
        count++;
        // printf("%s=%s\n", _paramk, _paramv);
    }

       /* for (i = 0; i < count; i++) {
            if (strcmp(paramName[i], "参数分隔标志位") == 0) {
                break;
            }
            if (paramValue[i] == 1) {
                device->busRange[device->busNum] = &lines[i];
                device->busNum++;
            }
        }

        // 整定值赋值
        device->busCurrentDiffSetValue = busFindSetValueIndex("母线电流差动保护", paramName, paramValue, count, device);
        device->busCurrentDiffTimeSetValue = busFindSetValueIndex("母线电流差动保护时间", paramName, paramValue, count, device);*/
        device->side1CurrSetValue = busFindSetValueIndex("一次侧额定电流（kA）", paramName, paramValue, count, device);
        device->side2VoltSetValue = busFindSetValueIndex("二次侧额定电压（V）", paramName, paramValue, count, device);
        device->side2CurrSetValue = busFindSetValueIndex("二次侧额定电流（A）", paramName, paramValue, count, device);
        device->busDiffSetValue = busFindSetValueIndex("差动保护启动电流定值（kA）", paramName, paramValue, count, device);
        device->busDiffEnable = busFindSetValueIndex("差动保护投入", paramName, paramValue, count, device);
        device->busCTBreakBlockSetValue = busFindSetValueIndex("CT断线闭锁定值（kA）", paramName, paramValue, count, device); // CT断线闭锁定值
        device->busCTBreakWarnSetValue = busFindSetValueIndex("CT断线告警定值（kA）", paramName, paramValue, count, device); // CT断线告警定值

        if (fabs(device->side1VoltSetValue- 500.0) < 0.5) {

        } else if (fabs(device->side1VoltSetValue- 220.0) < 0.5) {
            device->busBrkFailureSetValue[0] = busFindSetValueIndex("母联分段失灵保护电流定值（kA）", paramName, paramValue, count, device); 
            device->busBrkFailureSetTime[0] = busFindSetValueIndex("母联分段失灵保护时间（s）", paramName, paramValue, count, device); 
            device->busIndependentOpEnable[0] = busFindSetValueIndex("母联1分列运行", paramName, paramValue, count, device);
            device->busIndependentOpEnable[1] = busFindSetValueIndex("母联2分列运行", paramName, paramValue, count, device);
            device->busIndependentOpEnable[2] = busFindSetValueIndex("分段1分列运行", paramName, paramValue, count, device);
            device->busIndependentOpEnable[3] = busFindSetValueIndex("分段2分列运行", paramName, paramValue, count, device);
            device->busIncompletePhaseEnable[0] = busFindSetValueIndex("母联1非全相保护投入", paramName, paramValue, count, device);
            device->busIncompletePhaseEnable[1] = busFindSetValueIndex("母联2非全相保护投入", paramName, paramValue, count, device);
            device->busIncompletePhaseEnable[2] = busFindSetValueIndex("分段1非全相保护投入", paramName, paramValue, count, device);
            device->busIncompletePhaseEnable[3] = busFindSetValueIndex("分段2非全相保护投入", paramName, paramValue, count, device);
            device->bypassBusMode = busFindSetValueIndex("旁母运行状态", paramName, paramValue, count, device);
            device->busBrkChargeOcIEnable[0] = busFindSetValueIndex("母联1充电过流I段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIEnable[1] = busFindSetValueIndex("母联2充电过流I段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIEnable[2] = busFindSetValueIndex("分段1充电过流I段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIEnable[3] = busFindSetValueIndex("分段2充电过流I段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIIEnable[0] = busFindSetValueIndex("母联1充电过流II段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIIEnable[1] = busFindSetValueIndex("母联2充电过流II段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIIEnable[2] = busFindSetValueIndex("分段1充电过流II段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcIIEnable[3] = busFindSetValueIndex("分段2充电过流II段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcZeroEnable[0] = busFindSetValueIndex("母联1充电过流零序段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcZeroEnable[1] = busFindSetValueIndex("母联2充电过流零序段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcZeroEnable[2] = busFindSetValueIndex("分段1充电过流零序段投入", paramName, paramValue, count, device);
            device->busBrkChargeOcZeroEnable[3] = busFindSetValueIndex("分段2充电过流零序段投入", paramName, paramValue, count, device);
        }
        /*device->bypassBusMode = 0; // 0-旁母未投运，1-双母运行I母带旁路，2-双母运行II母带旁路
        device->busDiffEnable = 1; // 差动保护投入

        for (i = 0; i < 4; i++) {
            device->busIndependentOpEnable[i] = 1; // 母联/分段分列运行压板
            device->busIncompletePhaseEnable[i] = 1; // 非全相保护投入
            device->busBrkChargeOcIEnable[i] = 1; // 充电过流I段
            device->busBrkChargeOcIIEnable[i] = 1; // 充电过流I段
            device->busBrkChargeOcZeroEnable[i] = 1; // 充电过流零序
            device->busBrkFailureSetValue[i] = 0.1; // 母联分段失灵保护电流定值
            device->busBrkFailureSetTime[i] = 0.2;
        }

        device->busBranchFailureEnable = 0; // 支路断路器失灵保护控制字

        for (i = 0; i < 29; i++) {
            device->LbranchUnlockFailureEnable[i] = 0; // 线路支路X解除复压闭锁控制字（28-ML1带路运行带旁母)
        }

        // 定值列表
        device->busDiffSetValue = 0.2; // 母线差动启动电流定值
        device->busCTBreakBlockSetValue = 0.5; // CT断线闭锁定值
        device->busCTBreakWarnSetValue = 0.3; // CT断线告警定值
        
        
        
        double failLowVoltBlockSetValue;
        double failZeroVoltBlockSetValue;
        double failNegVoltBlockSetValue;
        double failPhaseCurrentSetValue;
        double failZeroCurrentSetValue;
        double failNegCurrentSetValue;
        double failTimeSetI;
        double failTimeSetII;

        double chargeOcISetValue;
        double chargeOcISetTime;
        double chargeOcIISetValue;
        double chargeOcIISetTime;
        double chargeZeroCurrentSetValue;

        double openPhaseZeroCurrSetValue;
        double openPhaseNegCurrSetValue;
        double openPhaseSetTime;
        */

        busWriteLog(device, "读取母线保护定值");

    return 0;
}

char* strcat_self(char* dest, char* src){
    int d_len = strlen(dest);
    int s_len = strlen(src);
    char* ret = (char*)malloc(sizeof(char)*(d_len+s_len+2));
    int i = 0, j = 0;

    while (dest[i] != '\0') {
        ret[i] = dest[i];
        i++;
    }
    while (src[j] != '\0') {
        ret[i] = src[j];
        i++;
        j++;
    }
    ret[i] = '\0';

    return ret;
}

int hexToDec(char hex) {
    int ret = 0;
    if (hex - '0' >= 0 && hex - '0' < 10) {
        ret = hex - '0';
    } else {
        ret = hex - 'A' + 10;
    }
    return ret;
}

int hexToBinMatrix(char* hex, BusDevice* bus, int busNum, char tag) {
    int cnt = 0, i = 0;
    int tmp;
    if (tag == 't') { // topo
        while (hex[cnt] != '\0' && cnt*4 < MAX_BRK_NUM_500kV) {
            tmp = hexToDec(hex[cnt]);
            for (i = 3; i >= 0; i--) {
                bus->busTopo[cnt*4+i][busNum] = tmp%2;
                tmp /= 2;
                /*10 5 0
                5  2 1
                2  1 0
                1  0 1 
                */
            }
            cnt++;
        }
    } else if (tag == 'p') { // polar
        while (hex[cnt] != '\0' && cnt*4 < MAX_BRK_NUM_500kV) {
            tmp = hexToDec(hex[cnt]);
            for (i = 3; i >= 0; i--) {
                if (tmp%2) {
                    bus->busTopo[cnt*4+i][busNum] *= -1;
                }
                tmp /= 2;
                /*10 5 0
                5  2 1
                2  1 0
                1  0 1 
                */
            }
            cnt++;
        }
    } else {
        return 0;
    }
    
    return 1;
}

/**
 * 根据target名, 返回对应的整定值的大小
 * @param target
 * @param paramName
 * @param paramValue
 * @param n
 * @return
 */
double findSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, Device* device) {
    int i = 0;
    char errorLog[STRING_LENGTH];

    for (i = 0; i < n; i++) {
        if (strcmp(target, *(paramName+i)) == 0) {
            return paramValue[i];
        }
    }
    // 如果没有找到对应的整定值, 将错误信息记录到日志中
    sprintf(errorLog, "%s%s", target, "参数未定义!");
    writeErrorLog(device, errorLog);
    return 0;
}

double busFindSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, BusDevice* device) {
    int i = 0;
    char errorLog[STRING_LENGTH];

    for (i = 0; i < n; i++) {
        if (strcmp(target, *(paramName+i)) == 0) {
            return paramValue[i];
        }
    }
    // 如果没有找到对应的整定值, 将错误信息记录到日志中
    sprintf(errorLog, "%s%s", target, "参数未定义!");
    busWriteErrorLog(device, errorLog);
    return 0;
}

void initSwitchQueueTime(Device* device) {
    int i = 0;
    for (i = 0; i < QUEUE_LENGTH; i++) {
        device->switchQueue1[i].delayTime = MAX_VALUE;
        device->switchQueue2[i].delayTime = MAX_VALUE;
    }
}

int busUpTo10A(BusDevice* device) {
    device->sampleCount1++;
    if (device->sampleCount1 == 10) {
        device->sampleCount1 = 0;
        return 1;
    } else {
        return 0;
    }
}

int busUpTo5(BusDevice* device) {
    device->sampleCount2++;
    if (device->sampleCount2 == 5) {
        device->sampleCount2 = 0;
        return 1;
    } else {
        return 0;
    }
}

int upTo10A(Device* device) {
    device->sampleCount1++;
    if (device->sampleCount1 == 10) {
        device->sampleCount1 = 0;
        return 1;
    } else {
        return 0;
    }
}

int upTo5(Device* device) {
    device->sampleCount2++;
    if (device->sampleCount2 == 5) {
        device->sampleCount2 = 0;
        return 1;
    } else {
        return 0;
    }
}

// 仿真采样
void sample(Device* device, double time, double* port1, double* port2) {
    int i = 0;

    // 更新装置时间
    device->time = time;

    for (i = 0; i < 6; i++) {
        // 本侧电压电流
        device->sample[i] = port1[i];
        // 对侧电压电流
        device->sample[i+6] = port2[i];
    }

    device->sample[12] = port1[0];
    device->sample[13] = port1[1];
    device->sample[14] = port1[2];
    device->sample[15] = port1[9];
    if (device->ratedSideIVoltage - 220.0) {

    }

    device->sampleManBrkStatus[0] = (int)((2-port2[6])/2);
    device->sampleManBrkStatus[1] = (int)((2-port2[7])/2);
    device->sampleManBrkStatus[2] = (int)((2-port2[8])/2);

    for (i = 0; i < 3; i++) {
       // 断路器状态不需要记忆, 随采随用即可
        device->sampleBrkStatus[i] = (int)((2-port1[i+6])/2);
        device->sampleBrkStatus[i+3] = 1;
   }
    // 母线电压
    /*device->sample[12] = port1[9];
    device->sample[13] = port1[10];
    device->sample[14] = port1[11];

    // 采样值合位为0, 开位为2 --转换为--> 合位状态为1,开位状态为0
   for (i = 0; i < 3; i++) {
       // 断路器状态不需要记忆, 随采随用即可
        device->sampleBrkStatus[i] = (int)((2-port1[i+6])/2);
        device->sampleBrkStatus[i+3] = (int)((2-port2[i+6])/2);
   }
   // 手动操作位置
   device->sampleManBrkStatus[0] = (int)port2[6];
   device->sampleManBrkStatus[1] = (int)port2[7];
   device->sampleManBrkStatus[2] = (int)port2[8];*/

   // 外接零序
   //zeroSeqSample(device, port1);

    
}

void zeroSeqSample(Device* device, double* port1) {
    int i = 0;
    int j = 0;
    
    // 更新数据    
    for (i = RECORD_LENGTH-1; i >= 1; i--) {
        device->instZeroSeqI[i] = device->instZeroSeqI[i-1];
    }
    device->instZeroSeqI[0] = port1[9];

    // 滤波
    for (i = WINDOW-1; i >= 1; i--) {
        device->filterZeroSeqI[i] = device->filterZeroSeqI[i-1];
    }
    // 更新新的滤波后数据点
    lowPassFilter(device->filterZeroSeqI, device->instZeroSeqI);

    // 相量
    inst2phasor(device->filterZeroSeqI, 0, &device->zeroSeqCurrentOUT);


}

void sample2inst(Device* device) {
    int i = 0;
    int j = 0;
    double kCT = device->ratedSideICurrent / device->ratedSideIICurrent;
    double kPT = device->ratedSideIVoltage/ sqrt(3.0) / device->ratedSideIIVoltage;
    // 更新数据    
    for (i = RECORD_LENGTH-1; i >= 1; i--) {
        device->instTime[i] = device->instTime[i-1];
        device->instVma[i] = device->instVma[i-1];
        device->instVmb[i] = device->instVmb[i-1];
        device->instVmc[i] = device->instVmc[i-1];
        device->instIma[i] = device->instIma[i-1];
        device->instImb[i] = device->instImb[i-1];
        device->instImc[i] = device->instImc[i-1];

        device->instVna[i] = device->instVna[i-1];
        device->instVnb[i] = device->instVnb[i-1];
        device->instVnc[i] = device->instVnc[i-1];
        device->instIna[i] = device->instIna[i-1];
        device->instInb[i] = device->instInb[i-1];
        device->instInc[i] = device->instInc[i-1];

        device->instVmaBus[i] = device->instVmaBus[i-1];
        device->instVmbBus[i] = device->instVmbBus[i-1];
        device->instVmcBus[i] = device->instVmcBus[i-1];

        device->instZeroSeqI[i] = device->instZeroSeqI[i-1];
    }

    device->instTime[0] = device->time;
    device->instVma[0] = device->sample[0] * kPT;
    device->instVmb[0] = device->sample[1] * kPT;
    device->instVmc[0] = device->sample[2] * kPT;
    device->instIma[0] = device->sample[3] * kCT;
    device->instImb[0] = device->sample[4] * kCT;
    device->instImc[0] = device->sample[5] * kCT;

    device->instVna[0] = device->sample[6] * kPT;
    device->instVnb[0] = device->sample[7] * kPT;
    device->instVnc[0] = device->sample[8] * kPT;
    device->instIna[0] = device->sample[9] * kCT;
    device->instInb[0] = device->sample[10] * kCT;
    device->instInc[0] = device->sample[11] * kCT;

    device->instVmaBus[0] = device->sample[12] * kPT;
    device->instVmbBus[0] = device->sample[13] * kPT;
    device->instVmcBus[0] = device->sample[14] * kPT;

    device->instZeroSeqI[0] = device->sample[15] * kCT;


    // 断路器状态和手合开关状态
    for (i = POINTS-1; i >= 0; i--) {
        for (j = 0; j < 6; j++) {
            device->brkStatus[i][j] = device->brkStatus[i-1][j];
        }
    }
    for (j = 0; j < 6; j++) {
        device->brkStatus[0][j] = device->sampleBrkStatus[j];
    }

    for (i = POINTS-1; i >= 0; i--) {
        for (j = 0; j < 3; j++) {
            device->manBrkStatus[i][j] = device->manBrkStatus[i-1][j];
        }
    }
    for (j = 0; j < 3; j++) {
        device->manBrkStatus[0][j] = device->sampleManBrkStatus[j];
    }
    
}

void dataFilter(Device* device) {
    
    int i = 0;
    // 滤波后数据后移
    for (i = WINDOW-1; i >= 1; i--) {
        device->filterVma[i] = device->filterVma[i-1];
        device->filterVmb[i] = device->filterVmb[i-1];
        device->filterVmc[i] = device->filterVmc[i-1];
        device->filterIma[i] = device->filterIma[i-1];
        device->filterImb[i] = device->filterImb[i-1];
        device->filterImc[i] = device->filterImc[i-1];

        device->filterVna[i] = device->filterVna[i-1];
        device->filterVnb[i] = device->filterVnb[i-1];
        device->filterVnc[i] = device->filterVnc[i-1];
        device->filterIna[i] = device->filterIna[i-1];
        device->filterInb[i] = device->filterInb[i-1];
        device->filterInc[i] = device->filterInc[i-1];

        device->filterVmaBus[i] = device->filterVmaBus[i-1];
        device->filterVmbBus[i] = device->filterVmbBus[i-1];
        device->filterVmcBus[i] = device->filterVmcBus[i-1];
        device->filterZeroSeqI[i] = device->filterZeroSeqI[i-1];

        device->diffFiltVma[i] = device->diffFiltVma[i-1];
        device->diffFiltVmb[i] = device->diffFiltVmb[i-1];
        device->diffFiltVmc[i] = device->diffFiltVmc[i-1];
        device->diffFiltIma[i] = device->diffFiltIma[i-1];
        device->diffFiltImb[i] = device->diffFiltImb[i-1];
        device->diffFiltImc[i] = device->diffFiltImc[i-1];

        device->diffFiltVna[i] = device->diffFiltVna[i-1];
        device->diffFiltVnb[i] = device->diffFiltVnb[i-1];
        device->diffFiltVnc[i] = device->diffFiltVnc[i-1];
        device->diffFiltIna[i] = device->diffFiltIna[i-1];
        device->diffFiltInb[i] = device->diffFiltInb[i-1];
        device->diffFiltInc[i] = device->diffFiltInc[i-1];

    }

    // 更新新的滤波后数据点

    differenceFilter(device->diffFiltVma, device->instVma);
    differenceFilter(device->diffFiltVmb, device->instVmb);
    differenceFilter(device->diffFiltVmc, device->instVmc);

    differenceFilter(device->diffFiltIma, device->instIma);
    differenceFilter(device->diffFiltImb, device->instImb);
    differenceFilter(device->diffFiltImc, device->instImc);

    differenceFilter(device->diffFiltVna, device->instVna);
    differenceFilter(device->diffFiltVnb, device->instVnb);
    differenceFilter(device->diffFiltVnc, device->instVnc);

    differenceFilter(device->diffFiltIna, device->instIna);
    differenceFilter(device->diffFiltInb, device->instInb);
    differenceFilter(device->diffFiltInc, device->instInc);
    differenceFilter(device->diffFiltZeroSeqI, device->instZeroSeqI);


    lowPassFilter(device->filterVma, device->diffFiltVma);
    lowPassFilter(device->filterVmb, device->diffFiltVmb);
    lowPassFilter(device->filterVmc, device->diffFiltVmc);

    lowPassFilter(device->filterIma, device->diffFiltIma);
    lowPassFilter(device->filterImb, device->diffFiltImb);
    lowPassFilter(device->filterImc, device->diffFiltImc);

    lowPassFilter(device->filterVna, device->diffFiltVna);
    lowPassFilter(device->filterVnb, device->diffFiltVnb);
    lowPassFilter(device->filterVnc, device->diffFiltVnc);

    lowPassFilter(device->filterIna, device->diffFiltIna);
    lowPassFilter(device->filterInb, device->diffFiltInb);
    lowPassFilter(device->filterInc, device->diffFiltInc);

    lowPassFilter(device->filterVmaBus, device->instVmaBus);
    lowPassFilter(device->filterVmbBus, device->instVmbBus);
    lowPassFilter(device->filterVmcBus, device->instVmcBus);

    lowPassFilter(device->filterZeroSeqI, device->diffFiltZeroSeqI);

    /*lowPassFilter(device->filterVma, device->instVma);
    lowPassFilter(device->filterVmb, device->instVmb);
    lowPassFilter(device->filterVmc, device->instVmc);

    lowPassFilter(device->filterIma, device->instIma);
    lowPassFilter(device->filterImb, device->instImb);
    lowPassFilter(device->filterImc, device->instImc);

    lowPassFilter(device->filterVna, device->instVna);
    lowPassFilter(device->filterVnb, device->instVnb);
    lowPassFilter(device->filterVnc, device->instVnc);

    lowPassFilter(device->filterIna, device->instIna);
    lowPassFilter(device->filterInb, device->instInb);
    lowPassFilter(device->filterInc, device->instInc);

    lowPassFilter(device->filterVmaBus, device->instVmaBus);
    lowPassFilter(device->filterVmbBus, device->instVmbBus);
    lowPassFilter(device->filterVmcBus, device->instVmcBus);

    lowPassFilter(device->filterZeroSeqI, device->instZeroSeqI);*/

}

void toPhasor(Device* device) {

    double timeDelay = (0.02/(double)POINTS)*54; // 48+3+3
    if (device->startFlag == 0 || device->time - device->startTime > timeDelay) {
        inst2phasor(device->filterVma, 0, &device->phasor[0]);
        inst2phasor(device->filterVmb, 0, &device->phasor[1]);
        inst2phasor(device->filterVmc, 0, &device->phasor[2]);

        inst2phasor(device->filterIma, 0, &device->phasor[3]);
        inst2phasor(device->filterImb, 0, &device->phasor[4]);
        inst2phasor(device->filterImc, 0, &device->phasor[5]);
    
        inst2phasor(device->filterVna, 0, &device->phasor[6]);
        inst2phasor(device->filterVnb, 0, &device->phasor[7]);
        inst2phasor(device->filterVnc, 0, &device->phasor[8]);

        inst2phasor(device->filterIna, 0, &device->phasor[9]);
        inst2phasor(device->filterInb, 0, &device->phasor[10]);
        inst2phasor(device->filterInc, 0, &device->phasor[11]);
    
        inst2phasor(device->filterVmaBus, 0, &device->busPhasor[0]);
        inst2phasor(device->filterVmbBus, 0, &device->busPhasor[1]);
        inst2phasor(device->filterVmcBus, 0, &device->busPhasor[2]);

        inst2phasor(device->filterZeroSeqI, 0, &device->zeroSeqCurrentOUT);
    } else {
        halfWaveFourier(device->filterVma, 0, &device->phasor[0]);
        halfWaveFourier(device->filterVmb, 0, &device->phasor[1]);
        halfWaveFourier(device->filterVmc, 0, &device->phasor[2]);

        halfWaveFourier(device->filterIma, 0, &device->phasor[3]);
        halfWaveFourier(device->filterImb, 0, &device->phasor[4]);
        halfWaveFourier(device->filterImc, 0, &device->phasor[5]);
    
        halfWaveFourier(device->filterVna, 0, &device->phasor[6]);
        halfWaveFourier(device->filterVnb, 0, &device->phasor[7]);
        halfWaveFourier(device->filterVnc, 0, &device->phasor[8]);

        halfWaveFourier(device->filterIna, 0, &device->phasor[9]);
        halfWaveFourier(device->filterInb, 0, &device->phasor[10]);
        halfWaveFourier(device->filterInc, 0, &device->phasor[11]);
    
        halfWaveFourier(device->filterVmaBus, 0, &device->busPhasor[0]);
        halfWaveFourier(device->filterVmbBus, 0, &device->busPhasor[1]);
        halfWaveFourier(device->filterVmcBus, 0, &device->busPhasor[2]);

        halfWaveFourier(device->filterZeroSeqI, 0, &device->zeroSeqCurrentOUT);

    }
    // if (device->startFlag == 0 || device->time - device->startTime > timeDelay) {
    //     inst2phasor(device->instVma, 0, &device->phasor[0]);
    //     inst2phasor(device->instVmb, 0, &device->phasor[1]);
    //     inst2phasor(device->instVmc, 0, &device->phasor[2]);

    //     inst2phasor(device->instIma, 0, &device->phasor[3]);
    //     inst2phasor(device->instImb, 0, &device->phasor[4]);
    //     inst2phasor(device->instImc, 0, &device->phasor[5]);
    
    //     inst2phasor(device->instVna, 0, &device->phasor[6]);
    //     inst2phasor(device->instVnb, 0, &device->phasor[7]);
    //     inst2phasor(device->instVnc, 0, &device->phasor[8]);

    //     inst2phasor(device->instIna, 0, &device->phasor[9]);
    //     inst2phasor(device->instInb, 0, &device->phasor[10]);
    //     inst2phasor(device->instInc, 0, &device->phasor[11]);
    
    //     inst2phasor(device->instVmaBus, 0, &device->busPhasor[0]);
    //     inst2phasor(device->instVmbBus, 0, &device->busPhasor[1]);
    //     inst2phasor(device->instVmcBus, 0, &device->busPhasor[2]);

    //     inst2phasor(device->instZeroSeqI, 0, &device->zeroSeqCurrentOUT);
    // } else {
    //     halfWaveFourier(device->instVma, 0, &device->phasor[0]);
    //     halfWaveFourier(device->instVmb, 0, &device->phasor[1]);
    //     halfWaveFourier(device->instVmc, 0, &device->phasor[2]);

    //     halfWaveFourier(device->instIma, 0, &device->phasor[3]);
    //     halfWaveFourier(device->instImb, 0, &device->phasor[4]);
    //     halfWaveFourier(device->instImc, 0, &device->phasor[5]);
    
    //     halfWaveFourier(device->instVna, 0, &device->phasor[6]);
    //     halfWaveFourier(device->instVnb, 0, &device->phasor[7]);
    //     halfWaveFourier(device->instVnc, 0, &device->phasor[8]);

    //     halfWaveFourier(device->instIna, 0, &device->phasor[9]);
    //     halfWaveFourier(device->instInb, 0, &device->phasor[10]);
    //     halfWaveFourier(device->instInc, 0, &device->phasor[11]);
    
    //     halfWaveFourier(device->instVmaBus, 0, &device->busPhasor[0]);
    //     halfWaveFourier(device->instVmbBus, 0, &device->busPhasor[1]);
    //     halfWaveFourier(device->instVmcBus, 0, &device->busPhasor[2]);

    //     halfWaveFourier(device->instZeroSeqI, 0, &device->zeroSeqCurrentOUT);

    // }
    // double k = 0;
    // double m = -100000, n = -100000;
    // int i = 0;
    // if (device->time > 0.2) {
        
    //     for (i = 0; i < POINTS; i++) {
    //         m = fabs(device->instIma[i]) > m ? fabs(device->instIma[i]) : m ;
    //         n = fabs(device->filterIma[i]) > n ? fabs(device->filterIma[i]) : n ;
    //     }

    //     k = m/n;
    //     k = 1;
    //     for (i = 0; i < 12; i++) {
    //         device->phasor[i] = phasorNumMulti(k, device->phasor[i]);
    //     }
    // }

}

void busSample(BusDevice* device, double* port1, double* port2, double* port3, double* port4) {
    int i = 0;

    for (i = 0; i < 6; i++) {
        // 母联1电压电流
        device->busSample[i] = port1[i];
        // 母联2电压电流
        device->busSample[i+6] = port2[i];
        // 分段1电压电流
        device->busSample[i+12] = port3[i];
        // 分段2电压电流
        device->busSample[i+18] = port4[i];
    }

}

void busSample2inst(BusDevice* device) {
    int i = 0;
    double kCT = 1;//device->side1CurrSetValue / device->side2CurrSetValue;
    double kPT = 1;//device->side1VoltSetValue / sqrt(3.0) / device->side2VoltSetValue;
    
    // 更新数据    
    for (i = RECORD_LENGTH-1; i >= 1; i--) {
        device->busInstTime[i] = device->busInstTime[i-1];
        device->busTieInstVma[i] = device->busTieInstVma[i-1];
        device->busTieInstVmb[i] = device->busTieInstVmb[i-1];
        device->busTieInstVmc[i] = device->busTieInstVmc[i-1];
        device->busTieInstIma[i] = device->busTieInstIma[i-1];
        device->busTieInstImb[i] = device->busTieInstImb[i-1];
        device->busTieInstImc[i] = device->busTieInstImc[i-1];

        device->busSecInstVma[i] = device->busSecInstVma[i-1];
        device->busSecInstVmb[i] = device->busSecInstVmb[i-1];
        device->busSecInstVmc[i] = device->busSecInstVmc[i-1];
        device->busSecInstIma[i] = device->busSecInstIma[i-1];
        device->busSecInstImb[i] = device->busSecInstImb[i-1];
        device->busSecInstImc[i] = device->busSecInstImc[i-1];

        device->busTieInstVna[i] = device->busTieInstVna[i-1];
        device->busTieInstVnb[i] = device->busTieInstVnb[i-1];
        device->busTieInstVnc[i] = device->busTieInstVnc[i-1];
        device->busTieInstIna[i] = device->busTieInstIna[i-1];
        device->busTieInstInb[i] = device->busTieInstInb[i-1];
        device->busTieInstInc[i] = device->busTieInstInc[i-1];

        device->busSecInstVna[i] = device->busSecInstVna[i-1];
        device->busSecInstVnb[i] = device->busSecInstVnb[i-1];
        device->busSecInstVnc[i] = device->busSecInstVnc[i-1];
        device->busSecInstIna[i] = device->busSecInstIna[i-1];
        device->busSecInstInb[i] = device->busSecInstInb[i-1];
        device->busSecInstInc[i] = device->busSecInstInc[i-1];
    }

    device->busInstTime[0] = device->time;
    device->busTieInstVma[0] = device->busSample[0] * kPT;
    device->busTieInstVmb[0] = device->busSample[1] * kPT;
    device->busTieInstVmc[0] = device->busSample[2] * kPT;
    device->busTieInstIma[0] = device->busSample[3] * kCT;
    device->busTieInstImb[0] = device->busSample[4] * kCT;
    device->busTieInstImc[0] = device->busSample[5] * kCT;

    device->busTieInstVna[0] = device->busSample[6] * kPT;
    device->busTieInstVnb[0] = device->busSample[7] * kPT;
    device->busTieInstVnc[0] = device->busSample[8] * kPT;
    device->busTieInstIna[0] = device->busSample[9] * kCT;
    device->busTieInstInb[0] = device->busSample[10] * kCT;
    device->busTieInstInc[0] = device->busSample[11] * kCT;

    device->busSecInstVma[0] = device->busSample[12] * kPT;
    device->busSecInstVmb[0] = device->busSample[13] * kPT;
    device->busSecInstVmc[0] = device->busSample[14] * kPT;
    device->busSecInstIma[0] = device->busSample[15] * kCT;
    device->busSecInstImb[0] = device->busSample[16] * kCT;
    device->busSecInstImc[0] = device->busSample[17] * kCT;

    device->busSecInstVna[0] = device->busSample[18] * kPT;
    device->busSecInstVnb[0] = device->busSample[19] * kPT;
    device->busSecInstVnc[0] = device->busSample[20] * kPT;
    device->busSecInstIna[0] = device->busSample[21] * kCT;
    device->busSecInstInb[0] = device->busSample[22] * kCT;
    device->busSecInstInc[0] = device->busSample[23] * kCT;

    
}

void busDataFilter(BusDevice* device) {
    
    int i = 0;
    // 滤波后数据后移
    for (i = WINDOW-1; i >= 1; i--) {
        device->busTieFiltVma[i] = device->busTieFiltVma[i-1];
        device->busTieFiltVmb[i] = device->busTieFiltVmb[i-1];
        device->busTieFiltVmc[i] = device->busTieFiltVmc[i-1];
        device->busTieFiltIma[i] = device->busTieFiltIma[i-1];
        device->busTieFiltImb[i] = device->busTieFiltImb[i-1];
        device->busTieFiltImc[i] = device->busTieFiltImc[i-1];

        device->busSecFiltVma[i] = device->busSecFiltVma[i-1];
        device->busSecFiltVmb[i] = device->busSecFiltVmb[i-1];
        device->busSecFiltVmc[i] = device->busSecFiltVmc[i-1];
        device->busSecFiltIma[i] = device->busSecFiltIma[i-1];
        device->busSecFiltImb[i] = device->busSecFiltImb[i-1];
        device->busSecFiltImc[i] = device->busSecFiltImc[i-1];

        device->busTieFiltVna[i] = device->busTieFiltVna[i-1];
        device->busTieFiltVnb[i] = device->busTieFiltVnb[i-1];
        device->busTieFiltVnc[i] = device->busTieFiltVnc[i-1];
        device->busTieFiltIna[i] = device->busTieFiltIna[i-1];
        device->busTieFiltInb[i] = device->busTieFiltInb[i-1];
        device->busTieFiltInc[i] = device->busTieFiltInc[i-1];

        device->busSecFiltVna[i] = device->busSecFiltVna[i-1];
        device->busSecFiltVnb[i] = device->busSecFiltVnb[i-1];
        device->busSecFiltVnc[i] = device->busSecFiltVnc[i-1];
        device->busSecFiltIna[i] = device->busSecFiltIna[i-1];
        device->busSecFiltInb[i] = device->busSecFiltInb[i-1];
        device->busSecFiltInc[i] = device->busSecFiltInc[i-1];

        device->busTieDiffFiltVma[i] = device->busTieDiffFiltVma[i-1];
        device->busTieDiffFiltVmb[i] = device->busTieDiffFiltVmb[i-1];
        device->busTieDiffFiltVmc[i] = device->busTieDiffFiltVmc[i-1];
        device->busTieDiffFiltIma[i] = device->busTieDiffFiltIma[i-1];
        device->busTieDiffFiltImb[i] = device->busTieDiffFiltImb[i-1];
        device->busTieDiffFiltImc[i] = device->busTieDiffFiltImc[i-1];

        device->busSecDiffFiltVma[i] = device->busSecDiffFiltVma[i-1];
        device->busSecDiffFiltVmb[i] = device->busSecDiffFiltVmb[i-1];
        device->busSecDiffFiltVmc[i] = device->busSecDiffFiltVmc[i-1];
        device->busSecDiffFiltIma[i] = device->busSecDiffFiltIma[i-1];
        device->busSecDiffFiltImb[i] = device->busSecDiffFiltImb[i-1];
        device->busSecDiffFiltImc[i] = device->busSecDiffFiltImc[i-1];

        device->busTieDiffFiltVna[i] = device->busTieDiffFiltVna[i-1];
        device->busTieDiffFiltVnb[i] = device->busTieDiffFiltVnb[i-1];
        device->busTieDiffFiltVnc[i] = device->busTieDiffFiltVnc[i-1];
        device->busTieDiffFiltIna[i] = device->busTieDiffFiltIna[i-1];
        device->busTieDiffFiltInb[i] = device->busTieDiffFiltInb[i-1];
        device->busTieDiffFiltInc[i] = device->busTieDiffFiltInc[i-1];

        device->busSecDiffFiltVna[i] = device->busSecDiffFiltVna[i-1];
        device->busSecDiffFiltVnb[i] = device->busSecDiffFiltVnb[i-1];
        device->busSecDiffFiltVnc[i] = device->busSecDiffFiltVnc[i-1];
        device->busSecDiffFiltIna[i] = device->busSecDiffFiltIna[i-1];
        device->busSecDiffFiltInb[i] = device->busSecDiffFiltInb[i-1];
        device->busSecDiffFiltInc[i] = device->busSecDiffFiltInc[i-1];
    }

    // 更新新的滤波后数据点

    differenceFilter(device->busTieDiffFiltVma, device->busTieInstVma);
    differenceFilter(device->busTieDiffFiltVmb, device->busTieInstVmb);
    differenceFilter(device->busTieDiffFiltVmc, device->busTieInstVmc);

    differenceFilter(device->busTieDiffFiltIma, device->busTieInstIma);
    differenceFilter(device->busTieDiffFiltImb, device->busTieInstImb);
    differenceFilter(device->busTieDiffFiltImc, device->busTieInstImc);

    differenceFilter(device->busTieDiffFiltVna, device->busTieInstVna);
    differenceFilter(device->busTieDiffFiltVnb, device->busTieInstVnb);
    differenceFilter(device->busTieDiffFiltVnc, device->busTieInstVnc);

    differenceFilter(device->busTieDiffFiltIna, device->busTieInstIna);
    differenceFilter(device->busTieDiffFiltInb, device->busTieInstInb);
    differenceFilter(device->busTieDiffFiltInc, device->busTieInstInc);

    differenceFilter(device->busSecDiffFiltVma, device->busSecInstVma);
    differenceFilter(device->busSecDiffFiltVmb, device->busSecInstVmb);
    differenceFilter(device->busSecDiffFiltVmc, device->busSecInstVmc);

    differenceFilter(device->busSecDiffFiltIma, device->busSecInstIma);
    differenceFilter(device->busSecDiffFiltImb, device->busSecInstImb);
    differenceFilter(device->busSecDiffFiltImc, device->busSecInstImc);

    differenceFilter(device->busSecDiffFiltVna, device->busSecInstVna);
    differenceFilter(device->busSecDiffFiltVnb, device->busSecInstVnb);
    differenceFilter(device->busSecDiffFiltVnc, device->busSecInstVnc);

    differenceFilter(device->busSecDiffFiltIna, device->busSecInstIna);
    differenceFilter(device->busSecDiffFiltInb, device->busSecInstInb);
    differenceFilter(device->busSecDiffFiltInc, device->busSecInstInc);

    lowPassFilter(device->busTieFiltVma, device->busTieDiffFiltVma);
    lowPassFilter(device->busTieFiltVmb, device->busTieDiffFiltVmb);
    lowPassFilter(device->busTieFiltVmc, device->busTieDiffFiltVmc);

    lowPassFilter(device->busTieFiltIma, device->busTieDiffFiltIma);
    lowPassFilter(device->busTieFiltImb, device->busTieDiffFiltImb);
    lowPassFilter(device->busTieFiltImc, device->busTieDiffFiltImc);

    lowPassFilter(device->busTieFiltVna, device->busTieDiffFiltVna);
    lowPassFilter(device->busTieFiltVnb, device->busTieDiffFiltVnb);
    lowPassFilter(device->busTieFiltVnc, device->busTieDiffFiltVnc);

    lowPassFilter(device->busTieFiltIna, device->busTieDiffFiltIna);
    lowPassFilter(device->busTieFiltInb, device->busTieDiffFiltInb);
    lowPassFilter(device->busTieFiltInc, device->busTieDiffFiltInc);

    lowPassFilter(device->busSecFiltVma, device->busSecDiffFiltVma);
    lowPassFilter(device->busSecFiltVmb, device->busSecDiffFiltVmb);
    lowPassFilter(device->busSecFiltVmc, device->busSecDiffFiltVmc);

    lowPassFilter(device->busSecFiltIma, device->busSecDiffFiltIma);
    lowPassFilter(device->busSecFiltImb, device->busSecDiffFiltImb);
    lowPassFilter(device->busSecFiltImc, device->busSecDiffFiltImc);

    lowPassFilter(device->busSecFiltVna, device->busSecDiffFiltVna);
    lowPassFilter(device->busSecFiltVnb, device->busSecDiffFiltVnb);
    lowPassFilter(device->busSecFiltVnc, device->busSecDiffFiltVnc);

    lowPassFilter(device->busSecFiltIna, device->busSecDiffFiltIna);
    lowPassFilter(device->busSecFiltInb, device->busSecDiffFiltInb);
    lowPassFilter(device->busSecFiltInc, device->busSecDiffFiltInc);

  
}

void toPhasorForBus(BusDevice* device) {

    int i;
    double k = 0;
    Phasor temp = {0,0};
   
    double timeDelay = (0.02/(double)POINTS)*54; // 48+3+3
    if (device->busStartFlag[0] == 0 || device->time - device->busStartTime[0] > timeDelay) {
    inst2phasor(device->busTieFiltVma, 0, &device->busPhasor[0]);
    inst2phasor(device->busTieFiltVmb, 0, &device->busPhasor[1]);
    inst2phasor(device->busTieFiltVmc, 0, &device->busPhasor[2]);

    inst2phasor(device->busTieFiltIma, 0, &device->busPhasor[3]);
    inst2phasor(device->busTieFiltImb, 0, &device->busPhasor[4]);
    inst2phasor(device->busTieFiltImc, 0, &device->busPhasor[5]);

    inst2phasor(device->busTieFiltVna, 0, &device->busPhasor[6]);
    inst2phasor(device->busTieFiltVnb, 0, &device->busPhasor[7]);
    inst2phasor(device->busTieFiltVnc, 0, &device->busPhasor[8]);

    inst2phasor(device->busTieFiltIna, 0, &device->busPhasor[9]);
    inst2phasor(device->busTieFiltInb, 0, &device->busPhasor[10]);
    inst2phasor(device->busTieFiltInc, 0, &device->busPhasor[11]);

    inst2phasor(device->busSecFiltVma, 0, &device->busPhasor[12]);
    inst2phasor(device->busSecFiltVmb, 0, &device->busPhasor[13]);
    inst2phasor(device->busSecFiltVmc, 0, &device->busPhasor[14]);

    inst2phasor(device->busSecFiltIma, 0, &device->busPhasor[15]);
    inst2phasor(device->busSecFiltImb, 0, &device->busPhasor[16]);
    inst2phasor(device->busSecFiltImc, 0, &device->busPhasor[17]);

    inst2phasor(device->busSecFiltVna, 0, &device->busPhasor[18]);
    inst2phasor(device->busSecFiltVnb, 0, &device->busPhasor[19]);
    inst2phasor(device->busSecFiltVnc, 0, &device->busPhasor[20]);

    inst2phasor(device->busSecFiltIna, 0, &device->busPhasor[21]);
    inst2phasor(device->busSecFiltInb, 0, &device->busPhasor[22]);
    inst2phasor(device->busSecFiltInc, 0, &device->busPhasor[23]);
    // inst2phasor(device->busTieInstVma, 0, &device->busPhasor[0]);
    // inst2phasor(device->busTieInstVmb, 0, &device->busPhasor[1]);
    // inst2phasor(device->busTieInstVmc, 0, &device->busPhasor[2]);

    // inst2phasor(device->busTieInstIma, 0, &device->busPhasor[3]);
    // inst2phasor(device->busTieInstImb, 0, &device->busPhasor[4]);
    // inst2phasor(device->busTieInstImc, 0, &device->busPhasor[5]);

    // inst2phasor(device->busTieInstVna, 0, &device->busPhasor[6]);
    // inst2phasor(device->busTieInstVnb, 0, &device->busPhasor[7]);
    // inst2phasor(device->busTieInstVnc, 0, &device->busPhasor[8]);

    // inst2phasor(device->busTieInstIna, 0, &device->busPhasor[9]);
    // inst2phasor(device->busTieInstInb, 0, &device->busPhasor[10]);
    // inst2phasor(device->busTieInstInc, 0, &device->busPhasor[11]);

    // inst2phasor(device->busSecInstVma, 0, &device->busPhasor[12]);
    // inst2phasor(device->busSecInstVmb, 0, &device->busPhasor[13]);
    // inst2phasor(device->busSecInstVmc, 0, &device->busPhasor[14]);

    // inst2phasor(device->busSecInstIma, 0, &device->busPhasor[15]);
    // inst2phasor(device->busSecInstImb, 0, &device->busPhasor[16]);
    // inst2phasor(device->busSecInstImc, 0, &device->busPhasor[17]);

    // inst2phasor(device->busSecInstVna, 0, &device->busPhasor[18]);
    // inst2phasor(device->busSecInstVnb, 0, &device->busPhasor[19]);
    // inst2phasor(device->busSecInstVnc, 0, &device->busPhasor[20]);

    // inst2phasor(device->busSecInstIna, 0, &device->busPhasor[21]);
    // inst2phasor(device->busSecInstInb, 0, &device->busPhasor[22]);
    // inst2phasor(device->busSecInstInc, 0, &device->busPhasor[23]);
    
    } else {
    halfWaveFourier(device->busTieFiltVma, 0, &device->busPhasor[0]);
    halfWaveFourier(device->busTieFiltVmb, 0, &device->busPhasor[1]);
    halfWaveFourier(device->busTieFiltVmc, 0, &device->busPhasor[2]);

    halfWaveFourier(device->busTieFiltIma, 0, &device->busPhasor[3]);
    halfWaveFourier(device->busTieFiltImb, 0, &device->busPhasor[4]);
    halfWaveFourier(device->busTieFiltImc, 0, &device->busPhasor[5]);

    halfWaveFourier(device->busTieFiltVna, 0, &device->busPhasor[6]);
    halfWaveFourier(device->busTieFiltVnb, 0, &device->busPhasor[7]);
    halfWaveFourier(device->busTieFiltVnc, 0, &device->busPhasor[8]);

    halfWaveFourier(device->busTieFiltIna, 0, &device->busPhasor[9]);
    halfWaveFourier(device->busTieFiltInb, 0, &device->busPhasor[10]);
    halfWaveFourier(device->busTieFiltInc, 0, &device->busPhasor[11]);

    halfWaveFourier(device->busSecFiltVma, 0, &device->busPhasor[12]);
    halfWaveFourier(device->busSecFiltVmb, 0, &device->busPhasor[13]);
    halfWaveFourier(device->busSecFiltVmc, 0, &device->busPhasor[14]);

    halfWaveFourier(device->busSecFiltIma, 0, &device->busPhasor[15]);
    halfWaveFourier(device->busSecFiltImb, 0, &device->busPhasor[16]);
    halfWaveFourier(device->busSecFiltImc, 0, &device->busPhasor[17]);

    halfWaveFourier(device->busSecFiltVna, 0, &device->busPhasor[18]);
    halfWaveFourier(device->busSecFiltVnb, 0, &device->busPhasor[19]);
    halfWaveFourier(device->busSecFiltVnc, 0, &device->busPhasor[20]);

    halfWaveFourier(device->busSecFiltIna, 0, &device->busPhasor[21]);
    halfWaveFourier(device->busSecFiltInb, 0, &device->busPhasor[22]);
    halfWaveFourier(device->busSecFiltInc, 0, &device->busPhasor[23]);
    // halfWaveFourier(device->busTieInstVma, 0, &device->busPhasor[0]);
    // halfWaveFourier(device->busTieInstVmb, 0, &device->busPhasor[1]);
    // halfWaveFourier(device->busTieInstVmc, 0, &device->busPhasor[2]);

    // halfWaveFourier(device->busTieInstIma, 0, &device->busPhasor[3]);
    // halfWaveFourier(device->busTieInstImb, 0, &device->busPhasor[4]);
    // halfWaveFourier(device->busTieInstImc, 0, &device->busPhasor[5]);

    // halfWaveFourier(device->busTieInstVna, 0, &device->busPhasor[6]);
    // halfWaveFourier(device->busTieInstVnb, 0, &device->busPhasor[7]);
    // halfWaveFourier(device->busTieInstVnc, 0, &device->busPhasor[8]);

    // halfWaveFourier(device->busTieInstIna, 0, &device->busPhasor[9]);
    // halfWaveFourier(device->busTieInstInb, 0, &device->busPhasor[10]);
    // halfWaveFourier(device->busTieInstInc, 0, &device->busPhasor[11]);

    // halfWaveFourier(device->busSecInstVma, 0, &device->busPhasor[12]);
    // halfWaveFourier(device->busSecInstVmb, 0, &device->busPhasor[13]);
    // halfWaveFourier(device->busSecInstVmc, 0, &device->busPhasor[14]);

    // halfWaveFourier(device->busSecInstIma, 0, &device->busPhasor[15]);
    // halfWaveFourier(device->busSecInstImb, 0, &device->busPhasor[16]);
    // halfWaveFourier(device->busSecInstImc, 0, &device->busPhasor[17]);

    // halfWaveFourier(device->busSecInstVna, 0, &device->busPhasor[18]);
    // halfWaveFourier(device->busSecInstVnb, 0, &device->busPhasor[19]);
    // halfWaveFourier(device->busSecInstVnc, 0, &device->busPhasor[20]);

    // halfWaveFourier(device->busSecInstIna, 0, &device->busPhasor[21]);
    // halfWaveFourier(device->busSecInstInb, 0, &device->busPhasor[22]);
    // halfWaveFourier(device->busSecInstInc, 0, &device->busPhasor[23]);
    

    }
    // double m = -100000, n = -100000;
    // if (device->relayTime > 0.2) {
        
    //     for (i = 0; i < POINTS; i++) {
    //         m = fabs(device->busTieInstIma[i]) > m ? fabs(device->busTieInstIma[i]) : m ;
    //         n = fabs(device->busTieFiltIma[i]) > n ? fabs(device->busTieFiltIma[i]) : n ;
    //     }

    //     k = m/n;
    //     k = 1;
    //     for (i = 0; i < 24; i++) {
    //         device->busPhasor[i] = phasorNumMulti(k, device->busPhasor[i]);
    //     }
    // }
    for (i = 0; i < 24; i++) {
        device->busPhasor[i+24] = phasorNumMulti(-1.0, device->busPhasor[i]);
    }
}


/**
 * 低通滤波函数
 * 参数分别为滤波后数组和滤波前数组
 *
 */
void diffFilter(double* af, double* bf){
    af[0] = (double)(bf[0] - bf[1])*7.649;

}

void lowPassFilter(double* aft, double* bef) {
    // 滤波参数直接给出,使用bufferFilter.m计算得到 此处为48点采样,100hz截止频率
    double at = 0.0976;
    double bt = -0.9428;
    double ct = 0.3333;

//    double at = 0.014401;
//    double bt = -1.632993;
//    double ct = 0.690599;

    aft[0] = at*bef[0] + 2*at*bef[1] + at*bef[2] - bt*aft[1] - ct*aft[2];
    // aft[0] = bef[0];
}

/**
 * 差分滤波函数
 * y(n) = x(n) - x(n-K), K=3
 **/
void differenceFilter(double* after, double* before) {
    after[0] = before[0] - before[2];
    after[0] *= 3.83; //差分滤波幅值还原
}

/**
 * 全周傅式算法
 */
void inst2phasor(double* inst, int start, Phasor* phasor) {
    int i = start;
    // 因为定义的全局变量, 需要先把上次计算值清掉
    phasor->real = 0;
    phasor->img = 0;


    // for (i = start; i < start + (int)POINTS; i++) {
    //     phasor->real += inst[i]*sin(2*PI*(start-i)/(double)POINTS);
    //     phasor->img  += inst[i]*cos(2*PI*(start-i)/(double)POINTS);
    // }
    for (i = start; i < start + (int)POINTS; i++) {
        phasor->real += inst[i]*sin(2*PI*(start-i)/(double)POINTS);
        phasor->img  += inst[i]*cos(2*PI*(start-i)/(double)POINTS);
    }

    // C语言语法规则: 2/400等于零!
    phasor->real = phasor->real * (2/(double)POINTS);
    phasor->img =  phasor->img * (2/(double)POINTS);
}

/**
 * 半周傅式算法
 */
void halfWaveFourier(double* inst, int start, Phasor* phasor) {
    int i = start;
    int reverse = start*2 + POINTS/2 - 1;
    // 因为定义的全局变量, 需要先把上次计算值清掉
    phasor->real = 0;
    phasor->img = 0;


    // for (i = start; i < start + POINTS/2; i++) {
    //     phasor->real += inst[i]*sin(2*PI*(start-i)/(double)POINTS);
    //     phasor->img  += inst[i]*cos(2*PI*(start-i)/(double)POINTS);
    // }
    for (i = start; i < start + POINTS/2; i++) {
        phasor->real += inst[i]*sin(2*PI*(start-i)/(double)POINTS);
        phasor->img  += inst[i]*cos(2*PI*(start-i)/(double)POINTS);
    }

    // C语言语法规则: 2/400等于零!
    phasor->real = phasor->real * (4.0/(double)POINTS);
    phasor->img =  phasor->img * (4.0/(double)POINTS);
}


void inst2harmonic(double* inst, int start, Phasor* phasor, int k) {
    int i = start;
    // 因为定义的全局变量, 需要先把上次计算值清掉
    phasor->real = 0;
    phasor->img = 0;


    for (i = start; i < start + POINTS; i++) {
        phasor->real += inst[i]*sin(2*PI*k*(start-i)/(double)POINTS);
        phasor->img  += inst[i]*cos(2*PI*k*(start-i)/(double)POINTS);
    }

    // C语言语法规则: 2/400等于零!
    phasor->real = phasor->real * (2/(double)POINTS);
    phasor->img =  phasor->img * (2/(double)POINTS);
}

int phaseMatch(int phase){
    int i;
    switch(phase){
        case 0: i = 1; break;
        case 1: i = 2; break;
        case 2: i = 0; break;
    }
    return i;
}

double phasorAbs(Phasor p) {
    return sqrt(p.real*p.real + p.img*p.img);
}

/**
 * 将幅值-相角组合为相量
 **/
Phasor ampAngle2phasor(double amp, double angle) {
    Phasor Z;
    double rad;

    rad = angle / 180 * PI;

    Z.real = amp*cos(rad);
    Z.img = amp*sin(rad);

    return Z;
}

/**
 * 相量角度
 * 角度值(°):0-360
 */
double phasorAngle(Phasor p){
    double temp;

    temp = atan2(p.img, p.real) * 180.0 / PI;
    temp = temp < -0.00001 ? temp+360 : temp;

    return temp;
}


/**
 * 计算常数与相量乘积
 */
Phasor phasorNumMulti(double a, Phasor p) {
    p.real *= a;
    p.img *= a;

    return p;
}


Phasor phasorAdd(Phasor pa, Phasor pb) {
    Phasor p;

    p.real = pa.real + pb.real;
    p.img = pa.img + pb.img;

    return p;
}

Phasor phasorSub(Phasor pa, Phasor pb) {
    Phasor p;

    p.real = pa.real - pb.real;
    p.img = pa.img - pb.img;

    return p;
}

/**
 * 相量乘法
 */
Phasor phasorMulti(Phasor pa, Phasor pb){
    Phasor p;
    double a = pa.real;
    double b = pa.img;
    double c = pb.real;
    double d = pb.img;

    p.real = a * c - b * d;
    p.img = a * d + b * c;

    return p;
}


/**
 * 相量除法
 */
Phasor phasorDiv(Phasor pa, Phasor pb){
    Phasor p;
    double a = pa.real;
    double b = pa.img;
    double c = pb.real;
    double d = pb.img;

    p.real = (a * c + b * d) / (c * c + d * d);
    p.img = (b * c - a * d) / (c * c + d * d);

    return p;
}


/**
 * 逆时针旋转相量
 * @param:角度数
 */
Phasor phasorContrarotate(Phasor p, double angle) {
    Phasor newp;
    double radian;

    radian = angle / 180 * PI;

    newp.real = p.real*cos(radian) - p.img*sin(radian);
    newp.img = p.img*cos(radian) + p.real*sin(radian);

    return newp;
}


/**
 * 序量计算
 * @param:seq指示正序/负序/零序
 */
Phasor phasorSeq(Phasor pa, Phasor pb, Phasor pc, int seq) {
    Phasor seqPhasor;
    Phasor rpa;
    Phasor rpb;
    Phasor rpc;
    Phasor sum;

    if (seq == 1) {
        // 正序
        rpa = pa;
        rpb = phasorContrarotate(pb, 120.0);
        rpc = phasorContrarotate(pc, 240.0);

    } else if (seq == 2) {
        // 负序
        rpa = pa;
        rpb = phasorContrarotate(pb, 240.0);
        rpc = phasorContrarotate(pc, 120.0);

    } else if (seq == 0) {
        // 零序
        rpa = pa;
        rpb = pb; 
        rpc = pc;
    }

    sum = phasorAdd(rpa, phasorAdd(rpb, rpc));
    return phasorNumMulti(1/3.0, sum);
}


/**
 * 返回相角差
 * 注意：atan(double y, double x) 参数顺序！
 */
double phasorAngleDiff(Phasor pa, Phasor pb) {
    double ang1;
    double ang2;
    double res;

    ang1 = atan2(pa.img, pa.real) * 180.0 / PI;
    ang2 = atan2(pb.img, pb.real) * 180.0 / PI;

//    printf("%f\n", ang1);
//    printf("%f\n", ang2);

    // 转成0~360
    ang1 = ang1 < -0.0001 ? ang1+360.0 : ang1;
    ang2 = ang2 < -0.0001 ? ang2+360.0 : ang2;

    res = ang1 - ang2;

    if (res < 0) {
        res = res + 360;
    }
    return res;

}

Phasor phasorForHarmonic(double* inst, int start, int k) {

    Phasor h;
    inst2harmonic(inst, start, &h, k);

    return h;
}

/**
 * 日志模块 
 * 参考 C语言实现写入日志文件 https://blog.csdn.net/sunlion81/article/details/8647028
 */

/**
* 写入日志文件
* @param filename [in]: 日志文件名
* @param buffer [in]: 日志内容
* @param buf_size [in]: 日志内容大小
* @return 空
*/
void writeLog(Device* device, char* content) {
    if (content != NULL && notYet(device, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            { 
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ", device->time, device->deviceName);
        
                fprintf(fp, content);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;  
            }
        }
    }
}

/**
 * 写入错误日志信息
 * @param device
 * @param content
 */
void writeErrorLog(Device* device, char* content) {
    if (content != NULL && notYet(device, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            {
                fprintf(fp, "[错误] Simulation Time: %fs [%s]: ", device->time, device->deviceName);

                fprintf(fp, content);
                fprintf(fp, "\n");
                fclose(fp);
                fp = NULL;
            }
        }
    }
}


/**
 * 上面日志函数的重载形式, 主要用于相别信息
*/
void writeLogWithPhase(Device* device, char* content, int phase) {
    char formatContent[128];
    char charPhase;

    // 将相别数字转换为字母
    charPhase = (char)('A'+phase);

    // 格式化字符串
    sprintf(formatContent, content, charPhase);
    
    if (formatContent != NULL && notYet(device, formatContent)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            { 
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ",device->time, device->deviceName);
        
                fprintf(fp, formatContent);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;

            }
        }
    }
}

void busWriteLog(BusDevice* device, char* content) {
    if (content != NULL && busNotYet(device, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            { 
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ", device->relayTime, device->deviceName);
        
                fprintf(fp, content);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;  
            }
        }
    }
}

/**
 * 写入错误日志信息
 * @param device
 * @param content
 */
void busWriteErrorLog(BusDevice* device, char* content) {
    if (content != NULL && busNotYet(device, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            {
                fprintf(fp, "[错误] Simulation Time: %fs [%s]: ", device->relayTime, device->deviceName);

                fprintf(fp, content);
                fprintf(fp, "\n");
                fclose(fp);
                fp = NULL;
            }
        }
    }
}


/**
 * 上面日志函数的重载形式, 主要用于相别信息
*/
void busWriteLogWithPhase(BusDevice* device, char* content, int phase) {
    char formatContent[128];
    char charPhase;

    // 将相别数字转换为字母
    charPhase = (char)('1'+phase);

    // 格式化字符串
    sprintf(formatContent, content, charPhase);
    
    if (formatContent != NULL && busNotYet(device, formatContent)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(device->globalFileName, "at+");
            if (fp != NULL)
            { 
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ",device->relayTime, device->deviceName);
        
                fprintf(fp, formatContent);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;

            }
        }
    }
}


/**
 * hash算法
 * @param:需要求hash值的字符串
 * @param:数组长度
 */
unsigned int SDBMHash(char *str, int arrLength) {
    unsigned int hash = 0; 
    while (*str)
    {
        // equivalent to: hash = 65599*hash + (*str++);
        hash = (*str++) + (hash << 6) + (hash << 16) - hash;
    }
 
    return (hash & 0x7FFFFFFF)%arrLength;
}


/**
 * notYet函数
 * 功能:将该函数的返回值作为判别条件,可以保证条件语句内的
 * 语句仅执行一次, 使用方法如下:
 * if (notYet(device, "描述代码段的作用")) {
 *     // 执行语句...
 * }
 * 原理:利用"描述代码段的作用"作为该代码段的标识,通过hash计算基本确保唯一性
 */
int notYet(Device* device, char* str) {
    unsigned int hashCode;

    hashCode = SDBMHash(str, MAXSIZE);
    if (device->notYetFlag[hashCode] == 0) {
        device->notYetFlag[hashCode] = 1;
        return 1;
    } 
    return 0;    
}

int busNotYet(BusDevice* device, char* str) {
    unsigned int hashCode;

    hashCode = SDBMHash(str, MAXSIZE);
    if (device->notYetFlag[hashCode] == 0) {
        device->notYetFlag[hashCode] = 1;
        return 1;
    } 
    return 0;    
}

double* lineInstSearchSideM(Device line[], int phase, int lineNum) {
    switch (phase) {
        case 0: return line[lineNum].instIma;
        case 1: return line[lineNum].instImb;
        case 2: return line[lineNum].instImc;
    }
}

double* lineInstSearchSideN(Device line[], int phase, int lineNum) {
    switch (phase) {
        case 0: return line[lineNum].instIna;
        case 1: return line[lineNum].instInb;
        case 2: return line[lineNum].instInc;
    }
}

double* busInstSearch(BusDevice* bus, int phase, int brkNum) {
    switch (brkNum) {
        case 0:  
            switch (phase) {
              case 0: return bus->busTieInstIma;
              case 1: return bus->busTieInstIma;
              case 2: return bus->busTieInstIma;
            }
        case 1: 
            switch (phase) {
              case 0: return bus->busTieInstIna;
              case 1: return bus->busTieInstInb;
              case 2: return bus->busTieInstInc;
            }
        case 2:
            switch (phase) {
              case 0: return bus->busSecInstIma;
              case 1: return bus->busSecInstImb;
              case 2: return bus->busSecInstImc;
            }
        case 3:
            switch (phase) {
              case 0: return bus->busSecInstIna;
              case 1: return bus->busSecInstInb;
              case 2: return bus->busSecInstInc;
            }
    }
   
}

/**
 * 根据当前时间寻找对应记忆量
 * 想得到谁的当前时间的记忆量, 就传入谁的记忆量数组
 */
Phasor memoryPhasorValue(Device* device, Phasor* memoryPhasors){
    double t , tFault;
    int tDelta, i;

    t = device->time;
    tFault = device->startTime;
    tDelta = (int) ((t-tFault)/0.02*POINTS);
    i = tDelta % POINTS;

    return memoryPhasors[i];
}

int memoryIndex(Device* device) {
    double t , tFault;
    int tDelta, i;

    t = device->time;
    tFault = device->startTime;
    tDelta = (int) ((t-tFault)/0.02*POINTS);
    i = tDelta % POINTS;

    return i;
}

double* findInstCurrentPtr(Device* device, int phase) {
    switch (phase) {
        case 0: return device->filterIma;
        case 1: return device->filterImb;
        case 2: return device->filterImc;
    }
}

int memoryIndexForBus(BusDevice* device) {
    double t , tFault;
    int tDelta, i;

    t = device->relayTime;
    tFault = device->busStartTime[0];
    tDelta = (int) ((t-tFault)/0.02*POINTS);
    i = tDelta % POINTS;

    return i;
}
/**
 * 
 * 母线保护-线路、主变对应电气量及记忆量查找
 * 
 **/

void busLineSynCheck(BusDevice* bus, Device line[]) {
    int i, j, flag = 0;
    for (i = 0; i < MAX_LINE_NUM_220kV; i++) {
        if (line[i].time != 0) {
            if (line[i].time != bus->time) {
                line[i].busSynFlag = 0;
                bus->busDiffDelayFlag = 1;
                bus->relayTime = line[i].time;
                flag = 1;
                // if (line[i].time > 0.218)
                //     writeLog(&line[i], "该线路不同步");
                /*for (j = 0; j < 48; j++) {
                    bus->busPhasor[j] = bus->busPrePhasor[j];
                }*/
                
            } else {
                line[i].busSynFlag = 1;
                // if (line[i].time > 0.218)
                //     writeLog(&line[i], "该线路同步");
            }
        }
    }
    if (flag == 0) bus->relayTime = bus->time;
}

void busLineSynCheck_500kV(BusDevice* bus, Device line[]) {
    int i, j, flag = 0;
    for (i = 0; i < MAX_BRK_NUM_500kV; i++) {
        if (line[i].time != 0) {
            if (line[i].time != bus->time) {
                line[i].busSynFlag = 0;
                bus->busDiffDelayFlag = 1;
                bus->relayTime = line[i].time;
                flag = 1;
                // if (line[i].time > 0.218)
                //     writeLog(&line[i], "该线路不同步");
                /*for (j = 0; j < 36; j++) {
                    bus->busPhasor[j] = bus->busPrePhasor[j];
                }*/
            } else {
                line[i].busSynFlag = 1;
                // if (line[i].time > 0.218)
                //     writeLog(&line[i], "该线路同步");
            }
        } 
    }
    if (flag == 0) bus->relayTime = bus->time;

}

Phasor* findBusMemory(BusDevice* bus, int phase, int brkNo) {
    // 母联的记忆电流量的方向在差动保护计算时处理，这里只返回记忆量
    Phasor* mem;
    switch (brkNo) {
        case 0: mem = findTie1MemoryPhasor(bus, phase); break;
        case 1: mem = findTie2MemoryPhasor(bus, phase); break;
        case 2: mem = findSec1MemoryPhasor(bus, phase); break;
        case 3: mem = findSec2MemoryPhasor(bus, phase); break;
    }
    return mem;
}

Phasor* findTie1MemoryPhasor(BusDevice* bus, int phase) {
    Phasor* mem;
    switch (phase) {
        case 0: mem = busMemoryPhasor(bus, bus->busTieMemIma); break;
        case 1: mem = busMemoryPhasor(bus, bus->busTieMemImb); break;
        case 2: mem = busMemoryPhasor(bus, bus->busTieMemImc); break;
    }
    return mem;
}

Phasor* findTie2MemoryPhasor(BusDevice* bus, int phase) {
    Phasor* mem;
    switch (phase) {
        case 0: mem = busMemoryPhasor(bus, bus->busTieMemIna); break;
        case 1: mem = busMemoryPhasor(bus, bus->busTieMemInb); break;
        case 2: mem = busMemoryPhasor(bus, bus->busTieMemInc); break;
    }
    return mem;
}

Phasor* findSec1MemoryPhasor(BusDevice* bus, int phase) {
    Phasor* mem;
    switch (phase) {
        case 0: mem = busMemoryPhasor(bus, bus->busSecMemIma); break;
        case 1: mem = busMemoryPhasor(bus, bus->busSecMemImb); break;
        case 2: mem = busMemoryPhasor(bus, bus->busSecMemImc); break;
    }
    return mem;
}

Phasor* findSec2MemoryPhasor(BusDevice* bus, int phase) {
    Phasor* mem;
    switch (phase) {
        case 0: mem = busMemoryPhasor(bus, bus->busSecMemIna); break;
        case 1: mem = busMemoryPhasor(bus, bus->busSecMemInb); break;
        case 2: mem = busMemoryPhasor(bus, bus->busSecMemInc); break;
    }
    return mem;
}

Phasor* findLineMemory(BusDevice* bus, Device line[], int num, int phase, int side) {
    Phasor* mem;
    switch (side) {
        case 1: 
           switch (phase) {
               case 0: mem = lineMemoryPhasor(bus, &line[num], line[num].memoryIma); break;
               case 1: mem = lineMemoryPhasor(bus, &line[num], line[num].memoryImb); break;
               case 2: mem = lineMemoryPhasor(bus, &line[num], line[num].memoryImc); break;
            }
            break;
        case -1:
           switch (phase) {
               case 0: mem = lineMemoryPhasor(bus, &line[num], line[num].memoryIma); break;
               case 1: mem = lineMemoryPhasor(bus, &line[num], line[num].memoryImb); break;
               case 2: mem = lineMemoryPhasor(bus, &line[num], line[num].memoryImc); break;
            }
            break;
    }

    return mem;
}

Phasor* findLineRelayTimePhasor(int flag, Device* line, int index, int phase) {
    // index: 1 - In ,-1 - Im
    if (flag == 1) {
        switch (index) {
            case -1: return &line->phasor[phase + 3];
            case 1: return &line->phasor[phase + 3];
        }
    } else {
        switch (index) {
            case -1: return &line->prePhasor[phase + 3];
            case 1: return &line->prePhasor[phase + 3];
        }
    }
}

Phasor* findBusRelayTimePhasor(int flag, BusDevice* bus, int brkNum, int index, int phase) {
    if (flag == 0) {
        switch (index) {
            case -1: return &bus->busPhasor[24 + 6*brkNum + phase + 3];
            case 1: return &bus->busPhasor[6*brkNum + phase + 3];
        }
    } else {
        switch (index) {
            case -1: return &bus->busPrePhasor[24 + 6*brkNum + phase + 3];
            case 1: return &bus->busPrePhasor[6*brkNum + phase + 3];
        }
    }
}

Phasor* lineMemoryPhasor(BusDevice* bus, Device* device, Phasor* memoryPhasors) {
    double t , tFault;
    int tDelta, i;

    t = bus->relayTime;
    //tFault = device->startTime;
    tFault = bus->busStartTime[0];
    tDelta = (int) ((t-tFault)/0.02*POINTS);
    i = tDelta % POINTS;

    return &memoryPhasors[i];
}

Phasor* busMemoryPhasor(BusDevice* bus, Phasor* memoryPhasors) {
    double t , tFault;
    int tDelta, i;

    t = bus->relayTime;
    tFault = bus->busStartTime[0];
    tDelta = (int) ((t-tFault)/0.02*POINTS);
    i = tDelta % POINTS;

    return &memoryPhasors[i];
}

void findLineConnectBus(BusDevice* bus, int lineNo, int* result) {
    int i, k = 0;
    for (i = 0; i < 4; i++) {
        if (bus->busTopo[lineNo][i] != 0 && k < 2) {
            result[k] = i;
            k++;
        }
    }
}



/**
 * 录波
 * 录波数据以文本格式输出到log/<本次仿真文件夹>/<装置名-record.txt>
 */
void recordData(Device* device) {
    int len = RECORD_LENGTH;
    char recordFileName[STRING_LENGTH];
    FILE *fp;
    int i = 0;

    sprintf(recordFileName, "%s-record.txt", device->deviceFileName);
    fp = fopen(recordFileName, "at+");

    if (fp != NULL) {
        // 标题
        fprintf(fp, "[%s]录波数据---kV/kA\n", device->deviceName);
        fprintf(fp, "======TIME===========Va===========Vb============Vc=============Ia============Ib=============Ic=====\n");

        for (i = len-1; i >= 0; i--) {
            fprintf(fp, "%12.6f, %12.6f, %12.6f, %12.6f, %12.6f, %12.6f, %12.6f\n",
                    device->instTime[i],
                    device->instVma[i], device->instVmb[i], device->instVmc[i],
                    device->instIma[i], device->instImb[i], device->instImc[i]);
        }
        fclose(fp);
        fp = NULL;
    }
}

/**
 * 1.运行时手跳, 输出永久置1, 闭锁重合闸
 * 2.手合, 100ms内故障, 加速跳闸, 闭锁重合闸
 */
void finalOutput(Device *device) {
    int i = 0;

    device->outRes[0] = device->tripFlag[0];
    device->outRes[1] = device->tripFlag[1];
    device->outRes[2] = device->tripFlag[2];

    for (i = 0; i < 3; i++) {
        if (device->manTripFlag[i] == 1) {
            device->outRes[i] = 1;
            // 闭锁重合闸
            device->reCloseBlocked = 1;
        }
        // 手合状态为1(开位)且手跳为0(说明开位不是由手跳导致), 置断路器为1
        if (device->manBrkStatus[0][i] == 1 && device->manTripFlag[i] == 0) {
            device->outRes[i] = 1;
        }

    }

}




/**
 * 交换机排队及延时
 */
void tranSwitchRelay(tranDevice* trandevice, double timeD, const double* tport1, const double* tport2) {

    double min1 = trandevice->switch1DelayMin;
    double max1 = trandevice->switch1DelayMax;
    double min2 = trandevice->switch2DelayMin;
    double max2 = trandevice->switch2DelayMax;

    if (max1 - min1 < -0.0001) {
        tranWriteErrorLog(trandevice, "交换机A延时参数错误!");
    }
    if (max2 - min2 < -0.001) {
        tranWriteErrorLog(trandevice , "交换机B延时参数错误");
    }

    double random1 = min1 + rand()%200000 / 200000.0 * (max1-min1);

    // double random1 = 0.1;
    double random2 = 0; // 固定延时为3个仿真步长左右
    int i;

    int q1 = trandevice->queueLength1;
    int q2 = trandevice->queueLength2;

    // 维护指示队列长度的计数器qLength
    // 新到达的数据包放到switchQueue[qLength]位置
    // 仿真10次, 执行一次采样函数, 更新交换机队列
    if (tranUpTo5(trandevice)) {
        if (q1 < QUEUE_LENGTH) {
            // 当队列排满时, 采样值不能进入队列, 模拟丢包
            trandevice->switchQueue1[q1].delayTime = timeD + random1;
            for (i = 0; i < 12; ++i) {
                trandevice->switchQueue1[q1].frame[i] = tport1[i];
            }
            if (q1 <= QUEUE_LENGTH-2) {
                trandevice->queueLength1++;
            }
        } else {
            tranWriteLog(trandevice, "信道阻塞, 交换机A丢包");
        }

        if (q2 < QUEUE_LENGTH) {
            trandevice->switchQueue2[q2].delayTime = timeD + random2;
            for (i = 0; i < 12; ++i) {
                trandevice->switchQueue2[q2].frame[i] = tport2[i];
            }
            if (q2 <= QUEUE_LENGTH-2) {
                trandevice->queueLength2++;
            }
        } else {
            tranWriteLog(trandevice, "信道阻塞, 交换机B丢包");
        }
    }


    // 每次仿真都执行
    // 如果判定需要输出, 从队头(0索引位置)取数, 更新队列, 更新交换机端口switchPort
    if (trandevice->queueLength1 > 0 && trandevice->switchQueue1[0].delayTime <= timeD) {
        // 延时到达, 可以输出
        for (i = 0; i < 12; ++i) {
            trandevice->switchPort1[i] = trandevice->switchQueue1[0].frame[i];
        }
        // 更新队列
        for (i = 0; i < trandevice->queueLength1; i++) {
            trandevice->switchQueue1[i] = trandevice->switchQueue1[i+1];
        }
        trandevice->switchQueue1[trandevice->queueLength1].delayTime = MAX_VALUE;
        trandevice->queueLength1--;
    }

    if (trandevice->queueLength2 > 0 && trandevice->switchQueue2[0].delayTime <= timeD) {
        // 延时到达, 可以输出
        for (i = 0; i < 12; ++i) {
            trandevice->switchPort2[i] = trandevice->switchQueue2[0].frame[i];
        }
        // 更新队列
        for (i = 0; i < trandevice->queueLength2; i++) {
            trandevice->switchQueue2[i] = trandevice->switchQueue2[i+1];
        }
        trandevice->switchQueue2[trandevice->queueLength2].delayTime = MAX_VALUE;
        trandevice->queueLength2--;
    }
}
//
//
///**
// * 仿真链接函数
// * 综合以下功能
// * 初始化/仿真步长设置/跳闸指令
// */
//

void tranLinkSimulation(tranDevice* trandevice, char* deviceName, double time, int deviceEnable, double* tport1, double* tport2, double* tranTripSignal) {


    // 设置整定值
    // 设置整定值
    if (tranNotYet(trandevice, "设置保护装置名及保护定值")) {
        tranDeviceInit(trandevice, deviceName, deviceEnable, 'T');
    }

    tranSwitchRelay(trandevice, time, tport1, tport2);


    // 只有装置启用情况下才进行计算
    // 仿真程序跑10次, 进行一次采样和保护计算
    if (trandevice->deviceEnable == 1 && tranUpTo10A(trandevice) == 1) {
        tranSample(trandevice, time, trandevice->switchPort1, trandevice->switchPort2);
        tran(trandevice);
    }
    int i = 0;
    for(i=0; i<12 ; i++){
        tranTripSignal[i] = trandevice->tranProRes[i];
    }

}



void tranDeviceInit(tranDevice* trandevice, char* deviceName, int deviceEnable, char type){
    // 设置装置名
    if (deviceEnable == 0) {
        // 装置不启用
        trandevice->deviceEnable = 0;
    } else {
        trandevice->deviceEnable = 1;

        strcpy(trandevice->deviceName, deviceName);

        // 设置globalFileName和deviceFileName
        sprintf(trandevice->globalFileName, "%s/log.txt", logDirName); // 不同装置共用log.txt
        sprintf(trandevice->deviceFileName, "%s/%s", logDirName, deviceName); // 不同装置录波文件分别存放, 按装置名分开

        if (type == 'T') {
            // 读取配置文件, 设置整定值
            tranReadConfiguration(trandevice, 'T');

            // 初始化交换机队列时间延时默认值MAX_VALUE
            tranInitSwitchQueueTime(trandevice);

            // 初始化完毕,记录日志
            tranWriteLog(trandevice, "变压器保护装置初始化完毕");
        }

    }

}



int tranReadConfiguration(tranDevice* trandevice, char elementType) {
    // 双数组 参数名-参数对应
    char paramName[PARAM_COUNT][STRING_LENGTH];
    double paramValue[PARAM_COUNT];
    char fileName[STRING_LENGTH];

    int count = 0;
    int i, j;

    // 根据device中的deviceName打开对应的配置文件并进行赋值
    sprintf(fileName, "%s%s%s", "..\\config\\", trandevice->deviceName, ".conf");

    FILE *file = fopen(fileName, "r");
    if (file == NULL)
    {
        tranWriteErrorLog(trandevice, "未找到配置文件!");
        return -1;
    }

    char buf[MAX_BUF_LEN];
    int text_comment = 0;
    while (fgets(buf, MAX_BUF_LEN, file) != NULL)
    {
        Trim(buf);
        // to skip text comment with flags /* ... */
        if (buf[0] != '#' && (buf[0] != '/' || buf[1] != '/'))
        {
            if (strstr(buf, "/*") != NULL)
            {
                text_comment = 1;
                continue;
            }
            else if (strstr(buf, "*/") != NULL)
            {
                text_comment = 0;
                continue;
            }
        }
        if (text_comment == 1)
            continue;

        int buf_len = strlen(buf);
        // ignore and skip the line with first chracter '#', '=' or '/'
        if (buf_len <= 1 || buf[0] == '#' || buf[0] == '=' || buf[0] == '/')
            continue;
        buf[buf_len - 1] = '\0';

        char _paramk[MAX_KEY_LEN] = {0}, _paramv[MAX_VAL_LEN] = {0};
        int _kv = 0, _klen = 0, _vlen = 0;
        int i = 0;
        for (i = 0; i < buf_len; ++i)
        {
            if (buf[i] == ' ')
                continue;
            // scan param key name
            if (_kv == 0 && buf[i] != '=')
            {
                if (_klen >= MAX_KEY_LEN)
                    break;
                _paramk[_klen++] = buf[i];
                continue;
            }
            else if (buf[i] == '=')
            {
                _kv = 1;
                continue;
            }
            // scan param key value
            if (_vlen >= MAX_VAL_LEN || buf[i] == '#')
                break;
            _paramv[_vlen++] = buf[i];
        }
        if (strcmp(_paramk, "") == 0 || strcmp(_paramv, "") == 0)
            continue;

        sprintf(paramName[count], _paramk);
        paramValue[count] = atof(_paramv);
        count++;
        // printf("%s=%s\n", _paramk, _paramv);
    }

    if (elementType == 'T') {
        trandevice->switch1DelayMin = tranFindSetValueIndex("交换机A最小延时", paramName, paramValue, count, trandevice);
        trandevice->switch1DelayMax = tranFindSetValueIndex("交换机A最大延时", paramName, paramValue, count, trandevice);
        trandevice->switch2DelayMin = tranFindSetValueIndex("交换机B最小延时", paramName, paramValue, count, trandevice);
        trandevice->switch2DelayMax = tranFindSetValueIndex("交换机B最大延时", paramName, paramValue, count, trandevice);

        trandevice->ratedV10 = tranFindSetValueIndex("高压侧电压额定值", paramName, paramValue, count, trandevice);
        trandevice->ratedV20 = tranFindSetValueIndex("中压侧电压额定值", paramName, paramValue, count, trandevice);
        trandevice->ratedV30 = tranFindSetValueIndex("低压侧电压额定值", paramName, paramValue, count, trandevice);
        trandevice->ratedI10 = tranFindSetValueIndex("高压侧电流额定值", paramName, paramValue, count, trandevice);
        trandevice->ratedI20 = tranFindSetValueIndex("中压侧电流额定值", paramName, paramValue, count, trandevice);
        trandevice->ratedI30 = tranFindSetValueIndex("低压侧电流额定值", paramName, paramValue, count, trandevice);

        trandevice->pt1 = tranFindSetValueIndex("高压侧电压互感器变比", paramName, paramValue, count, trandevice);
        trandevice->pt2 = tranFindSetValueIndex("中压侧电压互感器变比", paramName, paramValue, count, trandevice);
        trandevice->pt3 = tranFindSetValueIndex("低压侧电压互感器变比", paramName, paramValue, count, trandevice);
        trandevice->ct1 = tranFindSetValueIndex("高压侧电流互感器变比", paramName, paramValue, count, trandevice);
        trandevice->ct2 = tranFindSetValueIndex("中压侧电流互感器变比", paramName, paramValue, count, trandevice);
        trandevice->ct3 = tranFindSetValueIndex("低压侧电流互感器变比", paramName, paramValue, count, trandevice);

        trandevice->startEnable[0] = tranFindSetValueIndex("差动速断", paramName, paramValue, count, trandevice);
        trandevice->startEnable[1] = tranFindSetValueIndex("纵差保护", paramName, paramValue, count, trandevice);
        trandevice->startEnable[2] = tranFindSetValueIndex("分相差动", paramName, paramValue, count, trandevice);
        trandevice->startEnable[3] = tranFindSetValueIndex("低压侧小区差动", paramName, paramValue, count, trandevice);
        trandevice->startEnable[4] = tranFindSetValueIndex("分侧差动", paramName, paramValue, count, trandevice);
        trandevice->startEnable[5] = tranFindSetValueIndex("CT断线闭锁差动", paramName, paramValue, count, trandevice);
//
        trandevice->tranStartSetValue[0] = tranFindSetValueIndex("稳态差动起动定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[1] = tranFindSetValueIndex("差动速断定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[2] = tranFindSetValueIndex("分侧差动起动定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[3] = tranFindSetValueIndex("工频变化量差流起动定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[4] = tranFindSetValueIndex("零序比率差动起动定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[5] = tranFindSetValueIndex("低压侧小区差动起动定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[6] = tranFindSetValueIndex("高压侧相间电流工频变化量起动定值", paramName, paramValue, count, trandevice);
        trandevice->tranStartSetValue[7] = tranFindSetValueIndex("中压侧相间电流工频变化量起动定值", paramName, paramValue, count, trandevice);
//
        trandevice->backEnable1[0] =  tranFindSetValueIndex("高压侧相间阻抗保护", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[1] =  tranFindSetValueIndex("高压侧接地阻抗保护", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[2] =  tranFindSetValueIndex("高压侧复压闭锁过流保护", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[3] =  tranFindSetValueIndex("高压侧零序过流I段", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[4] =  tranFindSetValueIndex("高压侧零序过流I段带方向", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[5] =  tranFindSetValueIndex("高压侧零序过流I段指向母线", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[6] =  tranFindSetValueIndex("高压侧零序过流II段", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[7] =  tranFindSetValueIndex("高压侧零序过流II段带方向", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[8] =  tranFindSetValueIndex("高压侧零序过流II段指向母线", paramName, paramValue, count, trandevice);
        trandevice->backEnable1[9] =  tranFindSetValueIndex("高压侧零序过流III段", paramName, paramValue, count, trandevice);

        trandevice->zph = tranFindSetValueIndex("高压侧阻抗保护Zp定值", paramName, paramValue, count, trandevice);
        trandevice->znh = tranFindSetValueIndex("高压侧阻抗保护Zn定值", paramName, paramValue, count, trandevice);
        trandevice->kh = tranFindSetValueIndex("高压侧阻抗零序补偿系数", paramName, paramValue, count, trandevice);
        trandevice->time_seth = tranFindSetValueIndex("高压侧阻抗保护延时定值", paramName, paramValue, count, trandevice);

        trandevice->zeroCur1[0] = tranFindSetValueIndex("高零序过流I段定值", paramName, paramValue, count, trandevice);
        trandevice->zeroCur1[1] = tranFindSetValueIndex("高零序过流II段定值", paramName, paramValue, count, trandevice);
        trandevice->zeroCur1[2] = tranFindSetValueIndex("高零序过流III段定值", paramName, paramValue, count, trandevice);
        trandevice->zeroTime1[0] = tranFindSetValueIndex("高零序过流I段时限", paramName, paramValue, count, trandevice);
        trandevice->zeroTime1[1] = tranFindSetValueIndex("高零序过流II段时限", paramName, paramValue, count, trandevice);
        trandevice->zeroTime1[2] = tranFindSetValueIndex("高零序过流III段时限", paramName, paramValue, count, trandevice);

        trandevice->backEnable2[0] =  tranFindSetValueIndex("中压侧相间阻抗保护", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[1] =  tranFindSetValueIndex("中压侧接地阻抗保护", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[2] =  tranFindSetValueIndex("中压侧复压闭锁过流保护", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[3] =  tranFindSetValueIndex("中压侧零序过流I段", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[4] =  tranFindSetValueIndex("中压侧零序过流I段带方向", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[5] =  tranFindSetValueIndex("中压侧零序过流I段指向母线", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[6] =  tranFindSetValueIndex("中压侧零序过流II段", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[7] =  tranFindSetValueIndex("中压侧零序过流II段带方向", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[8] =  tranFindSetValueIndex("中压侧零序过流II段指向母线", paramName, paramValue, count, trandevice);
        trandevice->backEnable2[9] =  tranFindSetValueIndex("中压侧零序过流III段", paramName, paramValue, count, trandevice);
//
        trandevice->zpv = tranFindSetValueIndex("中压侧阻抗保护Zp定值", paramName, paramValue, count, trandevice);
        trandevice->znv = tranFindSetValueIndex("中压侧阻抗保护Zn定值", paramName, paramValue, count, trandevice);
        trandevice->kv = tranFindSetValueIndex("中压侧阻抗零序补偿系数", paramName, paramValue, count, trandevice);
        trandevice->time_setv = tranFindSetValueIndex("中压侧阻抗保护延时定值", paramName, paramValue, count, trandevice);

        trandevice->zeroCur2[0] = tranFindSetValueIndex("中零序过流I段定值", paramName, paramValue, count, trandevice);
        trandevice->zeroCur2[1] = tranFindSetValueIndex("中零序过流II段定值", paramName, paramValue, count, trandevice);
        trandevice->zeroCur2[2] = tranFindSetValueIndex("中零序过流III段定值", paramName, paramValue, count, trandevice);
        trandevice->zeroTime2[0] = tranFindSetValueIndex("中零序过流I段时限", paramName, paramValue, count, trandevice);
        trandevice->zeroTime2[1] = tranFindSetValueIndex("中零序过流II段时限", paramName, paramValue, count, trandevice);
        trandevice->zeroTime2[2] = tranFindSetValueIndex("中零序过流III段时限", paramName, paramValue, count, trandevice);

        trandevice->zeroCur2[3] = tranFindSetValueIndex("公共绕组零序过流定值", paramName, paramValue, count, trandevice);
        trandevice->zeroTime2[3] = tranFindSetValueIndex("公共绕组零序过流时间", paramName, paramValue, count, trandevice);
        trandevice->zeroTrip = tranFindSetValueIndex("公共绕组零序过流保护跳闸", paramName, paramValue, count, trandevice);

        trandevice->overCur3 = tranFindSetValueIndex("低绕组过流保护", paramName, paramValue, count, trandevice);
        trandevice->lowVolt3 = tranFindSetValueIndex("低绕组过压保护", paramName, paramValue, count, trandevice);
        trandevice->overCur_set3 = tranFindSetValueIndex("低绕组过流定值", paramName, paramValue, count, trandevice);
        trandevice->lowVol_set3=  tranFindSetValueIndex("低绕组低电压闭锁定值", paramName, paramValue, count, trandevice);
        trandevice->lowTimeSet3 = tranFindSetValueIndex("低绕组复压过流时限", paramName, paramValue, count, trandevice);

        trandevice->overCur4 = tranFindSetValueIndex("低过流保护", paramName, paramValue, count, trandevice);
        trandevice->lowVolt4 = tranFindSetValueIndex("低复压过流定值", paramName, paramValue, count, trandevice);
        trandevice->overCur_set4 = tranFindSetValueIndex("低过流定值", paramName, paramValue, count, trandevice);
        trandevice->lowVol_set4 =  tranFindSetValueIndex("低低电压闭锁定值", paramName, paramValue, count, trandevice);
        trandevice->lowTimeSet4 = tranFindSetValueIndex("低复压过流时限", paramName, paramValue, count, trandevice);

        trandevice->compSet1[0] = tranFindSetValueIndex("高低电压闭锁定值", paramName, paramValue, count, trandevice);
        trandevice->compSet1[1] = tranFindSetValueIndex("高负序电压闭锁定值", paramName, paramValue, count, trandevice);
        trandevice->compSet1[2] = tranFindSetValueIndex("高复压过流定值", paramName, paramValue, count, trandevice);
        trandevice->compSet1[3] =  tranFindSetValueIndex("高复压过流时限", paramName, paramValue, count, trandevice);

        trandevice->compSet2[0] = tranFindSetValueIndex("中低电压闭锁定值", paramName, paramValue, count, trandevice);
        trandevice->compSet2[1] = tranFindSetValueIndex("中负序电压闭锁定值", paramName, paramValue, count, trandevice);
        trandevice->compSet2[2] = tranFindSetValueIndex("中复压过流定值", paramName, paramValue, count, trandevice);
        trandevice->compSet2[3] = tranFindSetValueIndex("中复压过流时限", paramName, paramValue, count, trandevice);

//

        trandevice->ratedV1 = (trandevice->ratedV10)*(double)(1000/trandevice->pt1);
        trandevice->ratedV2 = (trandevice->ratedV20)*(double)(1000/trandevice->pt2);
        trandevice->ratedV3 = (trandevice->ratedV30)*(double)(1000/trandevice->pt3);
        trandevice->ratedI1 = (trandevice->ratedI10)*(double)(1000/trandevice->ct1);
        trandevice->ratedI2 = (trandevice->ratedI20)*(double)(1000/trandevice->ct2);
        trandevice->ratedI3 = (trandevice->ratedI30)*(double)(1000/trandevice->ct3);





        tranWriteLog(trandevice, "读取变压器保护定值");
    }
    return 0;
}

/**
 * 根据target名, 返回对应的整定值的大小
 * @param target
 * @param paramName
 * @param paramValue
 * @param n
 * @return
 */
double tranFindSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, tranDevice* trandevice) {
    int i = 0;
    char errorLog[STRING_LENGTH];

    for (i = 0; i < n; i++) {
        if (strcmp(target, *(paramName+i)) == 0) {
            return paramValue[i];
        }
    }
    // 如果没有找到对应的整定值, 将错误信息记录到日志中
    sprintf(errorLog, "%s%s", target, "参数未定义!");
    tranWriteErrorLog(trandevice, errorLog);
    return 0;
}


void tranInitSwitchQueueTime(tranDevice* trandevice) {
    int i = 0;
    for (i = 0; i < QUEUE_LENGTH; i++) {
        trandevice->switchQueue1[i].delayTime = MAX_VALUE;
        trandevice->switchQueue2[i].delayTime = MAX_VALUE;
    }
}


int tranUpTo10A(tranDevice* trandevice) {
    trandevice->sampleCount1++;
    if (trandevice->sampleCount1 == 10) {
        trandevice->sampleCount1 = 0;
        return 1;
    } else {
        return 0;
    }
}

int tranUpTo5(tranDevice* trandevice) {
    trandevice->sampleCount2++;
    if (trandevice->sampleCount2 == 5) {
        trandevice->sampleCount2 = 0;
        return 1;
    } else {
        return 0;
    }
}



// 仿真采样
void tranSample(tranDevice* trandevice, double time, double* tport1, double* tport2) {
    int i = 0;

    // 更新装置时间
    trandevice->time = time;

    for (i = 0; i < 12; i++) {
        // 本侧电压电流
        trandevice->tranSample[i] = tport1[i];
        // 对侧电压电流
        trandevice->tranSample[i+12] = tport2[i];
    }
}


void tranSample2inst(tranDevice* trandevice) {
    int i = 0;
    int j = 0;

    // 更新数据
    for (i = RECORD_LENGTH-1; i >= 1; i--) {
        trandevice->instTime[i] = trandevice->instTime[i-1];

        trandevice->instVa1[i] = trandevice->instVa1[i-1];
        trandevice->instVb1[i] = trandevice->instVb1[i-1];
        trandevice->instVc1[i] = trandevice->instVc1[i-1];
        trandevice->instIa1[i] = trandevice->instIa1[i-1];
        trandevice->instIb1[i] = trandevice->instIb1[i-1];
        trandevice->instIc1[i] = trandevice->instIc1[i-1];

        trandevice->instVa2[i] = trandevice->instVa2[i-1];
        trandevice->instVb2[i] = trandevice->instVb2[i-1];
        trandevice->instVc2[i] = trandevice->instVc2[i-1];
        trandevice->instIa2[i] = trandevice->instIa2[i-1];
        trandevice->instIb2[i] = trandevice->instIb2[i-1];
        trandevice->instIc2[i] = trandevice->instIc2[i-1];

        trandevice->instVa3[i] = trandevice->instVa3[i-1];
        trandevice->instVb3[i] = trandevice->instVb3[i-1];
        trandevice->instVc3[i] = trandevice->instVc3[i-1];
        trandevice->instIa3[i] = trandevice->instIa3[i-1];
        trandevice->instIb3[i] = trandevice->instIb3[i-1];
        trandevice->instIc3[i] = trandevice->instIc3[i-1];

        trandevice->instVa4[i] = trandevice->instVa4[i-1];
        trandevice->instVb4[i] = trandevice->instVb4[i-1];
        trandevice->instVc4[i] = trandevice->instVc4[i-1];
        trandevice->instIa4[i] = trandevice->instIa4[i-1];
        trandevice->instIb4[i] = trandevice->instIb4[i-1];
        trandevice->instIc4[i] = trandevice->instIc4[i-1];


    }

    trandevice->instTime[0] = trandevice->time;

    trandevice->instVa1[0] = trandevice->tranSample[0];
    trandevice->instVb1[0] = trandevice->tranSample[1];
    trandevice->instVc1[0] = trandevice->tranSample[2];
    trandevice->instIa1[0] = trandevice->tranSample[3];
    trandevice->instIb1[0] = trandevice->tranSample[4];
    trandevice->instIc1[0] = trandevice->tranSample[5];

    trandevice->instVa2[0] = trandevice->tranSample[6];
    trandevice->instVb2[0] = trandevice->tranSample[7];
    trandevice->instVc2[0] = trandevice->tranSample[8];
    trandevice->instIa2[0] = trandevice->tranSample[9];
    trandevice->instIb2[0] = trandevice->tranSample[10];
    trandevice->instIc2[0] = trandevice->tranSample[11];

    trandevice->instVa3[0] = trandevice->tranSample[12];
    trandevice->instVb3[0] = trandevice->tranSample[13];
    trandevice->instVc3[0] = trandevice->tranSample[14];
    trandevice->instIa3[0] = trandevice->tranSample[15];
    trandevice->instIb3[0] = trandevice->tranSample[16];
    trandevice->instIc3[0] = trandevice->tranSample[17];

    trandevice->instVa4[0] = trandevice->tranSample[18];
    trandevice->instVb4[0] = trandevice->tranSample[19];
    trandevice->instVc4[0] = trandevice->tranSample[20];
    trandevice->instIa4[0] = trandevice->tranSample[21];
    trandevice->instIb4[0] = trandevice->tranSample[22];
    trandevice->instIc4[0] = trandevice->tranSample[23];


}

void tranDataFilter(tranDevice* trandevice) {

    int i = 0;
    // 滤波后数据后移
    for (i = WINDOW-1; i >= 1; i--) {

        trandevice->filterVa1m[i] = trandevice->filterVa1m[i-1];
        trandevice->filterVb1m[i] = trandevice->filterVb1m[i-1];
        trandevice->filterVc1m[i] = trandevice->filterVc1m[i-1];
        trandevice->filterIa1m[i] = trandevice->filterIa1m[i-1];
        trandevice->filterIb1m[i] = trandevice->filterIb1m[i-1];
        trandevice->filterIc1m[i] = trandevice->filterIc1m[i-1];

        trandevice->filterVa2m[i] = trandevice->filterVa2m[i-1];
        trandevice->filterVb2m[i] = trandevice->filterVb2m[i-1];
        trandevice->filterVc2m[i] = trandevice->filterVc2m[i-1];
        trandevice->filterIa2m[i] = trandevice->filterIa2m[i-1];
        trandevice->filterIb2m[i] = trandevice->filterIb2m[i-1];
        trandevice->filterIc2m[i] = trandevice->filterIc2m[i-1];

        trandevice->filterVa3m[i] = trandevice->filterVa3m[i-1];
        trandevice->filterVb3m[i] = trandevice->filterVb3m[i-1];
        trandevice->filterVc3m[i] = trandevice->filterVc3m[i-1];
        trandevice->filterIa3m[i] = trandevice->filterIa3m[i-1];
        trandevice->filterIb3m[i] = trandevice->filterIb3m[i-1];
        trandevice->filterIc3m[i] = trandevice->filterIc3m[i-1];

        trandevice->filterVa4m[i] = trandevice->filterVa4m[i-1];
        trandevice->filterVb4m[i] = trandevice->filterVb4m[i-1];
        trandevice->filterVc4m[i] = trandevice->filterVc4m[i-1];
        trandevice->filterIa4m[i] = trandevice->filterIa4m[i-1];
        trandevice->filterIb4m[i] = trandevice->filterIb4m[i-1];
        trandevice->filterIc4m[i] = trandevice->filterIc4m[i-1];

        trandevice->filterVa1[i] = trandevice->filterVa1[i-1];
        trandevice->filterVb1[i] = trandevice->filterVb1[i-1];
        trandevice->filterVc1[i] = trandevice->filterVc1[i-1];
        trandevice->filterIa1[i] = trandevice->filterIa1[i-1];
        trandevice->filterIb1[i] = trandevice->filterIb1[i-1];
        trandevice->filterIc1[i] = trandevice->filterIc1[i-1];

        trandevice->filterVa2[i] = trandevice->filterVa2[i-1];
        trandevice->filterVb2[i] = trandevice->filterVb2[i-1];
        trandevice->filterVc2[i] = trandevice->filterVc2[i-1];
        trandevice->filterIa2[i] = trandevice->filterIa2[i-1];
        trandevice->filterIb2[i] = trandevice->filterIb2[i-1];
        trandevice->filterIc2[i] = trandevice->filterIc2[i-1];

        trandevice->filterVa3[i] = trandevice->filterVa3[i-1];
        trandevice->filterVb3[i] = trandevice->filterVb3[i-1];
        trandevice->filterVc3[i] = trandevice->filterVc3[i-1];
        trandevice->filterIa3[i] = trandevice->filterIa3[i-1];
        trandevice->filterIb3[i] = trandevice->filterIb3[i-1];
        trandevice->filterIc3[i] = trandevice->filterIc3[i-1];

        trandevice->filterVa4[i] = trandevice->filterVa4[i-1];
        trandevice->filterVb4[i] = trandevice->filterVb4[i-1];
        trandevice->filterVc4[i] = trandevice->filterVc4[i-1];
        trandevice->filterIa4[i] = trandevice->filterIa4[i-1];
        trandevice->filterIb4[i] = trandevice->filterIb4[i-1];
        trandevice->filterIc4[i] = trandevice->filterIc4[i-1];



    }

    diffFilter(trandevice->filterVa1m, trandevice->instVa1);
    diffFilter(trandevice->filterVb1m, trandevice->instVb1);
    diffFilter(trandevice->filterVc1m, trandevice->instVc1);
    diffFilter(trandevice->filterIa1m, trandevice->instIa1);
    diffFilter(trandevice->filterIb1m, trandevice->instIb1);
    diffFilter(trandevice->filterIc1m, trandevice->instIc1);

    diffFilter(trandevice->filterVa2m, trandevice->instVa2);
    diffFilter(trandevice->filterVb2m, trandevice->instVb2);
    diffFilter(trandevice->filterVc2m, trandevice->instVc2);
    diffFilter(trandevice->filterIa2m, trandevice->instIa2);
    diffFilter(trandevice->filterIb2m, trandevice->instIb2);
    diffFilter(trandevice->filterIc2m, trandevice->instIc2);

    diffFilter(trandevice->filterVa3m, trandevice->instVa3);
    diffFilter(trandevice->filterVb3m, trandevice->instVb3);
    diffFilter(trandevice->filterVc3m, trandevice->instVc3);
    diffFilter(trandevice->filterIa3m, trandevice->instIa3);
    diffFilter(trandevice->filterIb3m, trandevice->instIb3);
    diffFilter(trandevice->filterIc3m, trandevice->instIc3);

    diffFilter(trandevice->filterVa4m, trandevice->instVa4);
    diffFilter(trandevice->filterVb4m, trandevice->instVb4);
    diffFilter(trandevice->filterVc4m, trandevice->instVc4);
    diffFilter(trandevice->filterIa4m, trandevice->instIa4);
    diffFilter(trandevice->filterIb4m, trandevice->instIb4);
    diffFilter(trandevice->filterIc4m, trandevice->instIc4);
//
    lowPassFilter(trandevice->filterVa1, trandevice->filterVa1m);
    lowPassFilter(trandevice->filterVb1, trandevice->filterVb1m);
    lowPassFilter(trandevice->filterVc1, trandevice->filterVc1m);
    lowPassFilter(trandevice->filterIa1, trandevice->filterIa1m);
    lowPassFilter(trandevice->filterIb1, trandevice->filterIb1m);
    lowPassFilter(trandevice->filterIc1, trandevice->filterIc1m);

    lowPassFilter(trandevice->filterVa2, trandevice->filterVa2m);
    lowPassFilter(trandevice->filterVb2, trandevice->filterVb2m);
    lowPassFilter(trandevice->filterVc2, trandevice->filterVc2m);
    lowPassFilter(trandevice->filterIa2, trandevice->filterIa2m);
    lowPassFilter(trandevice->filterIb2, trandevice->filterIb2m);
    lowPassFilter(trandevice->filterIc2, trandevice->filterIc2m);

    lowPassFilter(trandevice->filterVa3, trandevice->filterVa3m);
    lowPassFilter(trandevice->filterVb3, trandevice->filterVb3m);
    lowPassFilter(trandevice->filterVc3, trandevice->filterVc3m);
    lowPassFilter(trandevice->filterIa3, trandevice->filterIa3m);
    lowPassFilter(trandevice->filterIb3, trandevice->filterIb3m);
    lowPassFilter(trandevice->filterIc3, trandevice->filterIc3m);

    lowPassFilter(trandevice->filterVa4, trandevice->filterVa4m);
    lowPassFilter(trandevice->filterVb4, trandevice->filterVb4m);
    lowPassFilter(trandevice->filterVc4, trandevice->filterVc4m);
    lowPassFilter(trandevice->filterIa4, trandevice->filterIa4m);
    lowPassFilter(trandevice->filterIb4, trandevice->filterIb4m);
    lowPassFilter(trandevice->filterIc4, trandevice->filterIc4m);



}

void tranToPhasor(tranDevice* trandevice) {
    inst2phasor(trandevice->filterVa1, 0, &trandevice->phasor[0]);
    inst2phasor(trandevice->filterVb1, 0, &trandevice->phasor[1]);
    inst2phasor(trandevice->filterVc1, 0, &trandevice->phasor[2]);
    inst2phasor(trandevice->filterIa1, 0, &trandevice->phasor[3]);
    inst2phasor(trandevice->filterIb1, 0, &trandevice->phasor[4]);
    inst2phasor(trandevice->filterIc1, 0, &trandevice->phasor[5]);

    inst2phasor(trandevice->filterVa2, 0, &trandevice->phasor[6]);
    inst2phasor(trandevice->filterVb2, 0, &trandevice->phasor[7]);
    inst2phasor(trandevice->filterVc2, 0, &trandevice->phasor[8]);
    inst2phasor(trandevice->filterIa2, 0, &trandevice->phasor[9]);
    inst2phasor(trandevice->filterIb2, 0, &trandevice->phasor[10]);
    inst2phasor(trandevice->filterIc2, 0, &trandevice->phasor[11]);

    inst2phasor(trandevice->filterVa3, 0, &trandevice->phasor[12]);
    inst2phasor(trandevice->filterVb3, 0, &trandevice->phasor[13]);
    inst2phasor(trandevice->filterVc3, 0, &trandevice->phasor[14]);
    inst2phasor(trandevice->filterIa3, 0, &trandevice->phasor[15]);
    inst2phasor(trandevice->filterIb3, 0, &trandevice->phasor[16]);
    inst2phasor(trandevice->filterIc3, 0, &trandevice->phasor[17]);

    inst2phasor(trandevice->filterVa4, 0, &trandevice->phasor[18]);
    inst2phasor(trandevice->filterVb4, 0, &trandevice->phasor[19]);
    inst2phasor(trandevice->filterVc4, 0, &trandevice->phasor[20]);
    inst2phasor(trandevice->filterIa4, 0, &trandevice->phasor[21]);
    inst2phasor(trandevice->filterIb4, 0, &trandevice->phasor[22]);
    inst2phasor(trandevice->filterIc4, 0, &trandevice->phasor[23]);

}




/**
 * 日志模块
 * 参考 C语言实现写入日志文件 https://blog.csdn.net/sunlion81/article/details/8647028
 */

/**
* 写入日志文件
* @param filename [in]: 日志文件名
* @param buffer [in]: 日志内容
* @param buf_size [in]: 日志内容大小
* @return 空
*/
void tranWriteLog(tranDevice* trandevice, char* content) {
    if (content != NULL && tranNotYet(trandevice, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(trandevice->globalFileName, "at+");
            if (fp != NULL)
            {
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ", trandevice->time, trandevice->deviceName);

                fprintf(fp, content);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;
            }
        }
    }
}
//
///**
// * 写入错误日志信息
// * @param device
// * @param content
// */
void tranWriteErrorLog(tranDevice* trandevice, char* content) {
    if (content != NULL && tranNotYet(trandevice, content)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(trandevice->globalFileName, "at+");
            if (fp != NULL)
            {
                fprintf(fp, "[错误] Simulation Time: %fs [%s]: ", trandevice->time, trandevice->deviceName);

                fprintf(fp, content);
                fprintf(fp, "\n");
                fclose(fp);
                fp = NULL;
            }
        }
    }
}
//
//
///**
// * 上面日志函数的重载形式, 主要用于相别信息
//*/
void tranWriteLogWithPhase(tranDevice* trandevice, char* content, int phase) {
    char formatContent[128];
    char charPhase;

    // 将相别数字转换为字母
    charPhase = (char)('A'+phase);

    // 格式化字符串
    sprintf(formatContent, content, charPhase);

    if (formatContent != NULL && tranNotYet(trandevice, formatContent)) {
        // 写日志
        {
            FILE *fp;
            fp = fopen(trandevice->globalFileName, "at+");
            if (fp != NULL)
            {
                fprintf(fp, "[信息] Simulation Time: %fs [%s]: ",trandevice->time, trandevice->deviceName);

                fprintf(fp, formatContent);
                fprintf(fp, "...OK\n");
                fclose(fp);
                fp = NULL;

            }
        }
    }
}



/**
 * notYet函数
 * 功能:将该函数的返回值作为判别条件,可以保证条件语句内的
 * 语句仅执行一次, 使用方法如下:
 * if (notYet(device, "描述代码段的作用")) {
 *     // 执行语句...
 * }
 * 原理:利用"描述代码段的作用"作为该代码段的标识,通过hash计算基本确保唯一性
 */
int tranNotYet(tranDevice* trandevice, char* str) {
    unsigned int hashCode;

    hashCode = SDBMHash(str, MAXSIZE);
    if (trandevice->tranNotYetFlag[hashCode] == 0) {
        trandevice->tranNotYetFlag[hashCode] = 1;
        return 1;
    }
    return 0;
}

/**
 * 根据当前时间寻找对应记忆量
 * 想得到谁的当前时间的记忆量, 就传入谁的记忆量数组
 */
Phasor tranMemoryPhasorValue(tranDevice* trandevice, Phasor* tranMemoryPhasors, int n){
    double t , tFault;
    int tDelta, i;

    t = trandevice->time;
    tFault = trandevice->startTime[n];
    tDelta = (int) ((t-tFault)/0.02*POINTS);
    i = tDelta % POINTS;

    return tranMemoryPhasors[i];
}
//
///**
// * 录波
// * 录波数据以文本格式输出到log/<本次仿真文件夹>/<装置名-record.txt>
// */
void tranRecordData(tranDevice* trandevice) {
    int len = RECORD_LENGTH;
    char recordFileName[STRING_LENGTH];
    FILE *fp;
    int i = 0;

    sprintf(recordFileName, "%s-record.txt", trandevice->deviceFileName);
    fp = fopen(recordFileName, "at+");

    if (fp != NULL) {
        // 标题
        fprintf(fp, "[%s]录波数据---kV/kA\n", trandevice->deviceName);
        fprintf(fp, "======TIME===========Va===========Vb============Vc=============Ia============Ib=============Ic=====\n");

        for (i = len-1; i >= 0; i--) {
            fprintf(fp, "%12.6f, %12.6f, %12.6f, %12.6f, %12.6f, %12.6f, %12.6f\n",
                    trandevice->instTime[i],
                    trandevice->instVa1[i], trandevice->instVb1[i], trandevice->instVc1[i],
                    trandevice->instIa1[i], trandevice->instIb1[i], trandevice->instIc1[i]);
        }
        fclose(fp);
        fp = NULL;
    }
}


void tranHalfPhasor(tranDevice* trandevice) {
    halfWaveFourier(trandevice->filterVa1, 0, &trandevice->halfphasor[0]);
    halfWaveFourier(trandevice->filterVb1, 0, &trandevice->halfphasor[1]);
    halfWaveFourier(trandevice->filterVc1, 0, &trandevice->halfphasor[2]);
    halfWaveFourier(trandevice->filterIa1, 0, &trandevice->halfphasor[3]);
    halfWaveFourier(trandevice->filterIb1, 0, &trandevice->halfphasor[4]);
    halfWaveFourier(trandevice->filterIc1, 0, &trandevice->halfphasor[5]);
    halfWaveFourier(trandevice->filterVa2, 0, &trandevice->halfphasor[6]);
    halfWaveFourier(trandevice->filterVb2, 0, &trandevice->halfphasor[7]);
    halfWaveFourier(trandevice->filterVc2, 0, &trandevice->halfphasor[8]);
    halfWaveFourier(trandevice->filterIa2, 0, &trandevice->halfphasor[9]);
    halfWaveFourier(trandevice->filterIb2, 0, &trandevice->halfphasor[10]);
    halfWaveFourier(trandevice->filterIc2, 0, &trandevice->halfphasor[11]);
    halfWaveFourier(trandevice->filterVa3, 0, &trandevice->halfphasor[12]);
    halfWaveFourier(trandevice->filterVb3, 0, &trandevice->halfphasor[13]);
    halfWaveFourier(trandevice->filterVc3, 0, &trandevice->halfphasor[14]);
    halfWaveFourier(trandevice->filterIa3, 0, &trandevice->halfphasor[15]);
    halfWaveFourier(trandevice->filterIb3, 0, &trandevice->halfphasor[16]);
    halfWaveFourier(trandevice->filterIc3, 0, &trandevice->halfphasor[17]);
    halfWaveFourier(trandevice->filterVa4, 0, &trandevice->halfphasor[18]);
    halfWaveFourier(trandevice->filterVb4, 0, &trandevice->halfphasor[19]);
    halfWaveFourier(trandevice->filterVc4, 0, &trandevice->halfphasor[20]);
    halfWaveFourier(trandevice->filterIa4, 0, &trandevice->halfphasor[21]);
    halfWaveFourier(trandevice->filterIb4, 0, &trandevice->halfphasor[22]);
    halfWaveFourier(trandevice->filterIc4, 0, &trandevice->halfphasor[23]);

}

void tranHarmPhasor(tranDevice* trandevice){
    inst2harmonic(trandevice->filterIa1, 0, &trandevice->basePhasor[3], 1);
    inst2harmonic(trandevice->filterIa1, 0, &trandevice->secPhasor[3], 2);
    inst2harmonic(trandevice->filterIa1, 0, &trandevice->thrPhasor[3], 3);
    inst2harmonic(trandevice->filterIa1, 0, &trandevice->fifPhasor[3], 5);

    inst2harmonic(trandevice->filterIb1, 0, &trandevice->basePhasor[4], 1);
    inst2harmonic(trandevice->filterIb1, 0, &trandevice->secPhasor[4], 2);
    inst2harmonic(trandevice->filterIb1, 0, &trandevice->thrPhasor[4], 3);
    inst2harmonic(trandevice->filterIb1, 0, &trandevice->fifPhasor[4], 5);

    inst2harmonic(trandevice->filterIc1, 0, &trandevice->basePhasor[5], 1);
    inst2harmonic(trandevice->filterIc1, 0, &trandevice->secPhasor[5], 2);
    inst2harmonic(trandevice->filterIc1, 0, &trandevice->thrPhasor[5], 3);
    inst2harmonic(trandevice->filterIc1, 0, &trandevice->fifPhasor[5], 5);

}

void halfWaveInte(double* inst, int start, double* amp) {
    int i = start;
//    int reverse = start*2 + POINTS/2 - 1;
    // 因为定义的全局变量, 需要先把上次计算值清掉
    amp[0] = 0.0;

    for (i = start; i < start + POINTS/2; i++) {
        if(inst[i] > 0)
            amp[0] = amp[0] + inst[i];
        if(inst[i] <= 0)
            amp[0] = amp[0] - inst[i];

    }
    amp[0] = amp[0]*PI/(double)POINTS;

}

void tranHalfInte(tranDevice* trandevice){

    halfWaveInte(trandevice->filterIa1, 0, &trandevice->halfIa1);
    halfWaveInte(trandevice->filterIb1, 0, &trandevice->halfIb1);
    halfWaveInte(trandevice->filterIc1, 0, &trandevice->halfIc1);
    halfWaveInte(trandevice->filterIa2, 0, &trandevice->halfIa2);
    halfWaveInte(trandevice->filterIb2, 0, &trandevice->halfIb2);
    halfWaveInte(trandevice->filterIc2, 0, &trandevice->halfIc2);
    halfWaveInte(trandevice->filterIa3, 0, &trandevice->halfIa3);
    halfWaveInte(trandevice->filterIb3, 0, &trandevice->halfIb3);
    halfWaveInte(trandevice->filterIc3, 0, &trandevice->halfIc3);

    halfWaveInte(trandevice->filterIa1, POINTS, &trandevice->halfIa1m);
    halfWaveInte(trandevice->filterIb1, POINTS, &trandevice->halfIb1m);
    halfWaveInte(trandevice->filterIc1, POINTS, &trandevice->halfIc1m);
    halfWaveInte(trandevice->filterIa2, POINTS, &trandevice->halfIa2m);
    halfWaveInte(trandevice->filterIb2, POINTS, &trandevice->halfIb2m);
    halfWaveInte(trandevice->filterIc2, POINTS, &trandevice->halfIc2m);
    halfWaveInte(trandevice->filterIa3, POINTS, &trandevice->halfIa3m);
    halfWaveInte(trandevice->filterIb3, POINTS, &trandevice->halfIb3m);
    halfWaveInte(trandevice->filterIc3, POINTS, &trandevice->halfIc3m);



}

/**
 * 0/1互换
 * double形式的1.0转换为int形式的0, double形式的0.0装换为int形式的1
 */
int zeroOne(double src) {
    if (-0.001 < src && src < 0.001) {
        return 1;
    } else if (0.999 < src && src < 1.001) {
        return 0;
    } else {
        return (int)src;
    }
}

/**
 * 浮点数相等比较, 若相等返回1, 不相等返回0
 */
int doubleEqual(double a, double b) {
    if (fabs(a - b) < EPSINON) {
        return 1;
    } else {
        return 0;
    }
}


/**
 * 输入前一点和当前点的值, 如果上升沿, 置位, 否则返回原数
 * 注意: 上升沿捕捉完成后, 需要清掉, 否则可能一直动作, 失去了捕捉上升沿的效果
 */
int risingEdgeSet(int* cur, int* pre, int old, int set) {
    if (*cur - *pre == 1) {
        *cur = 0;
        *pre = 0;
        return set;
    }
    *cur = 0;
    *pre = 0;
    return old;
}


int risingEdgeSetDouble(double* cur, double* pre, int old, int set) {
    if (*cur - *pre > 0.9) {
        *cur = 0;
        *pre = 0;
        return set;
    }
    *cur = 0;
    *pre = 0;
    return old;
}

