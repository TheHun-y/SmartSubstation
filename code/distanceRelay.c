#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

#include <math.h>


double Bdcyuanjian(Device *device, Phasor a, Phasor b, Phasor c);

double Yiduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f, Phasor g);

double Erduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f);

double Gzdybhlyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f);

double Fqxzhengbiyuanjian(Device *device, double a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f, Phasor g);

double Dcgzkaifangyuanjian(Device *device, Phasor a, Phasor b, int c, double d, double t);

double Zbguoliuyuanjian(Device *device, Phasor a, double t);

double Yiduanxiangjianjuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f,Phasor g);

double Erduanxiangjianjuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f,Phasor g);

double Erduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f);

double Sanduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f);

double Sanduanxiangjianjuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f);

double Fuhexianzhixiangjianjidianqi(Device* device, Phasor a, Phasor b, Phasor c,Phasor d, Phasor e, Phasor f);

double Fuhexianzhijiedijidianqi(Device* device, Phasor a, Phasor b, Phasor c,Phasor d, Phasor e, Phasor f);

double Didianyayuanjian(Device* device, Phasor a, Phasor b, Phasor c,Phasor d, Phasor e, Phasor f);

int julipanbie(Phasor a, Phasor b, Phasor c);





/**
 * 距离保护算法
 * 接地距离保护 + 相间距离保护
 * @param device
 * @param phase
 */

/**
 * 以下仅为Demo, 需要重写, 包括接地距离+相间距离
 * 说明:
 * 1.整定值如果在dataStruct.h未定义, 暂时直接写在代码中, 到后期再进行统一
 * 2.函数具体定义可通过ctrl+<单击>查看
 * 3.该保护函数按相调用, 对于单相phase=0代表A相, 对于相间phase=0代表AB相
 * 4.对于不需要按相调用的保护, 一次判别故障后三相Flag置位即可
 */

void distanceRelay(Device *device, int phase) {


    int* tripFlagI_a;
    int* tripFlagI_b;
    int* tripFlagI_c;
    int* tripFlagII_a;
    int* tripFlagII_b;
    int* tripFlagII_c;
    int* tripFlagIII_a;
    int* tripFlagIII_b;
    int* tripFlagIII_c;

    tripFlagI_a = &device->distanceITripFlag[0];
    tripFlagI_b = &device->distanceITripFlag[1];
    tripFlagI_c = &device->distanceITripFlag[2];

    tripFlagII_a = &device->distanceIITripFlag[0];
    tripFlagII_b = &device->distanceIITripFlag[1];
    tripFlagII_c = &device->distanceIITripFlag[2];

    tripFlagIII_a = &device->distanceIIITripFlag[0];
    tripFlagIII_b = &device->distanceIIITripFlag[1];
    tripFlagIII_c = &device->distanceIIITripFlag[2];

    //左边初始化
    int YDJD = 0;
    int BDC = 0;
    int FQX = 0;
    int DC = 0;
    int ZBGL = 0;
    int BHQD = 0;
    int YDXJ = 0;
    int EDXJ = 0;
    int DHSX = 0;
    int SXHZ = 0;
    int SDJD = 0;
    int SDXJ = 0;
    int SDHZ = 0;
    int EDJD = 0;

    int FHXZXJ=0;
    int FHXZJD=0;

    int YDJD1 = 0;
    int YDXJ1 = 0;
    int EDJD1 = 0;
    int EDXJ1 =0 ;
    int SDJD1=0;
    int SDXJ1=0;

    BHQD = 1;
    SXHZ = 0;
    DHSX = 0;
    SDHZ = 0;



    //与整定值配合，默认都投入，需要整定。
    int TZDBS = device->powerSwingBlockEnable; // 振荡闭锁
    int TYDJD = device->distanceIEnable; // 接地I段
    int TYDXJ = device->distanceIEnable; // 相间I段
    int TEDJD = device->distanceIIEnable; // 接地II段
    int TEDXJ = device->distanceIIEnable; // 相间II段
    int TSCJSE = device->speedupDistanceIIEnable; // 三重加速II段
    int TSCJSS = device->speedupDistanceIIIEnable; // 三重加速III段
    int TSDJD = device->distanceIIIEnable; // 接地III段
    int TSDXJ = device->distanceIIIEnable; // 相间III段
    int DYJXL = device->voltFromLinePTEnable; // 电压取线路PT
    int FHXZ=device->loadLimitDistanceEnable; //负荷限制继电器

    double JDJLEDTimeset;
    JDJLEDTimeset = device->p2gDistanceTimeSetValue[1];          //接地距离二段时间
    double XJJLEDTimeset;
    XJJLEDTimeset = device->p2pDistanceTimeSetValue[1];          //相间距离二段时间

    double JDJLSDTimeset;
    JDJLSDTimeset = device->p2gDistanceTimeSetValue[2];          //接地距离三段时间
    double XJJLSDTimeset;
    XJJLSDTimeset = device->p2pDistanceTimeSetValue[2];          //相间距离三段时间

    int M1 = 0;
    int M2 = 0;
    int M3 = 0;
    int M4 = 0;
    int M5 = 0;
    int M6 = 0;

    int M10 = 0;
    int M11 = 0;
    int M12 = 0;
    int M13 = 0;
    int M14 = 0;
    int M15 = 0;

    int M16 = 0;
    int M17 = 0;
    int M18 = 0;
    int M19 = 0;
    int M20 = 0;
    int M21 = 0;
    int M22 = 0;

    int DDY=0;

    Phasor Ua, Ub, Ia, Ib, Uc, Ic;              //相分量
    Ua = device->phasor[0];
    Ia = device->phasor[3];
    Ub = device->phasor[1];
    Ib = device->phasor[4];
    Uc = device->phasor[2];
    Ic = device->phasor[5];




    Phasor U1, U2, U0, I1, I2, I0;             //序分量
    U1 = phasorSeq(Ua, Ub, Uc, 1);
    U2 = phasorSeq(Ua, Ub, Uc, 2);
    U0 = phasorSeq(Ua, Ub, Uc, 0);
    I1 = phasorSeq(Ia, Ib, Ic, 1);
    I2 = phasorSeq(Ia, Ib, Ic, 2);
    I0 = phasorSeq(Ia, Ib, Ic, 0);


    //不对称开放元件 开放时为1；


    Phasor Ua0, Ub0, Uc0;                 //电压记忆量

    Ua0 = memoryPhasorValue(device, device->memoryVma);
    Ub0 = memoryPhasorValue(device, device->memoryVmb);
    Uc0 = memoryPhasorValue(device, device->memoryVmc);

    Phasor deltaUopa, deltaUopb, deltaUopc, deltaUopab, deltaUopbc, deltaUopca;

    deltaUopa = phasorSub(Ua, Ua0);
    deltaUopb = phasorSub(Ub, Ub0);
    deltaUopc = phasorSub(Uc, Uc0);
    deltaUopab = phasorSub(deltaUopa, deltaUopb);
    deltaUopbc = phasorSub(deltaUopb, deltaUopc);
    deltaUopca = phasorSub(deltaUopc, deltaUopa);

    double faulttype;                                       //故障类型：1.a相接地；2.b相接地；3.c相接地；4.ab故障；5.bc故障；6.ca故障；



    Phasor Ia0, Ib0, Ic0;                 //电压记忆量

    Ia0 = memoryPhasorValue(device, device->memoryIma);
    Ib0 = memoryPhasorValue(device, device->memoryImb);
    Ic0 = memoryPhasorValue(device, device->memoryImc);

    Phasor deltaIopa, deltaIopb, deltaIopc, deltaIopab, deltaIopbc, deltaIopca;    //电流工频变化量

    deltaIopa = phasorSub(Ia, Ia0);
    deltaIopb = phasorSub(Ib, Ib0);
    deltaIopc = phasorSub(Ic, Ic0);
    deltaIopab = phasorSub(deltaIopa, deltaIopb);
    deltaIopbc = phasorSub(deltaIopb, deltaIopc);
    deltaIopca = phasorSub(deltaIopc, deltaIopa);

    //FQX动作元件

    double PHI;                        //对称故障正序电压和电流夹角
    PHI = phasorAngleDiff(U1, I1);

    Phasor Uos;
    Uos = phasorNumMulti(cos(PHI / 180 * PI), U1);      //振荡中心电压

    double UN;         //额定电压 需要给定
    UN = device->ratedSideIVoltage;
    //UN=device->;                //PT一次侧额定电压未给定 先暂且给为500kV

    double Ufi;
    Ufi = UN / 1.732 * 1.411;

    double t;
    double tFault;
    double deltat;

    t = device->time;
    tFault = device->startTime;
    deltat = t - tFault;

    double CHZ;
    CHZ=device->reCloseTimes;


   if(deltat>=0.02){

   
    Phasor U10;                       //正序记忆电压
    U10 = phasorSeq(Ua0, Ub0, Uc0, 1);


    BDC = Bdcyuanjian(device, I0, I2, I1); // 不对称故障开放
    YDJD1 = Yiduanjiedijuliyuanjian(device, Ua, Ub, Uc, Ia, Ib, Ic, U10);
    EDJD1 = Erduanjiedijuliyuanjian(device, Ua, Ub, Uc, Ia, Ib, Ic);
    faulttype = Gzdybhlyuanjian(device, deltaUopa, deltaUopb, deltaUopc, deltaUopab, deltaUopbc, deltaUopca);
    FQX = Fqxzhengbiyuanjian(device, faulttype, Ia, Ib, Ic, deltaIopab, deltaIopbc, deltaIopca);
    ZBGL = Zbguoliuyuanjian(device, I1, t - tFault);
    DC = Dcgzkaifangyuanjian(device, Uos, Ua, ZBGL, Ufi, t - tFault);

    YDXJ1 = Yiduanxiangjianjuliyuanjian(device, Ua, Ub, Uc, Ia, Ib, Ic,U10);
    EDXJ1 = Erduanxiangjianjuliyuanjian(device, Ua, Ub, Uc, Ia, Ib, Ic,U10);
    SDXJ1 = Sanduanxiangjianjuliyuanjian(device, Ua, Ub, Uc, Ia, Ib, Ic);
    SDJD1 = Sanduanjiedijuliyuanjian(device, Ua, Ub, Uc, Ia, Ib, Ic);

    FHXZXJ=Fuhexianzhixiangjianjidianqi(device,Ua,Ub,Uc,Ia,Ib,Ic);
    FHXZJD=Fuhexianzhijiedijidianqi(device,Ua,Ub,Uc,Ia,Ib,Ic);


    if((CHZ==1.0)&&((phasorAbs(Ia)>4)||(phasorAbs(Ib)>4)||(phasorAbs(Ic)>4))){
        DHSX=1;
        SXHZ=1;
    }

    if(phasorAbs(U1)<0.1*UN){
        DDY=1;
    }
    else{DDY=0;}

   YDJD=(YDJD1&&(!(FHXZ&&FHXZJD))&&!DDY);       //负荷限制闭锁 
   YDXJ=(YDXJ1&&(!(FHXZ&&FHXZXJ)));      //负荷限制闭锁

   EDJD=(EDJD1&&(!(FHXZ&&FHXZJD))&&!DDY);       //负荷限制闭锁 
   EDXJ=(EDXJ1&&(!(FHXZ&&FHXZXJ)));      //负荷限制闭锁

   SDJD=(SDJD1&&(!(FHXZ&&FHXZJD))&&!DDY);       //负荷限制闭锁 
   SDXJ=(SDXJ1&&(!(FHXZ&&FHXZXJ)));      //负荷限制闭锁

    M1 = (BDC || FQX || DC);
    M3 = (ZBGL && BHQD);

    M2 = (M1 || M3);
    M4 = ((YDJD && TYDJD)|| (YDXJ && TYDXJ));


    int ZBKF;
    if (TZDBS) { ZBKF = M2; }
    else { ZBKF = 1; }

    if (ZBKF) {
        writeLog(device, "振荡闭锁开放");
    }

    M5 = (ZBKF && M4);
    if (M5 && device->CTBreakFlag == 0) {
        if(DDY==1){
             *tripFlagI_a = 1;
              *tripFlagI_b = 1;
               *tripFlagI_c = 1;
            writeLog(device, "低压距离保护I段动作");
        }
        else{

        faulttype = Gzdybhlyuanjian(device, deltaUopa, deltaUopb, deltaUopc, deltaUopab, deltaUopbc, deltaUopca);
          if (faulttype == 7) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
          writeLog(device, "三相距离I段动作"); 
        }

       if(YDJD && TYDJD){
        if (faulttype == 1) {
            *tripFlagI_a = 1;
            writeLog(device, "a相接地距离I段动作");
        }
        if (faulttype == 2) {
            *tripFlagI_b = 1;
            writeLog(device, "b相接地距离I段动作");
        }
        if (faulttype == 3) {
            *tripFlagI_c = 1;
            writeLog(device, "c相接地距离I段动作");
        }
        if ((faulttype == 4)&&(phasorAbs(I0)>0.1)) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
            writeLog(device, "a相接地距离I段动作"); 
            writeLog(device, "b相接地距离I段动作");
        }
        if ((faulttype == 5)&&(phasorAbs(I0)>0.1)) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
            writeLog(device, "b相接地距离I段动作");
            writeLog(device, "c相接地距离I段动作");
        }
        if ((faulttype == 6)&&(phasorAbs(I0)>0.1)) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
            writeLog(device, "a相接地距离I段动作");
            writeLog(device, "c相接地距离I段动作");
        }

        }
        if(YDXJ && TYDXJ){
          if (faulttype == 4) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
            writeLog(device, "ab相间距离I段动作");
        }
        if (faulttype == 5) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
            writeLog(device, "bc相间距离I段动作");
        }
         if (faulttype == 6) {
            *tripFlagI_a = 1;
            *tripFlagI_b = 1;
            *tripFlagI_c = 1;
            writeLog(device, "ac相间距离I段动作");
        }
       

        }
        }
    }

    M6 = (EDJD && TEDJD && ZBKF);


    M11 = (EDXJ && TEDXJ && ZBKF);


    int JDJLEDTime;
    int XJJLEDTime;

    int JDJLSDTime;
    int XJJLSDTime;


    if (deltat >= JDJLEDTimeset && device->startFlag == 1) { JDJLEDTime = 1; }
    else { JDJLEDTime = 0; }

    if (deltat >= XJJLEDTimeset) { XJJLEDTime = 1; }
    else { XJJLEDTime = 0; }

    if (deltat >= JDJLSDTimeset) { JDJLSDTime = 1; }
    else { JDJLSDTime = 0; }

    if (deltat >= XJJLSDTimeset) { XJJLSDTime = 1; }
    else { XJJLSDTime = 0; }

    M10 = ((M6 && JDJLEDTime) || (M11 && XJJLEDTime));
    if(M10){
           if(DDY==1){
             *tripFlagI_a = 1;
              *tripFlagI_b = 1;
               *tripFlagI_c = 1;
            writeLog(device, "低压距离保护II段动作");
        }
        else{
         
          if (faulttype == 7) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "三相距离II段动作");    
        }

    

    if(M6&&JDJLEDTime){
             if (faulttype == 1) {
            *tripFlagII_a = 1;
            writeLog(device, "a相接地距离II段动作");}
             if (faulttype == 2) {
            *tripFlagII_b = 1;
            writeLog(device, "b相接地距离II段动作");
        }
        if (faulttype == 3) {
            *tripFlagII_c = 1;
            writeLog(device, "c相接地距离II段动作");
        }
        if ((faulttype == 4)&&(phasorAbs(I0)>0.1) ){
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "a相接地距离II段动作");
            writeLog(device, "b相接地距离II段动作");
        }

        if ((faulttype == 5)&&(phasorAbs(I0)>0.1)) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "b相接地距离II段动作");
            writeLog(device, "c相接地距离II段动作");

        }
        if ((faulttype == 6) &&(phasorAbs(I0)>0.1)){
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "c相接地距离II段动作");
            writeLog(device, "a相接地距离II段动作");
        }
      

    }

    if (M11 && XJJLEDTime) {
      
        if (faulttype == 4) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "ab相间距离II段动作");
        }

        if (faulttype == 5) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "bc相间距离II段动作");

        }
        if (faulttype == 6) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "ca相间距离II段动作");
        }
    }
    }
    }
    M12 = (M6 || M12);
    M13 = (DHSX && M12);
    M14 = (EDJD || EDXJ);
    M15 = (SDXJ || SDJD);
    M16 = ((M14 && TSCJSE) || (M15 && TSCJSS));
    M17 = (SXHZ && M16);
    M22 = (SDHZ && M15);
    M18 = (M13 || M17 || M22);
    M19 = ((!DYJXL) && M18);
    M20 = (M19 || M18);   //延时需要考虑
    if (M20) {
        if (faulttype == 1) {
            *tripFlagII_a = 1;
            writeLog(device, "a相接地距离加速动作");
        }
        if (faulttype == 2) {
            *tripFlagII_b = 1;
            writeLog(device, "b相接地距离加速动作");
        }
        if (faulttype == 3) {
            *tripFlagII_c = 1;
            writeLog(device, "c相接地距离加速动作");
        }
        if (faulttype == 4) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "ab相间距离加速动作");
        }
        if (faulttype == 5) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "bc相间距离加速动作");
        }
        if (faulttype == 6) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "ca相间距离加速动作");

        }
        if (faulttype == 7) {
            *tripFlagII_a = 1;
            *tripFlagII_b = 1;
            *tripFlagII_c = 1;
            writeLog(device, "三相距离加速动作");

        }

    }

    M21 = ((SDJD && TSDJD && JDJLSDTime) || (SDXJ && TSDXJ && XJJLSDTime));
    if (M21) {
         if(DDY==1){
             *tripFlagI_a = 1;
              *tripFlagI_b = 1;
               *tripFlagI_c = 1;
            writeLog(device, "低压距离保护III段动作");
        }
        else{
        if (faulttype == 1) {
            *tripFlagIII_a = 1;
            writeLog(device, "a相接地距离III段动作");
        }
        if (faulttype == 2) {
            *tripFlagIII_b = 1;
            writeLog(device, "b相接地距离III段动作");

        }
        if (faulttype == 3) {
            *tripFlagIII_c = 1;
            writeLog(device, "c相接地距离III段动作");

        }
        if (faulttype == 4) {
            *tripFlagIII_a = 1;
            *tripFlagIII_b = 1;
            *tripFlagIII_c = 1;
            if((SDJD && TSDJD && JDJLSDTime)&&(phasorAbs(I0)>0.1)){
              writeLog(device, "a相接地距离III段动作");  
              writeLog(device, "b相接地距离III段动作");  
            }
            if(SDXJ && TSDXJ && XJJLSDTime){
            writeLog(device, "ab相间距离III段动作");  
            }

        }

        if (faulttype == 5) {
            *tripFlagIII_a = 1;
            *tripFlagIII_b = 1;
            *tripFlagIII_c = 1;
              if((SDJD && TSDJD && JDJLSDTime)&&(phasorAbs(I0)>0.1)){
              writeLog(device, "b相接地距离III段动作");  
              writeLog(device, "c相接地距离III段动作");  
            }
             if(SDXJ && TSDXJ && XJJLSDTime){
            writeLog(device, "bc相间距离III段动作");  
            }

        }
        if (faulttype == 6) {
            *tripFlagIII_a = 1;
            *tripFlagIII_b = 1;
            *tripFlagIII_c = 1;
              if((SDJD && TSDJD && JDJLSDTime)&&(phasorAbs(I0)>0.1)){
              writeLog(device, "a相接地距离III段动作");  
              writeLog(device, "c相接地距离III段动作");  
            }
             if(SDXJ && TSDXJ && XJJLSDTime){
            writeLog(device, "ac相间距离III段动作");  
            }

        }

        if (faulttype == 7) {
            *tripFlagIII_a = 1;
            *tripFlagIII_b = 1;
            *tripFlagIII_c = 1;
            writeLog(device, "三相距离III段动作");

        }

    }
    }


}
}

double Yiduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f, Phasor g) {
    Phasor Z1set;
    Z1set = ampAngle2phasor(device->p2gDistanceSetValue[0], device->lineZ1Angle);//device->distanceSetValue[0];
    double t1set = 0.0;

    double Kset = device->KZ;  //零序补偿系数  0--2  (Z0-Z1)/3Z1

    double angle1 = device->p2gDistanceDevAngle; //接地距离正序电压极化移相角theta1
    double angle3 = device->lineZ1Angle;  //正序灵敏角  55--89度
    double angle4 = device->lineZ0Angle;  //零序灵敏角  55--89度

    Phasor I1, I2, I0, U1, U2, U0, Ub1, Uc1, Ia1, Ib1, Ic1;
    U1 = phasorSeq(a, b, c, 1);             //正序电压
    Ub1 = phasorSeq(b, c, a, 1);
    Uc1 = phasorSeq(c, a, b, 1);
    Ia1 = phasorSeq(d, e, f, 1);
    Ib1 = phasorSeq(e, f, d, 1);
    Ic1 = phasorSeq(f, d, e, 1);
    I0 = phasorSeq(d, e, f, 0);            //零序电流
    double UN;
    UN = device->ratedSideIVoltage;

    //UN =500;                //额定电压 需要整定
    Phasor Uopfi, Upfi;
    Phasor Uopfia1, Upfia1, Upfia2;
    Phasor Uopfib1, Upfib1, Upfib2;
    Phasor Uopfic1, Upfic1, Upfic2;


    Uopfia1 = phasorSub(a, phasorMulti(Z1set, phasorAdd(d, phasorNumMulti(3 * Kset, I0))));
    Upfia1 = phasorContrarotate((phasorNumMulti(-1, U1)), angle1);
    Uopfib1 = phasorSub(b, phasorMulti(Z1set, phasorAdd(e, phasorNumMulti(3 * Kset, I0))));
    Upfib1 = phasorContrarotate((phasorNumMulti(-1, Ub1)), angle1);
    Uopfic1 = phasorSub(c, phasorMulti(Z1set, phasorAdd(f, phasorNumMulti(3 * Kset, I0))));
    Upfic1 = phasorContrarotate((phasorNumMulti(-1, Uc1)), angle1);
    Upfia2 = phasorContrarotate(phasorNumMulti(-1, I0), angle4);
    Upfib2 = Upfia2;
    Upfic2 = Upfia2;

        if (julipanbie(Uopfia1, Upfia1, Upfia2)) {
            
            return 1;

        } else {
            if (julipanbie(Uopfib1, Upfib1, Upfib2)) {
              
                return 1;

            } else {
                if (julipanbie(Uopfic1, Upfic1, Upfic2)) {
                   
                    return 1;

                } else {

                    return 0;

                }

            }
        }


    

}


double Bdcyuanjian(Device *device, Phasor a, Phasor b, Phasor c) {
    double d, e, f;
    d = 0;     //判断值
    e = phasorAbs(a) + phasorAbs(b);
    f = 5 * phasorAbs(c);   //默认正序电流前面系数为5
    if (e > f) {
        d = 1;
    } else { d = 0; }
    return d;
}

double Erduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f) {
    Phasor Z2set;
    Z2set = ampAngle2phasor(device->p2gDistanceSetValue[1], device->lineZ1Angle);//device->distanceSetValue[0];
    double t2set = device->p2gDistanceTimeSetValue[1];


    double Kset = device->KZ;  //零序补偿系数  0--2  (Z0-Z1)/3Z1
    double angle1 = device->p2gDistanceDevAngle; //接地距离正序电压极化移相角theta1
    double angle3 = device->lineZ1Angle;  //正序灵敏角  55--89度
    double angle4 = device->lineZ0Angle;  //零序灵敏角  55--89度

    Phasor I1, I2, I0, U1, U2, U0;
    Phasor Ub1, Uc1;
    U1 = phasorSeq(a, b, c, 1);             //正序电压
    Ub1 = phasorSeq(b, c, a, 1);
    Uc1 = phasorSeq(c, a, b, 1);
    I0 = phasorSeq(d, e, f, 0);

    Phasor Uopfia1, Upfia1, Upfia2;
    Phasor Uopfib1, Upfib1, Upfib2;
    Phasor Uopfic1, Upfic1, Upfic2;

    Uopfia1 = phasorSub(a, phasorMulti(Z2set, phasorAdd(d, phasorNumMulti(3 * Kset, I0))));
    Upfia1 = phasorContrarotate((phasorNumMulti(-1, U1)), angle1);
    Uopfib1 = phasorSub(b, phasorMulti(Z2set, phasorAdd(e, phasorNumMulti(3 * Kset, I0))));
    Upfib1 = phasorContrarotate((phasorNumMulti(-1, Ub1)), angle1);
    Uopfic1 = phasorSub(c, phasorMulti(Z2set, phasorAdd(f, phasorNumMulti(3 * Kset, I0))));
    Upfic1 = phasorContrarotate((phasorNumMulti(-1, Uc1)), angle1);

    Upfia2 = phasorContrarotate(phasorNumMulti(-1, I0), angle3);
    Upfib2 = Upfia2;
    Upfic2 = Upfia2;


    if (julipanbie(Uopfia1, Upfia1, Upfia2)) {

        return 1;

    } else {
        if (julipanbie(Uopfib1, Upfib1, Upfib2)) {

            return 1;

        } else {
            if (julipanbie(Uopfic1, Upfic1, Upfic2)) { ;
                return 1;

            } else {
                return 0;
            }

        }
    }

}


double Gzdybhlyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f)          //
{
    Phasor Ia0,Ib0,Ic0,I00;
    Phasor Ika,Ikb,Ikc;
    Phasor Ika0,Ikb0,Ikc0;
    Phasor Uopa,Uopb,Uopc;
    Phasor Uopa0,Uopb0,Uopc0;
    Phasor deltaUa,deltaUb,deltaUc;
    Phasor Ua0,Ub0,Uc0;
    Phasor Uab,Ubc,Uca;
    Phasor Uab0,Ubc0,Uca0;
    Phasor Iab,Ibc,Ica;
    Phasor Iab0,Ibc0,Ica0;
    Phasor Uopab,Uopbc,Uopca;
    Phasor Uopab0,Uopbc0,Uopca0;
    Phasor deltaUab,deltaUbc,deltaUca;

    Phasor Ia,Ib,Ic,I0;

    Phasor Ua, Ub, Uc;            //相分量
    Ua = device->phasor[0];

    Ub = device->phasor[1];

    Uc = device->phasor[2];

    Uab=phasorSub(Ua,Ub);
    Ubc=phasorSub(Ub,Uc);
    Uca=phasorSub(Uc,Ua);

    double amp = device->deltaImpedance;
    double angle = device->lineZ1Angle;
    Phasor Zzd = ampAngle2phasor(amp, angle);
    double kz = device->KZ;
    
    Ia=  device->phasor[3];
    Ib=  device->phasor[4];
    Ic=  device->phasor[5];
    I0=phasorSeq(Ia,Ib,Ic,0);
    Iab=phasorSub(Ia,Ib);
    Ibc=phasorSub(Ib,Ic);
    Ica=phasorSub(Ic,Ia);

    Ika = phasorAdd(Ia, phasorNumMulti(3*kz, I0));
    Ikb=  phasorAdd(Ib, phasorNumMulti(3*kz, I0));
    Ikc=  phasorAdd(Ic, phasorNumMulti(3*kz, I0));

    Ia0=memoryPhasorValue(device, device->memoryIma);
    Ib0=memoryPhasorValue(device, device->memoryImb);
    Ic0=memoryPhasorValue(device, device->memoryImc);
    I00=phasorSeq(Ia0,Ib0,Ic0,0);
    Iab0=phasorSub(Ia0,Ib0);
    Ibc0=phasorSub(Ib0,Ic0);
    Ica0=phasorSub(Ic0,Ia0);
    
    Ika0 = phasorAdd(Ia0, phasorNumMulti(3*kz, I00));
    Ikb0=  phasorAdd(Ib0, phasorNumMulti(3*kz, I00));
    Ikc0=  phasorAdd(Ic0, phasorNumMulti(3*kz, I00));


    Ua0=memoryPhasorValue(device, device->memoryVma);
    Ub0=memoryPhasorValue(device, device->memoryVmb);
    Uc0=memoryPhasorValue(device, device->memoryVmc);
    Uab0=phasorSub(Ua0,Ub0);
    Ubc0=phasorSub(Ub0,Uc0);
    Uca0=phasorSub(Uc0,Ua0);

    Uopa=phasorSub(Ua, phasorMulti(Zzd, Ika));
    Uopb=phasorSub(Ub, phasorMulti(Zzd, Ikb));
    Uopc=phasorSub(Uc, phasorMulti(Zzd, Ikc));

    Uopa0=phasorSub(Ua0, phasorMulti(Zzd, Ika0));
    Uopb0=phasorSub(Ub0, phasorMulti(Zzd, Ikb0));
    Uopc0=phasorSub(Uc0, phasorMulti(Zzd, Ikc0));

    deltaUa=phasorSub(Uopa,Uopa0);
    deltaUb=phasorSub(Uopb,Uopb0);
    deltaUc=phasorSub(Uopc,Uopc0);

    Uopab=phasorSub(Uab, phasorMulti(Zzd, Iab));
    Uopbc=phasorSub(Ubc, phasorMulti(Zzd, Ibc));
    Uopca=phasorSub(Uca, phasorMulti(Zzd, Ica));

    Uopab0=phasorSub(Uab0, phasorMulti(Zzd, Iab0));
    Uopbc0=phasorSub(Ubc0, phasorMulti(Zzd, Ibc0));
    Uopca0=phasorSub(Uca0, phasorMulti(Zzd, Ica0));

    deltaUab=phasorSub(Uopab,Uopab0);
    deltaUbc=phasorSub(Uopbc,Uopbc0);
    deltaUca=phasorSub(Uopca,Uopca0);

    double g, h, i, j, k, l;
    double m;
    double UN;
    double Ufi;

    double index = 1.5;

    UN = device->ratedSideIVoltage;    //线电压需要整定
    //UN=500;   //先给定为500kV
    Ufi = UN / 1.732 * 1.414;

    Phasor U1, U2, U0;           //序分量
    U1 = phasorSeq(Ua, Ub, Uc, 1);
    U2 = phasorSeq(Ua, Ub, Uc, 2);
    U0 = phasorSeq(Ua, Ub, Uc, 0);

    Phasor I2;
    I2=phasorSeq(Ia,Ib,Ic,2);

    if ((phasorAbs(I2) <0.01) && (phasorAbs(I0) < 0.01)) {
        return 7;                           //三相短路
    } else {
        g = phasorAbs(deltaUa);
        h = phasorAbs(deltaUb);
        i = phasorAbs(deltaUc);
        j = phasorAbs(deltaUab);
        k = phasorAbs(deltaUbc);
        l = phasorAbs(deltaUca);
        if ((g >= index * h) && (g >= index * i)) {
            return 1;       //a相故障
        } else {
            if ((h >= index * g) && (h >= index * i)) { return 2; }      //b相故障
            else {
                if ((i >= index * g) && (i >=index * h)) { return 3; }       //c相故障
                else {
                    if ((j > k) && (j > l)) { return 4; }             //ab相故障
                    else {
                        if ((k > j) && (k > l)) { return 5; }              //bc相故障
                        else {
                            if ((l > k) && (l > j)) { return 6; }             //ac相故障
                            else return 0;          //无故障
                        }
                    }
                }
            }
        }
    }
}

// 非全相振闭开放元件
double Fqxzhengbiyuanjian(Device *device, double a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f, Phasor g)    //非全相振荡闭锁元件
{
    double signal = 0;

    double Iset1 = device->psbOpenPhaseSetValue[0];          //无流门槛值   0.1kA
    double Iset2 = device->psbOpenPhaseSetValue[1];       //工频变化量门槛值---需要整定  2KA
    double absb, absc, absd, abse, absf, absg;
    absb = phasorAbs(b);
    absc = phasorAbs(c);
    absd = phasorAbs(d);
    abse = phasorAbs(e);
    absf = phasorAbs(f);
    absg = phasorAbs(g);

    if (((a == 1) && (absb >= Iset1)) || ((a == 2) && (absc >= Iset1)) || ((a == 3) && (absd >= Iset1)) ||
        ((a == 4) && ((absb >= Iset1) || (absc >= Iset1))) || ((a == 5) && ((absc >= Iset1) || (absd >= Iset1))) ||
        ((a == 6) && ((absb >= Iset1) || (absd >= Iset1))) ||
        ((absb <= Iset1) && (absf >= Iset2)) || ((absc <= Iset1) && (absg >= Iset2)) ||
        ((absd <= Iset1) && (abse >= Iset2))) { signal = 1; }
    else signal = 0;

    return signal;

}


double Dcgzkaifangyuanjian(Device *device, Phasor a, Phasor b, int c, double d, double t) {
    double absa;
    double angle;
    angle = phasorAngleDiff(a, b);
    absa = phasorAbs(a);

    if (c == 1) {
        if ((absa > -0.03 * d) && (absa < 0.08 * d) && (t >= 0.31)) { return 1; }
        else {
            if ((absa > -0.1 * d) && (absa < 0.25 * d) && (t >= 0.66)) { return 1; }
            else { return 0; }

        }
    } else {
        if ((absa > -0.03 * d) && (absa < 0.08 * d) && (t >= 0.15)) { return 1; }
        else {
            if ((absa > -0.1 * d) && (absa < 0.25 * d) && (t >= 0.5)) { return 1; }
            else { return 0; }

        }
    }


}

// 振闭过流元件
double Zbguoliuyuanjian(Device *device, Phasor a, double t) {
    double absa, Iset;
    absa = phasorAbs(a);
    Iset = device->psbCurrentSetValue;       //振闭过流定值 需要整定；
    if ((((absa > Iset) && (t <= 0.01)) || (absa <= Iset)) && (t <= 0.16))
        return 1;
    else
        return 0;
}

// 相间I段
double Yiduanxiangjianjuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f,Phasor g) {

    double t1set = 0;                 ////距离保护I段时间定值
    Phasor Z1set;
    Z1set = ampAngle2phasor(device->p2pDistanceSetValue[0], device->lineZ1Angle);

    double angle1 = device->p2pDistanceDevAngle;               //相间距离正序电压极化移相角theta2
    double angle3 = device->lineZ1Angle;                //正序灵敏角  55--89度

    Phasor Uopab, Uopbc, Uopca;
    Phasor Upab1, Upbc1, Upca1, Upab2, Upbc2, Upca2;
    Phasor Uab, Ubc, Uca, Iab, Ibc, Ica;
    Phasor U1ab, U1bc, U1ca;
    Phasor Cankao;
    Cankao.real = 1;
    Cankao.img = 0;
    double absUopab1, angleUopab1;
    double absUpab1, angleUpab1;
    double absUpab2, angleUpab2;

    double absUopbc1, angleUopbc1;
    double absUpbc1, angleUpbc1;
    double absUpbc2, angleUpbc2;

    double absUopca1, angleUopca1;
    double absUpca1, angleUpca1;
    double absUpca2, angleUpca2;


    Uab = phasorSub(a, b);
    Ubc = phasorSub(b, c);
    Uca = phasorSub(c, a);

    Iab = phasorSub(d, e);
    Ibc = phasorSub(e, f);
    Ica = phasorSub(f, d);


    U1ab = phasorSub((phasorSeq(a, b, c, 1)), (phasorSeq(b, c, a, 1)));
    U1bc = phasorSeq(Ubc, Uca, Uab, 1);
    U1ca = phasorSeq(Uca, Uab, Ubc, 1);

    Uopab = phasorSub(Uab, phasorMulti(Z1set, Iab));
    Uopbc = phasorSub(Ubc, phasorMulti(Z1set, Ibc));
    Uopca = phasorSub(Uca, phasorMulti(Z1set, Ica));

    Upab1 = phasorContrarotate(phasorNumMulti(-1, U1ab), angle1);
    Upbc1 = phasorContrarotate(phasorNumMulti(-1, U1bc), angle1);
    Upca1 = phasorContrarotate(phasorNumMulti(-1, U1ca), angle1);

    Upab2 = phasorContrarotate(phasorNumMulti(-1, Iab), angle3);
    Upbc2 = phasorContrarotate(phasorNumMulti(-1, Ibc), angle3);
    Upca2 = phasorContrarotate(phasorNumMulti(-1, Ica), angle3);

    absUopab1 = phasorAbs(Uopab);
    angleUopab1 = phasorAngleDiff(Uopab, Cankao);
    absUpab1 = phasorAbs(Upab1);
    angleUpab1 = phasorAngleDiff(Upab1, Cankao);
    absUpab2 = phasorAbs(Upab2);
    angleUpab2 = phasorAngleDiff(Upab2, Cankao);
     
    double UN;
    UN = device->ratedSideIVoltage;

    //UN =500;                //额定电压 需要整定
     Phasor U1;
      U1=phasorSeq(a,b,c,1);
      if(phasorAbs(U1)<0.1*UN){

      Phasor Uopfi,Upfi;
        Uopfi = phasorSub(a, (phasorMulti(Z1set, d)));
        Upfi = phasorNumMulti(-1, g);
        if (julipanbie(Uopfi, Upfi, Upfi)) {
            
            return 1;
        } else {
            return 0;
        }
      }
else{
    if (julipanbie(Uopab, Upab1, Upab2)) {
        return 1;
    } else {
        if (julipanbie(Uopbc, Upbc1, Upbc2)) {

            return 1;

        } else {
            if (julipanbie(Uopca, Upca1, Upca2)) {

                return 1;

            } else {
                return 0;
            }

        }

    }
}


}

// 相间II段
double Erduanxiangjianjuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f,Phasor g) {

    ////相间距离保护II段时间定值
    Phasor Z2set;
    Z2set = ampAngle2phasor(device->p2pDistanceSetValue[1], device->lineZ1Angle);
    double angle1 = device->p2pDistanceDevAngle;               //相间距离正序电压极化移相角theta2
    double angle3 = device->lineZ1Angle;                //正序灵敏角  55--89度

    Phasor Uopab, Uopbc, Uopca;
    Phasor Upab1, Upbc1, Upca1, Upab2, Upbc2, Upca2;
    Phasor Uab, Ubc, Uca, Iab, Ibc, Ica;
    Phasor U1ab, U1bc, U1ca;

    Uab = phasorSub(a, b);
    Ubc = phasorSub(b, c);
    Uca = phasorSub(c, a);

    Iab = phasorSub(d, e);
    Ibc = phasorSub(e, f);
    Ica = phasorSub(f, d);

    U1ab = phasorSeq(Uab, Ubc, Uca, 1);
    U1bc = phasorSeq(Ubc, Uca, Uab, 1);
    U1ca = phasorSeq(Uca, Uab, Ubc, 1);

    Uopab = phasorSub(Uab, phasorMulti(Z2set, Iab));
    Uopbc = phasorSub(Ubc, phasorMulti(Z2set, Ibc));
    Uopca = phasorSub(Uca, phasorMulti(Z2set, Ica));

    Upab1 = phasorContrarotate(phasorNumMulti(-1, U1ab), angle1);
    Upbc1 = phasorContrarotate(phasorNumMulti(-1, U1bc), angle1);
    Upca1 = phasorContrarotate(phasorNumMulti(-1, U1ca), angle1);

    Upab2 = phasorContrarotate(phasorNumMulti(-1, Iab), angle3);
    Upbc2 = phasorContrarotate(phasorNumMulti(-1, Ibc), angle3);
    Upca2 = phasorContrarotate(phasorNumMulti(-1, Ica), angle3);

    double UN;
    UN = device->ratedSideIVoltage;

    //UN =500;                //额定电压 需要整定
     Phasor U1;
      U1=phasorSeq(a,b,c,1);
      if(phasorAbs(U1)<0.1*UN){
      Phasor Uopfi,Upfi;
        Uopfi = phasorSub(a, (phasorMulti(Z2set, d)));
        Upfi = phasorNumMulti(-1, g);
        if (julipanbie(Uopfi, Upfi, Upfi)) {
     
            return 1;
        } else {
            return 0;
        }
      }
else{

    if (julipanbie(Uopab, Upab1, Upab2)) {

        return 1;

    } else {
        if (julipanbie(Uopbc, Upbc1, Upbc2)) {

            return 1;

        } else {
            if (julipanbie(Uopca, Upca1, Upca2)) {
                return 1;

            } else {
                return 0;
            }
        }
    }

}
}


// 接地III段
double Sanduanjiedijuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f) {

    Phasor Z3set;
    Z3set = ampAngle2phasor(device->p2gDistanceSetValue[2], device->lineZ1Angle);
    double t3set = device->p2gDistanceTimeSetValue[2];

    double Kset = device->KZ;  //零序补偿系数  0--2
    Phasor I1, I2, I0, U1, U2, U0;
    Phasor Ub1, Uc1;
    U1 = phasorSeq(a, b, c, 1);             //正序电压
    Ub1 = phasorSeq(b, c, a, 1);
    Uc1 = phasorSeq(c, a, b, 1);
    I0 = phasorSeq(d, e, f, 0);

    Phasor Uopfia1, Upfia1;
    Phasor Uopfib1, Upfib1;
    Phasor Uopfic1, Upfic1;


    Uopfia1 = phasorSub(a, phasorMulti(Z3set, phasorAdd(d, phasorNumMulti(3 * Kset, I0))));
    Upfia1 = phasorNumMulti(-1, U1);
    Uopfib1 = phasorSub(b, phasorMulti(Z3set, phasorAdd(e, phasorNumMulti(3 * Kset, I0))));
    Upfib1 = phasorNumMulti(-1, Ub1);
    Uopfic1 = phasorSub(c, phasorMulti(Z3set, phasorAdd(f, phasorNumMulti(3 * Kset, I0))));
    Upfic1 = phasorNumMulti(-1, Uc1);

    if (julipanbie(Uopfia1, Upfia1, Upfia1)) {

        return 1;

    } else {
        if (julipanbie(Uopfib1, Upfib1, Upfib1)) {

            return 1;

        } else {
            if (julipanbie(Uopfic1, Upfic1, Upfic1)) {

                return 1;

            } else {
                return 0;
            }

        }
    }

}

// 相间III段
double Sanduanxiangjianjuliyuanjian(Device *device, Phasor a, Phasor b, Phasor c, Phasor d, Phasor e, Phasor f) {

    double t3set = device->p2pDistanceTimeSetValue[2];                 ////相间距离保护III段时间定值
    Phasor Z3set;
    Z3set = ampAngle2phasor(device->p2pDistanceSetValue[2], device->lineZ1Angle);          //相间距离保护III段整定值
    double angle1 = device->p2pDistanceDevAngle;               //相间距离正序电压极化移相角theta2
    double angle3 = device->lineZ1Angle;                //正序灵敏角  55--89度

    Phasor Uopab, Uopbc, Uopca;
    Phasor Upab1, Upbc1, Upca1;
    Phasor Uab, Ubc, Uca, Iab, Ibc, Ica;
    Phasor U1ab, U1bc, U1ca;

    Uab = phasorSub(a, b);
    Ubc = phasorSub(b, c);
    Uca = phasorSub(c, a);

    Iab = phasorSub(d, e);
    Ibc = phasorSub(e, f);
    Ica = phasorSub(f, d);

    U1ab = phasorSeq(Uab, Ubc, Uca, 1);
    U1bc = phasorSeq(Ubc, Uca, Uab, 1);
    U1ca = phasorSeq(Uca, Uab, Ubc, 1);

    Uopab = phasorSub(Uab, phasorMulti(Z3set, Iab));
    Uopbc = phasorSub(Ubc, phasorMulti(Z3set, Ibc));
    Uopca = phasorSub(Uca, phasorMulti(Z3set, Ica));

    Upab1 = phasorNumMulti(-1, U1ab);
    Upbc1 = phasorNumMulti(-1, U1bc);
    Upca1 = phasorNumMulti(-1, U1ca);

    double UN;   
    UN = device->ratedSideIVoltage;

    //UN =500;                //额定电压 需要整定
     Phasor U1;
      U1=phasorSeq(a,b,c,1);
      if(phasorAbs(U1)<0.1*UN){
      Phasor Uopfi,Upfi;
      Phasor Uopfi2;
        Uopfi = phasorAdd(a, (phasorMulti(Z3set, d)));
        Uopfi2=phasorSub(a, (phasorMulti(Z3set, d)));
        
        Upfi = phasorNumMulti(-1, a);
        if (julipanbie(Uopfi, Upfi, Upfi)||(julipanbie(Uopfi2,Upfi,Upfi))) {
         writeLog(device, "低压距离保护元件满足条件");
            return 1;
        } else {
            return 0;
        }
      }
else{


    if (julipanbie(Uopab, Upab1, Upab1)) {

        return 1;


    } else {
        if (julipanbie(Uopbc, Upbc1, Upbc1)) {

            return 1;

        } else {
            if (julipanbie(Uopca, Upca1, Upca1)) {

                return 1;

            } else {
                return 0;
            }
        }
    }
}

}


int julipanbie(Phasor a, Phasor b, Phasor c) {

    int M1;
    int M2;

    double angle1;
    double angle2;
    angle1 = phasorAngleDiff(a, b);
    angle2 = phasorAngleDiff(a, c);

    if ((((angle1 >= 0) && (angle1 < 90)) || ((angle1 > 270) && (angle1 <= 360))) ||
        (((angle1 <= 0) && (angle1 > -90)) || ((angle1 < -270) && (angle1 >= -360)))) {
        M1 = 1;

    } else { M1 = 0; }

    if ((((angle2 >= 0) && (angle2 < 90)) || ((angle2 > 270) && (angle2 <= 360))) ||
        (((angle2 <= 0) && (angle2 > -90)) || ((angle2 < -270) && (angle2 >= -360)))) {
        M2 = 1;

    } else { M2 = 0; }

    if (M1 && M2) {
        return 1;
    } else
        return 0;

}


double Fuhexianzhixiangjianjidianqi(Device* device, Phasor a, Phasor b, Phasor c,Phasor d, Phasor e, Phasor f){
Phasor Uab,Ubc,Uca;
Phasor Iab,Ibc,Ica;
Phasor Zmab,Zmbc,Zmca;
double argab1,argbc1,argca1,argab2,argbc2,argca2;

Phasor Rzd;
     Rzd.real = device->loadLimitSetValue;
     Rzd.img = 0.0;

Phasor fuRzd;
fuRzd.real = -1*(device->loadLimitSetValue);
     fuRzd.img = 0.0;

double theta = device->lineZ1Angle;

Uab=phasorSub(a,b);
Ubc=phasorSub(b,c);
Uca=phasorSub(c,a);

Iab=phasorSub(d,e);
Ibc=phasorSub(e,f);
Ica=phasorSub(f,d);

Zmab=phasorDiv(Uab,Iab);
Zmbc=phasorDiv(Ubc,Ibc);
Zmca=phasorDiv(Uca,Ica);

argab1=phasorAngle(phasorSub(Zmab,Rzd));
argab2=phasorAngle(phasorSub(Zmab,fuRzd));
argbc1=phasorAngle(phasorSub(Zmbc,Rzd));
argbc2=phasorAngle(phasorSub(Zmbc,fuRzd));
argca1=phasorAngle(phasorSub(Zmca,Rzd));
argca2=phasorAngle(phasorSub(Zmca,fuRzd));

if((theta<=argab1<=(180+theta))&&((0<=argab2<=theta)||((180+theta)<=argab2<=360))){
    return 0;
}
else{
if((theta<=argbc1<=(180+theta))&&((0<=argbc2<=theta)||((180+theta)<=argbc2<=360)))
    return 0;

else{
    if((theta<=argca1<=(180+theta))&&((0<=argca2<=theta)||((180+theta)<=argca2<=360)))
    return 0;
    else{
        return 1;
    }
}
}


}


double Fuhexianzhijiedijidianqi(Device* device, Phasor a, Phasor b, Phasor c,Phasor d, Phasor e, Phasor f){
Phasor Ika,Ikb,Ikc,I0;

Phasor Zma,Zmb,Zmc;
double arga1,argb1,argc1,arga2,argb2,argc2;

Phasor Rzd;
     Rzd.real = device->loadLimitSetValue;
     Rzd.img = 0.0;

Phasor fuRzd;
fuRzd.real = -1*(device->loadLimitSetValue);
     fuRzd.img = 0.0;

double theta = device->lineZ1Angle;

double k = device->KZ;

I0=phasorSeq(d,e,f,0);
Ika = phasorAdd(d, phasorNumMulti(3.0*k, I0));
Ikb = phasorAdd(e, phasorNumMulti(3.0*k, I0));
Ikc = phasorAdd(f, phasorNumMulti(3.0*k, I0));



Zma=phasorDiv(a,Ika);
Zmb=phasorDiv(b,Ikb);
Zmc=phasorDiv(c,Ikc);

arga1=phasorAngle(phasorSub(Zma,Rzd));
arga2=phasorAngle(phasorSub(Zma,fuRzd));
argb1=phasorAngle(phasorSub(Zmb,Rzd));
argb2=phasorAngle(phasorSub(Zmb,fuRzd));
argc1=phasorAngle(phasorSub(Zmc,Rzd));
argc2=phasorAngle(phasorSub(Zmc,fuRzd));

if((theta<=arga1<=(180+theta))&&((0<=arga2<=theta)||((180+theta)<=arga2<=360))){
    return 0;
}
else{
if((theta<=argb1<=(180+theta))&&((0<=argb2<=theta)||((180+theta)<=argb2<=360)))
    return 0;

else{
    if((theta<=argc1<=(180+theta))&&((0<=argc2<=theta)||((180+theta)<=argc2<=360)))
    return 0;
    else{
        return 1;
    }
}
}


}

