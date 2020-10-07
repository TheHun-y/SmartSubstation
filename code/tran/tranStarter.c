#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
void calIdmax(tranDevice* trandevice);

void tranLonDiff(tranDevice* trandevice);

void tranDeltaDiffStart(tranDevice* trandevice);
void tranDeltaCal(tranDevice* trandevice);

void tranRecordMemory(tranDevice* trandevice);

void recordMemoryhalf(tranDevice* trandevice);

void tranFre(tranDevice* trandevice);

void tranCalFre(tranDevice* trandevice);

void overExc(tranDevice* trandevice);

void zeroStart(tranDevice* trandevice);

void impStarth(tranDevice* trandevice);
void impStartv(tranDevice* trandevice);

void lonDiffStart(tranDevice* trandevice);

void splitStart(tranDevice* trandevice);

void zeroCurStarth(tranDevice* trandevice);
void zeroCurStartv(tranDevice* trandevice);
void zeroCurStartt(tranDevice* trandevice);

void phaseCurStarth(tranDevice* trandevice);
void phaseCurStartv(tranDevice* trandevice);
void phaseCurStartl1(tranDevice* trandevice);
void phaseCurStartl2(tranDevice* trandevice);

void calIdmax(tranDevice* trandevice){

    double voltageH = trandevice->ratedV1;
    double voltageV = trandevice->ratedV2;
    double voltageL = trandevice->ratedV3;

    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Ial, Ibl, Icl;
    Phasor Ida, Idb, Idc;
    double Idmax;


    Iah = phasorSub(trandevice->phasor[3],trandevice->phasor[5]);
    Ibh = phasorSub(trandevice->phasor[4],trandevice->phasor[3]);
    Ich = phasorSub(trandevice->phasor[5],trandevice->phasor[4]);

    Iav = phasorSub(trandevice->phasor[9],trandevice->phasor[11]);
    Ibv = phasorSub(trandevice->phasor[10],trandevice->phasor[9]);
    Icv = phasorSub(trandevice->phasor[11],trandevice->phasor[10]);

    Iav = phasorNumMulti((voltageV / voltageH), Iav);
    Ibv = phasorNumMulti((voltageV / voltageH), Ibv);
    Icv = phasorNumMulti((voltageV / voltageH), Icv);

    Ial = phasorNumMulti((voltageL / voltageH), trandevice->phasor[15]);
    Ibl = phasorNumMulti((voltageL / voltageH), trandevice->phasor[16]);
    Icl = phasorNumMulti((voltageL / voltageH), trandevice->phasor[17]);

    Ida = phasorAdd(Iah, Iav);
    Ida = phasorAdd(Ida, Ial);
    Idb = phasorAdd(Ibh, Ibv);
    Idb = phasorAdd(Idb, Ibl);
    Idc = phasorAdd(Ich, Icv);
    Idc = phasorAdd(Idc, Icl);

    Idmax = phasorAbs(Ida);
    if (phasorAbs(Idb) > Idmax) {
        Idmax = phasorAbs(Idb);
    }
    if (phasorAbs(Idc) > Idmax) {
        Idmax = phasorAbs(Idc);
    }

    trandevice->Idt = Idmax;
    trandevice->Idta = phasorAbs(Ida);
    trandevice->Idtb = phasorAbs(Idb);
    trandevice->Idtc = phasorAbs(Idc);

    trandevice->tranProRes[11] = Idmax;
}



void tranRecordMemory(tranDevice* trandevice) {
    // 记录当前时刻
    int i;
    for (i = 3*POINTS; i > 2*POINTS; i--){
        inst2phasor(trandevice->filterVa1, i, &trandevice->memoryVa1[3*POINTS-i]);  inst2phasor(trandevice->filterIa1, i, &trandevice->memoryIa1[3*POINTS-i]);
        inst2phasor(trandevice->filterVb1, i, &trandevice->memoryVb1[3*POINTS-i]);  inst2phasor(trandevice->filterIb1, i, &trandevice->memoryIb1[3*POINTS-i]);
        inst2phasor(trandevice->filterVc1, i, &trandevice->memoryVc1[3*POINTS-i]);  inst2phasor(trandevice->filterIc1, i, &trandevice->memoryIc1[3*POINTS-i]);

        inst2phasor(trandevice->filterVa2, i, &trandevice->memoryVa2[3*POINTS-i]);  inst2phasor(trandevice->filterIa2, i, &trandevice->memoryIa2[3*POINTS-i]);
        inst2phasor(trandevice->filterVb2, i, &trandevice->memoryVb2[3*POINTS-i]);  inst2phasor(trandevice->filterIb2, i, &trandevice->memoryIb2[3*POINTS-i]);
        inst2phasor(trandevice->filterVc2, i, &trandevice->memoryVc2[3*POINTS-i]);  inst2phasor(trandevice->filterIc2, i, &trandevice->memoryIc2[3*POINTS-i]);

        inst2phasor(trandevice->filterVa3, i, &trandevice->memoryVa3[3*POINTS-i]);  inst2phasor(trandevice->filterIa3, i, &trandevice->memoryIa3[3*POINTS-i]);
        inst2phasor(trandevice->filterVb3, i, &trandevice->memoryVb3[3*POINTS-i]);  inst2phasor(trandevice->filterIb3, i, &trandevice->memoryIb3[3*POINTS-i]);
        inst2phasor(trandevice->filterVc3, i, &trandevice->memoryVc3[3*POINTS-i]);  inst2phasor(trandevice->filterIc3, i, &trandevice->memoryIc3[3*POINTS-i]);

        inst2phasor(trandevice->filterVa4, i, &trandevice->memoryVa4[3*POINTS-i]);  inst2phasor(trandevice->filterIa4, i, &trandevice->memoryIa4[3*POINTS-i]);
        inst2phasor(trandevice->filterVb4, i, &trandevice->memoryVb4[3*POINTS-i]);  inst2phasor(trandevice->filterIb4, i, &trandevice->memoryIb4[3*POINTS-i]);
        inst2phasor(trandevice->filterVc4, i, &trandevice->memoryVc4[3*POINTS-i]);  inst2phasor(trandevice->filterIc4, i, &trandevice->memoryIc4[3*POINTS-i]);

    }
}


void recordMemoryhalf(tranDevice* trandevice) {
    // 记录当前时刻
    int i;
    int k = 3;
    for (i = 3*POINTS; i > 2*POINTS; i--){
        halfWaveFourier(trandevice->filterVa1, i, &trandevice->halfmemoryVa1[k*POINTS-i]);  halfWaveFourier(trandevice->filterIa1, i, &trandevice->halfmemoryIa1[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVb1, i, &trandevice->halfmemoryVb1[k*POINTS-i]);  halfWaveFourier(trandevice->filterIb1, i, &trandevice->halfmemoryIb1[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVc1, i, &trandevice->halfmemoryVc1[k*POINTS-i]);  halfWaveFourier(trandevice->filterIc1, i, &trandevice->halfmemoryIc1[k*POINTS-i]);

        halfWaveFourier(trandevice->filterVa2, i, &trandevice->halfmemoryVa2[k*POINTS-i]);  halfWaveFourier(trandevice->filterIa2, i, &trandevice->halfmemoryIa2[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVb2, i, &trandevice->halfmemoryVb2[k*POINTS-i]);  halfWaveFourier(trandevice->filterIb2, i, &trandevice->halfmemoryIb2[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVc2, i, &trandevice->halfmemoryVc2[k*POINTS-i]);  halfWaveFourier(trandevice->filterIc2, i, &trandevice->halfmemoryIc2[k*POINTS-i]);

        halfWaveFourier(trandevice->filterVa3, i, &trandevice->halfmemoryVa3[k*POINTS-i]);  halfWaveFourier(trandevice->filterIa3, i, &trandevice->halfmemoryIa3[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVb3, i, &trandevice->halfmemoryVb3[k*POINTS-i]);  halfWaveFourier(trandevice->filterIb3, i, &trandevice->halfmemoryIb3[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVc3, i, &trandevice->halfmemoryVc3[k*POINTS-i]);  halfWaveFourier(trandevice->filterIc3, i, &trandevice->halfmemoryIc3[k*POINTS-i]);

        halfWaveFourier(trandevice->filterVa4, i, &trandevice->halfmemoryVa4[k*POINTS-i]);  halfWaveFourier(trandevice->filterIa4, i, &trandevice->halfmemoryIa4[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVb4, i, &trandevice->halfmemoryVb4[k*POINTS-i]);  halfWaveFourier(trandevice->filterIb4, i, &trandevice->halfmemoryIb4[k*POINTS-i]);
        halfWaveFourier(trandevice->filterVc4, i, &trandevice->halfmemoryVc4[k*POINTS-i]);  halfWaveFourier(trandevice->filterIc4, i, &trandevice->halfmemoryIc4[k*POINTS-i]);

    }
}

void tranLonDiff(tranDevice* trandevice){
    //稳态差动起动值
    double Icdqd = trandevice->tranStartSetValue[0];
    double Idmax;
    double time;

    time = trandevice->time;

    Idmax = trandevice->Idt;

    if (Idmax > Icdqd && time > 0.02) {
        trandevice->lonDiffStartFlag = 1;
        trandevice->startTime[0] = trandevice->time;
        tranWriteLog(trandevice, "稳态差流起动");
    }
}



void tranDeltaDiffStart(tranDevice* trandevice){
    //稳态差动起动值
    double time = trandevice->time;
    double Idtset = 1.25*(trandevice->tranStartSetValue[2]);

    double voltageH = trandevice->ratedV1;
    double voltageV = trandevice->ratedV2;
    double voltageL = trandevice->ratedV3;

    double Ida_base, Idb_base, Idc_base;
    double Ira_base, Irb_base, Irc_base;

    double Ifah, Ifbh, Ifch;
    double Ifav, Ifbv, Ifcv;
    double Ifal, Ifbl, Ifcl;
    double Ifda, Ifdb, Ifdc;

    Ifah = trandevice->halfIa1[0] - trandevice->halfIc1[0];
    Ifbh = trandevice->halfIb1[0] - trandevice->halfIa1[0];
    Ifch = trandevice->halfIc1[0] - trandevice->halfIb1[0];

    Ifav = trandevice->halfIa2[0] - trandevice->halfIc2[0];
    Ifbv = trandevice->halfIb2[0] - trandevice->halfIa2[0];
    Ifcv = trandevice->halfIc2[0] - trandevice->halfIb2[0];

    Ifav = (double)(voltageV / voltageH)* Ifav;
    Ifbv = (double)(voltageV / voltageH)* Ifbv;
    Ifcv = (double)(voltageV / voltageH)* Ifcv;

    Ifal = (double)(voltageL / voltageH)* trandevice->halfIa3[0];
    Ifbl = (double)(voltageL / voltageH)* trandevice->halfIb3[0];
    Ifcl = (double)(voltageL / voltageH)* trandevice->halfIc3[0];

    Ifah = fabs(Ifah);
    Ifbh = fabs(Ifbh);
    Ifch = fabs(Ifch);
    Ifav = fabs(Ifav);
    Ifbv = fabs(Ifbv);
    Ifcv = fabs(Ifcv);
    Ifal = fabs(Ifal);
    Ifbl = fabs(Ifbl);
    Ifcl = fabs(Ifcl);

    Ifda = Ifah + Ifav + Ifal;
    Ifdb = Ifbh + Ifbv + Ifbl;
    Ifdc = Ifch + Ifcv + Ifal;

    double Ifahm, Ifbhm, Ifchm;
    double Ifavm, Ifbvm, Ifcvm;
    double Ifalm, Ifblm, Ifclm;
    double Ifdam, Ifdbm, Ifdcm;

    Ifahm = trandevice->halfIa1m[0] - trandevice->halfIc1m[0];
    Ifbhm = trandevice->halfIb1m[0] - trandevice->halfIa1m[0];
    Ifchm = trandevice->halfIc1m[0] - trandevice->halfIb1m[0];

    Ifavm = trandevice->halfIa2m[0] - trandevice->halfIc2m[0];
    Ifbvm = trandevice->halfIb2m[0] - trandevice->halfIa2m[0];
    Ifcvm = trandevice->halfIc2m[0] - trandevice->halfIb2m[0];

    Ifavm = (double)(voltageV / voltageH)* Ifavm;
    Ifbvm = (double)(voltageV / voltageH)* Ifbvm;
    Ifcvm = (double)(voltageV / voltageH)* Ifcvm;

    Ifalm = (double)(voltageL / voltageH)* trandevice->halfIa3m[0];
    Ifblm = (double)(voltageL / voltageH)* trandevice->halfIb3m[0];
    Ifclm = (double)(voltageL / voltageH)* trandevice->halfIc3m[0];
    Ifahm = fabs(Ifahm);
    Ifbhm = fabs(Ifbhm);
    Ifchm = fabs(Ifchm);
    Ifavm = fabs(Ifavm);
    Ifbvm = fabs(Ifbvm);
    Ifcvm = fabs(Ifcvm);
    Ifalm = fabs(Ifalm);
    Ifblm = fabs(Ifblm);
    Ifclm = fabs(Ifclm);

    Ifdam = Ifahm + Ifavm + Ifalm;
    Ifdbm = Ifbhm + Ifbvm + Ifblm;
    Ifdcm = Ifchm + Ifcvm + Ifalm;

    Ida_base = Ifdam - Ifda;
    Idb_base = Ifdbm - Ifdb;
    Idc_base = Ifdcm - Ifdc;

    Ira_base = fabs(Ifah - Ifahm) + fabs(Ifav - Ifavm) + fabs(Ifal - Ifalm);
    Irb_base = fabs(Ifbh - Ifbhm) + fabs(Ifbv - Ifbvm) + fabs(Ifbl - Ifblm);
    Irc_base = fabs(Ifch - Ifchm) + fabs(Ifcv - Ifavm) + fabs(Ifcl - Ifclm);
//
////    trandevice->test[0] = Ida_base;
////    trandevice->test[1] = phasorAbs(Ifda);
//
//    trandevice->Ida_base = Ida_base;
//    trandevice->Idb_base = Idb_base;
//    trandevice->Idc_base = Idc_base;
//    trandevice->Ira_base = Ira_base;
//    trandevice->Irb_base = Irb_base;
//    trandevice->Irc_base = Irc_base;
//
    if (Ida_base > Idtset || Idb_base > Idtset || Idc_base > Idtset && time > 0.02) {
        trandevice->deltaDiffStartFlag = 1;
        trandevice->startTime[1] = trandevice->time;

        tranWriteLog(trandevice, "工频变化量差流起动");
    }

}

void tranDeltaCal(tranDevice* trandevice){

    double voltageH = trandevice->ratedV1;
    double voltageV = trandevice->ratedV2;
    double voltageL = trandevice->ratedV3;

    double Ida_base, Idb_base, Idc_base;
    double Ira_base, Irb_base, Irc_base;

    Phasor Ifah, Ifbh, Ifch;
    Phasor Ifav, Ifbv, Ifcv;
    Phasor Ifal, Ifbl, Ifcl;
    Phasor Ifda, Ifdb, Ifdc;

    Ifah = phasorSub(trandevice->phasor[3],trandevice->phasor[5]);
    Ifbh = phasorSub(trandevice->phasor[4],trandevice->phasor[3]);
    Ifch = phasorSub(trandevice->phasor[5],trandevice->phasor[4]);

    Ifav = phasorSub(trandevice->phasor[9],trandevice->phasor[11]);
    Ifbv = phasorSub(trandevice->phasor[10],trandevice->phasor[9]);
    Ifcv = phasorSub(trandevice->phasor[11],trandevice->phasor[10]);

    Ifav = phasorNumMulti((double)(voltageV / voltageH), Ifav);
    Ifbv = phasorNumMulti((double)(voltageV / voltageH), Ifbv);
    Ifcv = phasorNumMulti((double)(voltageV / voltageH), Ifcv);

    Ifal = phasorNumMulti((double)(voltageL / voltageH), trandevice->phasor[15]);
    Ifbl = phasorNumMulti((double)(voltageL / voltageH), trandevice->phasor[16]);
    Ifcl = phasorNumMulti((double)(voltageL / voltageH), trandevice->phasor[17]);

    Ifda = phasorAdd(Ifah, Ifav);
    Ifda = phasorAdd(Ifda, Ifal);
    Ifdb = phasorAdd(Ifbh, Ifbv);
    Ifdb = phasorAdd(Ifdb, Ifbl);
    Ifdc = phasorAdd(Ifch, Ifcv);
    Ifdc = phasorAdd(Ifdc, Ifcl);

    Phasor Ifahm, Ifbhm, Ifchm;
    Phasor Ifavm, Ifbvm, Ifcvm;
    Phasor Ifalm, Ifblm, Ifclm;
    Phasor Ifdam, Ifdbm, Ifdcm;

    Ifahm = phasorSub(trandevice->memoryIa1[0],trandevice->memoryIc1[0]);
    Ifbhm = phasorSub(trandevice->memoryIb1[0],trandevice->memoryIa1[0]);
    Ifchm = phasorSub(trandevice->memoryIc1[0],trandevice->memoryIb1[0]);

    Ifavm = phasorSub(trandevice->memoryIa2[0],trandevice->memoryIc2[0]);
    Ifbvm = phasorSub(trandevice->memoryIb2[0],trandevice->memoryIa2[0]);
    Ifcvm = phasorSub(trandevice->memoryIc2[0],trandevice->memoryIb2[0]);

    Ifavm = phasorNumMulti((double)(voltageV / voltageH), Ifavm);
    Ifbvm = phasorNumMulti((double)(voltageV / voltageH), Ifbvm);
    Ifcvm = phasorNumMulti((double)(voltageV / voltageH), Ifcvm);

    Ifalm = phasorNumMulti((double)(voltageL / voltageH), trandevice->memoryIa3[0]);
    Ifblm = phasorNumMulti((double)(voltageL / voltageH), trandevice->memoryIb3[0]);
    Ifclm = phasorNumMulti((double)(voltageL / voltageH), trandevice->memoryIc3[0]);

    Ifdam = phasorAdd(Ifahm, Ifavm);
    Ifdam = phasorAdd(Ifdam, Ifalm);
    Ifdbm = phasorAdd(Ifbhm, Ifbvm);
    Ifdbm = phasorAdd(Ifdbm, Ifblm);
    Ifdcm = phasorAdd(Ifchm, Ifcvm);
    Ifdcm = phasorAdd(Ifdcm, Ifclm);

    Ida_base = phasorAbs(phasorSub(Ifdam, Ifda));
    Idb_base = phasorAbs(phasorSub(Ifdbm, Ifdb));
    Idc_base = phasorAbs(phasorSub(Ifdcm, Ifdc));

    Ira_base = phasorAbs(phasorSub(Ifah, Ifahm))+phasorAbs(phasorSub(Ifav, Ifavm))+phasorAbs(phasorSub(Ifal, Ifalm));
    Irb_base = phasorAbs(phasorSub(Ifbh, Ifbhm))+phasorAbs(phasorSub(Ifbv, Ifbvm))+phasorAbs(phasorSub(Ifbl, Ifblm));
    Irc_base = phasorAbs(phasorSub(Ifch, Ifchm))+phasorAbs(phasorSub(Ifcv, Ifcvm))+phasorAbs(phasorSub(Ifcl, Ifclm));

    trandevice->Ida_base = Ida_base;
    trandevice->Idb_base = Idb_base;
    trandevice->Idc_base = Idc_base;
    trandevice->Ira_base = Ira_base;
    trandevice->Irb_base = Irb_base;
    trandevice->Irc_base = Irc_base;

}


void tranFre(tranDevice* trandevice) {
    // 记录当前时刻
    inst2phasor(trandevice->filterVa1, 1, &trandevice->freVa1[0]);
    inst2phasor(trandevice->filterVa2, 1, &trandevice->freVa2[0]);
    inst2phasor(trandevice->filterVa3, 1, &trandevice->freVa3[0]);

}

void tranCalFre(tranDevice* trandevice){
    double Vfa1, Vfa2, Vfa3;

    Vfa1 = phasorAngle(trandevice->phasor[0]) - phasorAngle(trandevice->freVa1[0]);
    Vfa2 = phasorAngle(trandevice->phasor[6]) - phasorAngle(trandevice->freVa2[0]);
    Vfa3 = phasorAngle(trandevice->phasor[12]) - phasorAngle(trandevice->freVa3[0]);

    if(Vfa1 > 180){
        Vfa1 = -Vfa1 + 360.0;
    }
    else if(Vfa1 < -180){
        Vfa1 = Vfa1 + 360;
    }
    if(Vfa2 > 180){
        Vfa2 = -Vfa2 + 360.0;
    }
    else if(Vfa2 < -180){
        Vfa2 = Vfa2 + 360;
    }
    if(Vfa3 > 180){
        Vfa3 = -Vfa3 + 360.0;
    }
    else if(Vfa3 < -180){
        Vfa3 = Vfa3 + 360;
    }

    trandevice->fre1 = Vfa1 *50.0*48.0/360;
    trandevice->fre2 = Vfa2 *50.0*48.0/360;
    trandevice->fre3 = Vfa3 *50.0*48.0/360;

}

void overExc(tranDevice* trandevice){
    double Va1, Va2, Va3;
    Va1 = phasorAbs(trandevice->phasor[0]);
    trandevice->n1 = Va1 / trandevice->fre1;

    Va2 = phasorAbs(trandevice->phasor[6]);
    trandevice->n2 = Va2 / trandevice->fre2;

    Va3 = phasorAbs(trandevice->phasor[12]);
    trandevice->n3 = Va3 / trandevice->fre3;

}

void zeroStart(tranDevice* trandevice){
    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Iaw, Ibw, Icw;
    Phasor Ih0, Iv0, Iw0;
    Phasor I0all;
    double time;
    time = trandevice->time;

    Iah = trandevice->phasor[3];
    Ibh = trandevice->phasor[4];
    Ich = trandevice->phasor[5];
    Ih0 = phasorAdd(Iah, Ibh);
    Ih0 = phasorAdd(Ih0, Ich);

    Iav = trandevice->phasor[9];
    Ibv = trandevice->phasor[10];
    Icv = trandevice->phasor[11];
    Iv0 = phasorAdd(Iav, Ibv);
    Iv0 = phasorAdd(Iv0, Icv);


    Iaw = trandevice->phasor[21];
    Ibw = trandevice->phasor[22];
    Icw = trandevice->phasor[23];
    Iw0 = phasorAdd(Iaw, Ibw);
    Iw0 = phasorAdd(Iw0, Icw);

    I0all = phasorAdd(Ih0, Iv0);
    I0all = phasorAdd(I0all, Iw0);


//    trandevice->test[0] = phasorAbs(I0all);

    double I0qd = 0.5;
    if((phasorAbs(I0all) > 0.95*I0qd) && (time > 0.02)){
        trandevice->zeroDiffStartFlag = 1;
        trandevice->startTime[2] = trandevice->time;

        tranWriteLog(trandevice, "零序比例差动起动");
    }

}

void lowDiffStart(tranDevice* trandevice){
    double time;
    double Ixcd = 0.5;

    time = trandevice->time;

    Phasor Iaw, Ibw, Icw;
    Phasor Ial, Ibl, Icl;
    Phasor Iad, Ibd, Icd;
    double Idmax;

    Ial = trandevice->phasor[15];
    Ibl = trandevice->phasor[16];
    Icl = trandevice->phasor[17];

    Iaw = trandevice->phasor[18];
    Ibw = trandevice->phasor[19];
    Icw = trandevice->phasor[20];

    Iad = phasorAdd(Ial, Iaw);
    Iad = phasorSub(Iad, Icw);
    Ibd = phasorAdd(Ibl, Ibw);
    Ibd = phasorSub(Ibd, Iaw);
    Icd = phasorAdd(Icl, Icw);
    Icd = phasorSub(Icd, Ibw);

    Idmax = phasorAbs(Iad);
    if(Idmax<phasorAbs(Ibd)){
        Idmax = phasorAbs(Ibd);
    }
    if(Idmax<phasorAbs(Icd)){
        Idmax = phasorAbs(Icd);
    }

    if(Idmax > Ixcd && time > 0.02){
        trandevice->lowDiffStartFlag = 1;
        trandevice->startTime[3] = trandevice->time;
        tranWriteLog(trandevice, "低压侧小区差动起动");
    }
}

void impStarth(tranDevice* trandevice){

    double time;
    double Ihset;
    double Ihe;
    Phasor Ifah, Ifbh, Ifch;
    time = trandevice->time;
    Ihe = 1.732 * 1.414 * trandevice->ratedI1;
    Ihset= trandevice->tranStartSetValue[6];

    Ifah = phasorSub(trandevice->phasor[3],trandevice->phasor[5]);
    Ifbh = phasorSub(trandevice->phasor[4],trandevice->phasor[3]);
    Ifch = phasorSub(trandevice->phasor[5],trandevice->phasor[4]);

    Phasor Ifahm, Ifbhm, Ifchm;

    Ifahm = phasorSub(trandevice->memoryIa1[0],trandevice->memoryIc1[0]);
    Ifbhm = phasorSub(trandevice->memoryIb1[0],trandevice->memoryIa1[0]);
    Ifchm = phasorSub(trandevice->memoryIc1[0],trandevice->memoryIb1[0]);

    double Ihda, Ihdb, Ihdc;

    Ihda = phasorAbs(phasorSub(Ifah, Ifahm));
    Ihdb = phasorAbs(phasorSub(Ifbh, Ifbhm));
    Ihdc = phasorAbs(phasorSub(Ifch, Ifchm));

    double Ihmax;
    Ihmax = Ihda;
    if(Ihmax < Ihdb){
        Ihmax = Ihdb;
    }
    if(Ihmax < Ihdc){
        Ihmax = Ihdc;
    }

    Phasor Ih2;
    Phasor Ih2a, Ih2b, Ih2c;


    Ih2a = Ifah;
    Ih2b = phasorContrarotate(Ifbh, 240.0);
    Ih2c = phasorContrarotate(Ifch, 120.0);
    Ih2 = phasorAdd(Ih2a, phasorAdd(Ih2b ,Ih2c));


    if(Ihmax > 1.25*Ihset && phasorAbs(Ih2) > 0.2*Ihe && time > 0.02){
        trandevice->impStartFlag_h = 1;
        trandevice->startTime[4] = trandevice->time;
        tranWriteLog(trandevice, "高压侧阻抗保护起动");
    }

}

void impStartv(tranDevice* trandevice){

    double time;
    double Ivset;
    double Ive;
    Phasor Ifav, Ifbv, Ifcv;
    time = trandevice->time;
    Ive = 1.732 * 1.414 * trandevice->ratedI2;
    Ivset= trandevice->tranStartSetValue[7];

    Ifav = phasorSub(trandevice->phasor[9],trandevice->phasor[11]);
    Ifbv = phasorSub(trandevice->phasor[10],trandevice->phasor[9]);
    Ifcv = phasorSub(trandevice->phasor[11],trandevice->phasor[10]);

    Phasor Ifavm, Ifbvm, Ifcvm;

    Ifavm = phasorSub(trandevice->memoryIa2[0],trandevice->memoryIc2[0]);
    Ifbvm = phasorSub(trandevice->memoryIb2[0],trandevice->memoryIa2[0]);
    Ifcvm = phasorSub(trandevice->memoryIc2[0],trandevice->memoryIb2[0]);

    double Ivda, Ivdb, Ivdc;

    Ivda = phasorAbs(phasorSub(Ifav, Ifavm));
    Ivdb = phasorAbs(phasorSub(Ifbv, Ifbvm));
    Ivdc = phasorAbs(phasorSub(Ifcv, Ifcvm));

    double  Ivmax;

    Ivmax = Ivda;
    if(Ivmax < Ivdb){
        Ivmax = Ivdb;
    }
    if(Ivmax < Ivdc){
        Ivmax = Ivdc;
    }

    Phasor Iv2;
    Phasor Iv2a, Iv2b, Iv2c;

    Iv2a = Ifav;
    Iv2b = phasorContrarotate(Ifbv, 240.0);
    Iv2c = phasorContrarotate(Ifcv, 120.0);
    Iv2 = phasorAdd(Iv2a, phasorAdd(Iv2b ,Iv2c));

//    trandevice->test[0] = Ivmax;
//    trandevice->test[1] = phasorAbs(Iv2);
//    trandevice->test[2] = 1.25*Ivset;
//    trandevice->test[3] = 0.6*Ive;

    if(Ivmax > 1.25*Ivset && phasorAbs(Iv2) > 0.2*Ive && time > 0.02){
        trandevice->impStartFlag_v = 1;
        trandevice->startTime[5] = trandevice->time;
        tranWriteLog(trandevice, "中压侧阻抗保护起动");
    }



}

void splitStart(tranDevice* trandevice){
    double Ifcdqd = 0.5;
    double Ie = trandevice->ratedI2 * 1.414;

    Phasor Iah, Ibh, Ich;
    Phasor Iav, Ibv, Icv;
    Phasor Iaw, Ibw, Icw;
    Phasor Ida, Idb, Idc;

    double Ifda, Ifdb, Ifdc;
    double Idmax;
    double time;

    time = trandevice->time;

    Iah = trandevice->phasor[3];
    Ibh = trandevice->phasor[4];
    Ich = trandevice->phasor[5];

    Iav = trandevice->phasor[9];
    Ibv = trandevice->phasor[10];
    Icv = trandevice->phasor[11];

    Iaw = trandevice->phasor[21];
    Ibw = trandevice->phasor[22];
    Icw = trandevice->phasor[23];

    Ida = phasorAdd(Iah, Iav);
    Ida = phasorAdd(Ida, Iaw);
    Idb = phasorAdd(Ibh, Ibv);
    Idb = phasorAdd(Idb, Ibw);
    Idc = phasorAdd(Ich, Icv);
    Idc = phasorAdd(Idc, Icw);

    Ifda = phasorAbs(Ida);
    Ifdb = phasorAbs(Idb);
    Ifdc = phasorAbs(Idc);

    Idmax = Ifda;
    if(Idmax < Ifdb){
        Idmax = Ifdb;
    }
    if(Idmax < Ifdc){
        Idmax = Ifdc;
    }

    if(Idmax > Ifcdqd && time > 0.02){
        trandevice->splitStartFlag = 1;
        trandevice->startTime[3] = trandevice->time;
        tranWriteLog(trandevice, "分侧差动保护起动");

    }

}

void zeroCurStarth(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    Phasor I0;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->zeroCur1[0];

    Ia = trandevice->phasor[3];
    Ib = trandevice->phasor[4];
    Ic = trandevice->phasor[5];
    I0 = phasorAdd(Ia, Ib);
    I0 = phasorAdd(I0, Ic);

    if(phasorAbs(I0) > Iset && time > 0.02){
        trandevice->zeroCurStartFlagh = 1;
        trandevice->backTime[0] = trandevice->time;
        tranWriteLog(trandevice, "高压侧零序保护起动");

    }


}

void zeroCurStartv(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    Phasor I0;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->zeroCur2[0];

    Ia = trandevice->phasor[9];
    Ib = trandevice->phasor[10];
    Ic = trandevice->phasor[11];
    I0 = phasorAdd(Ia, Ib);
    I0 = phasorAdd(I0, Ic);

    if(phasorAbs(I0) > Iset && time > 0.02){
        trandevice->zeroCurStartFlagv = 1;
        trandevice->backTime[1] = trandevice->time;
        tranWriteLog(trandevice, "中压侧零序保护起动");

    }


}

void zeroCurStartt(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    Phasor I0;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->zeroCur2[3];

    Ia = trandevice->phasor[9];
    Ib = trandevice->phasor[10];
    Ic = trandevice->phasor[11];
    I0 = phasorAdd(Ia, Ib);
    I0 = phasorAdd(I0, Ic);

    if(phasorAbs(I0) > Iset && time > 0.02){
        trandevice->zeroCurStartFlagv = 1;
        trandevice->backTime[2] = trandevice->time;
        tranWriteLog(trandevice, "公共绕组零序保护起动");

    }

}

void phaseCurStarth(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    double Imax;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->compSet1[2];

    Ia = trandevice->phasor[3];
    Ib = trandevice->phasor[4];
    Ic = trandevice->phasor[5];
    Imax = phasorAbs(Ia);
    if(Imax < phasorAbs(Ib)){
        Imax = phasorAbs(Ib);
    }
    if(Imax < phasorAbs(Ic)){
        Imax = phasorAbs(Ic);
    }

    if(Imax > Iset && time > 0.02){
        trandevice->phaCurStartFlagh = 1;
        trandevice->backTime[3] = trandevice->time;
        tranWriteLog(trandevice, "高压侧过流保护起动");

    }


}

void phaseCurStartv(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    double Imax;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->compSet2[2];

    Ia = trandevice->phasor[9];
    Ib = trandevice->phasor[10];
    Ic = trandevice->phasor[11];
    Imax = phasorAbs(Ia);
    if(Imax < phasorAbs(Ib)){
        Imax = phasorAbs(Ib);
    }
    if(Imax < phasorAbs(Ic)){
        Imax = phasorAbs(Ic);
    }

    if(Imax > Iset && time > 0.02){
        trandevice->phaCurStartFlagv = 1;
        trandevice->backTime[4] = trandevice->time;
        tranWriteLog(trandevice, "中压侧过流保护起动");

    }


}


void phaseCurStartl1(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    double Imax;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->overCur_set4;

    Ia = trandevice->phasor[15];
    Ib = trandevice->phasor[16];
    Ic = trandevice->phasor[17];
    Imax = phasorAbs(Ia);
    if(Imax < phasorAbs(Ib)){
        Imax = phasorAbs(Ib);
    }
    if(Imax < phasorAbs(Ic)){
        Imax = phasorAbs(Ic);
    }

    if(Imax > Iset && time > 0.02){
        trandevice->phaCurStartFlagl1 = 1;
        trandevice->backTime[5] = trandevice->time;
        tranWriteLog(trandevice, "低压侧过流保护起动");

    }


}

void phaseCurStartl2(tranDevice* trandevice){
    Phasor Ia, Ib, Ic;
    double Imax;
    double time;
    time = trandevice->time;
    double Iset;

    Iset = 0.95*trandevice->overCur_set3;

    Ia = trandevice->phasor[18];
    Ib = trandevice->phasor[19];
    Ic = trandevice->phasor[20];
    Imax = phasorAbs(Ia);
    if(Imax < phasorAbs(Ib)){
        Imax = phasorAbs(Ib);
    }
    if(Imax < phasorAbs(Ic)){
        Imax = phasorAbs(Ic);
    }

    if(Imax > Iset && time > 0.02){
        trandevice->phaCurStartFlagl2 = 1;
        trandevice->backTime[6] = trandevice->time;
        tranWriteLog(trandevice, "低绕组过流保护起动");

    }


}