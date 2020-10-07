#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>

void impRelayh(tranDevice* trandevice){

    double time;
    double startTime_h;

    Phasor Vah, Vbh, Vch;
    Phasor Iah, Ibh ,Ich;

    Phasor Vhab, Vhbc, Vhca;
    Phasor Ihab, Ihbc, Ihca;


    int flag1;
    int flag2;
    int flagh;
    flag1 = 0;
    flagh = 0;


    double zph, znh, zpv, znv;
    zph = trandevice->zph;
    znh = trandevice->znh;
    zpv = trandevice->zpv;
    znv = trandevice->znv;

    Phasor Uhap1 ,Uhap2, Uhbp1, Uhbp2, Uhcp1, Uhcp2;
    Phasor Uhab1 ,Uhab2, Uhbc1, Uhbc2, Uhca1, Uhca2;

    Phasor Iaph, Ibph, Icph;
    Phasor Ih0;
    Phasor zph_p, znh_p, zpv_p, znv_p;
    Phasor temp_hap, temp_hbp, temp_hcp;
    Phasor temp_hab, temp_hbc, temp_hca;
    double angle_hap, angle_hbp, angle_hcp;
    double angle_hab, angle_hbc, angle_hca;


    Vah = trandevice->phasor[0];
    Vbh = trandevice->phasor[1];
    Vch = trandevice->phasor[2];
    Iah = trandevice->phasor[3];
    Ibh = trandevice->phasor[4];
    Ich = trandevice->phasor[5];


    Ih0 = phasorAdd(Iah, Ibh);
    Ih0 = phasorAdd(Ih0, Ich);
    Ih0 = phasorNumMulti(0.1, Ih0);
    Iaph = phasorAdd(Iah,Ih0);
    Ibph = phasorAdd(Ibh,Ih0);
    Icph = phasorAdd(Ich,Ih0);

    zph_p.real = zph*0.1736;
    zph_p.img = zph*0.9848;
    znh_p.real = znh*0.1736;
    znh_p.img = znh*0.9848;

    Uhap1 = phasorSub(Vah, phasorMulti(zph_p, Iaph));
    Uhap2 = phasorAdd(Vah, phasorMulti(znh_p, Iaph));
    temp_hap = phasorDiv(Uhap1, Uhap2);
    angle_hap = phasorAngle(temp_hap);

    Uhbp1 = phasorSub(Vbh, phasorMulti(zph_p, Ibph));
    Uhbp2 = phasorAdd(Vbh, phasorMulti(znh_p, Ibph));
    temp_hbp = phasorDiv(Uhbp1, Uhbp2);
    angle_hbp = phasorAngle(temp_hbp);

    Uhcp1 = phasorSub(Vch, phasorMulti(zph_p, Icph));
    Uhcp2 = phasorAdd(Vch, phasorMulti(znh_p, Icph));
    temp_hcp = phasorDiv(Uhcp1, Uhcp2);
    angle_hcp = phasorAngle(temp_hcp);

    if(angle_hap>90 && angle_hap<270){
        flag1 = 1;
    }
    if(angle_hbp>90 && angle_hbp<270){
        flag1 = 1;
    }
    if(angle_hcp>90 && angle_hcp<270){
        flag1 = 1;
    }


    Vhab = phasorSub(Vah, Vbh);
    Vhbc = phasorSub(Vbh, Vch);
    Vhca = phasorSub(Vch, Vah);
    Ihab = phasorSub(Iah, Ibh);
    Ihbc = phasorSub(Ibh, Ich);
    Ihca = phasorSub(Ich, Iah);

    Uhab1 = phasorSub(Vhab, phasorMulti(zph_p, Ihab));
    Uhab2 = phasorAdd(Vhab, phasorMulti(znh_p, Ihab));
    temp_hab = phasorDiv(Uhab1, Uhab2);
    angle_hab = phasorAngle(temp_hab);

    Uhbc1 = phasorSub(Vhbc, phasorMulti(zph_p, Ihbc));
    Uhbc2 = phasorAdd(Vhbc, phasorMulti(znh_p, Ihbc));
    temp_hbc = phasorDiv(Uhbc1, Uhbc2);
    angle_hbc = phasorAngle(temp_hbc);

    Uhca1 = phasorSub(Vhca, phasorMulti(zph_p, Ihca));
    Uhca2 = phasorAdd(Vhca, phasorMulti(znh_p, Ihca));
    temp_hca = phasorDiv(Uhca1, Uhca2);
    angle_hca = phasorAngle(temp_hca);

    if(angle_hab>90 && angle_hab<270){
        flag1 = 1;
    }
    if(angle_hbc>90 && angle_hbc<270){
        flag1 = 1;
    }
    if(angle_hca>90 && angle_hca<270){
        flag1 = 1;
    }


    time = trandevice->time;
    startTime_h = trandevice->startTime[4];

    Phasor Izdh1, Izdh2, Izdh0;


    Izdh1 = phasorSeq(Iah, Ibh, Ich, 1);
    Izdh2 = phasorSeq(Iah, Ibh, Ich, 2);
    Izdh0 = phasorSeq(Iah, Ibh, Ich, 0);


    double time_seth;
    time_seth = trandevice->time_seth;

    int flagzdh = 0;
    if(((phasorAbs(Izdh1) > 1.6*trandevice->ratedI1) || ((phasorAbs(Izdh2)+phasorAbs(Izdh0))>(3*phasorAbs(Izdh1))))&& time-startTime_h>0.03 && time - startTime_h <0.19 && flagzdh == 0){
        flagzdh = 1;
    }


    if(trandevice->impStartFlag_h == 1 && flag1 == 1 && time - startTime_h > 0.02 + time_seth && flagzdh == 1){
        trandevice->tranProRes[9] = 1;
//        int i = 0;
//        for(i=0; i<3 ; i++){
//            trandevice->tranProRes[i] = 1;
//        }
        tranWriteLog(trandevice, "高压侧阻抗保护动作");

    }



}


void impRelayv(tranDevice* trandevice) {

    double time;
    double startTime_v;

    Phasor Vav, Vbv, Vcv;
    Phasor Iav, Ibv, Icv;

    Phasor Vvab, Vvbc, Vvca;
    Phasor Ivab, Ivbc, Ivca;


    int flag2;
    int flagv;

    flag2 = 0;
    flagv = 0;

    double zpv, znv;
    zpv = trandevice->zpv;
    znv = trandevice->znv;


    Phasor Uvap1, Uvap2, Uvbp1, Uvbp2, Uvcp1, Uvcp2;
    Phasor Uvab1, Uvab2, Uvbc1, Uvbc2, Uvca1, Uvca2;


    Phasor Iapv, Ibpv, Icpv;
    Phasor Iv0;
    Phasor zpv_p, znv_p;

    Phasor temp_vap, temp_vbp, temp_vcp;
    Phasor temp_vab, temp_vbc, temp_vca;
    double angle_vap, angle_vbp, angle_vcp;
    double angle_vab, angle_vbc, angle_vca;

    Vav = trandevice->phasor[6];
    Vbv = trandevice->phasor[7];
    Vcv = trandevice->phasor[8];
    Iav = trandevice->phasor[9];
    Ibv = trandevice->phasor[10];
    Icv = trandevice->phasor[11];

    Vvab = phasorSub(Vav, Vbv);
    Vvbc = phasorSub(Vbv, Vcv);
    Vvca = phasorSub(Vcv, Vav);
    Ivab = phasorSub(Iav, Ibv);
    Ivbc = phasorSub(Ibv, Icv);
    Ivca = phasorSub(Icv, Iav);

    Iv0 = phasorAdd(Iav, Ibv);
    Iv0 = phasorAdd(Iv0, Icv);
    Iv0 = phasorNumMulti(0.1, Iv0);
    Iapv = phasorAdd(Iav, Iv0);
    Ibpv = phasorAdd(Ibv, Iv0);
    Icpv = phasorAdd(Icv, Iv0);

    zpv_p.real = zpv * 0.1736;
    zpv_p.img = zpv * 0.9848;
    znv_p.real = znv * 0.1736;
    znv_p.img = znv * 0.9848;


    Uvap1 = phasorSub(Vav, phasorMulti(zpv_p, Iapv));
    Uvap2 = phasorAdd(Vav, phasorMulti(znv_p, Iapv));
    temp_vap = phasorDiv(Uvap1, Uvap2);
    angle_vap = phasorAngle(temp_vap);

    Uvbp1 = phasorSub(Vbv, phasorMulti(zpv_p, Ibpv));
    Uvbp2 = phasorAdd(Vbv, phasorMulti(znv_p, Ibpv));
    temp_vbp = phasorDiv(Uvbp1, Uvbp2);
    angle_vbp = phasorAngle(temp_vbp);

    Uvcp1 = phasorSub(Vcv, phasorMulti(zpv_p, Icpv));
    Uvcp2 = phasorAdd(Vcv, phasorMulti(znv_p, Icpv));
    temp_vcp = phasorDiv(Uvcp1, Uvcp2);
    angle_vcp = phasorAngle(temp_vcp);

    if (angle_vap > 90 && angle_vap < 270) {
        flag2 = 1;
    }
    if (angle_vbp > 90 && angle_vbp < 270) {
        flag2 = 1;
    }
    if (angle_vcp > 90 && angle_vcp < 270) {
        flag2 = 1;
    }

    Uvab1 = phasorSub(Vvab, phasorMulti(zpv_p, Ivab));
    Uvab2 = phasorAdd(Vvab, phasorMulti(znv_p, Ivab));
    temp_vab = phasorDiv(Uvab1, Uvab2);
    angle_vab = phasorAngle(temp_vab);

    Uvbc1 = phasorSub(Vvbc, phasorMulti(zpv_p, Ivbc));
    Uvbc2 = phasorAdd(Vvbc, phasorMulti(znv_p, Ivbc));
    temp_vbc = phasorDiv(Uvbc1, Uvbc2);
    angle_vbc = phasorAngle(temp_vbc);

    Uvca1 = phasorSub(Vvca, phasorMulti(zpv_p, Ivca));
    Uvca2 = phasorAdd(Vvca, phasorMulti(znv_p, Ivca));
    temp_vca = phasorDiv(Uvca1, Uvca2);
    angle_vca = phasorAngle(temp_vca);

    if (angle_vab > 90 && angle_vab < 270) {
        flag2 = 1;
    }
    if (angle_vbc > 90 && angle_vbc < 270) {
        flag2 = 1;
    }
    if (angle_vca > 90 && angle_vca < 270) {
        flag2 = 1;
    }
//    trandevice->tranProRes[10] = angle_vap;

//    trandevice->tranProRes[12] = angle_vca;


    time = trandevice->time;
    startTime_v = trandevice->startTime[5];

    Phasor Izdv1, Izdv2, Izdv0;

    Izdv1 = phasorSeq(Iav, Ibv, Icv, 1);
    Izdv2 = phasorSeq(Iav, Ibv, Icv, 2);
    Izdv0 = phasorSeq(Iav, Ibv, Icv, 0);
    trandevice->tranProRes[1] = phasorAbs(Izdv1);


    double time_setv;
    time_setv = trandevice->time_setv;

    int flagzdv = 0;
    if(((phasorAbs(Izdv1) > (1.414*trandevice->ratedI2)) || ((phasorAbs(Izdv2)+phasorAbs(Izdv0)) > (3*phasorAbs(Izdv1)))) && ((time-startTime_v)>0.03) && ((time - startTime_v) <0.19) && (flagzdv == 0)){
        flagzdv = 1;
    }

    if((trandevice->impStartFlag_v == 1) && (flag2 == 1) && ((time - startTime_v) > (0.02 + time_setv) && (flagzdv == 1))) {
        trandevice->tranProRes[2] = 1;
//        int i = 0;
//        for(i=0; i<3 ; i++){
//            trandevice->tranProRes[i+3] = 1;
//        }
        tranWriteLog(trandevice, "中压侧阻抗保护动作");

    }

}

//
//
//void impActh(tranDevice* trandevice){
//    double time;
//    time = trandevice->time;
//    double time_seth;
//    time_seth = trandevice->time_seth;
//
//    if(trandevice->flagh ==1 && time - trandevice->impTimeh > time_seth){
//        int i = 0;
//        for(i=0; i<3 ; i++){
//            trandevice->tranProRes[i] = 1;
//        }
//        tranWriteLog(trandevice, "高压侧阻抗保护动作");
//    }
//
//}
//
//void impActv(tranDevice* trandevice){
//    double time;
//    time = trandevice->time;
//    double time_setv;
//    time_setv = trandevice->time_setv;
//
//    if(trandevice->flagv ==1 && time - trandevice->impTimev > time_setv){
//        int i = 0;
//        for(i=0; i<3 ; i++){
//            trandevice->tranProRes[i+3] = 1;
//        }
//        tranWriteLog(trandevice, "中压侧阻抗保护动作");
//    }
//
//}
