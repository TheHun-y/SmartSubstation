#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <math.h>
#include <stdio.h>

extern void lonDiffRelay(tranDevice* trandevice);
extern void deltaDiffRelay(tranDevice* trandevice);
extern void zeroDiffRelay(tranDevice* trandevice);
extern void splitDiffRealy(tranDevice* trandevice);
extern void diffTrip(tranDevice* trandevice);
extern void lowDiffRelay(tranDevice* trandevice);

extern void impActh(tranDevice* trandevice);
extern void impActv(tranDevice* trandevice);
extern void impRelayh(tranDevice* trandevice);
extern void impRelayv(tranDevice* trandevice);
extern void zeroCurrh(tranDevice* trandevice);
extern void zeroCurrv(tranDevice* trandevice);
extern void zeroCurrt(tranDevice* trandevice);



void tran(tranDevice *trandevice){

    trandevice->tranProRes[11] = trandevice->filterVa1[0];

    // 将采样值存入瞬时值数组
    tranSample2inst(trandevice);

    // 瞬时值滤波后存入并更新滤波后数组
    tranDataFilter(trandevice);

    // 利用滤波后数据计算12通道相量,存入phasor数组
    tranToPhasor(trandevice);

    tranHalfPhasor(trandevice);

    calIdmax(trandevice);

    tranRecordMemory(trandevice);

    recordMemoryhalf(trandevice);

    tranFre(trandevice);

    tranCalFre(trandevice);

    overExc(trandevice);

    tranHarmPhasor(trandevice);

//    zeroCurrh(trandevice);
//    zeroCurrv(trandevice);

    tranHalfInte(trandevice);

    if(trandevice->startEnable[5] == 1 ){
        trandevice->mainFlag == 1;
    }

    if ((trandevice->lonDiffStartFlag == 0) && (trandevice->startEnable[0] > 0)){
        tranLonDiff(trandevice);
    }
    if((trandevice->lonDiffStartFlag == 1) && (trandevice->mainFlag == 0) && (trandevice->startEnable[0] == 1)){
        diffTrip(trandevice);
    }
    if((trandevice->lonDiffStartFlag == 1) && (trandevice->mainFlag == 0)){
        lonDiffRelay(trandevice);
    }

    if ((trandevice->deltaDiffStartFlag == 0) && (trandevice->startEnable[1] > 0)) {
        tranDeltaDiffStart(trandevice);
    }
    if((trandevice->deltaDiffStartFlag == 1) && (trandevice->mainFlag == 0) && (trandevice->startEnable[2] == 1)){
        deltaDiffRelay(trandevice);
    }

    if((trandevice->zeroDiffStartFlag == 0)  && (trandevice->startEnable[1] > 0)){
        zeroStart(trandevice);
    }
    if((trandevice->zeroDiffStartFlag == 1) && (trandevice->mainFlag == 0) && (trandevice->startEnable[1] == 1)){
        zeroDiffRelay(trandevice);
    }


    if((trandevice->splitDiffStartFlag == 0)  && (trandevice->startEnable[4] > 0)){
        splitStart(trandevice);
    }
    if((trandevice->splitStartFlag == 1) && (trandevice->mainFlag == 0)  && (trandevice->startEnable[4] == 1)){
        splitDiffRelay(trandevice);
    }


    if((trandevice->lowDiffStartFlag == 0) && (trandevice->startEnable[3] > 0)){
        lowDiffStart(trandevice);
    }
    if((trandevice->lowDiffStartFlag == 1) && (trandevice->mainFlag == 0)  && (trandevice->startEnable[3] == 1)){
        lowDiffRelay(trandevice);
    }

    if ((trandevice->impStartFlag_h == 0) && (trandevice->backEnable1[0] > 0) && (trandevice->backEnable1[1] > 0)){
        impStarth(trandevice);
    }
    if(trandevice->impStartFlag_h == 1){
        impRelayh(trandevice);
    }

    if ((trandevice->impStartFlag_v == 0) && (trandevice->backEnable2[0] > 0) && (trandevice->backEnable2[1] > 0)){
        impStartv(trandevice);
    }
    if(trandevice->impStartFlag_v == 1){
        impRelayv(trandevice);
    }

    if(trandevice->zeroCurStartFlagh == 0) {
        zeroCurStarth(trandevice);
    }
    if(trandevice->zeroCurStartFlagh == 1){
        zeroCurrh(trandevice);
    }

    if(trandevice->zeroCurStartFlagv == 0) {
        zeroCurStartv(trandevice);
    }
    if(trandevice->zeroCurStartFlagv == 1){
        zeroCurrv(trandevice);
    }

    if(trandevice->zeroCurStartFlagt == 0) {
        zeroCurStartt(trandevice);
    }
    if(trandevice->zeroCurStartFlagt == 1){
        zeroCurrt(trandevice);
    }


    if (trandevice->time < 0.2) {
        // 等待仿真进入稳定状态
        return;
    }

}
