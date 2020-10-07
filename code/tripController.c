#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"


void tripController(Device *device) {
    // 仿真刚开始时不处理
    if (device->time < 0.02)
        return;

    // 手合
    device->outRes[0] = risingEdgeSet(&device->manCloseFlag[0], &device->preManCloseFlag[0], device->outRes[0], 1);
    device->outRes[1] = risingEdgeSet(&device->manCloseFlag[1], &device->preManCloseFlag[1], device->outRes[1], 1);
    device->outRes[2] = risingEdgeSet(&device->manCloseFlag[2], &device->preManCloseFlag[2], device->outRes[2], 1);

//    if (device->time > 1.0) {
//        debugWithDouble(device, "跳闸信号%f", device->tripFlag[0]);
//    }
    // 线路保护
    device->outRes[0] = risingEdgeSetDouble(&device->tripFlag[0], &device->preTripFlag[0], device->outRes[0], 0);
    device->outRes[1] = risingEdgeSetDouble(&device->tripFlag[1], &device->preTripFlag[1], device->outRes[1], 0);
    device->outRes[2] = risingEdgeSetDouble(&device->tripFlag[2], &device->preTripFlag[2], device->outRes[2], 0);


    // 重合闸
    device->outRes[0] = risingEdgeSet(&device->recloseFlag[0], &device->preRecloseFlag[0], device->outRes[0], 1);
    device->outRes[1] = risingEdgeSet(&device->recloseFlag[1], &device->preRecloseFlag[1], device->outRes[1], 1);
    device->outRes[2] = risingEdgeSet(&device->recloseFlag[2], &device->preRecloseFlag[2], device->outRes[2], 1);

//    // 断路器失灵保护
//    device->outRes[0] = risingEdgeSet(&device->failedProtectionFlag[0], &device->preFailedProtectionFlag[0], device->outRes[0], 0);
//    device->outRes[1] = risingEdgeSet(&device->failedProtectionFlag[1], &device->preFailedProtectionFlag[1], device->outRes[1], 0);
//    device->outRes[2] = risingEdgeSet(&device->failedProtectionFlag[2], &device->preFailedProtectionFlag[2], device->outRes[2], 0);
//
//    // 三相不一致保护
//    device->outRes[0] = risingEdgeSet(&device->threePhaseNotFitFlag[0], &device->preThreePhaseNotFitFlag[0], device->outRes[0], 0);
//    device->outRes[1] = risingEdgeSet(&device->threePhaseNotFitFlag[1], &device->preThreePhaseNotFitFlag[1], device->outRes[1], 0);
//    device->outRes[2] = risingEdgeSet(&device->threePhaseNotFitFlag[2], &device->preThreePhaseNotFitFlag[2], device->outRes[2], 0);


}