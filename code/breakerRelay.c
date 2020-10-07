#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

extern int TWJ(Device* device, int phase);

extern void autoReclose(Device* device);
extern void singleReclose(Device* device, int phase);
extern void singleRecloseStarter(Device* device, int phase);
extern void threeReclose(Device* device);
extern void threeRecloseStarter(Device* device);

extern void failedProtection(Device* device);
extern int threeFailed(Device* device);
extern int singleFailed(Device* device, int phase);
extern int faultExist(Device* device);

extern void threePhaseNotFit(Device* device);

extern void reSetAllTripFlag(Device *device, int phase);


// 断路器保护
void breakerRelay(Device* device) {
    int phase = 0;
    // 将来修改为从配置文件中读取
    device->recloseEnable = 1;
    device->singleRecloseEnable = 1;
    device->threeRecloseEnable = 0;
    device->manCloseStableTimeSet = 0.15;
    device->singleRecloseTimeSet = 0.2;
    device->threeRecloseTimeSet = 0.2;

    device->failedProtectionEnable = 1;
    device->faultExistZeroSeqCurrentSet = 2.0;
    device->noCurrentSet = 0.1;
    device->failedProtectionTimeSet = 0.3;

    device->threePhaseNotFitEnable = 1;
    device->threePhaseNotFitTimeSet = 0.5;

    if (device->manCloseTime < 0.01 || device->time < device->manCloseTime + device->manCloseStableTimeSet) {
        return;
    }

    // 自动重合闸
    autoReclose(device);


    // 失灵保护
    // failedProtection(device);


    // 三相不一致保护
    threePhaseNotFit(device);

}

void autoReclose(Device* device) {
    int phase = 0;

    if (device->recloseEnable == 0) {
        return;
    }
    // 自动重合闸部分
    if (device->singleRecloseEnable == 1 && device->threeRecloseEnable == 0) {
        // 单相重合闸
        for (phase = 0; phase < 3; phase++) {
            singleReclose(device, phase);

        }
    } else if (device->singleRecloseEnable == 0 && device->threeRecloseEnable == 1) {
        // 三相重合闸 检测到任意一相位置不对应后, 启动重合闸计时, 计时到达后, 三重同时发脉冲
        threeReclose(device);
    } else {
        // 控制字设置错误, 不启动重合闸
        writeErrorLog(device, "重合闸重合方式整定错误");
    }
}

// 单相重合闸
void singleReclose(Device* device, int phase) {
    if (device->singleRecloseEnable == 0) {
        return;
    }

    // 重合闸闭锁判断
    if (device->recloseShutFlag[phase] == 1) {
        return;
    }

     // 判断重合闸启动, 位置不对应启动
    singleRecloseStarter(device, phase);
    if (device->recloseStart[phase] == 0) {
        return;
    }

    // 重合闸只会动作一次, 使用onlyOnce函数
    if (device->time > device->recloseStartTime[phase] + device->singleRecloseTimeSet) {
        if (notYetWithPhase(device, "%c相重合闸动作, 仅动作一次", phase)) {

            device->recloseFlag[phase] = 1;
            // 记录重合闸动作时间
            device->recloseOpTime[phase] = device->time;

            // 注意: 重合闸动作后将保护所有信号复归
            reSetAllTripFlag(device, phase);

            // 重置跳闸信号闭锁信号
            device->latchTripSignal[phase] = 0;
            writeLogWithPhase(device, "%c相重合闸重合动作", phase);
        }
    }
}



/**
 * 单相重合闸启动
 */
 void singleRecloseStarter(Device* device, int phase) {
     // 合闸后过一段时间进入
    if (device->manCloseTime < 0.01 || device->time < device->manCloseTime + device->manCloseStableTimeSet) {
        return;
    }
     // 已经合闸, 但断路器位置为0
     if (device->manBrkStatus[0][phase] == 1 && device->brkStatus[0][phase] == 0) {
         device->recloseStart[phase] = 1;
         // 记录重合闸启动时间
         if (notYetWithPhase(device, "%c相重合闸启动时间", phase)) {
             device->recloseStartTime[phase] = device->time;
         }
         writeLogWithPhase(device, "%c相自动重合闸启动", phase);
     }
 }


 /**
  * 三相重合闸
  */
 void threeReclose(Device* device) {
     if (device->threeRecloseEnable == 0) {
         return;
     }

     // 三相重合闸闭锁判断
     if (device->recloseShutFlag[0] && device->recloseShutFlag[1] && device->recloseShutFlag[2]) {
         return;
     }

     // 判断重合闸启动, 位置不对应启动
     threeRecloseStarter(device);
     if (device->recloseStart[0] == 0 || device->recloseStart[1] == 0 || device->recloseStart[2] == 0) {
         return;
     }

     // 重合闸只会动作一次, 使用onlyOnce函数
     if (device->time > device->recloseStartTime[0] + device->threeRecloseTimeSet) {
         if (notYet(device, "三相自动重合闸动作, 仅动作一次")) {
             int phase = 0;

             for (; phase < 3; phase++) {
                 device->recloseFlag[phase] = 1;

                 // 注意: 重合闸动作后将保护所有信号复归
                 reSetAllTripFlag(device, phase);

                 // 归零跳闸信号闭锁信号
                 device->latchTripSignal[phase] = 0;
             }
             writeLog(device, "%三相自动重合闸重合动作");
         }
     }




 }

/**
* 三相重合闸启动, 位置不对应启动, 任意一相出现位置不对应就启动三相重合闸
* @param device
* @return
*/
void threeRecloseStarter(Device* device) {
    int phase = 0;
    // 合闸后过一段时间进入
    if (device->manCloseTime < 0.01 || device->time < device->manCloseTime + device->manCloseStableTimeSet) {
        return;
    }

    // 判断三相, 只要有一相不一致就动作
    for (; phase < 3; phase++) {
        if (device->manBrkStatus[0][phase] == 1 && device->brkStatus[0][phase] == 0) {
            // 三相都跳开
            device->tripFlag[0] = 1;
            device->tripFlag[1] = 1;
            device->tripFlag[2] = 1;

            device->recloseStart[0] = 1;
            device->recloseStart[1] = 1;
            device->recloseStart[2] = 1;
            // 记录重合闸启动时间
            if (notYet(device, "记录三相重合闸启动时间")) {
                device->recloseStartTime[0] = device->time;
                device->recloseStartTime[1] = device->time;
                device->recloseStartTime[2] = device->time;
            }
            writeLog(device, "三相自动重合闸启动");
            break;
        }
    }
}


/**
 * 二、断路器失灵保护
 */
 void failedProtection(Device* device) {
     if (device->failedProtectionEnable == 0) {
         return;
     }
     /**
      * 情景一: 有三跳信号, 超过失灵保护动作时间后, 故障仍然存在
      * 三相跳闸信号是脉冲(会被tripController检测到高电平后置为0), 因此到遇到跳闸电平后, 需要自己锁存一份
      * 由于过电流保护没有统一的三跳信号, 使用三个变量分别锁存三相跳闸信号
      */
      int phase = 0;

      /* 警告: 暂时没有确定合适归零 */
      // 锁存跳闸信号
      for (phase = 0; phase < 3; phase++) {
          if (doubleEqual(device->tripFlag[phase], 1.0)) {
              device->latchTripSignal[phase] = 1;
          }
      }

      // 三相失灵保护
      if (threeFailed(device)) {
          for (phase = 0; phase < 3; phase++) {
              device->failedProtectionFlag[phase] = 1;
          }
          writeLog(device, "三跳失灵保护动作");
      }

      // 单相失灵保护
      for (phase = 0; phase < 3; phase++) {
          if (singleFailed(device, phase)) {
              device->failedProtectionFlag[phase] = 1;
              writeLogWithPhase(device, "%c相分相失灵保护动作", phase);
          }
      }

 }

 int threeFailed(Device* device) {
     // 保护三相都发出跳令, 其他情况直接返回0
     if (device->latchTripSignal[0] == 0 || device->latchTripSignal[1] == 0 || device->latchTripSignal[2] == 0) {
         return 0;
     }
     // 判断故障是否存在, 如果不存在故障, 返回0
     if (faultExist(device) == 0) {
         return 0;
     }
     // 三相跳令都为1, 且存在故障, 如果达到失灵保护整定时间
     if (device->time > device->startTime + device->failedProtectionTimeSet) {
         return 1;
     } else {
         return 0;
     }

 }


 int singleFailed(Device* device, int phase) {
     // 当前相是否发出过跳令, 如果锁存信号为0, 返回0
     if (device->latchTripSignal[phase] == 0) {
         return 0;
     }

     // 当前相电流是否>0.06In
     if (phasorAbs(device->phasor[3 + phase]) <= device->noCurrentSet) {
         // 小于无流整定值, 的确已经没有电流, 说明没有失灵
         return 0;
     }

     // 时间
     if (device->time > device->startTime + device->failedProtectionTimeSet) {
         return 1;
     }

     return 0;
 }

 /**
  * 2.1 判断当前是否存在故障
  * 只要任意一相存在故障, 就就认为故障存在, 返回1
  */
int faultExist(Device* device) {
    int phase = 0;
    // 零序电流
    Phasor zeroSeqCurrent;

    // 零序电流判定故障
    zeroSeqCurrent = phasorAdd(phasorAdd(device->phasor[3], device->phasor[4]), device->phasor[5]);
    if (phasorAbs(zeroSeqCurrent) > device->faultExistZeroSeqCurrentSet) {
        return 1;
    }

    // 相电流判定
    for (phase = 0; phase < 3; phase++) {
        if (phasorAbs(device->phasor[3 + phase]) > device->noCurrentSet) {
            return 1;
        }
    }

    return 0;
}


/**
 * 三相不一致保护
 * 发生任意一相位置不对应时, 三相不一致计时启动
 * 当延时到达后, 置位
 */
void threePhaseNotFit(Device* device) {

    if (device->threePhaseNotFitEnable == 0) {
        return;
    }

    // 任意一相TWJ，且不全部TWJ，三相不一致启动，计时
    if (TWJ(device, 0) || TWJ(device, 1) || TWJ(device, 2)) {
        if (!(TWJ(device, 0) && TWJ(device, 1) && TWJ(device, 2))) {
            if (notYet(device, "根据TWJ触发, 三相不一致启动")) {
                device->threePhaseNotFitStart = 1;
                device->threePhaseNotFitStartTime = device->time;
                writePlainLog(device, "TWJ, 三相不一致启动");
            }
        }
    }

    if (device->threePhaseNotFitStart && device->time > device->threePhaseNotFitStartTime + device->threePhaseNotFitTimeSet) {

        if (TWJ(device, 0) || TWJ(device, 1) || TWJ(device, 2)) {
            if (!(TWJ(device, 0) && TWJ(device, 1) && TWJ(device, 2))) {
                // 依然满足三相不一致条件
                if (notYet(device, "onlyOnce, 三相不一致动作")) {
                    device->threePhaseNotFitFlag[0] = 1;
                    device->threePhaseNotFitFlag[1] = 1;
                    device->threePhaseNotFitFlag[2] = 1;

                    // 闭锁重合闸
                    device->recloseShutFlag[0] = 1;
                    device->recloseShutFlag[1] = 1;
                    device->recloseShutFlag[2] = 1;
                    writeLog(device, "三相不一致动作");

                }
            }
        }
    }

}

/**
 * TWJ: 位置不对应 + 本相无流
 */
int TWJ(Device* device, int phase) {
     if (device->manBrkStatus[0][phase] == 1 && device->brkStatus[0][phase] == 0 && phasorAbs(device->phasor[3 + phase]) < device->noCurrentSet) {
         return 1;
     } else {
         return 0;
     }
}

