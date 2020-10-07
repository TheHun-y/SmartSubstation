#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

void loadLimitRelayPhase(Device* device, int phase);
void loadLimitRelayGround(Device* device, int phase);

 void loadLimitRelay(Device* device, int phase){
  
        loadLimitRelayPhase(device, phase);
        loadLimitRelayGround(device, phase);

}

void loadLimitRelayPhase(Device* device, int phase){

     Phasor Up1, Up2, Upp, Ip1, Ip2, Ipp, Zm;
     double arg;

     Phasor Rzd;
     Rzd.real = device->loadLimitSetValue;
     Rzd.img = 0.0;
     
     double theta = device->lineZ1Angle;

     // 变量本地化
    Up1 = device->phasor[phase];
    Ip1 = device->phasor[phase + 3];
    Up2 = device->phasor[phaseMatch(phase)];
    Ip2 = device->phasor[phaseMatch(phase) + 3];

    Upp = phasorSub(Up1, Up2);
    Ipp = phasorSub(Ip1, Ip2);

    // 计算测量阻抗及判据实现
    Zm = phasorDiv(Upp, Ipp);

    arg = phasorAngle(phasorSub(Zm, Rzd));

    if (theta <= arg <= (theta + 180 )){
        device->loadLimitTripFlag[phase + 3] = 1;
    }

 }

void loadLimitRelayGround(Device* device, int phase){

     Phasor Up, Ip, I0, Ik, Zm;
     double arg;

     Phasor Rzd;
     Rzd.real = device->loadLimitSetValue;
     Rzd.img = 0.0;
     
     double theta = device->lineZ1Angle;
     
     double k = device->KZ;

     // 变量本地化
    Up = device->phasor[phase];
    Ip = device->phasor[phase + 3];
    I0 = phasorSeq(device->phasor[3], device->phasor[4], device->phasor[5], 0);
    Ik = phasorAdd(Ip, phasorNumMulti(3.0*k, I0));

    // 计算测量阻抗及判据实现
    Zm = phasorDiv(Up, Ik);

    arg = phasorAngle(phasorSub(Zm, Rzd));

    if (theta <= arg <= (theta + 180 )){
        device->loadLimitTripFlag[phase] = 1;
    }

 }

