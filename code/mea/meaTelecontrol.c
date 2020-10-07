//
// Created by yl on 2020/10/5.
//
#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
//#include "..\\dataStruct.h"
//#include "..\\common.h"

void meaTelecontrol(meaDevice* device) {
    device->tripSignals = device->remoteControlSignal;
}
