cmake_minimum_required(VERSION 3.14)
project(RelayTest C)

set(CMAKE_C_STANDARD 99)

add_executable(RelayTest
        code/common.c
        code/common.h
        code/dataStruct.h
        code/relayMain.c
        code/line.c
        code/lineStarter.c
        code/currentDiffRelay.c
        code/deltaDistanceRelay.c
        code/distanceRelay.c
        code/zeroSeqCurrentRelay.c
        code/overCurrentRelay.c
        code/PTbreakRelay.c
        code/CTBreakRelay.c
        code/lineTripGenerate.c
        code/bus.c
        code/bus_500kV.c
        code/busAntiSaturation.c
        code/busAntiSaturation_500kV.c
        code/busBlockLogic.c
        code/busBranchFailureRelay.c
        code/busBrkFailureJudgeAux_500kV.c
        code/busBrkFailureRelay_500kV.c
        code/busBrkFailureRelay.c
        code/busDeadZoneRelay.c
        code/busDiffBackup.c
        code/busDiffRelay.c
        code/busDiffRelay_500kV.c
        code/busModeRecognize.c
        code/busOpenPhaseRelay.c
        code/busOverCurrentRelay.c
        code/busPTBreakRelay.c
        code/busStarter.c
        code/busStarter_500kV.c
        code/busTripGenerate.c
        code/mea/meaSample.c
        code/tran/tran.c
        code/tran/tranStarter.c
        code/tran/deltaDiff.c
        code/tran/lonDiff.c
        code/tran/splitDiff.c
        code/tran/zeroDiff.c
        code/tran/lowSideDiff.c
        code/tran/imp.c
        code/tran/overCur.c
        code/mea/meaTelemetry.c code/mea/meaRecord.c code/mea/meaTelecontrol.c)