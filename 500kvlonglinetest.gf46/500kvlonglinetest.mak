
#------------------------------------------------------------------------------
# Project '500kvlonglinetest' make using the 'GFortran 4.6.2' compiler.
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
# All project
#------------------------------------------------------------------------------

all: targets
	@echo !--Make: succeeded.



#------------------------------------------------------------------------------
# Directories, Platform, and Version
#------------------------------------------------------------------------------

Arch        = windows
EmtdcDir    = E:\PSCAD46\emtdc\gf46
EmtdcInc    = $(EmtdcDir)\inc
EmtdcBin    = $(EmtdcDir)\$(Arch)
EmtdcMain   = $(EmtdcBin)\main.obj
EmtdcLib    = $(EmtdcBin)\emtdc.lib


#------------------------------------------------------------------------------
# Fortran Compiler
#------------------------------------------------------------------------------

FC_Name         = gfortran.exe
FC_Suffix       = o
FC_Args         = -c -ffree-form -fdefault-real-8 -fdefault-double-8
FC_Debug        = 
FC_Preprocess   = 
FC_Preproswitch = 
FC_Warn         = -Wconversion
FC_Checks       = 
FC_Includes     = -I"$(EmtdcInc)" -I"$(EmtdcBin)"
FC_Compile      = $(FC_Name) $(FC_Args) $(FC_Includes) $(FC_Debug) $(FC_Warn) $(FC_Checks)

#------------------------------------------------------------------------------
# C Compiler
#------------------------------------------------------------------------------

CC_Name     = gcc.exe
CC_Suffix   = o
CC_Args     = -c
CC_Debug    = 
CC_Includes = -I"$(EmtdcInc)" -I"$(EmtdcBin)"
CC_Compile  = $(CC_Name) $(CC_Args) $(CC_Includes) $(CC_Debug)

#------------------------------------------------------------------------------
# Linker
#------------------------------------------------------------------------------

Link_Name   = gcc.exe
Link_Debug  = 
Link_Args   = -o $@
Link        = $(Link_Name) $(Link_Args) $(Link_Debug)

#------------------------------------------------------------------------------
# Build rules for generated files
#------------------------------------------------------------------------------


%.$(FC_Suffix): %.f
	@echo !--Compile: $<
	$(FC_Compile) $<


%.$(CC_Suffix): %.c
	@echo !--Compile: $<
	$(CC_Compile) $<



#------------------------------------------------------------------------------
# Build rules for file references
#------------------------------------------------------------------------------


user_source_1.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\common.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\common.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\common.c" .
	$(CC_Compile) "common.c"
	del "common.c"

user_source_2.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\overCurrentRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\overCurrentRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\overCurrentRelay.c" .
	$(CC_Compile) "overCurrentRelay.c"
	del "overCurrentRelay.c"

user_source_3.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\distanceRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\distanceRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\distanceRelay.c" .
	$(CC_Compile) "distanceRelay.c"
	del "distanceRelay.c"

user_source_4.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\lineStarter.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\lineStarter.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\lineStarter.c" .
	$(CC_Compile) "lineStarter.c"
	del "lineStarter.c"

user_source_5.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\line.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\line.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\line.c" .
	$(CC_Compile) "line.c"
	del "line.c"

user_source_6.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\deltaDistanceRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\deltaDistanceRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\deltaDistanceRelay.c" .
	$(CC_Compile) "deltaDistanceRelay.c"
	del "deltaDistanceRelay.c"

user_source_7.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\relayMain.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\relayMain.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\relayMain.c" .
	$(CC_Compile) "relayMain.c"
	del "relayMain.c"

user_source_8.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\currentDiffRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\currentDiffRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\currentDiffRelay.c" .
	$(CC_Compile) "currentDiffRelay.c"
	del "currentDiffRelay.c"

user_source_9.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\zeroSeqCurrentRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\zeroSeqCurrentRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\zeroSeqCurrentRelay.c" .
	$(CC_Compile) "zeroSeqCurrentRelay.c"
	del "zeroSeqCurrentRelay.c"

user_source_10.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\bus.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\bus.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\bus.c" .
	$(CC_Compile) "bus.c"
	del "bus.c"

user_source_11.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\CTBreakRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\CTBreakRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\CTBreakRelay.c" .
	$(CC_Compile) "CTBreakRelay.c"
	del "CTBreakRelay.c"

user_source_12.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\PTbreakRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\PTbreakRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\PTbreakRelay.c" .
	$(CC_Compile) "PTbreakRelay.c"
	del "PTbreakRelay.c"

user_source_13.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\loadLimitRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\loadLimitRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\loadLimitRelay.c" .
	$(CC_Compile) "loadLimitRelay.c"
	del "loadLimitRelay.c"

user_source_14.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\lineTripGenerate.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\lineTripGenerate.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\lineTripGenerate.c" .
	$(CC_Compile) "lineTripGenerate.c"
	del "lineTripGenerate.c"

user_source_15.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\bus_500kV.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\bus_500kV.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\bus_500kV.c" .
	$(CC_Compile) "bus_500kV.c"
	del "bus_500kV.c"

user_source_16.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busAntiSaturation.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busAntiSaturation.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busAntiSaturation.c" .
	$(CC_Compile) "busAntiSaturation.c"
	del "busAntiSaturation.c"

user_source_17.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busAntiSaturation_500kV.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busAntiSaturation_500kV.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busAntiSaturation_500kV.c" .
	$(CC_Compile) "busAntiSaturation_500kV.c"
	del "busAntiSaturation_500kV.c"

user_source_18.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBlockLogic.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBlockLogic.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBlockLogic.c" .
	$(CC_Compile) "busBlockLogic.c"
	del "busBlockLogic.c"

user_source_19.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBranchFailureRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBranchFailureRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBranchFailureRelay.c" .
	$(CC_Compile) "busBranchFailureRelay.c"
	del "busBranchFailureRelay.c"

user_source_20.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureJudgeAux_500kV.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureJudgeAux_500kV.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureJudgeAux_500kV.c" .
	$(CC_Compile) "busBrkFailureJudgeAux_500kV.c"
	del "busBrkFailureJudgeAux_500kV.c"

user_source_21.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureRelay_500kV.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureRelay_500kV.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureRelay_500kV.c" .
	$(CC_Compile) "busBrkFailureRelay_500kV.c"
	del "busBrkFailureRelay_500kV.c"

user_source_22.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busBrkFailureRelay.c" .
	$(CC_Compile) "busBrkFailureRelay.c"
	del "busBrkFailureRelay.c"

user_source_23.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDeadZoneRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDeadZoneRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDeadZoneRelay.c" .
	$(CC_Compile) "busDeadZoneRelay.c"
	del "busDeadZoneRelay.c"

user_source_24.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffBackup.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffBackup.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffBackup.c" .
	$(CC_Compile) "busDiffBackup.c"
	del "busDiffBackup.c"

user_source_25.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffRelay.c" .
	$(CC_Compile) "busDiffRelay.c"
	del "busDiffRelay.c"

user_source_26.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffRelay_500kV.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffRelay_500kV.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busDiffRelay_500kV.c" .
	$(CC_Compile) "busDiffRelay_500kV.c"
	del "busDiffRelay_500kV.c"

user_source_27.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busModeRecognize.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busModeRecognize.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busModeRecognize.c" .
	$(CC_Compile) "busModeRecognize.c"
	del "busModeRecognize.c"

user_source_28.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busOpenPhaseRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busOpenPhaseRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busOpenPhaseRelay.c" .
	$(CC_Compile) "busOpenPhaseRelay.c"
	del "busOpenPhaseRelay.c"

user_source_29.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busOverCurrentRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busOverCurrentRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busOverCurrentRelay.c" .
	$(CC_Compile) "busOverCurrentRelay.c"
	del "busOverCurrentRelay.c"

user_source_30.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busPTBreakRelay.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busPTBreakRelay.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busPTBreakRelay.c" .
	$(CC_Compile) "busPTBreakRelay.c"
	del "busPTBreakRelay.c"

user_source_31.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busStarter.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busStarter.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busStarter.c" .
	$(CC_Compile) "busStarter.c"
	del "busStarter.c"

user_source_32.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busStarter_500kV.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busStarter_500kV.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busStarter_500kV.c" .
	$(CC_Compile) "busStarter_500kV.c"
	del "busStarter_500kV.c"

user_source_33.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\busTripGenerate.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busTripGenerate.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\busTripGenerate.c" .
	$(CC_Compile) "busTripGenerate.c"
	del "busTripGenerate.c"

user_source_34.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaTelemetry.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaTelemetry.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaTelemetry.c" .
	$(CC_Compile) "meaTelemetry.c"
	del "meaTelemetry.c"

user_source_35.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaSample.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaSample.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaSample.c" .
	$(CC_Compile) "meaSample.c"
	del "meaSample.c"

user_source_36.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\imp.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\imp.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\imp.c" .
	$(CC_Compile) "imp.c"
	del "imp.c"

user_source_37.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\deltaDiff.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\deltaDiff.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\deltaDiff.c" .
	$(CC_Compile) "deltaDiff.c"
	del "deltaDiff.c"

user_source_38.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaRecord.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaRecord.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaRecord.c" .
	$(CC_Compile) "meaRecord.c"
	del "meaRecord.c"

user_source_39.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\lowSideDiff.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\lowSideDiff.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\lowSideDiff.c" .
	$(CC_Compile) "lowSideDiff.c"
	del "lowSideDiff.c"

user_source_40.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\lonDiff.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\lonDiff.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\lonDiff.c" .
	$(CC_Compile) "lonDiff.c"
	del "lonDiff.c"

user_source_41.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\splitDiff.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\splitDiff.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\splitDiff.c" .
	$(CC_Compile) "splitDiff.c"
	del "splitDiff.c"

user_source_42.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\overCur.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\overCur.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\overCur.c" .
	$(CC_Compile) "overCur.c"
	del "overCur.c"

user_source_43.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\tranStarter.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\tranStarter.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\tranStarter.c" .
	$(CC_Compile) "tranStarter.c"
	del "tranStarter.c"

user_source_44.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\tran.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\tran.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\tran.c" .
	$(CC_Compile) "tran.c"
	del "tran.c"

user_source_45.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\zeroDiff.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\zeroDiff.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\tran\zeroDiff.c" .
	$(CC_Compile) "zeroDiff.c"
	del "zeroDiff.c"

user_source_46.$(CC_Suffix): E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaTelecontrol.c
	@echo !--Compile: "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaTelecontrol.c"
	copy "E:\Modeling_of_Smart_Substations\MeasurementTest\code\mea\meaTelecontrol.c" .
	$(CC_Compile) "meaTelecontrol.c"
	del "meaTelecontrol.c"

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------


FC_Objects = \
 Station.$(FC_Suffix) \
 Main.$(FC_Suffix)

FC_ObjectsLong = \
 "Station.$(FC_Suffix)" \
 "Main.$(FC_Suffix)"

CC_Objects = \
  user_source_1.$(CC_Suffix) \
  user_source_2.$(CC_Suffix) \
  user_source_3.$(CC_Suffix) \
  user_source_4.$(CC_Suffix) \
  user_source_5.$(CC_Suffix) \
  user_source_6.$(CC_Suffix) \
  user_source_7.$(CC_Suffix) \
  user_source_8.$(CC_Suffix) \
  user_source_9.$(CC_Suffix) \
  user_source_10.$(CC_Suffix) \
  user_source_11.$(CC_Suffix) \
  user_source_12.$(CC_Suffix) \
  user_source_13.$(CC_Suffix) \
  user_source_14.$(CC_Suffix) \
  user_source_15.$(CC_Suffix) \
  user_source_16.$(CC_Suffix) \
  user_source_17.$(CC_Suffix) \
  user_source_18.$(CC_Suffix) \
  user_source_19.$(CC_Suffix) \
  user_source_20.$(CC_Suffix) \
  user_source_21.$(CC_Suffix) \
  user_source_22.$(CC_Suffix) \
  user_source_23.$(CC_Suffix) \
  user_source_24.$(CC_Suffix) \
  user_source_25.$(CC_Suffix) \
  user_source_26.$(CC_Suffix) \
  user_source_27.$(CC_Suffix) \
  user_source_28.$(CC_Suffix) \
  user_source_29.$(CC_Suffix) \
  user_source_30.$(CC_Suffix) \
  user_source_31.$(CC_Suffix) \
  user_source_32.$(CC_Suffix) \
  user_source_33.$(CC_Suffix) \
  user_source_34.$(CC_Suffix) \
  user_source_35.$(CC_Suffix) \
  user_source_36.$(CC_Suffix) \
  user_source_37.$(CC_Suffix) \
  user_source_38.$(CC_Suffix) \
  user_source_39.$(CC_Suffix) \
  user_source_40.$(CC_Suffix) \
  user_source_41.$(CC_Suffix) \
  user_source_42.$(CC_Suffix) \
  user_source_43.$(CC_Suffix) \
  user_source_44.$(CC_Suffix) \
  user_source_45.$(CC_Suffix) \
  user_source_46.$(CC_Suffix)

CC_ObjectsLong = \
  "common.$(CC_Suffix)" \
  "overCurrentRelay.$(CC_Suffix)" \
  "distanceRelay.$(CC_Suffix)" \
  "lineStarter.$(CC_Suffix)" \
  "line.$(CC_Suffix)" \
  "deltaDistanceRelay.$(CC_Suffix)" \
  "relayMain.$(CC_Suffix)" \
  "currentDiffRelay.$(CC_Suffix)" \
  "zeroSeqCurrentRelay.$(CC_Suffix)" \
  "bus.$(CC_Suffix)" \
  "CTBreakRelay.$(CC_Suffix)" \
  "PTbreakRelay.$(CC_Suffix)" \
  "loadLimitRelay.$(CC_Suffix)" \
  "lineTripGenerate.$(CC_Suffix)" \
  "bus_500kV.$(CC_Suffix)" \
  "busAntiSaturation.$(CC_Suffix)" \
  "busAntiSaturation_500kV.$(CC_Suffix)" \
  "busBlockLogic.$(CC_Suffix)" \
  "busBranchFailureRelay.$(CC_Suffix)" \
  "busBrkFailureJudgeAux_500kV.$(CC_Suffix)" \
  "busBrkFailureRelay_500kV.$(CC_Suffix)" \
  "busBrkFailureRelay.$(CC_Suffix)" \
  "busDeadZoneRelay.$(CC_Suffix)" \
  "busDiffBackup.$(CC_Suffix)" \
  "busDiffRelay.$(CC_Suffix)" \
  "busDiffRelay_500kV.$(CC_Suffix)" \
  "busModeRecognize.$(CC_Suffix)" \
  "busOpenPhaseRelay.$(CC_Suffix)" \
  "busOverCurrentRelay.$(CC_Suffix)" \
  "busPTBreakRelay.$(CC_Suffix)" \
  "busStarter.$(CC_Suffix)" \
  "busStarter_500kV.$(CC_Suffix)" \
  "busTripGenerate.$(CC_Suffix)" \
  "meaTelemetry.$(CC_Suffix)" \
  "meaSample.$(CC_Suffix)" \
  "imp.$(CC_Suffix)" \
  "deltaDiff.$(CC_Suffix)" \
  "meaRecord.$(CC_Suffix)" \
  "lowSideDiff.$(CC_Suffix)" \
  "lonDiff.$(CC_Suffix)" \
  "splitDiff.$(CC_Suffix)" \
  "overCur.$(CC_Suffix)" \
  "tranStarter.$(CC_Suffix)" \
  "tran.$(CC_Suffix)" \
  "zeroDiff.$(CC_Suffix)" \
  "meaTelecontrol.$(CC_Suffix)"

UserLibs =

SysLibs  = -lgfortran -lstdc++ -lquadmath -lwsock32

Binary   = 500kvlonglinetest.exe

$(Binary): $(FC_Objects) $(CC_Objects) $(UserLibs)
	@echo !--Link: $@
	$(Link) "$(EmtdcMain)" *.$(CC_Suffix) $(UserLibs) "$(EmtdcLib)" $(SysLibs)

targets: $(Binary)


clean:
	-del EMTDC_V*
	-del *.obj
	-del *.o
	-del *.exe
	@echo !--Make clean: succeeded.



