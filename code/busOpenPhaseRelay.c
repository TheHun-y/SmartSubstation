// 母联失灵保护及分段失灵保护

#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

void busOpenphaseRelay(BusDevice* bus) 
{   
	//以下为母联非全相保护
	double BusTieOpenphase2setvalue=0.05;//母联非全相负序电流整定值
	double BusTieOpenphase0setvalue=0.05;//母联非全相零序电流整定值
	double BusSecOpenphase2setvalue=0.05;//分段非全相负序电流整定值
	double BusSecOpenphase0setvalue=0.05;//分段非全相零序电流整定值
	double BusOpenphaseTimeSetValue=0.005;//非全相延时整定值

	//参数本地化
	int i,HWJ[12],TWJ[12];
	for(i=0;i<12;i++)
	{
        HWJ[i] =bus->busBrkStatus[i];
		TWJ[i] =!(bus->busBrkStatus[i]);
    }
	Phasor IBT[6]={bus->busPhasor[3],bus->busPhasor[4],bus->busPhasor[5],bus->busPhasor[9],bus->busPhasor[10],bus->busPhasor[11]};//母联断路器1（3 4 5）,2（9 10 11）对应的ABC三相电流值
	Phasor BusTieI0Current=phasorSeq(IBT[0], IBT[1], IBT[2], 0);//母联I零序电流
	Phasor BusTieI2Current=phasorSeq(IBT[0], IBT[1], IBT[2], 2);//母联I负序电流
	Phasor BusTieII0Current=phasorSeq(IBT[3], IBT[4], IBT[5], 0);//母联II零序电流
	Phasor BusTieII2Current=phasorSeq(IBT[3], IBT[4], IBT[5], 2);//母联II负序电流
	Phasor IBF[6]={bus->busPhasor[15],bus->busPhasor[16],bus->busPhasor[17],bus->busPhasor[21],bus->busPhasor[22],bus->busPhasor[23]};
    Phasor BusSecI0Current=phasorSeq(IBF[0], IBF[1], IBF[2], 0);//分段I零序电流
	Phasor BusSecI2Current=phasorSeq(IBF[0], IBF[1], IBF[2], 2);//分段I负序电流
	Phasor BusSecII0Current=phasorSeq(IBF[3], IBF[4], IBF[5], 0);//分段II零序电流
	Phasor BusSecII2Current=phasorSeq(IBF[3], IBF[4], IBF[5], 2);//分段II负序电流
	
   //if(((TWJ[0]||TWJ[1]||TWJ[2])&&(HWJ[0]||HWJ[1]||HWJ[2]))&&(phasorAbs(BusTieI0Current)>BusTieOpenphase0setvalue||phasorAbs(BusTieI2Current)>BusTieOpenphase2setvalue)&&(bus->busOpenphaseEnable[0]&&bus->busOpenphaseEnable[1]))
   if(((TWJ[0]||TWJ[1]||TWJ[2])&&(HWJ[0]||HWJ[1]||HWJ[2]))&&(phasorAbs(BusTieI0Current)>BusTieOpenphase0setvalue||phasorAbs(BusTieI2Current)>BusTieOpenphase2setvalue)&&(bus->busIncompletePhaseEnable[0]))
   { 
     if(bus->busTieOpenphaseStartTime[0]==0) //起始时间是0
	  {bus->busTieOpenphaseStartTime[0]=bus->time;} //则判据满足，把当前时间给起始时间
     
   if( bus->busTieOpenphaseStartTime[0]!=0 && (bus->time - bus->busTieOpenphaseStartTime[0] >=BusOpenphaseTimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOpenphaseFlag[0]= 1;
			bus->busTieOpenphaseStartTime[0]=0;//动作之后还要清一次
            busWriteLog(bus, "非全相保护动作，母联I跳闸");
        }
   }
   else{bus->busTieOpenphaseStartTime[0]=0;} //判据不满足时，把保护起始时间清掉


   //if(((TWJ[3]||TWJ[4]||TWJ[5])&&(HWJ[3]||HWJ[4]||HWJ[5]))&&( phasorAbs(BusTieII0Current)>BusTieOpenphase0setvalue||phasorAbs(BusTieII2Current)>BusTieOpenphase2setvalue)&&(bus->busOpenphaseEnable[2]&&bus->busOpenphaseEnable[3]))
   if(((TWJ[3]||TWJ[4]||TWJ[5])&&(HWJ[3]||HWJ[4]||HWJ[5]))&&( phasorAbs(BusTieII0Current)>BusTieOpenphase0setvalue||phasorAbs(BusTieII2Current)>BusTieOpenphase2setvalue)&&(bus->busIncompletePhaseEnable[1]))
   { 
     if(bus->busTieOpenphaseStartTime[1]==0) //起始时间是0
	  {bus->busTieOpenphaseStartTime[1]=bus->time;} //则判据满足，把当前时间给起始时间
     
   if( bus->busTieOpenphaseStartTime[1]!=0 && (bus->time - bus->busTieOpenphaseStartTime[1] >=BusOpenphaseTimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOpenphaseFlag[1]= 1;
			bus->busTieOpenphaseStartTime[1]=0;//动作之后还要清一次
            busWriteLog(bus, "非全相保护动作，母联II跳闸");
        }
   }
   else{bus->busTieOpenphaseStartTime[1]=0;} //判据不满足时，把保护起始时间清掉
  
   //以下为分段非全相保护
   //if(((TWJ[6]||TWJ[7]||TWJ[8])&&(HWJ[6]||HWJ[7]||HWJ[8]))&&( phasorAbs(BusSecI0Current)>BusSecOpenphase0setvalue||phasorAbs(BusSecI2Current)>BusSecOpenphase2setvalue)&&(bus->busOpenphaseEnable[4]&&bus->busOpenphaseEnable[5]))
   if(((TWJ[6]||TWJ[7]||TWJ[8])&&(HWJ[6]||HWJ[7]||HWJ[8]))&&( phasorAbs(BusSecI0Current)>BusSecOpenphase0setvalue||phasorAbs(BusSecI2Current)>BusSecOpenphase2setvalue)&&(bus->busIncompletePhaseEnable[2]))
   { 
     if(bus->busSecOpenphaseStartTime[0]==0) //起始时间是0
	  {bus->busSecOpenphaseStartTime[0]=bus->time;} //则判据满足，把当前时间给起始时间
     
   if( bus->busSecOpenphaseStartTime[0]!=0 && (bus->time - bus->busSecOpenphaseStartTime[0] >=BusOpenphaseTimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOpenphaseFlag[0]= 1;
			bus->busSecOpenphaseStartTime[0]=0;//动作之后还要清一次
            busWriteLog(bus, "非全相保护动作，分段I跳闸");
        }
   }
   else{bus->busSecOpenphaseStartTime[0]=0;} //判据不满足时，把保护起始时间清掉

	//if(((TWJ[9]||TWJ[10]||TWJ[11])&&(HWJ[9]||HWJ[10]||HWJ[11]))&&( phasorAbs(BusSecII0Current)>BusSecOpenphase0setvalue||phasorAbs(BusSecII2Current)>BusSecOpenphase2setvalue)&&(bus->busOpenphaseEnable[6]&&bus->busOpenphaseEnable[7]))
	if(((TWJ[9]||TWJ[10]||TWJ[11])&&(HWJ[9]||HWJ[10]||HWJ[11]))&&( phasorAbs(BusSecII0Current)>BusSecOpenphase0setvalue||phasorAbs(BusSecII2Current)>BusSecOpenphase2setvalue)&&(bus->busIncompletePhaseEnable[3]))
   { 
     if(bus->busSecOpenphaseStartTime[1]==0) //起始时间是0
	  {bus->busSecOpenphaseStartTime[1]=bus->time;} //则判据满足，把当前时间给起始时间
    
   if( bus->busSecOpenphaseStartTime[1]!=0 && (bus->time - bus->busSecOpenphaseStartTime[1] >=BusOpenphaseTimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOpenphaseFlag[1]= 1;
			bus->busSecOpenphaseStartTime[1]=0;//动作之后还要清一次
            busWriteLog(bus, "非全相保护动作，分段II跳闸");
        }
   }
    else{bus->busSecOpenphaseStartTime[1]=0;} //判据不满足时，把保护起始时间清掉
  
}