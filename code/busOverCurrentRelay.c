// 母联及分段充电过流保护

#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"

void busOverCurrentRelay(BusDevice* bus) 
{   
  //以下为母联充电过流保护
	//double BusTieOverCurrentSetValue[2];//母联充电过流I\II整定值
	double BusTieOverCurrentSetValue[2]={1,0.8};//母联充电过流I\II整定值
    double BusTieZeroSetValue=0.3;//bus->chargeZeroCurrentSetValue;//母联零序过流整定值,3倍
	double BusOverCurrentTimeSetValue[2]={0.2,0.4};//母联和分段过流保护延时定值
	//参数本地化
	Phasor IBC[6]={bus->busPhasor[3],bus->busPhasor[4],bus->busPhasor[5],bus->busPhasor[9],bus->busPhasor[10],bus->busPhasor[11]};//母联断路器1（3 4 5）,2（9 10 11）对应的ABC三相电流值
	Phasor BusTiezerocurrent[2];
	BusTiezerocurrent[0]=phasorSeq(IBC[0], IBC[1], IBC[2], 0);//母联I零序电流
	BusTiezerocurrent[1]=phasorSeq(IBC[3], IBC[4], IBC[5], 0);//母联II零序电流
  //以下为分段充电过流保护
    double BusSecOverCurrentSetValue[2]={1,0.8};//分段充电过流整定值
	double BusSecZeroSetValue=0.3;//分段零序过流整定值
    double BusZerotimeSetValue=0.3;//母联和分段零序判据延时
	//参数本地化
    Phasor IFD[6]={bus->busPhasor[15],bus->busPhasor[16],bus->busPhasor[17],bus->busPhasor[21],bus->busPhasor[22],bus->busPhasor[23]};
    Phasor BusSeczerocurrent[2];
	BusSeczerocurrent[0]= phasorNumMulti(3.0,phasorSeq(IFD[0], IFD[1], IFD[2], 0));//分段I零序电流
	BusSeczerocurrent[1]=phasorNumMulti(3.0,phasorSeq(IFD[3], IFD[4], IFD[5], 0));//分段II零序电流
  
  if(bus->busBrkChargeOcIEnable[0]==1)
  {
  if( phasorAbs(IBC[0])>BusTieOverCurrentSetValue[0] || phasorAbs(IBC[1])>BusTieOverCurrentSetValue[0] || phasorAbs(IBC[2])>BusTieOverCurrentSetValue[0] )//电流幅值大于定值
   {
   bus->busTieOverCurrentStartFlag[0]=1;
   busWriteLog(bus, "母联1充电相过流保护I段启动");
   if(bus->busTieOverCurrentStartTime[0]==0) //且起始时间是0
	  {bus->busTieOverCurrentStartTime[0]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busTieOverCurrentStartTime[0]!=0 && (bus->time - bus->busTieOverCurrentStartTime[0] >= BusOverCurrentTimeSetValue[0]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOverCurrentFlag[0]= 1;
			bus->busTieOverCurrentStartTime[0]=0;//动作之后还要清一次
            busWriteLog(bus, "母联1充电相过流保护I段跳母联1");
        }
   }
  else{bus->busTieOverCurrentStartTime[0]=0;} //判据不满足时，把保护起始时间清掉
  }
//MLI 1

  if(bus->busBrkChargeOcIEnable[1]==1)
  {
  if( phasorAbs(IBC[3])>BusTieOverCurrentSetValue[0] || phasorAbs(IBC[4])>BusTieOverCurrentSetValue[0] || phasorAbs(IBC[5])>BusTieOverCurrentSetValue[0] )//电流幅值大于定值
   {
   bus->busTieOverCurrentStartFlag[1]=1;
   busWriteLog(bus, "母联2充电相过流保护I段启动");
   if(bus->busTieOverCurrentStartTime[1]==0) //且起始时间是0
	  {bus->busTieOverCurrentStartTime[1]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busTieOverCurrentStartTime[1]!=0 && (bus->time - bus->busTieOverCurrentStartTime[1] >= BusOverCurrentTimeSetValue[0]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOverCurrentFlag[1]= 1;
			bus->busTieOverCurrentStartTime[1]=0;//动作之后还要清一次
            busWriteLog(bus, "母联2充电相过流保护I段跳母联2");
        }
   }
  else{bus->busTieOverCurrentStartTime[1]=0;} //判据不满足时，把保护起始时间清掉
  }
//MLII 1


  if(bus->busBrkChargeOcIEnable[2]==1)
  {
  if( phasorAbs(IFD[0])>BusSecOverCurrentSetValue[0] || phasorAbs(IFD[1])>BusSecOverCurrentSetValue[0] || phasorAbs(IFD[2])>BusSecOverCurrentSetValue[0] )//电流幅值大于定值
   {
   bus->busSecOverCurrentStartFlag[0]=1;
   busWriteLog(bus, "分段1充电相过流保护I段启动");
   if(bus->busSecOverCurrentStartTime[0]==0) //且起始时间是0
	  {bus->busSecOverCurrentStartTime[0]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busSecOverCurrentStartTime[0]!=0 && (bus->time - bus->busSecOverCurrentStartTime[0] >= BusOverCurrentTimeSetValue[0]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOverCurrentFlag[0]= 1;
			bus->busSecOverCurrentStartTime[0]=0;//动作之后还要清一次
            busWriteLog(bus, "分段1充电相过流保护I段跳分段1");
        }
   }
  else{bus->busSecOverCurrentStartTime[0]=0;} //判据不满足时，把保护起始时间清掉
  }
//FDI 1

  if(bus->busBrkChargeOcIEnable[3]==1)
  {
  if( phasorAbs(IFD[3])>BusSecOverCurrentSetValue[0] || phasorAbs(IFD[4])>BusSecOverCurrentSetValue[0] || phasorAbs(IFD[5])>BusSecOverCurrentSetValue[0] )//电流幅值大于定值
   {
   bus->busSecOverCurrentStartFlag[1]=1;
   busWriteLog(bus, "分段2充电相过流保护I段启动");
   if(bus->busSecOverCurrentStartTime[1]==0) //且起始时间是0
	  {bus->busSecOverCurrentStartTime[1]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busSecOverCurrentStartTime[1]!=0 && (bus->time - bus->busSecOverCurrentStartTime[1] >= BusOverCurrentTimeSetValue[0]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOverCurrentFlag[1]= 1;
			bus->busSecOverCurrentStartTime[1]=0;//动作之后还要清一次
            busWriteLog(bus, "分段2充电相过流保护I段跳分段2");
        }
   }
  else{bus->busSecOverCurrentStartTime[1]=0;} //判据不满足时，把保护起始时间清掉
  }
//FDII 1



  if(bus->busBrkChargeOcIIEnable[0]==1)
  {
  if( phasorAbs(IBC[0])>BusTieOverCurrentSetValue[1] || phasorAbs(IBC[1])>BusTieOverCurrentSetValue[1] || phasorAbs(IBC[2])>BusTieOverCurrentSetValue[1] )//电流幅值大于定值
   {
   bus->busTieOverCurrentStartFlag[2]=1;
   busWriteLog(bus, "母联1充电相过流保护II段启动");
   if(bus->busTieOverCurrentStartTime[2]==0) //且起始时间是0
	  {bus->busTieOverCurrentStartTime[2]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busTieOverCurrentStartTime[2]!=0 && (bus->time - bus->busTieOverCurrentStartTime[2] >= BusOverCurrentTimeSetValue[1]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOverCurrentFlag[2]= 1;
			bus->busTieOverCurrentStartTime[2]=0;//动作之后还要清一次
            busWriteLog(bus, "母联1充电相过流保护II段跳母联1");
        }
   }
  else{bus->busTieOverCurrentStartTime[2]=0;} //判据不满足时，把保护起始时间清掉
  }
//MLI 2

  if(bus->busBrkChargeOcIIEnable[1]==1)
  {
  if( phasorAbs(IBC[3])>BusTieOverCurrentSetValue[1] || phasorAbs(IBC[4])>BusTieOverCurrentSetValue[1] || phasorAbs(IBC[5])>BusTieOverCurrentSetValue[1] )//电流幅值大于定值
   {
   bus->busTieOverCurrentStartFlag[3]=1;
   busWriteLog(bus, "母联2充电相过流保护II段启动");
   if(bus->busTieOverCurrentStartTime[3]==0) //且起始时间是0
	  {bus->busTieOverCurrentStartTime[3]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busTieOverCurrentStartTime[3]!=0 && (bus->time - bus->busTieOverCurrentStartTime[3] >= BusOverCurrentTimeSetValue[1]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOverCurrentFlag[3]= 1;
			bus->busTieOverCurrentStartTime[3]=0;//动作之后还要清一次
            busWriteLog(bus, "母联2充电相过流保护II段跳母联2");
        }
   }
  else{bus->busTieOverCurrentStartTime[3]=0;} //判据不满足时，把保护起始时间清掉
  }
//MLII 2


  if(bus->busBrkChargeOcIIEnable[2]==1)
  {
  if( phasorAbs(IFD[0])>BusSecOverCurrentSetValue[1] || phasorAbs(IFD[1])>BusSecOverCurrentSetValue[1] || phasorAbs(IFD[2])>BusSecOverCurrentSetValue[1] )//电流幅值大于定值
   {
   bus->busSecOverCurrentStartFlag[2]=1;
   busWriteLog(bus, "分段1充电过流保护II段启动");
   if(bus->busSecOverCurrentStartTime[2]==0) //且起始时间是0
	  {bus->busSecOverCurrentStartTime[2]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busSecOverCurrentStartTime[2]!=0 && (bus->time - bus->busSecOverCurrentStartTime[2] >= BusOverCurrentTimeSetValue[1]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOverCurrentFlag[2]= 1;
			bus->busSecOverCurrentStartTime[2]=0;//动作之后还要清一次
            busWriteLog(bus, "分段1充电过流保护II段跳分段1");
        }
   }
  else{bus->busSecOverCurrentStartTime[2]=0;} //判据不满足时，把保护起始时间清掉
  }
//FDI 2

  if(bus->busBrkChargeOcIIEnable[3]==1)
  {
  if( phasorAbs(IFD[3])>BusSecOverCurrentSetValue[1] || phasorAbs(IFD[4])>BusSecOverCurrentSetValue[1] || phasorAbs(IFD[5])>BusSecOverCurrentSetValue[1] )//电流幅值大于定值
   {
   bus->busSecOverCurrentStartFlag[3]=1;
   busWriteLog(bus, "分段2充电相过流保护II段启动");
   if(bus->busSecOverCurrentStartTime[3]==0) //且起始时间是0
	  {bus->busSecOverCurrentStartTime[3]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busSecOverCurrentStartTime[3]!=0 && (bus->time - bus->busSecOverCurrentStartTime[3] >= BusOverCurrentTimeSetValue[1]) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOverCurrentFlag[3]= 1;
			bus->busSecOverCurrentStartTime[3]=0;//动作之后还要清一次
            busWriteLog(bus, "分段2充电相过流保护II段跳分段2");
        }
   }
  else{bus->busSecOverCurrentStartTime[3]=0;} //判据不满足时，把保护起始时间清掉
  }
//FDII 2






if(bus->busBrkChargeOcZeroEnable[0]==1)
 {
  if( phasorAbs(BusTiezerocurrent[0])>BusTieZeroSetValue)//电流幅值大)//电流幅值大于定值
   {
   bus->busTieOverCurrentStartFlag[4]=1;
   busWriteLog(bus, "母联1充电过流零序保护启动");
   if(bus->busTieOverCurrentStartTime[4]==0) //且起始时间是0
	  {bus->busTieOverCurrentStartTime[4]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busTieOverCurrentStartTime[4]!=0 && (bus->time - bus->busTieOverCurrentStartTime[4] >= BusZerotimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOverCurrentFlag[4]= 1;
			bus->busTieOverCurrentStartTime[4]=0;//动作之后还要清一次
            busWriteLog(bus, "母联1充电过流零序保护跳母联1");
        }
   }
  else{bus->busTieOverCurrentStartTime[4]=0;} //判据不满足时，把保护起始时间清掉
  }
//MLI 0

if(bus->busBrkChargeOcZeroEnable[1]==1)
 {
  if(phasorAbs(BusTiezerocurrent[1])>BusTieZeroSetValue)//电流幅值大)//电流幅值大于定值
   {
   bus->busTieOverCurrentStartFlag[5]=1;
   busWriteLog(bus, "母联2充电过流零序保护启动");
   if(bus->busTieOverCurrentStartTime[5]==0) //且起始时间是0
	  {bus->busTieOverCurrentStartTime[5]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busTieOverCurrentStartTime[5]!=0 && (bus->time - bus->busTieOverCurrentStartTime[5] >= BusZerotimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieOverCurrentFlag[5]= 1;
			bus->busTieOverCurrentStartTime[5]=0;//动作之后还要清一次
            busWriteLog(bus, "母联2充电过流零序保护跳母联2");
        }
   }
  else{bus->busTieOverCurrentStartTime[5]=0;} //判据不满足时，把保护起始时间清掉
  }
//MLII 0


if(bus->busBrkChargeOcZeroEnable[2]==1)
 {
  if(phasorAbs(BusSeczerocurrent[0])>BusSecZeroSetValue)//电流幅值大)//电流幅值大于定值
   {
   bus->busSecOverCurrentStartFlag[4]=1;
   busWriteLog(bus, "分段1充电过流零序保护启动");
   if(bus->busSecOverCurrentStartTime[4]==0) //且起始时间是0
	  {bus->busSecOverCurrentStartTime[4]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busSecOverCurrentStartTime[4]!=0 && (bus->time - bus->busSecOverCurrentStartTime[4] >= BusZerotimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOverCurrentFlag[4]= 1;
			bus->busSecOverCurrentStartTime[4]=0;//动作之后还要清一次
            busWriteLog(bus, "分段1充电过流零序保护跳分段1");
        }
   }
  else{bus->busSecOverCurrentStartTime[4]=0;} //判据不满足时，把保护起始时间清掉
  }
//FDI 0

if(bus->busBrkChargeOcZeroEnable[3]==1)
 {
  if(phasorAbs(BusSeczerocurrent[1])>BusSecZeroSetValue)//电流幅值大)//电流幅值大于定值
   {
   bus->busSecOverCurrentStartFlag[5]=1;
   busWriteLog(bus, "分段2充电过流零序保护启动");
   if(bus->busSecOverCurrentStartTime[5]==0) //且起始时间是0
	  {bus->busSecOverCurrentStartTime[5]=bus->time;} //则判据满足，把当前时间给起始时间

   if( bus->busSecOverCurrentStartTime[5]!=0 && (bus->time - bus->busSecOverCurrentStartTime[5] >= BusZerotimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busSecOverCurrentFlag[5]= 1;
			bus->busSecOverCurrentStartTime[5]=0;//动作之后还要清一次
            busWriteLog(bus, "分段2充电过流零序保护跳分段2");
        }
   }
  else{bus->busSecOverCurrentStartTime[5]=0;} //判据不满足时，把保护起始时间清掉
  }



}