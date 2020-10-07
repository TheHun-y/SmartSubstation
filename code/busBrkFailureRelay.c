// 母联失灵保护及分段失灵保护

#include "..\\code\\dataStruct.h"
#include "..\\code\\common.h"
#include <stdio.h>

void busBrkFailureRelay(BusDevice* bus) 
{  
	//以下为母联失灵保护
	int a = 0,b = 0,c = 0,d = 0;//内部逻辑判断结果
	double BusFailuCurSetValue=bus->busBrkFailureSetValue[0];//母联分段失灵电流定值
	double BusFailuTimeSetValue=bus->busBrkFailureSetTime[0];//母联分段失灵动作延时定值
	double BusTie_ThrPhaFailueOpen=0;//母联三相启动失灵开入控制字
	//参数本地化
	Phasor IBC[6]={bus->busPhasor[3],bus->busPhasor[4],bus->busPhasor[5],bus->busPhasor[9],bus->busPhasor[10],bus->busPhasor[11]};//母联断路器1(3 4 5),2(9 10 11)对应的ABC三相电流值
	double BusvotBlockOpen[4]={bus->voltBlockFlag[0],bus->voltBlockFlag[1],bus->voltBlockFlag[2],bus->voltBlockFlag[3]};//母联断路器1,2对应的I,II母电压闭锁状态
	double BusTripFlag[4]={bus->busBrkTripFlag[0],bus->busBrkTripFlag[1],bus->busBrkTripFlag[2],bus->busBrkTripFlag[3]};//母差保护动作状态

	//以下为分段失灵保护
    Phasor IFD[6]={bus->busPhasor[15],bus->busPhasor[16],bus->busPhasor[17],bus->busPhasor[21],bus->busPhasor[22],bus->busPhasor[23]};
    double IN=1.0;//电流额定值

   if( (BusTripFlag[0] || bus->busTieOverCurrentFlag[0]  || bus->busTieOverCurrentFlag[2]  || bus->busTieOverCurrentFlag[4] || BusTie_ThrPhaFailueOpen ))
   {a=1;}
   if( phasorAbs(IBC[0])>BusFailuCurSetValue || phasorAbs(IBC[1])>BusFailuCurSetValue || phasorAbs(IBC[2])>BusFailuCurSetValue )//电流幅值大于定值
   {b=1;}

   if(BusvotBlockOpen[0]==0 && a && b) //判断是1
    { if(bus->busTieFailueStartTime[0]==0) //且起始时间是0
	  {bus->busTieFailueStartTime[0]=bus->relayTime;} //则判据满足，把当前时间给起始时间

   if( bus->busTieFailueStartTime[0]!=0 && (bus->relayTime - bus->busTieFailueStartTime[0] >= BusFailuTimeSetValue) ) //当起始时间不是零，且到达延时时间，则动作写Flag
       { 
            bus->busTieFailureFlag[0]= 1;
			bus->busTieFailueStartTime[0]=0;//动作之后还要清一次
            busWriteLog(bus, "母联I失灵跳I母");
        }
    }
    else{bus->busTieFailueStartTime[0]=0;} //判据不满足时，把保护起始时间清掉


  if(BusvotBlockOpen[1]==0 && a && b)
    { if(bus->busTieFailueStartTime[1]==0)
	  {bus->busTieFailueStartTime[1]=bus->relayTime;}

   if( bus->busTieFailueStartTime[1]!=0 && (bus->relayTime - bus->busTieFailueStartTime[1] >= BusFailuTimeSetValue) )
       { 
            bus->busTieFailureFlag[1]= 1;
			bus->busTieFailueStartTime[1]=0;//动作之后还要清一次
            busWriteLog(bus, "母联I失灵跳II母");
        }
    }
    else{bus->busTieFailueStartTime[1]=0;}

   //以上为母联断路器1的失灵保护，母联断路器2的失灵保护如下
   
   if( (BusTripFlag[1] || bus->busTieOverCurrentFlag[1]  || bus->busTieOverCurrentFlag[3]  || bus->busTieOverCurrentFlag[5] || BusTie_ThrPhaFailueOpen ))
   {c=1;}
   if( phasorAbs(IBC[3])>BusFailuCurSetValue || phasorAbs(IBC[4])>BusFailuCurSetValue || phasorAbs(IBC[5])>BusFailuCurSetValue )
   {d=1;}

   if(BusvotBlockOpen[2]==0 && c && d)
    { if(bus->busTieFailueStartTime[2]==0)
	  {bus->busTieFailueStartTime[2]=bus->relayTime;}

   if( bus->busTieFailueStartTime[2]!=0 && (bus->relayTime - bus->busTieFailueStartTime[2] >= BusFailuTimeSetValue) )
       { 
            bus->busTieFailureFlag[2]= 1;
			bus->busTieFailueStartTime[2]=0;//动作之后还要清一次
            busWriteLog(bus, "母联II失灵跳I母");
        }
    }
   else{bus->busTieFailueStartTime[2]=0;}

   if(BusvotBlockOpen[3]==0 && c && d)
    { if(bus->busTieFailueStartTime[3]==0)
	  {bus->busTieFailueStartTime[3]=bus->relayTime;}
    

   if( bus->busTieFailueStartTime[3]!=0 && (bus->relayTime - bus->busTieFailueStartTime[3] >= BusFailuTimeSetValue) )
       { 
            bus->busTieFailureFlag[3]= 1;
			bus->busTieFailueStartTime[3]=0;//动作之后还要清一次
            busWriteLog(bus, "母联II失灵跳II母");
        }
     }
   else{bus->busTieFailueStartTime[3]=0;}


//分段失灵保护
   if((phasorAbs(IFD[0])>0.04*IN || phasorAbs(IFD[1])>0.04*IN || phasorAbs(IFD[2])>0.04*IN)&& ((BusTripFlag[2]) || bus->busSecOverCurrentFlag[0]|| bus->busSecOverCurrentFlag[2]|| bus->busSecOverCurrentFlag[4]))
   {
   bus->busSecFailureStartFlag[0]=1;
   busWriteLog(bus, "分段I失灵启动");
   }

   if((phasorAbs(IFD[0])>BusFailuCurSetValue||phasorAbs(IFD[1])>BusFailuCurSetValue||phasorAbs(IFD[2])>BusFailuCurSetValue) && bus->busSecFailureStartFlag[0] && (BusvotBlockOpen[0]==0 ))
   { if(bus->busSecFailueStartTime[0]==0)
	  {bus->busSecFailueStartTime[0]=bus->relayTime;}

   if( bus->busSecFailueStartTime[0]!=0 && (bus->relayTime - bus->busSecFailueStartTime[0] >= BusFailuTimeSetValue) )
       { 
            bus->busSecFailureFlag[0]= 1;
			bus->busSecFailueStartTime[0]=0;//动作之后还要清一次
            busWriteLog(bus, "分段I失灵跳I-1母");
        }
    }
   else{bus->busSecFailueStartTime[0]=0;}

   if((phasorAbs(IFD[0])>BusFailuCurSetValue||phasorAbs(IFD[1])>BusFailuCurSetValue||phasorAbs(IFD[2])>BusFailuCurSetValue) && bus->busSecFailureStartFlag[0] && (BusvotBlockOpen[2]==0 ))
   { if(bus->busSecFailueStartTime[1]==0)
	  {bus->busSecFailueStartTime[1]=bus->relayTime;}

   if( bus->busSecFailueStartTime[1]!=0 && (bus->relayTime - bus->busSecFailueStartTime[1] >= BusFailuTimeSetValue) )
       { 
            bus->busSecFailureFlag[1]= 1;
			bus->busSecFailueStartTime[1]=0;//动作之后还要清一次
            busWriteLog(bus, "分段I失灵跳I-2母");
        }
    }
   else{bus->busSecFailueStartTime[1]=0;}

   if((phasorAbs(IFD[3])>0.04*IN || phasorAbs(IFD[4])>0.04*IN || phasorAbs(IFD[5])>0.04*IN)&& ((BusTripFlag[3]) || bus->busSecOverCurrentFlag[1]|| bus->busSecOverCurrentFlag[3]|| bus->busSecOverCurrentFlag[5]))
   {
   bus->busSecFailureStartFlag[1]=1;
   busWriteLog(bus, "分段II失灵启动");
   }

   if((phasorAbs(IFD[3])>BusFailuCurSetValue||phasorAbs(IFD[4])>BusFailuCurSetValue||phasorAbs(IFD[5])>BusFailuCurSetValue) && bus->busSecFailureStartFlag[1] && (BusvotBlockOpen[1]==0))
   { if(bus->busSecFailueStartTime[2]==0)
	  {bus->busSecFailueStartTime[2]=bus->relayTime;}

   if( bus->busSecFailueStartTime[2]!=0 && (bus->relayTime - bus->busSecFailueStartTime[2] >= BusFailuTimeSetValue) )
       { 
            bus->busSecFailureFlag[2]= 1;
			bus->busSecFailueStartTime[2]=0;//动作之后还要清一次
            busWriteLog(bus, "分段II失灵跳II-1母");
        }
    }
   else{bus->busSecFailueStartTime[2]=0;}

   if((phasorAbs(IFD[3])>BusFailuCurSetValue||phasorAbs(IFD[4])>BusFailuCurSetValue||phasorAbs(IFD[5])>BusFailuCurSetValue) && bus->busSecFailureStartFlag[1] && (BusvotBlockOpen[3]==0))
   { if(bus->busSecFailueStartTime[3]==0)
	  {bus->busSecFailueStartTime[3]=bus->relayTime;}

   if( bus->busSecFailueStartTime[3]!=0 && (bus->relayTime - bus->busSecFailueStartTime[3] >= BusFailuTimeSetValue) )
       { 
            bus->busSecFailureFlag[3]= 1;
			bus->busSecFailueStartTime[3]=0;//动作之后还要清一次
            busWriteLog(bus, "分段II失灵跳II-2母");
        }
     }
   else{bus->busSecFailueStartTime[3]=0;}      
}