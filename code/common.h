// common

void globalInit();
void switchRelay(Device* device, double timeD, const double* port1, const double* port2);
void lineLinkSimulation(Device* device, Device* deviceN, char* deviceName, double time, int deviceEnable, double* port1, double* port2, double* tripSignal);

void busLinkSimulation(BusDevice* device, Device line[], Device lineRelay[], char* deviceName, double time, int deviceEnable, double* port1, double* port2,double* port3, double* port4, double* port5,double* port6, double* port7, double* port8, double* port9, double* port10, double* port11, double* port12, double* port13, double* port14, double* port15, double* tripSignal);
void busLinkSimulation_500kV(BusDevice* device, Device line[], Device lineRelay[], char* deviceName, double time, int deviceEnable);

void busStatusRenew(BusDevice* device, double time,double* port1, double* port2,double* port3, double* port4, double* port5,double* port6, double* port7, double* port8, double* port9, double* port10, double* port11);
void deviceInit(Device* device, char* deviceName, int deviceEnable, char type);

void busDeviceInit(BusDevice* device, char* deviceName, int deviceEnable, char type);

int Trim(char s[]);

int readConfiguration(Device* device, char elementType);

int busReadConfiguration(BusDevice* device, char elementType);
double findSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, Device* device);

double busFindSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, BusDevice* device);

void initSwitchQueueTime(Device* device);

char* strcat_self(char* dest, char* src);
int hexToDec(char hex);
int hexToBinMatrix(char* hex, BusDevice* bus, int busNum, char tag);


int upTo10A(Device* device);
int busUpTo10A(BusDevice* device);
int upTo5(Device* device);
int busUpTo5(BusDevice* device);



// 仿真采样
void sample(Device* device, double time, double* port1, double* port2);

void zeroSeqSample(Device* device, double* port1);

void sample2inst(Device* device); 

void dataFilter(Device* device);

void toPhasor(Device* device);

void busSample(BusDevice* device, double* port1, double* port2, double* port3, double* port4);

void busSample2inst(BusDevice* device);

void busDataFilter(BusDevice* device);
void toPhasorForBus(BusDevice* device);
void lowPassFilter(double* aft, double* bef);
void differenceFilter(double* after, double* before);
void inst2phasor(double* inst, int start, Phasor* phasor);
void halfWaveFourier(double* inst, int start, Phasor* phasor);

void inst2harmonic(double* inst, int start, Phasor* phasor, int k);

int phaseMatch(int phase);

double phasorAbs(Phasor p);
Phasor ampAngle2phasor(double amp, double angle);
double phasorAngle(Phasor p);
Phasor phasorNumMulti(double a, Phasor p);
Phasor phasorAdd(Phasor pa, Phasor pb);
Phasor phasorSub(Phasor pa, Phasor pb);
Phasor phasorMulti(Phasor pa, Phasor pb);
Phasor phasorDiv(Phasor pa, Phasor pb);
Phasor phasorContrarotate(Phasor p, double angle);
Phasor phasorSeq(Phasor pa, Phasor pb, Phasor pc, int seq);
double phasorAngleDiff(Phasor pa, Phasor pb);


void writeLog(Device* device, char* content);
void writeErrorLog(Device* device, char* content);
void writeLogWithPhase(Device* device, char* content, int phase);

void busWriteLog(BusDevice* device, char* content);
void busWriteErrorLog(BusDevice* device, char* content);
void busWriteLogWithPhase(BusDevice* device, char* content, int phase);

unsigned int SDBMHash(char *str, int arrLength);
int notYet(Device* device, char* str);
int busNotYet(BusDevice* device, char* str);

Phasor memoryPhasorValue(Device* device, Phasor* memoryPhasors);
int memoryIndex(Device* device);
int memoryIndexForBus(BusDevice* device);
double* findInstCurrentPtr(Device* device, int phase);

void busLineSynCheck(BusDevice* bus, Device line[]);
void busLineSynCheck_500kV(BusDevice* bus, Device line[]);
Phasor* findBusMemory(BusDevice* bus, int phase, int brkNo);
Phasor* findTie1MemoryPhasor(BusDevice* bus, int phase);
Phasor* findTie2MemoryPhasor(BusDevice* bus, int phase);
Phasor* findSec1MemoryPhasor(BusDevice* bus, int phase);
Phasor* findSec2MemoryPhasor(BusDevice* bus, int phase);
Phasor* findLineMemory(BusDevice* bus, Device line[], int num, int phase, int side);
Phasor* findLineRelayTimePhasor(int flag, Device* line, int index, int phase);
Phasor* findBusRelayTimePhasor(int flag, BusDevice* bus, int brkNum, int index, int phase);
Phasor* lineMemoryPhasor(BusDevice* bus, Device* device, Phasor* memoryPhasors);
Phasor* busMemoryPhasor(BusDevice* bus, Phasor* memoryPhasors);
void findLineConnectBus(BusDevice* bus, int lineNo, int* result);

void recordData(Device* device);
void finalOutput(Device *device);

double* lineInstSearchSideM(Device line[], int phase, int lineNum);
double* lineInstSearchSideN(Device line[], int phase, int lineNum);
double* busInstSearch(BusDevice* bus, int phase, int brkNum);
Phasor phasorForHarmonic(double* inst, int start, int k);


//变压器部分
void tranSwitchRelay(tranDevice* trandevice, double timeD, const double* tport1, const double* tport2);

void tranLinkSimulation(tranDevice* trandevice, char* deviceName, double time, int deviceEnable, double* tport1, double* tport2, double* tranTripSignal);

void tranDeviceInit(tranDevice* trandevice, char* deviceName, int deviceEnable, char type);

int tranReadConfiguration(tranDevice* trandevice, char elementType);

double tranFindSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, tranDevice* trandevice);

void tranInitSwitchQueueTime(tranDevice* trandevice);

int tranUpTo10A(tranDevice* trandevice);

int tranUpTo5(tranDevice* trandevice);

void tranSample(tranDevice* trandevice, double time, double* tport1, double* tport2);

void tranSample2inst(tranDevice* trandevice);

void tranDataFilter(tranDevice* trandevice);

void tranToPhasor(tranDevice* trandevice);

void tranWriteLog(tranDevice* trandevice, char* content);

void tranWriteErrorLog(tranDevice* trandevice, char* content);

void tranWriteLogWithPhase(tranDevice* trandevice, char* content, int phase);

int tranNotYet(tranDevice* trandevice, char* str);

Phasor tranMemoryPhasorValue(tranDevice* trandevice, Phasor* tranMemoryPhasors, int n);

void tranRecordData(tranDevice* trandevice);

void tranHalfPhasor(tranDevice* trandevice);

void tranHarmPhasor(tranDevice* trandevice);

void halfWaveInte(double* inst, int start, double* amp);

void tranHalfInte(tranDevice* trandevice);

// 测控装置
void meaWriteErrorLog(meaDevice* device, char* content);
void meaDeviceInit(meaDevice* device, char* deviceName, int deviceEnable);
void meaSample(meaDevice* device, double time, double* port1, double* port2, int* port_remote);
void meaWriteLog(meaDevice* device, char* content);
void meaWriteLogWithPhase(meaDevice* device, char* content, int phase);
int meaUpTo5(meaDevice* device);
int meaUpTo10A(meaDevice* device);
int meaNotYet(meaDevice* device, char* str);
double meaFindSetValueIndex(char* target, char (*paramName)[STRING_LENGTH], double* paramValue, int n, meaDevice* device);
int meaReadConfiguration(meaDevice* device);
void meaLinkSimulation(meaDevice* device, char* deviceName, double time, int deviceEnable, double* port1, double* port2, int* port_remote, int* tripSignals);
void meaSwitchRelay(meaDevice* device, double timeD, const double* port1, const double* port2);
void meaSample2Inst(meaDevice* device);
void meaDataFilter(meaDevice* device);
void meaToPhasor(meaDevice* device);

void meaTelemetry(meaDevice* device);
void meaRecord(meaDevice* device);
void meaTelecontrol(meaDevice* device);
int meaRecordCounter(meaDevice* device);

double rmsCalculate(double* inst, int startIdx);
double powerCalculate(double* voltA, double* currA, double* voltB, double* currB, double* voltC, double* currC, int startIdx, char type);
double powerFactorCalculate(double P, double Q);
double freqCalculate(double* inst, int pt, int startIdx);
double voltDiffCalculate(Phasor Ug, Phasor Ux);
double angleDiffCalculate(Phasor Ug, Phasor Ux);
double freqDiffCalculate(double fg, double fx);
double slipDiffCalculate(double fg, double fx);




