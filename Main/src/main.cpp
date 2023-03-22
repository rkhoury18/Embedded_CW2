#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "knobs.h"

// #define sender 0
// #define reciever 0

//#define DISABLE_THREADS
//#define TEST_SCAN_KEYS
//#define TEST_DISPLAY_UPDATE
#define SAMPLE_BUFFER_SIZE 128
//Variable neede for Double Buffer 
uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
volatile bool writeBuffer1 = false;
SemaphoreHandle_t sampleBufferMutex;

//Wave Selector code 

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[12] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};
  const char notes[12] = {'C','C','D','D','E','F','F','G','G','A','A','B'};
  const char sharps[12] = {' ','#',' ','#',' ',' ','#',' ','#',' ','#',' '};
  const uint32_t sineTable [1024] = {2147483647, 2160660358, 2173836574, 2187011798, 2200185533, 2213357285, 2226526556, 2239692851, 2252855675, 2266014531, 2279168925, 2292318361, 2305462344, 2318600379, 2331731972, 2344856628, 2357973852, 2371083153, 2384184034, 2397276004, 2410358570, 2423431238, 2436493517, 2449544915, 2462584941, 2475613103, 2488628912, 2501631876, 2514621507, 2527597315, 2540558813, 2553505511, 2566436923, 2579352561, 2592251940, 2605134574, 2617999977, 2630847665, 2643677156, 2656487964, 2669279609, 2682051609, 2694803483, 2707534750, 2720244932, 2732933549, 2745600125, 2758244182, 2770865244, 2783462836, 2796036484, 2808585714, 2821110054, 2833609033, 2846082179, 2858529024, 2870949098, 2883341933, 2895707065, 2908044026, 2920352352, 2932631580, 2944881248, 2957100895, 2969290059, 2981448284, 2993575110, 3005670081, 3017732741, 3029762638, 3041759317, 3053722327, 3065651218, 3077545540, 3089404846, 3101228689, 3113016624, 3124768208, 3136482997, 3148160551, 3159800430, 3171402196, 3182965412, 3194489643, 3205974454, 3217419414, 3228824091, 3240188057, 3251510883, 3262792142, 3274031411, 3285228267, 3296382286, 3307493051, 3318560141, 3329583142, 3340561637, 3351495213, 3362383459, 3373225964, 3384022321, 3394772123, 3405474966, 3416130446, 3426738161, 3437297714, 3447808706, 3458270741, 3468683426, 3479046369, 3489359179, 3499621468, 3509832850, 3519992940, 3530101356, 3540157718, 3550161645, 3560112763, 3570010696, 3579855072, 3589645520, 3599381671, 3609063159, 3618689620, 3628260690, 3637776010, 3647235222, 3656637968, 3665983896, 3675272653, 3684503889, 3693677258, 3702792414, 3711849012, 3720846714, 3729785179, 3738664071, 3747483057, 3756241803, 3764939981, 3773577262, 3782153321, 3790667837, 3799120487, 3807510954, 3815838922, 3824104077, 3832306109, 3840444708, 3848519568, 3856530385, 3864476857, 3872358686, 3880175573, 3887927226, 3895613353, 3903233663, 3910787870, 3918275690, 3925696840, 3933051042, 3940338018, 3947557494, 3954709199, 3961792862, 3968808218, 3975755001, 3982632952, 3989441810, 3996181319, 4002851226, 4009451280, 4015981231, 4022440835, 4028829847, 4035148028, 4041395140, 4047570946, 4053675216, 4059707718, 4065668226, 4071556516, 4077372366, 4083115556, 4088785871, 4094383096, 4099907023, 4105357441, 4110734147, 4116036937, 4121265613, 4126419976, 4131499834, 4136504995, 4141435270, 4146290475, 4151070425, 4155774941, 4160403846, 4164956966, 4169434129, 4173835167, 4178159914, 4182408207, 4186579887, 4190674795, 4194692779, 4198633686, 4202497369, 4206283681, 4209992481, 4213623629, 4217176987, 4220652423, 4224049805, 4227369006, 4230609900, 4233772365, 4236856283, 4239861538, 4242788015, 4245635606, 4248404202, 4251093699, 4253703997, 4256234997, 4258686604, 4261058725, 4263351271, 4265564156, 4267697297, 4269750612, 4271724026, 4273617463, 4275430852, 4277164125, 4278817217, 4280390065, 4281882611, 4283294798, 4284626573, 4285877885, 4287048688, 4288138938, 4289148594, 4290077616, 4290925972, 4291693628, 4292380555, 4292986729, 4293512125, 4293956725, 4294320512, 4294603471, 4294805592, 4294926868, 4294967294, 4294926868, 4294805592, 4294603471, 4294320512, 4293956725, 4293512125, 4292986729, 4292380555, 4291693628, 4290925972, 4290077616, 4289148594, 4288138938, 4287048688, 4285877885, 4284626573, 4283294798, 4281882611, 4280390065, 4278817217, 4277164125, 4275430852, 4273617463, 4271724026, 4269750612, 4267697297, 4265564156, 4263351271, 4261058725, 4258686604, 4256234997, 4253703997, 4251093699, 4248404202, 4245635606, 4242788015, 4239861538, 4236856283, 4233772365, 4230609900, 4227369006, 4224049805, 4220652423, 4217176987, 4213623629, 4209992481, 4206283681, 4202497369, 4198633686, 4194692779, 4190674795, 4186579887, 4182408207, 4178159914, 4173835167, 4169434129, 4164956966, 4160403846, 4155774941, 4151070425, 4146290475, 4141435270, 4136504995, 4131499834, 4126419976, 4121265613, 4116036937, 4110734147, 4105357441, 4099907023, 4094383096, 4088785871, 4083115556, 4077372366, 4071556516, 4065668226, 4059707718, 4053675216, 4047570946, 4041395140, 4035148028, 4028829847, 4022440835, 4015981231, 4009451280, 4002851226, 3996181319, 3989441810, 3982632952, 3975755001, 3968808218, 3961792862, 3954709199, 3947557494, 3940338018, 3933051042, 3925696840, 3918275690, 3910787870, 3903233663, 3895613353, 3887927226, 3880175573, 3872358686, 3864476857, 3856530385, 3848519568, 3840444708, 3832306109, 3824104077, 3815838922, 3807510954, 3799120487, 3790667837, 3782153321, 3773577262, 3764939981, 3756241803, 3747483057, 3738664071, 3729785179, 3720846714, 3711849012, 3702792414, 3693677258, 3684503889, 3675272653, 3665983896, 3656637968, 3647235222, 3637776010, 3628260690, 3618689620, 3609063159, 3599381671, 3589645520, 3579855072, 3570010696, 3560112763, 3550161645, 3540157718, 3530101356, 3519992940, 3509832850, 3499621468, 3489359179, 3479046369, 3468683426, 3458270741, 3447808706, 3437297714, 3426738161, 3416130446, 3405474966, 3394772123, 3384022321, 3373225964, 3362383459, 3351495213, 3340561637, 3329583142, 3318560141, 3307493051, 3296382286, 3285228267, 3274031411, 3262792142, 3251510883, 3240188057, 3228824091, 3217419414, 3205974454, 3194489643, 3182965412, 3171402196, 3159800430, 3148160551, 3136482997, 3124768208, 3113016624, 3101228689, 3089404846, 3077545540, 3065651218, 3053722327, 3041759317, 3029762638, 3017732741, 3005670081, 2993575110, 2981448284, 2969290059, 2957100895, 2944881248, 2932631580, 2920352352, 2908044026, 2895707065, 2883341933, 2870949098, 2858529024, 2846082179, 2833609033, 2821110054, 2808585714, 2796036484, 2783462836, 2770865244, 2758244182, 2745600125, 2732933549, 2720244932, 2707534750, 2694803483, 2682051609, 2669279609, 2656487964, 2643677156, 2630847665, 2617999977, 2605134574, 2592251940, 2579352561, 2566436923, 2553505511, 2540558813, 2527597315, 2514621507, 2501631876, 2488628912, 2475613103, 2462584941, 2449544915, 2436493517, 2423431238, 2410358570, 2397276004, 2384184034, 2371083153, 2357973852, 2344856628, 2331731972, 2318600379, 2305462344, 2292318361, 2279168925, 2266014531, 2252855675, 2239692851, 2226526556, 2213357285, 2200185533, 2187011798, 2173836574, 2160660358, 2147483647, 2134306936, 2121130720, 2107955496, 2094781761, 2081610009, 2068440738, 2055274443, 2042111619, 2028952763, 2015798369, 2002648933, 1989504950, 1976366915, 1963235322, 1950110666, 1936993442, 1923884141, 1910783260, 1897691290, 1884608724, 1871536056, 1858473777, 1845422379, 1832382353, 1819354191, 1806338382, 1793335418, 1780345787, 1767369979, 1754408481, 1741461783, 1728530371, 1715614733, 1702715354, 1689832720, 1676967317, 1664119629, 1651290138, 1638479330, 1625687685, 1612915685, 1600163811, 1587432544, 1574722362, 1562033745, 1549367169, 1536723112, 1524102050, 1511504458, 1498930810, 1486381580, 1473857240, 1461358261, 1448885115, 1436438270, 1424018196, 1411625361, 1399260229, 1386923268, 1374614942, 1362335714, 1350086046, 1337866399, 1325677235, 1313519010, 1301392184, 1289297213, 1277234553, 1265204656, 1253207977, 1241244967, 1229316076, 1217421754, 1205562448, 1193738605, 1181950670, 1170199086, 1158484297, 1146806743, 1135166864, 1123565098, 1112001882, 1100477651, 1088992840, 1077547880, 1066143203, 1054779237, 1043456411, 1032175152, 1020935883, 1009739027, 998585008, 987474243, 976407153, 965384152, 954405657, 943472081, 932583835, 921741330, 910944973, 900195171, 889492328, 878836848, 868229133, 857669580, 847158588, 836696553, 826283868, 815920925, 805608115, 795345826, 785134444, 774974354, 764865938, 754809576, 744805649, 734854531, 724956598, 715112222, 705321774, 695585623, 685904135, 676277674, 666706604, 657191284, 647732072, 638329326, 628983398, 619694641, 610463405, 601290036, 592174880, 583118282, 574120580, 565182115, 556303223, 547484237, 538725491, 530027313, 521390032, 512813973, 504299457, 495846807, 487456340, 479128372, 470863217, 462661185, 454522586, 446447726, 438436909, 430490437, 422608608, 414791721, 407040068, 399353941, 391733631, 384179424, 376691604, 369270454, 361916252, 354629276, 347409800, 340258095, 333174432, 326159076, 319212293, 312334342, 305525484, 298785975, 292116068, 285516014, 278986063, 272526459, 266137447, 259819266, 253572154, 247396348, 241292078, 235259576, 229299068, 223410778, 217594928, 211851738, 206181423, 200584198, 195060271, 189609853, 184233147, 178930357, 173701681, 168547318, 163467460, 158462299, 153532024, 148676819, 143896869, 139192353, 134563448, 130010328, 125533165, 121132127, 116807380, 112559087, 108387407, 104292499, 100274515, 96333608, 92469925, 88683613, 84974813, 81343665, 77790307, 74314871, 70917489, 67598288, 64357394, 61194929, 58111011, 55105756, 52179279, 49331688, 46563092, 43873595, 41263297, 38732297, 36280690, 33908569, 31616023, 29403138, 27269997, 25216682, 23243268, 21349831, 19536442, 17803169, 16150077, 14577229, 13084683, 11672496, 10340721, 9089409, 7918606, 6828356, 5818700, 4889678, 4041322, 3273666, 2586739, 1980565, 1455169, 1010569, 646782, 363823, 161702, 40426, 0, 40426, 161702, 363823, 646782, 1010569, 1455169, 1980565, 2586739, 3273666, 4041322, 4889678, 5818700, 6828356, 7918606, 9089409, 10340721, 11672496, 13084683, 14577229, 16150077, 17803169, 19536442, 21349831, 23243268, 25216682, 27269997, 29403138, 31616023, 33908569, 36280690, 38732297, 41263297, 43873595, 46563092, 49331688, 52179279, 55105756, 58111011, 61194929, 64357394, 67598288, 70917489, 74314871, 77790307, 81343665, 84974813, 88683613, 92469925, 96333608, 100274515, 104292499, 108387407, 112559087, 116807380, 121132127, 125533165, 130010328, 134563448, 139192353, 143896869, 148676819, 153532024, 158462299, 163467460, 168547318, 173701681, 178930357, 184233147, 189609853, 195060271, 200584198, 206181423, 211851738, 217594928, 223410778, 229299068, 235259576, 241292078, 247396348, 253572154, 259819266, 266137447, 272526459, 278986063, 285516014, 292116068, 298785975, 305525484, 312334342, 319212293, 326159076, 333174432, 340258095, 347409800, 354629276, 361916252, 369270454, 376691604, 384179424, 391733631, 399353941, 407040068, 414791721, 422608608, 430490437, 438436909, 446447726, 454522586, 462661185, 470863217, 479128372, 487456340, 495846807, 504299457, 512813973, 521390032, 530027313, 538725491, 547484237, 556303223, 565182115, 574120580, 583118282, 592174880, 601290036, 610463405, 619694641, 628983398, 638329326, 647732072, 657191284, 666706604, 676277674, 685904135, 695585623, 705321774, 715112222, 724956598, 734854531, 744805649, 754809576, 764865938, 774974354, 785134444, 795345826, 805608115, 815920925, 826283868, 836696553, 847158588, 857669580, 868229133, 878836848, 889492328, 900195171, 910944973, 921741330, 932583835, 943472081, 954405657, 965384152, 976407153, 987474243, 998585008, 1009739027, 1020935883, 1032175152, 1043456411, 1054779237, 1066143203, 1077547880, 1088992840, 1100477651, 1112001882, 1123565098, 1135166864, 1146806743, 1158484297, 1170199086, 1181950670, 1193738605, 1205562448, 1217421754, 1229316076, 1241244967, 1253207977, 1265204656, 1277234553, 1289297213, 1301392184, 1313519010, 1325677235, 1337866399, 1350086046, 1362335714, 1374614942, 1386923268, 1399260229, 1411625361, 1424018196, 1436438270, 1448885115, 1461358261, 1473857240, 1486381580, 1498930810, 1511504458, 1524102050, 1536723112, 1549367169, 1562033745, 1574722362, 1587432544, 1600163811, 1612915685, 1625687685, 1638479330, 1651290138, 1664119629, 1676967317, 1689832720, 1702715354, 1715614733, 1728530371, 1741461783, 1754408481, 1767369979, 1780345787, 1793335418, 1806338382, 1819354191, 1832382353, 1845422379, 1858473777, 1871536056, 1884608724, 1897691290, 1910783260, 1923884141, 1936993442, 1950110666, 1963235322, 1976366915, 1989504950, 2002648933, 2015798369, 2028952763, 2042111619, 2055274443, 2068440738, 2081610009, 2094781761, 2107955496, 2121130720, 2134306936};

//global variables  

  //Current note being printed
  volatile char currentnote;
  volatile char currentsharp;

  volatile uint8_t maxOct = 8;


  volatile uint32_t pressedKeysMin = 0; // 12 bits for first pos 12 bits for second pos, 00000000
  volatile uint32_t pressedKeysMaj = 0; // 12 bits for third pos, 0000
  SemaphoreHandle_t pressedKeysArrayMutex;

  //Keyboard connection values
  volatile uint8_t west_detect;
  volatile uint8_t east_detect;
  
  //Needed to know the position of the keytboard in the chain
  volatile uint8_t pos = 0;

  // Role of keyboard  in chain
  volatile bool receiver = false;
  volatile bool sender = false;
  volatile bool singleton = true;

  //Needed for communication between keyboards
  uint8_t RX_Message[8]={0};
  SemaphoreHandle_t CAN_TX_Semaphore;
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;

  //Sender variables
  volatile uint8_t octave_s;
  volatile uint8_t volume_s;
  volatile uint8_t A_s = 1;
  volatile uint8_t D_s = 1;
  volatile uint8_t S_s = 1;
  volatile uint8_t R_s = 1;
  volatile uint8_t wave_s;
  volatile int32_t vout_s = 0;
  volatile uint32_t maj_s = 0;
  volatile uint32_t min_s = 0;


  //Receiver variables
  volatile uint8_t volume_r = 4;
  volatile uint8_t octave_r = 4;
  volatile uint8_t wave_r = 0;
  volatile int32_t vout_r = 0;
  volatile uint8_t A_r = 1;
  volatile uint8_t D_r = 1;
  volatile uint8_t S_r = 1;
  volatile uint8_t R_r = 1;


  volatile uint8_t keyArray[7];
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t tvarMutex_s;
  SemaphoreHandle_t tvarMutex_r;

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;


//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

uint8_t readCols(){
  uint8_t c0 = digitalRead(C0_PIN);
  uint8_t c1 = digitalRead(C1_PIN);
  uint8_t c2 = digitalRead(C2_PIN);
  uint8_t c3 = digitalRead(C3_PIN);
  uint8_t result = c0 | (c1 << 1) | (c2 << 2) | (c3 << 3);
  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

//Needed for ADSR
volatile uint32_t t_r[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile uint32_t t_s[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint32_t ADSR(uint8_t i, uint8_t A, uint8_t D, uint8_t S, uint8_t R){ 
  if (receiver){
      //xSemaphoreTake(tvarMutex_r, portMAX_DELAY);
        // // Serial.printf("time instance: %d\n", t_r[i]);
        static uint32_t scale_r[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        if (t_r[i] == 0){
            scale_r[i] = 0;
        }
            
        else if (t_r[i] < ((A*9 * 22000) >> 8)){ //Attack envelope
          // Serial.printf("Attacking\n");
            scale_r[i] += 84*(256/(A));
        }
            
        else if (t_r[i] == ((A*9 * 22000) >> 8)){ //Attack peak
            scale_r[i] = 16777215;
        }
            
        else if (t_r[i] < (((A*9 + D*76)*22000) >> 8)){ //Decay envelope
          // Serial.printf("Decaying\n");

            scale_r[i] =- (16777216 - (204 << 16))/(22000*76) * (256/(D_s));
        }
            
        else if (t_r[i] < (((A*9 + D*76 + S*46)*22000) >> 8)){ //Sustain envelope
          // Serial.printf("Sustain\n");

            scale_r[i] = 204 << 16;
        }
            
        else if (t_r[i] < (((A*9 + D*76 + S*46 + R*60)*22000) >> 8)){
          // Serial.printf("Releasing\n");

          scale_r[i] -= (204 << 16)/(22000*60) * (256/(R));
        } //Release envelope
        else{
          // Serial.printf("Out of Rang\n");
          scale_r[i] = 0;
        }
          
        t_r[i]++;
        // scale_r[i] += 200;
        //xSemaphoreGive(tvarMutex_r);
        return scale_r[i];
  }
  if (sender){
    static uint32_t scale_s[36] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        if (t_s[i] == 0)
            scale_s[i] = 0;
            
        else if (t_s[i] < ((A*9 * 22000) >> 8)) //Attack envelope
            scale_s[i] += 84*(256/(A));
            
        else if (t_s[i] == ((A*9 * 22000) >> 8)) //Attack peak
            scale_s[i] = 16777215;
            
        else if (t_s[i] < (((A*9 + D*76)*22000) >> 8)) //Decay envelope
            scale_s[i] =- (16777216 - (204 << 16))/(22000*76) * (256/(D_s));
            
        else if (t_s[i] < (((A*9 + D*76 + S*46)*22000) >> 8)) //Sustain envelope
            scale_s[i] = 204 << 16;
            
        else if (t_s[i] < (((A*9 + D*76 + S*46 + R*60)*22000) >> 8)) //Release envelope
            scale_s[i] -= (204 << 16)/(22000*60) * (256/(R));
            
        else
            scale_s[i] = 0;
        xSemaphoreTake(tvarMutex_s, portMAX_DELAY);
          t_s[i]++;
        xSemaphoreGive(tvarMutex_s);
        return scale_s[i];

  }

}

void auto_detect(bool west, bool east){
  if(!west){ //most west module
    pos = 0;
    receiver = true;
    sender = false;
    if(east){ // >=2 modules
      singleton = false;
      uint8_t TX_Message[8] = {'H',pos,octave_r,volume_r,wave_r,0,0,0}; //Handshake, position, 0, 0, 0, 0, 0, 0
      xQueueSend(msgOutQ, TX_Message, 0);
    }
    else{
      maxOct=8;
      singleton = true;
    }
  }
  else{
    sender = true; //either east or middle 2/3 modules
    receiver = false;
    singleton = false;
  }
}

void sampleISR() {
  static uint32_t readCtr = 0;
    if (readCtr == SAMPLE_BUFFER_SIZE) {
      readCtr = 0;
      writeBuffer1 = !writeBuffer1;
      xSemaphoreGiveFromISR(sampleBufferMutex, NULL);
      }
	
    if (writeBuffer1)
      analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
    else
      analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
}

void ISRTask(void *pvParameters) {
  uint64_t pressedKeysArray = 0;
  uint8_t baseoct;
  uint8_t vol;
  uint8_t wave;
  uint8_t A;
  uint8_t D;
  uint8_t S;
  uint8_t R;
  while (1) {
    xSemaphoreTake(sampleBufferMutex, portMAX_DELAY);
      for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
        if(receiver){
          pressedKeysArray = ((((uint64_t)pressedKeysMaj) << 24) | pressedKeysMin);
          baseoct = octave_r - 4;
          vol = volume_r;
          wave = wave_r;
          A = A_r;
          D = D_r;
          S = S_r;
          R = R_r;
        }
        else if(sender){
          pressedKeysArray = ((uint64_t)min_s) << 24 | maj_s;
          baseoct = octave_s - pos - 4;
          vol = volume_s;
          wave = wave_s;
          A = A_s;
          D = D_s;
          S = S_s;
          R = R_s;
        }
          static uint32_t phaseAcc[36] = {0};
          static int increase[36] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
          int32_t polyphony_vout = 0;
          switch (wave){
              case 0 :  //Sawtooth
                for (uint8_t i=0; i<36;i++){
                  int8_t shift = baseoct + (uint8_t)(i/12);
                  uint32_t step = shift>0 ? stepSizes[i%12]<<shift : stepSizes[i%12]>>-shift;
                  if((pressedKeysArray >> i) & 0x1){
                    phaseAcc[i] += step;
                    int32_t Vout = (phaseAcc[i] >> 24) - 128;
                    polyphony_vout += (Vout*ADSR(i,A,D,S,R) >> 24);
                  }
                }
                break;
              case 1 : //Triangle
                for (uint8_t i=0; i<36;i++){
                  int8_t shift = baseoct + (uint8_t)(i/12);
                  uint32_t step = shift>0 ? stepSizes[i%12]<<shift : stepSizes[i%12]>>-shift;
                  if((pressedKeysArray >> i) & 0x1){
                    if (increase[i] == 1) {
                      if (phaseAcc[i] + 2*step >= phaseAcc[i]){
                        phaseAcc[i] += 2*step;
                      }
                      else{
                        increase[i] = -1;
                      }
                    }
                    else {
                      if ((phaseAcc[i] - 2*step) <= phaseAcc[i]){
                        phaseAcc[i] -= 2*step;
                      }
                      else{
                        increase[i] = 1;
                      } 
                    }
                    int32_t Vout = ((phaseAcc[i] >> 24));
                    polyphony_vout += ((uint64_t(Vout*ADSR(i,A,D,S,R))) >> 24) - 128;
                    
                  }
                }
                break;
              case 2 : //Square
                for (uint8_t i=0; i<36;i++){
                  int8_t shift = baseoct + (uint8_t)(i/12);
                  uint32_t step = shift>0 ? stepSizes[i%12]<<shift : stepSizes[i%12]>>-shift;
                  if((pressedKeysArray >> i) & 0x1){
                    if (increase[i] == 1) {
                      if ((phaseAcc[i] + 2*step) >= phaseAcc[i]){
                        phaseAcc[i] += 2*step;
                      }
                      else{
                        increase[i] = -1;
                      }
                    }
                    else {
                      if ((phaseAcc[i] - 2*step) <= phaseAcc[i]){
                        phaseAcc[i] -= 2*step;
                      }
                      else{
                        increase[i] = 1;
                      }
                    }
                    int32_t Vout = (increase[i] == 1) ? 255 : 0;
                    polyphony_vout += ((uint64_t(Vout*ADSR(i,A,D,S,R))) >> 24) - 128;
                  }
                }
                break;
              case 3 : //Sine
                for (uint8_t i=0; i<36;i++){
                  int8_t shift = baseoct + (uint8_t)(i/12);
                  uint32_t step = shift>0 ? stepSizes[i%12]<<shift : stepSizes[i%12]>>-shift;
                  if((pressedKeysArray >> i) & 0x1){
                    phaseAcc[i] += step;
                    int32_t Vout = (sineTable[phaseAcc[i] >> 22] >> 24);
                    polyphony_vout += ((uint64_t(Vout*ADSR(i,A,D,S,R))) >> 24) - 128;
                  }
                }
                break;
          }
          polyphony_vout = polyphony_vout >> (8 - vol);
          polyphony_vout = max(-128, min(127, (int)polyphony_vout));
            uint32_t lol = polyphony_vout;//Calculate one sample
            if (writeBuffer1)
              sampleBuffer1[writeCtr] = lol + 128;
            else
              sampleBuffer0[writeCtr] = lol + 128;
            
            vout_r = polyphony_vout;
      }
  }
}

void recieverTask(){
  uint32_t localpressedKeysMin = pressedKeysMin;
  uint16_t localpressedKeysMaj = pressedKeysMaj;
  if (RX_Message[0] == 'R'){
    if(RX_Message[3] == 1){
      localpressedKeysMin &= ~(1 << (12+RX_Message[2]));
      xSemaphoreTake(tvarMutex_r, portMAX_DELAY);
        t_r[RX_Message[2]+ 12] = 0;
      xSemaphoreGive(tvarMutex_r);
    }
    else if(RX_Message[3] == 2){
      localpressedKeysMaj &= ~(1 << RX_Message[2]);
      xSemaphoreTake(tvarMutex_r, portMAX_DELAY);
        t_r[RX_Message[2]+ 24] = 0;
      xSemaphoreGive(tvarMutex_r);
    }
  }
  //If key is pressed set to correct step size
  else if (RX_Message[0] == 'P'){
    if(RX_Message[3] == 1){
      localpressedKeysMin |= (1 << (12+RX_Message[2]));
    }
    else if(RX_Message[3] == 2){
      localpressedKeysMaj |= (1 << RX_Message[2]);
    }
  }
  else if (RX_Message[0] == 'M'){
    uint8_t localmaxOct = 8-RX_Message[1];
    __atomic_store_n(&maxOct, localmaxOct,__ATOMIC_RELAXED);

  }
  else if (RX_Message[0] == 'A'){
    uint8_t localR = RX_Message[1];
    uint8_t localS = RX_Message[2];
    uint8_t localD = RX_Message[3];
    uint8_t localA = RX_Message[4];
    __atomic_store_n(&A_r, localA,__ATOMIC_RELAXED);
    __atomic_store_n(&D_r, localD,__ATOMIC_RELAXED);
    __atomic_store_n(&S_r, localS,__ATOMIC_RELAXED);
    __atomic_store_n(&R_r, localR,__ATOMIC_RELAXED);
  }
  // Serial.printf("keys recieved %hu\n", localpressedKeysArrayMaj);
  //Write step size to the pressedKeys array to play it
  __atomic_store_n(&pressedKeysMin, localpressedKeysMin,__ATOMIC_RELAXED);
  __atomic_store_n(&pressedKeysMaj, localpressedKeysMaj,__ATOMIC_RELAXED);
}

void senderTask(){
//Message received is handshake 
  if (RX_Message[0] == 'H'){
    pos = RX_Message[1] + 1;
    octave_s = RX_Message[2] + 1;
    volume_s = RX_Message[3];

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      for (uint8_t i=5; i<7; i++) {
        setRow(i);
        digitalWrite(OUTR_PIN,1);         //Enable selected row
        digitalWrite(REN_PIN,1);          //Enable selected row
        delayMicroseconds(3);             //Wait for column inputs to stabilise
        keyArray[i] = readCols();         //Read column inputs
        digitalWrite(REN_PIN,0);          //Disable selected row
      }
      uint8_t west_detect = ((keyArray[5]&0x08)>>3)^0x01;
      uint8_t east_detect = ((keyArray[6]&0x08)>>3)^0x01;
    xSemaphoreGive(keyArrayMutex);

    //Send to other keyboards connected to get their position
    if(east_detect){
      uint8_t TX_Message[8] = {'H',pos, octave_s, volume_r, 0, 0, 0, 0};
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
    else{
      uint8_t TX_Message[8] = {'M',pos, 0, 0, 0, 0, 0, 0};
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
  }
  //Info from the knobs
  else if (RX_Message[0] == 'K'){
    octave_s = RX_Message[2] + pos;
    volume_s = RX_Message[3];
    wave_s = RX_Message[4];
  }
  //Voltage message to know what notes to play
  else if (RX_Message[0] == 'V'){
    uint32_t localmaj = RX_Message[3]<<16 | RX_Message[2]<<8 | RX_Message[1];
    uint16_t localmin = RX_Message[5]<<8 | RX_Message[4];
    __atomic_store_n(&maj_s, localmaj, __ATOMIC_RELAXED);
    __atomic_store_n(&min_s, localmin, __ATOMIC_RELAXED);

    uint64_t released = ~((uint64_t)localmin << 24 | localmaj);
    for(uint8_t i=0; i<36; i++){
      if(released & (1<<i)){
        xSemaphoreTake(tvarMutex_s, portMAX_DELAY);
          t_s[i] = 0;
        xSemaphoreGive(tvarMutex_s);
      }
    }
  }

}


void decodeTask(void * pvParameters){
  while(1){
  xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    if(receiver){
      recieverTask();
    }
    else if(sender){
      senderTask();
    }
  }
}


void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
	}
}


void sendSoundTask (void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  uint64_t localPressedKeys;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  volatile uint64_t prevPressed = 0;
  while(1){
    localPressedKeys = ((uint64_t)pressedKeysMaj) << 24 | pressedKeysMin;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    if(receiver && !singleton && localPressedKeys != prevPressed){
      uint8_t TX_Message[8] = {'V', (pressedKeysMin & 0xFF), (pressedKeysMin>>8 & 0xFF), (pressedKeysMin>>16 & 0xFF), (pressedKeysMaj & 0xFF), (pressedKeysMaj>>8 & 0xFF), 0, 0};
      // Serial.println("Sending:"+String(pressedKeysMaj));
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      // Serial.printf("Sending: %d\n", TX_Message[4]<<24 | TX_Message[3]<<16 | TX_Message[2]<<8 | TX_Message[1]);
    }
    else {
    }
    prevPressed = localPressedKeys;

  }
}

void scanKeysTask(void * pvParameters) {
   #ifndef TEST_SCAN_KEYS
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Classes initialisations and variables needed for task
    Knob Knob3(0,4);
    Knob Knob2(0,4);
    Knob Knob1 (0,0);
    Knob KnobA (0,1);
    Knob KnobD (0,1);
    Knob KnobS (0,1);
    Knob KnobR (0,1);
    uint16_t prevPressedKeys = 0;
    uint16_t pressedKeys = 0;
    uint8_t TX_Message[8] = {0};
    uint32_t reset[36] = {0};
    uint8_t prevOctave=4 ;
    uint8_t prevVolume=4;
    uint8_t prevWave = 0;
    uint8_t prevA = 0;
    uint8_t prevD = 0;
    uint8_t prevS = 0;
    uint8_t prevR = 0;
    bool start = true;
    uint8_t prevMaxOct = 0;

    while(1){
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        uint32_t localstepsize = 0;
        char localnote = currentnote;
        char localsharp = currentsharp;
        uint8_t localvolume_r = volume_r;
        uint8_t localoctave_r= octave_r;
        uint8_t localwave_r = wave_r;
        uint8_t localA_s = A_s;
        uint8_t localD_s = D_s;
        uint8_t localS_s = S_s;
        uint8_t localR_s = R_s;
        uint32_t localpressedKeysMin = pressedKeysMin;
        uint32_t localpressedKeysMaj = pressedKeysMaj;
        uint8_t localkeyArray[7] = {0};
        //Get keys info
        for (uint8_t i = 0; i < 7; i++){
          setRow(i);
          delayMicroseconds(3);
          uint8_t val = readCols();
          localkeyArray[i] = val;
        }

        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          memcpy((void*)keyArray, localkeyArray, sizeof(localkeyArray));
        xSemaphoreGive(keyArrayMutex);

       //Initializing our local variables
        pressedKeys = localkeyArray[0] | (localkeyArray[1]<<4) | (localkeyArray[2]<<8);
        uint8_t localwest_detect = ((localkeyArray[5]&0x08)>>3)^0x01;
        uint8_t localeast_detect = ((localkeyArray[6]&0x08)>>3)^0x01;

        //If the keyboards didn't get power at the same time run autodetect again to make sure we get a correct outcome
        if (start){
          auto_detect(localwest_detect, localeast_detect);
          start = false;
        }

        //If a keyboard is connected or disconnected run autodetect again
        if (localwest_detect != west_detect){ //west detect has changed
            if(receiver){
                localpressedKeysMin = 0;
                __atomic_store_n(&pressedKeysMin, localpressedKeysMin, __ATOMIC_RELAXED);
                __atomic_store_n(&pressedKeysMaj, 0x0000, __ATOMIC_RELAXED);
            }
            auto_detect(localwest_detect, localeast_detect);
            __atomic_store_n(&west_detect, localwest_detect, __ATOMIC_RELAXED);
        }
        if (localeast_detect != east_detect){
          if(receiver){
            localpressedKeysMin= 0;
            __atomic_store_n(&pressedKeysMin, localpressedKeysMin, __ATOMIC_RELAXED);
            __atomic_store_n(&pressedKeysMaj, 0x0000, __ATOMIC_RELAXED);
          }
          auto_detect(localwest_detect, localeast_detect);
          if(sender){
            uint8_t TX_Message[8] = {'H',pos, octave_s, volume_s, 0, 0, 0, 0};
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          }
          __atomic_store_n(&east_detect, localeast_detect, __ATOMIC_RELAXED);
        }
        if (prevMaxOct != maxOct){
              // Serial.println ("Setting max"+String(maxOct));
              Knob3.SetLimits(0,8);
              Knob2.SetLimits(0,maxOct);
              Knob1.SetLimits(0,3);
              KnobA.SetLimits(0,120);
              KnobD.SetLimits(0,120);
              KnobS.SetLimits(0,120);
              KnobR.SetLimits(0,120);
        }

        if(receiver){
          uint8_t currentBA_3 = localkeyArray[3] & 0x03; //00000011 select last 2 bits
          uint8_t currentBA_2 = (localkeyArray[3] & 0x0C)>>2; //00001100 select 2 bits before last 2 bits and shift
          uint8_t currentBA_1 = localkeyArray[4] & 0x03; //00000011 select last 2 bits
          Knob3.UpdateRotateVal(currentBA_3);
          Knob2.UpdateRotateVal(currentBA_2);
          Knob1.UpdateRotateVal(currentBA_1);
          localvolume_r = Knob3.CurRotVal();
          localoctave_r = Knob2.CurRotVal();
          localwave_r = Knob1.CurRotVal();

          //Send Knob info to other modules if something changed
          if (!singleton && (localoctave_r != prevOctave || localvolume_r != prevVolume || localwave_r != prevWave)){
            // Serial.println ("cur octave: "+String(localoctave_r)+" cur volume: "+String(localvolume_r));
            TX_Message[0] = 'K';
            TX_Message[2] = localoctave_r;
            TX_Message[3] = localvolume_r;
            TX_Message[4] = localwave_r;
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          }
        }

        if (sender){
          uint8_t currBA_3 = localkeyArray[3] & 0x03; //00000011 select last 2 bits
          uint8_t currBA_2 = (localkeyArray[3] & 0x0C)>>2; //00001100 select 2 bits before last 2 bits and shift
          uint8_t currBA_1 = localkeyArray[4] & 0x03; //00000011 select last 2 bits
          uint8_t currBA_0 = (localkeyArray[4] & 0x0C)>>2; //00001100 select 2 bits before last 2 bits and shift
          KnobR.UpdateRotateVal(currBA_3);
          KnobS.UpdateRotateVal(currBA_2);
          KnobD.UpdateRotateVal(currBA_1);
          KnobA.UpdateRotateVal(currBA_0);
          localR_s = KnobR.CurRotVal();
          localS_s = KnobS.CurRotVal();
          localD_s = KnobD.CurRotVal();
          localA_s = KnobA.CurRotVal();

          if ((localR_s != prevR) || (localS_s != prevS) || (localD_s != prevD) || (localA_s != prevA)){
            TX_Message[0] = 'A';
            TX_Message[1] = localR_s;
            TX_Message[2] = localS_s;
            TX_Message[3] = localD_s;
            TX_Message[4] = localA_s;
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          }

        }
        prevWave = localwave_r;
        prevOctave = localoctave_r;
        prevVolume = localvolume_r;
        prevR = localR_s;
        prevS = localS_s;
        prevD = localD_s;
        prevA = localA_s;
        //Code for getting cords 
        //Variables needed for getting cords
        uint16_t onehot = pressedKeys^0xFFF;
        uint16_t prevPressedkeysCopy = prevPressedKeys;
        uint16_t onehotCopy = onehot;
        uint8_t p_idx_array[12] = {12,12,12,12,12,12,12,12,12,12,12,12};
        uint8_t r_idx_array[12] = {12,12,12,12,12,12,12,12,12,12,12,12};
        uint8_t cur_idx;
        uint8_t prev_idx;
        uint8_t p_count = 0;
        uint8_t r_count = 0;
        bool pressed =  false;
        bool released = false;


        while (onehotCopy | prevPressedkeysCopy){
          //Checking if there are keys pressed or released
          if  (onehotCopy == 0){
            cur_idx = 12;
            prev_idx = __builtin_ctz(prevPressedkeysCopy);
          }
          else if (prevPressedkeysCopy == 0){
            prev_idx = 12;
            cur_idx = __builtin_ctz(onehotCopy);
          }
          else{
            cur_idx = __builtin_ctz(onehotCopy);
            prev_idx = __builtin_ctz(prevPressedkeysCopy);
          }
        
          //Checking if the keys pressed or deleted have previously been detected
          if (prev_idx==cur_idx){
            onehotCopy &= ~(1<<cur_idx);
            prevPressedkeysCopy &= ~(1<<cur_idx);
          }
          else if (prev_idx>cur_idx){
            pressed = true;
            p_idx_array[p_count] = cur_idx;
            onehotCopy &= ~(1<<cur_idx);
            p_count++;
          }
          else{
            released = true;
            r_idx_array[r_count] = prev_idx;
            prevPressedkeysCopy &= ~(1<<prev_idx);
            r_count++;
          }

        }
        if (pressed){
          Serial.println("Pressed");
          if(receiver){
              //Used for printing purposes
              localnote = notes[p_idx_array[0]];
              localsharp = sharps[p_idx_array[0]];

              
              //int8_t shift = localoctave_r-4;
              
              for (uint8_t i = 0; i < 12; i++){
                if (p_idx_array[i] != 12) {
                  //localstepsize = shift>0 ? stepSizes[p_idx_array[i]]<<shift : stepSizes[p_idx_array[i]]>>-shift;
                  localpressedKeysMin |= (1<<p_idx_array[i]);
                }
                else{
                  break;
                }
              }
          }
        else if(sender){
          for (uint8_t i = 0; i < 12; i++){
            //Store the pressed keys in the TX_Message to be received by the receiver
            if (p_idx_array[i] != 12) {
              TX_Message[0] = 'P';
              TX_Message[1] = octave_s;
              TX_Message[2] = p_idx_array[i];
              TX_Message[3] = pos;   
              xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
            }
            else{
              break;
            }
          }
        }
      }
      if (released){
        Serial.println("Released");
        if(receiver){
            if(!onehot){ //if no keys are pressed
                localnote = ' ';
                localsharp = ' ';
            }
            else if (!pressed){
                uint8_t curr_idx = __builtin_ctz(onehot);
                localnote = notes[curr_idx];
                localsharp = sharps[curr_idx];
            }
          for (uint8_t i = 0; i < 12; i++){
            if (r_idx_array[i] != 12) {
              Serial.println ("Setting to 0");
              Serial.println(String(r_idx_array[i]));
              xSemaphoreTake(tvarMutex_r, portMAX_DELAY);
              t_r[r_idx_array[i]]=0;
              xSemaphoreGive(tvarMutex_r);
              localpressedKeysMin &= ~(1<<r_idx_array[i]);
            }
            else{
              break;
            }
          }
        }
        else if(sender){
            for (uint8_t i = 0; i < 12; i++){
              //Send the released keys to the receiver
              if (r_idx_array[i] != 12) {
                TX_Message[0] = 'R';
                TX_Message[1] = octave_s;
                TX_Message[2] = r_idx_array[i];
                TX_Message[3] = pos;        
                xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
              }
              else{
                break;
              }
            }
        }
      }
      prevPressedKeys = onehot;
      prevMaxOct = maxOct;
      // Serial.printf("PressedKeys %hu\n",pressedKeysArray);
      // Serial.printf("Keys %hu\n",localpressedKeysArrayMaj);
      if (sender){
        __atomic_store_n(&A_s, localA_s, __ATOMIC_RELAXED);
        __atomic_store_n(&D_s, localD_s, __ATOMIC_RELAXED);
        __atomic_store_n(&S_s, localS_s, __ATOMIC_RELAXED);
        __atomic_store_n(&R_s, localR_s, __ATOMIC_RELAXED);
        
      }
      if (receiver){
        __atomic_store_n(&pressedKeysMin, localpressedKeysMin, __ATOMIC_RELAXED);
        __atomic_store_n(&currentnote, localnote, __ATOMIC_RELAXED);
        __atomic_store_n(&currentsharp, localsharp, __ATOMIC_RELAXED);
        __atomic_store_n(&volume_r, localvolume_r, __ATOMIC_RELAXED);
        __atomic_store_n(&octave_r, localoctave_r, __ATOMIC_RELAXED);
        __atomic_store_n(&wave_r, localwave_r, __ATOMIC_RELAXED);
      }
  }
  #else
    uint8_t localprevkey = -1;
    uint8_t TX_Message[8] = {0};
    uint32_t localstepsize = 0;
    char localnote = 0;
    char localsharp = 0;
    uint8_t localvolume_knob3 = volume_r;
    uint8_t localoctave_knob2 = octave_r;
    Knob Knob3(0);
    Knob Knob2(0);
    for (uint8_t idx = 0; idx < 12; idx++){
      for (uint8_t i = 0; i < 4; i++){
          setRow(i);
          delayMicroseconds(3);
          uint8_t val = readCols();
          xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          keyArray[i] = val;
          xSemaphoreGive(keyArrayMutex);
          uint8_t onehot = val^0x0F; //00001111 invert last 4 bits
          if (i < 3){
            for(int j = 0; j<4;j++){
              if (onehot & (1<<j)){
                uint8_t idx = 4*i + j;
                //u8g2.print(idx,DEC);
                int8_t shift = localoctave_knob2 - 4;
                if(shift > 0){
                  localstepsize = stepSizes[idx]<<shift;
                }
                else{
                  localstepsize = stepSizes[idx]>>-shift;
                }
                localnote = notes[idx];
                localsharp = sharps[idx];
                localprevkey = idx;
              }
            }
          }
          else{
            uint8_t currentBA_3 = val & 0x03; //00000011 select last 2 bits
            uint8_t currentBA_2 = (val & 0x0C)>>2; //000011 select last 2 bits
            Knob3.UpdateRotateVal(currentBA_3);
            Knob2.UpdateRotateVal(currentBA_2);
            localvolume_r = Knob3.CurRotVal();
            localoctave_knob2 = Knob2.CurRotVal();  
          }
        }
      localstepsize = stepSizes[idx];
      localnote = notes[idx];
      localsharp = sharps[idx];
      localprevkey = idx;
      localvolume_r = 0;
      localoctave_r = 0;

      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      TX_Message[2] = localprevkey;
      
      xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
    __atomic_store_n(&mastercurrentStepSize, localstepsize, __ATOMIC_RELAXED);
    __atomic_store_n(&currentnote, localnote, __ATOMIC_RELAXED);
    __atomic_store_n(&currentsharp, localsharp, __ATOMIC_RELAXED);
    __atomic_store_n(&volume_knob3, localvolume_knob3, __ATOMIC_RELAXED);
    __atomic_store_n(&octave_knob2, localoctave_knob2, __ATOMIC_RELAXED);
  }
  #endif
}

// Rhea Task: Fix max and min printing once Andy adds it
void displayUpdateTask(void * pvParameters){
  #ifndef TEST_DISPLAY_UPDATE
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_courB08_tr); // choose a suitable font
    // u8g2.setCursor(100,20);
    // u8g2.print(pos,DEC);
    if (receiver){
      uint8_t x, y;
      u8g2.drawStr(80,8,"Wave(K2):");
      u8g2.drawStr(2, 15,"Vol(K4):");
      u8g2.drawStr(2, 23,"Oct(K3):");
      u8g2.drawStr(2, 6,"Note:");
      u8g2.drawStr(2, 32, "K1");  
      u8g2.drawStr(39, 32, "K2");
      u8g2.drawStr(80, 32, "K3");
      u8g2.drawStr(116, 32, "K4");
      u8g2.setCursor(2,2);
      switch (wave_r){
        case 0:
              u8g2.drawLine(82,20,100,10);
              u8g2.drawLine(100,10,100,20);
              u8g2.drawLine(100,20,118,10);
              u8g2.drawLine(118,10,118,20);
              break;
        case 1:
              u8g2.drawLine(82,20,90,10);
              u8g2.drawLine(90,10,98,20);
              u8g2.drawLine(98,20,106,10);
              u8g2.drawLine(106,10,114,20);
              break;
        case 2:
              u8g2.drawLine(82,20,90,20);
              u8g2.drawLine(90,20,90,10);
              u8g2.drawLine(90,10,100,10);
              u8g2.drawLine(100,10,100,20);
              u8g2.drawLine(100,20,110,20);
              u8g2.drawLine(110,20,110,10);
              u8g2.drawLine(110,10,120,10);
              u8g2.drawLine(120,10,120,20);
              break;
        case 3: 
              for (x = 0; x < 40; x++) {
                y = 32 + 6 * sin(2 * PI * x / 40);
                u8g2.drawPixel(x+80, y-15);
              }
              break;
      }
      u8g2.setCursor(32,6);
      u8g2.print(currentnote);
      u8g2.print(currentsharp);
      u8g2.setCursor(50,16);
      if (volume_r==8){
        u8g2.drawStr(58, 15,"max");
      }
      else if (volume_r==0){
          u8g2.drawStr(58, 15,"min");
      }
      u8g2.print(volume_r,DEC);
      u8g2.setCursor(50,24);
      if (octave_r==8){
        u8g2.drawStr(58, 23,"max");
      }
      else if (octave_r==0){
          u8g2.drawStr(58, 23,"min");
      }
      u8g2.print(octave_r,DEC);
    }

    if (sender){
      u8g2.drawStr(80,8,"A:");
      u8g2.drawStr(80,18,"D:");
      u8g2.drawStr(80,28,"S:");
      u8g2.drawStr(110,8,"R:");
      u8g2.drawStr(2, 15,"Vol:");
      u8g2.drawStr(2, 23,"Oct:");
      u8g2.setCursor(87,8);
      u8g2.print(A_s,DEC);
      u8g2.setCursor(87,18);
      u8g2.print(D_s,DEC);
      u8g2.setCursor(87,28);
      u8g2.print(S_s,DEC);
      u8g2.setCursor(120,8);
      u8g2.print(R_s,DEC);
      // u8g2.drawStr(2, 6,"Note:");
      u8g2.setCursor(35,10);
      // u8g2.print(currentnote);
      // u8g2.print(currentsharp);
      u8g2.setCursor(25,16);
      if (volume_s==8){
        u8g2.drawStr(33, 15,"max");
      }
      else if (volume_s==0){
          u8g2.drawStr(33, 15,"min");
      }
      u8g2.print(volume_s,DEC);
      u8g2.setCursor(25,24);
      if (octave_s==8){
        u8g2.drawStr(33, 23,"max");
      }
      else if (octave_s==0){
          u8g2.drawStr(33, 23,"min");
      }
      u8g2.print(octave_s,DEC);

    }

    u8g2.sendBuffer();          
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
  #else
   for (uint8_t idx = 0; idx < 1; idx++){
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(2,10);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(idx,HEX);
    u8g2.print(idx,HEX);
    u8g2.print(idx,HEX);
    xSemaphoreGive(keyArrayMutex);
    u8g2.setCursor(2,20);
    u8g2.print(idx,DEC);
    u8g2.setCursor(10,20);
    u8g2.print(idx,DEC);
    u8g2.setCursor(2,30);
    u8g2.print((char)(idx+65));
    u8g2.print((char)(idx+65));

    u8g2.setCursor(66,30);
    u8g2.print((char) (idx+65));
    u8g2.print(idx);
    u8g2.print(idx);

    u8g2.sendBuffer();          
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }

  #endif
}

void setup() {
  // put your setup code here, to run once:
    msgInQ = xQueueCreate(36,8);
 
  #ifndef TEST_SCAN_KEYS
    msgOutQ = xQueueCreate(36,8);
  #else
    msgOutQ = xQueueCreate(384,8);
  #endif


  #ifndef DISABLE_THREADS
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */

    TaskHandle_t displayUpdateTaskHandle = NULL;
    xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateTaskHandle );

    TaskHandle_t ISRTaskHandle = NULL;
    xTaskCreate(
    ISRTask,		/* Function that implements the task */
    "ISRTaskUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &ISRTaskHandle );
    
    TaskHandle_t decodeTaskHandle = NULL;
    xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &decodeTaskHandle);

    TaskHandle_t CAN_TX_TaskHandle = NULL;
    xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CanTX",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &CAN_TX_TaskHandle);

    TaskHandle_t sendSoundTaskHandle = NULL;
    xTaskCreate(
    sendSoundTask,		/* Function that implements the task */
    "sendSound",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */ 
    &sendSoundTaskHandle);
  #endif

  #ifdef TEST_SCAN_KEYS
  TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */
  #endif

  #ifdef TEST_DISPLAY_UPDATE
  TaskHandle_t displayUpdateTaskHandle = NULL;
    xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateTaskHandle );
  #endif

  #ifdef TEST_DECODE
  TaskHandle_t decodeTaskHandle = NULL;
    xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &decodeTaskHandle);
  #endif

  #ifdef TEST_CAN_TX
  TaskHandle_t CAN_TX_TaskHandle = NULL;
    xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CanTX",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &CAN_TX_TaskHandle);
  #endif

  pressedKeysArrayMutex = xSemaphoreCreateMutex();
  tvarMutex_r = xSemaphoreCreateMutex();
  tvarMutex_s = xSemaphoreCreateMutex();
  sampleBufferMutex = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferMutex);
  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);




  CAN_Init(false);
  #ifndef DISABLE_THREADS
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  setCANFilter(0x123,0x7ff);
  CAN_Start();



  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  uint32_t id_0 = HAL_GetUIDw0();
  uint32_t id_1 = HAL_GetUIDw1();
  uint32_t id_2 = HAL_GetUIDw2();

  uint16_t id_0_half = (id_0 >> 16) ^ (id_0 & 0x00FF);
  uint16_t id_1_half = (id_1 >> 16) ^ (id_1 & 0x00FF);
  uint16_t id_2_half = (id_2 >> 16) ^ (id_2 & 0x00FF);

  uint8_t id_0_byte = (id_0_half >> 8) ^ (id_0_half & 0x00FF);
  uint8_t id_1_byte = (id_1_half >> 8) ^ (id_1_half & 0x00FF);
  uint8_t id_2_byte = (id_2_half >> 8) ^ (id_2_half & 0x00FF);

  uint8_t id = id_0_byte ^ id_1_byte ^ id_2_byte;

  // Serial.print("ID: ");
  // Serial.print(id_0, HEX);
  // Serial.print(id_1, HEX);
  // Serial.println(id_2, HEX);

  bool outBits[7] = {0,0,0,1,1,1,1};

  for (uint8_t i=0; i<7; i++) {
    setRow(i);                     //Set row address
    digitalWrite(OUT_PIN,outBits[i]); //Set value to latch in DFF
  }
  
  delayMicroseconds(1000);

  for (uint8_t i=5; i<7; i++) {
    setRow(i);
    digitalWrite(REN_PIN,1);          //Enable selected row
    delayMicroseconds(3);             //Wait for column inputs to stabilise
    keyArray[i] = readCols();         //Read column inputs
    digitalWrite(REN_PIN,0);          //Disable selected row
  }

  west_detect = ((keyArray[5]&0x08)>>3)^0x01;
  east_detect = ((keyArray[6]&0x08)>>3)^0x01;
  auto_detect(west_detect,east_detect);

  if(receiver){
    // Serial.println("Reciever");
  }
  else if(sender){
    // Serial.println("Sender");
  }
  else{
    // Serial.println("Error");
  }
  //Initialise UART
  Serial.begin(9600);
  // Serial.println("Hello World");
  // Serial.print("init west_detect: ");
  // Serial.println(west_detect);
  // Serial.print("init east_detect: ");
  // Serial.println(east_detect);

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  #ifndef DISABLE_THREADS
  sampleTimer->attachInterrupt(sampleISR);
  #endif
  sampleTimer->resume();

  #ifdef DISABLE_THREADS
    uint32_t startTime = micros();
    #ifdef TEST_SCAN_KEYS
      for (int iter = 0; iter < 32; iter++) {
        scanKeysTask(NULL);
      }
    #endif
    #ifdef TEST_DISPLAY_UPDATE
      for (int iter = 0; iter < 32; iter++) {
        displayUpdateTask(NULL);
      }
    #endif
    Serial.println(micros()-startTime);
    while(1);
  #else
    vTaskStartScheduler();
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
}