#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "knobs.h"

#define sender 0
//#define reciever 0

//#define DISABLE_THREADS
//#define TEST_SCAN_KEYS
//#define TEST_DISPLAY_UPDATE

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[12] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};
  const char notes[12] = {'C','C','D','D','E','F','F','G','G','A','A','B'};
  const char sharps[12] = {' ','#',' ','#',' ',' ','#',' ','#',' ','#',' '};
//global variables  
  //volatile uint32_t currentStepSize;
  //volatile uint32_t mastercurrentStepSize;
  volatile char currentnote;
  volatile char currentsharp;
  volatile uint8_t knob3rotation = 4;
  volatile uint8_t knob2rotation = 4;
  volatile uint8_t keyArray[7];
  volatile uint32_t localrangekeyarray[12] = {0};
  volatile uint32_t fullrangekeyarray[24] = {0};
  SemaphoreHandle_t localrangekeyarrayMutex;
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t CAN_TX_Semaphore;
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  uint8_t RX_Message[8]={0};
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

void sampleISR() {
  static uint32_t phaseAcc[36] = {0};
  int32_t polyphony_vout = 0;
  for(uint8_t i=0; i<12; i++){
    phaseAcc[i] += localrangekeyarray[i];
    int32_t Vout = (phaseAcc[i] >> 24) - 128;
    Vout = Vout >> (8 - knob3rotation);
    // if (polyphony_vout + Vout > 127){
    //   polyphony_vout = 127;
    // }
    // else if (polyphony_vout + Vout < -128){
    //   polyphony_vout = -128;
    // }
    // else{
    // polyphony_vout += Vout;
    // }
    polyphony_vout += Vout;
  }
  for(uint8_t i=0; i<24; i++){
    phaseAcc[12+i] += fullrangekeyarray[i];
    int32_t Vout = (phaseAcc[12+i] >> 24) - 128;
    Vout = Vout >> (8 - knob3rotation);
    // if (polyphony_vout + Vout > 127){
    //   polyphony_vout = 127;
    // }
    // else if (polyphony_vout + Vout < -128){
    //   polyphony_vout = -128;
    // }
    // else{
    // polyphony_vout += Vout;
    // }
    polyphony_vout += Vout;
  }
  analogWrite(OUTR_PIN, polyphony_vout + 128);
}

#ifdef reciever
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void decodeTask(void * pvParameters){
  uint32_t recievestep;
  while(1){
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    if (RX_Message[0] == 'R'){
      recievestep = 0;
    }
    else{
      int8_t shift = RX_Message[1]-4;
      if(shift > 0){
        recievestep = stepSizes[RX_Message[2]]<<shift;
      }
      else{
        recievestep = stepSizes[RX_Message[2]]>>-shift;
      }
    }
    fullrangekeyarray[(12*RX_Message[3])+RX_Message[2]] = recievestep;//write step size for keys recoeved from sending keyboards
  }
}
#endif

#ifdef sender
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
#endif



void scanKeysTask(void * pvParameters) {
  #ifndef TEST_SCAN_KEYS
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Knob Knob3(0,0,8);
    Knob Knob2(0,0,8);
    uint16_t prevfullkeys = 0;
    uint16_t fullkeys = 0;
    uint8_t TX_Message[8] = {0};
    while(1){
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        uint32_t localstepsize = 0;
        char localnote = currentnote;
        char localsharp = currentsharp;
        uint8_t localknob3rotation = knob3rotation;
        uint8_t localknob2rotation = knob2rotation;
        uint8_t localkeyArray[7] = {0};
        for (uint8_t i = 0; i < 4; i++){
          setRow(i);
          delayMicroseconds(3);
          uint8_t val = readCols();
          localkeyArray[i] = val;
        }
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          memcpy((void*)keyArray, localkeyArray, sizeof(localkeyArray));
        xSemaphoreGive(keyArrayMutex);
        fullkeys = localkeyArray[0] | (localkeyArray[1]<<4) | (localkeyArray[2]<<8);

        uint8_t currentBA_3 = localkeyArray[3] & 0x03; //00000011 select last 2 bits
        uint8_t currentBA_2 = (localkeyArray[3] & 0x0C)>>2; //000011 select last 2 bits
        Knob3.UpdateRotateVal(currentBA_3);
        Knob2.UpdateRotateVal(currentBA_2);
        localknob3rotation = Knob3.CurRotVal();
        localknob2rotation = Knob2.CurRotVal();

        uint16_t onehot = fullkeys^0xFFF; //00001111 invert last 4 bits
        uint16_t difference = onehot^prevfullkeys;
        if(difference){
          uint8_t idx_array[12] = {12,12,12,12,12,12,12,12,12,12,12,12};
          uint8_t c = 0;
          while(difference){
            uint8_t idx = __builtin_ctz(difference);
            idx_array[c] = (idx);
            difference &= ~(1<<idx);
            c++;
          }
          if(onehot>prevfullkeys){ //current is bigger than previous key is pressed
            localnote = notes[idx_array[0]];
            localsharp = sharps[idx_array[0]];
            #ifdef sender
            for (uint8_t i = 0; i < 12; i++){
              if (idx_array[i] != 12) {
                TX_Message[0] = 'P';
                TX_Message[1] = localknob2rotation;
                TX_Message[2] = idx_array[i];
                TX_Message[3] = sender;    
                xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
              }
              else{
                break;
              }
            }
            #endif

            #ifdef reciever
            int8_t shift = localknob2rotation-4;
            xSemaphoreTake(localrangekeyarrayMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 12; i++){
              if (idx_array[i] != 12) {
                if(shift > 0){
                  localstepsize = stepSizes[idx_array[i]]<<shift;
                }
                else{
                  localstepsize = stepSizes[idx_array[i]]>>-shift;
                }
                Serial.print("localstepsize: ");
                Serial.println(localstepsize);
                localrangekeyarray[idx_array[i]] = localstepsize;
              }
              else{
                break;
              }
            }
            xSemaphoreGive(localrangekeyarrayMutex);
            #endif
          }
          else{
            if(!onehot){ //if no keys are pressed
                localnote = ' ';
                localsharp = ' ';
              }
            else{
              uint8_t curr_idx = __builtin_ctz(onehot);
              localnote = notes[curr_idx];
              localsharp = sharps[curr_idx];
            }
            #ifdef sender
              for (uint8_t i = 0; i < 12; i++){
                if (idx_array[i] != 12) {
                  TX_Message[0] = 'R';
                  TX_Message[1] = localknob2rotation;
                  TX_Message[2] = idx_array[i];
                  TX_Message[3] = sender;    
                  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
                }
                else{
                  break;
                }
              }
            #endif
            
            #ifdef reciever
            xSemaphoreTake(localrangekeyarrayMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 12; i++){
              if (idx_array[i] != 12) {
                localrangekeyarray[idx_array[i]] = 0;
              }
              else{
                break;
              }
            }
            xSemaphoreGive(localrangekeyarrayMutex);
            #endif
          }
        }
        prevfullkeys = onehot;
        // for (uint8_t i = 0; i < 4; i++){
        //   setRow(i);
        //   delayMicroseconds(3);
        //   uint8_t val = readCols();
        //   xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        //   keyArray[i] = val;
        //   xSemaphoreGive(keyArrayMutex);
        //   uint8_t onehot = val^0x0F; //00001111 invert last 4 bits
        //   if (i < 3){
        //     uint8_t difference = onehot^prevkeyarray[i];
        //     if(difference){
        //       uint8_t idx = 4*i + __builtin_ctz(difference);
        //       Serial.println("idx:");
        //       Serial.println(idx);
        //       if (onehot>prevkeyarray[i]){ //current is bigger than previous key is pressed, also presume only one key can be pressed at a time
        //         localnote = notes[idx];
        //         localsharp = sharps[idx];
        //         #ifdef sender
        //           TX_Message[0] = 'P';
        //           TX_Message[1] = localknob2rotation;
        //           TX_Message[2] = idx;
        //           TX_Message[3] = sender;              
        //         #endif
        //         int8_t shift = localknob2rotation-4;
        //         if(shift > 0){
        //           localstepsize = stepSizes[idx]<<shift;
        //         }
        //         else{
        //           localstepsize = stepSizes[idx]>>-shift;
        //         }
        //         xSemaphoreTake(localrangekeyarrayMutex, portMAX_DELAY);
        //           localrangekeyarray[idx] = localstepsize;
        //         xSemaphoreGive(localrangekeyarrayMutex);
        //       }
        //       else{ //current is smaller than previous key is released
        //           if(!onehot){ //if no keys are pressed
        //             localnote = ' ';
        //             localsharp = ' ';
        //           }
        //           else{
        //             uint8_t curr_idx = 4*i + __builtin_ctz(onehot);
        //             localnote = notes[curr_idx];
        //             localsharp = sharps[curr_idx];
        //           }
        //         #ifdef sender
        //           TX_Message[0] = 'R';
        //           TX_Message[1] = localknob2rotation;
        //           TX_Message[2] = idx;
        //           TX_Message[3] = sender;
        //         #endif
        //         xSemaphoreTake(localrangekeyarrayMutex, portMAX_DELAY);
        //           fullrangekeyarray[idx] = 0;
        //         xSemaphoreGive(localrangekeyarrayMutex);
        //       }
        //     #ifdef sender
        //       xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        //     #endif
        //     }
        //     else{
        //       // uint8_t curr_idx = 4*i + __builtin_ctz(onehot);
        //       // localnote = notes[curr_idx];
        //       // localsharp = sharps[curr_idx];
        //     }
        //     prevkeyarray[i] = onehot;
        //   }
        //   else{
        //     uint8_t currentBA_3 = val & 0x03; //00000011 select last 2 bits
        //     uint8_t currentBA_2 = (val & 0x0C)>>2; //000011 select last 2 bits
        //     Knob3.UpdateRotateVal(currentBA_3);
        //     Knob2.UpdateRotateVal(currentBA_2);
        //     localknob3rotation = Knob3.CurRotVal();
        //     localknob2rotation = Knob2.CurRotVal();  
        //   }
        // }
    __atomic_store_n(&currentnote, localnote, __ATOMIC_RELAXED);
    __atomic_store_n(&currentsharp, localsharp, __ATOMIC_RELAXED);
    __atomic_store_n(&knob3rotation, localknob3rotation, __ATOMIC_RELAXED);
    __atomic_store_n(&knob2rotation, localknob2rotation, __ATOMIC_RELAXED);
    }
  #else
    uint8_t localprevkey = -1;
    uint8_t TX_Message[8] = {0};
    uint32_t localstepsize = 0;
    char localnote = 0;
    char localsharp = 0;
    uint8_t localknob3rotation = knob3rotation;
    uint8_t localknob2rotation = knob2rotation;
    Knob Knob3(0,0,8);
    Knob Knob2(0,0,8);
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
                int8_t shift = localknob2rotation - 4;
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
            localknob3rotation = Knob3.CurRotVal();
            localknob2rotation = Knob2.CurRotVal();  
          }
        }
      localstepsize = stepSizes[idx];
      localnote = notes[idx];
      localsharp = sharps[idx];
      localprevkey = idx;
      localknob3rotation = 0;
      localknob2rotation = 0;

      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      TX_Message[2] = localprevkey;
      
      xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
    __atomic_store_n(&mastercurrentStepSize, localstepsize, __ATOMIC_RELAXED);
    __atomic_store_n(&currentnote, localnote, __ATOMIC_RELAXED);
    __atomic_store_n(&currentsharp, localsharp, __ATOMIC_RELAXED);
    __atomic_store_n(&knob3rotation, localknob3rotation, __ATOMIC_RELAXED);
    __atomic_store_n(&knob2rotation, localknob2rotation, __ATOMIC_RELAXED);
  }
  #endif
}


void displayUpdateTask(void * pvParameters){
  #ifndef TEST_DISPLAY_UPDATE
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.setCursor(2,10);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[0],HEX);
    u8g2.print(keyArray[1],HEX);
    u8g2.print(keyArray[2],HEX);
    xSemaphoreGive(keyArrayMutex);
    u8g2.setCursor(2,20);
    u8g2.print(knob3rotation,DEC);
    u8g2.setCursor(10,20);
    u8g2.print(knob2rotation,DEC);
    u8g2.setCursor(2,30);
    u8g2.print(currentnote);
    u8g2.print(currentsharp);

    u8g2.setCursor(66,30);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

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

uint8_t one_hot_decode(uint8_t onehot){
  uint8_t count = 0;
  for(count; count<4;count++){
    if (onehot == 1){
      break;
    }
    else {
      onehot>>=1;
    }
  }
  return count;
}

void setup() {
  // put your setup code here, to run once:
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
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
    
    #ifdef reciever
    TaskHandle_t decodeTaskHandle = NULL;
    xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &decodeTaskHandle);
    #endif

    #ifdef sender
    TaskHandle_t CAN_TX_TaskHandle = NULL;
    xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CanTX",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &CAN_TX_TaskHandle);
    #endif
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

  localrangekeyarrayMutex = xSemaphoreCreateMutex();
  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  #ifndef DISABLE_THREADS
    sampleTimer->attachInterrupt(sampleISR);
  #endif
  sampleTimer->resume();
  
  bool mode = false;
  #ifdef sender
    #ifdef reciever
      mode = true;
    #endif
  #endif
  CAN_Init(mode);
  #ifndef DISABLE_THREADS
    #ifdef reciever
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    #endif
  #endif

  #ifndef DISABLE_THREADS
    #ifdef sender
    CAN_RegisterTX_ISR(CAN_TX_ISR);
    #endif
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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

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
  #endif

  #ifndef DISABLE_THREADS
    vTaskStartScheduler();
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
}