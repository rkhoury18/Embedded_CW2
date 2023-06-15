#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "knobs.h"

#define sender 0
//#define reciever 0

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[12] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};
  const char notes[12] = {'C','C','D','D','E','F','F','G','G','A','A','B'};
  const char sharps[12] = {'','#','','#','','','#','','#','','#',''};
//global variables  
  volatile uint32_t currentStepSize;
  volatile uint32_t mastercurrentStepSize;
  volatile char currentnote;
  volatile char currentsharp;
  volatile uint8_t knob3rotation = 4;
  volatile uint8_t knob2rotation = 4;

  volatile uint8_t keyArray[7];
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
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3rotation);
  analogWrite(OUTR_PIN, Vout + 128);
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
  uint32_t localstep;
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
    localstep = recievestep ? recievestep:mastercurrentStepSize;
  __atomic_store_n(&currentStepSize, localstep, __ATOMIC_RELAXED);
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
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Knob Knob3(0,0,8);
  Knob Knob2(0,0,8);
  uint8_t localprevkey = -1;
  uint8_t TX_Message[8] = {0};
  while(1){
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
      uint32_t localstepsize = 0;
      char localnote = 0;
      char localsharp = 0;
      uint8_t localknob3rotation = knob3rotation;
      uint8_t localknob2rotation = knob2rotation;
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
<<<<<<< HEAD
          uint8_t currezntBA = val & 0x03; //00000011 select last 2 bits
          switch (previousBA){
            case 0: 
              if(currentBA == 1 && localknob3rotation<8){
                localknob3rotation++;
                prevfunc = 1;
              }
              else if(currentBA == 3 && localknob3rotation>0 && localknob3rotation<8){
                localknob3rotation+=prevfunc;
              }
              break;
            case 1:
              if(currentBA == 0 && localknob3rotation>0){
                localknob3rotation--;
                prevfunc = -1;
              }
              else if(currentBA == 2 && localknob3rotation>0 && localknob3rotation<8){
                localknob3rotation+=prevfunc;
              }
              break;
            case 2:
              if(currentBA == 3 && localknob3rotation>0){
                localknob3rotation--;
                prevfunc = -1;
              }
              else if(currentBA == 1 && localknob3rotation>0 && localknob3rotation<8){
                localknob3rotation+=prevfunc;
              }
              break;
            case 3:
              if(currentBA == 2 && localknob3rotation<8){
                localknob3rotation++;
                prevfunc = 1;
              }
              else if(currentBA == 0 && localknob3rotation>0 && localknob3rotation<8){
                localknob3rotation+=prevfunc;
              }
              break;
          }
          previousBA = currentBA;    
=======
          uint8_t currentBA_3 = val & 0x03; //00000011 select last 2 bits
          uint8_t currentBA_2 = (val & 0x0C)>>2; //000011 select last 2 bits
          Knob3.UpdateRotateVal(currentBA_3);
          Knob2.UpdateRotateVal(currentBA_2);
          localknob3rotation = Knob3.CurRotVal();
          localknob2rotation = Knob2.CurRotVal();  
>>>>>>> c9419d761ac7d55a920114dc094d2cfd455678c1
        }
      }
    #ifdef sender
      if(localnote){
      TX_Message[0] = 'P';
      TX_Message[1] = localknob2rotation;
      TX_Message[2] = localprevkey;
    }
    else if(localprevkey != 255){
      TX_Message[0] = 'R';
      TX_Message[1] = localknob2rotation;
      TX_Message[2] = localprevkey;
    }
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
  #endif
  __atomic_store_n(&mastercurrentStepSize, localstepsize, __ATOMIC_RELAXED);
  __atomic_store_n(&currentnote, localnote, __ATOMIC_RELAXED);
  __atomic_store_n(&currentsharp, localsharp, __ATOMIC_RELAXED);
  __atomic_store_n(&knob3rotation, localknob3rotation, __ATOMIC_RELAXED);
  __atomic_store_n(&knob2rotation, localknob2rotation, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters){
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

    // u8g2.setCursor(66,30);
    // u8g2.print((char) RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);

    u8g2.sendBuffer();          
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
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
  msgOutQ = xQueueCreate(36,8);

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
  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  bool mode = false;
  #ifdef sender
    #ifdef reciever
      mode = true;
    #endif
  #endif
  CAN_Init(mode);
  #ifdef reciever
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  #endif
  #ifdef sender
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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}