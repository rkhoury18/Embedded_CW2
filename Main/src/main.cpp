#include <Arduino.h>
#include <U8g2lib.h>
#include <iostream>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "knobs.h"

// #define sender 0
// #define reciever 0

//#define DISABLE_THREADS
// #define TEST_SCAN_KEYS
//#define TEST_DISPLAY_UPDATE

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[12] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};
  const char notes[12] = {'C','C','D','D','E','F','F','G','G','A','A','B'};
  const char sharps[12] = {' ','#',' ','#',' ',' ','#',' ','#',' ','#',' '};
//global variables  
  //volatile uint32_t currentStepSize;
  //volatile uint32_t mastercurrentStepSize;
  //#ifdef reciever
  volatile char currentnote;
  volatile char currentsharp;
  volatile uint8_t volume_knob3 = 4;
  volatile uint8_t octave_knob2 = 4;
  volatile uint32_t fullrangekeyarray[36] = {0};
  SemaphoreHandle_t fullrangekeyarrayMutex;

  volatile uint8_t west_detect = 10;
  volatile uint8_t east_detect = 10;
  //#endif
  
  volatile uint8_t pos;

  volatile bool reciever = false;
  volatile bool sender = false;
  volatile bool singleton = true;

  uint8_t RX_Message[8]={0};
  SemaphoreHandle_t CAN_TX_Semaphore;
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;

  //#ifdef sender
  volatile uint8_t octave;
  volatile uint8_t volume;
  //#endif

  volatile uint8_t keyArray[7];
  SemaphoreHandle_t keyArrayMutex;

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


void auto_detect(bool west, bool east){
  if(!west){ //most west module
    pos = 0;
    reciever = true;
    sender = false;
    if(east){ //2 modules
      singleton = false;
      uint8_t TX_Message[8] = {'H',0,0,0,0,0,0,0}; //Handshake, id, modules, position, 0, 0, 0, 0, 0
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }
    else{
      singleton = true;
    }
  }
  else{
    sender = true; //either east or middle 2/3 modules
    reciever = false;
    singleton = false;
  //   while(RX_Message[0] != 'H'){
  //     xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
  //   }
  //   pos = RX_Message[1] + 1;
  //   if(east){
  //     uint8_t TX_Message[8] = {'H',pos, 0, 0, 0, 0, 0};
  //   }
  }
}


//#ifdef reciever
void sampleISR() {
  if(reciever){
    static uint32_t phaseAcc[36] = {0};
    int32_t polyphony_vout = 0;
    for(uint8_t i=0; i<36; i++){
      phaseAcc[i] += fullrangekeyarray[i];
      if(fullrangekeyarray[i]){
        int32_t Vout = ((phaseAcc[i] >> 24) - 128);
        polyphony_vout += Vout;
      }
    }
    polyphony_vout = polyphony_vout >> (8 - volume_knob3);
    polyphony_vout = max(-128, min(127, (int)polyphony_vout));
    analogWrite(OUTR_PIN, polyphony_vout + 128);
  }
}

void recieverTask(){
  uint32_t recievestep = 0;
  Serial.printf("Recieved: %c %d %d %d\n", RX_Message[0], RX_Message[1], RX_Message[2], RX_Message[3]);
  if (RX_Message[0] == 'R'){
    recievestep = 0;
  }
  else if (RX_Message[0] == 'P'){
    int8_t shift = RX_Message[1]-4;
    if(shift > 0){
      recievestep = stepSizes[RX_Message[2]]<<shift;
    }
    else{
      recievestep = stepSizes[RX_Message[2]]>>-shift;
    }
  }
  xSemaphoreTake(fullrangekeyarrayMutex, portMAX_DELAY);
  fullrangekeyarray[(12*RX_Message[3])+RX_Message[2]] = recievestep;//write step size for keys recoeved from sending keyboards
  xSemaphoreGive(fullrangekeyarrayMutex);
}

void senderTask(){
  if (RX_Message[0] == 'H'){
    pos = RX_Message[1] + 1;
    //Serial.printf("id is %d, pos is %d\n", RX_Message[1], pos);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      uint8_t west_detect = ((keyArray[5]&0x08)>>3)^0x01;
      uint8_t east_detect = ((keyArray[6]&0x08)>>3)^0x01;
    xSemaphoreGive(keyArrayMutex);
    if(east_detect){
      uint8_t TX_Message[8] = {'H',pos, 0, 0, 0, 0, 0, 0};
    }
    // Serial.printf("id is %d, octave is %d\n", sender_id, octave);
  }
  else if (RX_Message[0] == 'K'){
    octave = RX_Message[2] + pos;
    volume = RX_Message[3];
    //Serial.printf("octave is %d, volume is %d\n", octave, volume);
  }
}

void decodeTask(void * pvParameters){
  while(1){
  xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    if(reciever){
      recieverTask();
    }
    else if(sender){
      senderTask();
    }
  }
}
//#endif

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

//#ifdef sender
// void decodeTask(void * pvParameters){
//   
// }

//#endif



void scanKeysTask(void * pvParameters) {
  #ifndef TEST_SCAN_KEYS
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Knob Knob3(0,0,8);
    Knob Knob2(0,0,8);
    uint16_t prevfullkeys = 0;
    uint16_t fullkeys = 0;
    uint8_t TX_Message[8] = {0};
    uint32_t reset[36] = {0};

    while(1){
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        //#ifdef reciever
        uint32_t localstepsize = 0;
        char localnote = currentnote;
        char localsharp = currentsharp;
        uint8_t localvolume_knob3 = volume_knob3;
        uint8_t localoctave_knob2 = octave_knob2;
        //#endif
        
        uint8_t localkeyArray[7] = {0};
        for (uint8_t i = 0; i < 7; i++){
          setRow(i);
          delayMicroseconds(3);
          uint8_t val = readCols();
          localkeyArray[i] = val;
        }

        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          memcpy((void*)keyArray, localkeyArray, sizeof(localkeyArray));
        xSemaphoreGive(keyArrayMutex);

        fullkeys = localkeyArray[0] | (localkeyArray[1]<<4) | (localkeyArray[2]<<8);
        uint8_t localwest_detect = ((localkeyArray[5]&0x08)>>3)^0x01;
        uint8_t localeast_detect = ((localkeyArray[6]&0x08)>>3)^0x01;
        //Serial.printf("west_detect is %d, east_detect is %d\n", localwest_detect, localeast_detect);
        if (localwest_detect != west_detect){ //west detect has changed
        if(reciever){
          xSemaphoreTake(fullrangekeyarrayMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 36; i++){
              fullrangekeyarray[i] = 0;
            }
          xSemaphoreGive(fullrangekeyarrayMutex);
        }
          auto_detect(localwest_detect, localeast_detect);
          __atomic_store_n(&west_detect, localwest_detect, __ATOMIC_RELAXED);
        }
        if (localeast_detect != east_detect){ //east detect has changed
        if(reciever){
          xSemaphoreTake(fullrangekeyarrayMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 36; i++){
              fullrangekeyarray[i] = 0;
            }
          xSemaphoreGive(fullrangekeyarrayMutex);
        }
          auto_detect(localwest_detect, localeast_detect);
          __atomic_store_n(&east_detect, localeast_detect, __ATOMIC_RELAXED);
        }
        if(reciever){
          uint8_t currentBA_3 = localkeyArray[3] & 0x03; //00000011 select last 2 bits
          uint8_t currentBA_2 = (localkeyArray[3] & 0x0C)>>2; //000011 select last 2 bits
          Knob3.UpdateRotateVal(currentBA_3);
          Knob2.UpdateRotateVal(currentBA_2);
          localvolume_knob3 = Knob3.CurRotVal();
          localoctave_knob2 = Knob2.CurRotVal();
          if (!singleton){
            TX_Message[0] = 'K';
            TX_Message[2] = localoctave_knob2;
            TX_Message[3] = localvolume_knob3;
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          }
        }
        //#endif

        //Starting new approach 
        uint16_t onehot = fullkeys^0xFFF;
        uint16_t prevfullkeysCopy = prevfullkeys;
        uint16_t onehotCopy = onehot;
        uint8_t p_idx_array[12] = {12,12,12,12,12,12,12,12,12,12,12,12};
        uint8_t r_idx_array[12] = {12,12,12,12,12,12,12,12,12,12,12,12};
        uint8_t cur_idx;
        uint8_t prev_idx;
        uint8_t p_count = 0;
        uint8_t r_count = 0;
        bool pressed =  false;
        bool released = false;
        while (onehotCopy | prevfullkeysCopy){
          if  (onehotCopy == 0){
            cur_idx = 12;
             prev_idx = __builtin_ctz(prevfullkeysCopy);
          }
          else if (prevfullkeysCopy == 0){
            prev_idx = 12;
            cur_idx = __builtin_ctz(onehotCopy);
          }
          else{
            cur_idx = __builtin_ctz(onehotCopy);
            prev_idx = __builtin_ctz(prevfullkeysCopy);
          }

          if (prev_idx==cur_idx){
            onehotCopy &= ~(1<<cur_idx);
            prevfullkeysCopy &= ~(1<<cur_idx);
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
            prevfullkeysCopy &= ~(1<<prev_idx);
            r_count++;
          }

        }
        if (pressed){
          if(reciever){
            localnote = notes[p_idx_array[0]];
            localsharp = sharps[p_idx_array[0]];

            int8_t shift = localoctave_knob2-4;
            xSemaphoreTake(fullrangekeyarrayMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 12; i++){
              if (p_idx_array[i] != 12) {
                if(shift > 0){
                  localstepsize = stepSizes[p_idx_array[i]]<<shift;
                }
                else{
                  localstepsize = stepSizes[p_idx_array[i]]>>-shift;
                }
                // Serial.print("localstepsize: ");
                // Serial.println(localstepsize);
                fullrangekeyarray[p_idx_array[i]] = localstepsize;
              }
              else{
                break;
              }
            }
            xSemaphoreGive(fullrangekeyarrayMutex);
          }

          else if(sender){
            for (uint8_t i = 0; i < 12; i++){
              if (p_idx_array[i] != 12) {
                TX_Message[0] = 'P';
                TX_Message[1] = octave;
                TX_Message[2] = p_idx_array[i];
                TX_Message[3] = pos;
                //Serial.printf("TX_Message: %c %d %d %d\n", TX_Message[0], TX_Message[1], TX_Message[2], TX_Message[3]);    
                xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
              }
              else{
                break;
              }
            }
          }
        }
        if (released){
          if(reciever){
            if(!onehot){ //if no keys are pressed
              localnote = ' ';
              localsharp = ' ';
            }
            else if (!pressed){
              uint8_t curr_idx = __builtin_ctz(onehot);
              localnote = notes[curr_idx];
              localsharp = sharps[curr_idx];
            }

            xSemaphoreTake(fullrangekeyarrayMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < 12; i++){
              if (r_idx_array[i] != 12) {
                fullrangekeyarray[r_idx_array[i]] = 0;
              }
              else{
                break;
              }
            }
            xSemaphoreGive(fullrangekeyarrayMutex);
          }
          else if(sender){
              for (uint8_t i = 0; i < 12; i++){
                if (r_idx_array[i] != 12) {
                  TX_Message[0] = 'R';
                  TX_Message[1] = octave;
                  TX_Message[2] = r_idx_array[i];
                  TX_Message[3] = pos;    
                //Serial.printf("TX_Message: %c %d %d %d\n", TX_Message[0], TX_Message[1], TX_Message[2], TX_Message[3]);    
                  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
                }
                else{
                  break;
                }
              }
          }
        }
        prevfullkeys = onehot;
    if (reciever){
      __atomic_store_n(&currentnote, localnote, __ATOMIC_RELAXED);
      __atomic_store_n(&currentsharp, localsharp, __ATOMIC_RELAXED);
      __atomic_store_n(&volume_knob3, localvolume_knob3, __ATOMIC_RELAXED);
      __atomic_store_n(&octave_knob2, localoctave_knob2, __ATOMIC_RELAXED);
    }
  }
  #else
    uint8_t localprevkey = -1;
    uint8_t TX_Message[8] = {0};
    uint32_t localstepsize = 0;
    char localnote = 0;
    char localsharp = 0;
    uint8_t localvolume_knob3 = volume_knob3;
    uint8_t localoctave_knob2 = octave_knob2;
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
            localvolume_knob3 = Knob3.CurRotVal();
            localoctave_knob2 = Knob2.CurRotVal();  
          }
        }
      localstepsize = stepSizes[idx];
      localnote = notes[idx];
      localsharp = sharps[idx];
      localprevkey = idx;
      localvolume_knob3 = 0;
      localoctave_knob2 = 0;

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
    u8g2.setCursor(60,10);
    u8g2.print(pos,DEC);
    
    if (reciever){
      u8g2.setCursor(2,20);
      u8g2.print(volume_knob3,DEC);
      u8g2.setCursor(10,20);
      u8g2.print(octave_knob2,DEC);
      u8g2.setCursor(2,30);
      u8g2.print(currentnote);
      u8g2.print(currentsharp);
      u8g2.setCursor(70,10);
      u8g2.print('R');
    }
    //#endif

    if (sender){
      u8g2.setCursor(2,30);
      u8g2.print(octave);
      u8g2.setCursor(70,10);
      u8g2.print('S');
    }
    // u8g2.setCursor(66,30);
    // u8g2.print((char) RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);

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
    3,			/* Task priority */
    &CAN_TX_TaskHandle);
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

  //#ifdef reciever
  fullrangekeyarrayMutex = xSemaphoreCreateMutex();
  //#endif

  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  //#ifdef reciever

  //#endif

  CAN_Init(false);
  #ifndef DISABLE_THREADS
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  setCANFilter(0x123,0x7ff);
  CAN_Start();

  //#ifdef sender
  
  // #endif

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

  Serial.print("ID: ");
  Serial.print(id_0, HEX);
  Serial.print(id_1, HEX);
  Serial.println(id_2, HEX);

  bool outBits[7] = {0,0,0,1,1,1,1};

  for (uint8_t i=0; i<7; i++) {
    setRow(i);                     //Set row address
    digitalWrite(OUT_PIN,outBits[i]); //Set value to latch in DFF
  }
  
  delayMicroseconds(100);

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

  if(reciever){
    Serial.println("Reciever");
  }
  else if(sender){
    Serial.println("Sender");
  }
  else{
    Serial.println("Error");
  }
  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  Serial.print("init west_detect: ");
  Serial.println(west_detect);
  Serial.print("init east_detect: ");
  Serial.println(east_detect);

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