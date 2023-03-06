#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include "knobs.h"
#include <ES_CAN.h>



//Constants
  const uint32_t interval = 100; //Display update interval

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

  // All my global variables
  const uint32_t stepSizes [] = {50953930, 54077542, 57396381, 60715219, 64229283, 68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};
  char letterArray [13] = {'C','C','D','D','E','F','F','G','G','A','A','B',' '};
  char symbolArray [13] = {' ','#',' ','#',' ',' ','#',' ','#',' ','#',' ',' '};
  volatile uint32_t currentStepSize;
  volatile uint8_t keyArray[7];
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t RXMessageMutex;
  int8_t knob3Rotation;     
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  volatile uint8_t RX_Message[8]={0};   
  SemaphoreHandle_t CAN_TX_Semaphore;       


#ifdef receiver
  void CAN_RX_ISR (void) {
    uint8_t RX_Message_ISR[8];
    uint32_t ID;
    CAN_RX(ID, RX_Message_ISR);
    xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
  }
#endif


uint8_t readCols(){
  int c0 = digitalRead(C0_PIN);
  int c1 = digitalRead(C1_PIN);
  int c2 = digitalRead(C2_PIN);
  int c3 = digitalRead(C3_PIN);
  return (c0 << 0) | (c1 << 1) | (c2 << 2) | (c3 << 3); 
}

void setRow (uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, Vout + 128);
} 

int getIndex() {
  int index = 0;
  for (int i = 0; i < 3; i++) {
    uint8_t nibble = keyArray[i] & 0x0F;  // Extract the lowest 4 bits
    for (int j = 0; j <= 3; j++) {
      if ((nibble & (1 << j)) == 0) {
        return index;
      } else {
        index++;
      }
    }
  }
  return index;
}

int getIncrement(uint8_t input){
  switch (input)
  {
    case 0x01: return 1;
    case 0x04: return -1;
    case 0x0B : return -1;
    case 0x0E : return 1;
    case 0x0C : return 1;
    default: return 0;
  }
}

void scanKeysTask(void *pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int8_t knob3RotationLocal = 0;
  Knob Knob3(0,0,8);
  int curIndex = 0;
  int prevIndex = 0;
  uint8_t TX_Message[8] = {0};
  while (1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      for (uint8_t i = 0 ; i<4; i++) {
        setRow(i);
        delayMicroseconds(3);
        uint8_t keys = readCols();
        keyArray[i] = keys;
      }
      uint32_t localCurrentStepSize = 0;
      Knob3.UpdateRotateVal(keyArray[3] & 0x03);
      Knob3.SetLimits(0,8);
      knob3RotationLocal =Knob3.CurRotVal();
      if (keyArray[0]==15 && keyArray[1]==15 && keyArray[2]==15){
        curIndex = 12;
        if (prevIndex != curIndex){
          TX_Message[0]='R';
          TX_Message[1]= 4;
          TX_Message[2]= prevIndex;
        }
      }
      else{
        curIndex = getIndex();
        localCurrentStepSize = stepSizes[curIndex];
        if (prevIndex != curIndex){
          TX_Message[0]='P';
          TX_Message[1]= 4;
          TX_Message[2]= curIndex;
        }
      }
      xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
      prevIndex = curIndex;
      xSemaphoreGive(keyArrayMutex);
      __atomic_store_n(&knob3Rotation, knob3RotationLocal, __ATOMIC_RELAXED);
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

  }
}

void displayUpdateTask(void *pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  while (1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //Update display

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[0],HEX);
    u8g2.print(keyArray[1],HEX);
    u8g2.print(keyArray[2],HEX);
    u8g2.setCursor(20,30);
    u8g2.print(knob3Rotation,HEX);
    xSemaphoreGive(keyArrayMutex);
    u8g2.setCursor(66,30);
    xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
    xSemaphoreGive(RXMessageMutex);
    u8g2.setCursor(2,30);
    int index = getIndex();
    u8g2.print(letterArray[index]);
    u8g2.print(symbolArray[index]);
    u8g2.sendBuffer();          // transfer internal memory to the display
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

#ifdef receiver
  void decodeTask (void *pvParameters){
    uint32_t LocalcurrentStepSize;
    uint8_t LocalRX_Message[8]={0};
    while (1) {
      xQueueReceive(msgInQ, LocalRX_Message, portMAX_DELAY);
      if (LocalRX_Message[0]=='R'){
        LocalcurrentStepSize=0;
      }
      else {
        LocalcurrentStepSize =(stepSizes[LocalRX_Message[2]]) << (LocalRX_Message[1]-4);
      }
      xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
        for (int i=0 ; i<8 ; i++){
          RX_Message[i]=LocalRX_Message[i];
        }
      xSemaphoreGive(RXMessageMutex);
      
      __atomic_store_n(&currentStepSize, LocalcurrentStepSize, __ATOMIC_RELAXED);

    }
  }
#endif

#ifdef sender
void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
	xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
#endif



void setup() {
  // put your setup code here, to run once:
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &scanKeysHandle );

#ifdef sender
  TaskHandle_t CAN_TX_Handle = NULL;
  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX_Taskk",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &CAN_TX_Handle );
#endif

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdateTask",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );

#ifdef receiver
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decodeTask",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &decodeHandle );
#endif


  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  #ifdef receiver
    CAN_RegisterRX_ISR(CAN_RX_ISR);
  #endif
  #ifdef sender
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
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

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  keyArrayMutex = xSemaphoreCreateMutex();
  RXMessageMutex = xSemaphoreCreateMutex();
  vTaskStartScheduler();
}



void loop() {
  
}