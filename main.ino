#include "variant.h"
#include <due_can.h>

#define CAN_BAUD_RATE             CAN_BPS_250K     // need to set both mc & bms to the right baudrate
#define NDRIVE_RXID               0x210
#define NDRIVE_TXID               0x180
#define MAX_CAN_FRAME_DATA_LEN    8                // CAN frame max data length
#define BIT_CONVERSION_CONSTANT   (3.3/4095)       // ADC
#define TEST1_CAN0_TX_PRIO        15
#define CAN_MSG_DUMMY_DATA        0x11BFFA4E

CAN_FRAME frame1, frame2, incoming, BMS_frame1, BMS_incoming;
CAN_FRAME frame_n_actual, frame_torque_cmd;

float CHANNEL_0_REG = 0;
float CHANNEL_1_REG = 0;
float CHANNEL_2_REG = 0;
float CHANNEL_3_REG = 0;
float CHANNEL_4_REG = 0;
float CHANNEL_5_REG = 0;
float CHANNEL_6_REG = 0;
float CHANNEL_7_REG = 0;
float CHANNEL_8_REG = 0;
float CHANNEL_9_REG = 0;

uint32_t sentFrames, receivedFrames;

//Leave this defined if you use the native port or comment it out if you use the programming port
//#define Serial SerialUSB

void adc_setup(void)
{
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, 8);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_set_resolution(ADC, ADC_12_BITS);
  
  // Enable ADC channels arrange by arduino pins from A0 to A9
  adc_enable_channel(ADC, ADC_CHANNEL_7);
  adc_enable_channel(ADC, ADC_CHANNEL_6);
  adc_enable_channel(ADC, ADC_CHANNEL_5);
  adc_enable_channel(ADC, ADC_CHANNEL_4);
  adc_enable_channel(ADC, ADC_CHANNEL_3);
  adc_enable_channel(ADC, ADC_CHANNEL_2);
  adc_enable_channel(ADC, ADC_CHANNEL_1);
  adc_enable_channel(ADC, ADC_CHANNEL_0);
  adc_enable_channel(ADC, ADC_CHANNEL_10);
  adc_enable_channel(ADC, ADC_CHANNEL_11);
  
  // Enable ADC interrupt
  adc_enable_interrupt(ADC, ADC_IER_EOC7); //EOC9 so that interrupt triggered when analogue input channerl 9 has reached end of conversion
  
  // Trigger configuration
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
  
  // Enable ADC interrupt
  NVIC_EnableIRQ(ADC_IRQn);
  
  //start ADC conversion, note that ADC conversion has to be restarted once conversion is finished
  adc_start(ADC);
}

void ADC_Handler(void)
{
  // Check the ADC conversion status
  if ((adc_get_status(ADC) & ADC_ISR_EOC7) == ADC_ISR_EOC7)
  {
     //Get digital data value from ADC channels and can be used by application
     CHANNEL_0_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_0))*BIT_CONVERSION_CONSTANT;
     CHANNEL_1_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_1))*BIT_CONVERSION_CONSTANT;
     CHANNEL_2_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_2))*BIT_CONVERSION_CONSTANT;
     CHANNEL_3_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_3))*BIT_CONVERSION_CONSTANT;
     CHANNEL_4_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_4))*BIT_CONVERSION_CONSTANT;
     CHANNEL_5_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_5))*BIT_CONVERSION_CONSTANT;
     CHANNEL_6_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_6))*BIT_CONVERSION_CONSTANT;
     CHANNEL_7_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_7))*BIT_CONVERSION_CONSTANT;
     CHANNEL_8_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_10))*BIT_CONVERSION_CONSTANT; //notice that its channel 10
     CHANNEL_9_REG = ((float)adc_get_channel_value(ADC, ADC_CHANNEL_11))*BIT_CONVERSION_CONSTANT; //notice that its channel 11
  }     
  adc_start(ADC);
}

void printFrame_from_MC(CAN_FRAME &frame) {
  Serial.print("ID: 0x");
  Serial.print(frame.id, HEX);
  Serial.print(" Len: ");
  Serial.print(frame.length);
  Serial.print(" Data: 0x");
  for (int count = 0; count < frame.length; count++) {
    Serial.print(frame.data.bytes[count], HEX);
    Serial.print(" ");
  }
  Serial.print("\r\n");
}

void printFrame_from_BMS(CAN_FRAME &frame) {
   Serial.print("ID: 0x");
   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.length; count++) {
       Serial.print(frame.data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

//parses the standard frame sent by BMS every 1 second
void parseFrame_from_BMS(CAN_FRAME &frame){
  switch (frame.id) {
      //state
      case 0x622:
        Serial.print("State = ");
        Serial.print(frame.data.bytes[0],HEX);
        Serial.print("\n");
        break;
      //pack voltage
      case 0x623:
        Serial.print("Pack Voltage = ");
        Serial.print(frame.data.bytes[0],HEX);
        Serial.print(frame.data.bytes[1],HEX);
        Serial.print("V");
        Serial.print("\n");
        break;
      //current
      case 0x624:
        Serial.print("Current = ");
        Serial.print(frame.data.bytes[0],HEX);
        Serial.print(frame.data.bytes[1],HEX);
        Serial.print("A");
        Serial.print("\n");
        break;
      //SOC
      case 0x626:
        Serial.print("SOC = ");
        Serial.print(frame.data.bytes[0]);
        Serial.print("%");
        Serial.print("\n");
        break;
      //temperature
      case 0x627:
        Serial.print("Temperature = ");
        Serial.print(frame.data.bytes[0],HEX);
        Serial.print("C");
        Serial.print("\n");
        break;
      default:
        // do something
        break;
  }
}

void populate_frames() {                         // Frames to request for data from MC
  frame_n_actual.id = NDRIVE_RXID;
  frame_n_actual.length = 3;
  //frame_n_actual.data.low = 0x0064303d;
  //frame_n_actual.data.high = 0;
  frame_n_actual.data.bytes[0] = 0x3d;
  frame_n_actual.data.bytes[1] = 0x30;
  frame_n_actual.data.bytes[2] = 0x64;
  frame_n_actual.extended = 0;

  frame_torque_cmd.id = NDRIVE_RXID;             // Request for Torque data
  frame_torque_cmd.length = 3;
  frame_torque_cmd.data.low = 0x0064903d;
  frame_torque_cmd.data.high = 0;
  frame_torque_cmd.extended = 0;
}

void abort_all_requests() {
  CAN_FRAME frame_abort, incoming;
  uint32_t counter = 0;

  frame_abort.id = NDRIVE_RXID;
  frame_abort.length = 3;
  frame_abort.data.bytes[0] = 0x3d;
  frame_abort.data.bytes[1] = 0x00; // REGID
  frame_abort.data.bytes[2] = 0xff;

  frame_abort.data.bytes[1] = 0x30;
  CAN.sendFrame(frame_abort);
  delayMicroseconds(100);

  frame_abort.data.bytes[1] = 0x90;
  CAN.sendFrame(frame_abort);
  delayMicroseconds(100);

  /*
  while (counter < 5000) {
    if (CAN.rx_avail()) {
      CAN.get_rx_buff(incoming);
      
      if (incoming.id == NDRIVE_TXID) {
        frame_abort.data.bytes[1] = incoming.data.bytes[0];
        CAN.sendFrame(frame_abort);
        delayMicroseconds(100);
      }
      
      counter = 0;
    } else {
      counter++;
    }
  }
  */
}

/*
bool has_received_data(uint8_t data_address) {
  if (CAN.rx_avail()) {
    CAN.get_rx_buff(incoming);

    if (incoming.id == NDRIVE_TXID && incoming.data[0] == data_address) {
      return true;
    }

    delayMicroseconds(100);
  }
  
  return false;
}
*/

void setup()
{
  // start serial port at 115200 bps:
  Serial.begin(115200);      

  if (CAN.init(CAN_BAUD_RATE)&&CAN2.init(CAN_BAUD_RATE)) {
  } else {
    Serial.println("CAN initialization (sync) ERROR");
  }

  //sets the receiving filter to only receive from 0x622 to 0x630
  int filter;
  //extended

  /* CAN 1: MOTOR CONTROLLER COMMUNICATION */
  //Both of these lines create a filter on the corresponding CAN device that allows
  //just the one ID we're interested in to get through.
  //The syntax is (mailbox #, ID, mask, extended)
  //You can also leave off the mailbox number: (ID, mask, extended)
  CAN.setRXFilter(0, NDRIVE_TXID, 0x1FFFFFFF, false);
  populate_frames();

  abort_all_requests();

  /* CAN 2: BMS COMMUNICATION */
  for (filter = 0; filter < 3; filter++) {
  CAN2.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
  CAN2.setRXFilter(filter, 0x622, 0x1FFFFFF0, false);
  }  

  // frame for PID request requesting PID(F5)
  // BMS_frame1.id = 0x07E0;
  // BMS_frame1.extended = 0;
  // BMS_frame1.length = 8;
  // BMS_frame1.data.bytes[0] = 0x02;
  // BMS_frame1.data.bytes[1] = 0x21;
  // BMS_frame1.data.bytes[2] = 0xF5;
  // BMS_frame1.data.bytes[3] = 0x00;
  // BMS_frame1.data.bytes[4] = 0x00;
  // BMS_frame1.data.bytes[5] = 0x00;
  // BMS_frame1.data.bytes[6] = 0x00;
  // BMS_frame1.data.bytes[7] = 0x00;

  // CAN.sendFrame(BMS_frame1);
  // CAN2.sendFrame(BMS_frame1);
  // Serial.println("Frame Sent!");
  // printFrame_from_BMS(BMS_frame1);
}
/*
// Test rapid fire ping/pong of extended frames
static void test_1(void)
{
  CAN_FRAME inFrame;
  uint32_t counter = 0;
        
  // Send out the first frame
  CAN.sendFrame(frame_n_actual);
  sentFrames++;

  while (1==1) {
    if (CAN.rx_avail()) {
      CAN.get_rx_buff(incoming);
      CAN.sendFrame(frame_n_actual);
      delayMicroseconds(100);
      sentFrames++;
      receivedFrames++;
      counter++;
    }
    if (CAN2.rx_avail()) {
      CAN2.get_rx_buff(incoming);
      CAN2.sendFrame(frame_n_actual);
      delayMicroseconds(100);
      sentFrames++;
      receivedFrames++;
      counter++;
    }
    if (counter > 5000) {
      counter = 0;
      Serial.print("S: ");
      Serial.print(sentFrames);
      Serial.print(" R: ");
      Serial.println(receivedFrames);
    }
  }
}*/


void loop()
{
    CAN_FRAME incoming, test_frame, test_frame_2, test_frame_3,BMS_incoming;

    test_frame.id = NDRIVE_RXID;
    test_frame.length = 3;
    test_frame.data.bytes[0] = 0x3d;
    test_frame.data.bytes[1] = 0x30;
    test_frame.data.bytes[2] = 0x64;
    test_frame.extended = 0;

    test_frame_2.id = NDRIVE_RXID;
    test_frame_2.length = 3;
    test_frame_2.data.bytes[0] = 0x3d;
    test_frame_2.data.bytes[1] = 0x90;
    test_frame_2.data.bytes[2] = 0x64;
    test_frame_2.data.high = 0;
    test_frame_2.extended = 0;

    test_frame_3.id = NDRIVE_RXID;
    test_frame_3.length = 3;
    test_frame_3.data.bytes[0] = 0x90;
    test_frame_3.data.bytes[1] = 0xFC;
    test_frame_3.data.bytes[2] = 0x3F;

    //CAN.sendFrame(test_frame);
    //delayMicroseconds(100);
    //CAN.sendFrame(test_frame_2);
    //delayMicroseconds(100);
    uint8_t counter = 0;
    while (1) {

    // retrieves pedal input
    float reading = CHANNEL_0_REG;                // read values 

    test_frame_3.data.bytes[1] = reading & 0xff;  // write back to NDRIVE 
    test_frame_3.data.bytes[2] = (reading >> 8) & 0xff;
    Serial.print(test_frame_3.data.bytes[1], HEX);
    Serial.print(" ");
    Serial.print(test_frame_3.data.bytes[2], HEX);
    Serial.print(" ");
    Serial.println(reading, HEX);

    printFrame_from_MC(test_frame_3);
    delayMicroseconds(100);
    CAN.sendFrame(test_frame_3);
    delayMicroseconds(100);
    delay(1000);

    if (CAN.rx_avail()) {                         // MOTOR CONTROLLER READINGS
      CAN.get_rx_buff(incoming); 
      printFrame_from_MC(incoming);
    }
    if (CAN2.rx_avail()) {                        // BMS READINGS 
      CAN2.get_rx_buff(BMS_incoming);
      // Serial.print("CAN1 = "); 
      // printFrame(incoming);
      parseFrame_from_BMS(BMS_incoming);
    }
  }
}