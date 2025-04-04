#include "LTC681x.h"
#include "LTC6813.h"
#include "esp_log.h"
#include "bms_hardware.h"
#include "Preferences.h"

#include <CAN.h> // MAKE SURE TO DOWNGRADE
// Go to Boards Manager -> Downgrade ur esp32 by Espressif Systems to 2.0.17

#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2

const float VOLTAGES[]={
  4.087373015,
  4.085084739,
  4.057627183,
  4.030169509,
  4.011864471,
  3.999661035,
  3.981355997,
  3.963050843,
  3.905084812,
  3.828813626,
  3.767796561,
  3.709830298,
  3.621355909,
  3.557287927,
  3.487118692,
  3.410847507,
  3.334576321,
  3.221694711,
  3.081355892,
  3,
};
const float PERCENTAGE_CAPACITY[]={
  1,
  0.9992045768,
  0.9896654491,
  0.9770320806,
  0.9621314476,
  0.9465178241,
  0.9270601624,
  0.8759270414,
  0.7826139264,
  0.6959850776,
  0.6184986922,
  0.5371149754,
  0.4418232198,
  0.365716581,
  0.2868030144,
  0.2013033494,
  0.1342668398,
  0.07262271074,
  0.02309610172,
  0,
};

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 4;//!< Number of ICs in the daisy chain

uint8_t iter=0;
bool err=false;

/********************************************************************
 ADC Command Configurations. See LTC681x.h for options
*********************************************************************/
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted 
const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection 
const uint8_t SEL_REG_A = REG_1; //!< Register Selection 
const uint8_t SEL_REG_B = REG_2; //!< Register Selection 

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop

// RX and TX for CAN
const uint8_t RX_NEW_ESP = 22;
const uint8_t TX_NEW_ESP = 21;

cell_asic BMS_IC[TOTAL_IC];


bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false,false,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {false,false,false,false}; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV=UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7]= {false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {true,false,true,false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBITS[2]= {false,false}; //!< Digital Redundancy Path Selection//ps-0,1

char* err_info;

Preferences preferences;

void setup() {
  Serial.begin(115200);
  pinMode(15, OUTPUT);
  start_spi();
  pinMode(12, INPUT_PULLUP);
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
  preferences.begin("my-app", false);
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC,BMS_IC);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);   
  }  
  LTC6813_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC,BMS_IC);
  LTC6813_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC,BMS_IC);
  wakeup_sleep(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC,BMS_IC);
  LTC6813_wrcfgb(TOTAL_IC,BMS_IC);


 // start the CAN bus at 500 kbps
  CAN.setPins(RX_NEW_ESP, TX_NEW_ESP);
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  Serial.println("CAN started lol");

}

void print_cells(){
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    Serial.print(", ");
    for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++)
    {
      Serial.print(" C");
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
      Serial.print(",");
    }
    Serial.println();
  }
}
void check_error(int error)
{
  if (error == -1)
  {

    err=true;
    preferences.putString(0, "PEC Error");
    err_info="PEC Error";
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

float calculate_percentage(float cell_voltage){
  uint8_t index_low=0;
  Serial.print("cell voltage: ");
  Serial.println(cell_voltage);
  for(; index_low<sizeof(VOLTAGES)-1; ++index_low){
    if(VOLTAGES[index_low]<cell_voltage){
      break;
    }
  }
  Serial.print("index: ");
  Serial.println(index_low);
  return PERCENTAGE_CAPACITY[index_low]+(cell_voltage-VOLTAGES[index_low])*(PERCENTAGE_CAPACITY[index_low-1]-PERCENTAGE_CAPACITY[index_low])/(VOLTAGES[index_low-1]-VOLTAGES[index_low]);
}

void recieveData() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);
    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print(CAN.read(), HEX);
      }
      Serial.println();
    }

    Serial.println();



  }

  
}


void loop() {
   
  // while(!CAN.begin(500E3)) {
  //   Serial.println("Starting CAN failed!");
  //   delay(100);
  // }
  // put your main code here, to run repeatedly:
  //log_i("made it to loop");
  Serial.println("made it to loop");
  uint32_t time=millis();
  while((millis()-time)<250) {
    recieveData();
  }
  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
  LTC6813_pollAdc();
  wakeup_idle(TOTAL_IC);
  uint8_t error = LTC6813_rdcv(0, TOTAL_IC,BMS_IC);
  check_error(error);
  iter=(iter+1)%4;
  float sum_cells=0;
  float min_cell=100;
  for(uint8_t j=0; j<4; ++j){
    for(uint8_t k=0; k<7; ++k){
      float cell_val=BMS_IC[j].cells.c_codes[k]*0.0001;
      if(cell_val>4.15){
        preferences.putString("err", "overvolt");
        err_info="overvolt";
        err=true;
      }
      if(cell_val<3){
        preferences.putString("err", "undervolt");
        err_info="undervolt";
        err=true;
      }
      sum_cells+=cell_val;
      min_cell=min(min_cell, cell_val);
    }
  }
  if(err){
      digitalWrite(27, LOW);
  } else {
    digitalWrite(27, HIGH);
  }
  if(iter==0){
      print_cells();
      float mean_capacity=calculate_percentage(sum_cells/28);
      float actual_capacity=calculate_percentage(min_cell);
      Serial.print("Min Cell: ");
      Serial.print(min_cell);
      Serial.print(". Actual Capacity: ");
      Serial.print(actual_capacity);
      Serial.print(", Mean Capacity: ");
      Serial.print(mean_capacity);
      Serial.print(", err: ");
      Serial.print(err);
      if(err==true){
        Serial.print(", ");
        Serial.print(err_info);
      }
      Serial.print(", Stored Error: ");
      char * value;
      preferences.getString("err", value, 10);
      Serial.print(value);
      Serial.println();
      Serial.println();
      Serial.println();
  }
  
  Serial.println("startig packet");
  CAN.beginPacket(0x2FF);
  CAN.write(0x01); //Enable
  CAN.write(0xE8); //LSB: 1000
  CAN.write(0x03); //MSB: 1000
  CAN.write(0x8A); //LSB: 1120
  CAN.write(0x04); //MSB: 1120
  CAN.write(0x2C); //LSB: 200
  CAN.write(0x01); //LSB: 200
  CAN.write(0x00); //RESERVED
  CAN.endPacket();
  
}

