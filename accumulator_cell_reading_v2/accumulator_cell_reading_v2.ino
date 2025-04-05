#include "LTC681x.h"
#include "LTC6813.h"
#include "esp_log.h"
#include "bms_hardware.h"
#include "Preferences.h"
#include <CAN.h> // Requires ESP32 by Espressif Systems version 2.0.17

// Constants
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2

// Battery cell voltage lookup table for state of charge calculation
const float VOLTAGES[] = {
    4.087373015, 4.085084739, 4.057627183, 4.030169509, 4.011864471,
    3.999661035, 3.981355997, 3.963050843, 3.905084812, 3.828813626,
    3.767796561, 3.709830298, 3.621355909, 3.557287927, 3.487118692,
    3.410847507, 3.334576321, 3.221694711, 3.081355892, 3.0};

// Corresponding battery state of charge percentage
const float PERCENTAGE_CAPACITY[] = {
    1.0, 0.9992045768, 0.9896654491, 0.9770320806, 0.9621314476,
    0.9465178241, 0.9270601624, 0.8759270414, 0.7826139264, 0.6959850776,
    0.6184986922, 0.5371149754, 0.4418232198, 0.365716581, 0.2868030144,
    0.2013033494, 0.1342668398, 0.07262271074, 0.02309610172, 0.0};

// Configuration
const uint8_t TOTAL_IC = 4;                 // Number of ICs in the daisy chain
const uint16_t MEASUREMENT_LOOP_TIME = 500; // Loop time in milliseconds

// ADC Configuration
const uint8_t ADC_OPT = ADC_OPT_DISABLED;         // ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; // ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED;             // Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;   // Cell channels to convert
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;     // Auxiliary channels to convert
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;   // Status channels to convert
const uint8_t SEL_ALL_REG = REG_ALL;              // Select all registers
const uint8_t SEL_REG_A = REG_1;                  // Select register 1
const uint8_t SEL_REG_B = REG_2;                  // Select register 2

// Voltage thresholds
const uint16_t OV_THRESHOLD = 41000; // Over voltage threshold (4.1V)
const uint16_t UV_THRESHOLD = 30000; // Under voltage threshold (3.0V)

// Measurement loop configuration
const uint8_t WRITE_CONFIG = DISABLED; // Write to config registers in loop
const uint8_t READ_CONFIG = DISABLED;  // Read config registers in loop
const uint8_t MEASURE_CELL = ENABLED;  // Measure cell voltages in loop
const uint8_t MEASURE_AUX = DISABLED;  // Read auxiliary registers in loop
const uint8_t MEASURE_STAT = DISABLED; // Read status registers in loop
const uint8_t PRINT_PEC = DISABLED;    // Print PEC Error Count in loop

// CAN bus pins
const uint8_t RX_NEW_ESP = 22;
const uint8_t TX_NEW_ESP = 21;

// CAN message IDs
const uint16_t CAN_ID_BMS_STATUS = 0x2FF;
const uint16_t CAN_ID_CELL_MIN_MAX = 0x300;
const uint16_t CAN_ID_SOC = 0x301;
const uint16_t CAN_ID_ERROR = 0x302;

// Global variables
cell_asic BMS_IC[TOTAL_IC];
uint8_t iter = 0;
bool err = false;
char *err_info;
Preferences preferences;

// LTC6813 Configuration
bool REFON = true;                                     // Reference Powered Up Bit
bool ADCOPT = false;                                   // ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true, true}; // GPIO Pin Control 1-5
bool GPIOBITS_B[4] = {false, false, false, false};     // GPIO Pin Control 6-9
uint16_t UV = UV_THRESHOLD;                            // Under voltage threshold
uint16_t OV = OV_THRESHOLD;                            // Over voltage threshold
bool DCCBITS_A[12] = {false, false, false, false, false, false,
                      false, false, false, false, false, false};       // Discharge cells 1-12
bool DCCBITS_B[7] = {false, false, false, false, false, false, false}; // Discharge cells 0,13-15
bool DCTOBITS[4] = {true, false, true, false};                         // Discharge time (4 min)
bool FDRF = false;                                                     // Force Digital Redundancy Failure Bit
bool DTMEN = true;                                                     // Enable Discharge Timer Monitor
bool PSBITS[2] = {false, false};                                       // Digital Redundancy Path Selection

/**
 * Calculate battery state of charge percentage based on cell voltage
 */
float calculate_percentage(float cell_voltage)
{
  uint8_t index_low = 0;

  // Find position in voltage table
  for (; index_low < sizeof(VOLTAGES) - 1; ++index_low)
  {
    if (VOLTAGES[index_low] < cell_voltage)
    {
      break;
    }
  }

  // Linear interpolation between points
  return PERCENTAGE_CAPACITY[index_low] +
         (cell_voltage - VOLTAGES[index_low]) *
             (PERCENTAGE_CAPACITY[index_low - 1] - PERCENTAGE_CAPACITY[index_low]) /
             (VOLTAGES[index_low - 1] - VOLTAGES[index_low]);
}

/**
 * Print all cell voltages for debugging
 */
void print_cells()
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    Serial.print(", ");

    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
    {
      Serial.print(" C");
      Serial.print(i + 1, DEC);
      Serial.print(":");
      Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
      Serial.print(",");
    }
    Serial.println();
  }
}

/**
 * Check for and handle PEC errors
 */
void check_error(int error)
{
  if (error == -1)
  {
    err = true;
    preferences.putString("err", "PEC Error");
    err_info = "PEC Error";
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/**
 * Handle incoming CAN messages
 */
void receiveData()
{
  int packetSize = CAN.parsePacket();

  if (packetSize)
  {
    Serial.print("Received ");

    if (CAN.packetExtended())
    {
      Serial.print("extended ");
    }

    if (CAN.packetRtr())
    {
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr())
    {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    }
    else
    {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // Print packet data for non-RTR packets
      while (CAN.available())
      {
        Serial.print(CAN.read(), HEX);
      }
      Serial.println();
    }
    Serial.println();
  }
}

/**
 * Send BMS status over CAN bus
 *
 * @param enable BMS enable status
 * @param total_voltage Total battery pack voltage
 * @param min_voltage Minimum cell voltage
 * @param soc State of charge (0-100%)
 */
void sendBmsStatus(bool enable, float total_voltage, float min_voltage, float soc)
{
  // Convert values to integers
  uint16_t total_volts_mV = (uint16_t)(total_voltage * 1000); // Convert to millivolts
  uint16_t min_volts_mV = (uint16_t)(min_voltage * 10000);    // Convert to tenths of millivolts
  uint16_t soc_int = (uint16_t)(soc * 10);                    // One decimal place precision

  Serial.println("Sending BMS status packet");

  CAN.beginPacket(CAN_ID_BMS_STATUS);
  CAN.write(enable ? 0x01 : 0x00);         // Enable status
  CAN.write(total_volts_mV & 0xFF);        // Total voltage LSB
  CAN.write((total_volts_mV >> 8) & 0xFF); // Total voltage MSB
  CAN.write(min_volts_mV & 0xFF);          // Min cell voltage LSB
  CAN.write((min_volts_mV >> 8) & 0xFF);   // Min cell voltage MSB
  CAN.write(soc_int & 0xFF);               // SOC LSB
  CAN.write((soc_int >> 8) & 0xFF);        // SOC MSB
  CAN.write(err ? 0x01 : 0x00);            // Error flag

  if (CAN.endPacket())
  {
    Serial.println("BMS status sent successfully");
  }
}

/**
 * Send cell min/max information over CAN
 */
void sendCellMinMax(float min_cell, float max_cell, uint8_t min_cell_index, uint8_t max_cell_index)
{
  uint16_t min_mV = (uint16_t)(min_cell * 10000); // Convert to tenths of millivolts
  uint16_t max_mV = (uint16_t)(max_cell * 10000); // Convert to tenths of millivolts

  CAN.beginPacket(CAN_ID_CELL_MIN_MAX);
  CAN.write(min_mV & 0xFF);        // Min cell voltage LSB
  CAN.write((min_mV >> 8) & 0xFF); // Min cell voltage MSB
  CAN.write(min_cell_index);       // Min cell index
  CAN.write(max_mV & 0xFF);        // Max cell voltage LSB
  CAN.write((max_mV >> 8) & 0xFF); // Max cell voltage MSB
  CAN.write(max_cell_index);       // Max cell index
  CAN.write(0x00);                 // Reserved
  CAN.write(0x00);                 // Reserved

  if (CAN.endPacket())
  {
    Serial.println("Cell min/max data sent");
  }
}

/**
 * Send error status over CAN
 */
void sendErrorStatus(bool has_error, uint8_t error_type)
{
  CAN.beginPacket(CAN_ID_ERROR);
  CAN.write(has_error ? 0x01 : 0x00); // Error flag
  CAN.write(error_type);              // Error type: 1=PEC, 2=OV, 3=UV
  CAN.write(0x00);                    // Reserved
  CAN.write(0x00);                    // Reserved
  CAN.write(0x00);                    // Reserved
  CAN.write(0x00);                    // Reserved
  CAN.write(0x00);                    // Reserved
  CAN.write(0x00);                    // Reserved

  if (CAN.endPacket())
  {
    Serial.println("Error status sent");
  }
}

/**
 * Initialize the system
 */
void setup()
{
  Serial.begin(115200);
  pinMode(15, OUTPUT);
  start_spi();
  pinMode(12, INPUT_PULLUP);
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  // Initialize preferences
  preferences.begin("my-app", false);

  // Initialize LTC6813
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC, BMS_IC);

  // Configure all ICs
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    LTC6813_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic, BMS_IC, FDRF, DTMEN, PSBITS, GPIOBITS_B, DCCBITS_B);
  }

  // Reset and initialize
  LTC6813_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC, BMS_IC);
  LTC6813_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC, BMS_IC);

  // Wake up ICs and write configuration
  wakeup_sleep(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC, BMS_IC);
  LTC6813_wrcfgb(TOTAL_IC, BMS_IC);

  // Initialize CAN bus at 500 kbps
  // Note: Set bitrate to 1000E3 for 500 kbps because that makes perfect sense!! :) :)
  // (specific to the GOAT Sandeep's library)
  CAN.setPins(RX_NEW_ESP, TX_NEW_ESP);
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ; // Halt if CAN fails
  }

  Serial.println("CAN started successfully");
}

/**
 * Main program loop
 */
void loop()
{
  Serial.println("Running main loop");

  // Listen for CAN messages for 250ms
  uint32_t time = millis();
  while ((millis() - time) < 250)
  {
    receiveData();
  }

  // Wake up and measure cell voltages
  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  LTC6813_pollAdc();
  wakeup_idle(TOTAL_IC);

  // Read cell voltages
  uint8_t error = LTC6813_rdcv(0, TOTAL_IC, BMS_IC);
  check_error(error);

  // Process cell measurements
  iter = (iter + 1) % 4;
  float sum_cells = 0;
  float min_cell = 100;
  float max_cell = 0;
  uint8_t min_cell_index = 0;
  uint8_t max_cell_index = 0;
  uint8_t error_type = 0;

  // Check each cell voltage
  for (uint8_t j = 0; j < TOTAL_IC; ++j)
  {
    for (uint8_t k = 0; k < 7; ++k)
    {
      uint8_t cell_index = j * 7 + k;
      float cell_val = BMS_IC[j].cells.c_codes[k] * 0.0001;

      // Check for over/under voltage conditions
      if (cell_val > 4.15)
      {
        preferences.putString("err", "overvolt");
        err_info = "overvolt";
        err = true;
        error_type = 2; // Overvoltage
      }
      if (cell_val < 3.0)
      {
        preferences.putString("err", "undervolt");
        err_info = "undervolt";
        err = true;
        error_type = 3; // Undervoltage
      }

      sum_cells += cell_val;

      // Track min and max cells
      if (cell_val < min_cell)
      {
        min_cell = cell_val;
        min_cell_index = cell_index;
      }
      if (cell_val > max_cell)
      {
        max_cell = cell_val;
        max_cell_index = cell_index;
      }
    }
  }

  // Set error indicator pin
  if (err)
  {
    digitalWrite(27, LOW);
    if (error == -1)
    {
      error_type = 1; // PEC error
    }
  }
  else
  {
    digitalWrite(27, HIGH);
  }

  // Calculate battery capacity based on cell voltages
  float mean_capacity = calculate_percentage(sum_cells / 28);
  float actual_capacity = calculate_percentage(min_cell);
  float total_voltage = sum_cells;

  // Print diagnostic information every 4th iteration
  if (iter == 0)
  {
    print_cells();

    // Print diagnostic information
    Serial.print("Min Cell: ");
    Serial.print(min_cell);
    Serial.print(" (index ");
    Serial.print(min_cell_index);
    Serial.print("), Max Cell: ");
    Serial.print(max_cell);
    Serial.print(" (index ");
    Serial.print(max_cell_index);
    Serial.print(")");
    Serial.println();

    Serial.print("Total Voltage: ");
    Serial.print(total_voltage);
    Serial.print("V, Actual Capacity: ");
    Serial.print(actual_capacity);
    Serial.print("%, Mean Capacity: ");
    Serial.print(mean_capacity);
    Serial.print("%");
    Serial.println();

    Serial.print("Error: ");
    Serial.print(err);

    if (err == true)
    {
      Serial.print(", ");
      Serial.print(err_info);
    }

    Serial.print(", Stored Error: ");
    char value[10];
    preferences.getString("err", value, 10);
    Serial.print(value);
    Serial.println();
    Serial.println();
  }

  // Send CAN messages with actual BMS data
  sendBmsStatus(!err, total_voltage, min_cell, actual_capacity);

  // Send additional data in different messages
  sendCellMinMax(min_cell, max_cell, min_cell_index, max_cell_index);

  // Send error status if there's an error
  if (err)
  {
    sendErrorStatus(true, error_type);
  }
}