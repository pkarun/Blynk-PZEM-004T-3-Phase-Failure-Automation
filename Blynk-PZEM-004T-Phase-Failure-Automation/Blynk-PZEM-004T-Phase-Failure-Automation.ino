/*
   Blynk PZEM 004T v3.0 - 3 Phase Failure Motor Automation

   Source:
   
   https://github.com/pkarun/Blynk-PZEM-004T-3-Phase-Failure-Automation
   https://github.com/pkarun/Blynk-PZEM-004T-v3.0-Multiple-device
   https://github.com/pkarun/Blynk-PZEM-004T-v3.0

   Reference:

    https://community.blynk.cc/t/pzem-004t-v3-0-and-nodemcu-wemos-mini-running-on-blynk-how-to-procedure/39338
    https://asndiy.wordpress.com/2019/03/02/pzem-016-nodemcu-thingspeak/
    http://evertdekker.com/?p=1307
    https://didactronica.com/medidas-electricas-en-corriente-alterna-con-arduino-y-un-solo-dispositivo-pzem-004t/
    http://solar4living.com/pzem-arduino-modbus.htm
    http://www.desert-home.com/2018/07/pzem-016-another-chinese-power-monitor.html
    https://www.youtube.com/watch?v=gJRhfs6A1SA

   Wiring:

   PZEM 004T v3.0 to NodeMCU
   5v to vin
   RX to D6 (has TX Pin)
   TX to D5 (has RX Pin)
   GND to GND

*/

#define BLYNK_PRINT Serial        // Uncomment for debugging 

#include "settings.h"           
//#include "secret.h"               // <<--- UNCOMMENT this before you use and change values on config.h tab
#include "my_secret.h"              // <<--- COMMENT-OUT or REMOVE this line before you use. This is my personal settings.

#include <ArduinoOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>


#include <SoftwareSerial.h>                                   //(NODEMCU ESP8266)
SoftwareSerial pzemSerial(RX_PIN_NODEMCU, TX_PIN_NODEMCU);    //(RX,TX) NodeMCU connect to (TX,RX) of PZEM

/*
    This is the address of Pzem devices on the network. Each pzem device has to set unique
    address when we are working with muliple pzem device (multiple modbus devices/multiple slaves).
    For changing/assigning address to pzem device, goto setup() function. 
*/

static uint8_t pzemSlave1Addr = PZEM_SLAVE_1_ADDRESS;
static uint8_t pzemSlave2Addr = PZEM_SLAVE_2_ADDRESS;
static uint8_t pzemSlave3Addr = PZEM_SLAVE_3_ADDRESS;

unsigned long oldTime = 0;

ModbusMaster node1;
ModbusMaster node2;
ModbusMaster node3;

BlynkTimer timer;

double voltage_usage_1      = 0;
double current_usage_1      = 0;
double active_power_1       = 0;
double active_energy_1      = 0;
double frequency_1          = 0;
double power_factor_1       = 0;
double over_power_alarm_1   = 0;

double voltage_usage_2      = 0;
double current_usage_2      = 0;
double active_power_2       = 0;
double active_energy_2      = 0;
double frequency_2          = 0;
double power_factor_2       = 0;
double over_power_alarm_2   = 0;

double voltage_usage_3      = 0;
double current_usage_3      = 0;
double active_power_3       = 0;
double active_energy_3      = 0;
double frequency_3          = 0;
double power_factor_3       = 0;
double over_power_alarm_3   = 0;

double sum_of_voltage       = 0;
double sum_of_current       = 0;
double sum_of_power         = 0;
double sum_of_active_energy = 0;
double sum_of_frequency     = 0;
double sum_of_power_factor  = 0;

/* Relay */

int relay1State             = HIGH;                 // When power is back, keep relays in OFF condition HIGH is LOW in nodemcu
int pushButton1State        = HIGH;

int relay2State             = HIGH;
int pushButton2State        = HIGH;

int relay3State             = HIGH;
//int pushButton3State      = HIGH;

int relay4State             = HIGH;
//int pushButton4State      = HIGH;

int auto_mode_state_1       = HIGH;

int ReCnctFlag;               // Reconnection Flag
int ReCnctCount = 0;          // Reconnection counter

void setup()
{
  Serial.begin(115200);
  pzemSerial.begin(9600);

  /* start Modbus/RS-485 serial communication */

  node1.begin(pzemSlave1Addr, pzemSerial);
  node2.begin(pzemSlave2Addr, pzemSerial);
  node3.begin(pzemSlave3Addr, pzemSerial);

  /*********************************************************************************************\
      Change PZEM address
  \*********************************************************************************************/

  /*
      changeAddress(OldAddress, Newaddress)
      By Uncomment the function in the below line you can change the slave address from one of the nodes (pzem device),
      only need to be done ones. Preverable do this only with 1 slave in the network.
      If you forgot or don't know the new address anymore, you can use the broadcast address 0XF8 as OldAddress to change the slave address.
      Use this with one slave ONLY in the network.
      This is the first step you have to do when connecting muliple pzem devices. If you haven't set the pzem address, then this program won't
      works.
     1. Connect only one PZEM device to nodemcu and powerup your PZEM
     2. uncomment the changeAddress function call below i.e., changeAddress(OldAddress, Newaddress)
     3. change the Newaddress value to some other value. Ex: 0x01, 0x02, 0xF7 etc.,
     4. upload this program to nodemcu 
     5. if you see "Changing Slave Address" on serial monitor, then it successfully changed address 
     6. if you don't see that message, then click on RESET button on nodemcu
     7. Once this done you have successfully assigned address to pzem device.
     8. do the same steps for as many devices as you want. 
  */


// changeAddress(0XF8, 0x02);                 // uncomment to set pzem address


  /*********************************************************************************************\
      RESET PZEM Energy
  \*********************************************************************************************/

  /*
        By Uncomment the function in the below line you can reset the energy counter (Wh) back to zero from one of the slaves.
        resetEnergy(pzemSlaveAddr);
  */


// resetEnergy(0x01);                        // uncomment to reset pzem energy


#if defined(USE_LOCAL_SERVER)

  WiFi.begin(WIFI_SSID, WIFI_PASS);         // Non-blocking if no WiFi available
  Blynk.config(AUTH, SERVER, PORT);
  Blynk.connect();
#else
  WiFi.begin(WIFI_SSID, WIFI_PASS);         // Non-blocking if no WiFi available
  Blynk.config(AUTH);
  Blynk.connect();
#endif   
  
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();

  /*********************************************************************************************\
      RELAY code
  \*********************************************************************************************/

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_1, relay1State);


  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(PUSH_BUTTON_2, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_2, relay2State);

  pinMode(RELAY_PIN_3, OUTPUT);
  //  pinMode(PUSH_BUTTON_3, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_3, relay3State);

  pinMode(RELAY_PIN_4, OUTPUT);
  //  pinMode(PUSH_BUTTON_4, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_4, relay4State);

  timer.setInterval(GET_PZEM_DATA_TIME,       get_pzem_data);                   // How often you would like to call the function
  timer.setInterval(AUTO_MODE_TIME,           auto_mode);    
  timer.setInterval(PHYSICAL_BUTTON_TIME,     checkPhysicalButton);             // Setup a Relay function to be called every 100 ms
  timer.setInterval(SEND_TO_BLYNK_TIME,       sendtoBlynk);                     // Send PZEM values blynk server every 10 sec
} 

void sendtoBlynk()                                                           // Here we are sending PZEM data to blynk
{
  Blynk.virtualWrite(vPIN_VOLTAGE_1,               voltage_usage_1);
  Blynk.virtualWrite(vPIN_CURRENT_USAGE_1,         current_usage_1);
  Blynk.virtualWrite(vPIN_ACTIVE_POWER_1,          active_power_1);
  Blynk.virtualWrite(vPIN_ACTIVE_ENERGY_1,         active_energy_1);
  Blynk.virtualWrite(vPIN_FREQUENCY_1,             frequency_1);
  Blynk.virtualWrite(vPIN_POWER_FACTOR_1,          power_factor_1);
  Blynk.virtualWrite(vPIN_OVER_POWER_ALARM_1,      over_power_alarm_1);

  Blynk.virtualWrite(vPIN_VOLTAGE_2,               voltage_usage_2);
  Blynk.virtualWrite(vPIN_CURRENT_USAGE_2,         current_usage_2);
  Blynk.virtualWrite(vPIN_ACTIVE_POWER_2,          active_power_2);
  Blynk.virtualWrite(vPIN_ACTIVE_ENERGY_2,         active_energy_2);
  Blynk.virtualWrite(vPIN_FREQUENCY_2,             frequency_2);
  Blynk.virtualWrite(vPIN_POWER_FACTOR_2,          power_factor_2);
  Blynk.virtualWrite(vPIN_OVER_POWER_ALARM_2,      over_power_alarm_2);

  Blynk.virtualWrite(vPIN_VOLTAGE_3,               voltage_usage_3);
  Blynk.virtualWrite(vPIN_CURRENT_USAGE_3,         current_usage_3);
  Blynk.virtualWrite(vPIN_ACTIVE_POWER_3,          active_power_3);
  Blynk.virtualWrite(vPIN_ACTIVE_ENERGY_3,         active_energy_3);
  Blynk.virtualWrite(vPIN_FREQUENCY_3,             frequency_3);
  Blynk.virtualWrite(vPIN_POWER_FACTOR_3,          power_factor_3);
  Blynk.virtualWrite(vPIN_OVER_POWER_ALARM_3,      over_power_alarm_3);

  Blynk.virtualWrite(vPIN_SUM_VOLTAGE,             sum_of_voltage);
  Blynk.virtualWrite(vPIN_SUM_CURRENT_USAGE,       sum_of_current);
  Blynk.virtualWrite(vPIN_SUM_ACTIVE_POWER,        sum_of_power);
  Blynk.virtualWrite(vPIN_SUM_ACTIVE_ENERGY,       sum_of_active_energy);
  Blynk.virtualWrite(vPIN_SUM_FREQUENCY,           sum_of_frequency);
  Blynk.virtualWrite(vPIN_SUM_POWER_FACTOR,        sum_of_power_factor);
  }

void pzemdevice1()                                                            // Function to get PZEM device 1 data
{
  Serial.println("====================================================");     // PZEM Device 1 data fetching code starts here
  Serial.println("Now checking PZEM Device 1");
  uint8_t result1;

  ESP.wdtDisable();                                                           // Disable watchdog during modbus read or else ESP crashes when no slave connected
  result1 = node1.readInputRegisters(0x0000, 10);
  ESP.wdtEnable(1);                                                           // Enable watchdog during modbus read

  if (result1 == node1.ku8MBSuccess)
  {
    voltage_usage_1      = (node1.getResponseBuffer(0x00) / 10.0f);
    current_usage_1      = (node1.getResponseBuffer(0x01) / 1000.000f);
    active_power_1       = (node1.getResponseBuffer(0x03) / 10.0f);
    active_energy_1      = (node1.getResponseBuffer(0x05) / 1000.0f);
    frequency_1          = (node1.getResponseBuffer(0x07) / 10.0f);
    power_factor_1       = (node1.getResponseBuffer(0x08) / 100.0f);
    over_power_alarm_1   = (node1.getResponseBuffer(0x09));

    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_1);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_1, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_1);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_1, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_1);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_1);
    Serial.print("OVER_POWER_ALARM:  ");   Serial.println(over_power_alarm_1, 0);
    Serial.println("====================================================");
  }
  else {
    Serial.println("Failed to read PZEM Device 1");
    Serial.println("PZEM Device 1 Data");
    voltage_usage_1      = 0;
    current_usage_1      = 0;
    active_power_1       = 0;
    active_energy_1      = 0;
    frequency_1          = 0;
    power_factor_1       = 0;
    over_power_alarm_1   = 0;
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_1);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_1, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_1);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_1, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_1);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_1);
    Serial.print("OVER_POWER_ALARM:  ");   Serial.println(over_power_alarm_1, 0);
    Serial.println("====================================================");
    swith_off();                                                                  // Calling swith_off() to turn off relays
  }
}

void pzemdevice2()                                                                // Function to get PZEM device 2 data
{
  Serial.println("====================================================");
  Serial.println("Now checking PZEM Device 2");
  uint8_t result2;

  ESP.wdtDisable();
  result2 = node2.readInputRegisters(0x0000, 10);
  ESP.wdtEnable(1);

  if (result2 == node2.ku8MBSuccess)
  {
    voltage_usage_2      = (node2.getResponseBuffer(0x00) / 10.0f);
    current_usage_2      = (node2.getResponseBuffer(0x01) / 1000.000f);
    active_power_2       = (node2.getResponseBuffer(0x03) / 10.0f);
    active_energy_2      = (node2.getResponseBuffer(0x05) / 1000.0f);
    frequency_2          = (node2.getResponseBuffer(0x07) / 10.0f);
    power_factor_2       = (node2.getResponseBuffer(0x08) / 100.0f);
    over_power_alarm_2   = (node2.getResponseBuffer(0x09));

    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_2);         // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_2, 3);      // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_2);          // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_2, 3);      // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_2);             // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_2);
    Serial.print("OVER_POWER_ALARM:  ");   Serial.println(over_power_alarm_2, 0);
    Serial.println("====================================================");
    
  }
    else {
    Serial.println("Failed to read PZEM Device 2");
    Serial.println("PZEM Device 2 Data");
    voltage_usage_2      = 0;
    current_usage_2      = 0;
    active_power_2       = 0;
    active_energy_2      = 0;
    frequency_2          = 0;
    power_factor_2       = 0;
    over_power_alarm_2   = 0;
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_2);         // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_2, 3);      // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_2);          // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_2, 3);      // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_2);             // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_2);
    Serial.print("OVER_POWER_ALARM:  ");   Serial.println(over_power_alarm_2, 0);
    Serial.println("====================================================");
    swith_off(); 
  }
}

void pzemdevice3()                                                            // Function to get PZEM device 1 data
{
  Serial.println("====================================================");     // PZEM Device 1 data fetching code starts here
  Serial.println("Now checking PZEM Device 3");
  uint8_t result3;

  ESP.wdtDisable();                                                           // Disable watchdog during modbus read or else ESP crashes when no slave connected
  result3 = node3.readInputRegisters(0x0000, 10);
  ESP.wdtEnable(1);                                                           // Enable watchdog during modbus read

  if (result3 == node3.ku8MBSuccess)
  {
    voltage_usage_3      = (node3.getResponseBuffer(0x00) / 10.0f);
    current_usage_3      = (node3.getResponseBuffer(0x01) / 1000.000f);
    active_power_3       = (node3.getResponseBuffer(0x03) / 10.0f);
    active_energy_3      = (node3.getResponseBuffer(0x05) / 1000.0f);
    frequency_3          = (node3.getResponseBuffer(0x07) / 10.0f);
    power_factor_3       = (node3.getResponseBuffer(0x08) / 100.0f);
    over_power_alarm_3   = (node3.getResponseBuffer(0x09));

    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_3);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_3, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_3);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_3, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_3);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_3);
    Serial.print("OVER_POWER_ALARM:  ");   Serial.println(over_power_alarm_3, 0);
    Serial.println("====================================================");
    
  }
  else {
    Serial.println("Failed to read PZEM Device 3");
    Serial.println("PZEM Device 3 Data");
    voltage_usage_3      = 0;                                                     // Assigning 0 if it fails to read PZEM device
    current_usage_3      = 0;
    active_power_3       = 0;
    active_energy_3      = 0;
    frequency_3          = 0;
    power_factor_3       = 0;
    over_power_alarm_3   = 0;
    Serial.print("VOLTAGE:           ");   Serial.println(voltage_usage_3);       // V
    Serial.print("CURRENT_USAGE:     ");   Serial.println(current_usage_3, 3);    // A
    Serial.print("ACTIVE_POWER:      ");   Serial.println(active_power_3);        // W
    Serial.print("ACTIVE_ENERGY:     ");   Serial.println(active_energy_3, 3);    // kWh
    Serial.print("FREQUENCY:         ");   Serial.println(frequency_3);           // Hz
    Serial.print("POWER_FACTOR:      ");   Serial.println(power_factor_3);
    Serial.print("OVER_POWER_ALARM:  ");   Serial.println(over_power_alarm_3, 0);
    Serial.println("====================================================");
    swith_off();
  }
}

void sumofpzem()
{
    Serial.println("Sum of all 3 PZEM devices");
    sum_of_voltage        =   (voltage_usage_1 + voltage_usage_2 + voltage_usage_3);
    sum_of_current        =   (current_usage_1 + current_usage_2 + current_usage_3);
    sum_of_power          =   (active_power_1 + active_power_2 + active_power_3);
    sum_of_active_energy  =   (active_energy_1 + active_energy_2 + active_energy_3); 
    sum_of_frequency      =   (frequency_1 + frequency_2 + frequency_3);
    sum_of_power_factor   =   (power_factor_1 + power_factor_2 + power_factor_3); 
    
    Serial.print("SUM of VOLTAGE:           ");   Serial.println(sum_of_voltage);             // V
    Serial.print("SUM of CURRENT_USAGE:     ");   Serial.println(sum_of_current, 3);          // A
    Serial.print("SUM of ACTIVE_POWER:      ");   Serial.println(sum_of_power);               // W
    Serial.print("SUM of ACTIVE_ENERGY:     ");   Serial.println(sum_of_active_energy, 3);    // kWh
    Serial.print("SUM of FREQUENCY:         ");   Serial.println(sum_of_frequency);           // Hz
    Serial.print("SUM of POWER_FACTOR:      ");   Serial.println(sum_of_power_factor);
    Serial.println("====================================================");
}
void resetEnergy(uint8_t slaveAddr)                                                // Function to reset energy value on PZEM device.
{
  /* The command to reset the slave's energy is (total 4 bytes):
     Slave address + 0x42 + CRC check high byte + CRC check low byte. */
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  Serial.println("Resetting Energy");
  pzemSerial.write(slaveAddr);
  pzemSerial.write(resetCommand);
  pzemSerial.write(lowByte(u16CRC));
  pzemSerial.write(highByte(u16CRC));
  delay(1000);
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)                    // Function to change/assign pzem address
{
  static uint8_t SlaveParameter = 0x06;
  static uint16_t registerAddress = 0x0002;                                       // Register address to be changed
  uint16_t u16CRC = 0xFFFF;
  u16CRC = crc16_update(u16CRC, OldslaveAddr);
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));
  Serial.println("Changing Slave Address");
  pzemSerial.write(OldslaveAddr);
  pzemSerial.write(SlaveParameter);
  pzemSerial.write(highByte(registerAddress));
  pzemSerial.write(lowByte(registerAddress));
  pzemSerial.write(highByte(NewslaveAddr));
  pzemSerial.write(lowByte(NewslaveAddr));
  pzemSerial.write(lowByte(u16CRC));
  pzemSerial.write(highByte(u16CRC));
  Serial.println("Changing Slave Address is done"); 
  delay(1000);
}

/***************************************************
          Relay Functions
 **************************************************/

BLYNK_CONNECTED() {                                           // Every time we connect to the cloud...
  Serial.println("Blynk Connected");
  ReCnctCount = 0;
  Blynk.syncVirtual(VPIN_BUTTON_1);                           // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_2);
  Blynk.syncVirtual(VPIN_BUTTON_3);
  Blynk.syncVirtual(VPIN_BUTTON_4);
  Blynk.syncVirtual(VPIN_AUTO_MODE_BUTTON_1);

  /*  Alternatively, you could override server state using:
    Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
    Blynk.virtualWrite(VPIN_BUTTON_2, relay2State);
    Blynk.virtualWrite(VPIN_BUTTON_3, relay3State);
    Blynk.virtualWrite(VPIN_BUTTON_4, relay4State);
  */
}

/* When App button is pushed - switch the state */

BLYNK_WRITE(VPIN_BUTTON_1) {  
  relay1State = param.asInt();
  digitalWrite(RELAY_PIN_1, relay1State);
}
BLYNK_WRITE(VPIN_BUTTON_2) {
  relay2State = param.asInt();
  digitalWrite(RELAY_PIN_2, relay2State);
}
BLYNK_WRITE(VPIN_BUTTON_3) {
  relay3State = param.asInt();
  digitalWrite(RELAY_PIN_3, relay3State);
}
BLYNK_WRITE(VPIN_BUTTON_4) {
  relay4State = param.asInt();
  digitalWrite(RELAY_PIN_4, relay4State);
}
BLYNK_WRITE(VPIN_AUTO_MODE_BUTTON_1) {                      // Get auto mode button status value
  auto_mode_state_1 = param.asInt();
}
  
void checkPhysicalButton()                                  // Here we are going to check push button pressed or not and change relay state
{
  if (digitalRead(PUSH_BUTTON_1) == LOW) {
    // pushButton1State is used to avoid sequential toggles
    if (pushButton1State != LOW) {
      
      // Toggle Relay state
      relay1State = !relay1State;
      digitalWrite(RELAY_PIN_1, relay1State);

      // Update Button Widget
      Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
    }
    pushButton1State = LOW;
  } else {
    pushButton1State = HIGH;
  }

  if (digitalRead(PUSH_BUTTON_2) == LOW) {
    // pushButton2State is used to avoid sequential toggles
    if (pushButton2State != LOW) {

      // Toggle Relay state
      relay2State = !relay2State;
      digitalWrite(RELAY_PIN_2, relay2State);

      // Update Button Widget
      Blynk.virtualWrite(VPIN_BUTTON_2, relay2State);
    }
    pushButton2State = LOW;
  } else {
    pushButton2State = HIGH;
  }

/* push button 3 and push button 4 are disabled */

  //  if (digitalRead(PUSH_BUTTON_3) == LOW) {
  //    // pushButton3State is used to avoid sequential toggles
  //    if (pushButton3State != LOW) {
  //
  //      // Toggle Relay state
  //      relay3State = !relay3State;
  //      digitalWrite(RELAY_PIN_3, relay3State);
  //
  //      // Update Button Widget
  //      Blynk.virtualWrite(VPIN_BUTTON_3, relay3State);
  //    }
  //    pushButton3State = LOW;
  //  } else {
  //    pushButton3State = HIGH;
  //  }

  //  if (digitalRead(PUSH_BUTTON_4) == LOW) {
  //    // pushButton4State is used to avoid sequential toggles
  //    if (pushButton4State != LOW) {
  //
  //      // Toggle Relay state
  //      relay4State = !relay4State;
  //      digitalWrite(RELAY_PIN_4, relay4State);
  //
  //      // Update Button Widget
  //      Blynk.virtualWrite(VPIN_BUTTON_4, relay4State);
  //    }
  //    pushButton4State = LOW;
  //  } else {
  //    pushButton4State = HIGH;
  //  }
}

void get_pzem_data()                                          // Function to check time to see if it reached mentioned time to fetch PZEM data
{
    pzemdevice1(); 
    pzemdevice2();
    pzemdevice3();
    sumofpzem();
  }

void swith_off()                                        // Function to check if voltage low condition occurs if occurs, switch off relays
{
  if(voltage_usage_1 < VOLTAGE_1_CUTOFF || voltage_usage_2 < VOLTAGE_2_CUTOFF || voltage_usage_3 < VOLTAGE_3_CUTOFF){
    Serial.println("Low Voltage is detected!....");
    Serial.println("Switching off relay now..");

    Serial.println("Relay 1 OFF..");
    digitalWrite(RELAY_PIN_1, HIGH);
    Blynk.virtualWrite(VPIN_BUTTON_1, HIGH);
    
    Serial.println("Relay 2 OFF..");
    digitalWrite(RELAY_PIN_2, HIGH);
    Blynk.virtualWrite(VPIN_BUTTON_2, HIGH);
  }
}

void auto_mode()                                      // Function to check if auto mode is ON and all voltage value is greater than voltage cutoff value, then turn on 2 relays
{    
  if(auto_mode_state_1 == LOW && voltage_usage_1 > VOLTAGE_1_CUTOFF && voltage_usage_2 > VOLTAGE_2_CUTOFF && voltage_usage_3 > VOLTAGE_3_CUTOFF){  //checks if auto mode is ON and voltage values is greater than min value
    Serial.println("All condition is TRUE...swtiching on relay now.");
    
    digitalWrite(RELAY_PIN_1, LOW);                  // Turn on relay 1 
    Blynk.virtualWrite(VPIN_BUTTON_1, LOW);          // Update Blynk button status to ON
    Serial.println("RELAY 1 Turned ON");
    
    digitalWrite(RELAY_PIN_2, LOW);
    Blynk.virtualWrite(VPIN_BUTTON_2, LOW);
    Serial.println("RELAY 2 Turned ON");  
  }
}

void loop()
{
  ArduinoOTA.handle();                                                        // For OTA
  timer.run();

  if (Blynk.connected()) {                                                    // If connected run as normal
    Blynk.run();
  } 
  else if (ReCnctFlag == 0) {                                                 // If NOT connected and not already trying to reconnect, set timer to try to reconnect in 30 seconds
      ReCnctFlag = 1;                                                         // Set reconnection Flag
      Serial.println("Starting reconnection timer in 30 seconds...");
      timer.setTimeout(30000L, []() {                                         // Lambda Reconnection Timer Function
      ReCnctFlag = 0;                                                         // Reset reconnection Flag
      ReCnctCount++;                                                          // Increment reconnection Counter
      Serial.print("Attempting reconnection #");
      Serial.println(ReCnctCount);
      Blynk.connect();                                                        // Try to reconnect to the server
    });                                                                       // END Timer Function
    }
}
