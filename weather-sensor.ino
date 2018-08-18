/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0: Henrik EKblad
 * Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)
 * 
 * DESCRIPTION
 * This sketch provides an example of how to implement a humidity/temperature
 * sensor using a DHT11/DHT-22.
 *  
 * For more information, please visit:
 * http://www.mysensors.org/build/humidity
 * 
 */

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

// Pins CE, CSN for ARDUINO
#define MY_RF24_CE_PIN    9
#define MY_RF24_CS_PIN   10

// Pin for reset
#define RESET_PIN 2

// Device powered by battery
#define BATTERY_ON

#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include <MySensors.h>  
#include <AM2320.h>
#include "SparkFunMAX17043.h"

// Set this to the pin you connected the DHT's data pin to
#define DHT_MINIMUM_SAMPLING_PERIOD 2000

//
// Cycle
//
// Average cycle duration / sleep time duration in reality (in milliseconds)
// For each cycle, if probe value change, she's send to gateway
// Must be >1000ms for DHT22 and >2000ms for DHT11
static uint32_t CYCLE_DURATION = 600000; // 10 min
// Force sending an update of the probe value after n cycle ; even if probe value not changed since N cycle
// So, gateway receive minimum of every CYCLE_DURATION * FORCE_SEND_AFTER_N_CYCLE ms a probe updated value
static uint16_t FORCE_SEND_AFTER_N_CYCLE = 3; // 20 min * 3 = 1h
// Reset software after number of cycle (1 cycle = CYCLE_DURATION)
#define RESET_AFTER_N_CYCLE 72 // 1h/10min * 12 = 12h

//
// Battery
//
// Number of consecutive "low battery" values after which to consider this information as reliable.
#define SEND_BATTERY_LOW_AFTER_N_CYCLE 3
// Limit (in %) to consider battery as "low power"
// Under this value, node goind to sleep permanently
#define BATTERY_LOW_LIMIT 10 // 10%


//
// Node
//
// Node childs id (local to node)
#define CHILD_VALUE_ID_HUM 0
#define CHILD_VALUE_ID_TEMP 1
#define CHILD_RESET_ID 2
#define CHILD_CONFIG_ID 3
// Message which contain probe value
MyMessage messageValueHum(CHILD_VALUE_ID_HUM, V_HUM);
MyMessage messageValueTemp(CHILD_VALUE_ID_TEMP, V_TEMP);
MyMessage messageReset(CHILD_RESET_ID, V_STATUS);
MyMessage messageConfigCycleDuration(CHILD_CONFIG_ID, V_VAR1);
MyMessage messageConfigCycleN(CHILD_CONFIG_ID, V_VAR2);
// Battery gauge
MAX17043 batteryGauge;
// Probe
AM2320 dht;


float lastProbeValueTemp;
float lastProbeValueHum;
uint16_t cycleCptTemp;
uint16_t cycleCptHum;
uint16_t cycleCptReset;
uint8_t batteryLowCpt;
bool metric = true;


/**
 * Init node to gateway
 */
void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("TemperatureAndHumidity", "1.6.1");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_VALUE_ID_HUM, S_HUM);   // Probe value (humidity)
  present(CHILD_VALUE_ID_TEMP, S_TEMP); // Probe value (temperature)
  present(CHILD_RESET_ID, S_BINARY);    // Reset command
  present(CHILD_CONFIG_ID, S_CUSTOM);   // Configuration

  metric = getControllerConfig().isMetric;
}

/**
 * Setup node
 */
void setup()
{
  // Keep reset pin HIGH (need to be the first command to prevent infinite reset!)
  digitalWrite(RESET_PIN, HIGH);

  
  // 
  // DHT
  //
  #ifdef MY_DEBUG
  Serial.println("DHT begin");
  #endif
  dht.begin();

  // Configure pin
  pinMode(RESET_PIN, OUTPUT);
  
  // Battery
  initBattery();

  //
  // Weather
  //
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  #ifdef MY_DEBUG
  Serial.println("Wait DHT init");
  if (CYCLE_DURATION <= DHT_MINIMUM_SAMPLING_PERIOD) {
    Serial.println("Warning: CYCLE_DURATION is smaller than supported by the sensor!");
  }
  #endif
  sleep(DHT_MINIMUM_SAMPLING_PERIOD);

  // Force send at startup
  cycleCptTemp = FORCE_SEND_AFTER_N_CYCLE;

  #ifdef MY_DEBUG
  Serial.println("Setup end.");
  #endif
}

/**
 * Loop
 */
void loop()      
{  
  // Battery level (%)
  processBattery();
  
  // Probe value: If value has been sended, take the opportunity to make some check from server
  if (processWeather()) {
    // Configuration
    processCheckCommandFromServer();
    // Reset
    processCheckResetFromServer();
  }

  // Reset
  processReset();

  #ifdef MY_DEBUG
  Serial.println("Go to sleep!");
  #endif
  // Sleep for a while to save energy
  sleep(CYCLE_DURATION); 
}

/**
 * When receive message
 */
void receive(const MyMessage &message)
{
  // Expect one type of message from gateway. Check it
  if (
    message.sensor == CHILD_RESET_ID && 
    message.type == V_STATUS
  ) {
    #ifdef MY_DEBUG
    Serial.print("Message received. Require for reset: ");
    Serial.println(message.getBool());
    #endif
    // If reset requested
    if (message.getBool()) {
      // Restart node
      restart();
    }
  }
  // Receive configuration
  else if (
    message.sensor == CHILD_CONFIG_ID && 
    (message.type == V_VAR1 || message.type == V_VAR2)
  ) {
    #ifdef MY_DEBUG
    Serial.println("Config received.");
    #endif
    // Configure cycle duration
    if (message.type == V_VAR1) {
      uint32_t v = message.getULong();
      if (v > 0) {
        #ifdef MY_DEBUG
        Serial.print("Configure cycle duration: ");
        Serial.print(v);
        Serial.println(" ms");
        #endif
        CYCLE_DURATION = v;
      }
    }
    // Configure number of cycle to force update
    else if (message.type == V_VAR2 ) {
      uint16_t v = message.getUInt();
      if (v > 0) {
        #ifdef MY_DEBUG
        Serial.print("Configure number of cycle to force update: ");
        Serial.print(v);
        Serial.println(" ms");
        #endif
        FORCE_SEND_AFTER_N_CYCLE = v;
      }
    }
  }
}

/**
 * Restart
 */
void restart()
{
  doRestart(true);
}
/**
 * Do restart
 * 
 * @param bool software Use software method. Hardware method is used by default
 *                      Note that to use hardware restart, you need to link RESET_PIN to RST
 */
void doRestart(bool software)
{
  #ifdef MY_DEBUG
  Serial.println("Restart node");
  #endif

  // Before restart, set reset value to off
  send(messageReset.set(false));
  cycleCptReset = 0;

  // Restart software
  if (software) {
    /*
    wdt_enable(WDTO_15MS);
    while(1) {}
    */
    asm volatile ("  jmp 0");
  }
  // Restart hardware
  else {
    digitalWrite(RESET_PIN, LOW);
  }
}

/**
 * Read weather and send data
 * 
 * @return bool True if probe value has been send
 */
bool processWeather()
{
  bool r = false;
  float probeValue;
  
  // Force reading sensor, so it works also after sleep()
  if (!dht.measure()) {
    #ifdef MY_DEBUG
    Serial.println("Failed reading temperature and humidity from AMT2320!");
    int errorCode = dht.getErrorCode();
    switch (errorCode) {
      case 1: Serial.println("ERR: Sensor is offline"); break;
      case 2: Serial.println("ERR: CRC validation failed."); break;
    }
    #endif
  } else {
    // Get probe value (temperature) from DHT library
    probeValue = dht.getTemperature();
    #ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(probeValue);
    #endif
    // Only send probe value (temperature) if it changed since the last measurement or if we didn't send an update for n times
    if (probeValue != lastProbeValueTemp || cycleCptTemp == FORCE_SEND_AFTER_N_CYCLE) {
      lastProbeValueTemp = probeValue;
      // Reset no updates counter
      cycleCptTemp = 0;
      r = true;
      #ifdef MY_DEBUG
      Serial.println("Send value (temperature) to server");
      #endif
      send(messageValueTemp.set(probeValue, 1));
    } else {
      // Increase no update counter if the temperature stayed the same
      cycleCptTemp++;
    }
  
    // Get probe value (humidity) from DHT library
    probeValue = dht.getHumidity();
    #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(probeValue);
    #endif
    // Only send probe value (humidity) if it changed since the last measurement or if we didn't send an update for n times
    if (probeValue != lastProbeValueHum || cycleCptHum == FORCE_SEND_AFTER_N_CYCLE) {
      lastProbeValueHum = probeValue;
      // Reset no updates counter
      cycleCptHum = 0;
      r = true;
      #ifdef MY_DEBUG
      Serial.println("Send value (humidity) to server");
      #endif
      send(messageValueHum.set(probeValue, 1));
    } else {
      // Increase no update counter if the humidity stayed the same
      cycleCptHum++;
    }
  }

  return r;
}

/**
 * Init battery
 */
void initBattery()
{
  #ifdef BATTERY_ON
  //
  // Battery gauge
  //
  #ifdef MY_DEBUG
  Serial.println("Battery gauge begin");
  #endif
  batteryGauge.begin();
  #ifdef MY_DEBUG
  Serial.println("Battery gauge quick start");
  #endif
  batteryGauge.quickStart();
  #endif
}

/**
 * Process to read battery and send level
 * 
 * If battery level lower than limit, inter into deep sleep mode!
 */
void processBattery()
{
  #ifdef BATTERY_ON
  
  float batteryLevel = batteryGauge.getSOC();
  #ifdef MY_DEBUG
  Serial.print("Battery level: ");
  Serial.print(batteryLevel);
  Serial.println("%");
  #endif
  sendBatteryLevel(batteryLevel);
  // If lower that battery low limit, waiting for n consecutive check
  if (batteryLevel < BATTERY_LOW_LIMIT) {
    batteryLowCpt++;
  // Else, reset cpt (wrong check)
  } else {
    batteryLowCpt = 0;
  }
  // If battery low confirmed
  if (batteryLowCpt >= SEND_BATTERY_LOW_AFTER_N_CYCLE) {
    batteryLowCpt = 0;
    // Consider battery level as empty!
    sendBatteryLevel(0);
    // Sleep infinity
    #ifdef MY_DEBUG
    Serial.println("Low battery level! Enter into deep sleep mode!");
    #endif
    sleep(1, CHANGE, 0); // Interrupt with pin D3
  }

  #endif
}

/**
 * Process to restart node after n cycle
 */
void processReset()
{
  // Restart every N cycle
  cycleCptReset++;
  if (cycleCptReset >= RESET_AFTER_N_CYCLE) {    
    restart();
    // Below restart(): Never executed
  }
}

/**
 * Process a check reset command from server
 */
void processCheckResetFromServer()
{
  // Retrieve reset value from gateways (if order to restard has been sended)
  request(CHILD_RESET_ID, V_STATUS, 0);
  wait(1000); // Waiting for answer
}

/**
 * Process a check config command from server
 */
void processCheckCommandFromServer()
{
  // Request configuration
  request(CHILD_CONFIG_ID, V_VAR1, 0); // Cycle duration
  request(CHILD_CONFIG_ID, V_VAR2, 0); // Number of cycle
  wait(1000); // Waiting for answer
}
