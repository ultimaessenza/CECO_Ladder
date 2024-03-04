//#include <SoftwareSerial.h>
//#include <EEPROM.h>
#include "src/CECO_Ladder.h"

#define BTN_COUNT 2
#define BTN_CANCEL 12   // left side of ladder
#define BTN_CONFIRM 13  // right side of ladder

Adafruit_MPR121 touchSensor;
Preferences preferences;
CECO_Ladder* ceco_ladder;
//byte eeAddress = 0;

//serial members
//char frameStart = '!';
//char frameEnd = '#';

struct TouchInput {
  int touchID = 0;
  bool isTouched = false;
  bool isplaying = false;
  bool touchBegin = false;
  unsigned long touchDuration = 0;
};

struct Device_info {
  uint8_t ID_number;
  TouchInput sensors[12];
};

bool buttonWasReleased[] = { true, true };  // track state of input buttons (not ladder touches)
Device_info Device;

unsigned long previousMicros = 0;  // used to calculate deltaTime (more precision)

enum EventTypes {
  Touch = 0,
  Release
};

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("Init");

  pinMode(BTN_CONFIRM, INPUT_PULLUP);
  pinMode(BTN_CANCEL, INPUT_PULLUP);

  ceco_ladder = new CECO_Ladder(&touchSensor, &preferences);

  if (preferences.begin(ceco_ladder->prf_nameSpaces[0], true, ceco_ladder->prf_labels[0])) {
    preferences.end();
    if (!ceco_ladder->Begin(CECO_Ladder::alternativeBoot)) {
      Serial.println("MPR121 - Initialization Failed");
      ESP_LOGE(TAG, "CECO_Ladder::Begin() Failed");
    } else {
      Serial.println("MPR121 - Complete Setup");
      ESP_LOGI(TAG, "CECO_Ladder::Begin() Successful");
    }
  } else {
    if (!ceco_ladder->Begin(CECO_Ladder::adafruitDefaults)) {
      Serial.println("MPR121 - Initialization Failed");
      ESP_LOGE(TAG, "CECO_Ladder::begin() Failed");
    } else {
      Serial.println("MPR121 - Complete Setup");
      ESP_LOGI(TAG, "CECO_Ladder::begin() Successful");
    }
  }


  ESP_LOGI(TAG, "Setup() has exited.");
}

//----------------------------------------------------
// Check if the special buttons on the ladder
// pin = the digital pin # on Arduino
// index is used for managing button press/release logic
// Types: 0=inputTouch, 1=inputRelease, 2=BtnBack, 3=BtnFwd
//----------------------------------------------------
void ReadButton(uint8_t pin, uint8_t index) {
  bool digRead = digitalRead(pin);
  if (!digRead) {
    if (buttonWasReleased[index]) {
      buttonWasReleased[index] = false;
      int tType;
      if (pin == BTN_CANCEL) {
        tType = 2;  // check which button and convert to proper enum (int) value (currently either 2 or 3)
      } else if (pin == BTN_CONFIRM) {
        tType = 3;
      }
      Serial.println("{\"type\":" + String(tType) + ",\"touchID\":" + String(index + SENSOR_COUNT) + "}");
    }
  } else {
    if (!buttonWasReleased[index]) buttonWasReleased[index] = true;
  }
}

void loop() {
  //----------------------------------------------------
  // Check for button presses (separate from ladder inputs)
  //----------------------------------------------------
  ReadButton(BTN_CANCEL, 0);
  ReadButton(BTN_CONFIRM, 1);

  String msg = SerialReadLine();

  if (msg != "") {
    int delimiter_index = msg.indexOf(":");
    msg.toUpperCase();
    String sKey = "";
    String sVal = "";
    Serial.println("\n>" + msg);

    if (delimiter_index != -1) {
      // DELIMITER COMMANDS
      sKey = msg.substring(0, delimiter_index);
      sVal = msg.substring(delimiter_index + 1);
      //uint16_t iVal = constrain(sVal.toInt(), 0, UINT16_MAX);
      uint8_t iVal = constrain(sVal.toInt(), 0, UINT8_MAX);

      // ALL ":" COMMANDS OF LENGTH 1
      if (sKey.length() == 1) {
        if (sKey == "S") {
          ceco_ladder->currState = (CECO_Ladder::State)iVal;
        }
      }

      // ALL ":" COMMANDS OF LENGTH 2
      if (sKey.length() == 2) {
        if (sKey == "SR") {
          ceco_ladder->SoftReset(iVal);
        }
        if (sKey == "SU") {
          ceco_ladder->SwitchUser(iVal);
        }
      }

      // ALL ":" COMMANDS OF LENGTH 3
      if (sKey.length() == 3) {
        if (sKey == "TTH") {
          ceco_ladder->SetThresholdTouch(iVal);
        } else if (sKey == "RTH") {
          ceco_ladder->SetThresholdRelease(iVal);
        } else if (sKey == "THX") {
          ceco_ladder->SetThresholdTouchRelease(iVal);
        } else if (sKey == "CDC") {
          ceco_ladder->SetGlobalChargeCurrent(iVal);
        } else if (sKey == "CDT") {
          ceco_ladder->SetGlobalChargeTime(iVal);
        }
      }
    } else {
      // NO-DELIMITER COMMANDS
      sKey = msg.substring(0);

      if (sKey == "?") {
        ceco_ladder->PrintHelp();
      } else if (sKey == "?S") {
        ceco_ladder->PrintState();
      } else if (sKey == "?TTR") {
        ceco_ladder->PrintThreshold();
      } else if (sKey == "?CCT") {
        ceco_ladder->PrintCONFIG();
      } else if (sKey == "?PRF") {
        ceco_ladder->PrintPreferences();
      } else if (sKey == "SAVE") {
        ceco_ladder->SavePreferences();
      } else if (sKey == "LOAD") {
        ceco_ladder->LoadPreferences();
      } else if (sKey == "FACT") {
        ceco_ladder->RestorePreferences();
      } else if (sKey == "SR") {
        ceco_ladder->SoftReset();
      } else if (sKey == "REBOOT") {
        ceco_ladder->SystemReset();
      }
    }
  }

  switch (ceco_ladder->currState) {
    case CECO_Ladder::displayTouches:
      ProcessTouches();
      break;

    case CECO_Ladder::displayCalibration:
      DisplayCalibrationData();
      break;

    case CECO_Ladder::displayAndTouch:
      DisplayCalibrationData();
      ProcessTouches();
      break;

    case CECO_Ladder::displayNone:
      break;

    default:
      break;
  }
}

void DisplayCalibrationData() {
  // Send calibration information for each of the sensor inputs
  String msg = "";
  msg += "{\"type\":4,\"touchID\":";
  msg += "-1,\"msgData\":\"";
  for (uint8_t i = 0; i < 12; i++) {
    // JSON Output  Serial.println("{\"type\":0,\"touchID\":" + String(i) + "}");
    msg += String(touchSensor.filteredData(i)) + "/" + String(touchSensor.baselineData(i)) + (i < 11 ? "," : "");
  }
  msg += "\"}";
  Serial.println(msg);
}

//-------------------------------------------------------------------------
// Show verbose debug details
//-------------------------------------------------------------------------
void ShowDetails() {
  int count = SENSOR_COUNT;
  // Debug - Show filteredData and Baseline
  Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t 0x");
  Serial.println(touchSensor.touched(), HEX);
  Serial.print("Filt: ");
  for (uint8_t i = 0; i < count; i++) {
    Serial.print(touchSensor.filteredData(i));
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("Base: ");
  for (uint8_t i = 0; i < count; i++) {
    Serial.print(touchSensor.baselineData(i));
    Serial.print("\t");
  }
  Serial.println();
}

//-------------------------------------------------------------------------
// Read Capacitive Touch Inputs
//-------------------------------------------------------------------------
void ProcessTouches() {
  //----------------------------------------------------
  // Delta Time
  //----------------------------------------------------
  // Calculate deltaTime (the amount of time that has passed since the completion of the previous update cycle)

  unsigned long deltaTime_mS = (micros() - previousMicros) / 1000;
  //unsigned long deltaTime_uS = (micros() - previousMicros);

  // Immediately after calculating deltaTmie, update the previous timestamp
  previousMicros = micros();

  //----------------------------------------------------
  // Check each sensor to see if it is actively touched
  //----------------------------------------------------
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // The MPR121.touched() method returns a bitmask indicating which input was touched.
    // We left-shift 1 by i to check if the bit-shifted i matches the bitmask.
    // If this expression returns true, we know that input[i] is actively touched

    // Bitmask = https://en.wikipedia.org/wiki/Mask_(computing)

    // This print shows the bitshift happening in binary (turn off unless debugging)
    // Serial.print("Shift bits:"); Serial.println((1 << i), BIN);

    if (touchSensor.touched() & (1 << i)) {
      //----------------------------------------------------
      // On Touch
      // If this is an "initial touch" then set variable to indicate the touch "Start"
      //----------------------------------------------------
      if (!Device.sensors[i].isTouched) {
        //Serial.print("Touched:  "); Serial.println(i);

        // JSON Output
        Serial.println("{\"type\":0,\"touchID\":" + String(i) + "}");

        Device.sensors[i].isTouched = true;
        Device.sensors[i].touchBegin = true;
        Device.sensors[i].touchDuration = 0;
      } else {
        // If sensor[i] is currently touched (but is not the initial touch) then increment time.
        // (See deltaTime calculation at start of function)
        Device.sensors[i].touchDuration += deltaTime_mS;

        // If this was the initial touch, set the begin bool false
        if (Device.sensors[i].touchBegin) {
          Device.sensors[i].touchBegin = false;
        }
      }
    } else {
      //----------------------------------------------------
      // On Released
      //----------------------------------------------------
      if (Device.sensors[i].isTouched) {

        //Serial.print("Released: ");
        Serial.println("{\"type\":1,\"touchID\":" + String(i) + "}");

        Device.sensors[i].isTouched = false;
        Device.sensors[i].touchDuration = 0;
      }
    }
  }
}

// This is an ascii serial read-line that is non-blocking
String SerialReadLine() {
  char c;
  bool msgReceived = false;
  static uint8_t index = 0;
  static String strMsg = "";

  if (index == 0) strMsg = "";  // reset string if starting over

  while (Serial.available() > 0) {
    c = (char)Serial.read();
    if ((c == '\r') || (c == '\n')) {
      index = 0;
      msgReceived = true;
      break;
    }

    strMsg += c;
    index++;
  }

  return (msgReceived ? strMsg : String(""));
}

// template <class T> int EEPROM_writeAnything(const T& value, byte eeAddress)
// {
// 	int ee = eeAddress;
// 	const byte* p = (const byte*)(const void*)&value;
// 	unsigned int i;
// 	for (i = 0; i < sizeof(value); i++)
// 		EEPROM.write(ee++, *p++);
// 	return i;
// }

// template <class T> int EEPROM_readAnything(T& value, byte eeAddress)
// {
// 	int ee2 = eeAddress;
// 	byte* p = (byte*)(void*)&value;
// 	unsigned int i;
// 	for (i = 0; i < sizeof(value); i++)
// 		*p++ = EEPROM.read(ee2++);
// 	return i;
// }
