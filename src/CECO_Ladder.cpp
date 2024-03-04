#include "CECO_Ladder.h"

// Parameterized Constructor for the CECO_Ladder class that sets the following member fields.
// 1. Sets "device" and "preferences" to the passed in Adafruit_MPR121 and Preferences
//    class instances initialized in the .ino.
// 2. Sets the current ESP32-Preferences namespace array index, "currUser".
//     prf_nameSpaces[currUser = 0]: factory default calibrations.
//     prf_nameSpaces[currUser = 1]: user calibrations.
// 3. Sets the (volatile) Threshold Touch/Release arrays, the Charge Current/Time
//    and prints the current state.
CECO_Ladder::CECO_Ladder(Adafruit_MPR121* _device, Preferences* _preferences) {
  device = _device;
  preferences = _preferences;
  preferences->begin(prf_nameSpaces[1], false, prf_labels[0]);
  if (preferences->isKey(prf_User) == true) {
    currUser = preferences->getUChar(prf_User);
    ESP_LOGI(TAG, "lastUser key already exists (%d), currUser: %d", preferences->getUChar(prf_User), currUser);
    preferences->end();
    StartThresholdFields(currUser);
    StartGlobalChargeFields(currUser);
  } else {
    currUser = 1;
    preferences->putUChar(prf_User, currUser);
    ESP_LOGI(TAG, "lastUser key created (value: %d), currUser: %d", preferences->getUChar(prf_User), currUser);
    preferences->end();
    StartThresholdFields(0);
    StartGlobalChargeFields(0);
  }
  Serial.println("Changed state to: " + String(currState));
}

// Helper method for loading Threshold ESP32-Preferences.
//  Parameters: index of the ESP32-Preferences namespace to load from.
void CECO_Ladder::StartThresholdFields(uint8_t _user) {
  ESP_LOGI(TAG, "User %d called ThresholdFields()", _user);
  if (preferences->begin(prf_nameSpaces[_user], false, prf_labels[0])) {
    if (_user == 0) {
      for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        preferences->putUChar(prf_TTH[i], MPR121_TOUCH_THRESHOLD_DEFAULT);
        preferences->putUChar(prf_RTH[i], MPR121_RELEASE_THRESHOLD_DEFAULT);
      }
      preferences->end();
      ESP_LOGI(TAG, "Added user 0 TTH[i] & THR[i] defaults succesfully");

      preferences->begin(prf_nameSpaces[1], false, prf_labels[0]);
      for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        preferences->putUChar(prf_TTH[i], MPR121_TOUCH_THRESHOLD_DEFAULT);
        preferences->putUChar(prf_RTH[i], MPR121_RELEASE_THRESHOLD_DEFAULT);
      }
      ESP_LOGI(TAG, "Added user 1 TTH[i] & THR[i] defaults succesfully");
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      auto tth = preferences->getUChar(prf_TTH[i]);
      auto rth = preferences->getUChar(prf_RTH[i]);
      ESP_LOGI(TAG, "%d", tth);
      ESP_LOGI(TAG, "%d", rth);
      mThresholdTouch[i] = tth;
      mThresholdRelease[i] = rth;
    }
    preferences->end();
    ESP_LOGI(TAG, "Finished storing volatile TTH and RTH values");
  }
}

// Helper method for loading Global Charge ESP32-Preferences.
//  Parameters: index of the ESP32-Preferences namespace to load from.
void CECO_Ladder::StartGlobalChargeFields(uint8_t _user) {
  ESP_LOGI(TAG, "User %d called GlobalChargeFields()", _user);
  if (preferences->begin(prf_nameSpaces[_user], false, prf_labels[0])) {
    if (_user == 0) {
      preferences->putUChar(prf_gCDC, MPR121_CURRENT_CHARGE_DEFAULT);
      preferences->putUChar(prf_gCDT, MPR121_TIME_CHARGE_DEFAULT);
      preferences->end();
      ESP_LOGI(TAG, "Added user 0 CDC & CDT defaults succesfully");

      preferences->begin(prf_nameSpaces[1], false, prf_labels[0]);
      preferences->putUChar(prf_gCDC, MPR121_CURRENT_CHARGE_DEFAULT);
      preferences->putUChar(prf_gCDT, MPR121_TIME_CHARGE_DEFAULT);
      ESP_LOGI(TAG, "Added user 1 CDC & CDT defaults succesfully");
    }
    mGlobalChargeCurrent = preferences->getUChar(prf_gCDC);
    mGlobalChargeTime = preferences->getUChar(prf_gCDT);
    preferences->end();
    ESP_LOGI(TAG, "Finished storing volatile CDC and CDT values");
  }
}

// A duplicate (slightly modified version) of the Adafruit_MPR121::begin(), necessary
// for cold-booting the device into ESP32-Preferences stored calibration values.
//  Parameters: (Optional) the I2C address (default: to 0x5a), (Optional) the Wire object.
bool CECO_Ladder::Begin(BootParameter _boot, uint8_t i2caddr) {
  if (_boot == adafruitDefaults) {
    Serial.println("BOOTING UP IN FACTORY DEFAULT MODE"); /*
		This is the regular Adafruit_MPR121::begin()*/
    if (!device->begin(i2caddr)) {
      ESP_LOGE(TAG, "CECO_Ladder::Begin() call to Adafruit_121::begin() Failed");
      return false;
    } else {
      ESP_LOGI(TAG, "CECO_Ladder::Begin() call to Adafruit_121::begin() Successful");
      return true;
    }
  } else if (_boot == esp32Preferences) {
    Serial.println("BOOTING UP IN ESP32 PREFERENCES MODE"); /*
		This version handles the begin() process when there are values in the nvs*/

    if (i2c_dev) {
      delete i2c_dev;
    }
    i2c_dev = new Adafruit_I2CDevice(i2caddr);

    if (!i2c_dev->begin()) {
      ESP_LOGE(TAG, "i2c_dev->begin(), Failed.");
      return false;
    }

    // soft reset
    device->writeRegister(MPR121_SOFTRESET, 0x63);
    delay(1);
    for (uint8_t i = 0; i < 0x7F; i++) {
      Serial.print("$");
      Serial.print(i, HEX);
      Serial.print(": 0x");
      Serial.println(device->readRegister8(i));
    }

    device->writeRegister(MPR121_ECR, 0x0);

    uint8_t c = device->readRegister8(MPR121_CONFIG2);
    if (c != 0x24) {
      ESP_LOGE(TAG, "c != 0x24, Failed.");
      return false;
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, mThresholdTouch[i]);
      device->writeRegister(MPR121_RELEASETH_0 + 2 * i, mThresholdRelease[i]);
    }

    device->writeRegister(MPR121_MHDR, 0x01);
    device->writeRegister(MPR121_NHDR, 0x01);
    device->writeRegister(MPR121_NCLR, 0x0E);
    device->writeRegister(MPR121_FDLR, 0x00);

    device->writeRegister(MPR121_MHDF, 0x01);
    device->writeRegister(MPR121_NHDF, 0x05);
    device->writeRegister(MPR121_NCLF, 0x01);
    device->writeRegister(MPR121_FDLF, 0x00);

    device->writeRegister(MPR121_NHDT, 0x00);
    device->writeRegister(MPR121_NCLT, 0x00);
    device->writeRegister(MPR121_FDLT, 0x00);

    device->writeRegister(MPR121_DEBOUNCE, 0);

    // Charge Discharge values
    device->writeRegister(MPR121_CONFIG1, mGlobalChargeCurrent);
    device->writeRegister(MPR121_CONFIG2, mGlobalChargeTime);

#ifdef AUTOCONFIG
    device->writeRegister(MPR121_AUTOCONFIG0, 0x0B);
    device->writeRegister(MPR121_UPLIMIT, 200);
    device->writeRegister(MPR121_TARGETLIMIT, 180);
    device->writeRegister(MPR121_LOWLIMIT, 130);
#endif

    byte ECR_SETTING = B10000000 + SENSOR_COUNT;
    device->writeRegister(MPR121_ECR, ECR_SETTING);

    return true;
  } else if (_boot == alternativeBoot) {
    Serial.println("BOOTING UP IN FACTORY DEFAULT MODE WITH NVS VALUES");
    if (!device->begin(i2caddr)) {
      ESP_LOGE(TAG, "CECO_Ladder::Begin() call to Adafruit_121::begin() Failed");
      return false;
    } else {
      ESP_LOGI(TAG, "CECO_Ladder::Begin() call to Adafruit_121::begin() Successful");

      ResetAndPrepare();
      for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, mThresholdTouch[i]);
        device->writeRegister(MPR121_RELEASETH_0 + 2 * i, mThresholdRelease[i]);
      }
      SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);

      return true;
    }
  }
}

// Helper function that resets _device and prepares it to receive new register values.
//	Note1: Should be used in conjunction with SetThresholdTouch, SetThresholdRelease
//         and SetRegistersAndStart.
// 	Note2: The order should be.
//			1. ResetAndPrepare.
//			2. SetThresholdTouch.
//			3. SetThresholdRelease.
//			4. SetRegistersAndStart.
void CECO_Ladder::ResetAndPrepare() {
  // 5.13 of page 19 of https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
  device->writeRegister(MPR121_SOFTRESET, RESET_KEY);
  delay(1);
  device->writeRegister(MPR121_ECR, 0x0);
  device->writeRegister(MPR121_CONFIG2, 0x24);
}

// Helper function that transforms a user-input integer CDC value (6-bit max) to a
// register-ready CDC input (1-byte).
//  Parameters: the charge discharge current, and (optional) first filter iterations.
byte CECO_Ladder::EncodeGlobalCDC(uint8_t _cdc, uint8_t _ffi) {
  // 5.8 of page 14 of https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
  uint8_t ffi_max = 34;
  uint8_t cdc_max = 63;

  uint8_t ffi = constrain(_ffi, 6, ffi_max);
  uint8_t cdc = constrain(_cdc, 0, cdc_max);

  // bit positions:       7654 3210
  uint8_t ffi_pos = 6; /* 1100 0000 */
  uint8_t cdc_pos = 0; /* 0011 1111 */

  //uint8_t ffi_enc_max = 3;  /* Disabled, manual encoding required */
  if (ffi <= 6) {
    ffi = 0;
  } else if (ffi <= 10) {
    ffi = 1;
  } else if (ffi <= 18) {
    ffi = 2;
  } else if (ffi > 18) {
    ffi = 3;
  }

  /*
	// Since CDC's encoding is 1 to 1, (e.g. 0b00001111 = 15 = 15 uA) this portion is uneeded.
	// However, it was left for completion's sake.
	uint8_t cdc_enc_max = 63;
	cdc = ((cdc * cdc_enc_max) / cdc_max);
	*/

  return (ffi << ffi_pos) | (cdc << cdc_pos);
}

// Helper function that transforms a user-input CDT value (3-bit) to a register-ready CDT input (1-byte).
//  Parameters: the charge discharge time, and (optional) second filter iterations, and (optional) electrode sample interval.
//  Note1: CDT, SFI, and ESI require manual encoding.
byte CECO_Ladder::EncodeGlobalCDT(uint8_t _cdt, uint8_t _sfi, uint8_t _esi) {
  // 5.8 of page 14 of https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
  uint8_t cdt_max = 32;
  uint8_t sfi_max = 18;
  uint8_t esi_max = 128;

  ESP_LOGI(TAG, "cdt before constrain: %d", _cdt);
  ESP_LOGI(TAG, "sfi before constrain: %d", _sfi);
  ESP_LOGI(TAG, "esi bedore constrain: %d", _esi);

  uint8_t cdt = constrain(_cdt, 0, cdt_max); /* microseconds */
  uint8_t sfi = constrain(_sfi, 4, sfi_max); /* samples */
  uint8_t esi = constrain(_esi, 1, esi_max); /* miliseconds */

  ESP_LOGI(TAG, "cdt after constrain: %d", cdt);
  ESP_LOGI(TAG, "sfi after constrain: %d", sfi);
  ESP_LOGI(TAG, "esi after constrain: %d", esi);

  // bit positions:       7654 3210
  uint8_t cdt_pos = 5; /* 1110 0000 */
  uint8_t sfi_pos = 3; /* 0001 1000 */
  uint8_t esi_pos = 0; /* 0000 0111 */

  // IMPORTANT: global CDT can be either 0 microseconds (disabled)
  //            or 0.5 microseconds (default value). Because the
  //            CECO_Ladder works with uint8_t, a value of 0.5 is
  //            no different than 0, therefore:
  //            0 is assigned to the 0.5 microsecond default and
  //            disabling the global CDT is no longer possible.
  //     NOTE1: Adding floating point math can fix this.
  //     NOTE2: Valid values for CDT are [0.5, 1, 2, 4, 8, 16, 32]
  if (cdt == 0) {
    cdt = 1 << cdt_pos;
  } else if (cdt <= 1) {
    cdt = 2 << cdt_pos;
  } else if (cdt <= 2) {
    cdt = 3 << cdt_pos;
  } else if (cdt <= 4) {
    cdt = 4 << cdt_pos;
  } else if (cdt <= 8) {
    cdt = 5 << cdt_pos;
  } else if (cdt <= 16) {
    cdt = 6 << cdt_pos;
  } else if (cdt > 16) {
    cdt = 7 << cdt_pos;
  }

  if (sfi <= 4) {
    sfi = 0 << sfi_pos;
  } else if (sfi <= 6) {
    sfi = 1 << sfi_pos;
  } else if (sfi <= 10) {
    sfi = 2 << sfi_pos;
  } else if (esi > 10) {
    sfi = 3 << sfi_pos;
  }

  if (esi <= 1) {
    esi = 0 << esi_pos;
  } else if (esi <= 2) {
    esi = 1 << esi_pos;
  } else if (esi <= 4) {
    esi = 2 << esi_pos;
  } else if (esi <= 8) {
    esi = 3 << esi_pos;
  } else if (esi <= 16) {
    esi = 4 << esi_pos;
  } else if (esi <= 32) {
    esi = 5 << esi_pos;
  } else if (esi <= 64) {
    esi = 6 << esi_pos;
  } else if (esi > 64) {
    esi = 7 << esi_pos;
  }

  ESP_LOGI(TAG, "cdt after shift: %d", cdt);
  ESP_LOGI(TAG, "sfi after shift: %d", sfi);
  ESP_LOGI(TAG, "esi after shift: %d", esi);

  return cdt | sfi | esi;
}

// These writeRegister correspond to the MPR121 Electrode Configuration Settings.
//  Parameters: the whole CONFIG1 byte (FFI + gCDC), the whole CONFIG2 byte (gCDT + SFI + ESI).
// 	Note1: It was copied from Adafruit_MPR121.begin() to enable hot-swapping values.
// 	Note2: The MPR121_SOFTRESET sets every register to 0x0, we need to reconfigure them every time.
void CECO_Ladder::SetRegistersAndStart(byte _config1, byte _config2) {
  // The following is an extract of 5.5 of page 12 of https://www.nxp.com/docs/en/data-sheet/MPR121.pdf .
  // 	Maximum Half Delta (MHD): Determines the largest variation to pass through baseline filter.
  // 	Noise Half Delta (NHD): Determines the incremental change when non-noise drift is detected.
  // 	Noise Count Limit (NCL): Determines the number of samples consecutively greater than MHD value.
  // 	Filter Delay Count Limit (FDL): Determines the operation rate of the filter (higer value, operates slower).

  // Rising
  device->writeRegister(MPR121_MHDR, 0x01);  //1~3F
  device->writeRegister(MPR121_NHDR, 0x01);  //1~3F
  device->writeRegister(MPR121_NCLR, 0x0E);  //0~FF
  device->writeRegister(MPR121_FDLR, 0x00);  //0~FF

  // Falling
  device->writeRegister(MPR121_MHDF, 0x01);  //1~3F
  device->writeRegister(MPR121_NHDF, 0x05);  //1~3F
  device->writeRegister(MPR121_NCLF, 0x01);  //0~FF
  device->writeRegister(MPR121_FDLF, 0x00);  //0~FF

  // Touched
  device->writeRegister(MPR121_NHDT, 0x00);  //1~3F
  device->writeRegister(MPR121_NCLT, 0x00);  //0~FF
  device->writeRegister(MPR121_FDLT, 0x00);  //0~FF

  // Debounce
  // 5.7 of page 13 of https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
  device->writeRegister(MPR121_DEBOUNCE, 0);  // default at 0

  ESP_LOGI(TAG, "CDC and CDT:");
  ESP_LOGI(TAG, "%d", _config1);
  ESP_LOGI(TAG, "%d", _config2);

  // Charge Current and Charge Time
  // 5.8 of page 14 of https://www.nxp.com/docs/en/data-sheet/MPR121.pdf
  device->writeRegister(MPR121_CONFIG1, _config1);
  device->writeRegister(MPR121_CONFIG2, _config2);

  // enable X electrodes and start MPR121
  byte ECR_SETTING =
    B10000000 + SENSOR_COUNT;  // 5 bits for baseline tracking & proximity disabled + X
  // amount of electrodes running (12)
  device->writeRegister(MPR121_ECR, ECR_SETTING);  // start with above ECR setting
}

// Changes the Threshold Touch value and sets Threshold Release to 1/2 of that value.
// 	Parameters: a value for Threshold Touch.
//  Note: This is the one-stop-shop for hot-swapping values.
void CECO_Ladder::SetThresholdTouchRelease(uint8_t _val, bool _print) {
  // 1. Reset Device
  ResetAndPrepare();

  // 2. Change Threshold Values and update member fields
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, constrain(_val, 0x0, 0xFF));
    device->writeRegister(MPR121_RELEASETH_0 + 2 * i, constrain(_val, 0x0, 0xFF) / 2);
  }
  SaveThresholdFields();

  // 3. Set electrode filter values and start the device
  SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);

  // 4. Inform User of the changes
  if (_print)
    PrintThreshold();
}

// Sets _val as the touch threshold on every electrode.
// 	Parameters: A value for the touch threshold.
void CECO_Ladder::SetThresholdTouch(uint8_t _val, bool _print) {
  ResetAndPrepare();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, constrain(_val, 0x0, 0xFF));
    device->writeRegister(MPR121_RELEASETH_0 + 2 * i, mThresholdRelease[i]);
  }
  SaveThresholdFields('T');
  SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);
  if (_print)
    PrintThreshold('T');
}

// Sets each electrode's touch threshold to its respective value in _arr.
// 	Parameters: An array of touch threshold values to set, the array's size.
// 	Note1: Keeps the threshold release unchanged.
//  Note2: _size defaults to (#define) SENSOR_COUNT.
void CECO_Ladder::SetThresholdTouch(uint8_t* _arr, uint8_t _size, bool _print) {
  ResetAndPrepare();
  for (uint8_t i = 0; i < _size; i++) {
    device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, constrain(_arr[i], 0x0, 0xFF));
    device->writeRegister(MPR121_RELEASETH_0 + 2 * i, mThresholdRelease[i]);
  }
  SaveThresholdFields('T');
  SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);
  if (_print)
    PrintThreshold('T');
}

// Sets _val as the release threshold on every electrode.
// 	Parameters: A value for the release threshold.
void CECO_Ladder::SetThresholdRelease(uint8_t _val, bool _print) {
  ResetAndPrepare();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, mThresholdTouch[i]);
    device->writeRegister(MPR121_RELEASETH_0 + 2 * i, constrain(_val, 0x0, 0xFF));
  }
  SaveThresholdFields('R');
  SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);
  if (_print)
    PrintThreshold('R');
}

// Sets each electrode's release threshold to its respective value in _arr.
// 	Parameters: An array of release threshold values to set, the array's size.
//	Note1: Keeps the threshold touch unchanged.
//  Note2: _size defaults to (#define) SENSOR_COUNT.
void CECO_Ladder::SetThresholdRelease(uint8_t* _arr, uint8_t _size, bool _print) {
  ResetAndPrepare();
  for (uint8_t i = 0; i < _size; i++) {
    device->writeRegister(MPR121_TOUCHTH_0 + 2 * i, mThresholdTouch[i]);
    device->writeRegister(MPR121_RELEASETH_0 + 2 * i, constrain(_arr[i], 0x0, 0xFF));
  }
  SaveThresholdFields('R');
  SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);
  if (_print)
    PrintThreshold('R');
}

// Sets _val as the charge current per cycle of every electrode.
// 	Parameters: An integer value for the charge current.
void CECO_Ladder::SetGlobalChargeCurrent(uint8_t _val, bool _print) {
  ResetAndPrepare();
  SetThresholdTouch(mThresholdTouch, SENSOR_COUNT, false);
  SetThresholdRelease(mThresholdRelease, SENSOR_COUNT, false);
  SetRegistersAndStart(EncodeGlobalCDC(_val), mGlobalChargeTime);
  SaveGlobalChargeFields('C');
  if (_print)
    PrintCONFIG('C');
}

// Sets _val as the charge time per cycle of every electrode.
// 	Parameters: An integer value for the charge time.
void CECO_Ladder::SetGlobalChargeTime(uint8_t _val, bool _print) {
  ResetAndPrepare();
  SetThresholdTouch(mThresholdTouch, SENSOR_COUNT, false);
  SetThresholdRelease(mThresholdRelease, SENSOR_COUNT, false);
  SetRegistersAndStart(mGlobalChargeCurrent, EncodeGlobalCDT(_val));
  SaveGlobalChargeFields('T');
  if (_print)
    PrintCONFIG('T');
}

// Helper method that updates the whole byte of the Threshold member fields.
//  Parameters: (Optional) 'T' for touch-only, 'R' for release-only.
void CECO_Ladder::SaveThresholdFields(char _threshold) {
  _threshold = toupper(_threshold);
  if (_threshold == '\0' || _threshold == 'T') {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      mThresholdTouch[i] = device->readRegister8(MPR121_TOUCHTH_0 + 2 * i);
    }
  }
  if (_threshold == '\0' || _threshold == 'R') {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      mThresholdRelease[i] = device->readRegister8(MPR121_RELEASETH_0 + 2 * i);
    }
  }
}

// Helper method that updates the whole byte of the CONFIG1 asnd CONFIG2 member fields.
//  Parameters: (Optional) 'C' for CONFIG1-only, 'T' for CONFIG2-only.
void CECO_Ladder::SaveGlobalChargeFields(char _charge) {
  _charge = toupper(_charge);
  if (_charge == '\0' || _charge == 'C') {
    mGlobalChargeCurrent = device->readRegister8(MPR121_CONFIG1);
  }
  if (_charge == '\0' || _charge == 'T') {
    mGlobalChargeTime = device->readRegister8(MPR121_CONFIG2);
  }
}

// Soft resets the MPR121 with the currently loaded (volatile) values.
//	Parameters: (Optional) A display State (defaults to enum 0), and
//  (Optional) a bool to print out set values (default: false).
void CECO_Ladder::SoftReset(uint8_t _newState, bool _print) {
  ResetAndPrepare();
  SetThresholdTouch(mThresholdTouch, SENSOR_COUNT, _print);
  SetThresholdRelease(mThresholdRelease, SENSOR_COUNT, _print);
  SetRegistersAndStart(mGlobalChargeCurrent, mGlobalChargeTime);
  currState = (State)_newState;
  // Mock setup() printouts
  Serial.flush();
  Serial.println("Init");
  Serial.println("MPR121 - Complete Setup");
  Serial.println("Changed state to: " + String(currState));
}

// Resets the MPR121 with the stored ESP32-Preferences values of currUser.
//  Parameter: (Optional) A bool to print out set values (default: false).
void CECO_Ladder::SystemReset(bool _print) {
  ResetAndPrepare();
  if (preferences->begin(prf_nameSpaces[currUser], true, prf_labels[0])) {
    uint8_t readTTH[SENSOR_COUNT] = { 0 };
    uint8_t readRTH[SENSOR_COUNT] = { 0 };
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      readTTH[i] = preferences->getUChar(prf_TTH[i]);
      readRTH[i] = preferences->getUChar(prf_RTH[i]);
    }
    SetThresholdTouch(readTTH, SENSOR_COUNT, _print);
    SetThresholdRelease(readRTH, SENSOR_COUNT, _print);
    SaveThresholdFields();
    SetRegistersAndStart(preferences->getUChar(prf_gCDC), preferences->getUChar(prf_gCDT));
    SaveGlobalChargeFields();

    preferences->end();
  }
  // Mock setup() printouts
  Serial.flush();
  Serial.println("Init");
  Serial.println("MPR121 - Complete Setup");
  Serial.println("Changed state to: " + String(currState));
}

// Prints out the available system commands.
//  Manual Add: any new system commands.
void CECO_Ladder::PrintHelp() {
  Serial.println("COMMAND-LINE INTERFACE - HELP");
  Serial.println("NO-DELIMITER COMMANDS:");
  Serial.println(" ?\tPRINT HELP MENU");
  Serial.println(" ?S\tPRINT OUT CURRENT STATE");
  Serial.println(" ?TTR\tPRINT OUT CURRENT THRESHOLD TOUCH AND RELEASE VALUES");
  Serial.println(" ?CCT\tPRINT CHARGE-DISCHARGE CURRENT AND TIME VALUES");
  Serial.println(" ?PRF\tPRINT CURRENTLY STORED ESP32-PREFERENCES VALUES");
  Serial.println(" SAVE\tOVERWRITE ESP32-PREFERENCES VALUES WITH CURRENT USER VALUES");
  Serial.println(" LOAD\tLOAD CURRENT USER ESP32-PREFERENCES VALUES TO THE MPR121");
  Serial.println(" FACT\tSET ALL ESP32-PREFERENCES TO FACTORY DEFAULTS");
  Serial.println(" REBOOT\tRESET THE MPR121 WITH THE STORED ESP32-PREFERENCES VALUES");
  Serial.println(" SR\tSOFT RESET THE MPR121 WITH THE CURRENTLY LOADED VALUES");
  Serial.println("\nDELIMITER COMMANDS:");
  Serial.println(" SU:X\tSWITCH CURRENT USER TO X\t\t[1,2]");
  Serial.println(" SR:X\tSYSTEM RESET INTO STATE X");
  Serial.println(" TTH:X\tSET THRESHOLD TOUCH TO X\t\t[0-255]");
  Serial.println(" RTH:X\tSET THRESHOLD RELEASE TO X\t\t[0-255]");
  Serial.println(" THX:X\tSET THRESHOLD TOUCH TO X, RELEASE = X/2 [0-255]");
  Serial.println(" CDC:X\tSET CHARGE CURRENT TO X \u03BCA\t\t[0-63]");
  Serial.println(" CDT:X\tSET CHARGE TIME TO X \u03BCs\t\t\t[0.5, 1, 2, 4, 8, 16, 32]");
  ESP_LOGE(TAG, "Current Errors (if any):");
}

// Prints out the current display state.
//  Manual Add: any new cases for new (enum) states.
void CECO_Ladder::PrintState() {
  String printState = "";
  switch (currState) {
    case displayAndTouch:
      {
        printState = "Display Data And Touch Status";
        break;
      }
    case displayTouches:
      {
        printState = "Display Touch Status";
        break;
      }
    case displayCalibration:
      {
        printState = "Display Calibration Data";
        break;
      }
    case displayNone:
      {
        printState = "No Display";
        break;
      }
    default:
      {
        printState = "Undefined State";
        break;
      }
  }
  Serial.println("Current State is: " + printState);
}

// Prints the current threshold values by directly reading off of the registers.
// 	Parameters: (Optional) T or R for Touch-only or Release-only, respectively.
void CECO_Ladder::PrintThreshold(char _threshold) {
  _threshold = toupper(_threshold);
  if (_threshold == '\0' || _threshold == 'T') {
    Serial.println("Current Threshold Touch values: ");
    for (uint8_t i = 0; i < SENSOR_COUNT; i += 2) {
      Serial.println("E" + String(i) + ":\t" + String(device->readRegister8(MPR121_TOUCHTH_0 + 2 * i)) + "\tE" + String(i + 1) + ":\t" + String(device->readRegister8(MPR121_TOUCHTH_0 + 2 * i + 2)));
      ESP_LOGI(TAG, "%d:   %d\t%d:   %d", i, mThresholdTouch[i], (i + 1), mThresholdTouch[i + 1]);
    }
  }
  if (_threshold == '\0' || _threshold == 'R') {
    Serial.println("Current Threshold Release values: ");
    for (uint8_t i = 0; i < SENSOR_COUNT; i += 2) {
      Serial.println("E" + String(i) + ":\t" + String(device->readRegister8(MPR121_RELEASETH_0 + 2 * i)) + "\tE" + String(i + 1) + ":\t" + String(device->readRegister8(MPR121_RELEASETH_0 + 2 * i + 2)));
      ESP_LOGI(TAG, "%d:   %d\t%d:   %d", i, mThresholdRelease[i], (i + 1), mThresholdRelease[i + 1]);
    }
  }
}

// Prints the configuration values of CONFIG1 and CONFIG2 by directly reading off of their registers.
// 	Parameters: (Optional) F, C, T, S, or E, for FSI, CDC, CDT, SFI, or ESI, exclusively.
void CECO_Ladder::PrintCONFIG(char _param) {
  _param = toupper(_param);
  if (_param == '\0' || _param == 'F') {
    Serial.println("First Filter Iterations:\t" + DecodedPrintCONFIG1(device->readRegister8(MPR121_CONFIG2), firstFilterIterations));
  }
  if (_param == '\0' || _param == 'C') {
    Serial.println("Charge-Discharge Current:\t" + DecodedPrintCONFIG1(device->readRegister8(MPR121_CONFIG1), chargeDischargeCurrent));
  }
  ESP_LOGI(TAG, "%d", mGlobalChargeCurrent);

  if (_param == '\0' || _param == 'T') {
    Serial.println("Charge-Discharge Time:\t\t" + DecodedPrintCONFIG2(device->readRegister8(MPR121_CONFIG2), chargeDischargeTime));
  }
  if (_param == '\0' || _param == 'S') {
    Serial.println("Second Filter Iterations\t" + DecodedPrintCONFIG2(device->readRegister8(MPR121_CONFIG2), secondFilterIterations));
  }
  if (_param == '\0' || _param == 'E') {
    Serial.println("Electrode Sample Interval:\t" + DecodedPrintCONFIG2(device->readRegister8(MPR121_CONFIG2), electrodeSampleInterval));
  }
  ESP_LOGI(TAG, "%d", mGlobalChargeTime);
}

// Helper function that takes in the complete CONFIG1 register byte and a CONFIG1 parameter (FFI or global CDC)
// and returns a string with the corresponding decoded value.
//  Parameters: The byte value of the MPR121_CONFIG1 (0x5C) register and a CONFIG1 parameter enum.
//  Returns: A string with the decoded value of the choosen CONFIG1 parameter.
String CECO_Ladder::DecodedPrintCONFIG1(byte _byte, CONFIG1Parameter _c1Param) {
  String DecodedC1Parameter = "Invalid C1 Parameter";
  if (_c1Param == firstFilterIterations) {
    uint8_t ffi = ((_byte & 0b1100000000) >> 6);
    if (ffi == 0) {
      DecodedC1Parameter = "  6 samples";
    } else if (ffi == 1) {
      DecodedC1Parameter = " 10 samples";
    } else if (ffi == 2) {
      DecodedC1Parameter = " 18 samples";
    } else if (ffi == 3) {
      DecodedC1Parameter = " 34 samples";
    }
  } else if (_c1Param == chargeDischargeCurrent) {
    uint8_t cdc = ((_byte & 0b00111111) >> 0);
    if (cdc < 10) {
      DecodedC1Parameter = "  " + String(cdc) + " \u03BCA";
    } else {
      DecodedC1Parameter = " " + String(cdc) + " \u03BCA";
    }
  }

  return DecodedC1Parameter;
}

// Helper function that takes in the complete CONFIG2 register byte and a CONFIG2 parameter (global CDT, SFI, or ESI)
// and returns a string with the corresponding decoded value.
//  Parameters: The byte value of the MPR121_CONFIG2 (0x5D) register and a CONFIG2 parameter enum.
//  Returns: A string with the decoded value of the choosen CONFIG2 parameter.
String CECO_Ladder::DecodedPrintCONFIG2(byte _byte, CONFIG2Parameter _c2Param) {
  String DecodedC2Parameter = "Invalid C2 Parameter";
  if (_c2Param == chargeDischargeTime) {
    uint8_t cdt = ((_byte & 0b11100000) >> 5);
    if (cdt == 0) {
      DecodedC2Parameter = "disabled";
    } else if (cdt == 1) {
      DecodedC2Parameter = "0.5 \u03BCs";
    } else if (cdt == 2) {
      DecodedC2Parameter = "  1 \u03BCs";
    } else if (cdt == 3) {
      DecodedC2Parameter = "  2 \u03BCs";
    } else if (cdt == 4) {
      DecodedC2Parameter = "  4 \u03BCs";
    } else if (cdt == 5) {
      DecodedC2Parameter = "  8 \u03BCs";
    } else if (cdt == 6) {
      DecodedC2Parameter = " 16 \u03BCs";
    } else if (cdt == 7) {
      DecodedC2Parameter = " 32 \u03BCs";
    }
  } else if (_c2Param == secondFilterIterations) {
    uint8_t sfi = ((_byte & 0b00011000) >> 3);
    if (sfi == 0) {
      DecodedC2Parameter = "  4 samples";
    } else if (sfi == 1) {
      DecodedC2Parameter = "  6 samples";
    } else if (sfi == 2) {
      DecodedC2Parameter = " 10 samples";
    } else if (sfi == 3) {
      DecodedC2Parameter = " 18 samples";
    }
  } else if (_c2Param == electrodeSampleInterval) {
    uint8_t esi = ((_byte & 0b00000111) >> 0);
    if (esi == 0) {
      DecodedC2Parameter = "  1 ms";
    } else if (esi == 1) {
      DecodedC2Parameter = "  2 ms";
    } else if (esi == 2) {
      DecodedC2Parameter = "  4 ms";
    } else if (esi == 3) {
      DecodedC2Parameter = "  8 ms";
    } else if (esi == 4) {
      DecodedC2Parameter = " 16 ms";
    } else if (esi == 5) {
      DecodedC2Parameter = " 32 ms";
    } else if (esi == 6) {
      DecodedC2Parameter = " 64 ms";
    } else if (esi == 7) {
      DecodedC2Parameter = "128 ms";
    }
  }

  return DecodedC2Parameter;
}

// Prints out the currently stored ESP32-Preferences key-value pairs.
//  Parameters: None.
void CECO_Ladder::PrintPreferences() {
  Serial.println("Currently stored ESP32 Preferences for user: " + String(currUser));
  if (preferences->begin(prf_nameSpaces[currUser], false, prf_labels[0])) {
    Serial.println("namespace:\t\"" + String(prf_nameSpaces[currUser]) + "\"");
    Serial.println("label:\t\t\"" + String(prf_labels[0]) + "\"");

    Serial.println("Threshold Touch:");
    for (uint8_t i = 0; i < SENSOR_COUNT / 2; i++) {
      Serial.print(" ");
      Serial.print(preferences->getUChar(prf_TTH[i]));
    }
    Serial.print("\n");
    for (uint8_t i = SENSOR_COUNT / 2; i < SENSOR_COUNT; i++) {
      Serial.print(" ");
      Serial.print(preferences->getUChar(prf_TTH[i]));
    }
    Serial.print("\n");

    Serial.println("Threshold Release:");
    for (uint8_t i = 0; i < SENSOR_COUNT / 2; i++) {
      Serial.print(" ");
      Serial.print(preferences->getUChar(prf_RTH[i]));
    }
    Serial.print("\n");
    for (uint8_t i = SENSOR_COUNT / 2; i < SENSOR_COUNT; i++) {
      Serial.print(" ");
      Serial.print(preferences->getUChar(prf_RTH[i]));
    }
    Serial.print("\n");

    Serial.println("Global Charge Discharge Current:");
    Serial.print(" ");
    Serial.println(DecodedPrintCONFIG1(preferences->getUChar(prf_gCDC), chargeDischargeCurrent));

    Serial.println("Global Charge Discharge Time:");
    Serial.print(" ");
    Serial.println(DecodedPrintCONFIG2(preferences->getUChar(prf_gCDT), chargeDischargeTime));

    preferences->end();
  } else {
    ESP_LOGE(TAG, "Preferences failed to open.");
  }
}

// Saves the current values of Threshold Touch, Threshold Release, Charge Discharge Current,
// and Charge Discarge Time to its corresponding keys of the ESP32-Preferences key-value pair.
//  Parameters: None.
//  Note1: Does not save if prf_nameSpaces[currUser] is "factory".
void CECO_Ladder::SavePreferences() {
  if (preferences->begin(prf_nameSpaces[currUser], false, prf_labels[0]) && currUser != 0) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      preferences->putUChar(prf_TTH[i], device->readRegister8(MPR121_TOUCHTH_0 + 2 * i));
      preferences->putUChar(prf_RTH[i], device->readRegister8(MPR121_RELEASETH_0 + 2 * i));
    }
    preferences->putUChar(prf_gCDC, device->readRegister8(MPR121_CONFIG1));
    preferences->putUChar(prf_gCDT, device->readRegister8(MPR121_CONFIG2));

    preferences->end();
  }
  Serial.println("Settings stored on NVS.");
}

// Loads the ESP32-Preferences values for Threshold Touch, Threshold Release, Charge Discharge Current,
// and Charge Discarge Time onto the device's temporary (volatile) memory.
//  Parameters: None.
void CECO_Ladder::LoadPreferences() {
  if (preferences->begin(prf_nameSpaces[currUser], false, prf_labels[0])) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      mThresholdTouch[i] = preferences->getUChar(prf_TTH[i]);
      mThresholdRelease[i] = preferences->getUChar(prf_RTH[i]);
    }
    mGlobalChargeCurrent = preferences->getUChar(prf_gCDC);
    mGlobalChargeTime = preferences->getUChar(prf_gCDT);

    preferences->end();
  }
  Serial.println("ESP32-Preferences loaded onto memory.");
  Serial.println(" Please call a SoftReset to enable them.");
}

// Loads the factory default values of Threshold Touch, Threshold Release, Charge Discharge Current,
// and Charge Discarge Time onto the device's temporary (volatile) memory.
//  Parameters: None.
void CECO_Ladder::RestorePreferences() {
  auto tempUser = currUser;
  currUser = 0;
  LoadPreferences();
  currUser = tempUser;
}

// Changes the index for the active ESP32 Preferences namespace string (prf_nameSpaces[i]).
//  Paramenters: the index (1, 2, or 0) to change to.
//  Note1.
//     0 : Factory Defaults.
//     1 : NVS Calibration Settings 1.
//     2 : NVS Calibration Settings 2.
void CECO_Ladder::SwitchUser(uint8_t _user) {
  ESP_LOGI(TAG, "User %d called SwitchUser with parameter: %d", currUser, _user);
  if (preferences->begin(prf_nameSpaces[_user], true, prf_labels[0])) {
    currUser = _user;
    preferences->end();
    if (currUser == 0) {
      Serial.println("Warning: Factory defaults NVS User. SAVE deactivated.");
    }
  } else if (_user < 3) {
    preferences->begin(prf_nameSpaces[_user], false, prf_labels[0]);
    preferences->putUChar(prf_User, _user);
    currUser = _user;
    SavePreferences();
    Serial.println("Current User: " + String(currUser));
    // TODO: Set defaults for the newly created user.
  } else {
    Serial.println("Valid users values are 1, 2, or 0.");
  }
}