#pragma once

#include <Arduino.h>
#include <Adafruit_MPR121.h>
#include <Preferences.h>
#include <esp_log.h>

#define SENSOR_COUNT 12
#define RESET_KEY 0x63
#define MPR121_CURRENT_CHARGE_DEFAULT 0x10
#define MPR121_TIME_CHARGE_DEFAULT 0x20  // Datasheet default: 0x24 // Adafruit defaul: 0x20
#define TAG "DEBUG"


class CECO_Ladder {
  // ENUMS
public:
  enum State {
    displayTouches = 0,
    displayCalibration,
    displayAndTouch,
    displayNone
  };
  enum BootParameter {
    adafruitDefaults = 0,
    esp32Preferences,
    alternativeBoot
  };
  enum CONFIG1Parameter {
    firstFilterIterations = 0,
    chargeDischargeCurrent,
  };
  enum CONFIG2Parameter {
    chargeDischargeTime = 0,
    secondFilterIterations,
    electrodeSampleInterval
  };

  // Fixed string-arrays to use as ESP32-Preferences parameters.
  // Namespaces parameters stored as elements of an array to avoid Namespace-typos.
  //  Note1: Array index 255 may not be used to store a namespace, as
  //  it is used as a creation-check in CECO_Ladder::Init.
  const char* prf_nameSpaces[3] = {
    "factory",
    "calibration1",
    "calibration2"
  };
  // Partition table labels
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html
  const char* prf_labels[1] = {
    "nvs"
  };
  // Keys parameters stored as variables to avoid Key-typos
  //  Note1: Keys must be under 15 characters
  const char* prf_User = "lastUser";
  const char* prf_TTH[12] = {
    "TTH_00", "TTH_01", "TTH_02", "TTH_03", "TTH_04", "TTH_05",
    "TTH_06", "TTH_07", "TTH_08", "TTH_09", "TTH_10", "TTH_11"
  };
  const char* prf_RTH[12] = {
    "RTH_00", "RTH_01", "RTH_02", "RTH_03", "RTH_04", "RTH_05",
    "RTH_06", "RTH_07", "RTH_08", "RTH_09", "RTH_10", "RTH_11"
  };
  const char* prf_gCDC = "globalCDC";
  const char* prf_gCDT = "globalCDT";
  const char* prf_CDC[12] = {
    "CDC_00", "CDC_01", "CDC_02", "CDC_03", "CDC_04", "CDC_05",
    "CDC_06", "CDC_07", "CDC_08", "CDC_09", "CDC_10", "CDC_11"
  };
  const char* prf_CDT[12] = {
    "CDT_00", "CDT_01", "CDT_02", "CDT_03", "CDT_04", "CDT_05",
    "CDT_06", "CDT_07", "CDT_08", "CDT_09", "CDT_10", "CDT_11"
  };

  // MEMBER FIELDS
public:
  State currState = displayTouches;  // made state global to keep it persistant
  uint8_t currUser;                  // to be used for ESP32-Preferences Label (String array) index
  Adafruit_I2CDevice* i2c_dev = NULL;
private:
  byte mThresholdTouch[SENSOR_COUNT] = { 0 };    // stores default threshold touch values
  byte mThresholdRelease[SENSOR_COUNT] = { 0 };  // stores default threshold release values
  byte mGlobalChargeCurrent = 0x0;               // stores the CONFIG1 byte, not just charge-discharge current
  byte mGlobalChargeTime = 0x0;                  // stores the CONFIG2 byte, not just charge-discharge time
  Adafruit_MPR121* device = nullptr;

  // Arduino-esp32 Preferences API instance
  // https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/preferences.html
  Preferences* preferences = nullptr;

  // CONSTRUCTOR
public:
  CECO_Ladder(Adafruit_MPR121* _device, Preferences* _preferences);
  ~CECO_Ladder();

  // PROTOTYPES
private:  // Used internally by the CECO_Ladder class
  // step 1.
  void ResetAndPrepare();
  // step 2.
  void StartThresholdFields(uint8_t _user);
  // step 3.
  void StartGlobalChargeFields(uint8_t _user);
  // step 4.
  void SetRegistersAndStart(byte _config1, byte _config2);

  void SaveThresholdFields(char _threshold = '\0');
  void SaveGlobalChargeFields(char _charge = '\0');
  byte EncodeGlobalCDC(uint8_t _gcdc, uint8_t _ffi = 6);
  byte EncodeGlobalCDT(uint8_t _gcdt, uint8_t _sfi = 4, uint8_t _esi = 16);
  String DecodedPrintCONFIG1(byte _byte, CONFIG1Parameter _c1Param);
  String DecodedPrintCONFIG2(byte _byte, CONFIG2Parameter _c2Param);

public:
  // Used in the .ino
  //void Init(Adafruit_MPR121* _device, Preferences* _preferences);  // step 0. Must be called in setup() REPLACED WITH PARAMETERIZED CONSTRUCTOR
  bool Begin(BootParameter _boot, uint8_t i2caddr = MPR121_I2CADDR_DEFAULT);
  void SetThresholdTouchRelease(uint8_t _val, bool _print = true);
  void SetThresholdTouch(uint8_t _val, bool _print = true);
  void SetThresholdTouch(uint8_t* _arr, uint8_t _size = SENSOR_COUNT, bool _print = true);
  void SetThresholdRelease(uint8_t _val, bool _print = true);
  void SetThresholdRelease(uint8_t* _arr, uint8_t _size = SENSOR_COUNT, bool _print = true);
  void SetGlobalChargeCurrent(uint8_t _val, bool _print = true);
  void SetGlobalChargeTime(uint8_t _val, bool _print = true);
  void SoftReset(uint8_t _newState = 0, bool _print = false);
  void SystemReset(bool _print = false);
  void PrintHelp();
  void PrintState();
  void PrintThreshold(char _threshold = '\0');
  void PrintCONFIG(char _param = '\0');
  void PrintPreferences();
  void SavePreferences();
  void LoadPreferences();
  void RestorePreferences();
  void SwitchUser(uint8_t _user);
};
