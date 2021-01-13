#include <Servo.h>

#include "esp_task_wdt.h"
#include <ArduinoJson.h>
//
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <FS.h>
#include "SPIFFS.h"
//
#include <WiFi.h>
#include "time.h"
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

WebServer server(80);
//
const unsigned int pinThermVcc = 25;
const unsigned int pinThermADC = 35;
const unsigned int pinTSL235RVcc = 12;
const unsigned int pinTSL235Rdata = 34;
const unsigned int pinServo = 32;
const unsigned int pinHeater = 33;
const unsigned int pinFan = 13;

const unsigned int ledR = 14;
const unsigned int ledG = 27;
const unsigned int ledB = 26;
const unsigned int led2 = 2;

const unsigned int ThermR0 = 100000;
const int ThermTemp0 = 25;
const unsigned int ThermBeta = 3950;
const unsigned int ThermRSeries = 100000;

const char* ssid     = "Benicio";
const char* password = "989666195";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3 * 3600;
const int   daylightOffset_sec = 0;

#define FORMAT_SPIFFS_IF_FAILED true

TaskHandle_t Task0;

struct DataLog {
  int tempWell;
  unsigned long time_;
  float data_;
};

void writeFile(fs::FS &fs, const char * path, const char * message);
int readIntFromFile(const char *path);
void appendFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char * path);

class CycleStepper {
    const char* _json = "[{\"iterations\":1,\"cycle\":[{\"temp\":95,\"duration\":15,\"fluorRead\":false}]},{\"iterations\":5,\"cycle\":[{\"temp\":95,\"duration\":3,\"fluorRead\":false},{\"temp\":56,\"duration\":30,\"fluorRead\":true}]}]";
    const char* _cycleJson; // = "[{\"iterations\":1,\"cycle\":[{\"temp\":37,\"duration\":60,\"fluorRead\":false},{\"temp\":31,\"duration\":18,\"fluorRead\":true}]},{\"iterations\":1,\"cycle\":[{\"temp\":95,\"duration\":30,\"fluorRead\":false},{\"temp\":60,\"duration\":30,\"fluorRead\":false},{\"iterations\":2,\"cycle\":[{\"temp\":20,\"duration\":10,\"fluorRead\":false},{\"temp\":22,\"duration\":5,\"fluorRead\":false}]},{\"iterations\":2,\"cycle\":[{\"temp\":72,\"duration\":30,\"fluorRead\":true}]}]},{\"iterations\":2,\"cycle\":[{\"temp\":4,\"duration\":0}]}]";
    enum States {ZERO, TEST_END_REACHED, TEMP_OR_HEADER, TEMP, HEADER} state = ZERO;
    static const unsigned int MAX_SIZE_STATES = 16;
    struct StatesVariables {
      int iterations;
      int index;
    } statesVars[MAX_SIZE_STATES];
    int statesVarsCount;
    struct CurrentCycle {
      float temp;
      float duration;
      bool readFluor;
    } currentCycle;

  public:
    void init(const char* cycleJson) {
      Serial.printf("CycleStepper::init started on core %d\n", xPortGetCoreID());
      char *cycleJson_;
      cycleJson_ = (char*)malloc(strlen(cycleJson)+1);
      strncpy(cycleJson_, cycleJson, strlen(cycleJson)+1);
      _cycleJson = (const char*)cycleJson_;
      Serial.printf("Cyclestepper::init, _cycleJson = %s\n", _cycleJson);
    }
    void getCurrentCycle(float* temp, float* duration, bool*readFluor) {
      *temp = currentCycle.temp;
      *duration = currentCycle.duration;
      *readFluor = currentCycle.readFluor;
    }
    bool nextStep() {
      while (true) {
        DynamicJsonDocument doc(ESP.getMaxAllocHeap());
        doc.clear();
        deserializeJson(doc, _json);
        doc.shrinkToFit();
        Serial.printf("Cyclestepper::nextStep, _cycleJson = %s\n", (const char*)_cycleJson);
        JsonObject root_0_cycle_0 = doc[0]["cycle"][0];
        int root_0_cycle_0_temp = root_0_cycle_0["temp"]; // 95
        Serial.printf("root_0_cycle_0_temp = %d\n", root_0_cycle_0_temp);
        
        JsonArray arr = doc.as<JsonArray>();
        JsonArray::iterator it = arr.begin();
        for (int i = 0; i < (statesVarsCount - 1); i++) {
//          Serial.println((char*)(*it));
          for (int j = 0; j < statesVars[i].index; j++) ++it;
          JsonObject obj = (*it); // testar se realmente eh necessario, ja que it eh um ponteiro
          arr = obj["cycle"];
          it = arr.begin();
        }
        for (int i = 0; i < statesVars[statesVarsCount - 1].index; i++) {
          ++it;
        };
        JsonObject obj = (*it);
        Serial.printf("state = %d\n", state);
        switch (state) {
          case ZERO: {            
              for (int i = 0; i < MAX_SIZE_STATES; i++) {
                statesVars[i].index = 0;
                statesVars[i].iterations = 0;
              }
              statesVars[0].iterations = obj["iterations"];
              statesVarsCount = 1;
              state = TEST_END_REACHED;
              break;
            }
          case TEST_END_REACHED: {
              state = TEMP_OR_HEADER;
              if (it == arr.end()) {
                statesVars[statesVarsCount - 1].index = 0;
                if ((statesVarsCount == 1) && statesVars[0].iterations == 1) return false;
                if (statesVars[statesVarsCount - 2].iterations == 1) {
                  statesVars[statesVarsCount - 2].index++;
                  statesVarsCount--;
                } else {
                  statesVars[statesVarsCount - 2].iterations--;
                }

                state = TEST_END_REACHED;
              }
              break;
            }
          case TEMP_OR_HEADER: {
              float temp = obj["temp"];
              float header = obj["iterations"];
              if (temp) {
                state = TEMP;
              } else if (header) {
                state = HEADER;
              }
              break;
            }
          case TEMP: {
              currentCycle.temp = obj["temp"];
              currentCycle.duration = obj["duration"];
              currentCycle.readFluor = obj["fluorRead"];
              statesVars[statesVarsCount - 1].index++;
              state = TEST_END_REACHED;
              Serial.printf("currentCycle:\n\ttemp: %f\n\tduration: %f\n\treadFluor: %s\n\n", currentCycle.temp, currentCycle.duration, currentCycle.readFluor ? "true" : "false");
              return true;
            }
          case HEADER: {
              statesVars[statesVarsCount - 1].iterations = obj["iterations"];
              statesVars[statesVarsCount].index = 0;
              statesVarsCount++;
              state = TEST_END_REACHED;
              break;
            }
        }
      }
    }
    void finalize() {
      free((void*)_cycleJson);
    }
};
class Thermistor {
  public:
    unsigned int _pinADC;
    unsigned int _pinVcc;
    unsigned int _R0;
    float _temp0;
    unsigned int _beta;
    unsigned int _RSeries;
    void init(unsigned int Vcc, unsigned int ADC, unsigned int R0, float temp0, unsigned int beta, unsigned int RSeries) {
      _pinADC = ADC;
      _pinVcc = Vcc;
      _R0 = R0;
      _temp0 = temp0;
      _beta = beta;
      _RSeries = RSeries;
      pinMode(_pinADC, INPUT);
      pinMode(_pinVcc, OUTPUT);

    }
    float readTemperature(unsigned int counts) {
      unsigned int sumReadings = 0;
      digitalWrite(_pinVcc, HIGH);
      float temp = 0;
      for (unsigned int i = 0; i < counts; i++) {
        do
        {
          temp = analogRead(_pinADC);
        } while (!temp);
        sumReadings += temp;
      }
      digitalWrite(_pinVcc, LOW);
      temp = sumReadings / counts;
      temp = 4095 * _RSeries / temp - _RSeries;
      temp /= _R0;
      temp = log(temp);
      temp /= _beta;
      temp += 1 / (_temp0 + 273.15);
      temp = 1 / temp;
      temp -= 273.15;
      return temp;
    }
};
class TSL235R {
  public:
    unsigned int _pinVcc;
    unsigned int _pinData;
    unsigned long _nPulses;

    void init(unsigned int Vcc, unsigned int datap, unsigned long nPulses) {
      _pinVcc = Vcc;
      _pinData = datap;
      _nPulses = nPulses;
      pinMode(_pinVcc, OUTPUT);
      pinMode(_pinData, INPUT);

    }
    float readLight() {
      digitalWrite(_pinVcc, HIGH);
      unsigned long counts = 0;
      unsigned long startTime = 0;
      unsigned long endTime = 0;
      noInterrupts();
      bool state = digitalRead(_pinData);
      bool oldState = state;
      while (counts <= _nPulses) {
        state = digitalRead(_pinData);
        if (state != oldState) {
          oldState = state;
          if (state) {
            if (!counts) startTime = micros();
            counts++;
          }
        }
      }
      endTime = micros();
      interrupts();
      unsigned long interval = endTime - startTime; // What if endTime < startTime?
      double freq = 10000000 * counts / interval;
      return freq;
    }
};
class PID {
    float _error = 0, _integral = 0, _derivative = 0;
    float _kP = 30000, _kI = 0, _kD = 1000;
    float _setpoint = 0;
    unsigned long _lastUpdate = 0;
    int _lowerLimit = 0;
    int _upperLimit = 65535;

  public:
    void setPID(float kP, float kI, float kD) {
      _kP = kP;
      _kI = kI;
      _kP = kP;
    }
    void setSetpoint(float setpoint) {
      _setpoint = setpoint;
    }
    float getSetpoint() {
      return _setpoint;
    }
    void setLimits(int lower, int upper) {
      _lowerLimit = lower;
      _upperLimit = upper;
    }
    int output(int temp) {
      float newError = _setpoint - temp;
      if (_lastUpdate) {
        unsigned long dt = millis() - _lastUpdate;
        _integral += newError * dt;
        if (_integral >= _upperLimit)
          _integral = _upperLimit;
        _derivative = (newError - _error) / dt;
      }
      float P = _kP * newError;
      float I = _kI * _integral;
      float D = _kD * _derivative;
      float output = P + I + D;
      if (output <= _lowerLimit)
        output = _lowerLimit;
      if (output >= _upperLimit)
        output = _upperLimit;
      _error = newError;
      _lastUpdate = millis();
      return output;
    }
};
class DBase {
    void displayFreeHeap() {
       Serial.printf("Heap size: %d\tFree Heap: %d\tMin Free Heap: %d\tMax Alloc Heap: %d\n", ESP.getHeapSize(), heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }
/*    void createTables() {
      char* sql;
      sql = "CREATE TABLE IF NOT EXISTS logs (name, cycleID, creationDate)";
      db_exec(sql, NULL, 0);
      sql = "CREATE TABLE IF NOT EXISTS logsWells (logID, name, well)";
      db_exec(sql, NULL, 0);
      sql = "CREATE TABLE IF NOT EXISTS logsData (logID, tempWell, time, data)";
      db_exec(sql, NULL, 0);
      sql = "CREATE TABLE IF NOT EXISTS cycles (name, description, creationDate, cycle)";
      db_exec(sql, NULL, 0);
      return;
    }*/

  public:
    DBase() {
      Serial.print(F("\nDB started on core "));
      Serial.println(xPortGetCoreID());
      
      if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        Serial.println(F("Failed to mount file system"));
        return;
      }
      // list SPIFFS contents
      File root = SPIFFS.open("/");
      if (!root) {
        Serial.println(F("- failed to open directory"));
        return;
      }
      if (!root.isDirectory()) {
        Serial.println(F(" - not a directory"));
        return;
      }
    }
    int addLog(const char* logName, int cycleID, unsigned int wellsCount, char* wellsNames[], int wellsNumbers[]) {
      Serial.printf("\naddLog started on core %d\n", xPortGetCoreID());
      int lastIndex = readIntFromFile("/logs/lastIndex");
      if(lastIndex == -1) {
        Serial.printf("Error reading /logs/lastIndex\n");
        return -1;
      }
      char path[25];
      snprintf(path, sizeof(path), "/logs/%d.head", lastIndex);
      char dat[200];
      snprintf(dat, sizeof(dat), "%s,%d,%d\n", logName, cycleID, wellsCount); // INSERIR TB A DATA DE CRIACAO
      writeFile(SPIFFS, path, dat);
      for (int i = 0; i < wellsCount; i++) {
        snprintf(dat, sizeof(dat), "%s,%d\n", wellsNames[i], wellsNumbers[i]);
        appendFile(SPIFFS, path, dat);
      }
      snprintf(dat, sizeof(dat), "%d", lastIndex+1);
      writeFile(SPIFFS, "/logs/lastIndex", dat);
      Serial.printf("log created succesfully!\n");
      displayFreeHeap();
      return lastIndex;
    }
    void addDatapoints(int logID, struct DataLog *datalog) {
      Serial.printf("\naddDatapoints started on core %d\n", xPortGetCoreID());

      char path[25];
      snprintf(path, sizeof(path), "/logs/%d.data", logID);
      char dat[200];
      snprintf(dat, sizeof(dat), "%d,%d,%lu,%f\n", logID, datalog->tempWell, datalog->time_, datalog->data_);
      appendFile(SPIFFS, path, dat);
      displayFreeHeap();
    }
    void addCycle(char* name_, char* description, char* cycle) {
      Serial.printf("\naddCycle started on core %d\n", xPortGetCoreID());

      int lastIndex = readIntFromFile("/cycles/lastIndex");
      if(lastIndex == -1) {
        Serial.print(F("Error reading int from file\n"));
        return;
      }
      char path[25];
      snprintf(path, sizeof(path), "/cycles/%d.head", lastIndex);
      char dat[200];
      snprintf(dat, sizeof(dat), "%s,%s", name_, description); // INSERIR TB A DATA DE CRIACAO
      writeFile(SPIFFS, path, dat);

      snprintf(path, sizeof(path), "/cycles/%d.json", lastIndex);
      writeFile(SPIFFS, path, cycle);

      snprintf(dat, sizeof(dat), "%d", lastIndex+1);
      writeFile(SPIFFS, "/cycles/lastIndex", dat);
      server.send(200, "text/plain", "Inserido");
      Serial.printf("cycle inserted succesfully!\n");
      displayFreeHeap();
    }
    bool getCyclesHeads(unsigned int idMin, unsigned int idMax) {
      int lastIndex = readIntFromFile("/cycles/lastIndex");
      if(lastIndex == -1) {
        Serial.print(F("Error reading int from file\n"));
        return false;
      }
      int realIDMax = (idMax < lastIndex) ? idMax : lastIndex;
      char path[25];
      server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      server.send (200, "text/plain", "");
      for (int i = idMin; i < realIDMax; i++) {
        snprintf(path, sizeof(path), "/cycles/%d.head", i);
        File file = SPIFFS.open(path);
        if(!file){
            Serial.println("Failed to open file for reading");
            return false;
        }
        while(file.available()){
            server.sendContent(file.readString());
            server.sendContent("\n");
        }
        file.close();
      }
      server.sendContent("");
      Serial.println("Terminou a getCyclesHeads");
      displayFreeHeap();
      return true;
    }
    bool getCycleBody(unsigned int id, char** cycle) {
      Serial.print(F("getCycleBody\n"));
      char path[25];
      snprintf(path, sizeof(path), "/cycles/%d.json", id);
      
      if(!SPIFFS.exists(path)) {
        Serial.print(F("Cycle not found!\n"));
        return false;
      }
      File file = SPIFFS.open(path);
      if(!file){
          Serial.println("Failed to open file for reading");
          return false;
      }
      *cycle = (char*)malloc(file.size() + 1);
      strncpy(*cycle, file.readString().c_str(), file.size() + 1);
      file.close();
      Serial.printf("getCycleBody, cycle = %s\n", *cycle);
      displayFreeHeap();
      return true;
    }
    void getLogsHeads(unsigned int idMin, unsigned int idMax) {
      int lastIndex = readIntFromFile("/cycles/lastIndex");
      if(lastIndex == -1) {
        Serial.print(F("Error reading int from file\n"));
        return;
      }
      int realIDMax = (idMax < lastIndex) ? idMax : lastIndex;
      char path[25];
      server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      server.send (200, "text/plain", "");
      for (int i = idMin; i < realIDMax; i++) {
        snprintf(path, sizeof(path), "/logs/%d.head", i);
        File file = SPIFFS.open(path);
        if(!file){
            Serial.println("Failed to open file for reading");
            return;
        }
        while(file.available()){
            server.sendContent(file.readString());
        }
        file.close();
      }
      server.sendContent("");
      Serial.println("Terminou a getLogsHeads");
      displayFreeHeap();
    }
    void getLogData(unsigned int id) {
      char path[30];
      snprintf(path, sizeof(path), "/%d.dat", id);
      if (!SPIFFS.exists(path)) {
        Serial.println("Log not found!");
        server.send(200, "text/plain", "Log not found!");
        return;
      }
      File file = SPIFFS.open(path, "r");
      server.streamFile(file, "text/plain");
//      readFile(SPIFFS, path);
      file.close();
      displayFreeHeap();
      return;
    
    }
};
class ThermalCycler {
    Thermistor thermSensor;
    TSL235R lightSensor;
    Servo servo;
    DBase* _db;
    PID pid;
    CycleStepper cycleStepper;
    int* _wells;
    int _wellsCount;
    int _logID;
    char _logPath[30];
  public:
    bool activeFlag = false;
    bool init(int cycleID, char runName[], DBase* db, int* wells, char *wellsNames[], int wellsCount) {
      Serial.printf("ThermalCycler::init started on core %d\n", xPortGetCoreID());
      if(!SPIFFS.begin()){
        Serial.println("Card Mount Failed");
        return false;
      }
      char* cycle = NULL;
      if (!(_db->getCycleBody(cycleID, &cycle))) {
        return false;
      }
      cycleStepper.init((const char*)cycle);
      free(cycle);
      _logID = ((DBase*)db)->addLog(runName, cycleID, wellsCount, wellsNames, wells);
      snprintf(_logPath, sizeof(_logPath), "/%d.dat", _logID);
      deleteFile(SPIFFS, _logPath);
      _db = db;
      _wells = (int*)malloc(wellsCount*sizeof(int));
      if (_wells == NULL) {
        Serial.printf("Nao mallocou o _wells!\n");
        return false;
      }
      for (int i = 0; i < wellsCount; i++) {
        _wells[i] = wells[i];
      }
      _wellsCount = wellsCount;
      servo.attach(pinServo, 0/*Servo::CHANNEL_NOT_ATTACHED*/, 0, 180);
      ledcSetup(1, 5000, 16);
      ledcAttachPin(pinHeater, 1);
      thermSensor.init(pinThermVcc, pinThermADC, ThermR0, ThermTemp0, ThermBeta, ThermRSeries);
      lightSensor.init(pinTSL235RVcc, pinTSL235Rdata, 1000);
      return true;
    }
    void runPCR() {
      Serial.printf("ThermalCycler::runPCR started on core %d\n", xPortGetCoreID());
      char datalog[100];
      activeFlag = true;
      float thermReading = thermSensor.readTemperature(100);
      pid.setSetpoint(thermReading);
      unsigned long delayServo = millis();
      unsigned long timeTotal = millis();
      unsigned long timeStartRun = millis();
      digitalWrite(led2, HIGH);
      while (cycleStepper.nextStep()) {
        float temp, duration;
        bool readFluor = false;
        cycleStepper.getCurrentCycle(&temp, &duration, &readFluor);
        pid.setSetpoint(temp);
        bool rising = ((thermReading <= temp) ? true : false);
        Serial.print("rising = ");
        Serial.println(rising);
        bool reached = false;
        unsigned long timeStartStage = millis();
        unsigned long timeStartServo = millis();
        int iWell = 0;
        bool waitingServo = false;
        while (true) {
          delay(200); // TESTE
          thermReading = thermSensor.readTemperature(1000);
          timeTotal = millis();
          snprintf(datalog, sizeof(datalog), "0,%lu,%f\n", (timeTotal - timeStartRun), thermReading);
          appendFile(SPIFFS, _logPath, datalog);
          int currentPid = pid.output(thermReading);
          ledcWrite(1, currentPid);
          Serial.printf("time = %lu\ttemp = %f\tcurrentPID = %d\tsetpoint = %f\tduration = %f\n", (timeTotal - timeStartRun), thermReading, currentPid, temp, duration);
          if (!currentPid) digitalWrite(pinFan, HIGH);
          if (reached) {
            if ((millis() - timeStartStage) >= (duration * 1000)) {
              if (!readFluor) break;
              // Reading fluorescence:
              if (!waitingServo) {
                timeStartServo = millis();
                servo.write((_wells[iWell]-1)*72); // 72 = 360 / 5
                Serial.printf("servo position: %d\n", (_wells[iWell]-1)*72);
                waitingServo = true;
              }
              if ((millis() - timeStartServo) < 500) continue; // 500 is a delay to wait the servo to reach its position
              digitalWrite(ledB, HIGH);
              float fluorReading = lightSensor.readLight();
              digitalWrite(ledB, LOW);
              Serial.printf("time = %lu\tfluor = %f\n", (millis() - timeStartRun), fluorReading);
              snprintf(datalog, sizeof(datalog), "%d,%lu,%f\n", _wells[iWell], (millis() - timeStartRun), fluorReading);
              appendFile(SPIFFS, _logPath, datalog);
              waitingServo = false;
              iWell++;
              if (iWell >= _wellsCount) break;
            }
            continue;
          }
          if (rising && (thermReading < temp)) continue;
          if ((!rising) && (thermReading > temp)) continue;
          reached = true;
          timeStartStage = millis();
        }
      }
      activeFlag = false;
      digitalWrite(led2, LOW);
      free(_wells);
      cycleStepper.finalize();
      Serial.println("Encerrando TaskThermalCycler...");
      vTaskDelete(NULL);
    }
};

ThermalCycler thermalCycler;

// Request handlers:
/*
//void addLogReqHandler(DBase * db) {
//  Serial.printf("\naddLogReqHandler running on core %d\n", xPortGetCoreID());
//  if (server.hasArg("logName") && server.hasArg("cycleID") && server.hasArg("wellsCount") && server.hasArg("wellName") && server.hasArg("wellNumber")) {
//    const char* logName = server.arg("logName").c_str();
//    Serial.printf("logName = %s\n", logName);
//    int cycleID = server.arg("cycleID").toInt();
//    Serial.printf("cycleID = %d\n", cycleID);
//    int params = server.args();
//    unsigned int wellsCount = server.arg("wellsCount").toInt();
//    Serial.printf("wellsCount = %d\n", wellsCount);
//    const char* wellsNames[128];
//    int wellsNumbers[wellsCount];
//    int j = 0;
//    int k = 0;
//    Serial.printf("addLogReqHandler, 0\n");
//    for (int i = 0; i < params; i++) {
//      String p = server.arg(i);
//      if (p == "wellName") {
//        //        const char* wellNameTemp = ;
//        *(wellsNames + j) = p.c_str();
//        j++;
//      } else if (p == "wellNumber") {
//        wellsNumbers[k] = p.toInt();
//        k++;
//      }
//    }
//    if (!(j == k && j == wellsCount)) {
//      Serial.printf("addLogReqHandler: wellsCount != wellsNames and wellsNumbers size!\n");
//      server.send(200, "text/plain", "addLogReqHandler: wellsCount != wellsNames and wellsNumbers size!");
//      return;
//    }
//    db->addLog(logName, cycleID, wellsCount, wellsNames, wellsNumbers);
//  } else {
//    server.send(200, "text/plain", "Required parameter not received!");
//  }
//}
*/
void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}
int readIntFromFile(const char * path){
    Serial.printf("Reading file: %s\n", path);
    fs::FS &fs = SPIFFS;
    int n;
    if (!SPIFFS.exists(path)) {
      n = 0;
    } else {
      File file = fs.open(path);
      if(!file){
        Serial.println("Failed to open file for reading");
        return -1;
      }
      n = file.parseInt();
      file.close();
    }
    return n;
}
void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}
void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void addCycleReqHandler(DBase* db) {
  Serial.print("\naddCycleReqHandler running on core ");
  Serial.println(xPortGetCoreID());
  if (server.hasArg("cycleName") && server.hasArg("description") && server.hasArg("cycle")) {
    char cycleName[server.arg("cycleName").length()+1];
    strncpy(cycleName, server.arg("cycleName").c_str(), sizeof(cycleName));
    Serial.printf("length: %d\tcycleName = %s\n", server.arg("cycleName").length(), cycleName);
    char description[server.arg("description").length()+1];
    strncpy(description, server.arg("description").c_str(), sizeof(description));
    Serial.printf("description = %s\n", description);
    char cycle[server.arg("cycle").length()+1];
    strncpy(cycle, server.arg("cycle").c_str(), sizeof(cycle));
    Serial.printf("cycle = %s\n", cycle);
    db->addCycle(cycleName, description, cycle);
    server.send(200, "text/plain", "Vc mandou um addCycle");
    //Conferir se mandou mesmo!
  } else {
    server.send(200, "text/plain", "Required parameter not received!");
  }
}
void getCyclesHeadsReqHandler(DBase* db) {
  Serial.print("\ngetCyclesHeadsReqHandler running on core ");
  Serial.println(xPortGetCoreID());
  if (server.hasArg("idMin") && server.hasArg("idMax")) {
    int idMin = (server.arg("idMin")).toInt();
    int idMax = (server.arg("idMax")).toInt();
    if(db->getCyclesHeads(idMin, idMax)) {
      server.send(200, "text/plain", "erro ao processar o getCyclesHeads");
    } else {
      Serial.printf("getCyclesHeads realizado com sucesso!\n");
//      server.send(200, "text/plain", "vc mandou um getCyclesHeads");
    }
  } else {
    server.send(200, "text/plain", "Required parameter not received!");
  }
}
void getCycleBodyReqHandler(DBase* db) {
  Serial.print("\ngetCycleBodyReqHandler running on core ");
  Serial.println(xPortGetCoreID());
  if (server.hasArg("id")) {
    int id = server.arg("id").toInt();
    char *cycleBody = NULL;
    if(db->getCycleBody(id, &cycleBody)) {
      Serial.printf("getCycleBodyReqHandler, cycleBody = %s\n", cycleBody);
      server.send(200, "text/plain", cycleBody);
      free(cycleBody);
    } else {
      Serial.print(F("getCycleBodyReqHandler, getCycleBody returned false"));
      server.send(200, "text/plain", "Ciclo não encontrado");
    }
  } else {
    server.send(200, "text/plain", "Required parameter not received!");
  }
}
void getLogsHeadsReqHandler(DBase* db) {
  Serial.print("\ngetLogsHeadsReqHandler running on core ");
  Serial.println(xPortGetCoreID());
  if (server.hasArg("idMin") && server.hasArg("idMax")) {
    int idMin = server.arg("idMin").toInt();
    int idMax = server.arg("idMax").toInt();
    db->getLogsHeads(idMin, idMax);
  } else {
    server.send(200, "text/plain", "Required parameter not received!");
  }
}
void getLogDataReqHandler(DBase* db) {
  Serial.print("\ngetLogDataReqHandler running on core ");
  Serial.println(xPortGetCoreID());
  if (server.hasArg("id")) {
    int id = server.arg("id").toInt();
    db->getLogData(id);
  } else {
    server.send(200, "text/plain", "Required parameter not received!");
  }
}
void indexReqHandler() {

  Serial.print("\nindexReqHandler running on core ");
  Serial.println(xPortGetCoreID());
  server.send(200, "text/plain", "Hello, world");
}
void runPCRreqHandler(DBase* db) {
  if(thermalCycler.activeFlag) {
    server.send(200, "text/plain", "Machine is busy!");
    return;
  }
  if (!(server.hasArg("runName") && server.hasArg("cycleID") && server.hasArg("wellsCount") && server.hasArg("wellName") && server.hasArg("wellNumber"))) {
    server.send(200, "text/plain", "Required parameter not received!");
    return;
  }

  char runName[server.arg("runName").length()+1];
//  char runName[128];
  strncpy(runName, server.arg("runName").c_str(), sizeof(runName));

  int cycleID = server.arg("cycleID").toInt();
  int argsCount = server.args();
  int wellsCount = (argsCount - 2)/2; // total params - runName and cycleID
  Serial.printf("runPCRreqHandler {runName = %s\tcycleID = %d\twellsCount = %d}\n", runName, cycleID, wellsCount);
//  String wellsNames[wellsCount];
  char* wellsNames[wellsCount];
  int wellsNumbers[wellsCount];
  int j = 0;
  int k = 0;
  for (int i = 0; i < argsCount; i++) {
//    String p = server.arg(i);
    if (server.argName(i) == "wellName") {
      if (j >= wellsCount) break;
      wellsNames[j] = (char*)malloc(server.arg(i).length()+1);
      strncpy(wellsNames[j], server.arg(i).c_str(), server.arg(i).length()+1);
//      wellsNames[j] = p;
      j++;
    } else if (server.argName(i) == "wellNumber") {
      if (k >= wellsCount) break;
      wellsNumbers[k] = server.arg(i).toInt();
      k++;
    }
  }
  if (!(j == k && j == wellsCount)) {
    Serial.printf("addLogReqHandler: wellsCount != wellsNames and wellsNumbers size!\n");
    server.send(200, "text/plain", "addLogReqHandler: wellsCount != wellsNames and wellsNumbers size!");
    return;
  }

  if(thermalCycler.init(cycleID, runName, db, wellsNumbers, wellsNames, wellsCount)) {
    xTaskCreatePinnedToCore([](void*) -> void {
      thermalCycler.runPCR();
    }, "_TaskThermalCycler_", 5000, NULL, 1, &Task0, 0);  
  } else {
    server.send(200, "text/plain", "thermalCycler.init error");
  }
  for (int i = 0; i < wellsCount; i++) {
    free(wellsNames[i]);
  }
//  queueSQL = xQueueCreate(queueMaxSize, sizeof(struct DataLog));
  
  
}
void notFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  Serial.print("\nsetup started on core ");
  Serial.println(xPortGetCoreID());
  disableCore0WDT();
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(pinHeater, OUTPUT);
  digitalWrite(pinHeater, LOW);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if (MDNS.begin("Magnus_qPCR")) {
    Serial.println("MDNS responder started");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //
  DBase db;
  //
  server.on("/", HTTP_GET, indexReqHandler);
//  server.on("/addLog", HTTP_GET, [&db]() {
//    addLogReqHandler(&db);
//  });
  server.on("/addCycle", HTTP_GET, [&db]() {
    addCycleReqHandler(&db);
  });
  server.on("/getCyclesHeads", HTTP_GET, [&db]() {
    getCyclesHeadsReqHandler(&db);
  });
  server.on("/getCycleBody", HTTP_GET, [&db]() {
    getCycleBodyReqHandler(&db);
  });
  server.on("/getLogsHeads", HTTP_GET, [&db]() {
    getLogsHeadsReqHandler(&db);
  });
  server.on("/getLogData", HTTP_GET, [&db]() {
    getLogDataReqHandler(&db);
  });
  server.on("/runPCR", HTTP_GET, [&db]() {
    runPCRreqHandler(&db);
  });
  server.onNotFound(notFound);
  Serial.println("Chegou no final do setup");
  //
  server.begin();

  delay(500);
}

void loop() { 
  server.handleClient();
}
