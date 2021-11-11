/*
*******************************************************************************************************
*****************   DOIT-AM 8266MOD / 2 each BME280 / SERVO / IC2 OLED 128 x 32 / water sensor
*******************************************************************************************************
  0)
      try to equalize inside and outide airpressure for house.  Reduce outside air leaking into house
      when Dryer or gas Furnance or gas Hot Water tank are runing. i.e. if we exhaust air from house try to equalize
      Also: test for water in hot water emargeny pan/sound alarm if true

  1) Equal house internal pressure : attempt to reduce drafts
    1.1) measure air pressure outside "mySensorA": baro upstream of damper (decide whether we need third barometer out back)
    1.2) measure air pressure furnace room "mySensorB": baro downstream of damper
    1.3) compare air pressures and if delta >= threshold then open air damper vals.servoIncrement degrees per UPDATE_MILLIS milliseconds until full open
    1.4) once damper full open : gradually increase intake hose fan speed : vals.fanIncrement % per UPDATE_MILLIS ms

  2)
    Water usage : Hot and total : display using remote esp8266 & OLED) 1 each in Shower & Kitchen & Laundry Room.
    2.1) measure water flow - @ water main in house
    2.2) measure water flow - @ hot water tank intake

  3)
    Hot water tank leak
    3.1)  check water exists in water safety heater pan : sound alarm if true

  4)
    Publish web page
    4.1) inside/outside : BMP280: Temp, Baro, Humidity(if using BME280)
    4.2) Water usage : Hot/Main : Total since last bill date; since 00:00:00 today
    4,3) Water flow rate: Hot/Cold: GPM
    4.4) Hot Water Tank Leak (True/False)
    4.5) Draft damper posistion degrees; Draft fan Speed %

  5)
    Display OLED (#4) : Report at slaves
    5.1) Kitchen esp8266 & OLED : Process WebSock-JSON
    5.2) Shower esp8266 & OLED : Process WebSock-JSON
    5.3) Laundry Room I2C : OLED LCD 128x32 /LCD : SSD1306LCD  0x3C

NOTE:
    Web Sites:
      Goes into auto AP Mode if invalid WiFi credentials: AP: Air Pressure: 10.0.1.17
      When running: DHCP or user defined during credentials during AP MODE: IP : status page; IP/settings system settings page
      Supports OTA firmare updates in normal run mode and in AP WiFi Setup mode: http://10.0.1.17/uipdate or http://ip/update respectively
*/

// Arduino, I2C, OLED, BME/BMP280, Servo, PWM computer fan 3-4"
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Servo.h>

#define TINY_BME280_I2C
#include <TinyBME280.h>

// WEB
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <index.h>
#include <NTPClient.h>            // Include NTPClient library
#include <WiFiUdp.h>
#include <time.h>
#include <espnow.h>

// OTA
#include <AsyncElegantOTA.h>

#define TICKS_PER_GALLON_MAIN 1  // To Be Figured Out
#define TICKS_PER_GALLON_HOT  1  // need to figure out
#define UPDATE_MILLIS       3000   // How often to update values : struct: get temps, compute flows, compute accumated

#define SCL 5               // oled, bmp1, bmp2
#define SDA 4               // oled, bmp1, bmp2
#define SERVO_PIN 12         // pwm signal to damper servo
#define DRAFT_FAN_PIN 13      // pwm signal to draft fan
#define MAIN_FLOW_PIN 14      // flow pulses from main water, interupt
#define HOT_FLOW_PIN 15       // flow pulses from hot water, interupt && hotFlowPin
#define LEAK_DETECTOR_PIN A0  // water leak sensor input pin
#define LEAK_LIGHT_PIN    2

#define MAX_DAMPER 155
#define MIN_DAMPER 35
#define MAX_FAN 100
#define MIN_FAN 0
#define DAMPER_INCREMENT 3
#define FAN_INCREMENT 5
#define WATER_LEAK_THRESHOLD 900

const char AP_NAME[]="Air Pressure: 10.0.1.17";
const byte DNS_PORT  = 53;
bool AP_MODE=false;
int bufferHotTicks, bufferMainTicks; // added to reduce disable interupt period.

volatile unsigned int mainFlowTicks=0; //measuring the rising edges of the signal
volatile unsigned int hotFlowTicks=0;  //measuring the rising edges of the signal

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -8*60*60, 24*60*60*1000); // update 1 per day

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncWebSocket webSock("/");

//prototypes
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void increaseAirFlow();
void decreaseAirFlow();
void notFound(AsyncWebServerRequest *request);
std::string valFromJson(const std::string &json, const std::string &element);
bool wifiConnect(WiFiMode m);
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
bool saveBounds();
void setBounds(const std::string& s);
bool initLocalStruct();
bool initCreds();
void setCreds(const std::string& s);
bool saveCreds();

IPAddress  // soft AP IP info
          ip_STA(10,0,0,17)
        , ip_AP(10,0,1,17)
        , ip_AP_GW(10,0,1,17)
        , ip_subNet(255,255,255,128);

// ESP_NOW broadcast
uint8_t peer[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // MAC: MultiCast"


IRAM_ATTR void pulseCounter(){  // interupt on mainFlowPin
      if(digitalRead(HOT_FLOW_PIN))hotFlowTicks+=1;
      mainFlowTicks+=1;
      Serial.printf("ISR: %i  *****", mainFlowTicks);Serial.flush();
}

typedef struct WifiCreds_t{
      String    SSID;
      String    PWD;
      bool      isDHCP;
      IPAddress IP;
      IPAddress GW;
      IPAddress MASK;

      std::string toStr(){
        char s[150];
        sprintf(s, "{\"SSID\":%s,\"PWD\":%s,\"isDHCP\":%s,\"IP\":%s,\"GW\":%s,\"MASK\":%s", SSID.c_str(), PWD.c_str(), isDHCP?"true":"false", IP.toString().c_str(), GW.toString().c_str(), MASK.toString().c_str());
        return(s);
      }
} WifiCreds_t;
WifiCreds_t creds;

typedef struct measurements{
            float oat;              // outside air temp;
            float oap;              // outside air pressure: 
            int   oah;              // outside humidity
            float iat;              // inside air temp
            float iap;              // inside air pressure
            int   iah;              // inside humidity
            int      draftDamperPos;   // Servo degrees
            int      draftFanPct;      // Fan Speed Percent
            int      waterSensor;      // water tank : > 900 == leakining?
            int      mainGallonsTotal; // total for this Month
            int      mainGallonsToday; // total since 00:00     Zeroed on day update in loop()
            int      mainFlowGPM;      // flow = (ticks*(60000/(millis()-lastMillis)))/TICKS_PER_GALLON
            int      hotGallonsTotal;  // total this month.
            int      hotGallonsToday;  // total since 00:00     Zeroed on day update in loop()
            int      hotFlowGPM;       // flow = (ticks*(60000/(millis()-lastMillis)))/TICKS_PER_GALLON
       unsigned long lastMillis;       // last update time millis()
            int      today;            // Sunday == 0: updated in loop()
            float    pressureDelta;    // trigger value for damper & fan: OAP-IAP > pressureDelta
            int      waterAlert     = WATER_LEAK_THRESHOLD; // water sensor trigger value
            int      servoIncrement = DAMPER_INCREMENT;     // degrees to move damper servo
            int      fanIncrement   = FAN_INCREMENT;        // % to change fan speed
            int      billDay;          // Water Bill Date: format:dd.  Look up and hard code >>>HERE<<<

        std::string toStr(){    // make JSON string
            char c[335];
            int n = sprintf(c, "[{\"oat\":%.2f,\"oap\":%.2f,\"oah\":%i,\"iat\":%.2f,\"iap\":%.2f,\"iah\":%i,\"draftDamperPos\":%i,\"draftFanPct\":%i,\"waterSensor\":%i,\"mainGallonsTotal\":%i,\"mainGallonsToday\":%i,\"mainFlowGPM\":%i,\"hotGallonsTotal\":%i,\"hotGallonsToday\":%i,\"hotFlowGPM\":%i},{\"pressureDelta\":%.3f,\"waterAlert\":%i,\"servoIncrement\":%i,\"fanIncrement\":%i,\"billDay\":%i}]"
            , oat, oap, oah, iat, iap, iah, 155-draftDamperPos, draftFanPct, waterSensor, mainGallonsTotal, mainGallonsToday, mainFlowGPM, hotGallonsTotal, hotGallonsToday, hotFlowGPM, pressureDelta, waterAlert, servoIncrement, fanIncrement, billDay);
            return(std::string(c,n));
        }
} measurements;
measurements vals;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8x8(U8G2_R0, U8X8_PIN_NONE, 5, 4);

// BME280-A  0x77  // BME280-B 0x76  // OLED 128x32 0.91" 12832 SSD1306LCD  0x3C

tiny::BME280 mySensorA; //Uses default I2C address 0x77
tiny::BME280 mySensorB; //Uses I2C address 0x76 (jumper closed)

Servo damperServo;
Servo damperFan;

  void setup() {
    Serial.begin(115200);

    SPIFFS.begin();

    AP_MODE=!initCreds();
    AP_MODE=AP_MODE || !wifiConnect(WIFI_STA);

    if(AP_MODE){
          wifiConnect(WIFI_AP);

          // init Websock
          webSock.onEvent(onWsEvent);
          server.addHandler(&webSock);
                /* Setup the DNS server redirecting all the domains to the apIP */
          dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
          dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
          server.onNotFound(notFound);

          // Web Pages
          server.on("/"        , HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", SSIDPWD_HTML);});

          AsyncElegantOTA.begin(&server);    // Start ElegantOTA
          server.onNotFound(notFound);
          server.begin();
    // END: Get Credentials "AP_MODE"
    }else{// NOT get WiFi Credentials
          Wire.begin(4,5);  // I2C bmp1, bmp2, OLED
          timeClient.begin();
          timeClient.update();
          vals.today=timeClient.getDay(); // timeClient to determine 1st day of month and 00:00 each day
          vals.lastMillis=millis();       // init struct's seconds since epoch: 19700101  000000
          initLocalStruct();
          analogWriteMode(SERVO_PIN, 255, OUTPUT_OPEN_DRAIN);
          analogWriteFreq(1000);

          damperServo.attach(SERVO_PIN);   // pin servoPin PWM
          vals.draftDamperPos=MAX_DAMPER;
          damperServo.write(vals.draftDamperPos);   // move to damperClosed

          damperFan.attach(DRAFT_FAN_PIN);
          vals.draftFanPct=MIN_FAN;
          damperFan.write(vals.draftFanPct);
          
          pinMode(HOT_FLOW_PIN, INPUT_PULLDOWN_16);             //initializes digital pin hotFlowPin as an input
          pinMode(MAIN_FLOW_PIN, INPUT_PULLDOWN_16);            //initializes digital pin mainFlowPin as an input

          digitalPinToInterrupt(MAIN_FLOW_PIN);
          attachInterrupt(MAIN_FLOW_PIN, pulseCounter, RISING);  //and the interrupt is attached

          pinMode(LEAK_DETECTOR_PIN, INPUT_PULLDOWN_16);   // INPUT_PULLDOWN_16
          pinMode(LEAK_LIGHT_PIN, OUTPUT);digitalWrite(LEAK_LIGHT_PIN, false); // Hot Water Tank is Leaking Light

          mySensorA.setI2CAddress(0x77); //This is also the default value: I2C address must be set before begin()
          mySensorB.setI2CAddress(0x76); //This is also the default value: I2C address must be set before begin()

          if (mySensorA.begin() == false)Serial.println("Sensor A \"outside\" connect failed");
          if (mySensorB.beginI2C(0x76) == false)Serial.println("Sensor B \"inside\" connect failed");
        yield(); // I break for unicorns
          u8x8.begin();
          u8x8.setPowerSave(0);

          esp_now_init();
          esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
          esp_now_register_send_cb(OnDataSent);
          esp_now_register_recv_cb(OnDataRecv);
          esp_now_add_peer(peer, ESP_NOW_ROLE_COMBO, 3, NULL, 0);

          // init Websock
          webSock.onEvent(onWsEvent);
          server.addHandler(&webSock);

          // Web Pages
          server.on("/"        , HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", INDEX_HTML);});
          server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", SETTINGS_HTML);});

          AsyncElegantOTA.begin(&server);    // Start ElegantOTA
          server.onNotFound(notFound);
          server.begin();
    } // END: functional setup()
  }
  void loop() {
    if(AP_MODE)return;

    char line1[32], line2[32], line3[32];

    if(vals.today != timeClient.getDay()){ // if New Day
      vals.today = timeClient.getDay();
      vals.mainGallonsToday = 0;
      vals.hotGallonsToday = 0;

      time_t rawtime = timeClient.getEpochTime();
      struct tm * ti;
      ti = localtime (&rawtime);

      // if new day && Bill Day then reset total gallons
      if(vals.billDay == ti->tm_mday && (!ti->tm_min && !ti->tm_sec) && vals.mainGallonsTotal >10){  // 00:00:00 on bill day and haven't reset yet?
        vals.mainGallonsTotal = 0;
        vals.hotGallonsTotal = 0;
      } // END Bill Day
    } // END NEW DAY

    if(millis()-vals.lastMillis > UPDATE_MILLIS){ // if time to update values???
        ets_intr_lock();       // IRQ Disable
        bufferHotTicks = hotFlowTicks;
        bufferMainTicks = mainFlowTicks;
        mainFlowTicks = 0;
        hotFlowTicks  = 0;
        ets_intr_unlock();    // IRQ Enable

  #ifdef dbg
    Serial.printf("MainFlow Ticks: %i, HotFlow Ticks: %i\n", bufferMainTicks, bufferHotTicks);Serial.flush();
  #endif

        vals.mainFlowGPM = (bufferMainTicks/TICKS_PER_GALLON_MAIN)*(60000/(millis()-vals.lastMillis));
        vals.hotFlowGPM  = (bufferHotTicks/TICKS_PER_GALLON_HOT)*(60000/(millis()-vals.lastMillis));
      
        vals.mainGallonsToday += bufferMainTicks/TICKS_PER_GALLON_MAIN;
        vals.mainGallonsTotal += bufferMainTicks/TICKS_PER_GALLON_MAIN;
        vals.hotGallonsToday  += bufferHotTicks/TICKS_PER_GALLON_HOT;
        vals.hotGallonsTotal  += bufferHotTicks/TICKS_PER_GALLON_HOT;

        // GET bmp280: temp, pressure, humidity READINGS
        vals.oat=mySensorA.readFixedTempF() / 100.0f;
        vals.oap=mySensorA.readFixedPressure()* 0.000295f; // outside pressure Pa
        vals.oah=mySensorA.readFixedHumidity();
        vals.iat=mySensorB.readFixedTempF() / 100.0f;
        vals.iap=mySensorB.readFixedPressure() *0.000295f; // inside pressure Pa
        vals.iah=mySensorB.readFixedHumidity();

        if(vals.oap - vals.iap > vals.pressureDelta)increaseAirFlow();
        if(vals.iap - vals.oap > vals.pressureDelta)decreaseAirFlow();

        damperServo.write(vals.draftDamperPos);
        damperFan.write(vals.draftFanPct);

        vals.waterSensor = analogRead(LEAK_DETECTOR_PIN);
        digitalWrite(LEAK_LIGHT_PIN, vals.waterSensor>vals.waterAlert);

        sprintf(line1, "OAT %.2f  OAP %.2f  OAH %i",  vals.oat, vals.oap, vals.oah);
        sprintf(line2, "IAT %.2f  IAP %.2f  IAH %i",  vals.iat, vals.iap, vals.iah);
  //      sprintf(line3, "Damper %i %s", maxDamper-vals.draftDamperPos, vals.waterSensor>900?"FLOOD":"");
        sprintf(line3, "Damper %i G: %i, WS: %i", MAX_DAMPER-vals.draftDamperPos, vals.mainGallonsToday, vals.waterSensor);

        Serial.println(line1);
        Serial.printf("%s\tp1: %1.2f\tp2: %.2f\tp1-p2: %.2f\n", line2, vals.oap, vals.iap, vals.oap-vals.iap);
        Serial.printf("%s\n", line3);
        Serial.printf("\n******************>> IP: %s <<*****************************\n\n", WiFi.localIP().toString().c_str());

        u8x8.clearBuffer();
        u8x8.setFlipMode(1);
        u8x8.setFont(u8g2_font_6x10_tr);
        u8x8.drawStr(3,11,line1);
        u8x8.drawStr(3,20,line2);
        u8x8.drawStr(3,29,line3);
        u8x8.drawRFrame(0,0,128,32,3);
        u8x8.sendBuffer();

        webSock.textAll(vals.toStr().c_str()); delay(1);
        webSock.cleanupClients();
        vals.lastMillis=millis();
    } // END: Update vals

    delay(1);
  }  //  END LOOP
  // WebSock Event Handler: rcv : self - update s_Msg (save/not save); Others' - send via esp_Now
  void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
    if(type == WS_EVT_DATA){
      AwsFrameInfo * info = (AwsFrameInfo*)arg;
      if(info->final && !info->index && info->len == len){
        if(info->opcode == WS_TEXT){
          data[len]=0;
          std::string const s=(char *)data;
          if(AP_MODE){
            setCreds(s);
            AP_MODE=!saveCreds();
            ESP.restart();
          }else{
            setBounds(s);
            saveBounds();
          }
        }
      }
    }
    return;
  } // Web Sock recieve
  void decreaseAirFlow(){ // maxDamper, minDamper, damperIncrement, maxFan, minFan, fanIncrement
  Serial.printf("decreaseAirFlow: %i\n", MAX_DAMPER - vals.draftDamperPos);
      vals.draftDamperPos += vals.servoIncrement;
      if(vals.draftDamperPos >= MAX_DAMPER){
          vals.draftDamperPos = MAX_DAMPER;
          if(vals.draftFanPct -= vals.fanIncrement < MIN_FAN)
              vals.draftFanPct = MIN_FAN;
      }
  }
  void increaseAirFlow(){
  Serial.printf("increaseAirFlow: %i\n", MAX_DAMPER - vals.draftDamperPos);
    vals.draftDamperPos -= vals.servoIncrement;
    if(vals.draftDamperPos <= MIN_DAMPER){
        vals.draftDamperPos = MIN_DAMPER;
      if(vals.draftFanPct += vals.fanIncrement > MAX_FAN)
          vals.draftFanPct = MAX_FAN;
    }
  }
  // ESP NOW  sent call back(cb)
  void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus){
  #ifdef dbg        
    if (sendStatus == 0){
      Serial.printf("\nNOW: Delivery success: %s\n", vals.toStr().c_str());Serial.flush();
    }
    else{
      Serial.printf("NOW: Delivery FAIL: %s\n", vals.toStr().c_str());Serial.flush();
    }
  #endif
  }
  //ESP NOW  call back(cb) OnDataReceive
  void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
      return;
    }
  void notFound(AsyncWebServerRequest *request){request->send_P(200, "text/html", INDEX_HTML);}
  bool wifiConnect(WiFiMode m){
      WiFi.disconnect();
      WiFi.softAPdisconnect();
      WiFi.mode(m);
      switch(m){
              case WIFI_STA:
                                WiFi.begin(creds.SSID.c_str(), creds.PWD.c_str());
                                WiFi.channel(2);
                                if(!creds.isDHCP)
                                  WiFi.config(creds.IP, creds.GW, creds.MASK);
                                break;
              case WIFI_AP:
                                WiFi.softAPConfig(ip_AP, ip_AP_GW, ip_subNet);
                                WiFi.softAP(AP_NAME, "");
                                WiFi.begin();
                                break;
              case WIFI_AP_STA: break;
              case WIFI_OFF:    break;
      }
      unsigned int startup = millis();
      while(WiFi.status() != WL_CONNECTED){
            delay(250);
            Serial.print(".");
            if(millis() - startup >= 5000) break;
      }
      Serial.println("");Serial.flush();
      return(WiFi.status() == WL_CONNECTED);
  }
  void setBounds(const std::string& s){
    vals.waterAlert=::atoi(valFromJson(s, "waterAlert").c_str());
    vals.billDay=::atoi(valFromJson(s, "billDay").c_str());
    vals.pressureDelta=::atof(valFromJson(s, "pressureDelta").c_str());
    vals.servoIncrement=::atoi(valFromJson(s, "servoIncrement").c_str());
    vals.fanIncrement=::atoi(valFromJson(s, "fanIncrement").c_str());
  }
  bool saveCreds(){
    File f = SPIFFS.open(F("/creds.json"), "w");
    if(f){
          f.print(creds.toStr().c_str());
          f.close();
  #ifdef dbg
    Serial.printf("saveCreds Success: creds: %s\n", creds.toStr().c_str());Serial.flush();
  #endif
          return true;
    }
    else{
  #ifdef dbg
    Serial.printf("saveCreds FAILED: creds: %s\n", creds.toStr().c_str());Serial.flush();
  #endif
        return(false);
    }
  }
  bool initCreds(){
          File f = SPIFFS.open(F("/creds.json"), "r");
          if(f){
                  std::string s = f.readString().c_str();
                  f.close();
                  setCreds(s);
          }
          else return(false);
  #ifdef dbg
        Serial.printf("\nFailed init creds from SPIFFS: %s\n", creds.toStr().c_str());Serial.flush();
  #endif
    return(true);
  }
  void setCreds(const std::string& s){
    creds.SSID=valFromJson(s, "SSID").c_str();
    creds.PWD=valFromJson(s, "PWD").c_str();
    creds.isDHCP=::atoi(valFromJson(s, "isDHCP").c_str());
    creds.IP.fromString(valFromJson(s, "IP").c_str());
    creds.GW.fromString(valFromJson(s, "GW").c_str());
    creds.MASK.fromString(valFromJson(s, "MASK").c_str());
  }
  bool saveBounds(){
    File f = SPIFFS.open(F("/vals.json"), "w");
    if(f){
          f.print(vals.toStr().c_str());
          f.close();
          return true;
    }
    else{return false;}
  }
  bool initLocalStruct(){
          File f = SPIFFS.open(F("/vals.json"), "r");
          if(f){
                  std::string s = f.readString().c_str();
                  f.close();
                  setBounds(s);
                  return(true);
          }
          else return(false);
  #ifdef dbg
          Serial.printf("\nFailed init vals from SPIFFS: %s\n", vals.toStr().c_str());Serial.flush();
  #endif
  }
    // NOTE: Was getting Stack Dumps when trying to use <ArduinoJson.h>
    std::string valFromJson(const std::string &json, const std::string &element){
      size_t start, end;
      start = json.find(element);
      start = json.find(":", start)+1;
      if(json.substr(start,1) =="\"")start++;
      end  = json.find_first_of(",]}\"", start);
      return(json.substr(start, end-start));
    }
  /*
  // Update value in JSON string.  NOTE: Stack Dumps when trying to use <ArduinoJson.h>
  void update_JSON(std::string& json, const std::string &element, const std::string &value){
    size_t start, end;

    start = json.find(element.c_str());
    start = json.find(":", start)+1;
    end  = json.find_first_of(",}", start); // commented "{"
    json = json.substr(0,start) + value + json.substr(end);
  }
  */
