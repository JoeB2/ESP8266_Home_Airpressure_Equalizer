#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
#include <index.h>

const int servoPin=12;        // pwm signal to damper servo

#define maxDamper 155
#define minDamper 35
#define damperIncrement 5

// Replace with your network credentials
const char* ssid = "JRJAG";
const char* password = "GeorgeTheDogy";

AsyncWebServer server(80);
AsyncWebSocket webSock("/");

//prototypes
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void notFound(AsyncWebServerRequest *request);
std::string valFromJson(const std::string &json, const std::string &element);

Servo damperServo;
//Servo damperFan;

void setup() {
  Serial.begin(115200);

  analogWriteMode(servoPin, 255, OUTPUT_OPEN_DRAIN);
  analogWriteFreq(1000);

  damperServo.attach(servoPin); // pin servoPin PWM
  damperServo.write(155);       // move to damperClosed

    // Connect to Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.printf("\nConnected to %s\tIP address: %s\n\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    // init Websock
    webSock.onEvent(onWsEvent);
    server.addHandler(&webSock);

    // Web Pages
    server.on("/"        , HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", INDEX_HTML);});
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", SERVO_HTML);});

    server.onNotFound(notFound);
    server.begin();
}
char s[20];
void loop() {
  for(int i =minDamper;i<maxDamper;i++){
    delay(200);
    damperServo.write(i);
    yield();
    sprintf(s, "{\"degrees\":%i}", i);
    webSock.textAll(s);
  }
  for(int i =maxDamper;i>minDamper;i--){
    delay(200);
    damperServo.write(i);
    yield();
    sprintf(s, "{\"degrees\":%i}", i);
    webSock.textAll(s);
  }

}  //  END LOOP

// WebSock Event Handler: rcv : self - update s_Msg (save/not save); Others' - send via esp_Now
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if(info->final && !info->index && info->len == len){
      if(info->opcode == WS_TEXT){
        data[len]=0;
        std::string const s=(char *)data;
Serial.printf("WS: msg:%s, s_degrees:%s, degrees:%i\n\n",s.c_str(), valFromJson(s, "degrees").c_str(), ::atoi(valFromJson(s, "degrees").c_str()));

        damperServo.write(::atoi(valFromJson(s, "degrees").c_str()));
      }
    }
  }
  return;
} // Web Sock recieve
void notFound(AsyncWebServerRequest *request){request->send_P(200, "text/html", INDEX_HTML);}
// return a value from scaleJSON as std::string.  NOTE: Stack Dumps when trying to use <ArduinoJson.h>
std::string valFromJson(const std::string &json, const std::string &element){
  size_t start, end;

  start = json.find(element.c_str());
  start = json.find(":", start)+1;
  end  = json.find_first_of(",}", start); // commented "{"

  return(json.substr(start, end-start));
}
