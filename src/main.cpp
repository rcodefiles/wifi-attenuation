#define BLYNK_TEMPLATE_ID "TMPL68lhp5qok"
#define BLYNK_TEMPLATE_NAME "wificar"
#define BLYNK_AUTH_TOKEN "OWeeF5DZAbDH_GCW1xrncocWJeoKEcce"

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#else
  #include <WiFi.h>
#endif

#if defined(ESP8266)
  #include <ESP8266WebServer.h>
#else
  #include <WebServer.h>
#endif

#if defined(ESP8266)
  ESP8266WebServer server(80);
#else
  WebServer server(80);
#endif

#include <BlynkSimpleEsp8266.h>

char ssid[] = "REDACTED"; 
char pass[] = "REDACTED"; 


const float WHEEL_DIAMETER_MM = 65.0;
const float WHEELBASE_MM = 180.0;
const float MAX_RPM_WHEEL = 160.0;
const float SPEED_SCALE = 0.75;
float distance_travelled = 0.0; 
float last_logged_distance = 0.0;

float x_m = 0, y_m = 0, theta = 0; // pose
float cmdLeft = 0, cmdRight = 0;


const int ENA = 5;
const int ENB = 4;
const int INA = 0;
const int INB = 2;

void setMotorSpeed(int speedB, int speedA) {
    if (speedA > 0) { digitalWrite(INA, LOW); analogWrite(ENA, speedA); }
    else if (speedA < 0) { digitalWrite(INA, HIGH); analogWrite(ENA, -speedA); }

    if (speedB > 0) { digitalWrite(INB, HIGH); analogWrite(ENB, speedB); }
    else if (speedB < 0) { digitalWrite(INB, LOW); analogWrite(ENB, -speedB); }
}

enum PathAction { FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, STOP };

struct Waypoint {
  PathAction action;
  float param; 
};

Waypoint path[] = {
    {FORWARD, .15},
    {TURN_LEFT, 90.0},
    {FORWARD, .05},
    {TURN_LEFT, 90.0},
    {FORWARD, .15},
    {TURN_RIGHT, 90.0},
    {FORWARD, .05},
    {TURN_RIGHT, 90.0},
    {FORWARD, .15},
    {TURN_LEFT, 90.0},
    {FORWARD, .05},
    {TURN_LEFT, 90.0},
    {FORWARD, .15},
    {TURN_RIGHT, 90.0},
    {FORWARD, .05},
    {TURN_RIGHT, 90.0},
    {FORWARD, .15},
    {TURN_LEFT, 90.0},
    {FORWARD, .05},
    {TURN_LEFT, 90.0},
    {FORWARD, .15},
    {TURN_RIGHT, 90.0},
    {FORWARD, .05},
    {TURN_RIGHT, 90.0},
    {STOP, 0}
};

int currentWaypoint = 0;
float startDistance = 0;
float targetAngle = 0;
bool turning = false;

void followPath() {
    if (currentWaypoint >= sizeof(path)/sizeof(path[0])) return;

    Waypoint wp = path[currentWaypoint];
    if (wp.action == STOP) {
        digitalWrite(INA, LOW);
        digitalWrite(INB, LOW);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
    }
    switch (wp.action) {
        case FORWARD:
            cmdLeft = cmdRight = 1.0; 
            if (distance_travelled - startDistance >= wp.param) {
                currentWaypoint++;
                startDistance = distance_travelled;
                cmdLeft = cmdRight = 0;
            }
            break;
        case BACKWARD:
            cmdLeft = cmdRight = -1.0; 
            if (distance_travelled - startDistance >= wp.param) {
                currentWaypoint++;
                startDistance = distance_travelled;
                cmdLeft = cmdRight = 0;
            }
            break;

        case TURN_LEFT:
            if (!turning) {
                targetAngle = theta + radians(wp.param);
                turning = true;
            }
            cmdLeft = -0.5; cmdRight = 0.5;
            if (theta >= targetAngle) {
                turning = false;
                cmdLeft = cmdRight = 0;
                currentWaypoint++;
            }
            break;

        case TURN_RIGHT:
            if (!turning) {
                targetAngle = theta - radians(wp.param);
                turning = true;
            }
            cmdLeft = 0.5; cmdRight = -0.5;
            if (theta <= targetAngle) {
                turning = false;
                cmdLeft = cmdRight = 0;
                currentWaypoint++;
            }
            break;

        case STOP:
            cmdLeft = cmdRight = 0;
            if (distance_travelled - startDistance >= wp.param) {
                currentWaypoint++;
                startDistance = distance_travelled;
                cmdLeft = cmdRight = 0;
            }
            break;
    }

    int speedA = constrain(cmdRight * 255, -255, 255);
    int speedB = constrain(cmdLeft  * 255, -255, 255);
    setMotorSpeed(speedB, speedA);
}

unsigned long lastUpdate = 0;

void updatePose(unsigned long now) {
    if (lastUpdate == 0) { lastUpdate = now; return; }
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    float wheelCirc = 3.14159 * WHEEL_DIAMETER_MM / 1000.0;
    float rpmL = cmdLeft  * MAX_RPM_WHEEL * SPEED_SCALE;
    float rpmR = cmdRight * MAX_RPM_WHEEL * SPEED_SCALE;

    float vL = (rpmL * wheelCirc) / 60.0;
    float vR = (rpmR * wheelCirc) / 60.0;
    float v = (vL + vR) / 2.0;

    distance_travelled += v * dt;
    float omega = (vR - vL) / (WHEELBASE_MM / 1000.0);

    if (fabs(omega) < 1e-6) {
        x_m += v * dt * cos(theta);
        y_m += v * dt * sin(theta);
    } else {
        float dtheta = omega * dt;
        float r = v / omega;
        x_m += r * (sin(theta + dtheta) - sin(theta));
        y_m -= r * (cos(theta + dtheta) - cos(theta));
        theta += dtheta;
    }
}

const float LOG_DISTANCE = .05; 
String csvBuffer = "time_ms,x_m,y_m,RSSI\n";

void logCSV(unsigned long now) {
    if (distance_travelled - last_logged_distance >= LOG_DISTANCE) {
        last_logged_distance = distance_travelled;
        int rssi = WiFi.RSSI();
        csvBuffer += String(now) + "," + String(x_m,3) + "," + String(y_m,3) + "," + String(rssi) + "\n";
        Blynk.virtualWrite(V4, rssi); 
    }
}

void handleCSVDownload() {
    server.sendHeader("Content-Type", "text/csv");
    server.sendHeader("Content-Disposition", "attachment; filename=rover_log.csv");
    server.send(200, "text/csv", csvBuffer);
}

void setup() {
    Serial.begin(115200);
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);

    server.on("/download", handleCSVDownload);
    server.begin();

}

BLYNK_WRITE(V2) {
  int state = param.asInt();
  if (state) {
    Blynk.virtualWrite(V3, "Download CSV: http://" + WiFi.localIP().toString() + "/download");
  }
}


void loop() {
    Blynk.run();
    followPath();
    server.handleClient();

    unsigned long now = millis();
    updatePose(now);
    logCSV(now);

}
