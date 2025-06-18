#define DEBUG
#include "espConfig.h"
#include <time.h>  // cần cho time()

// Virtual pins
#define GAS_VALUE V1
#define LOG_TERMINAL V5

WidgetLED LEDCONNECT(V0);
WidgetTerminal terminal(LOG_TERMINAL);

unsigned long timeBlink = 0;
unsigned long lastAlert = 0;
String serialBuffer = "";

void setup() {
  Serial.begin(9600);
  espConfig.begin();

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // Đồng bộ thời gian thực

  delay(1000);

#ifdef DEBUG
  Serial.println("ESP Ready. Waiting for gas data...");
#endif

  terminal.clear();
  terminal.println("ESP Started.");
  terminal.flush();
}

void loop() {
  espConfig.run();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int ppm = serialBuffer.toInt();

      if (ppm > 0) {
        Blynk.virtualWrite(GAS_VALUE, ppm);

        if (ppm > 2000) {
          // Lấy thời gian thực
          time_t now = time(nullptr);
          struct tm *tm_struct = localtime(&now);
          char timeStr[10];
          sprintf(timeStr, "%02d:%02d:%02d", tm_struct->tm_hour, tm_struct->tm_min, tm_struct->tm_sec);

#ifdef DEBUG
          Serial.printf("[%s] Gas: %d ppm (CRITICAL)\n", timeStr, ppm);
#endif

          terminal.printf("[%s] Gas: %d ppm (CRITICAL)\n", timeStr, ppm);
          terminal.flush();

          // if (millis() - lastAlert > 15000) {
            // Blynk.logEvent("alert", "Gas > 2000 ppm! (" + String(ppm) + ")");
          Blynk.logEvent("alarm", "Gas > 2000 ppm! vượt quá mức cho phép");
          terminal.flush();


            // lastAlert = millis();
          // }
        }
      }

      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }

  if (millis() - timeBlink > 1000) {
    LEDCONNECT.setValue(!LEDCONNECT.getValue());
    timeBlink = millis();
  }
}

void app_loop() {}

BLYNK_CONNECTED() {
  Blynk.syncAll();
  terminal.println("Blynk Connected.");
  terminal.flush();
}
