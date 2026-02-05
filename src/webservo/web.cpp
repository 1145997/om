#include "web.h"




const char* ssid = "test";      // 替换为你的Wi-Fi名称
const char* password = "12345678";  // 替换为你的Wi-Fi密码

// 创建 Web 服务器实例
AsyncWebServer server(80);

static inline int clamp180(int v){
  if (v < 0) return 0;
  if (v > 180) return 180;
  return v;
}
void wifi_init() {
  // 1) 挂载 LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  // 2) 开 AP（自己发 WiFi），不要 WiFi.begin
  WiFi.mode(WIFI_AP);

  // 注意：AP 密码必须 >= 8 位；如果你就想用 123456，可以改成开放热点（见下面注释）
  bool ok = WiFi.softAP(ssid, password);
  // 如果想开放热点（无密码），用：
  // bool ok = WiFi.softAP(ssid);

  if (!ok) {
    Serial.println("softAP failed (password must be >= 8 chars if set)");
  }

  IPAddress ip = WiFi.softAPIP(); // 默认通常 192.168.4.1
  Serial.print("AP started. SSID=");
  Serial.println(ssid);
  Serial.print("AP IP=");
  Serial.println(ip);

  // 3) 主页：直接给 data/index.html
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // 4) API：控制舵机
  server.on("/api/servo", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("name") || !request->hasParam("value")) {
      request->send(400, "text/plain", "missing name/value");
      return;
    }

    String name = request->getParam("name")->value();
    int value = clamp180(request->getParam("value")->value().toInt());

    if (name == "Y")      Y.write(value);
    else if (name == "Z") Z.write(value);
    else if (name == "E") E.write(value);
    else {
      request->send(400, "text/plain", "bad name");
      return;
    }

    request->send(200, "text/plain", "OK");
  });

  server.begin();
}
