/*
 * XIAO ESP32-S3 Sense — minimal HTTP camera server for PoseNet real-time VPR
 *
 * Endpoints (after boot, IP is printed on the Serial Monitor @115200):
 *   http://<IP>/          -> tiny status page
 *   http://<IP>/capture   -> one JPEG frame (what posenet_predict.py polls)
 *   http://<IP>/status    -> plain-text "ok"
 *
 * Arduino IDE setup (critical settings):
 *   1. Boards Manager: install "esp32 by Espressif Systems" (2.0.14 or newer)
 *   2. Tools -> Board  : XIAO_ESP32S3
 *   3. Tools -> PSRAM  : OPI PSRAM        <-- camera WILL fail without this
 *   4. Tools -> USB CDC On Boot: Enabled  <-- so Serial prints appear over USB
 *   5. The camera goes on the Sense expansion board (the small round B2B connector)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ---------------- EDIT THESE ----------------
const char *WIFI_SSID = "DESKTOP-D5GK9FR 1976";
const char *WIFI_PASS = "]46205Ge";
// ---------------------------------------------

// SVGA 800x600 JPEG ~ 40-80 KB -> fastest polling over WiFi; the model only
// needs a 256px shortest side, so this is plenty. For max detail use
// FRAMESIZE_UXGA (slower, ~2-5 fps polling).
#define FRAME_SIZE FRAMESIZE_QVGA       // Reduced to QVGA (320x240) for max stability
#define JPEG_QUALITY 15                 // 0-63, lower = better quality / bigger file (15 is a safe middle ground)

// XIAO ESP32-S3 Sense (OV2640) camera pin map
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  5

WebServer server(80);

bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 10000000;            // Lowered from 20MHz to 10MHz for stability
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size = FRAME_SIZE;
  cfg.jpeg_quality = JPEG_QUALITY;
  cfg.fb_count = 1;                       // Reduced to 1 buffer to completely avoid memory overflow issues
  cfg.fb_location = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // Better for single buffer mode

  return esp_camera_init(&cfg) == ESP_OK;
}

void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  
  // Retry up to 3 times if capture fails
  for (int i = 0; i < 3 && !fb; i++) {
    delay(100);
    fb = esp_camera_fb_get();
  }
  
  if (!fb) {
    server.send(503, "text/plain", "capture failed");
    return;
  }
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!psramFound()) {
    Serial.println("FATAL: PSRAM not found. Set Tools > PSRAM > OPI PSRAM and re-flash.");
    while (true) delay(1000);
  }

  if (!initCamera()) {
    Serial.println("FATAL: camera init failed. Check the expansion board is seated.");
    while (true) delay(1000);
  }
  Serial.println("camera OK");

  // Warm up the camera and clear initial bad frames
  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(50);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("capture endpoint: http://%s/capture\n", WiFi.localIP().toString().c_str());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html",
                "<h3>XIAO Sense PoseNet camera server</h3>"
                "<p><a href='/capture'>/capture</a> - one JPEG frame</p>"
                "<p><a href='/status'>/status</a> - plain text ok</p>");
  });
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/status", HTTP_GET, []() { server.send(200, "text/plain", "ok"); });
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
