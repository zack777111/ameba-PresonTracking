// AMB82-mini: YOLO person detection + single-axis MG996R tracking.
#include <Arduino.h>
#include <AmebaServo.h>
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
extern "C" {
#include "FreeRTOS.h"
#include "queue.h"
}

char ssid[] = "wifi name";
char pass[] = "wifi password";
const int SERVO_PIN = 8;
const int VIDEO_CHANNEL = 0;
const int NN_CHANNEL = 3;
const int MIN_US = 1200; // Conservative starting travel; calibrate after mounting.
const int MAX_US = 1800;
const int CENTER_US = 1500;
const float DEAD_ZONE = 0.08f; // +/-8% of image width around center.
int direction = 1; // Toggle with 'i' if the camera turns away from the person.
int pulseUs = CENTER_US;
bool tracking = false;
bool motorEnabled = false;
unsigned long lastDetection = 0;
unsigned long lastMoveAt = 0;
bool targetPresent = false;

VideoSetting video(VIDEO_HD, 20, VIDEO_H264, 0);
VideoSetting neuralVideo(576, 320, 10, VIDEO_RGB, 0);
NNObjectDetection detector;
RTSP rtsp;
StreamIO videoLink(1, 1);
StreamIO neuralLink(1, 1);
AmebaServo motor;
struct Target {
  bool found;
  float left, right, top, bottom;
  int objects, people, bestPersonScore, bestType, bestScore;
};
QueueHandle_t targets;

void printStreamUrl() {
  Serial.println();
  Serial.println("========== RTSP URL ==========");
  Serial.print("rtsp://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(rtsp.getPort());
  Serial.println("==============================");
}

// The NN task publishes frames; only loop() controls the motor and overlay.
void onDetection(std::vector<ObjectDetectionResult> results) {
  Target target = {};
  target.objects = (int)results.size();
  target.bestType = -1;
  float largestArea = 0;
  for (auto &item : results) {
    if (item.score() > target.bestScore) {
      target.bestScore = item.score();
      target.bestType = item.type();
    }
    if (item.type() != 0) continue;
    ++target.people;
    if (item.score() > target.bestPersonScore) target.bestPersonScore = item.score();
    if (item.score() < 35) continue;
    float area = (item.xMax() - item.xMin()) * (item.yMax() - item.yMin());
    if (area <= largestArea) continue;
    largestArea = area;
    target.found = true;
    target.left = item.xMin(); target.right = item.xMax();
    target.top = item.yMin(); target.bottom = item.yMax();
  }
  xQueueOverwrite(targets, &target);
}

void enableMotor() {
  if (motorEnabled) return;
  motor.attach(SERVO_PIN, 1000, 2000);
  motor.writeMicroseconds(pulseUs);
  motorEnabled = true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  targets = xQueueCreate(1, sizeof(Target));
  if (!targets) {
    Serial.println("ERROR: target queue allocation failed");
    while (true) delay(1000);
  }
  Serial.println("Connecting WiFi; set ssid/pass in sketch first.");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(2000);
  }
  video.setBitrate(2 * 1024 * 1024);
  Camera.configVideoChannel(VIDEO_CHANNEL, video);
  Camera.configVideoChannel(NN_CHANNEL, neuralVideo);
  Camera.videoInit();
  rtsp.configVideo(video);
  rtsp.begin();
  // Show the URL before starting YOLO, whose SDK logs can flood Serial.
  printStreamUrl();
  Serial.println("Copy URL now. YOLO starts in 10 seconds.");
  Serial.flush();
  delay(10000);
  detector.configVideo(neuralVideo);
  detector.configThreshold(0.35f, 0.3f);
  detector.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
  detector.setResultCallback(onDetection);
  detector.begin();
  videoLink.registerInput(Camera.getStream(VIDEO_CHANNEL));
  videoLink.registerOutput(rtsp);
  neuralLink.registerInput(Camera.getStream(NN_CHANNEL));
  neuralLink.setStackSize();
  neuralLink.setTaskPriority();
  neuralLink.registerOutput(detector);
  if (videoLink.begin() != 0 || neuralLink.begin() != 0) {
    Serial.println("ERROR: stream initialization failed; reset board");
    while (true) delay(1000);
  }
  OSD.configVideo(VIDEO_CHANNEL, video);
  OSD.begin();
  Camera.channelBegin(VIDEO_CHANNEL);
  Camera.channelBegin(NN_CHANNEL);
  printStreamUrl();
  Serial.println("READY: t=track, s=stop PWM, c=center, i=reverse, u=RTSP URL");
  Serial.println("Tracking OFF. Mount camera before sending t.");
}

void loop() {
  while (Serial.available()) {
    char command = (char)Serial.read();
    if (command == 't') {
      enableMotor(); tracking = true;
      Serial.println("TRACKING_ON");
    } else if (command == 's') {
      tracking = false;
      if (motorEnabled) motor.detach();
      motorEnabled = false;
      Serial.println("PWM_STOPPED (power remains connected)");
    } else if (command == 'c') {
      tracking = false; enableMotor();
      pulseUs = CENTER_US; motor.writeMicroseconds(pulseUs);
      Serial.println("CENTER 1500us; tracking OFF");
    } else if (command == 'i') {
      direction = -direction;
      Serial.print("DIRECTION:"); Serial.println(direction);
    } else if (command == 'u') {
      printStreamUrl();
    }
  }
  Target target;
  static unsigned long lastReport = 0;
  static unsigned long lastHeartbeat = 0;
  static unsigned long receivedFrames = 0;
  if (xQueueReceive(targets, &target, 0) == pdTRUE) {
    ++receivedFrames;
    unsigned long now = millis();
    if (now - lastReport >= 1000) {
      lastReport = now;
      Serial.print("DETECT objects="); Serial.print(target.objects);
      Serial.print(" people="); Serial.print(target.people);
      Serial.print(" personScore="); Serial.print(target.bestPersonScore);
      Serial.print(" bestType="); Serial.print(target.bestType);
      Serial.print(" bestScore="); Serial.print(target.bestScore);
      Serial.print(" tracking="); Serial.println(tracking ? "ON" : "OFF");
    }
    // Cap correction speed even if NN results arrive faster than expected.
    bool canMove = now - lastMoveAt >= 100;
    lastDetection = now;
    targetPresent = target.found;
    OSD.createBitmap(VIDEO_CHANNEL);
    int centerX = video.width() / 2;
    int centerY = video.height() / 2;
    OSD.drawRect(VIDEO_CHANNEL, centerX - 8, centerY - 8, centerX + 8,
                 centerY + 8, 1, OSD_COLOR_CYAN);
    if (target.found) {
      OSD.drawRect(VIDEO_CHANNEL, (int)(target.left * (video.width() - 1)),
                   (int)(target.top * (video.height() - 1)),
                   (int)(target.right * (video.width() - 1)),
                   (int)(target.bottom * (video.height() - 1)), 3, OSD_COLOR_WHITE);
      float error = (target.left + target.right) * 0.5f - 0.5f;
      if (tracking && canMove && (error > DEAD_ZONE || error < -DEAD_ZONE)) {
        int step = (int)(error * 45.0f);
        step = constrain(step, -12, 12);
        pulseUs = constrain(pulseUs + direction * step, MIN_US, MAX_US);
        motor.writeMicroseconds(pulseUs);
        lastMoveAt = now;
      }
    }
    OSD.update(VIDEO_CHANNEL);
  }
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.print("NN_CALLBACK_FRAMES:"); Serial.println(receivedFrames);
  }
  // Missing frames never trigger movement; clear stale display after 1 second.
  if (targetPresent && millis() - lastDetection > 1000) {
    targetPresent = false;
    OSD.createBitmap(VIDEO_CHANNEL); OSD.update(VIDEO_CHANNEL);
    Serial.println("TARGET_LOST: holding position");
  }
  delay(5);
}
