// AMB82-mini: YOLO person detection + single-axis MG996R tracking.
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
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

char ssid[] = "XXXXXX";
char pass[] = "XXXXXXX";
const int SERVO_PIN = 8;
const int VIDEO_CHANNEL = 0;
const int NN_CHANNEL = 3;
const int MIN_US = 1000; // Calibrate these endpoints for a safe 180-degree sweep.
const int MAX_US = 2000;
const int CENTER_US = 1500;
const float DEAD_ZONE = 0.08f; // +/-8% of image width around center.
const int MAX_PERSON_CANDIDATES = 14;
const unsigned long LOST_TIMEOUT_MS = 10000;
const unsigned long MOVE_INTERVAL_MS = 100;
const int SEARCH_STEP_US = 8;
const int RETURN_STEP_US = 4;
int direction = 1; // Toggle with 'i' if the camera turns away from the person.
int pulseUs = CENTER_US;
bool tracking = false;
bool motorEnabled = false;
unsigned long lastMoveAt = 0;
unsigned long lastAnyPersonAt = 0;
unsigned long lastDetectionFrameAt = 0;
bool overlayShowingPerson = false;
int nextTargetId = 1;
int trackedTargetId = 0;
float lastTargetCenterX = 0.5f;
float lastTargetArea = 0;
int lastTargetPulseUs = CENTER_US;
int searchDirection = 1;
bool searchReachedFirstLimit = false;

enum TrackingState {
  STANDBY,
  TRACKING_TARGET,
  SEARCHING,
  WAITING_FOR_TARGET,
  RETURNING_HOME
};
TrackingState trackingState = STANDBY;

VideoSetting video(VIDEO_HD, 20, VIDEO_H264, 0);
VideoSetting neuralVideo(576, 320, 10, VIDEO_RGB, 0);
NNObjectDetection detector;
RTSP rtsp;
StreamIO videoLink(1, 1);
StreamIO neuralLink(1, 1);
AmebaServo motor;
struct Target {
  int objects, people, bestPersonScore, bestType, bestScore;
  struct PersonBox {
    float left, right, top, bottom;
    int score;
  } persons[MAX_PERSON_CANDIDATES];
};
QueueHandle_t targets;

float personCenterX(const Target::PersonBox &person) {
  return (person.left + person.right) * 0.5f;
}

float personArea(const Target::PersonBox &person) {
  return (person.right - person.left) * (person.bottom - person.top);
}

int largestPersonIndex(const Target &target) {
  int largestIndex = -1;
  float largestArea = 0;
  for (int i = 0; i < target.people; ++i) {
    float area = personArea(target.persons[i]);
    if (area > largestArea) {
      largestArea = area;
      largestIndex = i;
    }
  }
  return largestIndex;
}

int findTrackedPerson(const Target &target, bool searchingForTarget) {
  if (lastTargetArea <= 0) return -1;
  float expectedCenterX = lastTargetCenterX -
                          (pulseUs - lastTargetPulseUs) * direction / 1000.0f;
  expectedCenterX = constrain(expectedCenterX, 0.0f, 1.0f);
  float maxCenterDistance = searchingForTarget ? 0.45f : 0.30f;
  float bestCost = 1000.0f;
  int bestIndex = -1;
  for (int i = 0; i < target.people; ++i) {
    const Target::PersonBox &person = target.persons[i];
    float areaRatio = personArea(person) / lastTargetArea;
    if (areaRatio < 0.30f || areaRatio > 3.0f) continue;
    float centerDistance = fabsf(personCenterX(person) - expectedCenterX);
    if (centerDistance > maxCenterDistance) continue;
    float cost = centerDistance + fabsf(areaRatio - 1.0f) * 0.10f;
    if (cost < bestCost) {
      bestCost = cost;
      bestIndex = i;
    }
  }
  return bestIndex;
}

void rememberTarget(const Target::PersonBox &person, unsigned long now) {
  lastTargetCenterX = personCenterX(person);
  lastTargetArea = personArea(person);
  lastTargetPulseUs = pulseUs;
  lastAnyPersonAt = now;
}

void selectTarget(const Target::PersonBox &person, unsigned long now) {
  trackedTargetId = nextTargetId++;
  if (nextTargetId > 9999) nextTargetId = 1;
  trackingState = TRACKING_TARGET;
  rememberTarget(person, now);
  Serial.print("TARGET_SELECTED ID:"); Serial.println(trackedTargetId);
}

bool moveServoToward(int destinationUs, int stepUs, unsigned long now) {
  if (now - lastMoveAt < MOVE_INTERVAL_MS || pulseUs == destinationUs) return false;
  if (pulseUs < destinationUs) {
    pulseUs += stepUs;
    if (pulseUs > destinationUs) pulseUs = destinationUs;
  } else {
    pulseUs -= stepUs;
    if (pulseUs < destinationUs) pulseUs = destinationUs;
  }
  pulseUs = constrain(pulseUs, MIN_US, MAX_US);
  motor.writeMicroseconds(pulseUs);
  lastMoveAt = now;
  return true;
}

void beginSearch() {
  trackingState = SEARCHING;
  float lastError = lastTargetCenterX - 0.5f;
  searchDirection = lastError == 0 ? direction : (lastError * direction > 0 ? 1 : -1);
  searchReachedFirstLimit = false;
  Serial.print("TARGET_LOST: searching toward ");
  Serial.println(searchDirection > 0 ? "right" : "left");
}

void advanceSearch(unsigned long now) {
  int firstLimit = searchDirection > 0 ? MAX_US : MIN_US;
  int secondLimit = searchDirection > 0 ? MIN_US : MAX_US;
  if (!searchReachedFirstLimit) {
    moveServoToward(firstLimit, SEARCH_STEP_US, now);
    if (pulseUs == firstLimit) {
      searchReachedFirstLimit = true;
      searchDirection = -searchDirection;
    }
  } else {
    moveServoToward(secondLimit, SEARCH_STEP_US, now);
    if (pulseUs == secondLimit) {
      trackingState = WAITING_FOR_TARGET;
      Serial.println("SEARCH_COMPLETE: full configured sweep finished");
    }
  }
}

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
  for (auto &item : results) {
    if (item.score() > target.bestScore) {
      target.bestScore = item.score();
      target.bestType = item.type();
    }
    if (item.type() != 0) continue;
    if (item.score() > target.bestPersonScore) target.bestPersonScore = item.score();
    if (item.score() < 35) continue;
    if (target.people >= MAX_PERSON_CANDIDATES) continue;
    Target::PersonBox &person = target.persons[target.people++];
    person.left = item.xMin();
    person.right = item.xMax();
    person.top = item.yMin();
    person.bottom = item.yMax();
    person.score = item.score();
  }
  xQueueOverwrite(targets, &target);
}

void enableMotor() {
  if (motorEnabled) return;
  motor.attach(SERVO_PIN, MIN_US, MAX_US);
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
      trackedTargetId = 0;
      trackingState = STANDBY;
      lastAnyPersonAt = millis();
      Serial.println("TRACKING_ON");
    } else if (command == 's') {
      tracking = false;
      trackedTargetId = 0;
      trackingState = STANDBY;
      if (motorEnabled) motor.detach();
      motorEnabled = false;
      Serial.println("PWM_STOPPED (power remains connected)");
    } else if (command == 'c') {
      tracking = false;
      trackedTargetId = 0;
      trackingState = STANDBY;
      enableMotor();
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
  static unsigned long receivedResults = 0;
  if (xQueueReceive(targets, &target, 0) == pdTRUE) {
    ++receivedResults;
    unsigned long now = millis();
    lastDetectionFrameAt = now;
    if (now - lastReport >= 1000) {
      lastReport = now;
      Serial.print("DETECT objects="); Serial.print(target.objects);
      Serial.print(" people="); Serial.print(target.people);
      Serial.print(" personScore="); Serial.print(target.bestPersonScore);
      Serial.print(" bestType="); Serial.print(target.bestType);
      Serial.print(" bestScore="); Serial.print(target.bestScore);
      Serial.print(" tracking="); Serial.print(tracking ? "ON" : "OFF");
      Serial.print(" targetID="); Serial.println(trackedTargetId);
    }

    int displayPersonIndex = -1;
    if (tracking) {
      if (target.people > 0) lastAnyPersonAt = now;
      if (trackedTargetId == 0) {
        int candidateIndex = largestPersonIndex(target);
        if (candidateIndex >= 0) {
          selectTarget(target.persons[candidateIndex], now);
          displayPersonIndex = candidateIndex;
        }
      } else {
        int trackedIndex = findTrackedPerson(target, trackingState == SEARCHING ||
                                                       trackingState == WAITING_FOR_TARGET);
        if (trackedIndex >= 0) {
          if (trackingState == SEARCHING || trackingState == WAITING_FOR_TARGET) {
            Serial.print("TARGET_REACQUIRED ID:"); Serial.println(trackedTargetId);
          }
          trackingState = TRACKING_TARGET;
          rememberTarget(target.persons[trackedIndex], now);
          displayPersonIndex = trackedIndex;
        } else {
          if (trackingState == TRACKING_TARGET) beginSearch();
          if (trackingState == WAITING_FOR_TARGET) {
            int replacementIndex = largestPersonIndex(target);
            if (replacementIndex >= 0) {
              selectTarget(target.persons[replacementIndex], now);
              displayPersonIndex = replacementIndex;
            } else if (now - lastAnyPersonAt >= LOST_TIMEOUT_MS) {
              trackedTargetId = 0;
              trackingState = RETURNING_HOME;
              Serial.println("TARGET_ABANDONED: returning to center");
            }
          }
        }
      }

      if (displayPersonIndex >= 0 && trackingState == TRACKING_TARGET) {
        const Target::PersonBox &person = target.persons[displayPersonIndex];
        rememberTarget(person, now);
        float error = personCenterX(person) - 0.5f;
        if (error > DEAD_ZONE || error < -DEAD_ZONE) {
          int step = (int)(error * 45.0f);
          step = constrain(step, -12, 12);
          int destinationUs = constrain(pulseUs + direction * step, MIN_US, MAX_US);
          moveServoToward(destinationUs, 12, now);
        }
      } else if (trackedTargetId == 0 && trackingState != RETURNING_HOME &&
             pulseUs != CENTER_US && now - lastAnyPersonAt >= LOST_TIMEOUT_MS) {
        trackingState = RETURNING_HOME;
        Serial.println("NO_PERSON: returning to center");
      }
      if (trackingState == SEARCHING) advanceSearch(now);
      if (trackingState == RETURNING_HOME) {
        moveServoToward(CENTER_US, RETURN_STEP_US, now);
        if (pulseUs == CENTER_US) {
          trackingState = STANDBY;
          Serial.println("CENTERED: standing by");
        }
      }
    } else {
      displayPersonIndex = largestPersonIndex(target);
    }

    OSD.createBitmap(VIDEO_CHANNEL);
    int centerX = video.width() / 2;
    int centerY = video.height() / 2;
    OSD.drawRect(VIDEO_CHANNEL, centerX - 8, centerY - 8, centerX + 8,
                 centerY + 8, 1, OSD_COLOR_CYAN);
    if (displayPersonIndex >= 0) {
      const Target::PersonBox &person = target.persons[displayPersonIndex];
      int left = (int)(person.left * (video.width() - 1));
      int top = (int)(person.top * (video.height() - 1));
      int right = (int)(person.right * (video.width() - 1));
      int bottom = (int)(person.bottom * (video.height() - 1));
      OSD.drawRect(VIDEO_CHANNEL, left, top, right, bottom, 3, OSD_COLOR_WHITE);
      if (trackedTargetId > 0) {
        char label[16];
        snprintf(label, sizeof(label), "ID:%d", trackedTargetId);
        int labelY = top - OSD.getTextHeight(VIDEO_CHANNEL);
        if (labelY < 0) labelY = 0;
        OSD.drawText(VIDEO_CHANNEL, left, labelY, label, OSD_COLOR_CYAN);
      }
    }
    overlayShowingPerson = displayPersonIndex >= 0;
    OSD.update(VIDEO_CHANNEL);
  }
  if (millis() - lastHeartbeat >= 3000) {
    lastHeartbeat = millis();
    Serial.print("NN_RESULTS_CONSUMED:"); Serial.println(receivedResults);
  }
  if (overlayShowingPerson && lastDetectionFrameAt > 0 &&
      millis() - lastDetectionFrameAt > 1000) {
    overlayShowingPerson = false;
    OSD.createBitmap(VIDEO_CHANNEL); OSD.update(VIDEO_CHANNEL);
    Serial.println("DETECTION_FRAMES_LOST: clearing overlay");
  }
  delay(5);
}
