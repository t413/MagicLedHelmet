
#define USE_BLE
#include <FastLED.h>

#define DATA_PIN    13
#define NUM_LEDS    56
#define BRIGHTNESS         30
#define FRAMES_PER_SECOND  256
FASTLED_USING_NAMESPACE
CRGB leds[NUM_LEDS];

#ifdef USE_BLE
#include <BLEDevice.h>
static BLEAddress   myaddr("08:08:08:08:08:01");
static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb"); //use fff0 to find it, this to connect
static BLEUUID    charUUID("ffe1");
static BLEClient* myDevice = NULL;
static BLERemoteCharacteristic* pRemChar = NULL;
#endif

#define BREAKING_MIN 8
#define BREAKING_MAX 20
#define BREAKING_LEDS 8
#define QUICKSPEED 3000 //30kph 18mph

String str(const char *fmtStr, ...) {
  static char buf[200] = {'\0'};
  va_list arg_ptr;
  va_start(arg_ptr, fmtStr);
  vsprintf(buf, fmtStr, arg_ptr);
  va_end(arg_ptr);
  return String(buf);
}
String str(std::string s) { return String(s.c_str()); }
String str(bool v) { return v? " WIN" : " FAIL"; }

struct WheelData {
  int voltage;
  int speed, lastspeed;
  float dspeedfilt, speedfilt;
  int totalDist;
  int current;
  int temp;
  uint32_t time;
  String toString() const {
    return str("[%dV %0.1fdam/h %ddam %dA %dC]", voltage, speedfilt, totalDist, current, temp);
  }
};

WheelData wheelDat = {0};

int byteArrayInt2(byte low, byte high) {
  return (low & 255) + ((high & 255) * 256);
}

long byteArrayInt4(byte value1, byte value2, byte value3, byte value4) {
  return (((((long) ((value1 & 255) << 16))) | ((long) ((value2 & 255) << 24))) | ((long) (value3 & 255))) | ((long) ((value4 & 255) << 8));
}

#ifdef USE_BLE
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,uint8_t* pData,size_t length,bool isNotify) {
  uint32_t now = millis();
  if (length >= 20) {
    if (pData[0] != 170 || (pData[1] != 85)) return; //check header
    if (pData[16] == 169) { //regular data
      int16_t dspeeddt = (wheelDat.speed - wheelDat.lastspeed) * 1000 / (now - wheelDat.time);
      wheelDat.dspeedfilt -= 0.2 * (wheelDat.dspeedfilt - (dspeeddt));
      wheelDat.lastspeed = wheelDat.speed;
      wheelDat.voltage = byteArrayInt2(pData[2], pData[3]);
      wheelDat.speed = byteArrayInt2(pData[4], pData[5]);
      wheelDat.speedfilt -= 0.25 * (wheelDat.speedfilt - (wheelDat.speed));
      wheelDat.totalDist = byteArrayInt4(pData[6], pData[7], pData[8], pData[9]);
      wheelDat.current = ((pData[10]&0xFF) + (pData[11]<<8));
      wheelDat.temp = byteArrayInt2(pData[12], pData[13]);
      wheelDat.time = now;
      Serial.println("real data " + wheelDat.toString());
    } else if (pData[16] == 187) { //name report
      //TODO send serial request?
      Serial.print("GOT NAME");
      Serial.print("notify data: ");
      for(int i = 0; i < length; i++)
        Serial.print(pData[i], HEX);
      Serial.println();
    } else {
      // Serial.print("notify data: ");
      // for(int i = 0; i < length; i++)
      //   Serial.print(pData[i], HEX);
      // Serial.println();
    }
  }
}

bool connect(BLEClient* dev) {
  if (!dev) return false;
  Serial.println("* Connecting to: " + str(myaddr.toString()));
  bool res = myDevice->connect(myaddr);
  Serial.println("* Connected to server" + str(res));
  if (!res) return false;

  BLERemoteService* serv = dev->getService(serviceUUID);
  Serial.println("* Service to " + str(serv->toString()) + str(!!serv));
  if (!serv) { dev->disconnect(); return false; }

  pRemChar = serv->getCharacteristic(charUUID);
  Serial.println("* Characteristic to " + str(charUUID.toString()) + str(!!pRemChar));
  if (!pRemChar) { dev->disconnect(); return false; }

  if (!pRemChar->canNotify())
    Serial.println("* Can't register to be notified!?");
  pRemChar->registerForNotify(notifyCallback);

  uint8_t askname[20] = {170, 85, 0,0,0,0,0,0,0,0,0,0,0,0,0,0, 155, 20, 90, 90};
  pRemChar->writeValue(askname, sizeof(askname));
  return true;
}

void connectTask(void* params) {
  while (true) {
    if (myDevice && ! myDevice->isConnected()) {
      Serial.println("connecting device from task");
      bool res = connect(myDevice);
      Serial.println("reconnected: " + str(res));
    }
    vTaskDelay(5000);
  }
}
#endif

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2811,DATA_PIN,GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

#ifdef USE_BLE
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  myDevice = BLEDevice::createClient();
  xTaskCreate(connectTask, "connectTask", 10000, NULL, 1, NULL); //fn, name, stack size, parameter, priority, handle
#endif
}

#define MAX_VOXEL (NUM_LEDS/2)

int mapc(int v, int a, int b, int c, int d) { return constrain(::map(v, a, b, c, d), c, d); }

class Voxel {
  uint32_t starttime;
  uint16_t speed;
public:
  Voxel(uint16_t s=0, uint32_t t=0) : starttime(t), speed(s) { }
  String toString(uint32_t now) const { return str("<v s%d,%dmsa> ", speed, now - starttime); }
  bool valid(uint32_t now) const { return speed > 0 && getLedPos(now) < MAX_VOXEL; }
  uint8_t reset() { speed = 0; }
  uint8_t getLedPos(uint32_t now) const {
    uint16_t beat = ((now - starttime) * (speed << 12)) >> 16;
    return ::map(beat, 0, SHRT_MAX, 0, MAX_VOXEL); // = 1/(10/60/60 * 23leds/27cm)
  }
    // map(pos, 0, MAX_POS, 0, NUM_LEDS/2);
};
#define VOXELS_NUM 6
Voxel voxels[VOXELS_NUM] = {};
void enqueVox(int speed, uint32_t now) {
  for (int i = 0; i < VOXELS_NUM; i++)
    if (! voxels[i].valid(now)) {
      voxels[i] = Voxel(speed, now);
      return;
    }
}

//true=right-side
CRGB& side(bool side, int16_t pos) {
  if (side) return leds[constrain(MAX_VOXEL     + pos, MAX_VOXEL, NUM_LEDS)];
  else      return leds[constrain(MAX_VOXEL - 1 - pos,         0, MAX_VOXEL)];
}

uint16_t newVoxThresh = 3;
uint16_t fps = 0;

void loop() {
  uint32_t now = millis();
  fps++;
  EVERY_N_MILLISECONDS(350) { newVoxThresh = mapc(wheelDat.speedfilt, 0, QUICKSPEED, 4, 2*MAX_VOXEL/3); }
  EVERY_N_MILLISECONDS(1000) { Serial.printf("  (%d fps, thresh %d)\n", fps, newVoxThresh); fps = 0; }

  while (Serial.available()) {
    int v = Serial.parseInt() * 100;
    while (Serial.available() > 0)
       Serial.read();
    Serial.printf("read in %d\n", v);
    wheelDat.speedfilt = wheelDat.speed = v;
    wheelDat.time = now;
  }


  if ((now - wheelDat.time) < 4000) { //valid data!
    fadeToBlackBy(leds, NUM_LEDS, 40);
    uint8_t bright = fadeit(wheelDat.speedfilt, 100, 800); //1-8kph
    if (bright < 253) { //when going slow do this pattern
      juggle(quadwave8(now >> 6) / 6 + 165, 255 - bright); //color specific
    }
    if (bright > 0) {

      fadeToBlackBy(leds, NUM_LEDS, mapc(wheelDat.speedfilt, 0, QUICKSPEED, 0, 80));
      uint8_t minvoxp = 255;
      for (int i = 0; i < VOXELS_NUM; i++) {
        if (! voxels[i].valid(now)) { voxels[i].reset(); continue; }
        uint8_t pos = voxels[i].getLedPos(now);
        uint8_t spread = mapc(wheelDat.speedfilt, 0, QUICKSPEED, 1, 3);
        CHSV c1(quadwave8((now >> 5) + 88) / 6 + 160, 220, 255);
        CHSV c2(quadwave8( now >> 5      ) / 6 + 160, 220, 255);
        //TODO when speed > 3000 (18mph) GO RED
        for (uint8_t s = 0; s < spread; s++) {
          c1.v = c2.v = 255 - s*16;
          side(1, pos - s) |= c1; //right side
          side(0, pos - s) |= c2; //left side
        }
        minvoxp = min(minvoxp, pos);
      }
      if (minvoxp > newVoxThresh) //add new voxel
        enqueVox(wheelDat.speedfilt, now);

      if (wheelDat.dspeedfilt < -BREAKING_MIN) {
        for (int i = 0; i < BREAKING_LEDS; i++) {
          CHSV clr(0, 255, ::map(wheelDat.dspeedfilt, -BREAKING_MIN, -BREAKING_MAX, 0, 255)); //hue 0=red
          leds[i]                |= clr;
          leds[NUM_LEDS - 1 - i] |= clr;
        }
      }
      //TODO do brake lights when stopping.. at all?
    }

  } else {
    fadeToBlackBy( leds, NUM_LEDS, 18);
    uint8_t bright = fadeit(quadwave8((now >> 6) + 50), 65, 191);
    juggle(now >> 8, bright);
    confetti((now >> 8) + 20, 255 - bright);
  }


  FastLED.show();
  FastLED.delay(1000/FRAMES_PER_SECOND);
  // EVERY_N_SECONDS( 5 ) { }
}


void juggle(uint8_t hue, uint8_t brightness) {
  // eight colored dots, weaving in and out of sync with each other
  // fadeToBlackBy( leds, NUM_LEDS, 40);
  byte dothue = 0;
  for( int i = 0; i < 10; i++) {
    leds[beatsin16( 3*i/2 + 2, 0, NUM_LEDS-1 )] |= CHSV(dothue + hue, 220, brightness);
    dothue += 6;
  }
}

void confetti(uint8_t hue, uint8_t brightness) {
  // random colored speckles that blink in and fade smoothly
  //fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(hue + random8(64), 200, brightness);
}

uint8_t fadeit(int16_t in, int16_t minv, int16_t maxv) { //returns map between 0-255
  return ::map(constrain(in, minv, maxv), minv, maxv, 0, 255);
}
