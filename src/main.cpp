

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ArduinoLog.h>
#include <esp_sleep.h>
// #include <WiFi.h>


#define DEBUG 1
#define BATTERY_PIN 35
#define DEEPSLEEP 0

#if DEEPSLEEP
#warning "Deep Sleep defined"
#else
#warning "Light Sleep defined"
#endif



typedef struct sendObject {
  uint32_t timestamp;
  uint32_t gpslong, gpslat, gpsalt;
  uint32_t speed, direction;
  uint16_t voltage;
} sendObject_t;

/*
Payload decoder function:
function decodeUplink(input) {
  var data = {};
  data.timestamp = (input.bytes[0]) + (input.bytes[1]<<8) + (input.bytes[2]<<16) +  (input.bytes[3]<<24);
  data.long = ((input.bytes[4]) + (input.bytes[5]<<8) + (input.bytes[6]<<16) +  (input.bytes[7]<<24)) / 10000.0;
  data.lat = ((input.bytes[8]) + (input.bytes[9]<<8) + (input.bytes[10]<<16) +  (input.bytes[11]<<24)) / 10000.0;
  data.alt = ((input.bytes[12]) + (input.bytes[13]<<8) + (input.bytes[14]<<16) +  (input.bytes[15]<<24)) / 100.0;
  data.speed = ((input.bytes[16]) + (input.bytes[17]<<8) + (input.bytes[18]<<16) +  (input.bytes[19]<<24)) / 100.0;
  data.direction = ((input.bytes[20]) + (input.bytes[21]<<8) + (input.bytes[22]<<16) +  (input.bytes[23]<<24)) / 100.0;
  data.voltage = ((input.bytes[24]) + (input.bytes[25]<<8)) / 100.0
  return {
    data: data
  };
}
}

*/

sendObject_t whereAmI;

#if DEEPSLEEP
#define RTCMAXOBJ 100
RTC_NOINIT_ATTR sendObject_t objbuffer[RTCMAXOBJ];
RTC_NOINIT_ATTR int buflen, rtcqueued;
RTC_NOINIT_ATTR int RTCseqnoUp, RTCseqnoDn;
RTC_NOINIT_ATTR u4_t otaaDevAddr;
RTC_NOINIT_ATTR u1_t otaaNetwKey[16];
RTC_NOINIT_ATTR u1_t otaaApRtKey[16];
#else
#define RTCMAXOBJ 1000
 sendObject_t objbuffer[RTCMAXOBJ];
 int buflen, rtcqueued;
 int RTCseqnoUp, RTCseqnoDn;
 u4_t otaaDevAddr;
 u1_t otaaNetwKey[16];
 u1_t otaaApRtKey[16];
#endif

static int txcounter = 0;

#include <TinyGPS++.h>
HardwareSerial GPS(1);
TinyGPSPlus gps;


bool setup_complete = false;
bool led_dynamic = true; // LED shows if system is asleep or not
bool gps_wait_for_lock = true;
uint32_t lastGPS = 0;

#include "tbeam.h"

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}



static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120;
const unsigned GPS_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26,33,32},
};

// forward declarations
void setup_I2C(void);

void storeFrameCounters() {
  RTCseqnoUp = LMIC.seqnoUp;
  RTCseqnoDn = LMIC.seqnoDn;
  Log.verbose(F("Counters saved. up=%d down=%d"),RTCseqnoUp,RTCseqnoDn);
}

void restoreFrameCounters() {
  LMIC.seqnoUp = RTCseqnoUp;
  LMIC.seqnoDn = RTCseqnoDn;
  Log.verbose(F("Counters restored. up=%d down=%d"),RTCseqnoUp,RTCseqnoDn);

}

bool pushrtcbuffer(sendObject_t *o) {
  if (buflen < RTCMAXOBJ) {
    memcpy((objbuffer + buflen),o,sizeof(sendObject_t));
    buflen++;
    return true;
  }
  return false;
}

sendObject_t *toprtcbuffer() {
  if (buflen > 0) {
    return objbuffer + (buflen-1);
  } else {
    return NULL;
  }

}

sendObject_t *poprtcbuffer() {
  if (buflen > 0) {
    buflen--;
    return objbuffer + buflen;
  } else {
    return NULL;
  }
}


void dumprtcbuffer() {
  #if DEBUG

  Log.verbose(F("buflen = %d"),buflen);
  #endif
}

void setup_serial() {
  Serial.begin(115200);
#if DEBUG
  while (!Serial);
#endif
}

float getBatteryVoltage() {
  float vBat;
  // we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
  vBat = analogRead(BATTERY_PIN) * 2.0 * (3.3 / 1024.0);
  Log.verbose(F("vBat = %F"),vBat);
  return vBat;
}

void read_GPS() {
  Log.verbose(F("readGPS start"));
  if (!GPS) {
    Log.verbose(F("Initializing GPS"));
    GPS.begin(9600, SERIAL_8N1, 12, 15);
  }
  Log.verbose(F("Reading GPS"));
  unsigned long start = millis();
  do {
    os_runloop_once();
    while (GPS.available() > 0) {
      os_runloop_once();
      gps.encode(GPS.read());
      if (gps.charsProcessed() > 10)
        gps_wait_for_lock = false;
    }
  } while (millis() - start < 2000 || gps_wait_for_lock);
if (gps.charsProcessed() < 10) {
    Log.notice(F("No GPS data received"));
  } else {
    if (gps.location.isValid() && gps.location.isUpdated()) {
      Log.notice(F("GPS data: lat(%F) long(%F) height(%F)"),
        (double)(gps.location.lat()),
        (double)(gps.location.lng()),
        (double)(gps.altitude.meters()));

      whereAmI.gpslong = (uint32_t) (gps.location.lng() * 10000);
      whereAmI.gpslat = (uint32_t) (gps.location.lat() * 10000);
      whereAmI.gpsalt = (uint32_t) (gps.altitude.meters() * 100);
      whereAmI.speed = (uint32_t) (gps.speed.kmph() * 100);
      whereAmI.direction = (uint32_t) (gps.course.deg() * 100);

      Log.verbose(F("GPS movement: speed(%F km/h) deg(%F) "),
        gps.speed.kmph(),
        gps.course.deg());


      struct tm tm;
      tm.tm_sec=gps.time.second();
      tm.tm_min=gps.time.minute();
      tm.tm_hour=gps.time.hour();
      tm.tm_mday=gps.date.day();
      tm.tm_mon=gps.date.month()-1;
      tm.tm_year=gps.date.year()-1900;

      whereAmI.timestamp = (uint32_t) mktime(&tm);
      Log.verbose(F("Unix time %l"),whereAmI.timestamp);
      Log.verbose(F("ctime = %s"),ctime((time_t *) &whereAmI.timestamp));

      whereAmI.voltage = (uint16_t)(getBatteryVoltage() * 100.0);
      pushrtcbuffer(&whereAmI);
    }
  }
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 :
      Log.notice(F("Wakeup caused by external signal using RTC_IO"));
      break;
    case ESP_SLEEP_WAKEUP_EXT1 :
      Log.notice(F("Wakeup caused by external signal using RTC_CNTL"));
      break;
    case ESP_SLEEP_WAKEUP_TIMER :
      Log.notice(F("Wakeup caused by timer"));
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
      Log.notice(F("Wakeup caused by touchpad"));
      break;
    case ESP_SLEEP_WAKEUP_ULP :
      Log.notice(F("Wakeup caused by ULP program"));
      break;
    default :
      Log.notice(F("Wakeup was not caused by deep sleep: %d\n"),wakeup_reason); break;
  }
}

void sleepfor (int s) {
  if (!setup_complete)
    return;

  storeFrameCounters();
  Log.verbose(F("sleepfor(%d)"),s);
  esp_sleep_enable_timer_wakeup(s * 1000000);
  #if DEEPSLEEP
  esp_deep_sleep_start();
  #endif
  Serial.flush();
  Log.verbose(F("light sleep: %d"),esp_light_sleep_start());

}


void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      Log.notice(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
      // send data
      if (buflen > 0) { //  && rtcqueued == 0) {
        sendObject_t *o;
        dumprtcbuffer();
        o = toprtcbuffer();
        if (o != NULL) {
          rtcqueued++;
          LMIC_setTxData2(1, (unsigned char *) o, sizeof(sendObject_t), 1);
          Log.verbose(F("Packet queued"));
        }
      } else {
        sleepfor(TX_INTERVAL);
      }
    }
    // Next TX is scheduled after TX_COMPLETE event.

}

// -------------- Command Processing -----------------
void process_system_led_command(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  switch (buffer[0]) {
    case 0:
      led_dynamic = false;
      pinMode(LED_BUILTIN,LOW);
      break;
    case 1:
      led_dynamic = false;
      pinMode(LED_BUILTIN, HIGH);
      break;
    case 0xff:
      led_dynamic = true;
      break;
    default:
      Log.error(F("Unknown LED command %d"), buffer[0]);
      break;
  }
}

void process_system_set_epoch(unsigned char len, unsigned char *buffer) {
  if (len != 4) {
    Log.error(F("Epoch has wrong len (%d != 4)"),len);
    return;
  }
}

void process_system_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length system command"));
    return;
  }
  switch (buffer[0]) {
    case 0x03:
      process_system_led_command(len-1,buffer+1);
      break;
    case 0x04:
      process_system_set_epoch(len-1,buffer+1);
      break;
  }
}

void process_sensor_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length sensor command"));
    return;
  }
}

void process_received_lora(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  Log.verbose(F("Processing %d bytes of received data"),len);
  switch (buffer[0]) {
    case 0:
      process_system_command(len-1,buffer+1);
      break;
    case 1:
      process_sensor_command(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown command %d"),buffer[0]);
      break;
  }
}

// -----------------------
void onEvent (ev_t ev) {
    Log.verbose(F("onEvent:"));

    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Log.verbose(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Log.verbose(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Log.verbose(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Log.verbose(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Log.verbose(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Log.verbose(F("EV_JOINED"));
            otaaDevAddr = LMIC.devaddr;
            memcpy_P(otaaNetwKey, LMIC.nwkKey, 16);
            memcpy_P(otaaApRtKey, LMIC.artKey, 16);
            Log.verbose(F("got devaddr = 0x%X"), LMIC.devaddr);
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Log.verbose(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Log.verbose(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Log.verbose(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:

            Log.verbose(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
              Log.verbose(F("Received ack"));
              txcounter = 0;
              poprtcbuffer();
              if (rtcqueued > 0)
                rtcqueued--;

            }
            if (LMIC.dataLen) {
              Log.verbose(F("Received %d bytes of payload"),LMIC.dataLen);
              process_received_lora(LMIC.dataLen,LMIC.frame);
            }

            storeFrameCounters();
            // Schedule next transmission immediately
            os_setCallback(&sendjob,do_send);
            break;
        case EV_LOST_TSYNC:
            Log.verbose(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Log.verbose(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Log.verbose(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Log.verbose(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Log.verbose(F("EV_LINK_ALIVE"));
            break;
        case EV_SCAN_FOUND:
            Log.verbose(F("EV_SCAN_FOUND"));
            break;
        case EV_TXSTART:
            Log.verbose(F("EV_TXSTART: %d"),++txcounter);
            if (txcounter > 4)
              sleepfor(TX_INTERVAL);
            break;
        case EV_TXCANCELED:
            Log.verbose(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            Log.verbose(F("EV_RXSTART"));
            break;
        case EV_JOIN_TXCOMPLETE:
            Log.verbose(F("EV_JOIN_TXCOMPLETE: no join accepted"));
            break;
         default:
            Log.verbose(F("Unknown event %d"),ev);
            break;
    }
}



// Logging helper routines
void printTimestamp(Print* _logOutput) {
  char c[12];
  sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
  _logOutput->print('\n');
}


void setup_logging() {
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

void setOrRestorePersistentCounters() {
  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason != ESP_RST_DEEPSLEEP) && (reason != ESP_RST_SW)) {
    LMIC.seqnoUp = 0;
    LMIC.seqnoDn = 0;
    buflen = 0;
    rtcqueued = 0;
    gps_wait_for_lock = true;
    Log.verbose(F("rtc data initialized"));
  } else {
    restoreFrameCounters();
    gps_wait_for_lock = false;
  }
  Log.verbose(F("LMIC.seqnoUp = %d"),LMIC.seqnoUp);
  Log.verbose(F("LMIC.seqnoDn = %d"),LMIC.seqnoDn);
  Log.verbose(F("buflen = %d"),buflen);
}


void setup() {
  // WiFi.mode(WIFI_OFF);
  btStop();

  adcAttachPin(BATTERY_PIN);
  // adcStart(BATTERY_PIN);
  analogReadResolution(10);

  setup_serial();
  delay(5000);

  setup_logging();
  print_wakeup_reason(); // if wakeup
  // LMIC init
  os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();


  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason == ESP_RST_DEEPSLEEP) || (reason == ESP_RST_SW)) {
    LMIC_setSession(0x1, otaaDevAddr, otaaNetwKey, otaaApRtKey);
  }
  LMIC_setLinkCheckMode(0);

  setOrRestorePersistentCounters();

  // setup gps
  Log.verbose(F("Initializing GPS"));
  GPS.begin(9600, SERIAL_8N1, 12, 15);
}


void loop() {
  os_runloop_once();

  if (!setup_complete || (millis() > lastGPS + (GPS_INTERVAL * 1000))) {
    Log.verbose("main loop: reading GPS");
    read_GPS();
    lastGPS = millis();
    if (buflen > 0) {
      do_send(&sendjob);
    }
  }
  setup_complete = true;

}
