

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ArduinoLog.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
// #include <WiFi.h>


#define DEBUG 1
#define BATTERY_PIN 35
#define DEEPSLEEP 0
#define DO_OTAA 1


#if DEEPSLEEP
#pragma message "Deep Sleep defined"
#else
#pragma message "Light Sleep defined"
#endif

#if DO_OTAA
#pragma message "OTAA Activation defined"
#else
#pragma message "APB Activation defined"
#endif




typedef struct sendObject {
  uint32_t timestamp;
  uint32_t gpslong, gpslat, gpsalt;
  uint32_t speed, direction;
  uint16_t voltage;
  uint8_t listlen;
} sendObject_t;

typedef struct list {
  sendObject_t *o;
  bool queued;
  struct list *nxt,*prv;
} list_t;

list_t *firstInList = NULL,
  *lastInList = NULL,
  *deletedList;



sendObject_t whereAmI;
float lastlat = 0, lastlong = 0;
uint16_t lastVoltage = 0;
uint8_t nopushfor = 0;

#if DEEPSLEEP
RTC_NOINIT_ATTR int buflen, rtcqueued;
RTC_NOINIT_ATTR int RTCseqnoUp, RTCseqnoDn;
RTC_NOINIT_ATTR u4_t otaaDevAddr;
RTC_NOINIT_ATTR u1_t otaaNetwKey[16];
RTC_NOINIT_ATTR u1_t otaaApRtKey[16];
#endif

static int txcounter = 0;
static int sendingType = 1; // 1 = confirmed, 0 = unconfirmed

#include <TinyGPS++.h>
HardwareSerial GPS(1);
TinyGPSPlus gps;


bool setup_complete = false;
bool doNotSleep = true;
bool led_dynamic = true; // LED shows if system is asleep or not
bool gps_wait_for_lock = true;
uint32_t lastGPS = 0;
//  GPS interval dependent on speed of movement
// next time in millis() to get GPS
uint32_t nextGPS = 0, nextLORA = 0, loraDelta = 0;

#if DO_OTAA
#include "tbeam.h"
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
#else
#include "tbeam-02.h"
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif


static osjob_t sendjob;



// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26,33,32},
};

// forward declarations
void setup_I2C(void);

#if DEEPSLEEP
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
#endif

bool pushrtcbuffer(sendObject_t *o) {
  list_t *new_member;


  if (psramFound()) {
    new_member = (list_t *)ps_malloc(sizeof(list_t));
    if (new_member)
      new_member->o = (sendObject_t *)ps_malloc(sizeof(sendObject_t));
  } else {
    new_member = (list_t *)malloc(sizeof(list_t));
    if (new_member)
      new_member->o = (sendObject_t *)malloc(sizeof(sendObject_t));
  }

  // check if we have generated an object
  if (new_member && new_member->o) {
    memcpy(new_member->o,o,sizeof(sendObject_t));
    new_member->queued = false;

    // add it to the chain at the end
    // special case - chain is empty
    if (lastInList == NULL) {
      lastInList = new_member;
      firstInList= new_member;
      new_member->prv = NULL;
      new_member->nxt = NULL;
    } else {
      new_member->prv = lastInList;
      new_member->nxt = NULL;
      lastInList->nxt = new_member;
      lastInList = new_member;
    }
    return true;
  } else {
    return false;
  }
}

sendObject_t *poprtcbuffer() {
  // move last object of list to deletedList
  if (lastInList == NULL || lastInList->queued == false) {
    // nothing to do, list is empty
    return NULL;
  } else {
    lastInList->nxt = deletedList;
    deletedList = lastInList;
    lastInList = deletedList->prv;
    deletedList->prv = NULL;
    if (lastInList != NULL)
      lastInList->nxt = NULL;
    return deletedList->o;
  }
}

sendObject_t *poprtcbuffer2() {
  static sendObject_t o;
  // remove last object of list
  if (lastInList == NULL || lastInList->queued == false) {
    // nothing to do, list is empty
    return NULL;
  } else {
    // do we have an object to copy?
    if (lastInList->o == NULL) {
      // no - dump the last element and return NULL
      list_t *dumpit;
      dumpit = lastInList;
      lastInList = dumpit->prv;
      if (lastInList == NULL)
        firstInList = NULL;
      else
        lastInList->nxt = NULL;
      free(dumpit);
      return NULL;
    } else {
      memcpy(&o,lastInList->o,sizeof(sendObject_t));
      free(lastInList->o);
      list_t *dumpit;
      dumpit = lastInList;
      lastInList = dumpit->prv;
      if (lastInList == NULL)
        firstInList = NULL;
      else
        lastInList->nxt = NULL;
      free(dumpit);
      return &o;
    }
  }
}

uint32_t rtcbuflen() {
  list_t *o = firstInList;
  uint32_t len = 0;

  while (o) {
    len++;
    o = o->nxt;
  }
  return len;
}

void dumprtcbuffer() {
  #if DEBUG
  int buflen = 0;
  list_t *o;
  o = firstInList;
  while (o && buflen < 10000) {
    buflen++;
    o = o->nxt;
  }

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
    nextGPS = millis() + 1*60*1000; // try again in 1 minute
  } else {
    if (gps.location.isValid() && gps.location.isUpdated()) {
      lastGPS = millis();
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

      // only push if we have changed location
      float gpsdelta = abs(gps.location.lng()-lastlong) +
            abs(gps.location.lat()-lastlat);
      if (gpsdelta > 0.0002 &&
          int(gps.location.lng()) != 0 &&
          int(gps.location.lat()) != 0
          ) {
        pushrtcbuffer(&whereAmI);
        lastlat = gps.location.lat();
        lastlong= gps.location.lng();
        lastVoltage = whereAmI.voltage;
        nopushfor = 0;
      } else {
        Log.verbose(F("GPS delta too small (%F), not pushing"),gpsdelta);
      }

      // calculate next data gathering
      // speed = 0 -> 5 minutes
      // speed < 3 -> 2 minutes
      // speed < 30 -> 1 minute
      // speed < 100 -> 30s
      // speed > 100 -> 10s

      if (whereAmI.speed == 0)
        nextGPS = lastGPS+(60*1000);
      else
        nextGPS = lastGPS+10*1000;
    } else {
      Log.verbose(F("GPS valid: %T GPS update: %T"),
        gps.location.isValid(),
        gps.location.isUpdated());
      nextGPS = millis() + 5*1000;
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

  Log.verbose(F("sleepfor(%d seconds)"),s);
  esp_sleep_enable_timer_wakeup(s * 1000000);
  #if DEEPSLEEP
  storeFrameCounters();
  esp_deep_sleep_start();
  #endif
  Serial.flush();
  Log.verbose(F("light sleep complete: %d"),esp_light_sleep_start());

}


void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      Log.notice(F("OP_TXRXPEND, not sending"));
      loraDelta += 10;
      // LMIC_clrTxData();
    } else {
        // Prepare upstream data transmission at the next possible time.
      // send data
      if (lastInList != NULL) { //
        sendObject_t *o;
        dumprtcbuffer();
        o = lastInList->o;
        if (o != NULL) {
          uint32_t qlen = rtcbuflen();
          lastInList->queued = true;
          o->listlen = (qlen > 255) ? 255 : qlen;
          doNotSleep = true;
          LMIC_setTxData2(1,
            (unsigned char *) o,
            sizeof(sendObject_t),
            sendingType);
          Log.verbose(F("Packet queued timestamp %l"),o->timestamp);
        }
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
    case 0x07:
      // erase deleted queue
      break;
    case 0xff:
      // Reboot
      break;
  }
}

void process_sensor_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length sensor command"));
    return;
  }
}

void process_transmission_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length transmission command"));
    return;
  }
  switch (buffer[0]) {
    case 0x10:
      sendingType = 0;
      loraDelta = 1;
      break;
    case 0x11:
      sendingType = 1;
      break;
    case 0x1f:
      sendingType = 1;
      break;
    case 0x20:
      // send complete queue now
      break;
    case 0x21:
      // send deleted queue now
      break;
    default:
      Log.error(F("Unknown transmission command 0x%X"),buffer[0]);
  }
  Log.notice(F("sendingType  %d"),sendingType);

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
    case 2:
      process_transmission_command(len-1,buffer+1);
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
            // give it time to join
            loraDelta = 120;
            Log.verbose(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Log.verbose(F("EV_JOINED"));
            #if DEEPSLEEP
            otaaDevAddr = LMIC.devaddr;
            memcpy_P(otaaNetwKey, LMIC.nwkKey, 16);
            memcpy_P(otaaApRtKey, LMIC.artKey, 16);
            #endif
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
              txcounter = 0;
              poprtcbuffer();
              loraDelta =10;
              nextLORA = millis() + (1000*loraDelta);
              doNotSleep = false;
              Log.verbose(F("Received ack, loraDelta=%d"),loraDelta);
            }
            if (LMIC.dataLen) {
              Log.verbose(F("Received %d bytes of payload"),LMIC.dataLen);
              process_received_lora(LMIC.dataLen,LMIC.frame);
            }
#if DEEPSLEEP
            storeFrameCounters();
#endif
            // Schedule next transmission immediately
            // os_setCallback(&sendjob,do_send);
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
          if (loraDelta < 60*5) {
            loraDelta++;
            loraDelta *= 2;
          }
          Log.verbose(F("EV_TXSTART: txcounter=%d loraDelta=%d"),
            ++txcounter,
            loraDelta);
          if (txcounter >= 4)
            doNotSleep = false;
            break;
        case EV_TXCANCELED:
            Log.verbose(F("EV_TXCANCELED"));
            txcounter = 0;
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

void log_memory(bool all=true) {
  if (all) {
    Log.verbose(F("ESP total heap  %d"),ESP.getHeapSize());
    Log.verbose(F("ESP total PSRAM %d"),ESP.getPsramSize());
  }
  Log.verbose(F("ESP free  heap %d"),ESP.getFreeHeap());
  Log.verbose(F("ESP free PSRAM %d"),ESP.getFreePsram());
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

#if DEEPSLEEP
void setOrRestorePersistentCounters() {
  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason != ESP_RST_DEEPSLEEP) && (reason != ESP_RST_SW)) {
    LMIC.seqnoUp = 0;
    LMIC.seqnoDn = 0;
    gps_wait_for_lock = true;
    Log.verbose(F("rtc data initialized"));
  } else {
    restoreFrameCounters();
    gps_wait_for_lock = false;
  }
  Log.verbose(F("LMIC.seqnoUp = %d"),LMIC.seqnoUp);
  Log.verbose(F("LMIC.seqnoDn = %d"),LMIC.seqnoDn);
}
#endif

void setup() {
  // WiFi.mode(WIFI_OFF);
  btStop();
  esp_wifi_stop();

  adcAttachPin(BATTERY_PIN);
  // adcStart(BATTERY_PIN);
  analogReadResolution(10);

  setup_serial();
  delay(5000);

  setup_logging();
  print_wakeup_reason(); // if wakeup
  log_memory(true);
  // LMIC init
  os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#if DO_OTAA
  #if DEEPSLEEP
  esp_reset_reason_t reason = esp_reset_reason();
  if ((reason == ESP_RST_DEEPSLEEP) || (reason == ESP_RST_SW)) {
    LMIC_setSession(0x1, otaaDevAddr, otaaNetwKey, otaaApRtKey);
  }
  setOrRestorePersistentCounters();
  #endif
#else
LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7,14);


  // setup gps
  Log.verbose(F("Initializing GPS"));
  GPS.begin(9600, SERIAL_8N1, 12, 15);
  nextGPS = millis();
  nextLORA= millis();
  doNotSleep = true;
}


void loop() {
  os_runloop_once();

  if (!setup_complete || (millis() > nextGPS)) {
    Log.verbose("main loop: reading GPS");
    read_GPS();
    Log.verbose("main loop: nextGPS = %l",nextGPS);
    nextLORA = millis()-1;

  }

  if (!setup_complete || (millis() > nextLORA)) {
    if (lastInList != NULL) {
      Log.verbose("main loop: sending loraDelta = %d",loraDelta);
      do_send(&sendjob);
      nextLORA = millis() + (1000*loraDelta);
      Log.verbose("main loop: nextLORA = %l",nextLORA);

    } else { // nothing queued
      nextLORA = nextGPS+1;
      doNotSleep = false;
    }
  }

  long int sleeptime = min(nextLORA-millis(),nextGPS-millis());
  if (!doNotSleep && sleeptime > 30000 && setup_complete) {
    Log.verbose("nextLora = %l",nextLORA);
    Log.verbose("nextGPS  = %l",nextGPS);
    Log.verbose("sleeptime= %l",sleeptime);
    LMIC_clrTxData();
    txcounter=0;
    sleepfor(sleeptime / 1000);
  }

  setup_complete = true;


}
