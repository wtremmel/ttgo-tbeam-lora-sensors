

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ArduinoLog.h>
#include <esp_sleep.h>
#include <WiFi.h>


#define DEBUG 1

#include <CayenneLPP.h>
CayenneLPP lpp(51);

#define RTCBUFSIZE (1024*3)
#define RTCMAXOBJ 10

typedef struct sendObject {
  uint32_t timestamp;
  uint32_t gpslong, gpslat, gpsalt;
  uint32_t speed, direction;
} sendObject_t;

sendObject_t whereAmI;

RTC_NOINIT_ATTR uint8_t rtcbufstart;
RTC_NOINIT_ATTR int rtcbufindex;
RTC_NOINIT_ATTR uint8_t rtcbuf[RTCBUFSIZE];
RTC_NOINIT_ATTR uint8_t rtcobjlen[RTCMAXOBJ];
RTC_NOINIT_ATTR uint16_t rtcobjstart[RTCMAXOBJ];



#include <TinyGPS++.h>
HardwareSerial GPS(1);
TinyGPSPlus gps;


bool setup_complete = false;
bool led_dynamic = true; // LED shows if system is asleep or not
bool gps_wait_for_lock = true;

#include "tbeam.h"

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

RTC_NOINIT_ATTR int RTCseqnoUp, RTCseqnoDn;
RTC_NOINIT_ATTR u4_t otaaDevAddr;
RTC_NOINIT_ATTR u1_t otaaNetwKey[16];
RTC_NOINIT_ATTR u1_t otaaApRtKey[16];


static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 90;

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

bool pushrtcbuffer(uint8_t* obj, uint8_t size) {
  if (((rtcbufstart + size) <= RTCBUFSIZE) &&
      (rtcbufindex < RTCMAXOBJ)) {
    memcpy(rtcbuf + rtcbufstart,obj,size);
    rtcbufindex++;
    rtcobjstart[rtcbufindex] = rtcbufstart;
    rtcobjlen[rtcbufindex] = size;
    rtcbufstart += size;
    return true;
  }
  return false;
}

 bool poprtcbuffer() {
   // dump the last object from the buffer
   if (rtcbufindex >= 0) {
     rtcbufstart -= rtcobjlen[rtcbufindex];
     rtcbufindex--;
     return true;
   }
   return false;
 }

void dumprtcbuffer() {
  #if DEBUG
  int i;
  Log.verbose(F("RTCbufindex = %d"),rtcbufindex);
  Log.verbose(F("RTCbufstart = %d"),rtcbufstart);
  for (i=0; i <= rtcbufindex; i++) {
    Log.verbose(F("RTC entry %d len = %d"),i,rtcobjlen[i]);
  }
  #endif
}

void setup_serial() {
  Serial.begin(115200);
#if DEBUG
  while (!Serial);
#endif
}

// ----------- Battery stuff
#define HAS_BATTERY_PROBE ADC1_GPIO35_CHANNEL
#define BATT_FACTOR 2

#define DEFAULT_VREF 1100 // tbd: use adc2_vref_to_gpio() for better estimate
#define NO_OF_SAMPLES 64  // we do some multisampling to get better values
#include <driver/adc.h>
#include <esp_adc_cal.h>
esp_adc_cal_characteristics_t *adc_characs =
    (esp_adc_cal_characteristics_t *)calloc(
        1, sizeof(esp_adc_cal_characteristics_t));

static const adc1_channel_t adc_channel = HAS_BATTERY_PROBE;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

void calibrate_voltage(void) {
  // configure ADC
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(adc1_config_channel_atten(adc_channel, atten));
  // calibrate ADC
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_characs);
  // show ADC characterization base
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    ESP_LOGI(TAG,
             "ADC characterization based on Two Point values stored in eFuse");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    ESP_LOGI(TAG,
             "ADC characterization based on reference voltage stored in eFuse");
  } else {
    ESP_LOGI(TAG, "ADC characterization based on default reference voltage");
  }
}

float my_voltage() {
  // multisample ADC
  uint32_t adc_reading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw(adc_channel);
  }
  adc_reading /= NO_OF_SAMPLES;
  // Convert ADC reading to voltage in mV
  uint16_t voltage =
      (uint16_t)esp_adc_cal_raw_to_voltage(adc_reading, adc_characs);
#ifdef BATT_FACTOR
  voltage *= BATT_FACTOR;
#endif
  ESP_LOGD(TAG, "Raw: %d / Voltage: %dmV", adc_reading, voltage);
  return (float)(voltage) / 100.0;
}



void read_voltage() {
  float v = my_voltage();
  // lpp.addAnalogInput(5,v);
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
    while (GPS.available() > 0) {
      gps.encode(GPS.read());
      if (gps.charsProcessed() > 10)
        gps_wait_for_lock = false;
    }
  } while (millis() - start < 2000 || gps_wait_for_lock);
if (gps.charsProcessed() < 10) {
    Log.notice(F("No GPS data received"));
  } else {
    lpp.reset();
    if (gps.location.isValid() && gps.location.isUpdated()) {
      Log.notice(F("GPS data: lat(%F) long(%F) height(%F)"),
        (double)(gps.location.lat()),
        (double)(gps.location.lng()),
        (double)(gps.altitude.meters()));
      lpp.addGPS(2,gps.location.lat(), gps.location.lng(), gps.altitude.meters());

      whereAmI.gpslong = (uint32_t) (gps.location.lng() * 1000);
      whereAmI.gpslat = (uint32_t) (gps.location.lat() * 1000);
      whereAmI.gpsalt = (uint32_t) (gps.altitude.meters() * 100);
      whereAmI.speed = (uint32_t) (gps.speed.kmph() * 100);
      whereAmI.direction = (uint32_t) (gps.course.deg() * 100);

      Log.verbose(F("GPS movement: speed(%F km/h) deg(%F) "),
        gps.speed.kmph(),
        gps.course.deg());
      lpp.addDirection(3,gps.course.deg());

      struct tm tm;
      tm.tm_sec=gps.time.second();
      tm.tm_min=gps.time.minute();
      tm.tm_hour=gps.time.hour();
      tm.tm_mday=gps.date.day();
      tm.tm_mon=gps.date.month()-1;
      tm.tm_year=gps.date.year()+2000;

      Log.verbose(F("Unix time %l"),mktime(&tm));
      lpp.addUnixTime(1,mktime(&tm));
      whereAmI.timestamp = mktime(&tm);
      pushrtcbuffer(lpp.getBuffer(),lpp.getSize());
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
  esp_deep_sleep_start();
  Log.verbose(F("light sleep: %d"),esp_light_sleep_start());

}


void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      (F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
      // send data
      while (rtcbufindex >= 0) {
        dumprtcbuffer();
        // LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        LMIC_setTxData2(1, (unsigned char *) &whereAmI, sizeof(sendObject_t), 0);
        Log.verbose(F("Packet queued"));
        poprtcbuffer();
      }
      sleepfor(TX_INTERVAL);
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
            if (LMIC.txrxFlags & TXRX_ACK)
              Log.verbose(F("Received ack"));
            if (LMIC.dataLen) {
              Log.verbose(F("Received %d bytes of payload"),LMIC.dataLen);
              process_received_lora(LMIC.dataLen,LMIC.frame);
            }
            storeFrameCounters();
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
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
            Log.verbose(F("EV_TXSTART"));
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
    rtcbufstart = 0;
    rtcbufindex = -1;
    gps_wait_for_lock = true;
    Log.verbose(F("rtc data initialized"));
  } else {
    restoreFrameCounters();
    gps_wait_for_lock = false;
  }
  Log.verbose(F("LMIC.seqnoUp = %d"),LMIC.seqnoUp);
  Log.verbose(F("LMIC.seqnoDn = %d"),LMIC.seqnoDn);
  Log.verbose(F("rtcbufstart = %d"),rtcbufstart);
  Log.verbose(F("rtcbufindex = %d"),rtcbufindex);
}


void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();

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
  read_GPS();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}


void loop() {
  setup_complete = true;
  // Log.verbose(F("entering main loop"));
  os_runloop_once();
}
