#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>
#include <lmic.h>
#include <hal/hal.h>

Adafruit_BME280 bme; // I2C
BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes);

unsigned long delayTime = 10000;

char datasend[9];

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// Fuer Dauerbetrieb 180
const unsigned TX_INTERVAL = 180;

// Pin mapping for Dragino Shield
const lmic_pinmap lmic_pins = {
  .nss = 10,                      // chip select
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,                       // reset pin
  .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void setup() {
  Serial.begin(9600);
  if (! bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  LightSensor.begin();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // disable channels (only use channel 0 - 868.1 MHz)
  // LMIC_disableChannel(0);
  LMIC_disableChannel(1);
  LMIC_disableChannel(2);
  LMIC_disableChannel(3);
  LMIC_disableChannel(4);
  LMIC_disableChannel(5);
  LMIC_disableChannel(6);
  LMIC_disableChannel(7);
  LMIC_disableChannel(8);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC.datarate = DR_SF7;
  LMIC_setDrTxpow(LMIC.datarate, 20);
  LMIC.rps = updr2rps(LMIC.datarate);

  // Start job
  do_send(&sendjob);
}

int readTemp() {
  int temp = bme.readTemperature() * 100.0F;
  return temp;
}

int readBaro() {
  int baro = bme.readPressure() / 10.0F;
  return baro;
}

int readHumi() {
  int humi = bme.readHumidity() * 100.0F;
  return humi;
}

int readLux() {
  int lux = LightSensor.GetLightIntensity();
  return lux;
}

void printTemp(int temp) {
  Serial.print("Temperature = ");
  Serial.print(temp / 100.0F, 2);
  Serial.println(" *C");
}

void printBaro(int baro) {
  Serial.print("Pressure = ");
  Serial.print(baro / 10.0F, 1);
  Serial.println(" mbar (abs)");
}

void printHumi(int humi) {
  Serial.print("Humidity = ");
  Serial.print(humi / 100.0F, 2);
  Serial.println(" % (rel)");
}

void printLux(int lux) {
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    readAllValues();
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, datasend, sizeof(datasend) - 1, 0);
    Serial.println(F("Packet queued"));
    Serial.print(F("LMIC.freq:"));
    Serial.println(LMIC.freq);
    Serial.println("");
    Serial.println(F("Receive data:"));
    // Next TX is scheduled after TX_COMPLETE event.
  }
}

void readAllValues() {
  int temp = readTemp();
  int baro = readBaro();
  int humi = readHumi();
  int lux = readLux();
  Serial.println("---------- Values ----------");
  printTemp(temp);
  printBaro(baro);
  printHumi(humi);
  printLux(lux);
  Serial.println();
  datasend[0] = temp;
  datasend[1] = temp >> 8;
  datasend[2] = baro;
  datasend[3] = baro >> 8;
  datasend[4] = humi;
  datasend[5] = humi >> 8;
  datasend[6] = lux;
  datasend[7] = lux >> 8;
}

void loop() {
  if (!(LMIC.opmode & OP_TXRXPEND)) {
    readAllValues();
    delay(delayTime);
  }
  os_runloop_once();
}
