
#define PROD

#ifdef PROD

#include "LMIC-node.h"
#include "Gps.h"
#include <CayenneLPP.h>
#include <WiFi.h>
#include "MPU6050.h"

#define EXT_BUTTON_PIN 38
#define IMU_INT_PIN 35
#define uS_TO_S_FACTOR 1000000ULL
// #define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

// const uint8_t payloadBufferLength = 32; // Adjust to fit max payload length

CayenneLPP lpp(51); // here we will construct Cayenne Low Power Payload (LPP) - see https://community.mydevices.com/t/cayenne-lpp-2-0/7510
GPS gps;            // class that is encapsulating additional GPS functionality
MPU6050 mpu(Wire);
RTC_DATA_ATTR int bootCount = 0;
int retryCount = 0;
RTC_DATA_ATTR lmic_t RTC_LMIC;
double lat, lon, alt, kmph; // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
// float tmp, hum, pressure, alt_barometric; // BME280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
int sats;   // GPS satellite count
char s[32]; // used to sprintf for Serial output
// bool status; // status after reading from BME280
// float vBat; // battery voltage
// long nextPacketTime;
bool goToDeepSleep = false;
bool goToDeepSleepWithIMUWakeup = false;

const unsigned int GPS_FIX_RETRY_DELAY = 10; // wait this many seconds when no GPS fix is received to retry
// const unsigned int SHORT_TX_INTERVAL = 20; // when driving, send packets every SHORT_TX_INTERVAL seconds
// const double MOVING_KMPH = 10.0; // if speed in km/h is higher than MOVING_HMPH, we assume that car is moving

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀

// uint8_t payloadBuffer[payloadBufferLength];
static osjob_t doWorkJob;
uint32_t doWorkIntervalSeconds = DO_WORK_INTERVAL_SECONDS; // Change value in platformio.ini

// Note: LoRa module pin mappings are defined in the Board Support Files.

// Set LoRaWAN keys defined in lorawan-keys.h.
#ifdef OTAA_ACTIVATION
static const u1_t PROGMEM DEVEUI[8] = {OTAA_DEVEUI};
static const u1_t PROGMEM APPEUI[8] = {OTAA_APPEUI};
static const u1_t PROGMEM APPKEY[16] = {OTAA_APPKEY};
// Below callbacks are used by LMIC for reading above values.
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#else
                                                           // ABP activation
static const u4_t DEVADDR = ABP_DEVADDR;
static const PROGMEM u1_t NWKSKEY[16] = {ABP_NWKSKEY};
static const u1_t PROGMEM APPSKEY[16] = {ABP_APPSKEY};
// Below callbacks are not used be they must be defined.
void os_getDevEui(u1_t *buf) {}
void os_getArtEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
#endif

void LoraWANPrintLMICOpmode(void)
{
    Serial.print(F("LMIC.opmode: "));
    if (LMIC.opmode & OP_NONE)
    {
        Serial.print(F("OP_NONE "));
    }
    if (LMIC.opmode & OP_SCAN)
    {
        Serial.print(F("OP_SCAN "));
    }
    if (LMIC.opmode & OP_TRACK)
    {
        Serial.print(F("OP_TRACK "));
    }
    if (LMIC.opmode & OP_JOINING)
    {
        Serial.print(F("OP_JOINING "));
    }
    if (LMIC.opmode & OP_TXDATA)
    {
        Serial.print(F("OP_TXDATA "));
    }
    if (LMIC.opmode & OP_POLL)
    {
        Serial.print(F("OP_POLL "));
    }
    if (LMIC.opmode & OP_REJOIN)
    {
        Serial.print(F("OP_REJOIN "));
    }
    if (LMIC.opmode & OP_SHUTDOWN)
    {
        Serial.print(F("OP_SHUTDOWN "));
    }
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.print(F("OP_TXRXPEND "));
    }
    if (LMIC.opmode & OP_RNDTX)
    {
        Serial.print(F("OP_RNDTX "));
    }
    if (LMIC.opmode & OP_PINGINI)
    {
        Serial.print(F("OP_PINGINI "));
    }
    if (LMIC.opmode & OP_PINGABLE)
    {
        Serial.print(F("OP_PINGABLE "));
    }
    if (LMIC.opmode & OP_NEXTCHNL)
    {
        Serial.print(F("OP_NEXTCHNL "));
    }
    if (LMIC.opmode & OP_LINKDEAD)
    {
        Serial.print(F("OP_LINKDEAD "));
    }
    if (LMIC.opmode & OP_LINKDEAD)
    {
        Serial.print(F("OP_LINKDEAD "));
    }
    if (LMIC.opmode & OP_TESTMODE)
    {
        Serial.print(F("OP_TESTMODE "));
    }
    if (LMIC.opmode & OP_UNJOIN)
    {
        Serial.print(F("OP_UNJOIN "));
    }
}

void LoraWANDebug(lmic_t lmic_check)
{
    Serial.println("");
    Serial.println("");

    LoraWANPrintLMICOpmode();
    Serial.println("");

    Serial.print(F("LMIC.seqnoUp = "));
    Serial.println(lmic_check.seqnoUp);

    Serial.print(F("LMIC.globalDutyRate = "));
    Serial.print(lmic_check.globalDutyRate);
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(lmic_check.globalDutyRate) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("LMIC.globalDutyAvail = "));
    Serial.print(lmic_check.globalDutyAvail);
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(lmic_check.globalDutyAvail) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("LMICbandplan_nextTx = "));
    Serial.print(LMICbandplan_nextTx(os_getTime()));
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(LMICbandplan_nextTx(os_getTime())) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("os_getTime = "));
    Serial.print(os_getTime());
    Serial.print(F(" osTicks, "));
    Serial.print(osticks2ms(os_getTime()) / 1000);
    Serial.println(F(" sec"));

    Serial.print(F("LMIC.txend = "));
    Serial.println(lmic_check.txend);
    Serial.print(F("LMIC.txChnl = "));
    Serial.println(lmic_check.txChnl);

    Serial.println(F("Band \tavail \t\tavail_sec\tlastchnl \ttxcap"));
    for (u1_t bi = 0; bi < MAX_BANDS; bi++)
    {
        Serial.print(bi);
        Serial.print("\t");
        Serial.print(lmic_check.bands[bi].avail);
        Serial.print("\t\t");
        Serial.print(osticks2ms(lmic_check.bands[bi].avail) / 1000);
        Serial.print("\t\t");
        Serial.print(lmic_check.bands[bi].lastchnl);
        Serial.print("\t\t");
        Serial.println(lmic_check.bands[bi].txcap);
    }
    Serial.println("");
    Serial.println("");
}

void PrintRuntime()
{
    long seconds = millis() / 1000;
    Serial.print("Runtime: ");
    Serial.print(seconds);
    Serial.println(" seconds");
}

void PrintLMICVersion()
{
    Serial.print(F("LMIC: "));
    Serial.print(ARDUINO_LMIC_VERSION_GET_MAJOR(ARDUINO_LMIC_VERSION));
    Serial.print(F("."));
    Serial.print(ARDUINO_LMIC_VERSION_GET_MINOR(ARDUINO_LMIC_VERSION));
    Serial.print(F("."));
    Serial.print(ARDUINO_LMIC_VERSION_GET_PATCH(ARDUINO_LMIC_VERSION));
    Serial.print(F("."));
    Serial.println(ARDUINO_LMIC_VERSION_GET_LOCAL(ARDUINO_LMIC_VERSION));
}

void SaveLMICToRTC(int deepsleep_sec)
{
    Serial.println(F("Save LMIC to RTC"));
    RTC_LMIC = LMIC;
    unsigned long now = millis();
    // EU Like Bands
#if defined(CFG_LMIC_EU_like)
    Serial.println(F("Reset CFG_LMIC_EU_like band avail"));
    for (int i = 0; i < MAX_BANDS; i++)
    {
        ostime_t correctedAvail = RTC_LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
        if (correctedAvail < 0)
        {
            correctedAvail = 0;
        }
        RTC_LMIC.bands[i].avail = correctedAvail;
    }

    RTC_LMIC.globalDutyAvail = RTC_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (RTC_LMIC.globalDutyAvail < 0)
    {
        RTC_LMIC.globalDutyAvail = 0;
    }
#else
    Serial.println(F("No DutyCycle recalculation function!"));
#endif
}

void LoadLMICFromRTC()
{
    Serial.println(F("Load LMIC from RTC"));
    LMIC = RTC_LMIC;
}

void getAxpStatus()
{
    // axp.setPowerOutPut(AXP192_DCDC1, false);
    Serial.println("Bat charge coulomb: " + String(axp.getBattChargeCoulomb()));
    Serial.println("DCDC1 voltage: " + String(axp.getDCDC1Voltage()));
    Serial.println("DCDC2 voltage: " + String(axp.getDCDC2Voltage()));
    Serial.println("DCDC3 voltage: " + String(axp.getDCDC3Voltage()));
    Serial.println("LDO2 voltage: " + String(axp.getLDO2Voltage()));
    Serial.println("LDO3 voltage: " + String(axp.getLDO3Voltage()));
}

void printWakeupReason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

void enterSleepMode(int seconds = DO_WORK_INTERVAL_SECONDS)
{
    esp_sleep_enable_timer_wakeup(seconds * uS_TO_S_FACTOR);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    display.setPowerSave(1);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // Lora off
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS off
    PrintRuntime();
    Serial.println("Going to sleep");
    Serial.flush();
    // SaveLMICToRTC(DO_WORK_INTERVAL_SECONDS);
    delay(10);
    esp_deep_sleep_start();
}


void enterSleepModeWithIMUWakeup()
{
    mpu.setAccWakeUp();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    display.setPowerSave(1);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF); // Lora off
    axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF); // GPS off
    PrintRuntime();
    Serial.println("Going to sleep");
    Serial.flush();
    // SaveLMICToRTC(DO_WORK_INTERVAL_SECONDS);
    delay(10);
    esp_deep_sleep_start();
}


int16_t getSnrTenfold()
{
    // Returns ten times the SNR (dB) value of the last received packet.
    // Ten times to prevent the use of float but keep 1 decimal digit accuracy.
    // Calculation per SX1276 datasheet rev.7 §6.4, SX1276 datasheet rev.4 §6.4.
    // LMIC.snr contains value of PacketSnr, which is 4 times the actual SNR value.
    return (LMIC.snr * 10) / 4;
}

int16_t getRssi(int8_t snr)
{
    // Returns correct RSSI (dBm) value of the last received packet.
    // Calculation per SX1276 datasheet rev.7 §5.5.5, SX1272 datasheet rev.4 §5.5.5.

#define RSSI_OFFSET 64
#define SX1276_FREQ_LF_MAX 525000000 // per datasheet 6.3
#define SX1272_RSSI_ADJUST -139
#define SX1276_RSSI_ADJUST_LF -164
#define SX1276_RSSI_ADJUST_HF -157

    int16_t rssi;

#ifdef MCCI_LMIC

    rssi = LMIC.rssi - RSSI_OFFSET;

#else
    int16_t rssiAdjust;
#ifdef CFG_sx1276_radio
    if (LMIC.freq > SX1276_FREQ_LF_MAX)
    {
        rssiAdjust = SX1276_RSSI_ADJUST_HF;
    }
    else
    {
        rssiAdjust = SX1276_RSSI_ADJUST_LF;
    }
#else
    // CFG_sx1272_radio
    rssiAdjust = SX1272_RSSI_ADJUST;
#endif

    // Revert modification (applied in lmic/radio.c) to get PacketRssi.
    int16_t packetRssi = LMIC.rssi + 125 - RSSI_OFFSET;
    if (snr < 0)
    {
        rssi = rssiAdjust + packetRssi + snr;
    }
    else
    {
        rssi = rssiAdjust + (16 * packetRssi) / 15;
    }
#endif

    return rssi;
}

void printEvent(ostime_t timestamp,
                const char *const message,
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true,
                bool eventLabel = false)
{
#ifdef USE_DISPLAY
    if (target == PrintTarget::All || target == PrintTarget::Display)
    {
        display.clearLine(TIME_ROW);
        display.setCursor(COL_0, TIME_ROW);
        display.print(F("Time:"));
        display.print(timestamp);
        display.clearLine(EVENT_ROW);
        if (clearDisplayStatusRow)
        {
            display.clearLine(STATUS_ROW);
        }
        display.setCursor(COL_0, EVENT_ROW);
        display.print(message);
    }
#endif

#ifdef USE_SERIAL
    // Create padded/indented output without using printf().
    // printf() is not default supported/enabled in each Arduino core.
    // Not using printf() will save memory for memory constrainted devices.
    String timeString(timestamp);
    uint8_t len = timeString.length();
    uint8_t zerosCount = TIMESTAMP_WIDTH > len ? TIMESTAMP_WIDTH - len : 0;

    if (target == PrintTarget::All || target == PrintTarget::Serial)
    {
        printChars(serial, '0', zerosCount);
        serial.print(timeString);
        serial.print(":  ");
        if (eventLabel)
        {
            serial.print(F("Event: "));
        }
        serial.println(message);
    }
#endif
}

void printEvent(ostime_t timestamp,
                ev_t ev,
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true)
{
#if defined(USE_DISPLAY) || defined(USE_SERIAL)
    printEvent(timestamp, lmicEventNames[ev], target, clearDisplayStatusRow, true);
#endif
}

void printFrameCounters(PrintTarget target = PrintTarget::All)
{
#ifdef USE_DISPLAY
    if (target == PrintTarget::Display || target == PrintTarget::All)
    {
        display.clearLine(FRMCNTRS_ROW);
        display.setCursor(COL_0, FRMCNTRS_ROW);
        display.print(F("Up:"));
        display.print(LMIC.seqnoUp);
        display.print(F(" Dn:"));
        display.print(LMIC.seqnoDn);
    }
#endif

#ifdef USE_SERIAL
    if (target == PrintTarget::Serial || target == PrintTarget::All)
    {
        printSpaces(serial, MESSAGE_INDENT);
        serial.print(F("Up: "));
        serial.print(LMIC.seqnoUp);
        serial.print(F(",  Down: "));
        serial.println(LMIC.seqnoDn);
    }
#endif
}

void printSessionKeys()
{
#if defined(USE_SERIAL) && defined(MCCI_LMIC)
    u4_t networkId = 0;
    devaddr_t deviceAddress = 0;
    u1_t networkSessionKey[16];
    u1_t applicationSessionKey[16];
    LMIC_getSessionKeys(&networkId, &deviceAddress,
                        networkSessionKey, applicationSessionKey);

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Network Id: "));
    serial.println(networkId, DEC);

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Device Address: "));
    serial.println(deviceAddress, HEX);

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Application Session Key: "));
    printHex(serial, applicationSessionKey, 16, true, '-');

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Network Session Key:     "));
    printHex(serial, networkSessionKey, 16, true, '-');
#endif
}

void printDownlinkInfo(void)
{
#if defined(USE_SERIAL) || defined(USE_DISPLAY)

    uint8_t dataLength = LMIC.dataLen;
    // bool ackReceived = LMIC.txrxFlags & TXRX_ACK;

    int16_t snrTenfold = getSnrTenfold();
    int8_t snr = snrTenfold / 10;
    int8_t snrDecimalFraction = snrTenfold % 10;
    int16_t rssi = getRssi(snr);

    uint8_t fPort = 0;
    if (LMIC.txrxFlags & TXRX_PORT)
    {
        fPort = LMIC.frame[LMIC.dataBeg - 1];
    }

#ifdef USE_DISPLAY
    display.clearLine(EVENT_ROW);
    display.setCursor(COL_0, EVENT_ROW);
    display.print(F("RX P:"));
    display.print(fPort);
    if (dataLength != 0)
    {
        display.print(" Len:");
        display.print(LMIC.dataLen);
    }
    display.clearLine(STATUS_ROW);
    display.setCursor(COL_0, STATUS_ROW);
    display.print(F("RSSI"));
    display.print(rssi);
    display.print(F(" SNR"));
    display.print(snr);
    display.print(".");
    display.print(snrDecimalFraction);
#endif

#ifdef USE_SERIAL
    printSpaces(serial, MESSAGE_INDENT);
    serial.println(F("Downlink received"));

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("RSSI: "));
    serial.print(rssi);
    serial.print(F(" dBm,  SNR: "));
    serial.print(snr);
    serial.print(".");
    serial.print(snrDecimalFraction);
    serial.println(F(" dB"));

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Port: "));
    serial.println(fPort);

    if (dataLength != 0)
    {
        printSpaces(serial, MESSAGE_INDENT);
        serial.print(F("Length: "));
        serial.println(LMIC.dataLen);
        printSpaces(serial, MESSAGE_INDENT);
        serial.print(F("Data: "));
        printHex(serial, LMIC.frame + LMIC.dataBeg, LMIC.dataLen, true, ' ');
    }
#endif
#endif
}

void printHeader(void)
{
#ifdef USE_DISPLAY
    display.clear();
    display.setCursor(COL_0, HEADER_ROW);
    // display.print(F("LMIC-node"));
    display.print("Boot count: " + String(bootCount));
#ifdef ABP_ACTIVATION
    display.drawString(ABPMODE_COL, HEADER_ROW, "ABP");
#endif
#ifdef CLASSIC_LMIC
    display.drawString(CLMICSYMBOL_COL, HEADER_ROW, "*");
#endif
    // display.drawString(COL_0, DEVICEID_ROW, deviceId);
    display.setCursor(COL_0, DEVICEID_ROW);
    display.print("Batt: " + String(axp.getBattVoltage() / 1000));
    display.setCursor(COL_0, INTERVAL_ROW);
    display.print(F("Interval:"));
    display.print(doWorkIntervalSeconds);
    display.print("s");
#endif

#ifdef USE_SERIAL
    serial.println(F("\n\nLMIC-node\n"));
    serial.print(F("Device-id:     "));
    serial.println(deviceId);
    serial.print(F("LMIC library:  "));
#ifdef MCCI_LMIC
    serial.println(F("MCCI"));
#else
    serial.println(F("Classic [Deprecated]"));
#endif
    serial.print(F("Activation:    "));
#ifdef OTAA_ACTIVATION
    serial.println(F("OTAA"));
#else
    serial.println(F("ABP"));
#endif
#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
    serial.print(F("LMIC debug:    "));
    serial.println(LMIC_DEBUG_LEVEL);
#endif
    serial.print(F("Interval:      "));
    serial.print(doWorkIntervalSeconds);
    serial.println(F(" seconds"));
    if (activationMode == ActivationMode::OTAA)
    {
        serial.println();
    }
#endif
}

#ifdef ABP_ACTIVATION
void setAbpParameters(dr_t dataRate = DefaultABPDataRate, s1_t txPower = DefaultABPTxPower)
{
// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power (note: txpow is possibly ignored by the library)
    LMIC_setDrTxpow(dataRate, txPower);
}
#endif // ABP_ACTIVATION

void initLmic(bit_t adrEnabled = 1,
              dr_t abpDataRate = DefaultABPDataRate,
              s1_t abpTxPower = DefaultABPTxPower)
{
    // ostime_t timestamp = os_getTime();

    // Initialize LMIC runtime environment
    os_init();
    // Reset MAC state
    LMIC_reset();

#ifdef ABP_ACTIVATION
    setAbpParameters(abpDataRate, abpTxPower);
#endif

    // Enable or disable ADR (data rate adaptation).
    // Should be turned off if the device is not stationary (mobile).
    // 1 is on, 0 is off.
    LMIC_setAdrMode(adrEnabled);

    if (activationMode == ActivationMode::OTAA)
    {
#if defined(CFG_us915) || defined(CFG_au915)
        // NA-US and AU channels 0-71 are configured automatically
        // but only one group of 8 should (a subband) should be active
        // TTN recommends the second sub band, 1 in a zero based count.
        // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
        LMIC_selectSubBand(1);
#endif
    }

// Relax LMIC timing if defined
#if defined(LMIC_CLOCK_ERROR_PPM)
    uint32_t clockError = 0;
#if LMIC_CLOCK_ERROR_PPM > 0
#if defined(MCCI_LMIC) && LMIC_CLOCK_ERROR_PPM > 4000
// Allow clock error percentage to be > 0.4%
#define LMIC_ENABLE_arbitrary_clock_error 1
#endif
    clockError = (LMIC_CLOCK_ERROR_PPM / 100) * (MAX_CLOCK_ERROR / 100) / 100;
    LMIC_setClockError(clockError);
#endif

#ifdef USE_SERIAL
    serial.print(F("Clock Error:   "));
    serial.print(LMIC_CLOCK_ERROR_PPM);
    serial.print(" ppm (");
    serial.print(clockError);
    serial.println(")");
#endif
#endif

#ifdef MCCI_LMIC
    // Register a custom eventhandler and don't use default onEvent() to enable
    // additional features (e.g. make EV_RXSTART available). User data pointer is omitted.
    LMIC_registerEventCb(&onLmicEvent, nullptr);
#endif
}

#ifdef MCCI_LMIC
void onLmicEvent(void *pUserData, ev_t ev)
#else
void onEvent(ev_t ev)
#endif
{
    // LMIC event handler
    ostime_t timestamp = os_getTime();

    switch (ev)
    {
#ifdef MCCI_LMIC
    // Only supported in MCCI LMIC library:
    case EV_RXSTART:
        // Do not print anything for this event or it will mess up timing.
        break;

    case EV_TXSTART:
        setTxIndicatorsOn();
        printEvent(timestamp, ev);
        break;

    case EV_JOIN_TXCOMPLETE:
    case EV_TXCANCELED:
        setTxIndicatorsOn(false);
        printEvent(timestamp, ev);
        break;
#endif
    case EV_JOINED:
        setTxIndicatorsOn(false);
        printEvent(timestamp, ev);
        printSessionKeys();

        // Disable link check validation.
        // Link check validation is automatically enabled
        // during join, but because slow data rates change
        // max TX size, it is not used in this example.
        LMIC_setLinkCheckMode(0);

        // The doWork job has probably run already (while
        // the node was still joining) and have rescheduled itself.
        // Cancel the next scheduled doWork job and re-schedule
        // for immediate execution to prevent that any uplink will
        // have to wait until the current doWork interval ends.
        os_clearCallback(&doWorkJob);
        os_setCallback(&doWorkJob, doWorkCallback);
        break;

    case EV_TXCOMPLETE:
        // Transmit completed, includes waiting for RX windows.
        setTxIndicatorsOn(false);
        printEvent(timestamp, ev);
        printFrameCounters();

        Check if downlink was received
        if (LMIC.dataLen != 0 || LMIC.dataBeg != 0)
        {
            uint8_t fPort = 0;
            if (LMIC.txrxFlags & TXRX_PORT)
            {
                fPort = LMIC.frame[LMIC.dataBeg - 1];
            }
            printDownlinkInfo();
            processDownlink(timestamp, fPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        }
        // LoraWANPrintLMICOpmode();
        // SaveLMICToRTC(DO_WORK_INTERVAL_SECONDS);
        // enterSleepMode();
        goToDeepSleep = true;
        break;

    // Below events are printed only.
    case EV_SCAN_TIMEOUT:
    case EV_BEACON_FOUND:
    case EV_BEACON_MISSED:
    case EV_BEACON_TRACKED:
    case EV_RFU1: // This event is defined but not used in code
    case EV_JOINING:
    case EV_JOIN_FAILED:
    case EV_REJOIN_FAILED:
    case EV_LOST_TSYNC:
    case EV_RESET:
    case EV_RXCOMPLETE:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
#ifdef MCCI_LMIC
    // Only supported in MCCI LMIC library:
    case EV_SCAN_FOUND: // This event is defined but not used in code
#endif
        printEvent(timestamp, ev);
        break;

    default:
        printEvent(timestamp, "Unknown Event");
        break;
    }
}

static void doWorkCallback(osjob_t *job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWork() which is called below.

    ostime_t timestamp = os_getTime();
#ifdef USE_SERIAL
    serial.println();
    printEvent(timestamp, "doWork job started", PrintTarget::Serial);
#endif

    // Do the work that needs to be performed.
    processWork(timestamp);

    // // This job must explicitly reschedule itself for the next run.
    // ostime_t startAt = timestamp + sec2osticks((int64_t)doWorkIntervalSeconds);
    // os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);
}

lmic_tx_error_t scheduleUplink(uint8_t fPort, uint8_t *data, uint8_t dataLength, bool confirmed = false)
{
    // This function is called from the processWork() function to schedule
    // transmission of an uplink message that was prepared by processWork().
    // Transmission will be performed at the next possible time

    ostime_t timestamp = os_getTime();
    printEvent(timestamp, "Packet queued");

    lmic_tx_error_t retval = LMIC_setTxData2(fPort, data, dataLength, confirmed ? 1 : 0);
    timestamp = os_getTime();

    if (retval == LMIC_ERROR_SUCCESS)
    {
#ifdef CLASSIC_LMIC
        // For MCCI_LMIC this will be handled in EV_TXSTART
        setTxIndicatorsOn();
#endif
    }
    else
    {
        String errmsg;
#ifdef USE_SERIAL
        errmsg = "LMIC Error: ";
#ifdef MCCI_LMIC
        errmsg.concat(lmicErrorNames[abs(retval)]);
#else
        errmsg.concat(retval);
#endif
        printEvent(timestamp, errmsg.c_str(), PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
        errmsg = "LMIC Err: ";
        errmsg.concat(retval);
        printEvent(timestamp, errmsg.c_str(), PrintTarget::Display);
#endif
    }
    return retval;
}

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

// static volatile uint16_t counter_ = 0;

// uint16_t getCounterValue()
// {
//     // Increments counter and returns the new value.
//     // delay(50); // Fake this takes some time
//     return ++counter_;
// }

// void resetCounter()
// {
//     // Reset counter to 0
//     counter_ = 0;
// }

void processWork(ostime_t doWorkJobTimeStamp)
{
    // This function is called from the doWorkCallback()
    // callback function when the doWork job is executed.

    // Uses globals: payloadBuffer and LMIC data structure.

    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWork if using OTAA and still joining.
    if (LMIC.devaddr != 0)
    {
        // Collect input data.
        // For simplicity LMIC-node uses a counter to simulate a sensor.
        // The counter is increased automatically by getCounterValue()
        // and can be reset with a 'reset counter' command downlink message.

        // uint16_t counterValue = getCounterValue();
        ostime_t timestamp = os_getTime();

#ifdef USE_DISPLAY
        // Interval and Counter values are combined on a single row.
        // This allows to keep the 3rd row empty which makes the
        // information better readable on the small display.
        display.clearLine(INTERVAL_ROW);
        display.setCursor(COL_0, INTERVAL_ROW);
        display.print("I:");
        display.print(doWorkIntervalSeconds);
        display.print("s");
        // display.print(" Ctr:");
        // display.print(counterValue);
#endif
#ifdef USE_SERIAL
        printEvent(timestamp, "Input data collected", PrintTarget::Serial);
        printSpaces(serial, MESSAGE_INDENT);
        // serial.print(F("COUNTER value: "));
        // serial.println(counterValue);
#endif

        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.

        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
// TxRx is currently pending, do not send.
#ifdef USE_SERIAL
            printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
            printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
#endif
        }
        else
        {
            bool isGpsFix = gps.checkGpsFix();
            // isGpsFix = true;
            Serial.println("Gps fix: " + String(isGpsFix));
            if (isGpsFix)
            {
                // Prepare upstream data transmission at the next possible time.
                gps.getLatLon(&lat, &lon, &alt, &kmph, &sats);
                Serial.println("lat: " + String(lat) + " lon: " + String(lon));
                // we have all the data that we need, let's construct LPP packet for Cayenne
                lpp.reset();
                lpp.addGPS(1, lat, lon, alt);
                lpp.addAnalogInput(2, kmph);
                lpp.addAnalogInput(3, sats);
                lpp.addAnalogInput(4, axp.getBattVoltage() / 1000);
                // lpp.addAnalogInput(8, alt_barometric);

                // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send to TTN
                scheduleUplink(1, lpp.getBuffer(), lpp.getSize());
                // Serial.print(lpp.getSize());
                // Serial.println(F(" bytes long LPP packet queued."));

                // display.setCursor(0, 0);
                // display.print("Fix: " + String(lat) + " " + String(lon));

                // DynamicJsonDocument jsonBuffer(1024);
                // CayenneLPP lpp(160);
                // JsonObject root = jsonBuffer.to<JsonObject>();
                // lpp.decodeTTN(lpp.getBuffer(), lpp.getSize(), root);
                // serializeJsonPretty(root, Serial);
                // Serial.println();

                // digitalWrite(BUILTIN_LED, HIGH);
                // Serial.println("Success");
            }
            else
            {
                mpu.update();
                Serial.println("Failed " + String(retryCount) + " " + String(mpu.getAccAngleX()));
                os_setTimedCallback(&doWorkJob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), doWorkCallback);
                // try again in a few 'GPS_FIX_RETRY_DELAY' seconds...
                // if (retryCount > 3)
                // {
                //     goToDeepSleepWithIMUWakeup = true;
                //     // retryCount = 0;
                // }
                // else
                // {
                //     os_setTimedCallback(&doWorkJob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), doWorkCallback);
                //     retryCount++;
                // }
                
                // os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);
                // timestamp + sec2osticks((int64_t)doWorkIntervalSeconds);
            }
        }
    }
}

// void processDownlink(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t *data, uint8_t dataLength)
// {
//     // This function is called from the onEvent() event handler
//     // on EV_TXCOMPLETE when a downlink message was received.

//     // Implements a 'reset counter' command that can be sent via a downlink message.
//     // To send the reset counter command to the node, send a downlink message
//     // (e.g. from the TTN Console) with single byte value resetCmd on port cmdPort.

//     const uint8_t cmdPort = 100;
//     const uint8_t resetCmd = 0xC0;

//     if (fPort == cmdPort && dataLength == 1 && data[0] == resetCmd)
//     {
// #ifdef USE_SERIAL
//         printSpaces(serial, MESSAGE_INDENT);
//         serial.println(F("Reset cmd received"));
// #endif
//         ostime_t timestamp = os_getTime();
//         resetCounter();
//         printEvent(timestamp, "Counter reset", PrintTarget::All, false);
//     }
// }

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀

void setup()
{
    // boardInit(InitType::Hardware) must be called at start of setup() before anything else.
    bool hardwareInitSucceeded = boardInit(InitType::Hardware);

#ifdef USE_DISPLAY
    initDisplay();
#endif

#ifdef USE_SERIAL
    initSerial(MONITOR_SPEED, WAITFOR_SERIAL_S);
#endif

    boardInit(InitType::PostInitSerial);

#if defined(USE_SERIAL) || defined(USE_DISPLAY)
    printHeader();
#endif

    if (!hardwareInitSucceeded)
    {
#ifdef USE_SERIAL
        serial.println(F("Error: hardware init failed."));
        serial.flush();
#endif
#ifdef USE_DISPLAY
        // Following mesage shown only if failure was unrelated to I2C.
        display.setCursor(COL_0, FRMCNTRS_ROW);
        display.print(F("HW init failed"));
#endif
        abort();
    }

    initLmic();

    //  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
    //  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
    //  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

    // Place code for initializing sensors etc. here.
    ++bootCount;
    // resetCounter();
    printWakeupReason();
    mpu.begin();
    // Wire.begin(21, 22);
    // WiFi.mode(WIFI_OFF);
    // btStop();
    gps.init();
    pinMode(EXT_BUTTON_PIN, INPUT);
    pinMode(IMU_INT_PIN, INPUT);
    if (RTC_LMIC.seqnoUp != 0)
    {
        LoadLMICFromRTC();
    }
    LoraWANDebug(LMIC);

    //  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
    //  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
    //  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀

    if (activationMode == ActivationMode::OTAA)
    {
        LMIC_startJoining();
    }

    // Schedule initial doWork job for immediate execution.
    os_setCallback(&doWorkJob, doWorkCallback);
}

void loop()
{
    os_runloop_once();

    // Serial.println("Pin status" + String(digitalRead(EXT_BUTTON_PIN)));
    // if (digitalRead(EXT_BUTTON_PIN) == 0)
    // {
    //     // getAxpStatus();
    //     mpu.update();
    //     Serial.println(abs(mpu.getGyroX()) + abs(mpu.getGyroY()) + abs(mpu.getGyroY()));
    //     // mpu.setAccWakeUp();
    //     esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
    //     esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); // all RTC Peripherals are powered //deep-sleep
    //     Serial.println("Sleep");
    //     Serial.flush();
    //     display.setPowerSave(1);
    //     axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    //     axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    //     delay(10);
    //     esp_deep_sleep_start();
    // }

    static unsigned long lastPrintTime = 0;
    const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((DO_WORK_INTERVAL_SECONDS * 1000)));
    if (!timeCriticalJobs && goToDeepSleep == true && !(LMIC.opmode & OP_TXRXPEND))
    {
        Serial.print(F("Can go sleep "));
        LoraWANPrintLMICOpmode();
        SaveLMICToRTC(DO_WORK_INTERVAL_SECONDS);
        enterSleepMode();
    }
    else if(!timeCriticalJobs && goToDeepSleepWithIMUWakeup == true && !(LMIC.opmode & OP_TXRXPEND))
    {
        Serial.print(F("Can go sleep(IMU) "));
        LoraWANPrintLMICOpmode();
        SaveLMICToRTC(DO_WORK_INTERVAL_SECONDS);
        enterSleepModeWithIMUWakeup();
    }
    else if (lastPrintTime + 2000 < millis())
    {
        Serial.print(F("Cannot sleep "));
        Serial.print(F("TimeCriticalJobs: "));
        Serial.print(timeCriticalJobs);
        Serial.print(" ");
        LoraWANPrintLMICOpmode();
        PrintRuntime();
        lastPrintTime = millis();
    }
}

#endif

// -------------------------------------------------------------------------------

#ifdef DEV
// #include <TinyGPSPlus.h>
// /*
//    This sample sketch should be the first you try out when you are testing a TinyGPSPlus
//    (TinyGPSPlus) installation.  In normal use, you feed TinyGPSPlus objects characters from
//    a serial NMEA GPS device, but this example uses static strings for simplicity.
// */

// // A sample NMEA stream.
// const char *gpsStream =
//   "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
//   "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
//   "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
//   "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
//   "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
//   "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// // The TinyGPSPlus object
// TinyGPSPlus gps;

// void displayInfo()
// {
//   Serial.print(F("Location: "));
//   if (gps.location.isValid())
//   {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
// }

// void setup()
// {
//   Serial.begin(115200);

//   Serial.println(F("BasicExample.ino"));
//   Serial.println(F("Basic demonstration of TinyGPSPlus (no device needed)"));
//   Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
//   Serial.println(F("by Mikal Hart"));
//   Serial.println();

// //   while (*gpsStream)
//     // if (gps.encode(*gpsStream++))
//       displayInfo();

//   Serial.println();
//   Serial.println(F("Done."));
// }

// void loop()
// {
//     Serial.println(F("BasicExample.ino"));
//   Serial.println(F("Basic demonstration of TinyGPSPlus (no device needed)"));
//   Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
//   Serial.println(F("by Mikal Hart"));
//   Serial.println();

//   while (*gpsStream)
//     if (gps.encode(*gpsStream++))
//       displayInfo();

//   Serial.println();
//   Serial.println(F("Done."));
//   delay(5000);
// }

// #include <TinyGPSPlus.h>
// // #include <SoftwareSerial.h>
// /*
//    This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
//    It requires the use of SoftwareSerial, and assumes that you have a
//    4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
// */
// static const int RXPin = 34, TXPin = 12;
// static const uint32_t GPSBaud = 9600;

// // The TinyGPSPlus object
// TinyGPSPlus gps;

// HardwareSerial GPSSerial(1);

// // HardwareSerial GPSSerial(1);

// //   GPSSerial.begin(9600, SERIAL_8N1, TXPin, RXPin);
// //   GPSSerial.setTimeout(2);

// // The serial connection to the GPS device
// // HardwareSerial ss(RXPin, TXPin);
// // HardwareSerial& serial = Serial;

// void displayInfo()
// {
//   Serial.print(F("Location: "));
//   if (gps.location.isValid())
//   {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid())
//   {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid())
//   {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   }
//   else
//   {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
// }

// void setup()
// {
//   Serial.begin(115200);
//     GPSSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
//   GPSSerial.setTimeout(2);

//   Serial.println(F("DeviceExample.ino"));
//   Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
//   Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
//   Serial.println(F("by Mikal Hart"));
//   Serial.println();
// }

// void loop()
// {
//   // This sketch displays information every time a new sentence is correctly encoded.
//   while (GPSSerial.available() > 0)
//     if (gps.encode(GPSSerial.read()))
//       displayInfo();

//   if (millis() > 5000 && gps.charsProcessed() < 10)
//   {
//     Serial.println(F("No GPS detected: check wiring."));
//     while(true);
//   }
// }

// #include <TinyGPS++.h>

// TinyGPSPlus gps;
// HardwareSerial Serial1(1);

// static void smartDelay(unsigned long ms)
// {
//   unsigned long start = millis();
//   do
//   {
//     while (Serial1.available())
//       gps.encode(Serial1.read());
//   } while (millis() - start < ms);
// }

// void setup()
// {
//   Serial.begin(115200);
//   Serial1.begin(9600, SERIAL_8N1, 12, 34);   //17-TX 18-RX
// }

// void loop()
// {
//   Serial.print("Latitude  : ");
//   Serial.println(gps.location.lat(), 5);
//   Serial.print("Longitude : ");
//   Serial.println(gps.location.lng(), 4);
//   Serial.print("Satellites: ");
//   Serial.println(gps.satellites.value());
//   Serial.print("Altitude  : ");
//   Serial.print(gps.altitude.feet() / 3.2808);
//   Serial.println("M");
//   Serial.print("Time      : ");
//   Serial.print(gps.time.hour());
//   Serial.print(":");
//   Serial.print(gps.time.minute());
//   Serial.print(":");
//   Serial.println(gps.time.second());
//   Serial.println("**********************");

//   smartDelay(1000);

//   if (millis() > 5000 && gps.charsProcessed() < 10)
//     Serial.println(F("No GPS data received: check wiring"));
// }

// #define SDA_PIN 21
// #define SCL_PIN 22
// #define AXP_IRQ_PIN 35
// #define GPS_RX_PIN 34
// #define GPS_TX_PIN 12

// AXP20X_Class axp;
// SFE_UBLOX_GPS gps;
// // HardwareSerial Serial1(1);
// // TinyGPSPlus gps;

// void setup()
// {
//     Serial.begin(115200);
//     while(!Serial);
//     Wire.begin(SDA_PIN, SCL_PIN);
//     if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
//         Serial.println("AXP Power begin failed");
//         while (1);
//     }
//     axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LoRa
//     axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS
//     Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

//     if (gps.begin(Serial1)) {
//            Serial.println("Connected to GPS");
//            gps.setUART1Output(COM_TYPE_NMEA);
//            Serial.println("GPS serial connected, output set to NMEA");
//            gps.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
//            gps.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
//            gps.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
//            gps.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
//            gps.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
//            gps.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
//            gps.saveConfiguration();
//         //    gps.factoryReset();
//     }
// }

// void loop()
// {
//     if (Serial1.available())
//     {
//         Serial.write(Serial1.read()); // print anything comes in from the GPS
//     }
// }

#include <TinyGPS++.h>
#include "gps.h"
#include <U8x8lib.h>
#include "LMIC-node.h"
#include <CayenneLPP.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define AXP_IRQ_PIN 35
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12

// AXP20X_Class axp;
// TinyGPSPlus gps;
CayenneLPP lpp(51);
GPS gps;

// A sample NMEA stream.
const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

char t[32];
double lat, lon, alt, kmph;
int sats; // GPS satellite count

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    // display.begin();
    initDisplay();
    boardInit(InitType::Hardware);
    display.clear();
    display.setCursor(0, 0);
    display.print(F("LMIC-node"));
}

void loop()
{
    // This sketch displays information every time a new sentence is correctly encoded.
    //   while (Serial1.available() > 0)
    //     if (gps.encode(Serial1.read()))
    //       displayInfo();

    //   if (millis() > 5000 && gps.charsProcessed() < 10)
    //   {
    //     Serial.println(F("No GPS detected: check wiring."));
    //     while(true);
    //   }

    // while (*gpsStream)
    //     if (gps.encode(*gpsStream++))
    //         displayInfo();

    delay(1000);

    // if (gps.checkGpsFix())
    // {
    gps.encodeDebug();
    // Prepare upstream data transmission at the next possible time.
    gps.getLatLon(&lat, &lon, &alt, &kmph, &sats);
    // Serial.println("lat: " + String(lat) + " lon: " + String(lon));
    // we have all the data that we need, let's construct LPP packet for Cayenne
    lpp.reset();
    lpp.addGPS(1, lat, lon, alt);
    lpp.addAnalogInput(2, kmph);
    lpp.addGenericSensor(3, sats);
    lpp.addAnalogInput(4, axp.getBattVoltage() / 1000);
    // lpp.addGenericSensor

    Serial.print(lpp.getSize());
    // Serial.println(lpp.getBuffer());
    Serial.println(F(" bytes long LPP packet queued."));
    // digitalWrite(BUILTIN_LED, HIGH);
    // display.clear();
    display.setCursor(0, 0);
    display.print("Fix:" + String(lat) + " " + String(lon));

    DynamicJsonDocument jsonBuffer(1024);
    // CayenneLPP lpp(160);
    JsonObject root = jsonBuffer.to<JsonObject>();
    lpp.decodeTTN(lpp.getBuffer(), lpp.getSize(), root);
    serializeJsonPretty(root, Serial);
    Serial.println();

    // }
    // else
    // {
    //     // display.clear();
    //     display.setCursor(0, 20);
    //     display.print("No Fix          ");
    // }
    delay(1000);
    // Serial.println("Bat%: " + String(axp.getBattPercentage()));
    Serial.println("Bat voltage: " + String(axp.getBattVoltage()));
    display.setCursor(0, 30);
    display.print(axp.getBattVoltage());
}

#endif