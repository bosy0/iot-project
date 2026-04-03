/*
 * M5Stack Air Monitor
 *
 * Capteurs : PMS5003 (particules), BME680 (VOC), ENV III SHT3X (temp/humi),
 *            QMP6988 (pression/altitude), GPS
 * Bouton A  : allumer / éteindre l'écran
 * Bouton B  : page précédente
 * Bouton C  : page suivante
 */

#include <M5Stack.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include <PMS.h>
#include <TinyGPS++.h>
#include "SHT3X.h"
#include "QMP6988.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_SGP30.h>

// ─── Configuration réseau ─────────────────────────────────────────────────────
#define WIFI_SSID  "Pixel 8a Marius"
#define WIFI_PASS  "Mariusdup"
#define MQTT_HOST  "632e64128d12456aa5bb05d63a8d5f49.s1.eu.hivemq.cloud"
#define MQTT_PORT  8883
#define MQTT_USER  "iot_project"
#define MQTT_PASS  "7%tWH4bUoN!Y3#gg7"
#define DEVICE_ID  "m5stack_1"

// ─── Couleurs ─────────────────────────────────────────────────────────────────
#define CLR_BG     0x0000   // Noir
#define CLR_BAR    0x1082   // Gris très sombre (barres haut/bas)
#define CLR_SEP    0x3186   // Gris séparateurs
#define CLR_LABEL  0x8C71   // Gris clair (labels)
#define CLR_GOOD   0x07E0   // Vert
#define CLR_OK     0xFFE0   // Jaune
#define CLR_BAD    0xF800   // Rouge
#define CLR_CYAN   0x07FF   // Cyan (titres de section)

// ─── Dimensions de l'écran (320x240) ─────────────────────────────────────────
#define BAR_H   28          // Hauteur barre de statut (haut)
#define BTN_Y   216         // Y de la barre boutons (bas)
#define CONT_Y  (BAR_H + 1) // Début zone contenu (y=29)

// ─── Objets matériel ─────────────────────────────────────────────────────────
HardwareSerial SerialPMS(1);    // PMS5003 : UART1 RX=36 TX=26
HardwareSerial SerialGPS(2);    // GPS     : UART2 RX=16 TX=17

PMS            pms(SerialPMS);
PMS::DATA      pmsData;
TinyGPSPlus    gps;
Adafruit_BME680 bme;
SHT3X          sht;
QMP6988        qmp;
Adafruit_SGP30 sgp;

WiFiClientSecure wifiClient;
PubSubClient     mqtt(wifiClient);

// ─── Données capteurs ─────────────────────────────────────────────────────────
struct {
    uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
    float    temp = 0, humi = 0, pres = 0, alt = 0, voc = 0;
    double   lat = 0, lng = 0, gpsAlt = 0;
    int      sats = 0;
    bool     gpsValid = false;
    int      battery  = 0;
    bool     charging = false;
    uint16_t tvoc = 0, eco2 = 0;
    bool     sgpOk = false;
} d;

// ─── État UI ─────────────────────────────────────────────────────────────────
int  page     = 0;
bool screenOn = true;

// ─── Timers (ms) ──────────────────────────────────────────────────────────────
uint32_t tPms = 0, tBme = 0, tBat = 0, tMqtt = 0, tRecon = 0, tDisplay = 0;

// ─── Prototypes ───────────────────────────────────────────────────────────────
void drawStatusBar();
void drawBtnBar();
void drawContent();
void drawPageAir();
void drawPageEnv();
void drawPageGPS();
void drawPageNet();
void drawPageSGP();
void setScreen(bool on);
uint16_t qColor(float v, float good, float ok);
void drawProgressBar(int x, int y, int w, int h, float ratio, uint16_t color);

// ─────────────────────────────────────────────────────────────────────────────
// Constantes gestion connexion MQTT
uint32_t mqttRetryDelay = 2000; // Délai initial 2s
const uint32_t maxRetryDelay = 60000;

// ─── Paramètres Carte SD ─────────────────────────────────────────────────────────
bool sdAvailable = false;

void setup() {
    M5.begin();
    M5.Power.begin();

    // Silence total du haut-parleur (évite les bruits parasites DAC)
    dacWrite(25, 0);
    M5.Speaker.mute();

    M5.Lcd.fillScreen(CLR_BG);
    M5.Lcd.setBrightness(150);

    // Initialisation de la carte SD
    if (SD.begin()) {
        sdAvailable = true;
        Serial.println("Carte SD initialisée.");
    } else {
        Serial.println("Échec de l'initialisation de la carte SD.");
    }

    // Écran de démarrage
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("AIR MONITOR", 160, 100, 4);
    M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
    M5.Lcd.drawString("Initialisation...", 160, 135, 2);
    M5.Lcd.setTextDatum(TL_DATUM);

    // Sériaux capteurs
    SerialPMS.begin(9600, SERIAL_8N1, 36, 26);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
    pms.passiveMode();
    pms.wakeUp();

    // I2C et capteurs
    Wire.begin(21, 22);
    sht.begin(&Wire, SHT3X_I2C_ADDR,        21, 22);
    qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, 21, 22);

    if (sgp.begin()) {
        d.sgpOk = true;
        Serial.println("SGP30 detecte");
    } else {
        Serial.println("SGP30 non detecte");
    }

    if (!bme.begin(0x77) && !bme.begin(0x76)) {
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.setTextColor(CLR_BAD, CLR_BG);
        M5.Lcd.drawString("BME680 non detecte", 160, 160, 2);
        M5.Lcd.setTextDatum(TL_DATUM);
        delay(1500);
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);

    // WiFi & MQTT
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifiClient.setInsecure(); // TLS sans vérif. certificat (démo)
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setKeepAlive(30);

    // Batterie initiale
    d.battery  = M5.Power.getBatteryLevel();
    d.charging = M5.Power.isCharging();

    // Affichage principal
    M5.Lcd.fillScreen(CLR_BG);
    drawStatusBar();
    drawBtnBar();
    drawContent();
}

void loop() {
    M5.update();
    dacWrite(25, 0); // maintient le silence du haut-parleur

    uint32_t now = millis();

    // ── Boutons ───────────────────────────────────────────────────────────
    if (M5.BtnA.wasPressed()) {
        setScreen(!screenOn);
        return;
    }
    if (screenOn) {
        if (M5.BtnB.wasPressed()) {
            page = (page + 4) % 5; // bouton B // page précédente
            drawBtnBar();
            drawContent();
        }
        if (M5.BtnC.wasPressed()) {
            page = (page + 1) % 5; // bouton C // page suivante
            drawBtnBar();
            drawContent();
        }
    }

    // ── Lecture PMS5003 — toutes les 2 secondes ───────────────────────────
    if (now - tPms >= 2000) {
        tPms = now;
        pms.requestRead();
    }
    if (pms.read(pmsData)) {
        d.pm1  = pmsData.PM_AE_UG_1_0;
        d.pm25 = pmsData.PM_AE_UG_2_5;
        d.pm10 = pmsData.PM_AE_UG_10_0;
    }

    // ── Lecture BME680 + ENV III — toutes les 3 secondes ─────────────────
    if (now - tBme >= 3000) {
        tBme = now;
        if (bme.performReading())
            d.voc = bme.gas_resistance / 1000.0f;
        if (sht.update()) {
            d.temp = sht.cTemp;
            d.humi = sht.humidity;
        }
        if (qmp.update()) {
            d.pres = qmp.pressure / 100.0f; // Pa → hPa
            d.alt  = qmp.altitude;
        }
    }

    if (d.sgpOk && sgp.IAQmeasure()) {
        d.tvoc = sgp.TVOC;
        d.eco2 = sgp.eCO2;
    }

    // ── Lecture GPS — continue ────────────────────────────────────────────
    while (SerialGPS.available()) gps.encode(SerialGPS.read());
    if (gps.location.isValid()) {
        d.lat = gps.location.lat();
        d.lng = gps.location.lng();
        d.gpsValid = true;
    }
    if (gps.satellites.isValid()) d.sats   = gps.satellites.value();
    if (gps.altitude.isValid())   d.gpsAlt = gps.altitude.meters();

    // ── Lecture batterie — toutes les 30 secondes ─────────────────────────
    if (now - tBat >= 30000) {
        tBat       = now;
        d.battery  = M5.Power.getBatteryLevel();
        d.charging = M5.Power.isCharging();
    }

    // ── Mise à jour affichage — toutes les 2 secondes ─────────────────────
    if (screenOn && now - tDisplay >= 2000) {
        tDisplay = now;
        drawStatusBar();
        drawContent();
    }

    // ── MQTT ──────────────────────────────────────────────────────────────
    if (WiFi.isConnected()) {

       if (!mqtt.connected()) {
            // On ne tente la reco que si le délai est passé
            if (now - tRecon >= mqttRetryDelay) {
                tRecon = now;
                Serial.println("Tentative de connexion MQTT...");
                
                if (mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
                    Serial.println("Connecté !");
                    mqttRetryDelay = 2000; // Reset du délai en cas de succès
                } else {
                    Serial.print("Échec, code erreur : ");
                    Serial.println(mqtt.state());
                    // Augmentation exponentielle du délai (x2)
                    mqttRetryDelay = min(mqttRetryDelay * 2, maxRetryDelay);
                }
            }
        } else {
            mqtt.loop(); // Traitement des messages entrant/sortant
        }
    } else {
        // Si le WiFi est perdu, on reset le timer de reco MQTT
        tRecon = now;
    }
 
    if (now - tMqtt >= 10000) {
        tMqtt = now;

        // 1. Construction du topic et du JSON
        char buf[600], topic[48];
        snprintf(topic, sizeof(topic), "sensors/%s/data", DEVICE_ID);
        snprintf(buf, sizeof(buf),
            "{\"pm1\":%d,\"pm25\":%d,\"pm10\":%d,"
            "\"voc\":%.1f,\"temp\":%.1f,\"humi\":%.1f,"
            "\"pres\":%.1f,\"alt\":%.1f,"
            "\"tvoc\":%d,\"eco2\":%d,"
            "\"lat\":%.6f,\"lng\":%.6f,\"gps_alt\":%.1f,\"sats\":%d,"
            "\"battery\":%d,\"charging\":%s}",
            d.pm1, d.pm25, d.pm10,
            d.voc, d.temp, d.humi, d.pres, d.alt,
            d.tvoc, d.eco2,
            d.gpsValid ? d.lat : 0.0,
            d.gpsValid ? d.lng : 0.0,
            d.gpsAlt, d.sats,
            d.battery,
            d.charging ? "true" : "false"
        );

        if (mqtt.connected()) {
            // 2. Vider le buffer SD si des données sont en attente
            if (sdAvailable && SD.exists("/buffer.txt")) {
                File file = SD.open("/buffer.txt", FILE_READ);
                if (file) {
                    while (file.available()) {
                        String line = file.readStringUntil('\n');
                        line.trim();
                        if (line.length() > 0) {
                            mqtt.publish(topic, line.c_str(), true);
                            delay(50);
                        }
                    }
                    file.close();
                    SD.remove("/buffer.txt");
                    Serial.println("Buffer SD vidé.");
                }
            }
            // 3. Envoyer la mesure live
            mqtt.publish(topic, buf, true);
            Serial.println("Donnée envoyée via MQTT.");

        } else if (sdAvailable) {
            // 4. Pas de MQTT → stocker sur SD si assez de place
            uint64_t freeBytes = SD.totalBytes() - SD.usedBytes();
            if (freeBytes < 10000) {
                Serial.println("Carte SD pleine ! Donnée perdue.");
            } else {
                File file = SD.open("/buffer.txt", FILE_APPEND);
                if (file) {
                    file.println(buf);
                    file.close();
                    Serial.println("Donnée mise en buffer SD.");
                } else {
                    Serial.println("Erreur ouverture buffer.txt !");
                }
            }
        }

        drawStatusBar();
    }
}    

// ─── Écran on/off ────────────────────────────────────────────────────────────
void setScreen(bool on) {
    screenOn = on;
    if (on) {
        M5.Lcd.wakeup();
        M5.Lcd.setBrightness(150);
        M5.Lcd.fillScreen(CLR_BG);
        drawStatusBar();
        drawBtnBar();
        drawContent();
    } else {
        M5.Lcd.setBrightness(0);
        M5.Lcd.sleep();
    }
}

// ─── Retourne une couleur selon des seuils qualité ────────────────────────────
uint16_t qColor(float v, float good, float ok) {
    if (v <= good) return CLR_GOOD;
    if (v <= ok)   return CLR_OK;
    return CLR_BAD;
}

// ─── Barre de progression colorée ────────────────────────────────────────────
void drawProgressBar(int x, int y, int w, int h, float ratio, uint16_t color) {
    int filled = (int)(w * constrain(ratio, 0.0f, 1.0f));
    M5.Lcd.drawRect(x, y, w, h, CLR_LABEL);
    if (filled > 2)
        M5.Lcd.fillRect(x + 1, y + 1, filled - 2, h - 2, color);
    M5.Lcd.fillRect(x + 1 + max(0, filled - 2), y + 1,
                    w - 2 - max(0, filled - 2), h - 2, CLR_BG);
}

// ─── Barre de statut (haut, 28px) ─────────────────────────────────────────────
void drawStatusBar() {
    M5.Lcd.fillRect(0, 0, 320, BAR_H, CLR_BAR);

    // Pastille de connexion (vert=MQTT, jaune=WiFi seul, rouge=hors ligne)
    uint16_t sc = mqtt.connected()    ? CLR_GOOD :
                  WiFi.isConnected()  ? CLR_OK   : CLR_BAD;
    M5.Lcd.fillCircle(14, BAR_H / 2, 6, sc);

    uint16_t sdColor = CLR_SEP; // Gris par défaut
    if (sdAvailable) {
        if (mqtt.connected()) {
            sdColor = CLR_CYAN;
        } else {
            // Clignote toutes les 500ms si on écrit sur SD
            sdColor = (millis() % 1000 < 500) ? CLR_BAD : CLR_BG;
        }
    }

    // Titre centré
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextColor(WHITE, CLR_BAR);
    M5.Lcd.drawString("AIR MONITOR", 160, BAR_H / 2, 2);

    // Batterie (droite)
    char batBuf[12];
    snprintf(batBuf, sizeof(batBuf), d.charging ? "~%d%%" : "%d%%", d.battery);
    M5.Lcd.setTextColor(d.battery > 20 ? CLR_GOOD : CLR_BAD, CLR_BAR);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(batBuf, 314, BAR_H / 2, 2);

    M5.Lcd.drawFastHLine(0, BAR_H, 320, CLR_SEP);
    M5.Lcd.setTextDatum(TL_DATUM);
}

// ─── Barre des boutons (bas, 24px) ────────────────────────────────────────────
void drawBtnBar() {
    M5.Lcd.fillRect(0, BTN_Y, 320, 240 - BTN_Y, CLR_BAR);
    M5.Lcd.drawFastHLine(0, BTN_Y, 320, CLR_SEP);

    const char* names[] = {"Air", "Env", "GPS", "Net", "SGP"};

    // Bouton A : écran
    M5.Lcd.setTextColor(CLR_LABEL, CLR_BAR);
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.drawString("[Ecran]", 6, BTN_Y + 12, 1);

    // Indicateur de page (centre)
    char pageBuf[16];
    snprintf(pageBuf, sizeof(pageBuf), "< %s >", names[page]);
    M5.Lcd.setTextColor(WHITE, CLR_BAR);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString(pageBuf, 160, BTN_Y + 12, 2);

    M5.Lcd.setTextDatum(TL_DATUM);
}

// ─── Routage des pages ────────────────────────────────────────────────────────
void drawContent() {
    // Effacer la zone contenu
    M5.Lcd.fillRect(0, CONT_Y, 320, BTN_Y - CONT_Y, CLR_BG);

    switch (page) {
        case 0: drawPageAir(); break;
        case 1: drawPageEnv(); break;
        case 2: drawPageGPS(); break;
        case 3: drawPageNet(); break;
        case 4: drawPageSGP(); break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Page 0 — Qualité de l'air (PM1.0 / PM2.5 / PM10 + VOC)
// ─────────────────────────────────────────────────────────────────────────────
void drawPageAir() {
    // Titre "PARTICULES"
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("PARTICULES", 160, CONT_Y + 4, 2);

    // 3 colonnes PM (centres à x=53, 160, 267)
    struct { const char* lbl; uint16_t val; float good; float ok; } pm[3] = {
        {"PM 1.0", d.pm1,   5,  15},
        {"PM 2.5", d.pm25, 10,  25},
        {"PM 10",  d.pm10, 20,  50},
    };
    const int col_y = CONT_Y + 26;
    const int cx[3] = {53, 160, 267};

    for (int i = 0; i < 3; i++) {
        uint16_t c = qColor((float)pm[i].val, pm[i].good, pm[i].ok);

        M5.Lcd.setTextDatum(TC_DATUM);
        M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
        M5.Lcd.drawString(pm[i].lbl, cx[i], col_y, 2);

        M5.Lcd.setTextColor(c, CLR_BG);
        M5.Lcd.drawNumber(pm[i].val, cx[i], col_y + 18, 4);

        M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
        M5.Lcd.drawString("ug/m3", cx[i], col_y + 48, 1);
    }

    // Séparateur
    const int sep_y = CONT_Y + 88;
    M5.Lcd.drawFastHLine(10, sep_y, 300, CLR_SEP);

    // Titre "QUALITE AIR (VOC)"
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("QUALITE AIR (VOC)", 160, sep_y + 8, 2);

    // Couleur et label VOC
    uint16_t vc;
    const char* vl;
    if      (d.voc > 300) { vc = CLR_GOOD; vl = "EXCELLENT"; }
    else if (d.voc > 150) { vc = CLR_GOOD; vl = "BON";       }
    else if (d.voc >  50) { vc = CLR_OK;   vl = "MOYEN";     }
    else                  { vc = CLR_BAD;  vl = "MAUVAIS";   }

    const int voc_y = sep_y + 30;
    char vocBuf[20];
    snprintf(vocBuf, sizeof(vocBuf), "%.0f kOhm", d.voc);

    M5.Lcd.setTextColor(vc, CLR_BG);
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.drawString(vocBuf, 10, voc_y, 2);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(vl, 310, voc_y, 2);

    // Barre de progression VOC (0–600 kOhm)
    drawProgressBar(10, voc_y + 16, 300, 14, d.voc / 600.0f, vc);

    M5.Lcd.setTextDatum(TL_DATUM);
}

// ─────────────────────────────────────────────────────────────────────────────
// Page 1 — Environnement (température, humidité, pression, altitude)
// ─────────────────────────────────────────────────────────────────────────────
void drawPageEnv() {
    // Titre
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("ENVIRONNEMENT", 160, CONT_Y + 4, 2);
    M5.Lcd.setTextDatum(TL_DATUM);

    // 4 lignes, chacune haute de 38px, centre vertical à row_y + 19
    struct { const char* label; char value[20]; uint16_t color; } rows[4];

    snprintf(rows[0].value, 20, "%.1f C",   d.temp);
    rows[0].label = "Temperature";
    rows[0].color = qColor(d.temp, 28.0f, 35.0f);

    snprintf(rows[1].value, 20, "%.1f %%",  d.humi);
    rows[1].label = "Humidite";
    rows[1].color = qColor(d.humi, 60.0f, 80.0f);

    snprintf(rows[2].value, 20, "%.0f hPa", d.pres);
    rows[2].label = "Pression";
    rows[2].color = WHITE;

    snprintf(rows[3].value, 20, "%.0f m",   d.alt);
    rows[3].label = "Altitude";
    rows[3].color = WHITE;

    int row_y = CONT_Y + 28;
    for (int i = 0; i < 4; i++) {
        int mid = row_y + 19;

        M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
        M5.Lcd.setTextDatum(ML_DATUM);
        M5.Lcd.drawString(rows[i].label, 14, mid, 2);

        M5.Lcd.setTextColor(rows[i].color, CLR_BG);
        M5.Lcd.setTextDatum(MR_DATUM);
        M5.Lcd.drawString(rows[i].value, 310, mid, 4);

        if (i < 3)
            M5.Lcd.drawFastHLine(10, row_y + 38, 300, CLR_SEP);

        row_y += 38;
    }
    M5.Lcd.setTextDatum(TL_DATUM);
}

// ─────────────────────────────────────────────────────────────────────────────
// Page 2 — Localisation GPS
// ─────────────────────────────────────────────────────────────────────────────
void drawPageGPS() {
    // Titre
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("LOCALISATION GPS", 160, CONT_Y + 4, 2);

    if (!d.gpsValid) {
        // Attente de fix
        M5.Lcd.setTextColor(CLR_OK, CLR_BG);
        M5.Lcd.drawString("Acquisition satellite...", 160, CONT_Y + 80, 2);

        char satBuf[32];
        snprintf(satBuf, sizeof(satBuf), "%d satellite(s) detecte(s)", d.sats);
        M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
        M5.Lcd.drawString(satBuf, 160, CONT_Y + 105, 2);

        M5.Lcd.setTextDatum(TL_DATUM);
        return;
    }

    M5.Lcd.setTextDatum(TL_DATUM);

    char latBuf[20], lngBuf[20], satBuf[16], altBuf[16];
    snprintf(latBuf, sizeof(latBuf), "%.5f", d.lat);
    snprintf(lngBuf, sizeof(lngBuf), "%.5f", d.lng);
    snprintf(satBuf, sizeof(satBuf), "%d",   d.sats);
    snprintf(altBuf, sizeof(altBuf), "%.0f m", d.gpsAlt);

    struct { const char* label; const char* value; } rows[4] = {
        {"Latitude",   latBuf},
        {"Longitude",  lngBuf},
        {"Satellites", satBuf},
        {"Altitude",   altBuf},
    };

    int row_y = CONT_Y + 28;
    for (int i = 0; i < 4; i++) {
        int mid = row_y + 19;

        M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
        M5.Lcd.setTextDatum(ML_DATUM);
        M5.Lcd.drawString(rows[i].label, 14, mid, 2);

        M5.Lcd.setTextColor(WHITE, CLR_BG);
        M5.Lcd.setTextDatum(MR_DATUM);
        M5.Lcd.drawString(rows[i].value, 310, mid, 4);

        if (i < 3)
            M5.Lcd.drawFastHLine(10, row_y + 38, 300, CLR_SEP);

        row_y += 38;
    }
    M5.Lcd.setTextDatum(TL_DATUM);
}

// Page 3 — Statuts réseau & SD (WiFi, MQTT, carte SD, buffer)
void drawPageNet() {
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("STATUTS RESEAU & SD", 160, CONT_Y + 4, 2);

    struct { const char* label; String value; uint16_t color; } netRows[4];

    // --- Ligne 1 : WiFi ---
    netRows[0].label = "WiFi (RSSI)";
    if (WiFi.isConnected()) {
        long rssi = WiFi.RSSI();
        netRows[0].value = String(rssi) + " dBm";
        netRows[0].color = (rssi > -70) ? CLR_GOOD : CLR_OK;
    } else {
        netRows[0].value = "DECONNECTE";
        netRows[0].color = CLR_BAD;
    }

    // --- Ligne 2 : MQTT ---
    netRows[1].label = "MQTT Cloud";
    if (mqtt.connected()) {
        netRows[1].value = "CONNECTE";
        netRows[1].color = CLR_GOOD;
    } else {
        netRows[1].value = "ERREUR " + String(mqtt.state());
        netRows[1].color = CLR_BAD;
    }

    // --- Ligne 3 : Carte SD ---
    netRows[2].label = "Carte SD";
    if (sdAvailable) {
        uint64_t freeBytes = (SD.totalBytes() - SD.usedBytes()) / (1024 * 1024);
        netRows[2].value = String((unsigned long)freeBytes) + " MB libres";
        netRows[2].color = (freeBytes < 10) ? CLR_BAD : CLR_GOOD;
    } else {
        netRows[2].value = "ABSENTE";
        netRows[2].color = CLR_SEP;
    }

    // --- Ligne 4 : Buffer SD ---
    netRows[3].label = "Buffer SD";
    if (sdAvailable) {
        if (SD.exists("/buffer.txt")) {
            File f = SD.open("/buffer.txt", FILE_READ);
            long sz = f.size();
            f.close();
            if (sz < 1024) {
                netRows[3].value = String(sz) + " B";
            } else {
                netRows[3].value = String(sz / 1024) + " KB";
            }
            netRows[3].color = CLR_BAD; // Rouge = données en attente
        } else {
            netRows[3].value = "VIDE (OK)";
            netRows[3].color = CLR_GOOD;
        }
    } else {
        netRows[3].value = "INACTIF";
        netRows[3].color = CLR_SEP;
    }

    // Affichage des lignes
    int row_y = CONT_Y + 28;
    for (int i = 0; i < 4; i++) {
        int mid = row_y + 19;
        M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
        M5.Lcd.setTextDatum(ML_DATUM);
        M5.Lcd.drawString(netRows[i].label, 14, mid, 2);

        M5.Lcd.setTextColor(netRows[i].color, CLR_BG);
        M5.Lcd.setTextDatum(MR_DATUM);
        M5.Lcd.drawString(netRows[i].value, 310, mid, 4);

        if (i < 3) M5.Lcd.drawFastHLine(10, row_y + 38, 300, CLR_SEP);
        row_y += 38;
    }
    M5.Lcd.setTextDatum(TL_DATUM);
}

// Page 4 — Capteur de composés organiques volatils (TVOC / eCO2 via SGP30)
void drawPageSGP() {
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.setTextColor(CLR_CYAN, CLR_BG);
    M5.Lcd.drawString("TVOC / eCO2 (SGP30)", 160, CONT_Y + 4, 2);

    if (!d.sgpOk) {
        M5.Lcd.setTextColor(CLR_BAD, CLR_BG);
        M5.Lcd.drawString("Capteur non detecte", 160, CONT_Y + 80, 2);
        M5.Lcd.setTextDatum(TL_DATUM);
        return;
    }

    // ── TVOC ──────────────────────────────────────────────────────────────
    uint16_t tvocColor = qColor((float)d.tvoc, 150, 500);
    const char* tvocLabel = (d.tvoc < 150) ? "EXCELLENT" :
                            (d.tvoc < 500) ? "MOYEN"    : "MAUVAIS";

    char tvocBuf[20];
    snprintf(tvocBuf, sizeof(tvocBuf), "%d ppb", d.tvoc);

    int y1 = CONT_Y + 34;
    M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.drawString("TVOC", 14, y1, 2);
    M5.Lcd.setTextColor(tvocColor, CLR_BG);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(tvocBuf, 200, y1, 4);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(tvocLabel, 310, y1, 2);
    drawProgressBar(10, y1 + 16, 300, 14, d.tvoc / 1000.0f, tvocColor);

    M5.Lcd.drawFastHLine(10, CONT_Y + 78, 300, CLR_SEP);

    // ── eCO2 ──────────────────────────────────────────────────────────────
    uint16_t eco2Color = qColor((float)d.eco2, 800, 1500);
    const char* eco2Label = (d.eco2 < 800)  ? "BON"     :
                            (d.eco2 < 1500) ? "MOYEN"  : "MAUVAIS";

    char eco2Buf[20];
    snprintf(eco2Buf, sizeof(eco2Buf), "%d ppm", d.eco2);

    int y2 = CONT_Y + 100;
    M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.drawString("eCO2", 14, y2, 2);
    M5.Lcd.setTextColor(eco2Color, CLR_BG);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(eco2Buf, 200, y2, 4);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(eco2Label, 310, y2, 2);
    drawProgressBar(10, y2 + 16, 300, 14, d.eco2 / 3000.0f, eco2Color);

    // ── Note de chauffe ───────────────────────────────────────────────────
    M5.Lcd.setTextColor(CLR_LABEL, CLR_BG);
    M5.Lcd.setTextDatum(TC_DATUM);
    M5.Lcd.drawString("(15 min de chauffe pour stabilisation)", 160, CONT_Y + 155, 1);

    M5.Lcd.setTextDatum(TL_DATUM);
}