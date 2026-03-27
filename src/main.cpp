#include <M5Stack.h>
#include <PMS.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "Adafruit_BME680.h"

HardwareSerial SerialPMS(1);
PMS pms(SerialPMS);
PMS::DATA pmsData;

TinyGPSPlus gps;
Adafruit_BME680 bme;

#define M5_GREY 0x3186

// ── Batterie ────────────────────────────────────────────────────────────────
void drawBattery()
{
    int bat = M5.Power.getBatteryLevel();
    uint16_t color = bat > 50 ? GREEN : (bat > 20 ? YELLOW : RED);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.setTextColor(color, M5_GREY);
    char buf[8];
    sprintf(buf, "%d%%", bat);
    M5.Lcd.drawString(buf, 288, 15, 2);
    M5.Lcd.setTextDatum(TL_DATUM);
}

// ── Couleur qualité air (partagée PM + BME680) ──────────────────────────────
// Retourne GREEN / CYAN / YELLOW / RED selon 3 seuils croissants
uint16_t qualityColor(float value, float seuil1, float seuil2, float seuil3)
{
    if (value < seuil1) return GREEN;
    if (value < seuil2) return CYAN;
    if (value < seuil3) return YELLOW;
    return RED;
}

// ── Particules (PM) ─────────────────────────────────────────────────────────
// cx = centre horizontal de la colonne, y = ligne de départ
void drawValue(int cx, int y, const char *label, int value, uint16_t color)
{
    M5.Lcd.setTextDatum(TC_DATUM);

    // Label
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.drawString(label, cx, y, 2);

    // Fond noir + valeur (font 4, taille 1 ≈ 26px haut, ~14px/chiffre)
    M5.Lcd.fillRect(cx - 42, y + 18, 84, 28, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    M5.Lcd.drawNumber(value, cx, y + 18, 4);

    // Unité
    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.drawString("ug/m3", cx, y + 50, 1);

    M5.Lcd.setTextDatum(TL_DATUM);
}

// ── Chimie (BME680) ──────────────────────────────────────────────────────────
void drawChemistry()
{
    if (!bme.performReading()) return;

    float gasKOhm = bme.gas_resistance / 1000.0f;
    // Seuils inversés : plus la résistance est haute, meilleur c'est
    uint16_t color = qualityColor(-gasKOhm, -300, -150, -50);
    const char *quality;
    if      (gasKOhm > 300) quality = "EXCELLENT";
    else if (gasKOhm > 150) quality = "BON";
    else if (gasKOhm >  50) quality = "MOYEN";
    else                    quality = "MAUVAIS";

    M5.Lcd.setTextDatum(ML_DATUM);

    // Valeur résistance gaz
    M5.Lcd.fillRect(65, 138, 145, 18, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    char buf[24];
    sprintf(buf, "%.0f kOhm", gasKOhm);
    M5.Lcd.drawString(buf, 65, 148, 2);

    // Qualité
    M5.Lcd.fillRect(215, 138, 102, 18, BLACK);
    M5.Lcd.drawString(quality, 217, 148, 2);

    M5.Lcd.setTextDatum(TL_DATUM);
}

// ── GPS ──────────────────────────────────────────────────────────────────────
void drawGPS()
{
    bool valid = gps.location.isValid();
    uint16_t color = valid ? GREEN : DARKGREY;

    M5.Lcd.setTextDatum(ML_DATUM);

    // Latitude
    M5.Lcd.fillRect(65, 183, 155, 18, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    if (valid) M5.Lcd.drawFloat(gps.location.lat(), 6, 65, 193, 2);
    else       M5.Lcd.drawString("-- No fix --",    65, 193, 2);

    // Longitude
    M5.Lcd.fillRect(65, 207, 155, 18, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    if (valid) M5.Lcd.drawFloat(gps.location.lng(), 6, 65, 217, 2);
    else       M5.Lcd.drawString("-- No fix --",    65, 217, 2);

    // Satellites
    M5.Lcd.fillRect(228, 183, 88, 18, BLACK);
    M5.Lcd.setTextColor(M5_GREY, BLACK);
    char satBuf[12];
    sprintf(satBuf, "Sats: %d", gps.satellites.value());
    M5.Lcd.drawString(satBuf, 230, 193, 1);

    M5.Lcd.setTextDatum(TL_DATUM);
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    M5.begin();
    M5.Power.begin();
    dacWrite(25, 0); // Coupe le haut-parleur

    M5.Lcd.setBrightness(100);
    M5.Lcd.fillScreen(BLACK);

    // En-tête
    M5.Lcd.fillRect(0, 0, 320, 30, M5_GREY);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextColor(WHITE, M5_GREY);
    M5.Lcd.drawString("MONITEUR QUALITE AIR", 150, 15, 2);
    drawBattery();

    // Séparateurs
    M5.Lcd.drawFastHLine(10, 128, 300, M5_GREY);  // PM / Chimie
    M5.Lcd.drawFastHLine(10, 172, 300, M5_GREY);  // Chimie / GPS

    // Labels de section fixes
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.setTextColor(M5_GREY, BLACK);
    M5.Lcd.drawString("CHIMIE", 10, 136, 1);
    M5.Lcd.drawString("GPS",    10, 180, 1);

    // Labels fixes chimie
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.drawString("VOC :", 10, 148, 2);

    // Labels fixes GPS
    M5.Lcd.drawString("Lat :", 10, 193, 2);
    M5.Lcd.drawString("Lon :", 10, 217, 2);
    M5.Lcd.setTextDatum(TL_DATUM);

    // UART PMS5003 sur Serial1 (RX=GPIO36, TX=GPIO26)
    SerialPMS.begin(9600, SERIAL_8N1, 36, 26);
    pms.passiveMode();
    pms.wakeUp();

    // UART GPS M003 sur Serial2 (bus inférieur GPIO16/17)
    Serial2.begin(9600, SERIAL_8N1, 16, 17);

    // BME680 sur I2C (SDA=GPIO21, SCL=GPIO22)
    Wire.begin(21, 22);
    if (!bme.begin(0x77) && !bme.begin(0x76)) {
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.setTextDatum(ML_DATUM);
        M5.Lcd.drawString("BME680 ERR", 65, 148, 2);
        M5.Lcd.setTextDatum(TL_DATUM);
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320°C pendant 150ms
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    static unsigned long lastPMS = 0;
    static unsigned long lastBME = 0;
    static unsigned long lastBat = 0;
    static double lastLat = 0, lastLng = 0;

    // PMS5003 : toutes les secondes
    if (millis() - lastPMS >= 1000) {
        lastPMS = millis();
        pms.requestRead();
    }
    if (pms.read(pmsData)) {
        // Seuils : excellent / bon / moyen / mauvais
        drawValue(53,  38, "PM 1.0", pmsData.PM_AE_UG_1_0,
            qualityColor(pmsData.PM_AE_UG_1_0,  5, 15, 25));

        drawValue(160, 38, "PM 2.5", pmsData.PM_AE_UG_2_5,
            qualityColor(pmsData.PM_AE_UG_2_5, 10, 25, 50));

        drawValue(267, 38, "PM 10",  pmsData.PM_AE_UG_10_0,
            qualityColor(pmsData.PM_AE_UG_10_0, 20, 50, 100));

        // Point clignotant de mise à jour
        static bool blink = false;
        M5.Lcd.fillCircle(308, 15, 4, blink ? GREEN : M5_GREY);
        blink = !blink;
    }

    // BME680 : toutes les 3 secondes
    if (millis() - lastBME >= 3000) {
        lastBME = millis();
        drawChemistry();
    }

    // Batterie : toutes les 30 secondes
    if (millis() - lastBat >= 30000) {
        lastBat = millis();
        drawBattery();
    }

    // GPS : lecture continue, affichage si position change
    while (Serial2.available())
        gps.encode(Serial2.read());
    if (gps.location.lat() != lastLat || gps.location.lng() != lastLng) {
        lastLat = gps.location.lat();
        lastLng = gps.location.lng();
        drawGPS();
    }

    M5.update();
}
