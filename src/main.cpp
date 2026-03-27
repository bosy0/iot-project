#include <M5Stack.h>
#include <PMS.h>
#include <TinyGPS++.h>

HardwareSerial SerialPMS(1); // Serial1 sur GPIO33(RX) / GPIO32(TX)
PMS pms(SerialPMS);
PMS::DATA data;

TinyGPSPlus gps; // GPS M003 sur Serial2 (GPIO16/17) via bus inférieur

// Couleurs personnalisées pour l'interface
#define M5_GREY 0x3186

void setup()
{
    M5.begin();
    M5.Power.begin();

    // Configuration de l'écran
    M5.Lcd.setBrightness(100);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);

    // En-tête
    M5.Lcd.fillRect(0, 0, 320, 30, M5_GREY);
    M5.Lcd.setTextDatum(MC_DATUM); // Centre le texte
    M5.Lcd.drawString("MONITEUR QUALITE AIR", 160, 15, 2);

    // Initialisation UART2 (PMS5003 : Baud 9600)
    // Pin 16 = RX (vers TX capteur), Pin 17 = TX (vers RX capteur)
    SerialPMS.begin(9600, SERIAL_8N1, 36, 26); // PMS5003 : RX=GPIO36, TX=GPIO26
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // GPS M003 : bus inférieur

    dacWrite(25, 0); // Coupe le haut-parleur

    pms.passiveMode();
    pms.wakeUp();

    // En-tête section GPS
    M5.Lcd.drawFastHLine(20, 165, 280, M5_GREY);
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.setTextColor(M5_GREY, BLACK);
    M5.Lcd.drawString("GPS", 10, 175, 1);
}

void drawValue(int x, int y, const char *label, int value, uint16_t color)
{
    // Dessine un petit encadré pour chaque mesure
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.drawString(label, x + 50, y, 2);

    M5.Lcd.fillRect(x + 50, y + 30, 90, 30, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.drawNumber(value, x + 50, y + 30, 4);
    M5.Lcd.setTextSize(1);
    M5.Lcd.drawString("ug/m3", x + 50, y + 60, 2);
}

void drawGPS()
{
    bool valid = gps.location.isValid();
    uint16_t color = valid ? GREEN : DARKGREY;

    // Latitude
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextDatum(ML_DATUM);
    M5.Lcd.drawString("Lat :", 10, 195, 2);
    M5.Lcd.fillRect(65, 185, 160, 18, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    if (valid)
        M5.Lcd.drawFloat(gps.location.lat(), 6, 65, 195, 2);
    else
        M5.Lcd.drawString("-- No fix --", 65, 195, 2);

    // Longitude
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.drawString("Lon :", 10, 218, 2);
    M5.Lcd.fillRect(65, 208, 160, 18, BLACK);
    M5.Lcd.setTextColor(color, BLACK);
    if (valid)
        M5.Lcd.drawFloat(gps.location.lng(), 6, 65, 218, 2);
    else
        M5.Lcd.drawString("-- No fix --", 65, 218, 2);

    // Nombre de satellites
    M5.Lcd.setTextColor(M5_GREY, BLACK);
    M5.Lcd.fillRect(230, 185, 90, 18, BLACK);
    char satBuf[16];
    sprintf(satBuf, "Sats: %d", gps.satellites.value());
    M5.Lcd.drawString(satBuf, 230, 195, 1);
}

void loop()
{
    static unsigned long lastPMS = 0;
    static double lastLat = 0, lastLng = 0;

    // PMS : lecture toutes les secondes
    if (millis() - lastPMS >= 1000)
    {
        lastPMS = millis();
        pms.requestRead();
    }

    if (pms.read(data))
    {
        // Affichage des 3 valeurs principales
        drawValue(10, 60, "PM 1.0", data.PM_AE_UG_1_0, CYAN);

        // PM 2.5 (La plus importante pour la santé)
        uint16_t pm25Color = GREEN;
        if (data.PM_AE_UG_2_5 > 25)
            pm25Color = YELLOW;
        if (data.PM_AE_UG_2_5 > 50)
            pm25Color = RED;
        drawValue(110, 60, "PM 2.5", data.PM_AE_UG_2_5, pm25Color);

        // PM 10
        drawValue(210, 60, "PM 10", data.PM_AE_UG_10_0, MAGENTA);

        // Séparateur
        M5.Lcd.drawFastHLine(20, 160, 280, M5_GREY);

        // Feedback visuel de mise à jour (petit point qui clignote)
        static bool blink = false;
        M5.Lcd.fillCircle(300, 15, 3, blink ? GREEN : M5_GREY);
        blink = !blink;
    }

    // Lecture GPS en continu
    while (Serial2.available())
        gps.encode(Serial2.read());

    // Redessine uniquement si la position a changé
    if (gps.location.lat() != lastLat || gps.location.lng() != lastLng)
    {
        lastLat = gps.location.lat();
        lastLng = gps.location.lng();
        drawGPS();
    }

    M5.update();
}