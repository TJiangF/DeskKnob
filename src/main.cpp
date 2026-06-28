#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI(240, 240);

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Test ===");
  Serial.printf("Pins: BL=%d CS=%d DC=%d SCK=%d MOSI=%d RST=%d\n",
                TFT_BL, TFT_CS, TFT_DC, TFT_SCLK, TFT_MOSI, TFT_RST);
#ifdef USE_HSPI_PORT
  Serial.println("SPI port: HSPI (SPI3)");
#else
  Serial.println("SPI port: FSPI (SPI2 / default)");
#endif
  Serial.printf("SPI_FREQUENCY=%d\n", SPI_FREQUENCY);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("BL ON");
  Serial.flush();

  Serial.println("calling tft.begin()...");
  Serial.flush();
  tft.begin();
  Serial.println("tft.begin OK");
  Serial.flush();

  tft.fillScreen(TFT_BLUE);
  Serial.println("fillScreen BLUE");
  delay(800);
  tft.fillScreen(TFT_RED);
  Serial.println("fillScreen RED");
  delay(800);
  tft.fillScreen(TFT_GREEN);
  Serial.println("fillScreen GREEN");
  Serial.println("=== Done ===");
}

void loop() { delay(1000); }
