#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

/*
 * =====================================================================
 *  DIY NVG (noktowizor) – ESP32-CAM (AI Thinker, ESP32-S) + OV7725 + GC9A01
 * =====================================================================
 *  - Start od razu po podaniu zasilania, bez WiFi i bez Bluetooth.
 *  - Obraz z kamery OV7725 (QVGA 320x240) -> wycięty środek 240x240
 *    -> okrągły ekran GC9A01 240x240 po SPI.
 *  - Ten sam kod dla obu oczu: zmień tylko EYE (EYE_LEFT / EYE_RIGHT).
 *
 *  Wymagania (Arduino IDE):
 *   1) Płytki: "esp32 by Espressif Systems" (2.0.x lub 3.x)
 *      Płytka: "AI Thinker ESP32-CAM"
 *   2) Biblioteka: "GFX Library for Arduino" (autor: Moon On Our Nation)
 *      Piny wpisane poniżej – NIE trzeba edytować plików biblioteki.
 *
 *  Połączenia ekranu GC9A01 -> ESP32-CAM:
 *   VCC -> 5V | GND -> GND | DIN -> IO13 | CLK -> IO14
 *   CS  -> IO15 | DC -> IO12 | RST -> IO2 | BL -> nie podłączony
 *
 *  Wgrywanie (FTDI232): IO0 -> GND, RESET, Upload, potem odłącz IO0 i RESET.
 * =====================================================================
 */

#include <Arduino.h>
#include "esp_camera.h"
#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =====================================================================
//  USTAWIENIA – zmieniaj tylko tutaj
// =====================================================================

// --- Które oko? ------------------------------------------------------
#define EYE_LEFT   0
#define EYE_RIGHT  1
#define EYE        EYE_LEFT   // <- zmień na EYE_LEFT albo EYE_RIGHT

#if EYE == EYE_LEFT
  #define SCREEN_ROTATION 0   // obrót ekranu: 0=0°, 1=90°, 2=180°, 3=270°
  #define MIRROR_X   0   // 1 = odbicie lustrzane w poziomie (lewo-prawo)
  #define FLIP_Y     0   // 1 = odbicie w pionie (góra-dół)
#else
  #define SCREEN_ROTATION 0   // obrót ekranu: 0=0°, 1=90°, 2=180°, 3=270°
  #define MIRROR_X   0   // 1 = odbicie lustrzane w poziomie (lewo-prawo)
  #define FLIP_Y     0   // 1 = odbicie w pionie (góra-dół)
#endif

// --- Obraz -----------------------------------------------------------
#define COLOR_MODE   0      // 0 = czarno-biały (najlepszy przy IR 850nm)
                             // 1 = zielony "fosfor" jak w klasycznym NVG
                             // 2 = oryginalne kolory z kamery
#define NVG_BRIGHTNESS   0      // jasność programowa: -100 ... +100 (0 = bez zmian)
#define NVG_CONTRAST     1.20f  // kontrast programowy: 0.50 ... 3.00 (1.00 = bez zmian)
#define NVG_GAMMA        0.85f  // < 1.0 rozjaśnia cienie, > 1.0 przyciemnia (1.00 = bez zmian)

// --- Diagnostyka -----------------------------------------------------
#define SHOW_FPS     1      // 1 = licznik FPS na ekranie i w Serial Monitor, 0 = wyłączony

// --- Sprzęt ----------------------------------------------------------
#define TFT_SPI_FREQ          40000000UL  // 40 MHz = stabilnie, 80 MHz = szybciej (krótkie przewody!)
#define CAM_XCLK_FREQ     20000000      // zegar kamery 20 MHz (przy problemach spróbuj 10000000)
#define DISABLE_BROWNOUT  1             // 1 = wyłącz reset przy chwilowym spadku napięcia

// =====================================================================
//  PINY – zgodne ze schematem z filmu
// =====================================================================

// Ekran GC9A01 (SPI)
#define TFT_MOSI  13   // DIN
#define TFT_SCLK  14   // CLK
#define TFT_CS    15   // CS
#define TFT_DC    12   // DC
#define TFT_RST    2   // RST

// Kamera – standardowe piny modułu AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN      4   // biała dioda na płytce – trzymamy wyłączoną

// =====================================================================
//  Kolory (RGB565) i bufory
// =====================================================================
#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_RED    0xF800
#define C_GREEN  0x07E0
#define C_YELLOW 0xFFE0

#define SCREEN_W   240
#define SCREEN_H   240
#define BAND_LINES 20   // ile linii wysyłamy do ekranu na raz

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, SCREEN_ROTATION, true /* IPS */);

static uint16_t lut[256];                         // jasność (0..255) -> kolor RGB565
static uint16_t lineBuf[SCREEN_W * BAND_LINES];   // bufor pasa linii

static uint32_t fpsTimer = 0;
static uint16_t fpsFrames = 0;
static uint16_t fpsValue = 0;
static uint16_t fbFailCount = 0;

// =====================================================================
//  Pomocnicze
// =====================================================================
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t bl) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (bl >> 3);
}

// Tablica: jasność -> kolor (kontrast, jasność, gamma, tryb koloru)
void buildLut() {
  for (int i = 0; i < 256; i++) {
    float v = i / 255.0f;
    v = (v - 0.5f) * NVG_CONTRAST + 0.5f + (NVG_BRIGHTNESS / 255.0f);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    v = powf(v, NVG_GAMMA);
    uint8_t g = (uint8_t)(v * 255.0f + 0.5f);
#if COLOR_MODE == 1
    lut[i] = rgb565(g / 5, g, g / 4);   // zielony fosfor
#else
    lut[i] = rgb565(g, g, g);           // czarno-biały
#endif
  }
}

void showMessage(const char *line1, const char *line2, const char *line3, uint16_t color) {
  gfx->fillScreen(C_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(color, C_BLACK);
  gfx->setCursor(30, 80);
  gfx->println(line1);
  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(30, 115);
  gfx->println(line2);
  gfx->setCursor(30, 135);
  gfx->println(line3);
}

// Próba uruchomienia kamery w danym formacie
bool tryCameraInit(pixformat_t format, int xclk) {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = xclk;
  config.pixel_format = format;
  config.frame_size   = FRAMESIZE_QVGA;   // 320x240 – OV7725 obsługuje max VGA
  config.jpeg_quality = 12;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count    = 2;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count    = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[KAMERA] init nieudany (format %d, XCLK %d Hz): 0x%x\n", (int)format, xclk, err);
    return false;
  }
  Serial.printf("[KAMERA] OK (format %d, XCLK %d Hz)\n", (int)format, xclk);
  return true;
}

bool initCamera() {
  // Kolejność prób: RGB565 -> GRAYSCALE -> wolniejszy zegar
  if (tryCameraInit(PIXFORMAT_RGB565, CAM_XCLK_FREQ)) return true;
  delay(100);
  if (tryCameraInit(PIXFORMAT_GRAYSCALE, CAM_XCLK_FREQ)) return true;
  delay(100);
  if (tryCameraInit(PIXFORMAT_RGB565, 10000000)) return true;
  return false;
}

void setupSensor() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  Serial.printf("[KAMERA] PID czujnika: 0x%02X %s\n", s->id.PID,
                s->id.PID == 0x77 ? "(OV7725)" : "(inny czujnik – kod nadal powinien działać)");
  // Automatyka ekspozycji i wzmocnienia – ważne w nocy przy oświetlaczu IR
  if (s->set_exposure_ctrl) s->set_exposure_ctrl(s, 1);
  if (s->set_gain_ctrl)     s->set_gain_ctrl(s, 1);
  if (s->set_whitebal)      s->set_whitebal(s, 1);
  // Odbicia robimy programowo (MIRROR_X / FLIP_Y), więc czujnik zostaje bez odbić
  if (s->set_hmirror)       s->set_hmirror(s, 0);
  if (s->set_vflip)         s->set_vflip(s, 0);
}

// =====================================================================
//  SETUP – uruchamia się od razu po podaniu zasilania
// =====================================================================
void setup() {
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  // Radio całkowicie wyłączone – brak WiFi i Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.begin(115200);
  Serial.println();
  Serial.println("=== DIY NVG start ===");
  Serial.printf("Oko: %s | PSRAM: %s\n", EYE == EYE_LEFT ? "LEWE" : "PRAWE", psramFound() ? "TAK" : "NIE");

  // Najpierw ekran – żeby móc pokazać ewentualny błąd kamery
  if (!gfx->begin(TFT_SPI_FREQ)) {
    Serial.println("[EKRAN] gfx->begin() nieudany – sprawdź połączenia ekranu");
  }
  gfx->fillScreen(C_BLACK);
  buildLut();

  // Kamera – jeśli nie wystartuje, komunikat i ponowna próba co 3 s
  while (!initCamera()) {
    Serial.println("[KAMERA] Camera init failed – sprawdź taśmę kamery i zasilanie 5V");
    showMessage("BLAD KAMERY", "Sprawdz tasme OV7725", "i zasilanie 5V (min. 1A)", C_RED);
    delay(3000);
  }
  setupSensor();

  // Pierwsze klatki po starcie bywają ciemne – pomijamy kilka
  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }

  gfx->fillScreen(C_BLACK);
  fpsTimer = millis();
}

// =====================================================================
//  LOOP – klatka z kamery -> ekran
// =====================================================================
void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    fbFailCount++;
    Serial.println("[KAMERA] Brak klatki");
    if (fbFailCount == 20) {
      showMessage("BRAK OBRAZU", "Kamera nie wysyla klatek", "Restart za chwile...", C_YELLOW);
    }
    if (fbFailCount > 40) ESP.restart();
    delay(50);
    return;
  }
  fbFailCount = 0;

  const int W = fb->width;
  const int H = fb->height;
  const bool gray = (fb->format == PIXFORMAT_GRAYSCALE);
  const int bpp = gray ? 1 : 2;

  // Wycinamy środek obrazu o rozmiarze ekranu
  const int outW = W < SCREEN_W ? W : SCREEN_W;
  const int outH = H < SCREEN_H ? H : SCREEN_H;
  const int srcX0 = (W - outW) / 2;
  const int srcY0 = (H - outH) / 2;
  const int dstX0 = (SCREEN_W - outW) / 2;
  const int dstY0 = (SCREEN_H - outH) / 2;

  // Zabezpieczenie przed niepełną klatką
  if (fb->len < (size_t)(W * H * bpp)) {
    esp_camera_fb_return(fb);
    return;
  }

  for (int y = 0; y < outH; y += BAND_LINES) {
    const int lines = (outH - y) < BAND_LINES ? (outH - y) : BAND_LINES;
    for (int j = 0; j < lines; j++) {
      const int oy = y + j;
      const int sy = srcY0 + (FLIP_Y ? (outH - 1 - oy) : oy);
      uint16_t *dst = lineBuf + j * outW;

      if (gray) {
        const uint8_t *src = fb->buf + sy * W + srcX0;
        for (int x = 0; x < outW; x++) {
          const int sx = MIRROR_X ? (outW - 1 - x) : x;
          dst[x] = lut[src[sx]];
        }
      } else {
        const uint8_t *src = fb->buf + (sy * W + srcX0) * 2;
        for (int x = 0; x < outW; x++) {
          const int sx = MIRROR_X ? (outW - 1 - x) : x;
          const uint16_t p = ((uint16_t)src[sx * 2] << 8) | src[sx * 2 + 1];
#if COLOR_MODE == 2
          dst[x] = p;
#else
          const uint8_t r  = (p >> 8) & 0xF8;
          const uint8_t g  = (p >> 3) & 0xFC;
          const uint8_t bl = (p << 3) & 0xF8;
          const uint8_t lum = (uint8_t)((r * 77 + g * 150 + bl * 29) >> 8);
          dst[x] = lut[lum];
#endif
        }
      }
    }
    gfx->draw16bitRGBBitmap(dstX0, dstY0 + y, lineBuf, outW, lines);
  }

  esp_camera_fb_return(fb);

#if SHOW_FPS
  fpsFrames++;
  const uint32_t now = millis();
  if (now - fpsTimer >= 1000) {
    fpsValue = fpsFrames;
    fpsFrames = 0;
    fpsTimer = now;
    Serial.printf("FPS: %u\n", fpsValue);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(C_GREEN, C_BLACK);
  gfx->setCursor(84, 22);
  gfx->printf("%2u FPS", fpsValue);
#endif
}
