#ifndef H_STM32_SIMPLESCOPE_HPP
#define H_STM32_SIMPLESCOPE_HPP

#define TEST_CHANNEL PB1 // comment out define to disable test mode
const uint8_t CHANNEL_1 = PB0;
const uint8_t CHANNEL_2 = PA5;
#define SD_ENABLED 0 // Disabled by default now

#include <stdint.h>
#include <Adafruit_ILI9341_STM.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <series/adc.h>
#include <libmaple/adc.h>
#include <libmaple/dma.h>
#include <io.h>
#include <libmaple/dma_common.h>
#include <wirish_time.h>

#include "Encoder.hpp"

// TFT Pin connections
#define TFT_DC      PA0
#define TFT_CS      PA1
#define TFT_RST     PA2
#define TFT_LED     PA3 // Backlight
#define DFL_BACKLIGHT_BRIGHTNESS 64 // 0 off, 255 max
// The TFT uses SPI1 - Alternate on these pins...
// TFT_CLK to   PB3
// TFT_MISO to  PB4
// TFT_MOSI to  PB5
// N/C TFT_NSS  PA15

#define VCC_3_3 3.33 // volts

// TODO: Add support for higher rail voltage

// Constant definitions
static const uint16_t ADC_RESOLUTION = 4096; // 12-bits
static const uint16_t EFFECTIVE_VERTICAL_RESOLUTION = 200; // pixels
static const uint16_t SCREEN_HORIZONTAL_RESOLUTION = 320; // pixels
static const uint16_t SCREEN_VERTICAL_RESOLUTION = 240; // pixels
static const uint16_t BUFFER_SIZE = 1024; // bytes - FFT code requires 1024 bytes

/* ============================== STATS BEGIN ============================== */
typedef struct StatsTimeStruct {
  uint32_t tStart;
  uint32_t Total; // counted in ms
  uint32_t Last;
  void Start() { tStart = micros(); }
  void Stop() { if(!tStart) return; uint32_t tStop = micros(); Last = tStop - tStart; Total += Last/1000; }
} StatsTimes;
typedef struct StatsStruct {
  public:
    uint32_t fpsLatest = 0;
    uint32_t uStart = 0;
    StatsTimes Menu = { 0, 0, 0 };
    StatsTimes FFT = { 0, 0, 0 };
    StatsTimes TFT = { 0, 0, 0 };
    StatsTimes DMAFill = { 0, 0, 0 };
    StatsTimes DMAMiss = { 0, 0, 0 };
    StatsTimes Loop = { 0, 0, 0 };
    void init() {
    }
} Stats;
Stats stats;
/* ============================== STATS END ============================== */
/* ============================== ADC DATA BEGIN ============================== */
#define FFT_METRIC_HEIGHT 10
enum CHANNELS : uint8_t {
  CHAN_1=0,
  CHAN_2,
  CHAN_1n2, // 1 & 2
  CHAN_MAX
};
const char* CHANNEL_OPTS[CHAN_MAX] = { "1", "2", "1&2" };
typedef struct dataBufferStruct {
  uint16_t data[BUFFER_SIZE];
  uint8_t dataPlot[SCREEN_HORIZONTAL_RESOLUTION + 1]; // one extra byte for final line
  uint16_t maxy, avgy, miny;
} dataBuffer;
struct fftDataStruct {
  dataBuffer bufPadding[1]; // Padding for channel 1
  uint32_t data32[BUFFER_SIZE];
  uint32_t fftData[BUFFER_SIZE];
};
struct channel2Struct {
  dataBuffer buf[2]; // two channels
  uint16_t pad[BUFFER_SIZE]; // blank space when using 2 channels
  uint32_t dualADCBuf[BUFFER_SIZE];
};
union unifiedDataStruct {
  struct fftDataStruct fft;
  struct channel2Struct channelData;
};
/* ============================== ADC DATA END ============================== */
/* ============================== TRIGGER BEGIN ============================== */
// Trigger configuration
static const uint16_t TRIGGER_ALIGN_STEP = 10;
static const uint16_t TRIGGER_ALIGN_MAX = 320;
enum TRIG_MODES : uint8_t {
  TRIG_SPECTRUM = 0,
  TRIG_RISING,
  TRIG_RISING1,
  TRIG_FALLING,
  TRIG_FALLING1,
  TRIG_NONE,
  TRIG_MAX
};
static const uint16_t TRIGGER_LIMIT_MARGIN = 8;
static const char* TRIG_MODES[TRIG_MAX] = { "Spec", "\x18", "\x18\x31", "\x19", "\x19\x31", "None" };
static const uint16_t TRIGGER_START_LIMIT = (BUFFER_SIZE - SCREEN_HORIZONTAL_RESOLUTION -1);
/* ============================== TRIGGER END ============================== */
/* ============================== DATA OBJECTS BEGIN ============================== */
struct variablesStruct {
  volatile bool dmaActive = false;
  volatile bool dmaFreshData = false;
};
struct configStruct { // Data to be stored in EEPROM
  uint16_t trigAlign = 20;
  uint16_t trigVolt = (ADC_RESOLUTION / 2)+TRIGGER_LIMIT_MARGIN;
  uint8_t chan_set = CHAN_1;
  uint8_t trigger = TRIG_RISING;
  uint8_t time_base = 6; // See Data_Rates table below 6 = 182
  uint8_t testMenuPctSelection = 4; // 4 = 50%
  uint8_t testMenuPeriodSelection = 2; // 2 = 1ms period / 1kHz
};
typedef struct scopeDataStruct {
  unifiedDataStruct bigData;
  SPIClass tftSPI = SPIClass(1); // use alternate channel
  Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST);
  struct variablesStruct vars;
  struct configStruct config;
  void init() {
    // TODO: Add EEPROM load/save of config struct
  }

  void clearAllData(int8_t chan) {
    bzero(bigData.channelData.buf[chan].data, BUFFER_SIZE*2); // shorts = x2 bytes
    bzero(bigData.channelData.buf[chan].dataPlot, SCREEN_HORIZONTAL_RESOLUTION+1);
  }
} scopeData;
scopeData d;
uint16_t leftStart = 0;
/* ============================== DATA OBJECTS END ============================== */

typedef struct freq_map_struct { // TODO: Overflowed 32 bits per entry ... clean this up
  uint32_t TBPerDivision:13; // 0-2047
  uint32_t Mult:3; // 1,2,4
  uint32_t Prescaler:2; // 0-3
  uint32_t SampleRate:3; // 0-7
  uint32_t Spectrum:12; // 0-4095
} freq_map;
#define DATA_RATE_TIMER_MIN 11 // Start this at last 'raw' number, 1119
#define MAX_DATA_RATES 14
// TODO: Need to fix commented out spectrum rates, if enabled
static const freq_map Data_Rates[MAX_DATA_RATES] = {
  {   4, 4, 0, 0, 2560}, // 250kHz division
  {   8, 2, 0, 0, 2560}, // 125kHz division
  {  16, 1, 0, 0, 2560}, // 63kHz division
  {  31, 1, 1, 0, 1280}, // 32kHz division
  //{  44, 1, 1, 1, 1384}, // 22kHz division
  {  62, 1, 3, 0, 640},  // 16kHz division
  //{  88, 1, 3, 1, 667},  // 12kHz division
  { 120, 1, 1, 4, 341},  // 8kHz division
  { 182, 1, 3, 3, 225},  // 5KHz division
  { 301, 1, 3, 5, 134},  // 3.3kHz division
  { 559, 1, 1, 7, 73},   // 1.8 kHz division
  { 839, 1, 2, 7, 49},   // 1.2 kHz division
  {1119, 1, 3, 7, 35},   // 893Hz division
  {2000, 1, 3, 7, 20},   //
  {4000, 1, 3, 7, 10},   //
  {8000, 1, 3, 7, 5},    //
};

/* ============================== TFT BEGIN ============================== */
#define MENU_HEIGHT 20
const uint16_t BLACK   = 0x0000;
const uint16_t BLUE    = 0x001F;
const uint16_t RED     = 0xF800;
const uint16_t GREEN   = 0x07E0;
const uint16_t CYAN    = 0x07FF;
const uint16_t MAGENTA = 0xF81F;
const uint16_t YELLOW  = 0xFFE0;
const uint16_t WHITE   = 0xFFFF;

const uint16_t BACKGROUND_COLOR = ILI9341_BLACK;
const uint16_t DIV_LINE_COLOR = ILI9341_BLUE;
const uint16_t CH_COLORS[CHAN_1n2] = {ILI9341_YELLOW, ILI9341_CYAN };
const uint16_t COL_MENU_BACKGROUND = BLACK;
const uint16_t COL_MENU_FG_INACTIVE = ILI9341_BLUE;
const uint16_t COL_MENU_FG_ACTIVE = GREEN;
const uint16_t TRIGGER_COLOR = ILI9341_PINK;

#define DIVISION_SIZE 40 // 40 pixels
#define SUBDIVISION_SIZE (DIVISION_SIZE/5) // pixels (DIVISION_SIZE / 5)

static uint8_t lastSignalZoom = -1;
void tft_init() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // free PB3
  afio_remap(AFIO_REMAP_SPI1); // use alternate SPI1
  d.tft.begin(d.tftSPI);
  d.tft.setRotation(3);
  pinMode(TFT_LED, OUTPUT);
  analogWrite(TFT_LED, DFL_BACKLIGHT_BRIGHTNESS);
}
/* ============================== TFT END ============================== */
/* ============================== MENUS BEGIN ============================== */
#define MENUBAR_ROW_HT 8
#define MENUBAR_SPACE 1
#define MENUBAR_Y 1
#define MENUBAR_TOPROW1_Y (MENUBAR_SPACE) // top row
#define MENUBAR_TOPROW2_Y (MENUBAR_TOPROW1_Y+MENUBAR_SPACE+MENUBAR_ROW_HT) // second from top row
#define MENUBAR_BOTROW2_Y (SCREEN_VERTICAL_RESOLUTION - MENUBAR_SPACE*2 - MENUBAR_ROW_HT*2) // second from bottom row
#define MENUBAR_BOTROW1_Y (SCREEN_VERTICAL_RESOLUTION - MENUBAR_SPACE - MENUBAR_ROW_HT) // bottom row
#define MENUBAR_HEIGHT (MENUBAR_ROW_HT*2+MENUBAR_SPACE*3)
typedef struct MenuBarStruct {
  uint8_t PriY;
  uint8_t SecY;
} MenuBar;
MenuBar MenuBars[2] = {{ MENUBAR_TOPROW1_Y, MENUBAR_TOPROW2_Y }, {MENUBAR_BOTROW1_Y, MENUBAR_BOTROW2_Y}};
/* ============================== MENUS END ============================== */

#ifdef TEST_CHANNEL
#include "TestMode.hpp"
#endif

#endif
