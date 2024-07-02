/*
 * STM32 Digital Oscilloscope
 * using the STM32F103C8 MCU and the ILI9341 320x240 TFT display
 * https://www.gameinstance.com/post/80/STM32-Oscilloscope-with-FFT-and-SD-export
 *
 *  GameInstance.com
 *  2016-2018
 *
 * Updates by Me
 *  - Changed to leverage rotary encoder
 *  - SDCard disabled by default
 */
#include <Adafruit_ILI9341_STM.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <table_fft.h>
#include <cr4_fft_1024_stm32.h>

#define TFT_DC      PA0
#define TFT_CS      PA1
#define TFT_RST     PA2
#define TFT_LED     PA3 // Backlight

#define ENC_BUTTON PB7
#define ENC_CLK PB8
#define ENC_DATA PB9
static int8_t encoderCount = 0;
static bool encoderClick = false;
static uint8_t encSetting = 0;
static uint8_t encDepth = 0;

static const uint8_t SD_CHIP_SELECT = PB12;
#define TEST_SIGNAL PA4
#define CHANNEL_1 PB0

#define SD_ENABLED 0 // Disabled by default now
#define TEST_ENABLED 1 // Consider disabling

#if SD_ENABLED
  #include "SdFat.h"
  SdFat sd(2);
  SdFile file;
#endif
static const uint16_t BLACK   = 0x0000;
static const uint16_t BLUE    = 0x001F;
static const uint16_t RED     = 0xF800;
static const uint16_t GREEN   = 0x07E0;
static const uint16_t CYAN    = 0x07FF;
static const uint16_t MAGENTA = 0xF81F;
static const uint16_t YELLOW  = 0xFFE0;
static const uint16_t WHITE   = 0xFFFF;

static const uint16_t BACKGROUND_COLOR = BLUE;
static const uint16_t DIV_LINE_COLOR = GREEN;
static const uint16_t CH1_SIGNAL_COLOR = YELLOW;

static const uint16_t ADC_RESOLUTION = 4096; // units
static const uint16_t EFFECTIVE_VERTICAL_RESOLUTION = 200; // pixels
static const uint16_t SCREEN_HORIZONTAL_RESOLUTION = 320; // pixels
static const uint16_t SCREEN_VERTICAL_RESOLUTION = 240; // pixels
static const uint16_t DIVISION_SIZE = 40; // pixels
static const uint16_t SUBDIVISION_SIZE = 8; // pixels (DIVISION_SIZE / 5)
static const uint16_t BUFFER_SIZE = 1024; // bytes
static const float ADC_SCREEN_FACTOR = (float)EFFECTIVE_VERTICAL_RESOLUTION / (float)ADC_RESOLUTION;
static const float VCC_3_3 = 3.3; // volts

const uint8_t DT_DT[]   = {4,       2,     1,     1,     1,     1,     1,     1,     1,     1,     1};
const uint8_t DT_PRE[]  = {0,       0,     0,     0,     0,     0,     0,     0,     0,     0,     1};
const uint8_t DT_SMPR[] = {0,       0,     0,     1,     2,     3,     4,     5,     6,     7,     7};
const float DT_FS[]     = {2571, 2571,  2571,  1800,  1384,   878,   667,   529,   429,   143,  71.4};
const float DT_DIV[]    = {3.9,  7.81, 15.63, 22.73, 29.41, 45.45, 55.55, 83.33, 95.24, 293.3, 586.6};

#if TEST_ENABLED
int16_t test_value = 0;
int16_t test_delay = 32;
#endif

// Trigger configuration
static uint16_t trigVolt = (ADC_RESOLUTION / 2);
static uint16_t trigAlign = 20;
#define TRIGGER_ALIGN_STEP 10
#define TRIGGER_ALIGN_MAX 320
enum TRIG_MODES {
  TRIG_SPECTRUM = 0,
  TRIG_RISING,
  TRIG_FALLING,
  TRIG_NONE,
  TRIG_MAX
};
const char* TRIG_MODES[TRIG_MAX] = { "Spectrum", "Rising", "Falling", "None" };
uint8_t trigger = TRIG_RISING;

Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST);

uint8_t bk[SCREEN_HORIZONTAL_RESOLUTION];
uint16_t data16[BUFFER_SIZE];
uint32_t data32[BUFFER_SIZE];
uint32_t y[BUFFER_SIZE];
uint8_t time_base = 7;
uint16_t i, j;
uint8_t state = 0;
uint16_t maxy, avgy, miny;

uint8_t h = 1, h2 = -1;
uint8_t freezeMode = 0;
bool bTitleChange = true, bScreenChange = true;
volatile static bool dma1_ch1_Active;
static uint8_t encoderVal = 0;

void encoderClkInt(void) {
  if (digitalRead(ENC_DATA)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
void encoderPushInt(void) {
  encoderClick = true;
}

// The following section was inspired by http://www.stm32duino.com/viewtopic.php?t=1145
void setADCs() {
  switch (DT_PRE[time_base]) {
    case 0: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_2); break;
    case 1: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_4); break;
    case 2: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_6); break;
    case 3: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_8); break;
    default: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_8);
  }
  switch (DT_SMPR[time_base]) {
    case 0: adc_set_sample_rate(ADC1, ADC_SMPR_1_5); break;
    case 1: adc_set_sample_rate(ADC1, ADC_SMPR_7_5); break;
    case 2: adc_set_sample_rate(ADC1, ADC_SMPR_13_5); break;
    case 3: adc_set_sample_rate(ADC1, ADC_SMPR_28_5); break;
    case 4: adc_set_sample_rate(ADC1, ADC_SMPR_41_5); break;
    case 5: adc_set_sample_rate(ADC1, ADC_SMPR_55_5); break;
    case 6: adc_set_sample_rate(ADC1, ADC_SMPR_71_5); break;
    case 7: adc_set_sample_rate(ADC1, ADC_SMPR_239_5); break;
    default: adc_set_sample_rate(ADC1, ADC_SMPR_239_5);
  }
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = PIN_MAP[CHANNEL_1].adc_channel;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
}

void real_to_complex(uint16_t *in, uint32_t *out, int len) {
  for (int i = 0; i < len; i++) out[i] = in[i];// * 8;
}

uint16_t asqrt(uint32_t x) { //good enough precision, 10x faster than regular sqrt
  int32_t op, res, one;
  op = x;
  res = 0;
  one = 1 << 30;
  while (one > op) one >>= 2;
  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res + 2 * one;
    }
    res /= 2;
    one /= 4;
  }
  return (uint16_t)res;
}

void inplace_magnitude(uint32_t *target, uint16_t len) {
  uint16_t *p16;
  for (int i = 0; i < len; i++) {
    int16_t real = target[i] & 0xFFFF;
    int16_t imag = target[i] >> 16;
    //    target[i] = 10 * log10(real*real + imag*imag);
    uint32_t magnitude = asqrt(real * real + imag * imag);
    target[i] = magnitude;
  }
}

uint32_t perform_fft(uint32_t *indata, uint32_t *outdata, const int len) {
  cr4_fft_1024_stm32(outdata, indata, len);
  inplace_magnitude(outdata, len);
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}

void adc_dma_enable(const adc_dev *dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}

#if SD_ENABLED
void export_to_sd() {
  tft.setCursor(170, 20);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("Writing to SD ...");
  tft.setCursor(170, 20);
  if (!sd.cardBegin(SD_CHIP_SELECT, SD_SCK_HZ(F_CPU/4))) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("No SD card detected");
    return;
  }
  delay(500);
  if (!sd.fsBegin()) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("File system init failed.");
    return;
  }
  uint8_t index;
  if (!sd.exists("DSO")) {
    // no pre-exising folder structure
    if (!sd.mkdir("DSO")) {
      tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      tft.print("Can't create folder");
      return;
    }
  }
  if (!sd.exists("DSO/data.idx")) {
    // no index file
    index = 1;
    if (!file.open("DSO/data.idx", O_CREAT | O_WRITE)) {
      tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      tft.print("Can't create idx file");
      return;
    }
    file.write(index);
    if (!file.sync() || file.getWriteError()) {
      tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      tft.print("Idx file write error");
      return;
    }
    file.close();
  } else {
    if (!file.open("DSO/data.idx", O_READ)) {
      tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      tft.print("Can't open idx file");
      return;
    }
    if (!file.read(&index, 1)) {
      tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      tft.print("Can't read idx file");
      return;
    }
    if (!file.sync() || file.getWriteError()) {
      tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      tft.print("File write error");
      return;
    }
    file.close();
  }
  String s = "DSO/Exp";
  s += index;
  s += ".dat";
  if (!file.open(s.c_str(), O_CREAT | O_WRITE | O_EXCL)) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("Can't data create file");
    return;
  }
  file.println("Time series");
  for (uint16_t i = 0; i < BUFFER_SIZE; i ++) {
    file.print(data16[i], DEC);file.print(", ");
  }
  file.println(" ");
  file.print("Fs: ");file.print(DT_FS[time_base]);file.println("kHz");
  file.println("Spectrum");
  for (uint16_t i = 0; i < BUFFER_SIZE/2; i++) {
    file.print(y[i], DEC);file.print(", ");
  }
  file.println(" ");
  if (!file.sync() || file.getWriteError()) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("File write error");
    return;
  }
  file.close();
  s += ".img";
  if (!file.open(s.c_str(), O_CREAT | O_WRITE | O_EXCL)) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("Can't image create file");
    return;
  }
  file.println("IMX");
  for (uint16_t i = 0; i < BUFFER_SIZE; i ++) {
    file.print(data16[i], DEC);file.print(", ");
  }
  file.println(" ");
  file.print("Fs: ");file.print(DT_FS[time_base]);file.println("kHz");
  file.println("Spectrum");
  for (uint16_t i = 0; i < BUFFER_SIZE/2; i ++) {
    file.print(y[i], DEC);file.print(", ");
  }
  file.println(" ");
  if (!file.sync() || file.getWriteError()) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("File write error");
    return;
  }
  file.close();
  index ++;
  if (!file.open("DSO/data.idx", O_CREAT | O_WRITE)) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("Can't create idx file");
    return;
  }
  file.write(index);
  if (!file.sync() || file.getWriteError()) {
    tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    tft.print("Idx file write error");
    return;
  }
  file.close();
  tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
  tft.print("File write success");
  delay(2000);
  tft.fillRect(170, 19, 150, 9, BACKGROUND_COLOR);
}
#endif

void setup() {
  tft.begin();
  tft.setRotation(3);
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DATA, INPUT);
  pinMode(ENC_BUTTON, INPUT);
  pinMode(TEST_SIGNAL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderClkInt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_BUTTON), encoderPushInt, FALLING);
  adc_calibrate(ADC1);
}


void actSplash() {
  tft.fillScreen(BACKGROUND_COLOR);
  tft.setCursor(15, 100);
  tft.setTextColor(YELLOW);
  tft.setTextSize(3);
  tft.println("STM32 SimpleScope");
  tft.setCursor(15, 150);
  tft.println("Was GameInstance.com");
  delay(500);
  tft.fillScreen(BACKGROUND_COLOR);
}

int8_t getEncValue() {
    int8_t retval = encoderCount;
    encoderCount -= retval;
    return retval;
}
void addEncValue(uint8_t *val, uint8_t limit, uint8_t mult = 1) {
    int8_t change = getEncValue();
    while (change < 0) change += limit;
    *val += (change * mult);
    *val = *val % limit;
}
void addEncValue(uint16_t *val, uint16_t limit, uint16_t mult = 1) {
    int16_t change = getEncValue();
    while (change < 0) change += limit;
    *val += (change * mult);
    *val = *val % limit;
}
enum TOP_MENU {
  MENU_TIMEBASE = 0,
  MENU_TRIGMODE = 1,
  MENU_TRIGVOLT = 2,
  MENU_TRIGALIGN = 3,
  MENU_FREEZE = 4,
  MENU_SIZE
}; //
#define NUM_ENC_MODES MENU_SIZE
const char* encModes[NUM_ENC_MODES] = { "Time Base", "Trig Mode", "Trig Volt", "Trig Align", "Freeze" };
void actTimeBase() {
  addEncValue(&time_base, sizeof(DT_DT));
  if (trigger == TRIG_SPECTRUM) {
    if (time_base <= 2) time_base = 3;
  }
  time_base = time_base % sizeof(DT_DT);
  h = DT_DT[time_base];
  bScreenChange = true;
}

void actInputCheck() {
  if (!encoderCount && !encoderClick) return; // skip when no buttons pressed
  bTitleChange = true;
  if (encoderClick && !encDepth) {
    encDepth++;
    encoderClick = false;
  } else if (encoderClick && encDepth) {
    encDepth--;
    encoderClick = false;
  } else if (encoderCount && !encDepth) {
    addEncValue(&encSetting, NUM_ENC_MODES);
  } else if (encDepth && encoderCount) {
    switch (encSetting) {
      case MENU_TIMEBASE:
        actTimeBase();
        break;
      case MENU_TRIGMODE:
        addEncValue(&trigger, TRIG_MAX);
        bScreenChange = true;
        break;
      case MENU_TRIGVOLT:
        addEncValue(&trigVolt, ADC_RESOLUTION, (ADC_RESOLUTION / 32));
        break;
      case MENU_TRIGALIGN:
        addEncValue(&trigAlign, TRIGGER_ALIGN_MAX, TRIGGER_ALIGN_STEP);
        break;
      case MENU_FREEZE:
        freezeMode = (freezeMode) ? 0 : encDepth*3;
        encoderCount = 0;
        break;
      default:
        if(encDepth) encDepth = 0;
        break;
    }
  }
}

void actAcquisition() {
  setADCs();
  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);
  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_16BITS, data16, DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
  dma_set_num_transfers(DMA1, DMA_CH1, BUFFER_SIZE);
  dma1_ch1_Active = 1;
  dma_enable(DMA1, DMA_CH1);                     // enable the DMA channel and start the transfer

  while (dma1_ch1_Active) {};                    // waiting for the DMA to complete
  dma_disable(DMA1, DMA_CH1);                    // end of DMA trasfer

  if (trigger == TRIG_SPECTRUM) {
    real_to_complex(data16, data32, BUFFER_SIZE);// data format conversion
    perform_fft(data32, y, BUFFER_SIZE);         // FFT computation
  }
}

uint32_t fpsLatest = 0;
uint16_t leftStart = 0;
uint16_t newTrigger() {
  if (trigger != TRIG_FALLING && trigger != TRIG_RISING) return 1;
  uint16_t newStart = 0;
  int16_t firstEdge = -1;
  for (i = trigAlign; i < BUFFER_SIZE-SCREEN_HORIZONTAL_RESOLUTION && newStart == 0; i++) {
    if (trigger == TRIG_RISING) {
      if (firstEdge >= 0 && data16[i] > trigVolt) newStart = i;
      if (firstEdge < 0 && data16[i] < trigVolt) firstEdge = i;
    } else { // FALLING
      if (firstEdge >= 0 && data16[i] < trigVolt) newStart = i;
      if (firstEdge < 0 && data16[i] > trigVolt) firstEdge = i;
    }
  }
  return newStart;
}
void actDrawSignals() {
  uint16_t newStart = newTrigger();
  if (bScreenChange) {
    // massive change on screen
    bScreenChange = false;
    tft.fillScreen(BACKGROUND_COLOR);
    bTitleChange = true;
  }
  else {
    if(!newStart) return;
    // clear previous wave
    if (trigger == 0) {
      // clear previous spectrum
      for (i = 1; i < SCREEN_HORIZONTAL_RESOLUTION; i++) {
        tft.drawLine(i, bk[i], i + 1, bk[i + 1], BACKGROUND_COLOR);
      }
    }
    else {
      // clear previous time samples
      for (i = 0, j = 0; j < SCREEN_HORIZONTAL_RESOLUTION; i++, j += h2) {
        tft.drawLine(j, bk[i], j + h2, bk[i + 1], BACKGROUND_COLOR);
      }
      leftStart = 0;
    }
  }
  // re-draw the divisions
  for (i = 0; i < SCREEN_HORIZONTAL_RESOLUTION; i += DIVISION_SIZE) {
    for (j = SCREEN_VERTICAL_RESOLUTION; j > 13; j -= ((i == 160) ? SUBDIVISION_SIZE : DIVISION_SIZE)) {
      tft.drawLine(i - 1, j, i + 1, j, DIV_LINE_COLOR);
    }
  }
  for (i = SCREEN_VERTICAL_RESOLUTION; i > 13; i -= DIVISION_SIZE) {
    for (j = 0; j < SCREEN_HORIZONTAL_RESOLUTION; j += ((i == 120) ? SUBDIVISION_SIZE : DIVISION_SIZE)) {
      tft.drawLine(j, i - 1, j, i + 1, DIV_LINE_COLOR);
    }
  }
  // draw current wave
  fpsLatest++; // Testing FPS Counter
  if (trigger == 0) {
    // display spectrum
    uint16_t max_y = 0, max_x = 0;
    uint16_t i_0, i_1;
    bool hit_max = false;
    for (i = 1; i < BUFFER_SIZE / 2; i++) {
      if (y[i] > max_y) {
        max_y = y[i];
        max_x = i;
      }
    }
    max_y = max(max_y, EFFECTIVE_VERTICAL_RESOLUTION);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    for (i = 1; i < SCREEN_HORIZONTAL_RESOLUTION; i++) {
      i_0 = (int)((float)i * (float)BUFFER_SIZE / (float)SCREEN_HORIZONTAL_RESOLUTION / 2.0);
      i_1 = (int)((float)(i + 1) * (float)BUFFER_SIZE / (float)SCREEN_HORIZONTAL_RESOLUTION / 2.0);
      if (hit_max) {
        // was in the vicinity of max
        i_0 = max_x;
        hit_max = false;
      } else if ((max_x <= i_1) && (i_0 <= max_x)) {
        // is in the vicinity of max
        if ((i_1 - max_x) <= (max_x - i_0)) {
          hit_max = true;
          i_1 = max_x;
        } else {
          i_0 = max_x;
        }
      }
      bk[i] = SCREEN_VERTICAL_RESOLUTION - (10 + ((float)y[i_0] / (float)max_y) * (float)(EFFECTIVE_VERTICAL_RESOLUTION - 10));
      tft.drawLine(
        i,
        bk[i],
        i + 1,
        SCREEN_VERTICAL_RESOLUTION - (10 + ((float)y[i_1] / (float)max_y) * (float)(EFFECTIVE_VERTICAL_RESOLUTION - 10)),
        CH1_SIGNAL_COLOR);
      if (i % DIVISION_SIZE == 0) {
        float freq = ((float)i / (float)SCREEN_HORIZONTAL_RESOLUTION * (float)DT_FS[time_base]) / 2.0;
        tft.setCursor(i - (freq > 100 ? 8 : 5) - (freq > (int)freq ? 4 : 0), SCREEN_VERTICAL_RESOLUTION - 7);
        tft.print(freq, 1);
      }
    }
    // clear previous stats
    tft.fillRect(7, 19, 150, 9, BACKGROUND_COLOR);
    tft.setCursor(8, 20);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    String s;
    s = "F: ";
    s += (float)max_x / (float)BUFFER_SIZE * (float)DT_FS[time_base];
    s += "kHz ";
    s += (float)20 * log10(max_y);
    s += "dB";
    tft.print(s);
  }
  else {
    // display time samples
    uint16_t maxy = 0;
    uint16_t miny = ADC_RESOLUTION;
    uint32_t avgy = 0;
    leftStart = newStart;
    if (leftStart >= trigAlign) leftStart -= trigAlign;
    for (i = leftStart; i < leftStart + SCREEN_HORIZONTAL_RESOLUTION; i++) {
      maxy = max(maxy, data16[i]);
      miny = min(miny, data16[i]);
      avgy += data16[i];
    }
    avgy /= SCREEN_HORIZONTAL_RESOLUTION;
    for (i = leftStart, j = 0; j < SCREEN_HORIZONTAL_RESOLUTION; i++, j += h) {
      uint16_t base = i - leftStart;
      bk[base] = SCREEN_VERTICAL_RESOLUTION - (20 + (data16[i] * ADC_SCREEN_FACTOR));
      bk[base + 1] = SCREEN_VERTICAL_RESOLUTION - (20 + (data16[i + 1] * ADC_SCREEN_FACTOR));
      tft.drawLine(j, bk[base], j + h, bk[base + 1], CH1_SIGNAL_COLOR);
      if (h > 1)
        tft.drawPixel(j, bk[base] - 1, GREEN);
    }
    // clear previous stats
    tft.fillRect(7, 19, 60, 9, BLUE);
    tft.setCursor(8, 20);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    String s;
    s = "Max: ";
    s += (float)maxy / (float)ADC_RESOLUTION * VCC_3_3;
    s += "V";
    tft.print(s);

    tft.fillRect(SCREEN_HORIZONTAL_RESOLUTION / 2 - 30, SCREEN_VERTICAL_RESOLUTION - 20, 60, 9, BLUE);
    tft.setCursor(SCREEN_HORIZONTAL_RESOLUTION / 2 - 29, SCREEN_VERTICAL_RESOLUTION - 19);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    s = "Avg: ";
    s += (float)avgy / (float)ADC_RESOLUTION * VCC_3_3;
    s += "V";
    tft.print(s);

    tft.fillRect(7, SCREEN_VERTICAL_RESOLUTION - 20, 60, 9, BLUE);
    tft.setCursor(8, SCREEN_VERTICAL_RESOLUTION - 19);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    s = "Min: ";
    s += (float)miny / (float)ADC_RESOLUTION * VCC_3_3;
    s += "V";
    tft.print(s);
    h2 = h;
  }
}

void actDrawTitleBar() {
  if (bTitleChange) {
    bTitleChange = false;
    tft.fillRect(0, 0, SCREEN_HORIZONTAL_RESOLUTION, 12, WHITE);
    tft.setCursor(8, 3);
    tft.setTextColor(BLUE);
    tft.setTextSize(1);
    String s = "CH1 ";
    s += ((float)trigVolt / (float)ADC_RESOLUTION * VCC_3_3);
    s += "V ";
    if (trigger == TRIG_SPECTRUM) {
      // spectrum
      s += (int)DT_FS[time_base];
      s += "kHz ";
    } else {
      // time samples
      s += DT_DIV[time_base];
      s += "us ";
    }
    s += TRIG_MODES[trigger];
    tft.print(s);
    if (freezeMode) {
      tft.setCursor(170, 3);
      tft.setTextColor(RED);
      tft.setTextSize(1);
      tft.print("Freeze");
    }
    tft.setCursor(215, 3);
    if (encDepth)
      tft.setTextColor(RED);
    else
      tft.setTextColor(BLUE);
    tft.setTextSize(1);
    tft.print(encModes[encSetting]);

    // TODO: Move FPS to refresh always
    uint32_t fps = fpsLatest / (millis() / 1000);
    tft.print(" FPS");
    tft.print(fps);
  }

  if (freezeMode == 3) {
    freezeMode = 1;
#if SD_ENABLED
    export_to_sd();
#endif
    bScreenChange = true;
  }
}

void loop() {
  switch(state) {
    case 0:
      actSplash();
      break;
    case 1:
#if TEST_ENABLED
      // test output
      //analogWrite(TEST_SIGNAL, (++test_value%256));
      analogWrite(TEST_SIGNAL, 127);
#endif
      break;
    case 2:
      actInputCheck();
      if (freezeMode) state +=2;
      break;
    case 3:
      actAcquisition();
      break;
    case 4:
      actDrawSignals();
      break;
    case 5:
      actDrawTitleBar();
      break;
  }
  if (++state > 5) state = 1;
}
