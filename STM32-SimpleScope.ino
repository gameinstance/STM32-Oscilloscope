/*
 * STM32 Simplescope using the STM32F103C8
 */

#include "STM32-SimpleScope.hpp"
#include "Menus.hpp"
#include "ADC.hpp"

void clearFormerData(uint8_t chan) {
  for (uint16_t i = 0, j = 0; j < SCREEN_HORIZONTAL_RESOLUTION; i++, j += lastSignalZoom) {
    d.tft.drawLine(j, d.bigData.channelData.buf[chan].dataPlot[i], j + lastSignalZoom, d.bigData.channelData.buf[chan].dataPlot[i + 1], BACKGROUND_COLOR);
  }
}
void drawTrigger() {
  int32_t y = MENU_HEIGHT + ((ADC_RESOLUTION - d.config.trigVolt) * EFFECTIVE_VERTICAL_RESOLUTION / ADC_RESOLUTION);
  d.tft.drawFastHLine(0, y, SCREEN_HORIZONTAL_RESOLUTION, TRIGGER_COLOR);
}
void drawDivisions() {
  // re-draw the divisions
  uint16_t j;
  for (uint16_t i = 0; i < SCREEN_HORIZONTAL_RESOLUTION; i += DIVISION_SIZE) {
    // TODO: Replace numbers with named constants
    for (j = SCREEN_VERTICAL_RESOLUTION; j > 13; j -= ((i == 160) ? SUBDIVISION_SIZE : DIVISION_SIZE)) {
      d.tft.drawFastHLine(i-1, j, 3, DIV_LINE_COLOR);
    }
  }
  drawTrigger();
}

uint16_t newTrigger() {
  if (menus.isInTriggerMenu()) return 0;
  if (!adc.isDataReady()) return 0; // nothing to plot yet
  if (d.config.trigger == TRIG_SPECTRUM || d.config.trigger == TRIG_NONE) return 1;
  // TODO: Expand to fancier triggers here
  uint16_t newStart = 0;
  int16_t firstEdge = -1;
  // We currently only trigger on active channel (or 1 if both)
  uint8_t chan = (d.config.chan_set == CHAN_2) ? CHAN_2 : CHAN_1;
  for (uint16_t i = d.config.trigAlign; i < TRIGGER_START_LIMIT && newStart == 0; i++) {
    if (d.config.trigger == TRIG_RISING || d.config.trigger == TRIG_RISING1) {
      if (firstEdge >= 0 && d.bigData.channelData.buf[chan].data[i] > d.config.trigVolt) newStart = i;
      if (firstEdge < 0 && d.bigData.channelData.buf[chan].data[i] < d.config.trigVolt) firstEdge = i;
    } else { // FALLING
      if (firstEdge >= 0 && d.bigData.channelData.buf[chan].data[i] < d.config.trigVolt) newStart = i;
      if (firstEdge < 0 && d.bigData.channelData.buf[chan].data[i] > d.config.trigVolt) firstEdge = i;
    }
  }
  return newStart;
}
void actDrawSignals() {
  uint16_t newStart = leftStart;
  if (menus.isRedrawFullNeeded()) {
    // massive change on screen
    d.tft.fillScreen(BACKGROUND_COLOR);
  } else if (menus.isFrozen()) {
    if (!menus.isFrozenRedrawNeeded()) return;
    if (d.config.chan_set != CHAN_2) clearFormerData(CHAN_1);
    if (d.config.chan_set != CHAN_1) clearFormerData(CHAN_2);
  } else {
    newStart = newTrigger();
    if(!newStart) return;
    if (d.config.chan_set != CHAN_2) clearFormerData(CHAN_1);
    if (d.config.chan_set != CHAN_1) clearFormerData(CHAN_2);
    leftStart = 0;
    if (d.config.trigger == TRIG_RISING1 || d.config.trigger == TRIG_FALLING1)
      menus.setFreeze();
  }
  // draw current wave
  stats.fpsLatest++; // Testing FPS Counter
  if (d.config.trigger == TRIG_SPECTRUM) {
    adc.drawSpectrogram(); // data is not being erased
  } else {
    uint16_t i, j;
    stats.TFT.Start();
    drawDivisions();
    // display time samples
    uint16_t maxy = 0;
    uint16_t miny = ADC_RESOLUTION;
    uint32_t avgy = 0;
    leftStart = newStart;
    if (!menus.isFrozen() && leftStart >= d.config.trigAlign) leftStart -= d.config.trigAlign;
    for (uint8_t chan = (d.config.chan_set == CHAN_2) ? CHAN_2 : CHAN_1; chan <= d.config.chan_set && chan < CHAN_1n2; chan++) {
      for (i = leftStart; i < leftStart + SCREEN_HORIZONTAL_RESOLUTION; i++) {
        maxy = max(maxy, d.bigData.channelData.buf[chan].data[i]);
        miny = min(miny, d.bigData.channelData.buf[chan].data[i]);
        avgy += d.bigData.channelData.buf[chan].data[i];
      }
      avgy /= SCREEN_HORIZONTAL_RESOLUTION;
      d.bigData.channelData.buf[chan].avgy = avgy;
      d.bigData.channelData.buf[chan].maxy = maxy;
      d.bigData.channelData.buf[chan].miny = miny;
    }
    uint8_t SignalMult = Data_Rates[d.config.time_base].Mult;
    lastSignalZoom = SignalMult;
    for (uint8_t chan = (d.config.chan_set == CHAN_2) ? CHAN_2 : CHAN_1; chan <= d.config.chan_set && chan < CHAN_1n2; chan++) {
      for (i = leftStart, j = 0; j < SCREEN_HORIZONTAL_RESOLUTION; i++, j += SignalMult) {
        uint16_t base = i - leftStart;
        d.bigData.channelData.buf[chan].dataPlot[base] = SCREEN_VERTICAL_RESOLUTION - (20 + (d.bigData.channelData.buf[chan].data[i] * EFFECTIVE_VERTICAL_RESOLUTION / ADC_RESOLUTION));
        d.bigData.channelData.buf[chan].dataPlot[base + 1] = SCREEN_VERTICAL_RESOLUTION - (20 + (d.bigData.channelData.buf[chan].data[i + 1] * EFFECTIVE_VERTICAL_RESOLUTION / ADC_RESOLUTION));
        d.tft.drawLine(j, d.bigData.channelData.buf[chan].dataPlot[base], j + SignalMult, d.bigData.channelData.buf[chan].dataPlot[base + 1], CH_COLORS[chan]);
      }
    }
    stats.TFT.Stop();
  }
  adc.setDataReadyDone();
}

void actSplash() {
  d.tft.fillScreen(BLACK);
  d.tft.setCursor(0, 100);
  d.tft.setTextColor(BLUE);
  d.tft.setTextSize(3);
  d.tft.println("STM32 SimpleScope");
  d.tft.setCursor(0, 150);
  d.tft.setTextSize(2);
  d.tft.println("Orig by GameInstance.com");
  d.tft.println("Extended by Alex");
  d.tft.println("All glory to God!");
  delay(2000);
  d.tft.fillScreen(BACKGROUND_COLOR);
}

void setup() {
  tft_init();
  stats.init();
  menus.init();
  adc.init();
#ifdef TEST_CHANNEL
  testMode.init(TEST_CHANNEL);
#endif
  actSplash();
  stats.uStart = micros();
}

void loop() {
  stats.Loop.Start();
  menus.actInputCheck();
  if (!menus.isFrozen())
    adc.check();
  actDrawSignals();
  if (menus.isRedrawMenuNeeded())
    menus.actDrawTitleBar();
  stats.Loop.Stop();
}
