#ifndef H_MENUS_HPP
#define H_MENUS_HPP

#if SD_ENABLED
void export_to_sd();
#endif

typedef struct MenuItemStruct {
  const uint8_t widthPx: 7;
  const uint8_t selectable:1;
  const char *text;
} MenuItem;

const bool MENU_TOP = false;
const bool MENU_BOT = true;

enum TOP_MENU : uint8_t {
  MENU_CH = 0,
  MENU_TRIGVOLT,
  MENU_TRIGMODE,
  MENU_TIMEBASE,
  MENU_FREEZE,
  MENU_TRIGXALIGN,
  MENU_TRIGYALIGN,
  MENU_PADDING,
  MENU_MORE,
  MENU_SIZE
};
const static MenuItem MenuItems[MENU_SIZE] = {
  { 30, true, "CH" },
  { 36, true, "TrigV" },
  { 30, true, "Trig" },
  { 52, true, "TimeBase" },
  { 30, true, "Stop" },
  { 41, true, "XAlign" },
  { 40, true, "YAlign" },
  { 20, false, " " }, // padding to push more to top right
  { 30, true, "More" },
};

enum CHANNEL_MENU : uint8_t {
  CHANNEL_1_YMin = 0,
  CHANNEL_1_YMax,
  CHANNEL_1_YAvg,
  CHANNEL_2_YMin,
  CHANNEL_2_YMax,
  CHANNEL_2_YAvg,
  CHANNEL_MENU_SIZE
};
const static MenuItem ChannelItems[CHANNEL_MENU_SIZE] {
  { 45, false, "MinY-1" },
  { 45, false, "MaxY-1" },
  { 45, false, "AvgY-1" },
  { 45, false, "MinY-2" },
  { 45, false, "MaxY-2" },
  { 45, false, "AvgY-2" },
};

enum STATS_MENU : uint8_t {
  STAT_MENU_LOOP = 0,
  STAT_MENU_FILL,
  STAT_MENU_MENU,
  STAT_MENU_TFT,
  STAT_MENU_PCT_FILL,
  STAT_MENU_PCT_MENU,
  STAT_MENU_PCT_TFT,
  STAT_MENU_PCT_UNK,
  STAT_MENU_FPS,
  STATS_ITEMS_SIZE
};
const static MenuItem StatsItems[STATS_ITEMS_SIZE] = {
  { 36, false, "Loop" },
  { 40, false, "Fill" },
  { 40, false, "Menu" },
  { 40, false, "TFT"  },
  { 30, false, "\%Fil"},
  { 30, false, "\%Mnu"},
  { 30, false, "\%TFT"},
  { 30, false, "\%Unk"},
  { 34, false, "FPS "},
};

enum MENU_MORE_ITEMS : uint8_t {
  MENU_MORE_CLOSED = 0,
  MENU_MORE_STATS,
  MENU_MORE_CHAN,
  MENU_MORE_SETTINGS,
#ifdef TEST_CHANNEL
  MENU_MORE_TESTMODE,
#endif
  MENU_MORE_SIZE
};
const static char *Menu_More_Items[MENU_MORE_SIZE] = { "-", "Stat", "Chan", "Cfg"
#ifdef TEST_CHANNEL
, "Test"
#endif
};
#ifdef TEST_CHANNEL
enum MENU_TEST_ITEMS : uint8_t {
  TEST_ITEMS_PERCENT,
  TEST_ITEMS_PERIOD,
  TEST_ITEMS_SIZE
};
const static MenuItem TestItems[TEST_ITEMS_SIZE] = {
  { 55, true, "Percent" },
  { 45, true, "Period" },
};
#endif
const static MenuItem ClosedItems[] = {};

float getVCC() {
  float vcc;
  ADC2->regs->CR2 |= ADC_CR2_TSVREFE;
  ADC2->regs->SQR3 = 17; // vRefInt
  adc_set_sample_rate(ADC2, ADC_SMPR_239_5);
  ADC2->regs->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC2->regs->SR & ADC_SR_EOC)) ; // wait for reading
  uint16_t vRef = ADC2->regs->DR;
  ADC2->regs->CR2 &= ~ADC_CR2_TSVREFE;
  vcc = 1.2 * 4095.0 / 3.0 / (float)vRef;
  return vcc;
}

// Oddly this is smaller as a struct than a struct pointer
typedef struct MenusStruct {
  private:
    const MenuItem *topMenu;
    const MenuItem *botMenu;
    uint8_t menuSelection = MENU_TRIGVOLT;
    uint8_t menuMore = MENU_MORE_CHAN;
    uint8_t topMenuSize:4;
    uint8_t botMenuSize:4;
    bool redrawMenu:1;
    bool redrawScreen:1;
    bool redrawFreeze:1;
    bool menuActive = MENU_TOP;
    uint8_t freezeMode:3;
  public:
    void init() {
      encoder.init();
      redrawMenu = false;
      redrawFreeze = false;
      redrawScreen = true;
      topMenu = MenuItems;
      botMenu = ChannelItems;
      topMenuSize = MENU_SIZE;
      setBotMenuSize();
    }
    void setBotMenuSize() {
      switch (menuMore) {
        case MENU_MORE_CLOSED:
          botMenu = ClosedItems;
          botMenuSize = 0; break;
        case MENU_MORE_STATS:
          botMenu = StatsItems;
          botMenuSize = STATS_ITEMS_SIZE; break;
        case MENU_MORE_CHAN:
          botMenu = ChannelItems;
          botMenuSize = CHANNEL_MENU_SIZE; break;
        case MENU_MORE_SETTINGS: botMenuSize = 0; break; // TODO: No settings yet
#ifdef TEST_CHANNEL
        case MENU_MORE_TESTMODE:
          botMenu = TestItems;
          botMenuSize = TEST_ITEMS_SIZE; break;
#endif
      }
    }
    uint8_t getNumSelectable(bool which) { // returns number of selectable items within the chosen menu
      uint8_t items = 0;
      const MenuItem *m = (which == MENU_TOP) ? topMenu : botMenu;
      int8_t i = (which == MENU_TOP) ? topMenuSize : botMenuSize;
      for (;i;) { if (m[--i].selectable) items++; }
      return items;
    }
    uint8_t getNextSelection(bool findFirst = false) {
      uint8_t sz = (menuActive == MENU_TOP) ? topMenuSize : botMenuSize;
      const MenuItem *m = (menuActive == MENU_TOP) ? topMenu : botMenu;
      uint8_t i = findFirst ? 0 : (menuSelection+1);
      for (; i < sz; i++) { if (m[i].selectable) return i; }
      return 0; // returns 0 if no next found
    }
    void actInputCheckMain() {
      switch (menuSelection) {
        case MENU_CH:
          if (d.config.trigger != TRIG_SPECTRUM) {
            encoder.addEncValue8(&d.config.chan_set, CHAN_MAX);
            if (d.config.chan_set != CHAN_2)
              d.clearAllData(CHAN_1);
            if (d.config.chan_set != CHAN_1)
              d.clearAllData(CHAN_2);
            redrawScreen = true;
          } else encoder.clearCounter();
          break;
        case MENU_TIMEBASE:
          encoder.addEncValue8(&d.config.time_base, MAX_DATA_RATES);
          redrawScreen = true;
          break;
        case MENU_TRIGMODE:
          {
            bool more = encoder.getCount() > 0 ? 1 : 0;
            encoder.addEncValue8(&d.config.trigger, TRIG_MAX);
            if (d.config.trigger == TRIG_SPECTRUM && d.config.chan_set > CHAN_1)
              d.config.trigger = more ? TRIG_RISING : TRIG_NONE;
            if (freezeMode) freezeMode = 0;
            redrawScreen = true;
          }
          break;
        case MENU_TRIGVOLT:
          encoder.addEncValue16(&d.config.trigVolt, ADC_RESOLUTION+TRIGGER_LIMIT_MARGIN, TRIGGER_LIMIT_MARGIN*2);
          if (d.config.trigVolt <= TRIGGER_LIMIT_MARGIN) d.config.trigVolt = TRIGGER_LIMIT_MARGIN; // can be neither 0 nor maximum value
          else if (d.config.trigVolt >= ADC_RESOLUTION - TRIGGER_LIMIT_MARGIN) d.config.trigVolt = (ADC_RESOLUTION - TRIGGER_LIMIT_MARGIN);
          redrawScreen = true;
          break;
        case MENU_TRIGXALIGN:
          encoder.addEncValue16(&d.config.trigAlign, TRIGGER_ALIGN_MAX, TRIGGER_ALIGN_STEP);
          break;
        case MENU_FREEZE:
          encoder.addEncValue16(&leftStart, TRIGGER_START_LIMIT);
          redrawFreeze = true;
          break;
        case MENU_MORE:
          encoder.addEncValue8(&menuMore, MENU_MORE_SIZE);
          setBotMenuSize();
          break;
        default:
          encoder.clearDepth();
          break;
      }
    }
    void actInputCheck() {
      if (!encoder.hasChanged()) return; // skip when no buttons pressed
      redrawMenu = true;
      if (encoder.wasClicked()) { // either enter a menu or exit it
        if (menuSelection == MENU_FREEZE) { // if freeze menu is selected, enable or disable freeze
          freezeMode = encoder.getDepth() ? 3 : 0;
          if (d.config.trigger == TRIG_RISING1 || d.config.trigger == TRIG_FALLING1) d.config.trigger--; // disable single trigger mode on exiting freeze
        }
        return; // processed the click
      } else if (!encoder.getDepth()) { // not in a menu yet
        int8_t encValue = encoder.getEncValue();
        uint8_t sizes[2] = {getNumSelectable(MENU_TOP), getNumSelectable(MENU_BOT)};
        uint8_t totalSize = sizes[0] + sizes[1];
        uint8_t nextValue;
        while (encValue < 0) { encValue += totalSize; }
        encValue = encValue % totalSize;
        for (; encValue; ) { // encValue is now a positive increment from the current position
          nextValue = getNextSelection();
          if (!nextValue) { // this row is finished
            if (sizes[!menuActive]) // we can switch to the other row
              menuActive = !menuActive;
            // else we just repeat this row
            nextValue = getNextSelection(true);
          }
          menuSelection = nextValue;
          encValue--;
        }
      } else { // process the menus
        if (!menuActive) { // top menu
          actInputCheckMain();
        } else { // one of the bottom menus
          switch (menuMore) {
            //case MENU_MORE_CLOSED:  break;
            //case MENU_MORE_STATS:   break;
            //case MENU_MORE_CHAN:    break;
            //case MENU_MORE_SETTINGS: break; // TODO: No settings yet
#ifdef TEST_CHANNEL
            case MENU_MORE_TESTMODE: actInputCheckTest(); break;
#endif
            default:
              encoder.clearDepth(); // clear the depth if we somehow get here
              break;
          }
        }
      }
    }
#ifdef TEST_CHANNEL
    void actInputCheckTest() {
      switch (menuSelection) {
        case TEST_ITEMS_PERCENT:
          encoder.addEncValue8(&d.config.testMenuPctSelection, testMenuPctSize);
          break;
        case TEST_ITEMS_PERIOD:
          encoder.addEncValue8(&d.config.testMenuPeriodSelection, testMenuPeriodSize);
          break;
      }
      testMode.setPeriodAndPct();
    }
    void actDrawTestBar(uint8_t i) {
      uint32_t value = 0;
      switch (i) {
        case TEST_ITEMS_PERIOD:
          value = testMode.getPeriod();
          break;
        case TEST_ITEMS_PERCENT:
          value = testMode.getPercent();
          break;
      }
      d.tft.print(value);
    }
#endif
    void actDrawChannelBar(uint8_t i) {
      uint16_t value = 0;
      switch(i) {
        case CHANNEL_1_YAvg:
          value = d.bigData.channelData.buf[CHAN_1].avgy;
          break;
        case CHANNEL_1_YMax:
          value = d.bigData.channelData.buf[CHAN_1].maxy;
          break;
        case CHANNEL_1_YMin:
          value = d.bigData.channelData.buf[CHAN_1].miny;
          break;
        case CHANNEL_2_YAvg:
          value = d.bigData.channelData.buf[CHAN_2].avgy;
          break;
        case CHANNEL_2_YMax:
          value = d.bigData.channelData.buf[CHAN_2].maxy;
          break;
        case CHANNEL_2_YMin:
          value = d.bigData.channelData.buf[CHAN_2].miny;
          break;
      }
      d.tft.print(value * VCC_3_3 / ADC_RESOLUTION);
    }
    void actDrawStatsBar(uint8_t i) {
      uint32_t totalTime = millis();
      switch(i) { // add data for chosen item
        case STAT_MENU_LOOP:
          d.tft.print(stats.Loop.Last);
          break;
        case STAT_MENU_FILL:
          d.tft.print(stats.DMAFill.Last);
          break;
        case STAT_MENU_MENU:
          d.tft.print(stats.Menu.Last);
          break;
        case STAT_MENU_TFT:
          d.tft.print(stats.TFT.Last);
          break;
        case STAT_MENU_PCT_FILL:
          d.tft.print(stats.DMAFill.Total * 100 / totalTime);
          break;
        case STAT_MENU_PCT_MENU:
          d.tft.print(stats.Menu.Total * 100 / totalTime);
          break;
        case STAT_MENU_PCT_TFT:
          d.tft.print(stats.TFT.Total * 100 / totalTime);
          break;
        case STAT_MENU_PCT_UNK:
          //d.tft.print(1000-(tot / millis()));
          d.tft.print(100 - ((stats.TFT.Total + stats.Menu.Total + stats.DMAFill.Total + stats.FFT.Total) * 100 / totalTime));
          break;
        case STAT_MENU_FPS:
          d.tft.print(stats.fpsLatest / (totalTime / 1000));
          break;
      }
    }
    void actDrawMainBar(uint8_t i) {
      char buf[20];
      switch(i) { // add data for chosen item
        case MENU_CH:
          d.tft.print(CHANNEL_OPTS[d.config.chan_set]);
          break;
        case MENU_TRIGVOLT:
          d.tft.print((float)d.config.trigVolt / (float)ADC_RESOLUTION * VCC_3_3);
          break;
        case MENU_TRIGMODE:
          d.tft.print(TRIG_MODES[d.config.trigger]);
          break;
        case MENU_TIMEBASE:
          if (d.config.trigger == TRIG_SPECTRUM) {
            sprintf(buf, "%lukHz", Data_Rates[d.config.time_base].Spectrum);
          } else {
            sprintf(buf, "%luus", Data_Rates[d.config.time_base].TBPerDivision);
          }
          d.tft.print(buf);
          break;
        case MENU_TRIGXALIGN:
          sprintf(buf, "%d%%", d.config.trigAlign * 100 / TRIGGER_ALIGN_MAX);
          d.tft.print(buf);
          break;
        case MENU_TRIGYALIGN:
          // TODO: Upcoming ...
          //d.tft.print((float)stats.DMAFill.Last / 1024.0 * DIVISION_SIZE); // derived TimeBase
          break;
        case MENU_FREEZE:
          if (freezeMode) {
            sprintf(buf,"%d%%", leftStart * 100 / TRIGGER_START_LIMIT);
            d.tft.print(buf);
          }
          break;
        case MENU_MORE:
          d.tft.print(Menu_More_Items[menuMore]);
          break;
      }
    }
    void doDrawMenu(bool which) {
      const MenuItem *m = (which == MENU_TOP) ? topMenu : botMenu;
      // wipe this line
      d.tft.setTextSize(1);
      uint16_t currentX = MENUBAR_SPACE;
      uint8_t i, iMax = (which == MENU_TOP) ? topMenuSize : botMenuSize;
      int encDepth = getMenuDepth();
      for (i = 0; i < iMax; i++ ) {
        d.tft.setCursor(currentX, MenuBars[which].PriY);
        if (which == menuActive && i == menuSelection && !encDepth) { // check for active menus
          d.tft.setTextColor(COL_MENU_FG_ACTIVE);
        } else { // This menu is not active
          if (menuActive == MENU_TOP && which ==menuActive && menuSelection == MENU_FREEZE && i == menuSelection)
            d.tft.setTextColor(RED);
          else
            d.tft.setTextColor(COL_MENU_FG_INACTIVE);
        }
        d.tft.print(m[i].text);
        d.tft.setCursor(currentX, MenuBars[which].SecY);
        currentX += m[i].widthPx;
        if (which == menuActive && i == menuSelection && encDepth)
          d.tft.setTextColor(COL_MENU_FG_ACTIVE);
        else
          d.tft.setTextColor(COL_MENU_FG_INACTIVE);
        if (which == MENU_TOP)
          actDrawMainBar(i);
        else
          switch (menuMore) {
            case MENU_MORE_CHAN:
              actDrawChannelBar(i);
              break;
            case MENU_MORE_STATS:
              actDrawStatsBar(i);
              break;
            case MENU_MORE_CLOSED:
              // TODO: resize main menu
              break;
            case MENU_MORE_SETTINGS:
              // TODO: add settings menu
              break;
    #ifdef TEST_CHANNEL
            case MENU_MORE_TESTMODE:
              actDrawTestBar(i);
              break;
    #endif
          }
      }
    }
    void actDrawTitleBar() {
      stats.Menu.Start();
      redrawMenu = false;
      d.tft.fillRect(0, 0, SCREEN_HORIZONTAL_RESOLUTION, MENUBAR_HEIGHT, COL_MENU_BACKGROUND);
      doDrawMenu(MENU_TOP);
      //if (menuMore != MENU_MORE_CLOSED) { // TODO: Enable after resize is finished
      // wipe bottom menubar
      d.tft.fillRect(0, SCREEN_VERTICAL_RESOLUTION-MENUBAR_HEIGHT, SCREEN_HORIZONTAL_RESOLUTION, SCREEN_VERTICAL_RESOLUTION, COL_MENU_BACKGROUND);
      //}
      doDrawMenu(MENU_BOT);
      if (freezeMode == 3) {
        freezeMode = 1;
#if SD_ENABLED
        export_to_sd();
#endif
        redrawScreen = true;
      }
      stats.Menu.Stop();
    }
    uint8_t getMenuDepth() { return encoder.getDepth(); }
    bool isRedrawFullNeeded() { if (!redrawScreen) return false; redrawScreen = false; redrawMenu = true; return true; }
    bool isRedrawMenuNeeded() { return redrawMenu; }
    bool isFrozen() { return freezeMode; }
    bool isFrozenRedrawNeeded() { if (!redrawFreeze) return false; redrawFreeze = false; return true; }
    void setFreeze() { freezeMode = 3; redrawMenu = true; menuSelection = MENU_FREEZE; encoder.setDepth(1); }
    bool isInTriggerMenu() { return (encoder.getDepth() && menuSelection == MENU_TRIGMODE); }
} Menus;
Menus menus;

#endif
