#ifndef H_TESTMODE_HPP
#define H_TESTMODE_HPP
/*
 * Since the F103 has no DAC, we can only generate PWM - Square waves - without additional hardware.
 */
const uint8_t testMenuPeriodSize = 5;
const uint8_t testMenuPctSize = 9;
typedef struct TestModeStruct {
  public:
    void init(const uint8_t usePin, const uint32_t usePeriod = 0, const uint8_t usePercent = 0) {
      Timer3.pause();
      Timer3.setPrescaleFactor(72); // 1 µs resolution, should match CPU frequency (x/1e6)
      pinMode(usePin, PWM);
      setPeriodAndPct(usePeriod, usePercent, true);
    }
    void setPeriodAndPct(const uint32_t usePeriod, uint8_t usePercent, bool kickTimer = true) {
      uint32_t usecs = usePeriod ?: getPeriod();
      uint8_t percentHigh = usePercent ? (usePercent % 100) : getPercent();
      if (kickTimer)
        Timer3.pause();
      Timer3.setOverflow(usecs);
      Timer3.setCompare(TIMER_CH4, usecs * percentHigh / 100);
      if (kickTimer) {
        Timer3.refresh();
        Timer3.resume();
      }
    }
    void setPeriodAndPct() { setPeriodAndPct(getPeriod(), getPercent()); }
    uint32_t getPeriod() { return testMenuPeriodChoices[d.config.testMenuPeriodSelection]*10; }
    uint8_t getPercent() { return testMenuPctChoices[d.config.testMenuPctSelection]; }
    const uint16_t testMenuPeriodChoices[testMenuPeriodSize] = { 1, 10, 100, 1000, 5000 }; // values listed are 1/10th of values used in uSec : e.g. 1 = 10uSec
    const uint8_t testMenuPctChoices[testMenuPctSize] = { 10, 20, 30, 40, 50, 60, 70, 80, 90 };
} TestMode;
TestMode testMode;
#endif
