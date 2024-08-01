#ifndef H_ENCODER_HPP
#define H_ENCODER_HPP

#define ENC_BUTTON PB7
#define ENC_CLK PB8
#define ENC_DATA PB9

#define ENC_CLICK_THRESHOLD_MS 100 // further debouncing
#define ENC_SPIN_FAST_MS 60 // *fast* spin detection time
#define ENC_SPIN_FAST_MULT 8 // a spin = this many turns when fast

// The interrupt routines have to use fixed named objects
typedef struct EncoderBaseStruct {
    volatile uint32_t ClickTime = 0; // last click interrupt
    volatile uint32_t SpinTime = 0; // last spin interrupt
    uint8_t Depth = 0;
    const uint8_t pinDa = ENC_DATA;
    const uint8_t pinCk = ENC_CLK;
    const uint8_t pinBt = ENC_BUTTON;
    volatile int8_t Count = 0;
    volatile bool Click = false;
} EncoderBase;
EncoderBase encBase;

struct EncoderStruct {
  void init() {
    pinMode(encBase.pinCk, INPUT);
    pinMode(encBase.pinDa, INPUT);
    pinMode(encBase.pinBt, INPUT);
    attachInterrupt(digitalPinToInterrupt(encBase.pinCk), encoderClkInt, FALLING);
    attachInterrupt(digitalPinToInterrupt(encBase.pinBt), encoderPushInt, FALLING);
  }
  static void encoderClkInt() {
    uint32_t now = millis();
    uint8_t add = 1;
    if ((now - encBase.SpinTime) < ENC_SPIN_FAST_MS) add = ENC_SPIN_FAST_MULT;
    encBase.SpinTime = now;
    if (digitalRead(encBase.pinDa)) {
      encBase.Count+=add;
    } else {
      encBase.Count-=add;
    }
  }
  static void encoderPushInt() {
    uint32_t now = millis();
    if ((now - encBase.ClickTime) < ENC_CLICK_THRESHOLD_MS) return;
    encBase.ClickTime = now;
    encBase.Click = true;
    encBase.SpinTime = now - (2 * ENC_SPIN_FAST_MS); // prevent from detecting fast immediately after a click
  }
  int8_t getEncValue() {
    int8_t retval = encBase.Count;
    encBase.Count -= retval;
    return retval;
  }
  void addEncValue8(uint8_t *val, const uint8_t limit, const uint8_t mult = 1) {
    int8_t change = getEncValue() * mult;
    while (change < 0) change += limit;
    *val += change;
    *val = *val % limit;
  }
  void addEncValue16(uint16_t *val, const uint16_t limit, const uint16_t mult = 1) {
    int16_t change = getEncValue() * mult;
    while (change < 0) change += limit;
    *val += change;
    *val = *val % limit;
  }
  void clearCounter() { encBase.Count = 0; }
  void clearDepth() { encBase.Depth = 0; }
  bool hasChanged() { return (encBase.Count!=0 || encBase.Click); } // skip when no buttons pressed
  uint8_t getDepth() { return encBase.Depth; }
  void setDepth(uint8_t newDepth) { encBase.Depth = newDepth; }
  int8_t getCount() { return encBase.Count; }
  void setCount(int8_t newValue) { encBase.Count = newValue; }
  bool wasClicked() {
    if (!encBase.Click) return false;
    if (encBase.Depth)
      encBase.Depth--;
    else
      encBase.Depth++;
    encBase.Click = false;
    encBase.Count = 0; // clear counter since we had a click event
    return true;
  }
};
struct EncoderStruct encoder;
#endif
