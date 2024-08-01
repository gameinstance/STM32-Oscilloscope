
/* ============================== SDCARD BEGIN ============================== */
#if SD_ENABLED
static const uint8_t SD_CHIP_SELECT = PB12;
#include <SdFat.h>
static SdFat sd;
static SdFile file;

void export_to_sd() {
  d.tft.setCursor(170, 20);
  d.tft.setTextColor(WHITE);
  d.tft.setTextSize(1);
  d.tft.print("Writing to SD ...");
  d.tft.setCursor(170, 20);
  if (!sd.begin(SD_CHIP_SELECT)){ //, SD_SCK_HZ(F_CPU/4))) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("No SD card detected");
    delay(500);
    return;
  }
  uint8_t index;
  if (!sd.exists("DSO")) {
    // no pre-exising folder structure
    if (!sd.mkdir("DSO")) {
      d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      d.tft.print("Can't create folder");
      return;
    }
  }
  if (!sd.exists("DSO/data.idx")) {
    // no index file
    index = 1;
    if (!file.open("DSO/data.idx", O_CREAT | O_WRITE)) {
      d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      d.tft.print("Can't create idx file");
      return;
    }
    file.write(index);
    if (!file.sync() || file.getWriteError()) {
      d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      d.tft.print("Idx file write error");
      return;
    }
    file.close();
  } else {
    if (!file.open("DSO/data.idx", O_READ)) {
      d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      d.tft.print("Can't open idx file");
      return;
    }
    if (!file.read(&index, 1)) {
      d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      d.tft.print("Can't read idx file");
      return;
    }
    if (!file.sync() || file.getWriteError()) {
      d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
      d.tft.print("File write error");
      return;
    }
    file.close();
  }
  String s = "DSO/Exp";
  s += index;
  s += ".dat";
  if (!file.open(s.c_str(), O_CREAT | O_WRITE | O_EXCL)) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("Can't data create file");
    return;
  }
  file.println("Time series");
  for (uint16_t i = 0; i < BUFFER_SIZE; i ++) {
    file.print(d.bigData.channelData.buf[CHAN_1].data[i], DEC);file.print(", ");
  }
  file.println(" ");
  file.print("Fs: ");file.print(Data_Rates[d.config.time_base].Spectrum);file.println("kHz");
  file.println("Spectrum");
  for (uint16_t i = 0; i < BUFFER_SIZE/2; i++) {
    file.print(d.bigData.fft.fftData[i], DEC);file.print(", ");
  }
  file.println(" ");
  if (!file.sync() || file.getWriteError()) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("File write error");
    return;
  }
  file.close();
  s += ".img";
  if (!file.open(s.c_str(), O_CREAT | O_WRITE | O_EXCL)) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("Can't image create file");
    return;
  }
  file.println("IMX");
  for (uint16_t i = 0; i < BUFFER_SIZE; i ++) {
    file.print(d.bigData.channelData.buf[CHAN_1].data[i], DEC);file.print(", ");
  }
  file.println(" ");
  file.print("Fs: ");file.print(Data_Rates[d.config.time_base].Spectrum);file.println("kHz");
  file.println("Spectrum");
  for (uint16_t i = 0; i < BUFFER_SIZE/2; i ++) {
    file.print(d.bigData.fft.fftData[i], DEC);file.print(", ");
  }
  file.println(" ");
  if (!file.sync() || file.getWriteError()) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("File write error");
    return;
  }
  file.close();
  index ++;
  if (!file.open("DSO/data.idx", O_CREAT | O_WRITE)) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("Can't create idx file");
    return;
  }
  file.write(index);
  if (!file.sync() || file.getWriteError()) {
    d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
    d.tft.print("Idx file write error");
    return;
  }
  file.close();
  d.tft.fillRect(169, 19, 150, 9, BACKGROUND_COLOR);
  d.tft.print("File write success");
  delay(2000);
  d.tft.fillRect(170, 19, 150, 9, BACKGROUND_COLOR);
}
#endif
/* ============================== SDCARD END ============================== */
