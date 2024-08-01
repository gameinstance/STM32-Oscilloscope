#ifndef H_ADC_HPP
#define H_ADC_HPP

// All the signal samples must be 32-bit data containing the 16-bit real part followed by the 16-bit imaginary part (in the little Endian order: imaginary_real).
extern "C" { void cr4_fft_1024_stm32(void *pssOUT, void *pssIN, unsigned short Nbin); } // 1024 points only

typedef struct ADCStruct {
  private:
    dma_tube_config tube;
    bool dataReady = false;
  public:
    void init() {
      pinMode(CHANNEL_1, INPUT_ANALOG);
      pinMode(CHANNEL_2, INPUT_ANALOG);
      adc_set_extsel(ADC1, ADC_EXT_EV_SWSTART);   //External trigger Event, ADC1 only!
      adc_calibrate(ADC1);
      adc_calibrate(ADC2);
      tube.tube_nr_xfers = BUFFER_SIZE;
      tube.tube_src = &ADC1->regs->DR;
      tube.tube_req_src = DMA_REQ_SRC_ADC1;
      tube.tube_flags = DMA_CFG_DST_INC | DMA_CFG_CMPLT_IE;
      tube.target_data = 0;
      Timer2.pause();
      Timer2.setCount(BUFFER_SIZE); // run this many times
      Timer2.attachInterrupt(TIMER_CH4, adc_tick);
      Timer2.setPeriod(1000); // in usec
      Timer2.setCompare(TIMER_CH4, 1); // pull data each 1 of period (or overflow???)
      Timer2.setMode(TIMER_CH4, TIMER_OUTPUTCOMPARE);
    }
    void setTube() {
      if (d.config.chan_set == CHAN_1n2) {
        tube.tube_dst_size = DMA_SIZE_32BITS;
        tube.tube_src_size = DMA_SIZE_32BITS;
        tube.tube_dst = d.bigData.channelData.dualADCBuf; // use our 32bit buffer
      } else {
        tube.tube_dst_size = DMA_SIZE_16BITS;
        tube.tube_src_size = DMA_SIZE_16BITS;
        tube.tube_dst = d.bigData.channelData.buf[d.config.chan_set].data;
      }
      ADC2->regs->SQR3 = PIN_MAP[CHANNEL_2].adc_channel;
      if (d.config.chan_set == CHAN_2)
        ADC1->regs->SQR3 = PIN_MAP[CHANNEL_2].adc_channel;
      else
        ADC1->regs->SQR3 = PIN_MAP[CHANNEL_1].adc_channel;
      if (d.config.chan_set == CHAN_1n2)
        ADC1->regs->CR1 |= (6<<16); // enable DUAL channel mode (primary ADC only)
      else
        ADC1->regs->CR1 &= ~(6<<16); // disable DUAL channel mode

      if (d.config.time_base >= DATA_RATE_TIMER_MIN) { // use timer instead of DMA?
        ADC1->regs->CR2 &= ~ADC_CR2_CONT;
        ADC2->regs->CR2 &= ~ADC_CR2_CONT;
        ADC1->regs->CR2 |= ADC_CR2_JEXTSEL_TIM2_TRGO | ADC_CR2_DMA;
        ADC2->regs->CR2 |= ADC_CR2_JEXTSEL_TIM2_TRGO | ADC_CR2_DMA;
        Timer2.pause();
        Timer2.setCount(BUFFER_SIZE); // run this many times
        Timer2.setPeriod(Data_Rates[d.config.time_base].TBPerDivision/40); // in usec
      } else {
        Timer2.pause();
        ADC1->regs->CR2 &= ~ADC_CR2_JEXTSEL_TIM2_TRGO;
        ADC2->regs->CR2 &= ~ADC_CR2_JEXTSEL_TIM2_TRGO;
        ADC1->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA; // Set continuous mode and DMA
        ADC2->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
      }
    }
    void setADC(adc_dev *adc) {
      switch (Data_Rates[d.config.time_base].SampleRate) {
        case 0: adc_set_sample_rate(adc, ADC_SMPR_1_5); break;
        case 1: adc_set_sample_rate(adc, ADC_SMPR_7_5); break;
        case 2: adc_set_sample_rate(adc, ADC_SMPR_13_5); break;
        case 3: adc_set_sample_rate(adc, ADC_SMPR_28_5); break;
        case 4: adc_set_sample_rate(adc, ADC_SMPR_41_5); break;
        case 5: adc_set_sample_rate(adc, ADC_SMPR_55_5); break;
        case 6: adc_set_sample_rate(adc, ADC_SMPR_71_5); break;
        case 7: adc_set_sample_rate(adc, ADC_SMPR_239_5); break;
        default: adc_set_sample_rate(adc, ADC_SMPR_239_5);
      }
      adc_set_reg_seqlen(adc, 1);
    }
    void setADCs() {
      switch (Data_Rates[d.config.time_base].Prescaler) {
        case 0: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_2); break;
        case 1: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_4); break;
        case 2: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_6); break;
        case 3: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_8); break;
        default: rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_8);
      }
      setADC(ADC1);
      setADC(ADC2);
    }

    static void DMA1_CH1_Event() {
      d.vars.dmaActive = false;
      d.vars.dmaFreshData = true;
      //result = dma_get_irq_cause(DMA1,DMA_CH1);
      stats.DMAFill.Stop();
      stats.DMAMiss.Start();
      dma_disable(DMA1, DMA_CH1); // end of DMA trasfer
    }
    static void adc_tick() {
      ADC1->regs->CR2 |= ADC_CR2_SWSTART; // perform one read
    }

    void actAcquisition() {
      setADCs();
      dma_init(DMA1);
      setTube();
      d.vars.dmaActive = true;
      dma_tube_cfg(DMA1, DMA_CH1, &tube);
      dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);
      dma_enable(DMA1, DMA_CH1);                     // enable the DMA channel and start the transfer
      stats.DMAMiss.Stop();
      stats.DMAFill.Start();
      ADC1->regs->CR2 |= ADC_CR2_SWSTART;
      Timer2.resume();
    }
    void check() {
      if (d.vars.dmaActive) return; // still reading data
      if (d.vars.dmaFreshData) {
        if (d.config.chan_set == CHAN_1n2) { // split out the received data into both buffers
          for (int16_t i = 0; i < BUFFER_SIZE; i++) {
            d.bigData.channelData.buf[CHAN_1].data[i] = (uint16_t)(d.bigData.channelData.dualADCBuf[i] & 0xfff);
            d.bigData.channelData.buf[CHAN_2].data[i] = (uint16_t)(d.bigData.channelData.dualADCBuf[i] >> 16) & 0xfff;
          }
        }
        dataReady = true;
        d.vars.dmaFreshData = false;
        if (d.config.trigger == TRIG_SPECTRUM) // TODO: with a little data rearrangement, fft could be done on ch1 or ch2
          fftExecute(d.bigData.channelData.buf[CHAN_1].data);
        return;
      }
      actAcquisition();
    }
    void fftRealToComplex(uint16_t *in, uint32_t *out) {
      for (int i = 0; i < BUFFER_SIZE; i++) out[i] = in[i];// * 8;
    }
    void fftExecute(uint16_t *inData) {
      stats.FFT.Start();
      fftRealToComplex(inData, d.bigData.fft.data32); // data format conversion
      cr4_fft_1024_stm32(d.bigData.fft.fftData, d.bigData.fft.data32, BUFFER_SIZE); // FFT computation
      fftInplaceMagnitude(d.bigData.fft.fftData);
      stats.FFT.Stop();
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
        // TODO: Try bitshift instead
        res /= 2;
        one /= 4;
      }
      return (uint16_t)res;
    }
    void fftInplaceMagnitude(uint32_t *target) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        int16_t real = target[i] & 0xFFFF;
        int16_t imag = target[i] >> 16;
        //    target[i] = 10 * log10(real*real + imag*imag);
        uint32_t magnitude = asqrt(real * real + imag * imag);
        target[i] = magnitude;
      }
    }
    void drawSpectrogram() {
      stats.TFT.Start();
      // display spectrum takes ~16ms now to draw
      uint16_t baseY = SCREEN_VERTICAL_RESOLUTION - FFT_METRIC_HEIGHT - MENUBAR_HEIGHT;
      lastSignalZoom = 1;
      uint16_t max_y = 0, max_x = 0;
      uint16_t i_0, i_1;
      bool hit_max = false;
      for (uint16_t i = 1; i < BUFFER_SIZE / 2; i++) {
        if (d.bigData.fft.fftData[i] > max_y) {
          max_y = d.bigData.fft.fftData[i];
          max_x = i;
        }
      }
      max_y = max(max_y, EFFECTIVE_VERTICAL_RESOLUTION);
      d.tft.setTextColor(WHITE);
      d.tft.setTextSize(1);
      for (uint16_t i = 1; i < SCREEN_HORIZONTAL_RESOLUTION; i++) {
        i_0 = (i * BUFFER_SIZE / SCREEN_HORIZONTAL_RESOLUTION / 2);
        i_1 = ((i + 1) * BUFFER_SIZE / SCREEN_HORIZONTAL_RESOLUTION / 2);
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
        d.bigData.channelData.buf[0].dataPlot[i] = baseY - (d.bigData.fft.fftData[i_0] * (EFFECTIVE_VERTICAL_RESOLUTION - FFT_METRIC_HEIGHT*2) / max_y);
        d.tft.drawLine(i, d.bigData.channelData.buf[0].dataPlot[i], i + 1,
          baseY - (d.bigData.fft.fftData[i_1] * (EFFECTIVE_VERTICAL_RESOLUTION - FFT_METRIC_HEIGHT*2) / max_y),
          CH_COLORS[CHAN_1]);
        if (menus.isRedrawMenuNeeded() && i % DIVISION_SIZE == 0) {
          float freq = ((float)i * (float)Data_Rates[d.config.time_base].Spectrum) / (float)SCREEN_HORIZONTAL_RESOLUTION / 2.0;
          if (d.config.time_base >= DATA_RATE_TIMER_MIN)
            freq *= 1000; // cvt to Hz
          d.tft.setCursor(i - (freq > 100 ? 8 : 5) - (freq > (int)freq ? 4 : 0), baseY+1);
          d.tft.print(freq, (d.config.time_base >= DATA_RATE_TIMER_MIN) ? 0 : 1);
        }
      }
      // clear previous stats
      d.tft.fillRect(7, MENUBAR_HEIGHT, 150, MENUBAR_HEIGHT+FFT_METRIC_HEIGHT, BACKGROUND_COLOR);
      d.tft.setCursor(8, MENUBAR_HEIGHT);
      d.tft.setTextColor(WHITE);
      d.tft.setTextSize(1);
      float freq = (float)max_x * (float)Data_Rates[d.config.time_base].Spectrum / (float)BUFFER_SIZE;
      // derive spectrum from 1kHz signal (1.0 / ...)
      //freq = 1.0 / ((float)max_x / (float)BUFFER_SIZE);
      if (d.config.time_base >= DATA_RATE_TIMER_MIN)
        freq *= 1000.0;
      d.tft.print("F: ");
      d.tft.print(freq); // 43304
      char buf[20];
      sprintf(buf, "%cHz ", (d.config.time_base < DATA_RATE_TIMER_MIN) ? 'k' : ' ');
      d.tft.print(buf);
      d.tft.print(20.0 * log10(max_y));
      d.tft.print( "dB");
      stats.TFT.Stop();
    }
    bool isActive() { return d.vars.dmaActive; }
    bool isDataReady() { return (dataReady); }
    void setDataReadyDone() { dataReady = false; }
} myADC;
myADC adc;

#endif
