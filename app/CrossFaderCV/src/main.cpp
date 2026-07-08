/*!
 * CrossFaderCV
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

/*
# CrossFaderCV

CrossFaderCVは、Equal PowerタイプのクロスフェーダCVを出力するファームウェアです。  
実際にクロスフェードさせるには別途リニアVCAが二基必要です。  

## 機能概要

- 入力
  - `CV` パンの位置制御CV
  - `POT` パンの位置
- 出力
  - `OUT1` カーブA
  - `OUT2` カーブB
  - `RGB LED` パンの位置を表示
*/

#include <Arduino.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <EEPROM.h>
#include "lib/Button.hpp"
#include "lib/SmoothAnalogRead.hpp"
#include "lib/RotaryEncoder.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EepRomConfigIO.hpp"
// #include "lib/Mcp4922SwSpi.hpp"
#include "lib/Mcp4922HwSpi.hpp"
#include "lib/pwm_wrapper.h"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"
#include "CrossFader.hpp"

// 標準インターフェース
static uint interruptSliceNum;
static SmoothAnalogRead cvIn;
static SmoothAnalogRead pot;
static RGBLEDPWMControl rgbLedControl;
static Mcp4922HwSpi dac;

// UIほか
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::GREEN;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
CrossFader crossFader;
uint16_t crossValue = DAC_RESO >> 1;

//////////////////////////////////////////

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    dac.out1(crossFader.gainA(crossValue));
    dac.out2(crossFader.gainB(crossValue));
}

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    set_sys_clock_hz(CPU_CLOCK, true);
    pinMode(23, OUTPUT);
    gpio_put(23, HIGH);

    cvIn.init(CV1);
    pot.init(POT1);
    dac.init(SPI_MOSI, SPI_SCK, SPI_CS);

    rgbLedControl.init(10000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLACK);
    rgbLedControl.ignoreMenuColor(true);
    rgbLedControl.setWave(MiniOsc::Wave::SQU);
    rgbLedControl.resetFreq();

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();

    crossFader.init();

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();

    crossValue = constrain(potValue + cvInValue, 0, ADC_RESO - 1);
    rgbLedControl.setRainbowLevel(crossValue, 0, ADC_RESO - 1);

    rgbLedControl.update();
    rgbLedControl.process();
    tight_loop_contents();
    sleep_us(100);
}

void setup1()
{
    // core0のsetupを終わらせてcore1開始したいので適当いれておく
    sleep_ms(500);
}

void loop1()
{
    tight_loop_contents();
    sleep_ms(10);
}
