/*!
 * Slope Generator
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <numeric>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <EEPROM.h>
#include "lib/Button.hpp"
#include "lib/SmoothAnalogRead.hpp"
#include "lib/RotaryEncoder.hpp"
// #include "lib/ADCErrorCorrection.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EepRomConfigIO.hpp"
#include "lib/Mcp4922SwSpi.hpp"
// #include "lib/Mcp4922HwSpi.hpp"
#include "lib/pwm_wrapper.h"
#include "lib/ValueLock.hpp"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"
#include "Helper.h"

#include "SlopeGenerator.hpp"

/*
# SlopeGenerator

SlopeGeneratorは西海岸シンセシス系のエンベロープジェネレータ系ファームウェアです。  
Cycle機能、Time入力、EOC出力を備え、Makenoise系の動作を簡易的にシミュレーションしています。  

## 機能概要

- 入力
  - `IN1` トリガー入力
  - `CV` Time入力
  - `POT` カーブ変化：`LOG~LINEAR~EXP`
  - `RE` 時間調整
- 出力
  - `OUT1` Slope出力
  - `OUT2` EOC出力
  - `RGB LED` Slope出力状態表示、Rise、Fall選択表示

## 使い方

- Rise、Fall設定
  - `RE押し込み` Cycleモード(LFOモード)のON/OFF
  - `Aボタン` Riseの設定を選択
  - `Bボタン` Fallの設定を選択
  - `RE長押し` トリガー/ゲートモードの切り替え
  - `RE操作` Rise、Fall時間調整
*/

enum SettingMenu
{
    SEL_RISE = 0,
    SEL_FALL,
    SEL_MAX = SEL_FALL,
};

enum ButtonCondition
{
    // 各ボタンの押下状態と各ボタンの組み合わせ
    // Button State: 0:None 1:Button down 2:Button up 3:Holding 4:Holded
    // U: button up
    // D: button down
    // H: holging
    // L: holded (leaved)
    // 0xMABR (Mode, A, B, RE(RotaryEncoder) button)
    NONE = 0x0000,
    UA = 0x0200,
    UB = 0x0020,
    URE = 0x0002,
    LRE = 0x0004,
    HA = 0x0300,
    HB = 0x0030,
    HA_UB = 0x0320,
    HA_RE = 0x0302,
};

// 標準インターフェース
static uint interruptSliceNum;
static RotaryEncoder enc;
static Button buttons[3];
// static SmoothAnalogRead in2;
static SmoothAnalogRead cvIn;
static SmoothAnalogRead pot;
static RGBLEDPWMControl rgbLedControl;
// static ADCErrorCorrection adcErrorCorrection;
static Mcp4922SwSpi dac;
static ValueLock potLock;

// gpio割り込み
static volatile bool in1EdgeLatch = false;

// UIほか
static SettingMenu settingMenu = SettingMenu::SEL_RISE;
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::GREEN;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
static SlopeGenerator slope;
static const uint16_t bias = DAC_RESO >> 1;
// timeCVはRise/Fallの設定値を1/10~10倍になるようにレシオを設定
static const float timeRatio = 10.0f / ADC_RESO;
static const float riseFallStep = 0.005f;
static const float adcScaleRatio = 2.0f / ADC_RESO;
static float rise = 0;
static float fall = riseFallStep * 20;

//////////////////////////////////////////

void updateMenuColor()
{
    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    oscColor = RGBLEDPWMControl::MenuColor::GREEN;
    switch (settingMenu)
    {
    case SettingMenu::SEL_RISE:
        oscColor = RGBLEDPWMControl::MenuColor::GREEN;
        break;
    case SettingMenu::SEL_FALL:
        oscColor = RGBLEDPWMControl::MenuColor::CYAN;
        break;
    default:
        break;
    }

    rgbLedControl.setMenuColor(oscColor);
}

void changeMenu(int encValue)
{
    settingMenu = (SettingMenu)constrain((int8_t)(settingMenu + encValue), SettingMenu::SEL_RISE, SettingMenu::SEL_FALL);
    updateMenuColor();
}

bool updatePotLock(int16_t potValue, int8_t sel1, int8_t sel2)
{
    static int8_t lastSel1 = -1;
    static int8_t lastSel2 = -1;

    if (lastSel1 != sel1 || lastSel2 != sel2)
    {
        potLock.setLock(true);
    }

    lastSel1 = sel1;
    lastSel2 = sel2;

    return potLock.update(potValue);
}

//////////////////////////////////////////

void process(int16_t cvInValue, int16_t potValue)
{
    // timeCVは電圧が高いほど周波数も高くなる。+電圧：時間-、-電圧：時間+
    float timeCV = -cvInValue * timeRatio + 1;
    float shape = (potValue - (ADC_RESO >> 1)) * adcScaleRatio;
    slope.update(rise * timeCV, fall * timeCV, shape);
    rgbLedControl.setLevelMap(slope.getValue() * 2047, 0, 2047, 7);
}

void operation(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        changeMenu(-1);
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        changeMenu(1);
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        slope.toggleCycle();
    }
    else if (buttonStates == ButtonCondition::LRE)
    {
        slope.toggleInputMode();
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        switch (settingMenu)
        {
        // RISE/FALLのエンコーダーでの設定は～2秒に制限（体感としてわかりにくいので）
        case SettingMenu::SEL_RISE:
            rise = constrain(rise + (encValue * riseFallStep), 0.0, 2.0);
            break;
        case SettingMenu::SEL_FALL:
            fall = constrain(fall + (encValue * riseFallStep), 0.0, 2.0);
            break;
        }
    }
}

//////////////////////////////////////////

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == IN1)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            in1EdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            in1EdgeLatch = false;
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    slope.process(in1EdgeLatch);
    float value = slope.getValue() * 2047;
    dac.out1(value + bias);
    dac.out2(slope.getEOC() ? (DAC_RESO - 1) : bias);
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

    enc.init(EC1B, EC1A, true);
    buttons[0].init(BTN_A);
    buttons[0].setHoldTime(350);
    buttons[1].init(BTN_B);
    buttons[1].setHoldTime(350);
    buttons[2].init(BTN_RE, false, false, true);
    buttons[2].setHoldTime(500);
    // in2.init(IN2);
    cvIn.init(CV1);
    pot.init(POT1);
    dac.init(SPI_MOSI, SPI_SCK, SPI_CS);

    rgbLedControl.init(10000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColorLevel(3);
    rgbLedControl.setMenuColor(oscColor);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::SQU);

    slope.init(SAMPLE_FREQ);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
    gpio_init(IN1);
    gpio_set_irq_enabled(IN1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection(true);
    // int16_t in2Value = in2.analogReadDirectFast();
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();

    process(cvInValue, potValue);

    rgbLedControl.update();
    rgbLedControl.process();
    tight_loop_contents();
}

void setup1()
{
    // core0のsetupを終わらせてcore1開始したいので適当いれておく
    sleep_ms(500);
}

void loop1()
{
    int8_t encValue = enc.getValue();
    uint8_t btnA = buttons[0].getState();
    uint8_t btnB = buttons[1].getState();
    uint8_t btnRE = buttons[2].getState();
    uint16_t buttonStates = (btnA << 8) + (btnB << 4) + btnRE;
    int16_t potValue = pot.getValue();

    operation(buttonStates, encValue, potValue);

    tight_loop_contents();
    sleep_ms(10);
}
