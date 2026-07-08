/*!
 * SmoothRandomCV
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

/*
# SmoothRandomCV

SmoothRandomCVは、Random+SH+Slewを組み合わせてスムーズに可変するCVを出力するファームウェアです。  
ホワイトノイズか1/fゆらぎの値をトリガー毎に固定し出力し、±5Vか0~5Vのminorスケールで出力します。  

## 機能概要

- 入力
  - `IN1` タップテンポトリガー入力
  - `POT` 到達時間調整
  - `RE` 各種設定
- 出力
  - `OUT1` SmoothRandomCV出力
  - `OUT2` タップテンポ出力
  - `RGB LED` タップテンポ表示、各種設定表示

## 使い方

- モード
  - `A,Bボタン` 設定モードを変更：`タップテンポ<->出力レベル設定<->アルゴリズム設定<->RUNモード<->オクターブ設定`  
  - `Aボタン押下中にBボタン押下` 現在の設定値を保存

- タップテンポ
  - `Aボタン` 押下した間隔でトリガーを出力します
- 出力レベル設定
  - `RE操作` ±5V出力、0V~のマイナースケール出力かを選択
- アルゴリズム設定
  - `RE操作` ホワイトノイズ、1/fゆらぎ（ピンクノイズ）のアルゴリズムを選択
- RUNモード
  - `RE操作` フリー、タップテンポでのSHを選択
- オクターブ設定
  - `RE操作` 1~5octまでを選択

## 現状補足

- 出力レベル±5V、RUNモードをフリーにしたらホワイトノイズとピンクが出る
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
#include "lib/ADCErrorCorrection.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EepRomConfigIO.hpp"
// #include "lib/Mcp4922SwSpi.hpp"
#include "lib/Mcp4922HwSpi.hpp"
#include "lib/pwm_wrapper.h"
#include "lib/ValueLock.hpp"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"

#include "Helper.h"
#include "lib/Quantizer.hpp"
#include "lib/EdgeChecker.hpp"
#include "SmoothRandomCV.hpp"

enum SettingMenu
{
    SEL_TAP_TEMPO = 0,
    SEL_STYLE,
    SEL_ALGORITHM,
    SEL_RUNMODE,
    SEL_OCTAVE,
    SEL_MAX = SEL_OCTAVE,
};

enum WaveStyle
{
    STYLE_RAW = 0,
    STYLE_PITCH,
    STYLE_MAX = STYLE_PITCH,
};

enum ButtonCondition
{
    // 各ボタンの押下状態と各ボタンの組み合わせ
    // Button State: 0:None 1:Button down 2:Button up 3:Holding 4:Holded B:Hold edge (Holding | 0x08)
    // U: button up
    // D: button down
    // H: holging
    // L: holded (leaved)
    // E: hold edge
    // 0xMABR (Mode, A, B, RE(RotaryEncoder) button)
    NONE = 0x0000,
    UA = 0x0200,
    UB = 0x0020,
    URE = 0x0002,
    HA = 0x0300,
    HB = 0x0030,
    HA_UB = 0x0320,
    HA_RE = 0x0302,
};

// 標準インターフェース
static uint interruptSliceNum;
static RotaryEncoder enc;
static Button buttons[3];
// static SmoothAnalogRead in1;
static SmoothAnalogRead cvIn;
static SmoothAnalogRead in2;
static SmoothAnalogRead pot;
static RGBLEDPWMControl rgbLedControl;
static ADCErrorCorrection adcErrorCorrection;
static Mcp4922HwSpi dac;
static ValueLock potLock;

// gpio割り込み
static EdgeChecker clockEdge;

// UIほか
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::BLACK;
static EEPROMConfigIO<SystemConfig> systemConfig(0);
static SettingMenu settingMenu = SettingMenu::SEL_TAP_TEMPO;

// 機能
static int16_t bias = DAC_RESO >> 1;
static SmoothRandomCV rndCV;
static Quantizer quantizer(bias);

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "D_SMTRND_000\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    WaveStyle style;
    int16_t curve;
    int8_t octave;
    int8_t algorithm;
    int8_t runMode;

    UserConfig()
    {
        style = STYLE_RAW;
        curve = 10;
        octave = 3;
        algorithm = 1;
        runMode = 1;
    }
};
static EEPROMConfigIO<UserConfig> userConfig(64); // systemConfigから64バイト開けておく

//////////////////////////////////////////

void updateMenuColor()
{
    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    oscColor = RGBLEDPWMControl::MenuColor::CYAN;
    switch (settingMenu)
    {
    case SettingMenu::SEL_TAP_TEMPO:
        rgbLedControl.resetFreq();
        break;
    case SettingMenu::SEL_STYLE:
        rgbLedControl.setFreq(2);
        break;
    case SettingMenu::SEL_ALGORITHM:
        rgbLedControl.setFreq(4);
        break;
    case SettingMenu::SEL_RUNMODE:
        rgbLedControl.setFreq(8);
        break;
    case SettingMenu::SEL_OCTAVE:
        rgbLedControl.setFreq(16);
        break;
    default:
        break;
    }

    rgbLedControl.setMenuColor(oscColor);
}

void changeMenu(int encValue)
{
    settingMenu = (SettingMenu)constrain((int8_t)(settingMenu + encValue), SettingMenu::SEL_TAP_TEMPO, SettingMenu::SEL_MAX);
    updateMenuColor();
}

bool updatePotLock(int16_t potValue, int8_t sel1)
{
    static int8_t lastSel1 = -1;

    if (lastSel1 != sel1)
    {
        potLock.setLock(true);
    }

    lastSel1 = sel1;

    return potLock.update(potValue);
}

//////////////////////////////////////////

void processEnv(int16_t cvInValue, int16_t potValue, int16_t in1Value, int16_t in2Value)
{
    static volatile ulong lastTime = 0;
    ulong currentTime = micros();
    bool in1EdgeLatch = clockEdge.isEdgeHigh();
    if (in1EdgeLatch)
    {
        lastTime = currentTime;
        dac.out2(DAC_RESO - 1);
    }
    else
    {
        int duration = clockEdge.getDurationMicros();
        if (currentTime - lastTime > duration)
        {
            lastTime = currentTime;
            in1EdgeLatch = true;
            dac.out2(DAC_RESO - 1);
        }
        else if (currentTime - lastTime > (duration >> 1))
        {
            dac.out2(DAC_RESO >> 1);
        }
        else
        {
            in1EdgeLatch = false;
        }
    }

    bool on = (userConfig.Config.runMode == 1) ? in1EdgeLatch : true;
    rndCV.update(on, true);
    int16_t level = (int16_t)rndCV.getLevel();
    if (userConfig.Config.style == WaveStyle::STYLE_PITCH)
    {
        level = quantizer.Quantize(map(level, 0, ADC_RESO - 1, 0, (7 * userConfig.Config.octave))) + bias;
    }

    dac.out1(level);

    // static int32_t sampleFreq = 0;
    // sampleFreq = getSamplingFrequency();
    // static int16_t cnt = 0;
    // if (++cnt > 10000)
    // {
    //     cnt = 0;
    //     Serial.print("SampleFreq:");
    //     Serial.print(sampleFreq);
    //     Serial.print(" level:");
    //     Serial.print(level);
    //     Serial.println();
    // }
}

void operationEnv(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        if (settingMenu == SettingMenu::SEL_TAP_TEMPO)
        {
            clockEdge.updateEdge(1);
            clockEdge.updateEdge(0);
        }
        changeMenu(-1);
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        changeMenu(1);
    }
    else if (buttonStates == ButtonCondition::URE)
    {
    }
    else if (buttonStates == ButtonCondition::HA)
    {
    }
    else if (buttonStates == ButtonCondition::HB)
    {
    }
    else if (buttonStates == ButtonCondition::HA_UB)
    {
        userConfig.saveUserConfig();
    }
    else if (buttonStates == ButtonCondition::HA_RE)
    {
    }
    else if (buttonStates == ButtonCondition::NONE)
    {
        switch (settingMenu)
        {
        case SEL_TAP_TEMPO:
            rgbLedControl.setFreq(1000.0 / clockEdge.getDurationMills());
            break;
        case SettingMenu::SEL_STYLE:
            userConfig.Config.style = (WaveStyle)constrain((int16_t)(userConfig.Config.style + encValue), WaveStyle::STYLE_RAW, WaveStyle::STYLE_MAX);
            rgbLedControl.setRainbowLevel((int16_t)userConfig.Config.style, (int16_t)WaveStyle::STYLE_RAW, (int16_t)WaveStyle::STYLE_MAX);
            break;
        case SettingMenu::SEL_ALGORITHM:
            userConfig.Config.algorithm = constrain(userConfig.Config.algorithm + encValue, 0, 1);
            rndCV.enableFluctuation(userConfig.Config.algorithm == 1 ? true : false);
            rgbLedControl.setRainbowLevel((int16_t)userConfig.Config.algorithm, (int16_t)0, (int16_t)1);
            break;
        case SettingMenu::SEL_RUNMODE:
            userConfig.Config.runMode = constrain(userConfig.Config.runMode + encValue, 0, 1);
            rgbLedControl.setRainbowLevel((int16_t)userConfig.Config.runMode, (int16_t)0, (int16_t)1);
            break;
        case SettingMenu::SEL_OCTAVE:
            userConfig.Config.octave = constrain(userConfig.Config.octave + encValue, 1, 5);
            rgbLedControl.setRainbowLevel((int16_t)userConfig.Config.octave, (int16_t)1, (int16_t)5);
            break;
        default:
            break;
        }
    }

    bool lock = updatePotLock(potValue, settingMenu);
    if (lock == false)
    {
        userConfig.Config.curve = constrain(map(potValue, 0, DAC_RESO - 1, 1, 32), 1, 32);
        rndCV.setCurve(userConfig.Config.curve);
        if (userConfig.Config.curve >= 32)
        {
            rndCV.enableSmoothing(false);
        }
        else
        {
            rndCV.enableSmoothing(true);
        }
    }
}

//////////////////////////////////////////

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
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
    cvIn.init(CV1);
    pot.init(POT1);
    dac.init(SPI_MOSI, SPI_SCK, SPI_CS);
    clockEdge.init(IN1);

    rgbLedControl.init(40000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLACK);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    quantizer.setScale(5); // minor
    rndCV.init(DAC_RESO);
    rndCV.setMaxLevel(DAC_RESO - 1);
    rndCV.setCurve(userConfig.Config.curve);
    rndCV.enableFluctuation(userConfig.Config.algorithm == 1 ? true : false);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    updateMenuColor();
}

void loop()
{
    int8_t encValue = enc.getDirection();
    int16_t cvInValue = cvIn.analogReadDirectFast() - bias;
    int16_t in1Value = 0;
    int16_t in2Value = 0;
    int16_t potValue = adcErrorCorrection.correctedAdc(pot.analogReadDirectFast());

    processEnv(cvInValue, potValue, in1Value, in2Value);

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
    int16_t potValue = adcErrorCorrection.correctedAdc(pot.getValue());

    operationEnv(buttonStates, encValue, potValue);

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
