/*!
 * DCF/DCA/DigitalLPG
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

/*
# DCF/DCA/DigitalLPG

DCF/DCA/DigitalLPGは、12bit44.1kHzで入出力するDCF/DCAファームウェアです。  
2次LPFのDCF->DCA接続モードと、2次LPFにディケイタイムを加えたLPGモードから選択して音色変化を与えることが出来ます。  

## 機能概要

- 入力
  - `IN1` DCF Freq CV入力
  - `IN2` DCA Level CV入力
  - `CV` 音声・CV入力
  - `POT` DCF 周波数調整
  - `RE` レゾナンス調整、各種設定など
- 出力
  - `OUT1` 音声・CV出力
  - `RGB LED` 設定表示など

## 使い方

- モード
  - `RE押し込み` LPGモード：DCF/DCAモード切り替え
  - `A,Bボタン` 設定モードを変更：`レゾナンス調整<->DCF Freq入力調整<->DCA Level入力調整<->LPGディケイタイム調整`  
  - `Aボタン押下中にBボタン押下` 現在の設定値を保存
  - `RE操作` 各設定値の調整
  - `POT` DCF 周波数調整
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

#include "Multifilter.hpp"

#undef SAMPLE_FREQ
#define SAMPLE_FREQ 44100

enum SettingMenu
{
    SEL_RESO = 0,
    SEL_CV_VCF,
    SEL_CV_VCA,
    SEL_CV_LPG_CLOSING,
    SEL_MAX = SEL_CV_LPG_CLOSING,
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
    HA = 0x0300,
    HB = 0x0030,
    HA_UB = 0x0320,
    HA_RE = 0x0302,
};

// 標準インターフェース
static uint interruptSliceNum;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead in1;
static SmoothAnalogRead cvIn;
static SmoothAnalogRead in2;
static SmoothAnalogRead pot;
static RGBLEDPWMControl rgbLedControl;
static ADCErrorCorrection adcErrorCorrection;
static Mcp4922HwSpi dac;
static ValueLock potLock;

// UIほか
static SettingMenu settingMenu = SettingMenu::SEL_RESO;
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::BLACK;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
static int16_t bias = DAC_RESO >> 1;
static MultiFilter filter(SAMPLE_FREQ);
#define LPG_LIKE_RELEASES_NUM 6
const float lpgClosingRatios[LPG_LIKE_RELEASES_NUM] = {
    0.9900, 0.9975, 0.9987, 0.9990, 0.9992, 0.9994};

static int16_t currentWaveValue = 0;
static float vcaRatio = 1.0;

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "D_VCFA_000\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    int16_t frequency;
    float resonance;
    float modFreq;
    float modAmp;
    int16_t lpgClosing;
    bool lpgMode;
    // FilterType filterType;

    UserConfig()
    {
        frequency = 100;
        resonance = 1;
        modFreq = 1.0;
        modAmp = 1.0;
        lpgClosing = 2;
        lpgMode = true;
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
    case SettingMenu::SEL_RESO:
        rgbLedControl.resetFreq();
        break;
    case SettingMenu::SEL_CV_VCF:
        rgbLedControl.setFreq(2);
        break;
    case SettingMenu::SEL_CV_VCA:
        rgbLedControl.setFreq(4);
        break;
    case SettingMenu::SEL_CV_LPG_CLOSING:
        rgbLedControl.setFreq(6);
        break;
    default:
        break;
    }

    rgbLedControl.setMenuColor(oscColor);
}

void changeMenu(int encValue)
{
    settingMenu = (SettingMenu)constrain((int8_t)(settingMenu + encValue), SettingMenu::SEL_RESO, SettingMenu::SEL_MAX);
    updateMenuColor();
}

//////////////////////////////////////////

void processEnv(int16_t cvInValue, int16_t potValue, int16_t in1Value, int16_t in2Value)
{
    static int16_t smoothIn1Value = 0;
    int16_t vcaModSource = in2Value;
    if (userConfig.Config.lpgMode == false)
    {
        if (smoothIn1Value <= in1Value)
        {
            // attack
            smoothIn1Value = in1Value;
        }
        else
        {
            // release
            float releaseRatio = 0.90;
            smoothIn1Value = (smoothIn1Value * releaseRatio) + (in1Value * (1.0 - releaseRatio));
        }

        vcaRatio = constrain(conv2LinScaled<float>(vcaModSource, userConfig.Config.modAmp), 0, 1.00);
        currentWaveValue = cvInValue;
    }
    else
    {
        if (smoothIn1Value <= in1Value)
        {
            // attack
            smoothIn1Value = (smoothIn1Value + in1Value) >> 1;
        }
        else
        {
            // release
            float releaseRatio = lpgClosingRatios[userConfig.Config.lpgClosing];
            smoothIn1Value = (smoothIn1Value * releaseRatio) + (in1Value * (1.0 - releaseRatio));
        }

        vcaModSource = smoothIn1Value + potValue;
        vcaRatio = constrain(conv2LinScaled<float>(vcaModSource, userConfig.Config.modAmp), 0, 1.00);
        currentWaveValue = cvInValue * vcaRatio;
    }

    int16_t freq = constrain(userConfig.Config.frequency + (conv2ExpScaled(smoothIn1Value, 16000) * userConfig.Config.modFreq), 10, 16000);

    filter.LowPass((float)freq, userConfig.Config.resonance);
}

void operationEnv(uint16_t buttonStates, int8_t encValue, int16_t potValue)
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
        userConfig.Config.lpgMode = !userConfig.Config.lpgMode;
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
        case SettingMenu::SEL_RESO:
            userConfig.Config.resonance = constrain(userConfig.Config.resonance + (encValue * 0.4), 1.0, 30.0);
            rgbLedControl.setRainbowLevel(userConfig.Config.resonance, 0, 30);
            break;
        case SettingMenu::SEL_CV_VCF:
            userConfig.Config.modFreq = constrain(userConfig.Config.modFreq + (encValue * 0.01), 0.0, 1.0);
            rgbLedControl.setRainbowLevel(userConfig.Config.modFreq * 100, 0, 100);
            break;
        case SettingMenu::SEL_CV_VCA:
            userConfig.Config.modAmp = constrain(userConfig.Config.modAmp + (encValue * 0.01), 0.0, 1.0);
            rgbLedControl.setRainbowLevel(userConfig.Config.modAmp * 100, 0, 100);
            break;
        case SettingMenu::SEL_CV_LPG_CLOSING:
            userConfig.Config.lpgClosing = constrain(userConfig.Config.lpgClosing + encValue, 0, LPG_LIKE_RELEASES_NUM - 1);
            rgbLedControl.setRainbowLevel(userConfig.Config.lpgClosing, 0, LPG_LIKE_RELEASES_NUM - 1);
            break;
        default:
            break;
        }
    }

    userConfig.Config.frequency = conv2ExpScaled(potValue, 16000);
}

//////////////////////////////////////////

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    int16_t value = currentWaveValue;
    if (userConfig.Config.lpgMode == false)
    {
        value = value * vcaRatio;
    }
    value = filter.Process((float)value);
    value = constrain(value + bias, 0, DAC_RESO - 1);
    dac.out1(value);
    // dac.out2(env[1].getLevel() + bias);
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
    in1.init(IN1);
    in2.init(IN2);
    cvIn.init(CV1);
    pot.init(POT1);
    dac.init(SPI_MOSI, SPI_SCK, SPI_CS);
    // agc.init(DAC_RESO, 1, 0.96);

    rgbLedControl.init(10000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLACK);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    // filter.LowPass((float)userConfig.Config.frequency, (float)userConfig.Config.resonance, SAMPLE_FREQ);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    int8_t encValue = enc.getDirection(true);
    int16_t cvInValue = cvIn.analogReadDirectFast() - bias;
    int16_t in1Value = adcErrorCorrection.correctedAdc(in1.analogReadDirectFast());
    int16_t in2Value = adcErrorCorrection.correctedAdc(in2.analogReadDirectFast());
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
