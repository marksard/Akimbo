/*!
 * RhythmBox
 * Copyright 2025 marksard
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
#include "lib/ADCErrorCorrection.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EepRomConfigIO.hpp"
#include "lib/Mcp4922SwSpi.hpp"
#include "lib/pwm_wrapper.h"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"
#include "Helper.h"

#include "lib/ActiveGainControl.hpp"
#include "SingleShotWave.hpp"
#include "wavetable/808_01/808_BD_02.h"
#include "wavetable/808_01/808_SD_01.h"
#include "wavetable/808_01/808_Rim_01.h"
#include "wavetable/808_01/808_OHH_01.h"
#include "wavetable/909_01/909_BD_Norm.h"
#include "wavetable/909_01/909_SD_Snap.h"
#include "wavetable/909_01/909_LT.h"
#include "wavetable/909_01/909_RC.h"
#include "wavetable/909_01/909_OH.h"

#undef SAMPLE_FREQ
#define SAMPLE_FREQ 44100 // 再生ファイルにあわせる

enum SoundParam
{
    WAVE,
    VOLUME,
    PITCH,
    PAN,
    DECAY,
    MUTE,
    SET_MAX = MUTE
};

enum SoundBank
{
    CH1 = 0,
    CH2,
    CH3,
    GP_MAX = CH3,
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
static Mcp4922SwSpi dac;

// gpio割り込み
static volatile bool in1EdgeLatch = false;
static volatile bool in2EdgeLatch = false;

// UIほか
#define OSC_MENU_MAX 4
static SoundParam selectParam = SoundParam::WAVE;
static SoundBank selectBank = SoundBank::CH1;
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::RED;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
static ActiveGainControl agc;
#define SOUND_BANK_COUNT 3
#define SOUND_WAVE_COUNT 3
SingleShotWave BD1(buf_808_BD_02, buf_size_808_BD_02);
SingleShotWave BD2(buf_909_BD_Norm, buf_size_909_BD_Norm);
SingleShotWave LT2(buf_909_LT, buf_size_909_LT);
SingleShotWave SD1(buf_808_SD_01, buf_size_808_SD_01);
SingleShotWave SD2(buf_909_SD_Snap, buf_size_909_SD_Snap);
SingleShotWave RS1(buf_808_Rim_01, buf_size_808_Rim_01);
SingleShotWave OH1(buf_808_OHH_01, buf_size_808_OHH_01);
SingleShotWave OH2(buf_909_OH, buf_size_909_OH);
SingleShotWave RC2(buf_909_RC, buf_size_909_RC);
SingleShotWave<int16_t> *pKit[SOUND_BANK_COUNT][SOUND_WAVE_COUNT] = 
{
    {&BD1, &BD2, &LT2},
    {&SD1, &SD2, &RS1},
    {&OH1, &OH2, &RC2}
};

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "RHYTHMBOX_001\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    int8_t selectWave[SOUND_BANK_COUNT];
    float volume[SOUND_BANK_COUNT];
    float pitch[SOUND_BANK_COUNT];
    int8_t pan[SOUND_BANK_COUNT];
    float decay[SOUND_BANK_COUNT];
    int8_t mute[SOUND_BANK_COUNT];

    UserConfig()
    {
        for (int i = 0; i < SOUND_BANK_COUNT; ++i)
        {
            selectWave[i] = 0;
            volume[i] = 1.0f;
            pitch[i] = 1.0f;
            pan[i] = 2;
            decay[i] = 1.0f;
            mute[i] = 0;
        }
        decay[SOUND_BANK_COUNT - 1] = 0.1f;
    }
};
static EEPROMConfigIO<UserConfig> userConfig(64); // systemConfigから64バイト開けておく

//////////////////////////////////////////

void changeSelectParam(int encValue)
{
    selectParam = constrainCyclic((SoundParam)(selectParam + encValue), SoundParam::WAVE, SoundParam::SET_MAX);

    switch (selectParam)
    {
    case SoundParam::WAVE:
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::RED);
        break;
    case SoundParam::VOLUME:
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::GREEN);
        break;
    case SoundParam::PITCH:
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLUE);
        break;
    case SoundParam::PAN:
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::YELLOW);
        break;
    case SoundParam::DECAY:
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::CYAN);
        break;
    case SoundParam::MUTE:
        rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::MAGENTA);
        break;
    }
}

void changeSelectBank(int encValue)
{
    selectBank = (SoundBank)constrain((selectBank + encValue), (int)SoundBank::CH1, (int)SoundBank::GP_MAX);

    switch (selectBank)
    {
    case SoundBank::CH1:
        rgbLedControl.resetFreq();
        break;
    case SoundBank::CH2:
        rgbLedControl.setFreq(2);
        break;
    case SoundBank::CH3:
        rgbLedControl.setFreq(4);
        break;
    }
}

//////////////////////////////////////////

void operationParameter(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    if (buttonStates == ButtonCondition::UA)
    {
        changeSelectBank(-1);
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        changeSelectBank(1);
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        changeSelectParam(1);
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
        switch (selectParam)
        {
        case SoundParam::WAVE:
            userConfig.Config.selectWave[selectBank] = constrain((userConfig.Config.selectWave[selectBank] + encValue), 0, SOUND_WAVE_COUNT - 1);
            rgbLedControl.setLevelMap(userConfig.Config.selectWave[selectBank], 0, SOUND_WAVE_COUNT - 1, 6);
            break;
        case SoundParam::VOLUME:
            userConfig.Config.volume[selectBank] = constrain(userConfig.Config.volume[selectBank] + (encValue * 0.05), 0.0f, 1.0f);
            pKit[selectBank][0]->setVolume(userConfig.Config.volume[selectBank]);
            pKit[selectBank][1]->setVolume(userConfig.Config.volume[selectBank]);
            pKit[selectBank][2]->setVolume(userConfig.Config.volume[selectBank]);
            rgbLedControl.setLevelMap((int16_t)(userConfig.Config.volume[selectBank] * 10), 0, 10, 6);
            break;
        case SoundParam::PITCH:
            userConfig.Config.pitch[selectBank] = constrain(userConfig.Config.pitch[selectBank] + (encValue * 0.05), 0.1f, 2.0f);
            pKit[selectBank][0]->setSpeed(userConfig.Config.pitch[selectBank]);
            pKit[selectBank][1]->setSpeed(userConfig.Config.pitch[selectBank]);
            pKit[selectBank][2]->setSpeed(userConfig.Config.pitch[selectBank]);
            rgbLedControl.setLevelMap((int16_t)(userConfig.Config.pitch[selectBank] * 10), 0, 10, 6);
            break;
        case SoundParam::PAN:
            userConfig.Config.pan[selectBank] = constrain(userConfig.Config.pan[selectBank] + encValue, 0, 4);
            rgbLedControl.setLevelMap(userConfig.Config.pan[selectBank], 0, 4, 6);
            break;
        case SoundParam::DECAY:
            userConfig.Config.decay[selectBank] = constrain(userConfig.Config.decay[selectBank] + (encValue * 0.05), 0.0f, 1.0f);
            rgbLedControl.setLevelMap((int16_t)(userConfig.Config.decay[selectBank] * 10), 0, 10, 6);
            break;
        case SoundParam::MUTE:
            userConfig.Config.mute[selectBank] = constrain(userConfig.Config.mute[selectBank] + encValue, 0, 1);
            rgbLedControl.setLevelMap(userConfig.Config.mute[selectBank], 0, 1, 6);
            break;
        }
    }

    int8_t muteIndex = map(potValue, 0, ADC_RESO - 1, SOUND_BANK_COUNT, 0);
    int8_t mutes[SOUND_BANK_COUNT + 1] = {-1, 1, 2, 0};
    for (int i = 0; i < SOUND_BANK_COUNT + 1; ++i)
    {
        int8_t mute = mutes[i];
        if (mute == -1)continue;
        if (i <= muteIndex)
        {
            pKit[mute][0]->setMute(true);
            pKit[mute][1]->setMute(true);
            pKit[mute][2]->setMute(true);
        }
        else
        {
            pKit[mute][0]->setMute(userConfig.Config.mute[mute] ? true : false);
            pKit[mute][1]->setMute(userConfig.Config.mute[mute] ? true : false);
            pKit[mute][2]->setMute(userConfig.Config.mute[mute] ? true : false);
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
    if (gpio == IN2)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            in2EdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            in2EdgeLatch = false;
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    int16_t levelL = 0;
    int16_t levelR = 0;
    for (int i = 0; i < SOUND_BANK_COUNT; ++i)
    {
        int16_t level = pKit[i][userConfig.Config.selectWave[i]]->updateWave();
        uint16_t panTmp = userConfig.Config.pan[i];
        uint16_t panL = panTmp <= 2 ? 0 : panTmp - 2;
        uint16_t panR = panTmp >= 2 ? 0 : 2 - panTmp;
        levelL += level >> panL;
        levelR += level >> panR;
    }
    agc.setCurrentLevel(levelL, levelR);

    levelL = agc.getProcessedLevel(levelL);
    levelR = agc.getProcessedLevel(levelR);

    dac.out1(levelL);
    dac.out2(levelR);
}

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    analogReadResolution(ADC_BIT);
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
    // gain最大値の設定は全部同時に鳴ることはほぼないため大きめにしている
    agc.init(DAC_RESO, SOUND_BANK_COUNT, 0.96);

    rgbLedControl.init(4000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    for (int i = 0; i < SOUND_BANK_COUNT; ++i)
    {
        for (int j = 0; j < SOUND_WAVE_COUNT; ++j)
        {
            pKit[i][j]->setVolume(userConfig.Config.volume[i]);
            pKit[i][j]->setSpeed(userConfig.Config.pitch[i]);
            pKit[i][j]->setMute(userConfig.Config.mute[i] ? true : false);
        }
    }

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    gpio_set_irq_enabled(IN1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IN2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection(true);
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();
    bool cvHigh = cvInValue > (ADC_RESO >> 2);

    agc.update(0.4);

    pKit[0][userConfig.Config.selectWave[0]]->play(in2EdgeLatch);
    pKit[1][userConfig.Config.selectWave[1]]->play(in1EdgeLatch);
    pKit[2][userConfig.Config.selectWave[2]]->play(cvHigh);
    pKit[0][userConfig.Config.selectWave[0]]->updateDecay(userConfig.Config.decay[0]);
    pKit[1][userConfig.Config.selectWave[1]]->updateDecay(userConfig.Config.decay[1]);
    pKit[2][userConfig.Config.selectWave[2]]->updateDecay(userConfig.Config.decay[2]);

    rgbLedControl.process();
    tight_loop_contents();
    sleep_us(500);
}

void setup1()
{
}

void loop1()
{
    int8_t encValue = enc.getValue();
    uint8_t btnA = buttons[0].getState();
    uint8_t btnB = buttons[1].getState();
    uint8_t btnRE = buttons[2].getState();
    uint16_t buttonStates = (btnA << 8) + (btnB << 4) + btnRE;
    int16_t potValue = pot.getValue();

    operationParameter(buttonStates, encValue, potValue);

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
