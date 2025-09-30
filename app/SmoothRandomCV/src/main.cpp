/*!
 * SmoothRandomCV
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
#include "lib/ValueLock.hpp"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"

#include "Helper.h"
#include "lib/Quantizer.hpp"
#include "SmoothRandomCV.hpp"

enum ChannelSelect
{
    RND_A = 0,
    RND_B,
    MAX = RND_B
};

enum SettingMenu
{
    SEL_STYLE = 0,
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
static ValueLock potLock;

// gpio割り込み
static volatile bool in1EdgeLatch = false;
static volatile bool in2EdgeLatch = false;

// UIほか
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::BLACK;
static EEPROMConfigIO<SystemConfig> systemConfig(0);
static SettingMenu settingMenu = SettingMenu::SEL_STYLE;
static ChannelSelect channelSelect = ChannelSelect::RND_A;

// 機能
#define RND_COUNT 2
static int16_t bias = DAC_RESO >> 1;
static SmoothRandomCV rndCV[RND_COUNT];
static Quantizer quantizer(bias);

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "D_SMTRND_001\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    WaveStyle style[RND_COUNT];
    int16_t curve[RND_COUNT];
    int8_t octave[RND_COUNT];
    int8_t algorithm[RND_COUNT];
    int8_t runMode[RND_COUNT];

    UserConfig()
    {
        for (int i = 0; i < RND_COUNT; ++i)
        {
            style[i] = STYLE_RAW;
            curve[i] = 10;
            octave[i] = 3;
            algorithm[i] = 1;
            runMode[i] = 1;
        }
    }
};
static EEPROMConfigIO<UserConfig> userConfig(64); // systemConfigから64バイト開けておく

//////////////////////////////////////////

void updateMenuColor()
{
    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    if (channelSelect == ChannelSelect::RND_A)
    {
        oscColor = RGBLEDPWMControl::MenuColor::CYAN;
        switch (settingMenu)
        {
        case SettingMenu::SEL_STYLE:
            rgbLedControl.resetFreq();
            break;
        case SettingMenu::SEL_ALGORITHM:
            rgbLedControl.setFreq(2);
            break;
        case SettingMenu::SEL_RUNMODE:
            rgbLedControl.setFreq(4);
            break;
        case SettingMenu::SEL_OCTAVE:
            rgbLedControl.setFreq(8);
            break;
        default:
            break;
        }
    }
    else if (channelSelect == ChannelSelect::RND_B)
    {
        oscColor = RGBLEDPWMControl::MenuColor::MAGENTA;
        switch (settingMenu)
        {
        case SettingMenu::SEL_STYLE:
            rgbLedControl.resetFreq();
            break;
        case SettingMenu::SEL_ALGORITHM:
            rgbLedControl.setFreq(2);
            break;
        case SettingMenu::SEL_RUNMODE:
            rgbLedControl.setFreq(4);
            break;
        case SettingMenu::SEL_OCTAVE:
            rgbLedControl.setFreq(8);
            break;
        default:
            break;
        }
    }

    rgbLedControl.setMenuColor(oscColor);
}

void changeMenu(int encValue)
{
    settingMenu = (SettingMenu)constrain((int8_t)(settingMenu + encValue), SettingMenu::SEL_STYLE, SettingMenu::SEL_MAX);
    updateMenuColor();
}

void toggleChannel()
{
    channelSelect = channelSelect == ChannelSelect::RND_A ? ChannelSelect::RND_B : ChannelSelect::RND_A;
    settingMenu = SettingMenu::SEL_STYLE;
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

void processEnv(int16_t cvInValue, int16_t potValue, int16_t in1Value, int16_t in2Value)
{
    bool edges[RND_COUNT] = {in1EdgeLatch, in2EdgeLatch};
    int16_t level[RND_COUNT] = {0};
    for (int i = 0; i < RND_COUNT; ++i)
    {
        bool on = (userConfig.Config.runMode[i] == 1) ? edges[i] : true;
        rndCV[i].update(on, true);
        level[i] = (int16_t)rndCV[i].getLevel();
        if (userConfig.Config.style[i] == WaveStyle::STYLE_PITCH)
        {
            level[i] = quantizer.Quantize(map(level[i], 0, ADC_RESO - 1, 0, (7 * userConfig.Config.octave[i]))) + bias;
        }
    }

    dac.out1(level[0]);
    dac.out2(level[1]);

    if (in1EdgeLatch)
    {
        in1EdgeLatch = false;
    }

    if (in2EdgeLatch)
    {
        in2EdgeLatch = false;
    }

    static int32_t sampleFreq = 0;
    sampleFreq = getSamplingFrequency();
    static int16_t cnt = 0;
    if (++cnt > 10000)
    {
        cnt = 0;
        Serial.print("SampleFreq:");
        Serial.print(sampleFreq);
        Serial.print(" level[0]:");
        Serial.print(level[0]);
        Serial.print(" level[1]:");
        Serial.print(level[1]);
        Serial.println();
    }
}

void operationEnv(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    int8_t index = channelSelect;
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
        toggleChannel();
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
        case SettingMenu::SEL_STYLE:
            userConfig.Config.style[index] = (WaveStyle)constrain((int8_t)(userConfig.Config.style[index] + encValue), WaveStyle::STYLE_RAW, WaveStyle::STYLE_MAX);
            break;
        case SettingMenu::SEL_ALGORITHM:
            userConfig.Config.algorithm[index] = constrain(userConfig.Config.algorithm[index] + encValue, 0, 1);
            rndCV[index].enableFluctuation(userConfig.Config.algorithm[index] == 1 ? true : false);
            break;
        case SettingMenu::SEL_RUNMODE:
            userConfig.Config.runMode[index] = constrain(userConfig.Config.runMode[index] + encValue, 0, 1);
            break;
        case SettingMenu::SEL_OCTAVE:
            userConfig.Config.octave[index] = constrain(userConfig.Config.octave[index] + encValue, 1, 5);
            break;
        default:
            break;
        }
    }

    bool lock = updatePotLock(potValue, settingMenu, channelSelect);
    if (lock == false)
    {
        // switch (settingMenu)
        // {
        // case SettingMenu::SEL_STYLE:
        //     break;
        // case SettingMenu::SEL_ALGORITHM:
        //     break;
        // case SettingMenu::SEL_RUNMODE:
        //     break;
        // case SettingMenu::SEL_OCTAVE:
        //     break;
        // default:
        //     break;
        // }

        userConfig.Config.curve[index] = constrain(map(potValue, 0, DAC_RESO - 1, 1, 32), 1, 32);
        rndCV[index].setCurve(userConfig.Config.curve[index]);
        if (userConfig.Config.curve[index] >= 32)
        {
            rndCV[index].enableSmoothing(false);
        }
        else
        {
            rndCV[index].enableSmoothing(true);
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
    else if (gpio == IN2)
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

    rgbLedControl.init(30000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(RGBLEDPWMControl::MenuColor::BLACK);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);
    
    quantizer.setScale(5); // minor
    for (int i = 0; i < RND_COUNT; ++i)
    {
        rndCV[i].init(DAC_RESO);
        rndCV[i].setMaxLevel(DAC_RESO - 1);
        rndCV[i].setCurve(userConfig.Config.curve[i]);
    }

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    gpio_set_irq_enabled(IN1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IN2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    updateMenuColor();
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
    // sleep_us(20);
}

void setup1()
{
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
