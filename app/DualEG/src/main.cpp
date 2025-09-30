/*!
 * DualEG
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
// #include "lib/ADCErrorCorrection.hpp"
#include "lib/RGBLEDPWMControl.hpp"
#include "lib/EepRomConfigIO.hpp"
#include "lib/Mcp4922SwSpi.hpp"
#include "lib/pwm_wrapper.h"
#include "lib/ValueLock.hpp"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"
#include "Helper.h"

#include "EnvelopeADSR.hpp"

enum EnvelopeSelect
{
    ENV_A = 0,
    ENV_B,
    SETUP,
    MAX = SETUP,
};

enum EnvelopeSettingMenu
{
    ATTACK = 0,
    DECAY,
    SUSTAIN,
    RELEASE,
    SEL_MAX = RELEASE,
};

enum CVInSelect
{
    CV_NOUSE = 0,
    CV_ATTACK,
    CV_DECAY,
    CV_SUSTAIN,
    CV_RELEASE,
    CV_MAX = CV_RELEASE,
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
// static ADCErrorCorrection adcErrorCorrection;
static Mcp4922SwSpi dac;
static ValueLock potLock;

// gpio割り込み
static volatile bool in1EdgeLatch = false;
static volatile bool in2EdgeLatch = false;

// UIほか
static EnvelopeSettingMenu envMenu = EnvelopeSettingMenu::ATTACK;
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::GREEN;
static EnvelopeSelect envSelect = EnvelopeSelect::ENV_A;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
#define ENV_COUNT 2
static EnvelopeADSR env[ENV_COUNT];
static int16_t bias = DAC_RESO >> 1;

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "D_ADSR_000\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    uint16_t attack[ENV_COUNT];
    uint16_t decay[ENV_COUNT];
    uint16_t sustain[ENV_COUNT];
    uint16_t release[ENV_COUNT];
    CVInSelect cvInSelect[ENV_COUNT];
    bool exp[ENV_COUNT];

    UserConfig()
    {
        for (uint8_t i = 0; i < ENV_COUNT; ++i)
        {
            attack[i] = 0;
            decay[i] = 200;
            sustain[i] = 0;
            release[i] = 40;
            cvInSelect[i] = CVInSelect::CV_DECAY;
            exp[i] = true;
        }
    }
};
static EEPROMConfigIO<UserConfig> userConfig(64); // systemConfigから64バイト開けておく

//////////////////////////////////////////

void updateMenuColor()
{
    // rgbLedControl.setFreq(0);
    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    int8_t index = envSelect;

    if (envSelect == EnvelopeSelect::ENV_A)
    {
        rgbLedControl.setMenuColorLevel(3);
    }
    else
    {
        rgbLedControl.setMenuColorLevel(5);
    }

    switch (envMenu)
    {
    case EnvelopeSettingMenu::ATTACK:
        oscColor = RGBLEDPWMControl::MenuColor::BLUE;
        break;
    case EnvelopeSettingMenu::DECAY:
        oscColor = RGBLEDPWMControl::MenuColor::CYAN;
        break;
    case EnvelopeSettingMenu::SUSTAIN:
        oscColor = RGBLEDPWMControl::MenuColor::GREEN;
        break;
    case EnvelopeSettingMenu::RELEASE:
        oscColor = RGBLEDPWMControl::MenuColor::MAGENTA;
        break;
    default:
        break;
    }

    rgbLedControl.setMenuColor(oscColor);
}

void changeMenu(int encValue)
{
    envMenu = (EnvelopeSettingMenu)constrain((envMenu + encValue), (int16_t)EnvelopeSettingMenu::ATTACK, (int16_t)EnvelopeSettingMenu::SEL_MAX);
    updateMenuColor();
}

void changeEnv(EnvelopeSelect select)
{
    if (select == envSelect)
    {
        return;
    }

    envSelect = select;
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

void updateCVInSelect(uint8_t i, CVInSelect sel, bool edge, int16_t modTime)
{
    if (edge)
    {
        switch (sel)
        {
        case CVInSelect::CV_ATTACK:
            env[i].setAttack(constrain(modTime + userConfig.Config.attack[i], 0, 5000));
            break;
        case CVInSelect::CV_DECAY:
            env[i].setDecay(constrain(modTime + userConfig.Config.decay[i], 0, 5000));
            break;
        case CVInSelect::CV_SUSTAIN:
            env[i].setSustain(constrain(modTime + userConfig.Config.sustain[i], 0, bias - 1));
            break;
        case CVInSelect::CV_RELEASE:
            env[i].setRelease(constrain(modTime + userConfig.Config.release[i], 0, 5000));
            break;
        default:
            break;
        }
    }
}

void processEnv(int16_t cvInValue, int16_t potValue)
{
    static CVInSelect lastCvInSelect[ENV_COUNT] = {CVInSelect::CV_NOUSE, CVInSelect::CV_NOUSE};
    bool isEdge[ENV_COUNT] = {in1EdgeLatch, in2EdgeLatch};

    int16_t modTime = cvInValue;
    for (uint8_t i = 0; i < ENV_COUNT; ++i)
    {
        bool edge = env[i].next(isEdge[i]);
        if (lastCvInSelect[i] != userConfig.Config.cvInSelect[i])
        {
            CVInSelect sel = lastCvInSelect[i];
            updateCVInSelect(i, sel, true, 0);
            lastCvInSelect[i] = userConfig.Config.cvInSelect[i];
        }

        CVInSelect sel = userConfig.Config.cvInSelect[i];
        updateCVInSelect(i, sel, edge, modTime);
    }

    int8_t index = envSelect;
    rgbLedControl.setLevelMap(env[index].getLevel(), 0, bias - 1, 7);
}

void operationEnv(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    int8_t index = envSelect;
    if (buttonStates == ButtonCondition::UA)
    {
        changeEnv(EnvelopeSelect::ENV_A);
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        changeEnv(EnvelopeSelect::ENV_B);
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        userConfig.Config.exp[index] = !userConfig.Config.exp[index];
        env[index].setExp(userConfig.Config.exp[index]);
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        userConfig.Config.cvInSelect[index] =
            (CVInSelect)constrain((userConfig.Config.cvInSelect[index] + encValue),
                                  (int16_t)CVInSelect::CV_NOUSE,
                                  (int16_t)CVInSelect::CV_MAX);
        rgbLedControl.setRainbowLevel(userConfig.Config.cvInSelect[index],
                                      (int16_t)CVInSelect::CV_NOUSE,
                                      (int16_t)CVInSelect::CV_MAX);
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
        changeMenu(encValue);
    }

    bool lock = updatePotLock(potValue, envMenu, envSelect);
    if (lock == false)
    {
        switch (envMenu)
        {
        case EnvelopeSettingMenu::ATTACK:
            potValue = conv2ExpScaled(potValue, 5000);
            userConfig.Config.attack[index] = potValue;
            if (userConfig.Config.cvInSelect[index] != CVInSelect::CV_ATTACK)
            {
                env[index].setAttack(potValue);
            }
            break;
        case EnvelopeSettingMenu::DECAY:
            potValue = conv2ExpScaled(potValue, 5000);
            userConfig.Config.decay[index] = potValue;
            if (userConfig.Config.cvInSelect[index] != CVInSelect::CV_DECAY)
            {
                env[index].setDecay(potValue);
            }
            break;
        case EnvelopeSettingMenu::SUSTAIN:
            potValue = map(potValue, 0, ADC_RESO - 1, 0, bias - 1);
            userConfig.Config.sustain[index] = potValue;
            if (userConfig.Config.cvInSelect[index] != CVInSelect::CV_SUSTAIN)
            {
                env[index].setSustain(potValue);
            }
            break;
        case EnvelopeSettingMenu::RELEASE:
            potValue = conv2ExpScaled(potValue, 5000);
            userConfig.Config.release[index] = potValue;
            if (userConfig.Config.cvInSelect[index] != CVInSelect::CV_RELEASE)
            {
                env[index].setRelease(potValue);
            }
            break;
        default:
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

    dac.out1(env[0].getLevel() + bias);
    dac.out2(env[1].getLevel() + bias);
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

    rgbLedControl.init(10000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(oscColor);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::SQU);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    for (uint8_t i = 0; i < ENV_COUNT; ++i)
    {
        env[i].init((float)(bias - 1));
        env[i].set(userConfig.Config.attack[i], userConfig.Config.decay[i], userConfig.Config.sustain[i], userConfig.Config.release[i]);
        env[i].setExp(userConfig.Config.exp[i]);
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
    int8_t encValue = enc.getDirection();
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();

    processEnv(cvInValue, potValue);

    rgbLedControl.update();
    rgbLedControl.process();
    tight_loop_contents();
    sleep_us(100);
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
    int16_t potValue = pot.getValue();

    operationEnv(buttonStates, encValue, potValue);

    tight_loop_contents();
    sleep_ms(10);
}
