/*!
 * ChordDCO
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

#include "lib/MultiWaveOsc.hpp"
#include "lib/ActiveGainControl.hpp"
#include "lib/EdgeChecker.hpp"
#include "lib/RandomFast.hpp"

enum OscillatorSelect
{
    OSCA = 0,
    SETUP,
    MAX = SETUP
};

enum OscillatorSettingMenu
{
    SEL_NOTE = 0,
    SEL_WAVE,
    SEL_CVIN,
    SEL_HOLD,
    SEL_MAX = SEL_HOLD,
};

enum CVInSelect
{
    NOUSE = 0,
    MORPH,
    LAST = MORPH
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
static EdgeChecker gateEdge;

// UIほか
#define OSC_MENU_MAX 4
static OscillatorSettingMenu oscMenu = OscillatorSettingMenu::SEL_NOTE;
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::CYAN;
static OscillatorSelect oscSelect = OscillatorSelect::OSCA;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
#define OSC_COUNT 4
static MultiWaveOsc osc[OSC_COUNT];
static ActiveGainControl agc;
static RandomFast rnd;
static const float voctIndexRatio = 60.0f / (float)(ADC_RESO - 1);
static const uint16_t bias = DAC_RESO >> 1;

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "CHORDVCO_000\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    int8_t courseIndex;
    int16_t morphCVMod;
    int16_t morphPot;
    CVInSelect cvInSelect;
    int16_t arpMode;
    uint16_t r7OctSelect;
    int16_t voctHold;
    int16_t scale;
    int8_t arpStep;
    MultiWaveOsc::Wave wave;

    UserConfig()
    {
        courseIndex = 36;
        morphCVMod = 0;
        morphPot = 0;
        cvInSelect = CVInSelect::MORPH;
        arpMode = 1;
        r7OctSelect = 0;
        voctHold = 0;
        scale = 0;
        arpStep = 0;
        wave = MultiWaveOsc::Wave::TRI;
    }
};
static EEPROMConfigIO<UserConfig> userConfig(64); // systemConfigから64バイト開けておく


// コードを構成する音の12音階上での位置
static uint8_t addRootScale[][7][OSC_COUNT] =
    {
        // natulal minor chord
        {
            {0, 3, 7, 10}, // Im7
            {0, 3, 6, 10}, // IIdim
            {0, 4, 7, 11}, // IIIM7
            {0, 3, 7, 10}, // IVm7
            {0, 3, 7, 10}, // Vm7
            {0, 4, 7, 11}, // VIM7
            {0, 4, 7, 10}, // VII7
        },
        // major chord
        {
            {0, 4, 7, 11}, // IM7
            {0, 3, 7, 10}, // IIm7
            {0, 3, 7, 10}, // IIIm7
            {0, 4, 7, 11}, // IVM7
            {0, 4, 7, 10}, // V7
            {0, 3, 7, 10}, // VIm7
            {0, 3, 6, 10}, // VIIdim7
        }};

// ルート起点としてノートの位置による構成コード（addRootScale）の指定
// ※キーからはずれたノートはとりあえず0にしている
static uint8_t rootScaleIndexFromSemitone[][12] =
    {
        // natulal minor chord
        {0, 0, 1, 2, 0, 3, 0, 4, 5, 0, 6, 0},
        // major chord
        {0, 0, 1, 0, 2, 3, 0, 4, 0, 5, 0, 6}};


static int8_t r7OctSet[][2] = 
{
    {  0,  0},
    {-12,  0},
    {  0,-12},
    {-12,-12},
    { 12,  0}
};
const uint16_t r7OctSetMax = sizeof(r7OctSet) / sizeof(r7OctSet[0]);

//////////////////////////////////////////

void updateMenuColor()
{
    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    if (oscSelect == OscillatorSelect::SETUP)
    {
        oscColor = RGBLEDPWMControl::MenuColor::WHITE;
    }
    else if (oscSelect == OscillatorSelect::OSCA)
    {
        oscColor = RGBLEDPWMControl::MenuColor::CYAN;
        switch (oscMenu)
        {
        case OscillatorSettingMenu::SEL_NOTE:
            rgbLedControl.resetFreq();
            break;
        case OscillatorSettingMenu::SEL_WAVE:
            rgbLedControl.setFreq(2);
            break;
        case OscillatorSettingMenu::SEL_CVIN:
            rgbLedControl.setFreq(4);
            break;
        case OscillatorSettingMenu::SEL_HOLD:
            rgbLedControl.setFreq(6);
            break;
        default:
            break;
        }
    }

    rgbLedControl.setMenuColor(oscColor);
}

void changeMenu(int encValue)
{
    oscMenu = (OscillatorSettingMenu)constrain((int8_t)(oscMenu + encValue), OscillatorSettingMenu::SEL_NOTE, OscillatorSettingMenu::SEL_MAX);
    updateMenuColor();
}

void changeOsc(OscillatorSelect select)
{
    if (select == oscSelect)
    {
        return;
    }

    oscSelect = select;
    oscMenu = OscillatorSettingMenu::SEL_NOTE;
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
void processVCO(int16_t in1Value, int16_t cvInValue, int16_t potValue)
{
    static int16_t voctIndex = 0;
    int16_t tempVoctIndex = (int16_t)(adcErrorCorrection.correctedAdc(in1Value) * voctIndexRatio + 0.5f);

    if (tempVoctIndex != voctIndex)
    {
        userConfig.Config.arpStep = 0;
        voctIndex = tempVoctIndex;
    }

    uint8_t rootDiff = (userConfig.Config.courseIndex + (int)voctIndex) % 12;
    uint8_t scaleIndex = rootScaleIndexFromSemitone[userConfig.Config.scale][rootDiff];
    int8_t rootMinus = r7OctSet[userConfig.Config.r7OctSelect][0];
    int8_t seventhMinus = r7OctSet[userConfig.Config.r7OctSelect][1];

    // arpeggio
    if (in1EdgeLatch)
    {
        in1EdgeLatch = false;
        if (userConfig.Config.arpMode == 0)
            userConfig.Config.arpStep = 0;
        else if (userConfig.Config.arpMode == 1)
            userConfig.Config.arpStep = constrainCyclic(userConfig.Config.arpStep + 1, 0, 3);
        else if (userConfig.Config.arpMode == 2)
            userConfig.Config.arpStep = constrainCyclic(userConfig.Config.arpStep - 1, 0, 3);
        else
            userConfig.Config.arpStep = rnd.getRandom16(4);
    }

    if (userConfig.Config.voctHold == 0 || (userConfig.Config.voctHold == 1 && gateEdge.getValue()))
    {
        osc[0].setFreqFromNoteIndex(userConfig.Config.courseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][0] + rootMinus);
        osc[1].setFreqFromNoteIndex(userConfig.Config.courseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][1]);
        osc[2].setFreqFromNoteIndex(userConfig.Config.courseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][2]);
        osc[3].setFreqFromNoteIndex(userConfig.Config.courseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][3] + seventhMinus);
    }

    for (int i = 0; i < OSC_COUNT; ++i)
    {
        if (userConfig.Config.cvInSelect == CVInSelect::MORPH)
        {
            userConfig.Config.morphCVMod = cvInValue;
            osc[i].setFolding(userConfig.Config.morphCVMod + userConfig.Config.morphPot);
            osc[i].setPhaseShift(userConfig.Config.morphCVMod + userConfig.Config.morphPot);
            osc[i].setPulseWidth(userConfig.Config.morphCVMod + userConfig.Config.morphPot);
        }
        else
        {
            osc[i].setFolding(userConfig.Config.morphPot);
            osc[i].setPhaseShift(userConfig.Config.morphPot);
            osc[i].setPulseWidth(userConfig.Config.morphPot);
        }
    }

    static uint16_t dispCount = 0;
    dispCount++;
    if (dispCount == 2000)
    {
        dispCount = 0;
        Serial.print(" vref:");
        Serial.print(adcErrorCorrection.getLastVRef(), 4);
        Serial.print(" noise:");
        Serial.print(adcErrorCorrection.getLastNoiseFloor(), 2);
        Serial.print(" course:");
        Serial.print(userConfig.Config.courseIndex);
        Serial.print(" voctIndex:");
        Serial.print(voctIndex);
        Serial.print(" userConfig.Config.arpStep:");
        Serial.print(userConfig.Config.arpStep);
        Serial.println();
    }
}

void operationVCO(uint16_t buttonStates, int8_t encValue, int16_t potValue)
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
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        userConfig.Config.courseIndex = constrain(userConfig.Config.courseIndex + (encValue * 12), 0, 127);
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
        switch (oscMenu)
        {
        case OscillatorSettingMenu::SEL_NOTE:
            userConfig.Config.courseIndex = constrain(userConfig.Config.courseIndex + encValue, 0, 127);
            if ((userConfig.Config.courseIndex % 12) == 0)
            {
                rgbLedControl.setFreq(50);
            }
            else
            {
                rgbLedControl.resetFreq();
            }
            break;
        case OscillatorSettingMenu::SEL_WAVE:
            userConfig.Config.wave = (MultiWaveOsc::Wave)constrain((int8_t)userConfig.Config.wave + encValue, (int8_t)MultiWaveOsc::Wave::SQU, (int8_t)MultiWaveOsc::Wave::MAX);
            for (int i = 0; i < OSC_COUNT; ++i )
            {
                osc[i].setWave(userConfig.Config.wave);
            }
            rgbLedControl.setRainbowLevel((int8_t)userConfig.Config.wave, (int8_t)MultiWaveOsc::Wave::SQU, (int8_t)MultiWaveOsc::Wave::MAX);
            break;
        case OscillatorSettingMenu::SEL_CVIN:
            userConfig.Config.cvInSelect = (CVInSelect)constrain((int)(userConfig.Config.cvInSelect + encValue), (int)CVInSelect::NOUSE, (int)CVInSelect::LAST);
            rgbLedControl.setRainbowLevel((int8_t)userConfig.Config.cvInSelect, (int8_t)CVInSelect::NOUSE, (int8_t)CVInSelect::LAST);
            break;
        case OscillatorSettingMenu::SEL_HOLD:
            userConfig.Config.voctHold = (CVInSelect)constrain((userConfig.Config.voctHold + encValue), 0, 1);
            rgbLedControl.setRainbowLevel(userConfig.Config.voctHold, 0, 1);
            break;
        default:
            break;
        }
    }

    bool lock = updatePotLock(potValue, oscMenu, oscSelect);
    if (lock == false)
    {
        switch (oscMenu)
        {
        case OscillatorSettingMenu::SEL_NOTE:
            userConfig.Config.r7OctSelect = map(potValue, 0, ADC_RESO - 1, 0, r7OctSetMax - 1);
            break;
        case OscillatorSettingMenu::SEL_WAVE:
            int16_t morph = potValue - (ADC_RESO >> 1);
            morph = morph > 36 || morph < -36 ? morph : 0; // 不感レンジ
            userConfig.Config.morphPot = morph;
            break;
        }
    }
}

//////////////////////////////////////////

void processSetup(int16_t in1Value, int16_t cvInValue, int16_t potValue)
{
    const static int16_t dacValues[3] = {0, DAC_RESO >> 1, DAC_RESO - 1};
    int16_t dacSelect = map(potValue, 0, ADC_RESO - 1, 0, 2);
    dac.out1(dacValues[dacSelect]);
    dac.out2(dacValues[dacSelect]);

    static uint16_t dispCount = 0;
    dispCount++;
    if (dispCount == 2000)
    {
        dispCount = 0;
        Serial.print(" vref:");
        Serial.print(adcErrorCorrection.getLastVRef(), 4);
        Serial.print(" noise:");
        Serial.print(adcErrorCorrection.getLastNoiseFloor(), 2);
        Serial.print(" in1Value:");
        Serial.print(in1Value);
        Serial.print(" cv:");
        Serial.print(cvInValue);
        Serial.print(" pot:");
        Serial.print(potValue);
        Serial.println();
    }
}

void operationSetup(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    int8_t index = oscSelect;
    if (buttonStates == ButtonCondition::UA)
    {
    }
    else if (buttonStates == ButtonCondition::UB)
    {
    }
    else if (buttonStates == ButtonCondition::URE)
    {
        changeOsc(OscillatorSelect::OSCA);
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
    }
}

//////////////////////////////////////////

void calibration(float &vref, float &noiseFloor)
{
    // DAC OUT1の出力値が4095=5000mVで調整済みなことが前提
    Serial.println("IN1 Calibration");
    noiseFloor = adcErrorCorrection.getADCAvg16(IN2);
    dac.out1(DAC_RESO - 1);
    sleep_ms(1);
    float adc = adcErrorCorrection.getADCAvg16(IN1);
    Serial.print("ADC at 5V:");
    Serial.print(adc);
    if (adc >= DAC_RESO - 3)
    {
        // 3.26989付近なので半分の電圧から推定しなおす
        dac.out1((DAC_RESO >> 1) + (DAC_RESO >> 3)); // 2.5V
        sleep_ms(1);
        adc = adcErrorCorrection.getADCAvg16(IN1);
        Serial.print(" at 2.5V:");
        Serial.print(adc);
        adc *= 2;
    }
    dac.out1(2048);
    sleep_ms(1);
    vref = adcErrorCorrection.getADC2VRef(adc);
    Serial.print(" vref:");
    Serial.print(vref, 4);
    Serial.print(" noiseFloor:");
    Serial.println(noiseFloor);

    adcErrorCorrection.generateLUT(vref, noiseFloor);
}

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == IN1)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            gateEdge.updateEdge(1);
            in1EdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            gateEdge.updateEdge(0);
            in1EdgeLatch = false;
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    if (oscSelect == OscillatorSelect::SETUP)
    {
        return;
    }

    int16_t sum = 0;
    int16_t values[OSC_COUNT] = {0};
    for (int i = 0; i < OSC_COUNT; ++i)
    {
        values[i] = osc[i].getWaveValue();
        sum += values[i] - bias;
    }

    agc.setCurrentLevel(sum);
    dac.out1(agc.getProcessedLevel(sum));
    dac.out2(values[userConfig.Config.arpStep]);
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
    agc.init(DAC_RESO, OSC_COUNT, 0.35);

    rgbLedControl.init(10000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(oscColor);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    for (int i = 0; i < OSC_COUNT; ++i)
    {
        osc[i].init(SAMPLE_FREQ, DAC_BIT);
        osc[i].setWave(userConfig.Config.wave);
        osc[i].startFolding(true);
    }

    // VOCT Calibration
    sleep_ms(100);
    if (gpio_get(BTN_A) == false)
    {
        // while (!Serial)
        // {
        // }

        float vref = 0.0;
        float noiseFloor = 0.0;
        calibration(vref, noiseFloor);
        systemConfig.Config.noiseFloor = noiseFloor;
        systemConfig.Config.vRef = vref;
        systemConfig.saveUserConfig();
        oscSelect = OscillatorSelect::SETUP;
    }

    // SETUP Mode
    else if (gpio_get(BTN_B) == false)
    {
        oscSelect = OscillatorSelect::SETUP;
    }

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    gpio_set_irq_enabled(IN1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);

}

void loop()
{
    int8_t encValue = enc.getDirection();
    int16_t in1Value = in1.analogReadDirectFast();
    // int16_t in2Value = in2.analogReadDirectFast();
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();

    agc.update(3);

    if (oscSelect == OscillatorSelect::SETUP)
    {
        processSetup(in1Value, cvInValue, potValue);
    }
    else
    {
        processVCO(in1Value, cvInValue, potValue);
    }

    rgbLedControl.process();
    tight_loop_contents();
    sleep_us(100);
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

    if (oscSelect == OscillatorSelect::SETUP)
    {
        operationSetup(buttonStates, encValue, potValue);
    }
    else
    {
        operationVCO(buttonStates, encValue, potValue);
    }

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
