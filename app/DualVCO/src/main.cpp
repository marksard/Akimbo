/*!
 * DualDCO
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

enum OscillatorSelect
{
    OSCA = 0,
    OSCB,
    SETUP,
    MAX = SETUP
};

enum OscillatorSettingMenu
{
    SEL_NOTE = 0,
    SEL_WAVE,
    SEL_CVIN,
    SEL_MAX = SEL_CVIN,
};

enum CVInSelect
{
    NOUSE = 0,
    MORPH,
    FM,
    SYNC,
    LAST = SYNC
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
static volatile bool cvEdgeLatch = false;

// UIほか
static OscillatorSettingMenu oscMenu = OscillatorSettingMenu::SEL_NOTE;
static RGBLEDPWMControl::MenuColor oscColor = RGBLEDPWMControl::MenuColor::CYAN;
static OscillatorSelect oscSelect = OscillatorSelect::OSCA;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

// 機能
#define OSC_COUNT 2
static MultiWaveOsc osc[OSC_COUNT];
static int16_t setupAdc = 0;

// ユーザー設定（EEPROM保存用）
struct UserConfig
{
    char ver[15] = "D_VCLFO_000\0"; // 構造体最初の15バイトは保存データ識別子。固有の文字列にすること。
    int8_t courseIndex[OSC_COUNT];
    float fineTune[OSC_COUNT];
    int16_t morphCVMod[OSC_COUNT];
    int16_t morphPot[OSC_COUNT];
    CVInSelect cvInSelect[OSC_COUNT];
    MultiWaveOsc::Wave wave[OSC_COUNT];

    UserConfig()
    {
        for (int i = 0; i < OSC_COUNT; ++i)
        {
            courseIndex[i] = 36;
            fineTune[i] = 0.0f;
            morphCVMod[i] = 0;
            morphPot[i] = 0;
            cvInSelect[i] = CVInSelect::MORPH;
            wave[i] = MultiWaveOsc::Wave::TRI;
        }
    }
};
static EEPROMConfigIO<UserConfig> userConfig(64); // systemConfigから64バイト開けておく

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
        default:
            break;
        }
    }
    else if (oscSelect == OscillatorSelect::OSCB)
    {
        oscColor = RGBLEDPWMControl::MenuColor::MAGENTA;
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

void toggleOsc()
{
    oscSelect = oscSelect == OscillatorSelect::OSCA ? OscillatorSelect::OSCB : OscillatorSelect::OSCA;
    oscMenu = OscillatorSettingMenu::SEL_NOTE;
    updateMenuColor();
}

bool updatePotLock(int16_t potValue, int8_t sel1, int8_t sel2)
{
    static int8_t lastSel1 = sel1;
    static int8_t lastSel2 = sel2;

    if (lastSel1 != sel1 || lastSel2 != sel2)
    {
        potLock.setLock(true);
        lastSel1 = sel1;
        lastSel2 = sel2;
        return true;
    }

    return potLock.update(potValue);
}


//////////////////////////////////////////

void processVCO(int16_t in1Value, int16_t in2Value, int16_t cvInValue, int16_t potValue)
{
    int16_t vOcts[2] = {0};
    vOcts[0] = in1Value;
    vOcts[1] = in2Value;

    for (int i = 0; i < OSC_COUNT; ++i)
    {
        float voctPowV = adcErrorCorrection.voctPow(vOcts[i]);
        float vOctFreq = 0.0f;
        if (userConfig.Config.courseIndex[i] <= 1)
        {
            vOctFreq = userConfig.Config.fineTune[i] * voctPowV;
        }
        else
        {
            vOctFreq = (osc[i].getFreqFromNoteIndex(userConfig.Config.courseIndex[i]) + userConfig.Config.fineTune[i]) * voctPowV;
            // vOctFreq = (osc[i].getFreqFromNoteIndex(courseIndex[i])) * powV;
        }

        if (userConfig.Config.cvInSelect[i] == CVInSelect::FM)
        {
            vOctFreq += cvInValue >> 1;
        }
        if (userConfig.Config.cvInSelect[i] == CVInSelect::SYNC)
        {
            if (cvEdgeLatch)
            {
                cvEdgeLatch = false;
                osc[i].reset();
            }
        }
        if (userConfig.Config.cvInSelect[i] == CVInSelect::MORPH)
        {
            userConfig.Config.morphCVMod[i] = cvInValue;
            osc[i].setFolding(userConfig.Config.morphCVMod[i] + userConfig.Config.morphPot[i]);
            osc[i].setPhaseShift(userConfig.Config.morphCVMod[i] + userConfig.Config.morphPot[i]);
            osc[i].setPulseWidth(userConfig.Config.morphCVMod[i] + userConfig.Config.morphPot[i]);
        }
        else
        {
            osc[i].setFolding(userConfig.Config.morphPot[i]);
            osc[i].setPhaseShift(userConfig.Config.morphPot[i]);
            osc[i].setPulseWidth(userConfig.Config.morphPot[i]);
        }

        osc[i].setFrequency(vOctFreq);
    }

    static uint16_t dispCount = 0;
    dispCount++;
    if (dispCount == 2000)
    {
        dispCount = 0;
        Serial.print(" vref:");
        Serial.print(adcErrorCorrection.getLastVRef(), 4);
        Serial.print(" noise:");
        Serial.print(adcErrorCorrection.getLastNoiseFloor(), 1);
        Serial.print(" setupAdc:");
        Serial.print(setupAdc);
        Serial.print(" fine:");
        Serial.print(userConfig.Config.fineTune[0]);
        // Serial.print(", ");
        // Serial.print(userConfig.Config.fineTune[1]);
        Serial.print(" vOcts:");
        Serial.print(vOcts[0]);
        // Serial.print(", ");
        // Serial.print(vOcts[1]);
        Serial.print(" voctRaw:");
        Serial.print(adcErrorCorrection.correctedAdc(vOcts[0]));
        // Serial.print(", ");
        // Serial.print(adcErrorCorrection.correctedAdc(vOcts[1]));
        Serial.print(" voctPowV:");
        Serial.print(adcErrorCorrection.voctPow(vOcts[0]));
        // Serial.print(", ");
        // Serial.print(adcErrorCorrection.voctPow(vOcts[1]));
        Serial.println();
    }
}

void operationVCO(uint16_t buttonStates, int8_t encValue, int16_t potValue)
{
    int8_t index = oscSelect;
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
        toggleOsc();
    }
    else if (buttonStates == ButtonCondition::HA)
    {
        userConfig.Config.courseIndex[index] = constrain(userConfig.Config.courseIndex[index] + (encValue * 12), 0, 127);
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
            userConfig.Config.courseIndex[index] = constrain(userConfig.Config.courseIndex[index] + encValue, 0, 127);
            if (userConfig.Config.courseIndex[index] == 0 || userConfig.Config.courseIndex[index] == 1)
            {
                rgbLedControl.setFreq(userConfig.Config.fineTune[index]);
            }
            else if ((userConfig.Config.courseIndex[index] % 12) == 0)
            {
                rgbLedControl.setBlink();
            }
            else
            {
                rgbLedControl.resetFreq();
            }
            break;
        case OscillatorSettingMenu::SEL_WAVE:
            userConfig.Config.wave[index] = (MultiWaveOsc::Wave)constrain((int8_t)userConfig.Config.wave[index] + encValue, (int8_t)MultiWaveOsc::Wave::SQU, (int8_t)MultiWaveOsc::Wave::MAX);
            osc[index].setWave(userConfig.Config.wave[index]);
            rgbLedControl.setRainbowLevel((int8_t)userConfig.Config.wave[index], (int8_t)MultiWaveOsc::Wave::SQU, (int8_t)MultiWaveOsc::Wave::MAX);
            break;
        case OscillatorSettingMenu::SEL_CVIN:
            userConfig.Config.cvInSelect[index] = (CVInSelect)constrain((int8_t)(userConfig.Config.cvInSelect[index] + encValue), (int8_t)CVInSelect::NOUSE, (int8_t)CVInSelect::LAST);
            rgbLedControl.setRainbowLevel((int8_t)userConfig.Config.cvInSelect[index], (int8_t)CVInSelect::NOUSE, (int8_t)CVInSelect::LAST);
            break;
        default:
            break;
        }
    }

    bool lock = updatePotLock(potValue, oscMenu, oscSelect);
    if (lock == false)
    {
        float fineTuneRange[2] = {0.0f, 0.0f};
        int8_t index = oscSelect;
        switch (oscMenu)
        {
        case OscillatorSettingMenu::SEL_NOTE:
            // LFO SUPER LOW
            if (userConfig.Config.courseIndex[index] == 0)
            {
                fineTuneRange[0] = 0.000001f;
                fineTuneRange[1] = 0.1f;
            }
            // LFO LOW
            else if (userConfig.Config.courseIndex[index] == 1)
            {
                fineTuneRange[0] = 0.1f;
                fineTuneRange[1] = osc[index].getFreqFromNoteIndex(userConfig.Config.courseIndex[index]);
            }
            // FineTune
            else
            {
                osc[index].getFineTuneRangeFromNoteIndex(userConfig.Config.courseIndex[index], fineTuneRange[0], fineTuneRange[1]);
            }

            userConfig.Config.fineTune[index] = mapFloat(potValue, 0, ADC_RESO - 1, fineTuneRange[0], fineTuneRange[1]);

            if (userConfig.Config.courseIndex[index] == 0 || userConfig.Config.courseIndex[index] == 1)
            {
                rgbLedControl.setFreq(userConfig.Config.fineTune[index]);
            }
            break;
        case OscillatorSettingMenu::SEL_WAVE:
            int16_t morph = potValue - (ADC_RESO >> 1);
            morph = morph > 36 || morph < -36 ? morph : 0; // 不感レンジ
            userConfig.Config.morphPot[(int)(oscSelect)] = morph;
            break;
        }
    }
}

//////////////////////////////////////////

void processSetup(int16_t in1Value, int16_t in2Value, int16_t cvInValue, int16_t potValue)
{
    const static int16_t dacValues[3] = { 0, DAC_RESO >> 1, DAC_RESO - 1 };
    int16_t dacSelect = map(potValue, 0, ADC_RESO - 512, 0, 2);
    dac.out1(dacValues[dacSelect]);
    dac.out2(dacValues[dacSelect]);
    
    static uint16_t dispCount = 0;
    dispCount++;
    if (dispCount == 2000)
    {
        dispCount = 0;
        Serial.print(" lastVref:");
        Serial.print(adcErrorCorrection.getLastVRef(), 4);
        Serial.print(" noise:");
        Serial.print(adcErrorCorrection.getLastNoiseFloor(), 2);
        Serial.print(" setupAdc:");
        Serial.print(setupAdc);
        Serial.print(" in:");
        Serial.print(in1Value);
        Serial.print(",");
        Serial.print(in2Value);
        Serial.print(" in cor:");
        Serial.print(adcErrorCorrection.correctedAdc(in1Value));
        Serial.print(",");
        Serial.print(adcErrorCorrection.correctedAdc(in2Value));
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
        toggleOsc();
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
    sleep_ms(100);
    dac.out1(DAC_RESO - 1);
    sleep_ms(100);
    float adc = adcErrorCorrection.getADCAvg16(IN1);
    sleep_ms(100);
    adc = adcErrorCorrection.getADCAvg16(IN1);
    setupAdc = adc;
    Serial.println("IN1 Calibration");
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
    sleep_ms(100);
    noiseFloor = adcErrorCorrection.getADCMax16(IN2);
    vref = adcErrorCorrection.getADC2VRef(adc);
    Serial.print(" vref:");
    Serial.print(vref, 4);
    Serial.print(" noiseFloor:");
    Serial.println(noiseFloor);

    adcErrorCorrection.generateLUT(vref, noiseFloor);
}

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == CV1)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            cvEdgeLatch = true;
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            cvEdgeLatch = false;
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

    dac.out1(osc[0].getWaveValue());
    dac.out2(osc[1].getWaveValue());
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
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();
    userConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    setupAdc = adcErrorCorrection.getADCAvg16(IN1);

    for (int i = 0; i < OSC_COUNT; ++i)
    {
        osc[i].init(SAMPLE_FREQ, DAC_BIT);
        osc[i].setWave(userConfig.Config.wave[i]);
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
        // while (!Serial)
        // {
        // }

        // sleep_ms(100);
        // Serial.println("out,raw_adc,cor_adc");
        // int16_t halfReso = DAC_RESO >> 1;
        // for (int i = 0; i <= 5; ++i)
        // {
        //     dac.out1(halfReso + (halfReso / 5.0 * i) - 1);
        //     sleep_ms(1);
        //     int16_t raw_adc = adcErrorCorrection.getADCAvg16(IN1);
        //     int16_t cor_adc = (int)adcErrorCorrection.correctedAdc(raw_adc);
        //     Serial.print((halfReso / 5.0 * i));
        //     Serial.print(",");
        //     Serial.print(raw_adc);
        //     Serial.print(",");
        //     Serial.print(cor_adc);
        //     Serial.println();
        // }

        oscSelect = OscillatorSelect::SETUP;
    }

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    gpio_set_irq_enabled(CV1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    updateMenuColor();
}

void loop()
{
    int8_t encValue = enc.getDirection();
    int16_t in1Value = in1.analogReadDirectFast();
    int16_t in2Value = in2.analogReadDirectFast();
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();

    if (oscSelect == OscillatorSelect::SETUP)
    {
        processSetup(in1Value, in2Value, cvInValue, potValue);
    }
    else
    {
        processVCO(in1Value, in2Value, cvInValue, potValue);
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
