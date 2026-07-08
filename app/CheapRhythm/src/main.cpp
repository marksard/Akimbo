/*!
 * CheapRhythm
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

/*
# CheapRhythm

CheapRhythmは、1OSC,1NOISE,1FILTER,1AMP,2ENV構成のドラムシンセ1基の音色をCVで切り替えて出力するドラムマシンファームウェアです。  
トリガーとCVの組み合わせでパターン演奏が可能です。  
LFO出力は内部で接続されていないので、IN2やCVに入れて活用することも可能です。  

## 機能概要

- 入力
  - `IN1` トリガー入力
  - `IN2` 音色選択`BD<->SD<->HH<->TOM`
  - `CV` ディケイ入力
  - `POT` LFO周波数調整(0.01~10Hz)、初期波形：三角波
  - `RE` 音色設定など
- 出力
  - `OUT1` ドラムシンセ出力
  - `OUT2` LFO出力
  - `RGB LED` LFO周期表示、設定表示など

## 使い方

- モード
  - `RE押し込み` バンク切り替え：`LFOモニタ->BD->SD->HH->TOM->LFOモニタ...`
  - `Aボタン` チープモード（FM・フィルターをバイパス）切り替え
  - `Bボタン` 内臓LFO波形変更：`矩形波->のこぎり波->三角波、ノイズ`

- 基音設定
  - `RE操作` 音色調整
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
// #include "lib/EepRomConfigIO.hpp"
// #include "lib/Mcp4922SwSpi.hpp"
#include "lib/Mcp4922HwSpi.hpp"
#include "lib/pwm_wrapper.h"
#include "lib/ValueLock.hpp"
#include "gpio_mapping.h"
#include "basic_definition.h"
// #include "SystemConfig.hpp"
#include "Helper.h"

#include "lib/EdgeChecker.hpp"
#include "lib/MiniOsc.hpp"
#include "CheapRhythm88.hpp"

enum Mode
{
    MONITOR,
    KICK,
    SNARE,
    HIHAT,
    TOM,
    MAX
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
// static SmoothAnalogRead in1;
static SmoothAnalogRead cvIn;
static SmoothAnalogRead in2;
static SmoothAnalogRead pot;
static RGBLEDPWMControl rgbLedControl;
static Mcp4922HwSpi dac;
static ValueLock potLock;
static EdgeChecker gateEdge;

// UIほか
static Mode mode = Mode::MONITOR;
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::RED;

CheapRhythm88 cr88;
const float adcTypeRatio = (double)CheapRhythm88::Type::DRUM_TYPE_MAX / ADC_RESO;
MiniOsc lfo;

//////////////////////////////////////////

void setDrumColor(CheapRhythm88::Type type)
{
    switch (type)
    {
    case CheapRhythm88::Type::DRUM_KICK:
        menuColor = RGBLEDPWMControl::MenuColor::RED;
        rgbLedControl.setMenuColor(menuColor);
        break;
    case CheapRhythm88::Type::DRUM_SNARE:
        menuColor = RGBLEDPWMControl::MenuColor::GREEN;
        rgbLedControl.setMenuColor(menuColor);
        break;
    case CheapRhythm88::Type::DRUM_HIHAT:
        menuColor = RGBLEDPWMControl::MenuColor::BLUE;
        rgbLedControl.setMenuColor(menuColor);
        break;
    case CheapRhythm88::Type::DRUM_TOM:
        menuColor = RGBLEDPWMControl::MenuColor::YELLOW;
        rgbLedControl.setMenuColor(menuColor);
        break;
    default:
        break;
    }
}

void changeMode(uint8_t addValue)
{
    if (addValue == 0)
        return;

    mode = (Mode)constrainCyclic(mode + addValue, 0, (int)Mode::MAX - 1);

    rgbLedControl.resetFreq();
    rgbLedControl.resetLevel();

    switch (mode)
    {
    case Mode::MONITOR:
        break;
    case Mode::KICK:
        setDrumColor(CheapRhythm88::Type::DRUM_KICK);
        break;
    case Mode::SNARE:
        setDrumColor(CheapRhythm88::Type::DRUM_SNARE);
        break;
    case Mode::HIHAT:
        setDrumColor(CheapRhythm88::Type::DRUM_HIHAT);
        break;
    case Mode::TOM:
        setDrumColor(CheapRhythm88::Type::DRUM_TOM);
        break;
    default:
        break;
    }
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

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    uint16_t audioOut = cr88.process();
    dac.out1(audioOut);
    dac.out2(lfo.getWaveValue());
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
    in2.init(IN2);
    cvIn.init(CV1);
    pot.init(POT1);
    dac.init(SPI_MOSI, SPI_SCK, SPI_CS);
    gateEdge.init(IN1);

    rgbLedControl.init(20000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    cr88.init(SAMPLE_FREQ, DAC_BIT, ADC_RESO);

    lfo.init(SAMPLE_FREQ, DAC_BIT);
    lfo.setWave(MiniOsc::Wave::TRI);
    lfo.setFrequency(1);
    lfo.setLevel(12);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    int8_t encValue = enc.getDirection();
    // int16_t in1Value = in1.analogReadDirectFast();
    int16_t in2Value = in2.analogReadDirectFast();
    int16_t cvInValue = cvIn.analogReadDirectFast(); // - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();

    static CheapRhythm88::Type drum = CheapRhythm88::Type::DRUM_KICK;
    static int16_t edgeCV = 0;
    static int16_t edgePitch = 0;
    bool in1EdgeLatch = gateEdge.isEdgeHigh();
    if (in1EdgeLatch)
    {
        cr88.start();
        in1EdgeLatch = false;
        edgeCV = cvInValue;

        uint8_t drumtype = in2Value * adcTypeRatio;
        int16_t pitch = (drumtype - (int)drumtype) * 100;
        edgePitch = pitch;
        drum = (CheapRhythm88::Type)(drumtype);
    }
    cr88.update(drum, edgePitch, edgeCV, gateEdge.getValue());

    // rgbLedControl.setRLevelMap(gateEdge.getValue() ? drumtype : 0, CheapRhythm88::Type::DRUM_KICK, CheapRhythm88::Type::DRUM_TYPE_MAX - 1);
    // rgbLedControl.setBLevelMap(edgeCV, 0, ADC_RESO - 1);

    rgbLedControl.process();
    tight_loop_contents();
    sleep_us(50);

    // static uint16_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.println();
    // }
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

    if (buttonStates == ButtonCondition::URE)
    {
        changeMode(1);
    }
    else if (buttonStates == ButtonCondition::UA)
    {
        cr88.setCheapMode(cr88.getCheapMode() ? false : true);
    }
    else if (buttonStates == ButtonCondition::UB)
    {
        lfo.setWave((MiniOsc::Wave)constrainCyclic((int8_t)(lfo.getWave() + 1), (int8_t)MiniOsc::Wave::SQU, (int8_t)MiniOsc::Wave::MAX));
    }

    switch (mode)
    {
    case Mode::MONITOR:
        break;
    case Mode::KICK:
        if (buttonStates == ButtonCondition::NONE)
        {
            cr88.addKickParamA(encValue);
        }
        break;
    case Mode::SNARE:
        if (buttonStates == ButtonCondition::NONE)
        {
            cr88.addSnareParamA(encValue);
        }
        break;
    case Mode::HIHAT:
        if (buttonStates == ButtonCondition::NONE)
        {
            cr88.addHihatParamA(encValue);
        }
        break;
    case Mode::TOM:
        if (buttonStates == ButtonCondition::NONE)
        {
            cr88.addTomParamA(encValue);
        }
        break;
    default:
        break;
    }

    bool lock = updatePotLock(potValue, mode, 0);
    if (lock == false)
    {
        float lfoFreq = 0.0f;
        switch (mode)
        {
        case Mode::MONITOR:
            lfoFreq = mapFloat(potValue, 0, ADC_RESO - 1, 0.01f, 10.0f);
            lfo.setFrequency(lfoFreq);
            rgbLedControl.setGLevel(8);
            rgbLedControl.setFreq(lfoFreq);
            break;
        default:
            break;
        }
    }

    rgbLedControl.update();
    tight_loop_contents();
    sleep_ms(10);
}
