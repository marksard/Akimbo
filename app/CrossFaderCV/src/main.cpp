/*!
 * CrossFaderCV
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
#include "lib/pwm_wrapper.h"
#include "lib/ValueLock.hpp"
#include "gpio_mapping.h"
#include "basic_definition.h"
#include "SystemConfig.hpp"
#include "Helper.h"
#include "CrossFader.hpp"

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
// static SmoothAnalogRead in1;
static SmoothAnalogRead cvIn;
// static SmoothAnalogRead in2;
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
CrossFader crossFader;
uint16_t crossValue = DAC_RESO >> 1;

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

    enc.init(EC1B, EC1A, true);
    buttons[0].init(BTN_A);
    buttons[0].setHoldTime(350);
    buttons[1].init(BTN_B);
    buttons[1].setHoldTime(350);
    buttons[2].init(BTN_RE, false, false, true);
    buttons[2].setHoldTime(500);
    // in1.init(IN1);
    // in2.init(IN2);
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
    gpio_init(IN1);
    gpio_init(IN2);
    gpio_set_irq_enabled(IN1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(IN2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection();
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

    tight_loop_contents();
    sleep_ms(10);
}
