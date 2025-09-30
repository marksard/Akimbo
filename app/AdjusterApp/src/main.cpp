/*!
 * DoubleDAC Adjustment Tool
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

enum ButtonCondition
{
    // 各ボタンの押下状態と各ボタンの組み合わせ
    // Button State: 0:None 1:Button down 2:Button up 3:Holding 4:Holded
    // U: button up
    // D: button down
    // H: holging
    // L: holded (leaved)
    // 0xMABR (Mode, A, B, RE(RotaryEncoder) button)
    NONE    = 0x0000,
    UA      = 0x0200,
    UB      = 0x0020,
    URE     = 0x0002,
    HA      = 0x0300,
    HB      = 0x0030,
    HA_HB   = 0x0330,
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
static RGBLEDPWMControl::MenuColor menuColor = RGBLEDPWMControl::MenuColor::RED;
static Mcp4922SwSpi dac;
static ADCErrorCorrection adcErrorCorrection;
static EEPROMConfigIO<SystemConfig> systemConfig(0);

//////////////////////////////////////////

void calibration(float &vref, float &noiseFloor)
{
    Serial.println("IN2 Calibration");
    noiseFloor = adcErrorCorrection.getADCAvg16(IN1);
    dac.out1(4095);
    sleep_ms(500);
    float adc = adcErrorCorrection.getADCAvg16(IN2);
    Serial.print("ADC at 5V:");
    Serial.print(adc);
    if (adc >= 4093)
    {
        // 3.26989付近なので半分の電圧から推定しなおす
        dac.out1(2048 + 1024);
        sleep_ms(500);
        adc = adcErrorCorrection.getADCAvg16(IN2);
        Serial.print(" at 2.5V:");
        Serial.print(adc);
        adc *= 2;
    }
    dac.out1(2048);

    vref = adcErrorCorrection.getADC2VRef(adc);
    adcErrorCorrection.generateLUT(vref, noiseFloor);
    Serial.print(" vref:");
    Serial.print(vref, 4);
    Serial.print(" noiseFloor:");
    Serial.println(noiseFloor);
}

void setup()
{
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

    rgbLedControl.init(20000, PWM_BIT, LED_R, LED_G, LED_B);
    rgbLedControl.setMenuColor(menuColor);
    rgbLedControl.ignoreMenuColor(false);
    rgbLedControl.setWave(MiniOsc::Wave::TRI);

    systemConfig.initEEPROM();
    systemConfig.loadUserConfig();

    adcErrorCorrection.init(systemConfig.Config.vRef, systemConfig.Config.noiseFloor);

    ///////////////////////
    while (!Serial)
    {
    }
    float vref = 0.0;
    float noiseFloor = 0.0;
    calibration(vref, noiseFloor);
    sleep_ms(100);
    ///////// raw adc input
    Serial.println("out,raw_adc,raw_diff,cor_adc,cor_diff");
    int16_t halfReso = DAC_RESO >> 1;
    for (int i = 0; i < halfReso; ++i)
    {
        dac.out1(halfReso + i);
        sleep_ms(1);
        int16_t raw_adc = adcErrorCorrection.getADCAvg16(IN2);
        int16_t raw_diff = (i * 2) - raw_adc;
        Serial.print(i * 2);
        Serial.print(",");
        Serial.print(raw_adc);
        Serial.print(",");
        Serial.print(raw_diff);
        int16_t cor_adc = (int)adcErrorCorrection.correctedAdc(raw_adc);
        int16_t cor_diff = (i * 2) - cor_adc;
        Serial.print(",");
        Serial.print(cor_adc);
        Serial.print(",");
        Serial.print(cor_diff);
        Serial.println();
    }
}

void loop()
{
    int8_t encValue = enc.getDirection();
    int16_t in1Value = in1.analogReadDirectFast();
    int16_t in2Value = in2.analogReadDirectFast();
    int16_t cvInValue = cvIn.analogReadDirectFast() - (ADC_RESO >> 1);
    int16_t potValue = pot.analogReadDirectFast();
    uint8_t btnA = buttons[0].getState();
    uint8_t btnB = buttons[1].getState();
    uint8_t btnRE = buttons[2].getState();
    uint16_t buttonStates = (btnA << 8) + (btnB << 4) + btnRE;
    Serial.print(" buttonStates:");
    Serial.print(buttonStates);
    Serial.print(" encValue:");
    Serial.print(encValue);
    Serial.print(" potValue:");
    Serial.print(potValue);
    Serial.print(" in1Value:");
    Serial.print(in1Value);
    Serial.print(" in2Value:");
    Serial.print(in2Value);
    Serial.println();

    // rgbLedControl.setGLevelMap(potValue, 0, ADC_RESO - 1);
    // rgbLedControl.update();
    // rgbLedControl.process();

    tight_loop_contents();
    // sleep_us(50);
    sleep_ms(50);
}
