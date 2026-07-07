/*!
 * Slope Generator
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

class SlopeGenerator
{
public:
    enum class InputMode
    {
        Trigger,
        Gate,
    };

    SlopeGenerator(float samplingFreq = 44100.0f)
    {
        init(samplingFreq);
    }

    void init(float samplingFreq)
    {
        currentV = 0.0f;
        isRising = false;
        lastGate = false;
        eocPulse = false;
        isActive = false;
        cycle = false;
        inputMode = InputMode::Trigger;
        riseTimeSec = 0.0f;
        fallTimeSec = 0.1f;
        internalShape = 0.0f;
        eocDurationSamples = 10;
        eocCounter = 0;
        if (samplingFreq > 0.0f)
        {
            sampleRate = samplingFreq;
            updateSteps();
            setEocWidth(0.001f); // 1ms
        }
    }

    void setEocWidth(float widthSec)
    {
        if (widthSec < 0.0f)
            widthSec = 0.001f;

        eocDurationSamples = (uint32_t)(widthSec * sampleRate);
        if (eocDurationSamples == 0)
            eocDurationSamples = 10;
    }

    inline void update(float riseTime, float fallTime, float shape)
    {
        // ~300秒に設定（伸ばせると思う）
        riseTimeSec = constrain(riseTime, 0.0, 300.0);
        fallTimeSec = constrain(fallTime, 0.0, 300.0);

        if (shape > 1.0f)
            shape = 1.0f;
        if (shape < -1.0f)
            shape = -1.0f;

        internalShape = -shape;
        updateSteps();
    }

    inline float process(bool gate)
    {
        // Trigger style EOC
        // if (eocCounter > 0)
        // {
        //     eocCounter--;
        //     if (eocCounter == 0)
        //     {
        //         eocPulse = false; // 指定サンプル数経過したらパルスOFF
        //     }
        // }

        bool gateRisingEdge = gate && !lastGate;
        bool gateFallingEdge = !gate && lastGate;
        lastGate = gate;

        if (inputMode == InputMode::Gate)
        {
            processGateMode(gate, gateRisingEdge, gateFallingEdge);
        }
        else
        {
            processTriggerMode(gateRisingEdge);
        }

        // --- 充放電の計算 ---
        if (isActive)
        {
            if (isRising)
            {
                eocPulse = false; // Gate style EOC

                float scale = (1.0f - internalShape) * (1.0f - currentV) + (internalShape + 1.0f) * currentV;

                if (scale < 0.01f)
                    scale = 0.01f;

                currentV += riseStepInv * scale;

                if (currentV >= 1.0f)
                {
                    currentV = 1.0f;
                    isRising = false;
                }
            }
            else
            {
                // 下降の計算
                float invV = 1.0f - currentV;
                float scale = (1.0f - internalShape) * (1.0f - invV) + (internalShape + 1.0f) * invV;

                if (scale < 0.01f)
                    scale = 0.01f;

                currentV -= fallStepInv * scale;

                // 終了
                if (currentV <= 0.0f)
                {
                    currentV = 0.0f;
                    isActive = false;

                    eocPulse = true;
                    eocCounter = eocDurationSamples;
                }
            }
        }

        return currentV;
    }

    // inline bool getIsRising() const { return isRising; }
    inline float getValue() const { return currentV; }
    inline bool getEOC() const { return eocPulse; }
    inline void toggleCycle() { cycle = !cycle; }
    inline void setInputMode(InputMode mode)
    {
        if (inputMode == mode)
        {
            return;
        }

        inputMode = mode;
        resetRuntimeState();
    }
    inline void toggleInputMode()
    {
        inputMode = (inputMode == InputMode::Trigger) ? InputMode::Gate : InputMode::Trigger;
        resetRuntimeState();
    }

private:
    float sampleRate;
    float currentV; // 現在の出力電圧状態 (0.0f ～ 1.0f)
    bool isRising;
    bool lastGate;
    bool eocPulse;
    bool isActive;
    bool cycle;
    InputMode inputMode;

    // --- 事前計算（キャッシュ）用メンバ変数 ---
    float riseTimeSec;   // 現在設定されている上昇時間（秒）
    float fallTimeSec;   // 現在設定されている下降時間（秒）
    float riseStepInv;   // 上昇時の基本ステップ幅（逆数キャッシュ）
    float fallStepInv;   // 下降時の基本ステップ幅（逆数キャッシュ）
    float internalShape; // 符号反転済みの形状パラメータ

    uint32_t eocDurationSamples;
    uint32_t eocCounter;

    void resetRuntimeState()
    {
        currentV = 0.0f;
        isRising = false;
        lastGate = false;
        eocPulse = false;
        isActive = false;
        eocCounter = 0;
    }

    void beginRise()
    {
        isRising = true;
        isActive = true;
    }

    void beginFall()
    {
        isRising = false;
        isActive = currentV > 0.0f;
    }

    void processTriggerMode(bool trigger)
    {
        // セルフサイクルの処理（動作中でなく、かつCycle有効ならトリガー）
        if (cycle && !isActive && currentV <= 0.0001f)
        {
            trigger = true;
        }

        // トリガーが入ったらRise（上昇）を開始
        if (trigger && !isRising)
        {
            beginRise();
        }
    }

    void processGateMode(bool gate, bool gateRisingEdge, bool gateFallingEdge)
    {
        if (gate && (gateRisingEdge || currentV < 1.0f))
        {
            beginRise();
        }
        else if (gateFallingEdge)
        {
            beginFall();
        }
        else if (cycle && !gate && !isActive && currentV <= 0.0001f)
        {
            beginRise();
        }

        if (gate && !isRising && currentV >= 1.0f)
        {
            isActive = false;
        }
    }

    // ステップ幅とパルス幅の再計算
    void updateSteps()
    {
        float r = (riseTimeSec < 0.0001f) ? 0.0001f : riseTimeSec;
        float f = (fallTimeSec < 0.0001f) ? 0.0001f : fallTimeSec;

        riseStepInv = 1.0f / (r * sampleRate);
        fallStepInv = 1.0f / (f * sampleRate);
    }
};
