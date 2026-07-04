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
    SlopeGenerator(float samplingFreq = 44100.0f)
    {
        init(samplingFreq);
    }

    void init(float samplingFreq)
    {
        current_v = 0.0f;
        is_rising = false;
        last_gate = false;
        eoc_pulse = false;
        is_active = false;
        rise_time_sec = 0.005f;
        fall_time_sec = 0.01f;
        internal_shape = 0.0f;
        eoc_duration_samples = 10;
        eoc_counter = 0;
        if (samplingFreq > 0.0f)
        {
            sample_rate = samplingFreq;
            updateSteps();
            setEocWidth(0.001f); // 1ms
        }
    }

    void setEocWidth(float width_sec)
    {
        if (width_sec < 0.0f)
            width_sec = 0.001f;

        eoc_duration_samples = (uint32_t)(width_sec * sample_rate);
        if (eoc_duration_samples == 0)
            eoc_duration_samples = 10;
    }

    inline void update(float rise_time, float fall_time, float shape)
    {
        // ~300秒に設定（伸ばせると思う）
        rise_time_sec = constrain(rise_time, 0.00, 300.0);
        fall_time_sec = constrain(fall_time, 0.00, 300.0);

        if (shape > 1.0f)
            shape = 1.0f;
        if (shape < -1.0f)
            shape = -1.0f;

        internal_shape = -shape;
        updateSteps();
    }

    inline float process(bool gate)
    {
        // Trigger style EOC
        // if (eoc_counter > 0)
        // {
        //     eoc_counter--;
        //     if (eoc_counter == 0)
        //     {
        //         eoc_pulse = false; // 指定サンプル数経過したらパルスOFF
        //     }
        // }

        // トリガー検出（ゲートの立ち上がり）
        bool trigger = gate && !last_gate;
        last_gate = gate;

        // セルフサイクルの処理（動作中でなく、かつCycle有効ならトリガー）
        if (cycle && !is_active && current_v <= 0.0001f)
        {
            trigger = true;
        }

        // トリガーが入ったらRise（上昇）を開始
        if (trigger && !is_rising)
        {
            is_rising = true;
            is_active = true;
        }

        // --- 充放電の計算 ---
        if (is_active)
        {
            if (is_rising)
            {
                eoc_pulse = false; // Gate style EOC

                float scale = (1.0f - internal_shape) * (1.0f - current_v) + (internal_shape + 1.0f) * current_v;

                if (scale < 0.01f)
                    scale = 0.01f;

                current_v += rise_step_inv * scale;

                if (current_v >= 1.0f)
                {
                    current_v = 1.0f;
                    is_rising = false;
                }
            }
            else
            {
                // 下降の計算
                float inv_v = 1.0f - current_v;
                float scale = (1.0f - internal_shape) * (1.0f - inv_v) + (internal_shape + 1.0f) * inv_v;

                if (scale < 0.01f)
                    scale = 0.01f;

                current_v -= fall_step_inv * scale;

                // 終了
                if (current_v <= 0.0f)
                {
                    current_v = 0.0f;
                    is_active = false;

                    eoc_pulse = true;
                    eoc_counter = eoc_duration_samples;
                }
            }
        }

        return current_v;
    }

    // inline bool isRising() const { return is_rising; }
    inline float getValue() const { return current_v; }
    inline bool getEOC() const { return eoc_pulse; }
    inline void toggleCycle() { cycle = !cycle; }

private:
    float sample_rate;
    float current_v; // 現在の出力電圧状態 (0.0f ～ 1.0f)
    bool is_rising;
    bool last_gate;
    bool eoc_pulse;
    bool is_active;
    bool cycle;

    // --- 事前計算（キャッシュ）用メンバ変数 ---
    float rise_time_sec;  // 現在設定されている上昇時間（秒）
    float fall_time_sec;  // 現在設定されている下降時間（秒）
    float rise_step_inv;  // 上昇時の基本ステップ幅（逆数キャッシュ）
    float fall_step_inv;  // 下降時の基本ステップ幅（逆数キャッシュ）
    float internal_shape; // 符号反転済みの形状パラメータ

    uint32_t eoc_duration_samples;
    uint32_t eoc_counter;

    // ステップ幅とパルス幅の再計算
    void updateSteps()
    {
        float r = (rise_time_sec < 0.0001f) ? 0.0001f : rise_time_sec;
        float f = (fall_time_sec < 0.0001f) ? 0.0001f : fall_time_sec;

        rise_step_inv = 1.0f / (r * sample_rate);
        fall_step_inv = 1.0f / (f * sample_rate);
    }
};
