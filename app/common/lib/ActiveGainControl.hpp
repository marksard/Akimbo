/*!
 * ActiveGainControl class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
// #include <algorithm> // std::max, std::min

/// @brief ActiveGainControl class
/// @details 音量を一定に保つためのクラス。音量が大きくなったらGainを下げる。音量が小さくなったらGainを上げる。
class ActiveGainControl
{
public:
    ActiveGainControl() : _reso(0), _bias(0), _gainMax(0), _divs(0), _peak(0), _gain(0), _level(0) {}

    /// @brief 初期化
    /// @param reso resoはPWMの分解能。12bitなら4096
    /// @param waveMixCount 波形の数。4つの波形を同時に再生する場合は4
    /// @param gainMax Gainの最大値。0.0-1.0の範囲で指定
    void init(int16_t reso, int16_t waveMixCount, float gainMax)
    {
        _reso = reso;
        _bias = reso >> 1;

        _gainMax = (int32_t)(gainMax * (1 << GAIN_SHIFT));
        _divs = _bias << GAIN_SHIFT;

        _peak = max(1, waveMixCount * (_bias - 1));
        _gain = _gainMax;
        _level = 0;
    }

    /// @brief 現在の音量を設定
    /// @param levelL 左チャンネルの音量。バイアスなしsigned
    /// @param levelR 右チャンネルの音量。バイアスなしsigned
    inline void setCurrentLevel(int16_t levelL, int16_t levelR = 0)
    {
        uint16_t l = abs(levelL);
        uint16_t r = abs(levelR);

        _level = max(l, r);
    }

    /// @brief AGCの処理後の音量を取得
    /// @param level 入力レベル。バイアスなしsigned
    /// @return AGCの処理後の音量 バイアス付きunsigned
    inline int16_t getProcessedLevel(int16_t level)
    {
        return constrain(((level * _gain) >> GAIN_SHIFT) + _bias, 0, _reso - 1);
    }

    /// @brief AGC更新
    /// @param decaySpeed ピーク減衰量(1以上)
    inline void update(uint16_t decaySpeed = 1)
    {
        if (_level > _peak)
        {
            _peak = _level;
        }
        else
        {
            _peak = (_peak > decaySpeed) ? (_peak - decaySpeed) : 1;
        }

        int32_t gain = _divs / _peak;

        _gain = (gain < _gainMax) ? gain : _gainMax;
    }

    /// @brief AGCの最大ゲインを設定
    /// @param gainMax (0.0-1.0の範囲で指定)
    inline void setGainMax(float gainMax)
    {
        gainMax = constrain(gainMax, 0.0f, 1.0f);
        _gainMax = (int32_t)(gainMax * (1 << GAIN_SHIFT));
    }

    void print()
    {
        Serial.print("gain:");
        Serial.print(_gain);
        Serial.print(" max:");
        Serial.print(_gainMax);
        Serial.print(" peak:");
        Serial.print(_peak);
        Serial.println();
    }

private:
    constexpr static int16_t GAIN_SHIFT = 10; // gain計算のためのシフト量
    int16_t _reso;
    int16_t _bias;
    int32_t _gainMax;
    int32_t _divs;
    uint16_t _peak;
    int32_t _gain;
    int16_t _level;
};
