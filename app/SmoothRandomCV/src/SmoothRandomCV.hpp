/*!
 * SmoothRandomCV class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "lib/RandomFast.hpp"

class SmoothRandomCV
{
public:
    SmoothRandomCV()
    {
    }

    SmoothRandomCV(uint16_t reso)
    {
        init(reso);
    }

    void init(uint16_t reso)
    {
        _lastMillis = 0;
        _holdMillis = 0;
        _lastLevel = 0.0;
        _holdLevel = 0.0;

        _curve = 3.0;
        _isSmoothing = true;
        _isFluctuation = true;
        _resoM1 = reso - 1;
        _maxLevel = _resoM1;

        _rnd.randomSeed(millis() * (analogRead(A0) + 1));
        _flactLevel = _rnd.getRandom16(0, 10001) / 10000.0f;
    }

    void setMaxLevel(uint16_t value) {
        _maxLevel = constrain(value, 0, _resoM1);
    }

    void setCurve(float value)
    {
        _curve = value;
    }

    void enableSmoothing(bool enable)
    {
        _isSmoothing = enable;
    }

    void enableFluctuation(bool enable)
    {
        _isFluctuation = enable;
    }

    float getLevel() { return _lastLevel; }
    bool isFluctuation() { return _isFluctuation; }

    // isTrigger有効ならtriggerに従う。それ以外は内部タイマーで自動更新
    // 更新タイミングで次の目標levelとfreqをランダムで決め、updateコールごとに
    // 簡易ローパスフィルタによってその目標値に近づくようにしている
    // curveはフィルターの急峻さ
    // 出力は周波数と12bitアナログ値
    bool update(bool trigger, bool isTrigger = true)
    {
        bool on = isTrigger ? trigger : ready();
        if (on)
        {
            if (_isFluctuation == false)
            {
                _flactLevel = _rnd.getRandom16(0, 10001) / 10000.0f;
            }
            else
            {
                _flactLevel = fluctuation1f(_flactLevel);
            }

            _holdLevel = _flactLevel * _maxLevel;
        }

        if (_isSmoothing == true)
        {
            float curveRight = _curve * 0.0001;
            float curveLeft = 1.0 - curveRight;
            _lastLevel = (_lastLevel * curveLeft) + (_holdLevel * curveRight);
        }
        else
        {
            _lastLevel = _holdLevel;
        }

        return on;
    }

private:
    RandomFast _rnd;

    float _lastLevel;
    ulong _lastMillis;
    float _holdLevel;
    ulong _holdMillis;

    float _curve;
    bool _isSmoothing;
    bool _isFluctuation;
    uint16_t _maxLevel;
    uint16_t _resoM1;

    float _flactLevel;

    bool ready()
    {
        ulong tm = millis();
        bool result = (tm - _lastMillis) > _holdMillis ? true : false;
        if (result)
        {
            _holdMillis = _rnd.getRandom16(10, 1000);
            _lastMillis = tm;
        }

        return result;
    }

    float fluctuation1f(float x) {
        if (x <= 0.1f || x >= 0.9f) {
            return (float)_rnd.getRandom16(0, 10001) / 10000.0f;  // 0.0～1.0の乱数
        }

        if (x < 0.5f) {
            return x + 2.0f * x * x;
        }
        // x < 0.95f
        return x - 2.0f * (1.0f - x) * (1.0f - x);
    }
};
