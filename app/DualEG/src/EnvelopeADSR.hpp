/*!
 * Envelope ADSR
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

class EnvelopeADSR
{
public:
    EnvelopeADSR()
    {
    }

    void init(float maxLevel)
    {
        _attackTime = 0;
        _decayTime = 0;
        _sustainLevel = 0;
        _releaseTime = 0;
        _attackCurveRatio = 3.0;
        _attackCurveExponent = exp(_attackCurveRatio);
        _decayCurveRatio = 3.0;
        _releaseCurveRatio = 3.0;
        _attackTimeRatio = 1.0;
        _decayTimeRatio = 1.0;
        _releaseTimeRatio = 1.0;

        _stageTime = 0;
        _stage = EnvelopeStage::END;
        _lastGate = 0;
        _lastMsec = 0;
        _level = 0;
        _lastLevel = 0;
        _maxLevel = maxLevel;

        _isExp = true;
    }

    /// @brief
    /// @param attack reso:maxLevel unit: millisec
    /// @param decay reso:maxLevel unit: millisec
    /// @param sustain reso:maxLevel unit: output level(_maxLevel)
    /// @param release reso:maxLevel unit: millisec
    void set(uint16_t attack, uint16_t decay, uint16_t sustain, uint16_t release)
    {
        setAttack(attack);
        setDecay(decay);
        setSustain(sustain);
        setRelease(release);
    }

    void setAttack(uint16_t value)
    {
        _attackTime = value;
        setAttackTimeRatio();
    }
    void setDecay(uint16_t value)
    {
        _decayTime = value;
        setDecayTimeRatio();
    }
    void setSustain(uint16_t value) { _sustainLevel = min(value, _maxLevel); }
    void setRelease(uint16_t value)
    {
        _releaseTime = value;
        setReleaseTimeRatio();
    }

    void setAttackCurveRatio(float value)
    {
        _attackCurveRatio = value;
        setAttackTimeRatio();
    }
    void setDecayCurveRatio(float value)
    {
        _decayCurveRatio = value;
        setDecayTimeRatio();
    }
    void setReleaseCurveRatio(float value)
    {
        _releaseCurveRatio = value;
        setReleaseTimeRatio();
    }

    void setExp(bool isExp)
    {
        _isExp = isExp;
    }
    
    uint16_t getLevel() { return (uint16_t)_level; }
    uint16_t getAttack() { return _attackTime; }
    uint16_t getDecay() { return _decayTime; }
    uint16_t getSustain() { return _sustainLevel; }
    uint16_t getRelease() { return _releaseTime; }
    bool getExp() { return _isExp; }

    bool next(uint8_t gate)
    {
        bool result = false;
        ulong msec = millis();

        if (_lastGate == 0 && gate == 1)
        {
            _lastGate = gate;
            _stage = EnvelopeStage::ATTACK;
            _stageTime = 0;
            _lastMsec = msec;
            _level = 0;
            result = true;
        }
        else if (_lastGate == 1 && gate == 0)
        {
            _lastGate = gate;
            _stage = EnvelopeStage::RELEASE;
            _stageTime = 0;
            _lastMsec = msec;
        }
        int16_t stepTime = msec - _lastMsec;

        switch (_stage)
        {
        case EnvelopeStage::ATTACK:
            if (_decayTime < 2)
            {
                if (_isExp)
                {
                    _level = expCurveAttack(_stageTime, _attackTimeRatio) * _sustainLevel;
                }
                else
                {
                    _level = min((float)_stageTime * (_sustainLevel / _attackTime), _sustainLevel);
                }
            }
            else
            {
                if (_isExp)
                {
                    _level = expCurveAttack(_stageTime, _attackTimeRatio) * _maxLevel;
                }
                else
                {
                    _level = min((float)_stageTime * (_maxLevel / _attackTime), _maxLevel);
                }
            }

            _lastLevel = _level;
            if (_attackTime < _stageTime)
            {
                _stage = EnvelopeStage::DECAY;
                _stageTime = 0;
            }

            _stageTime += stepTime;
            break;
        case EnvelopeStage::DECAY:
            if (_isExp)
            {
                _level = (expCurve(_stageTime, _decayTimeRatio) * (_maxLevel - _sustainLevel)) + _sustainLevel;
            }
            else
            {
                _level = max(_maxLevel - (_stageTime * (_maxLevel / _decayTime)), _sustainLevel);
            }

            _lastLevel = _level;
            if (_decayTime < _stageTime)
            {
                _stage = EnvelopeStage::SUSTAIN;
                _stageTime = 0;
            }

            _stageTime += stepTime;
            break;
        case EnvelopeStage::SUSTAIN:
            _level = _sustainLevel;
            _lastLevel = _level;
            break;
        case EnvelopeStage::RELEASE:
            if (_isExp)
            {
                _level = (expCurve(_stageTime, _releaseTimeRatio) * (_lastLevel));
            }
            else
            {
                _level = max(_lastLevel - (_stageTime * (_maxLevel / _releaseTime)), 0);
            }

            if (_releaseTime < _stageTime)
            {
                _stage = EnvelopeStage::END;
                _stageTime = 0;
                _level = 0;
            }

            _stageTime += stepTime;
            break;
        default:
            break;
        }

        _lastMsec = msec;

        return result;
    }

protected:
    inline void setAttackTimeRatio() { _attackTimeRatio = (_attackCurveRatio / _attackTime); _attackCurveExponent = exp(_attackCurveRatio); }
    inline void setDecayTimeRatio() { _decayTimeRatio = (_decayCurveRatio / _decayTime); }
    inline void setReleaseTimeRatio() { _releaseTimeRatio = (_releaseCurveRatio / _releaseTime); }

    inline float expCurve(float currentTime, float timeRatio)
    {
        return exp(-1.0 * currentTime * timeRatio);
    }

    inline float expCurveAttack(float currentTime, float timeRatio)
    {
        return exp(currentTime * timeRatio) / _attackCurveExponent;
    }

protected:
    enum EnvelopeStage
    {
        ATTACK = 0,
        DECAY,
        SUSTAIN,
        RELEASE,
        END
    };

    uint16_t _attackTime;
    uint16_t _decayTime;
    uint16_t _sustainLevel;
    uint16_t _releaseTime;
    float _attackCurveRatio;
    float _attackCurveExponent;
    float _decayCurveRatio;
    float _releaseCurveRatio;
    float _attackTimeRatio;
    float _decayTimeRatio;
    float _releaseTimeRatio;

    uint16_t _stageTime;
    EnvelopeStage _stage;
    uint8_t _lastGate;
    ulong _lastMsec;
    float _level;
    float _lastLevel;
    float _maxLevel;
    bool _isExp;
};
