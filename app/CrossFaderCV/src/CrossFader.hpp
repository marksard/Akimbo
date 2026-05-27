/*!
 * CrossFader
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <math.h>
#include <stdint.h>

// Equal PowerタイプクロスフェーダーCV出力計算クラス
// 入力範囲は12bit、出力範囲はAkimbo 12bit DACの仕様±5Vの上半分（0~5V）
// の11bit量で計算し、オフセットして出力
class CrossFader
{
public:
    static const uint16_t INPUT_RESO = 4096;
    static const uint16_t INPUT_HALF = INPUT_RESO >> 1;
    static const uint16_t OUTPUT_RESO = 2048;

private:
    static uint16_t xFadeHalf[INPUT_HALF];

public:
    inline static void init()
    {
        for (uint16_t i = 0; i < INPUT_HALF; i++)
        {
            float x = (float)i / (float)(INPUT_HALF - 1);
            float theta = x * (M_PI / 2.0f);
            float g = cosf(theta);
            uint16_t v = (uint16_t)lroundf(g * (OUTPUT_RESO - 1));
            xFadeHalf[i] = v;
        }
    }

    inline static uint16_t gainA(uint16_t n)
    {
        return xFadeHalf[n >> 1] + OUTPUT_RESO;
    }

    // B側ゲイン（Aの逆インデックス）
    inline static uint16_t gainB(uint16_t n)
    {
        return gainA(INPUT_RESO - 1 - n);
    }
};

uint16_t CrossFader::xFadeHalf[CrossFader::INPUT_HALF];
