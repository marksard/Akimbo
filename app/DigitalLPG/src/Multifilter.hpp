/*!
 * MultiFilter
 * The code is based on the following: 
 * https://www.utsbox.com/?page_id=728
 * 2-pole (Bi-Quad) type
 */

#pragma once
#include <Arduino.h>

class MultiFilter
{
private:
	// フィルタの係数
	float a1, a2, b0, b1, b2;
	float a0_inv; // a0の逆数を事前計算して保持する
	
	// 事前計算用の定数
	float two_pi_div_sr; // 2 * PI / samplingRate を保持する変数

	// バッファ
	float out1, out2;
	float in1, in2;

public:
	MultiFilter(float samplingRate = 44100.0f)
	{
		two_pi_div_sr = 2.0f * 3.14159265f / samplingRate;
		
		// メンバー変数を初期化
		a0_inv = 1.0f; 
		a1 = 0.0f;
		a2 = 0.0f;
		b0 = 1.0f;
		b1 = 0.0f;
		b2 = 0.0f;

		in1 = 0.0f;
		in2 = 0.0f;

		out1 = 0.0f;
		out2 = 0.0f;
	}

	inline float Process(float in)
	{
		float out = (b0 * in + b1 * in1 + b2 * in2 - a1 * out1 - a2 * out2) * a0_inv;

		in2 = in1;
		in1 = in;

		out2 = out1;
		out1 = out;

		return out;
	}

	void LowPass(float freq, float q)
	{
		float omega = freq * two_pi_div_sr;
		
		float sin_omega = sinf(omega);
		float cos_omega = cosf(omega);
		
		float alpha = sin_omega / (2.0f * q);

		float a0 = 1.0f + alpha;
		a0_inv = 1.0f / a0;

		a1 = -2.0f * cos_omega;
		a2 = 1.0f - alpha;
		
		float b_common = (1.0f - cos_omega) * 0.5f;
		b0 = b_common;
		b1 = 1.0f - cos_omega;
		b2 = b_common;
	}
};
