#pragma once
#include <FastNoise\FastNoise.h>
#include <FastSIMD\FastSIMD.h>

class Noise
{
public:
	Noise() = delete;
	Noise(int seed);
	~Noise() = default;

	float getRandom();
	float getNoise2D(float x, float y) const;
	float getNoise3D(float x, float y, float z) const;
private:
	int seed_;
	const FastNoise::SmartNode<FastNoise::Simplex> noise_;
};

