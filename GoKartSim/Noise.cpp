#include "Noise.h"
#include <cstdlib>

Noise::Noise(int seed) :
	seed_{ seed },
	noise_{ FastNoise::New<FastNoise::Simplex>(FastSIMD::eLevel::Level_Null) }
{
	srand(seed);
}

float Noise::getRandom() const
{
	auto random_float = rand() / static_cast<float>(RAND_MAX);
	return random_float;
}

float Noise::getNoise2D(float x, float y) const
{
	float noise_val = noise_->GenSingle2D(x, y, seed_);
	return noise_val;
}

float Noise::getNoise3D(float x, float y, float z) const
{
	float noise_val = noise_->GenSingle3D(x, y, z, seed_);
	return noise_val;
}