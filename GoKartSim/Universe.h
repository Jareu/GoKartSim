#pragma once
#include <chrono>
#include <memory>
#include "Noise.h"
#include "GoKart.h"

class Universe
{
public:
	Universe() = delete;
	Universe(int seed);
	~Universe() = default;

	std::shared_ptr<Noise> getNoise();
	double getGoKartProgress(uint8_t kart_number) const;
	void spawnGoKart(uint8_t kart_number);
	void tick();
	std::vector<float> getRaceData();
private:
	std::shared_ptr<Noise> noise_;
	std::unordered_map<uint8_t, GoKart> gokarts_;
	std::chrono::system_clock::time_point last_tick_time_;
};

