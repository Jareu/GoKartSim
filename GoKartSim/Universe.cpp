#include "Universe.h"
#include "globals.h"

Universe::Universe(int seed) :
	noise_{ std::make_shared<Noise>(seed) }
{

}

std::shared_ptr<Noise> Universe::getNoise()
{
	return noise_;
}

void Universe::spawnGoKart(uint8_t kart_number)
{
	GoKart new_kart = GoKart{ *this, kart_number, 0.0 };
	new_kart.placeAtStartLine(gokarts_.size());
	new_kart.setSpeed(PI / 100.0);
	gokarts_.insert({ kart_number, new_kart });
}

double Universe::getGoKartProgress(uint8_t kart_number) const
{
	auto find_gokart = gokarts_.find(kart_number);
	if (find_gokart != gokarts_.end())
	{
		return 0.0;
	}

	return find_gokart->second.getProgress();
}

void Universe::tick()
{
	const auto now = std::chrono::system_clock::now();
	auto seconds_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_tick_time_).count() / 1'000'000.0;

	std::unordered_map<uint8_t, GoKart>::iterator gokart_iterator = gokarts_.begin();
	while (gokart_iterator != gokarts_.end()) {
		auto& gokart = gokart_iterator->second;

		gokart.advance(seconds_elapsed);

		gokart_iterator++;
	}

	last_tick_time_ = now;
}