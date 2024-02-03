#include "Universe.h"
#include "globals.h"

Universe::Universe(int seed) :
	noise_{ std::make_shared<Noise>(seed) },
	last_tick_time_{ std::chrono::system_clock::now() },
	tcp_client_{"127.0.0.1", "5001"}
{
}

std::shared_ptr<Noise> Universe::getNoise()
{
	return noise_;
}

size_t Universe::getGoKartCount() const
{
	return gokarts_.size();
}

std::vector<uint8_t> Universe::getGoKartNumbers() const
{
	std::vector<uint8_t> gokart_ids;

	for (auto it = begin(gokarts_); it != end(gokarts_); ++it) {
		gokart_ids.emplace_back(it->first);
	}

	return gokart_ids;
}

GoKart* Universe::spawnGoKart(uint8_t kart_number)
{
	auto new_kart = std::make_unique<GoKart>(*this, kart_number, 0.0);
	new_kart->placeAtStartLine(gokarts_.size());
	GoKart* kart_ptr = new_kart.get();
	gokarts_.insert({ kart_number, std::move(new_kart) });
	return kart_ptr;
}

double Universe::getGoKartProgress(uint8_t kart_number) const
{
	auto find_gokart = gokarts_.find(kart_number);
	if (find_gokart != gokarts_.end())
	{
		return 0.0;
	}

	return find_gokart->second->getProgress();
}

void Universe::tick()
{
	const auto now = std::chrono::system_clock::now();
	auto seconds_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_tick_time_).count() / 1'000'000.0;

	std::unordered_map<uint8_t, std::unique_ptr<GoKart>>::iterator gokart_iterator = gokarts_.begin();
	while (gokart_iterator != gokarts_.end()) {
		auto& gokart = gokart_iterator->second;

		gokart->advance(seconds_elapsed);

		gokart_iterator++;
	}

	last_tick_time_ = now;
}

std::vector<float> Universe::getRaceData()
{
	std::vector<float> race_data{};

	for (const auto& gokart : gokarts_)
	{
		race_data.emplace_back(
			static_cast<float>(gokart.second->getProgress())
		);
	}

	return race_data;
}

void Universe::sendData(const std::string& data)
{
	tcp_client_.enqueue(data);
}