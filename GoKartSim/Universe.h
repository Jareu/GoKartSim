#pragma once
#include <chrono>
#include <memory>
#include "Noise.h"
#include "GoKart.h"
#include "TcpClient.h"

class Universe
{
public:
	Universe() = delete;
	Universe(int seed);
	~Universe() = default;

	std::shared_ptr<Noise> getNoise();
	double getGoKartProgress(uint8_t kart_number) const;
	GoKart* spawnGoKart(uint8_t kart_number);
	void tick();
	std::vector<uint8_t> getGoKartNumbers() const;
	std::vector<float> getRaceData();
	size_t getGoKartCount() const;
	void sendData(const std::string& data);
private:
	std::shared_ptr<Noise> noise_;
	std::unordered_map<uint8_t, std::unique_ptr<GoKart>> gokarts_;
	std::chrono::system_clock::time_point last_tick_time_;
	TcpClient tcp_client_;
};

