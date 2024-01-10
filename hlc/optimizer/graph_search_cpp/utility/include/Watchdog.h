#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <condition_variable>

class Watchdog {
	std::chrono::seconds _dt;

	bool _active = false;
	bool _cancelled = false;
	bool _reset = false;

	std::condition_variable _reset_condition;
	std::mutex _mutex;

	std::function<void(void)> _callback;

	void loop();

   public:
	explicit Watchdog(std::chrono::seconds dt, std::function<void()> callback) : _dt(dt), _callback(std::move(callback)) {}
	bool active();
	void stop();
	void reset();
	void start();
	~Watchdog();
};