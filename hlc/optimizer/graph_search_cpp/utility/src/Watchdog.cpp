#include "../include/Watchdog.h"

void Watchdog::loop() {
	_active = true;
	_cancelled = false;
	while (true) {
		{
			std::unique_lock<std::mutex> lock(_mutex);
			_reset = false;
			_reset_condition.wait_for(lock, _dt, [&reset = _reset]() { return reset; });
		}
		if (_cancelled) {
			break;
		}
		if (!_reset) {
			_active = false;
			_callback();
			break;
		}
	}
}
bool Watchdog::active() { return _active; }
void Watchdog::stop() {
	if (!_active || _reset) return;
	_cancelled = true;
	std::unique_lock<std::mutex> lock(_mutex);
	_reset = true;
	_reset_condition.notify_all();
	_active = false;
}
void Watchdog::reset() {
	if (!_active || _reset) return;
	std::unique_lock<std::mutex> lock(_mutex);
	_reset = true;
	_reset_condition.notify_all();
}
void Watchdog::start() {
	if (_active) return;
	std::thread t(&Watchdog::loop, this);
	t.detach();
}
Watchdog::~Watchdog() { stop(); }
