#include "servo.h"
#include <iostream>
#include <thread>
#include <chrono>

void ServoInterface::flip(double duration_sec) {
    std::cout << "[ServoInterface] flip placeholder, sleeping " << duration_sec << "s\n";
    std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
}
