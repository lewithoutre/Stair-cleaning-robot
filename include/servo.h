#pragma once

#include <thread>
#include <chrono>
#include <iostream>

class ServoInterface {
public:
    ServoInterface() {}
    ~ServoInterface() {}

    // Placeholder flip action; implement hardware-specific code here
    void flip(double duration_sec = 0.8);
};
