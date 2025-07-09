// main_application.h
// Main ECU application class - coordinates all subsystems

#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

#include <stdint.h>

class MainApplication {
public:
    void init();
    void run();
    
    // Diagnostics
    uint32_t getLoopCount() const { return loop_count; }
    uint32_t getLastLoopTime() const { return last_loop_time_us; }

private:
    uint32_t loop_count;
    uint32_t last_loop_time_us;
    uint32_t last_status_report_ms;
    
    void printStatusReport();
};

#endif