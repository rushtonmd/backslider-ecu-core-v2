// FILE 2: main_application.h
// ===========================
#ifndef MAIN_APPLICATION_H
#define MAIN_APPLICATION_H

class MainApplication {
public:
    void init();
    void run();

private:
    unsigned long lastBlink;
    bool ledState;
};

#endif