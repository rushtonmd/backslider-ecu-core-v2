/*
 * SimpleNextion.h - Lightweight Nextion Display Library
 * 
 * Simple write-only communication with Nextion displays
 * Uses minimal memory and provides essential functions only
 * 
 * Protocol: All commands end with 0xFF 0xFF 0xFF
 */

#ifndef SIMPLE_NEXTION_H
#define SIMPLE_NEXTION_H

#include <Arduino.h>
#include <HardwareSerial.h>

class SimpleNextion {
private:
    HardwareSerial* serial;
    bool initialized;
    
    // Send the 3-byte command terminator
    void endCommand();
    
    // Send a raw command string
    void sendCommand(const char* command);
    
public:
    // Constructor - pass any HardwareSerial instance
    SimpleNextion(HardwareSerial* serialPort = &Serial2);
    
    // Initialize the display communication
    bool begin(long baudRate = 9600, int rxPin = -1, int txPin = -1);
    
    // Basic display control
    void setBrightness(int brightness);  // 0-100
    void sleep();                        // Put display to sleep
    void wake();                         // Wake display
    void reset();                        // Software reset
    
    // Text object functions
    void setText(const char* objectName, const char* text);
    void setText(const char* objectName, String text);
    void setTextFloat(const char* objectName, float value, int decimals = 1);
    void setTextInt(const char* objectName, int value);
    
    // Number/Value object functions  
    void setValue(const char* objectName, int value);
    void setValueFloat(const char* objectName, float value);
    
    // Color functions (for supported objects)
    void setBackgroundColor(const char* objectName, int color);
    void setForegroundColor(const char* objectName, int color);
    
    // Progress bar functions
    void setProgress(const char* objectName, int percentage); // 0-100
    
    // Gauge/Meter functions
    void setGaugeValue(const char* objectName, int value);
    
    // Picture functions
    void setPicture(const char* objectName, int pictureId);
    
    // Page functions
    void setPage(int pageId);
    void setPage(const char* pageName);
    
    // Visibility functions
    void setVisible(const char* objectName, bool visible);
    
    // Utility functions
    bool isInitialized() const { return initialized; }
    void flush();  // Ensure all data is sent
    
    // Common Nextion colors (RGB565 format)
    static const int COLOR_BLACK   = 0x0000;
    static const int COLOR_WHITE   = 0xFFFF;
    static const int COLOR_RED     = 0xF800;
    static const int COLOR_GREEN   = 0x07E0;
    static const int COLOR_BLUE    = 0x001F;
    static const int COLOR_YELLOW  = 0xFFE0;
    static const int COLOR_CYAN    = 0x07FF;
    static const int COLOR_MAGENTA = 0xF81F;
    static const int COLOR_ORANGE  = 0xFC00;
    static const int COLOR_GRAY    = 0x8410;
};

// Convenience macros for common operations
#define NEXTION_UPDATE_SPEED(display, speed)     display.setTextFloat("speed", speed, 1)
#define NEXTION_UPDATE_GEAR(display, gear)       display.setTextInt("gear", (int)gear)
#define NEXTION_UPDATE_TEMP(display, temp)       display.setTextFloat("temp", temp, 1)
#define NEXTION_UPDATE_PROGRESS(display, obj, val) display.setProgress(obj, (int)val)

#endif // SIMPLE_NEXTION_H