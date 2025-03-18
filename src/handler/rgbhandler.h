#ifndef RGBHANDLER_H
#define RGBHANDLER_H

#include <Arduino.h>
#include <WS2812Serial.h>

// defines 7 sections with respective LED counts
constexpr uint8_t NUM_SECTIONS = 7;
constexpr uint16_t SECTION_SIZES[NUM_SECTIONS] = {13, 10, 10, 10, 13, 4, 4}; // example sizes, change later idfk

// compile-time calculation of total LED count
constexpr uint16_t TOTAL_LEDS = []()
{
    uint16_t total = 0;
    for (auto size : SECTION_SIZES)
        total += size;
    return total;
}();

// pin and brightness settings
constexpr uint8_t LED_PIN = 6;                 // pin controlling WS2812 LEDs
constexpr unsigned long DEFAULT_SPEED = 50;    // default effect speed
constexpr unsigned long MIN_SPEED = 20;        // minimum speed limit
constexpr unsigned long MAX_SPEED = 2000;      // maximum speed limit
constexpr uint8_t DEFAULT_BRIGHTNESS = 128;    // default brightness level
constexpr uint8_t MAX_BRIGHTNESS = 255;        // maximum brightness level

// effect types (NONE = static color, PULSE = fading effect, STREAK = moving trail)
enum EffectType
{
    NONE,
    PULSE,
    STREAK
};

class RGBHandler
{
public:
    RGBHandler(uint8_t kLED);
    bool Begin();
    void Update();

    // section-based LED control
    bool setSectionSolidColor(uint8_t section, uint8_t r, uint8_t g, uint8_t b); // sets a section to a solid color
    bool setSectionPulseEffect(uint8_t section, uint8_t r, uint8_t g, uint8_t b, unsigned long speed); // enables pulse effect
    bool setSectionStreakEffect(uint8_t section, uint8_t r, uint8_t g, uint8_t b, unsigned long speed); // enables streak effect
    bool stopSectionEffect(uint8_t section); // stops effect in a section
    bool setGlobalBrightness(uint8_t brightness); // adjusts global brightness
    bool processCommand(const String &command); // processes external command inputs
    void stopAllEffects(); // stops all active effects

    void PrintInfo(Print &output, bool printConfig) const;
    friend Print &operator<<(Print &output, const RGBHandler &handler);

private:
    uint8_t kLED;
    // storing effect states for each section
    struct SectionEffect
    {
        EffectType currentEffect = NONE;  // current effect type (NONE, PULSE, STREAK)
        unsigned long lastUpdate = 0;     // timestamp of last update
        unsigned long effectSpeed = DEFAULT_SPEED; // speed of the effect
        
        // pulse effect parameters
        uint8_t pulse_r, pulse_g, pulse_b; // base color for pulse effect
        uint16_t pulse_phase = 0;  // current phase of pulse effect
        uint16_t pulse_step = 0;   // step size for brightness change
        
        // streak effect parameters
        uint8_t streak_r, streak_g, streak_b; // color of the streak
        int streak_position = 0;   // current position of the streak
        uint8_t streak_trailLength = 2; // length of the trail
    };

    SectionEffect sections[NUM_SECTIONS]; // array storing effect states per section
    uint16_t sectionStarts[NUM_SECTIONS]; // array storing the starting indices of each section
    uint8_t globalBrightness = DEFAULT_BRIGHTNESS; // current global brightness
    WS2812Serial leds; // LED strip driver instance

    // utility functions for effects
    void applyBrightness(uint8_t r, uint8_t g, uint8_t b, uint8_t &r_out, uint8_t &g_out, uint8_t &b_out); // applies brightness scaling
    void updatePulse(uint8_t section);  // updates pulse effect for a section
    void updateStreak(uint8_t section); // updates streak effect for a section

    // tracking previous positions for streak effect
    static int prev_positions[NUM_SECTIONS];

    // memory buffers for WS2812Serial
    uint8_t drawingMemory[TOTAL_LEDS * 3] __attribute__((aligned(32))); // buffer for LED drawing
    alignas(32) uint8_t displayMemory[TOTAL_LEDS * 12]; // buffer for LED display

    // compile-time LED count calculation
    static constexpr uint16_t TOTAL_LEDS = []() {
        uint16_t total = 0;
        for (auto size : SECTION_SIZES) total += size;
        return total;
    }();
};

#endif
