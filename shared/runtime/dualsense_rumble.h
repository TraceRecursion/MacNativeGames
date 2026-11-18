/* The DualSense's haptic channels as the game's rumble output (runtime/dualsense_rumble.m). */
#pragma once
#include <stdbool.h>

/* Whether dualsense-rumble.txt allows the route (a missing file allows it). Reads the file once. */
bool ds_rumble_on(void);

/* One side's two bands for the next 48 kHz render block. `gain` is the rumble strength in force. */
void ds_rumble_set(unsigned side, float amp_low, float freq_low, float amp_high, float freq_high, float gain);

/* rumbleQueue: open the controller's endpoint when it appears, close it when it goes away. */
void ds_rumble_tick(void);

/* The audio route is open, so the caller leaves Core Haptics to it. */
bool ds_rumble_active(void);

/* Focus lost or the game stopped sending: fade both grips out. */
void ds_rumble_quiet(void);

/* App quit: stop the audio unit. */
void ds_rumble_shutdown(void);
