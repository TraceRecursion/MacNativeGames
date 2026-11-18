/* Switch HD rumble through the DualSense's own haptic channels, beside the system's audio output.
   2026-11-18, ported from seedds/nxnative PR #1993 (mac-dualsense-usb-haptics, Radium Wang, MIT).

   The DualSense's wireless receiver and its USB cable both expose ONE USB audio device with four output
   channels: 0/1 are the speaker, 2/3 drive the two grips' haptic actuators. This opens that device itself
   (found by transport, channel count and name, never by a saved device ID or UID) and writes silence to the
   speaker channels and the game's two vibration bands, synthesised at 48 kHz, to the haptic channels; the
   game's audio keeps going to whatever the system's default output is. While this route is open the game's
   rumble does not also play through Core Haptics (the two would add on a wired controller).

   The HID side clears the emulated-rumble flags of the controller that belongs to the same USB location, so
   the firmware does not mix its own rumble into the haptics. The report carries no LED, trigger or audio
   change; the effect ends when the app quits, the controller is unplugged, or the game goes quiet.

   dualsense-rumble.txt (next to controls.txt) saying off turns it off; a missing file keeps it on. When no
   such device is present (Bluetooth, another controller, another computer), the caller falls back to Core
   Haptics, which is the old path and needs no setting. */
#import <Foundation/Foundation.h>
#include <TargetConditionals.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include "runtime.h"
#include "dualsense_rumble.h"

#if !TARGET_OS_OSX
bool ds_rumble_on(void) { return false; }
void ds_rumble_set(unsigned side, float al, float fl, float ah, float fh, float gain) { (void)side; (void)al; (void)fl; (void)ah; (void)fh; (void)gain; }
void ds_rumble_tick(void) {}
bool ds_rumble_active(void) { return false; }
void ds_rumble_quiet(void) {}
void ds_rumble_shutdown(void) {}
#else
#import <AudioToolbox/AudioToolbox.h>
#import <CoreAudio/CoreAudio.h>
#import <IOKit/IOKitLib.h>
#import <IOKit/hid/IOHIDDevice.h>

enum { DS_RATE = 48000, DS_CHANNELS = 4, DS_QUEUE = 256, DS_RAMP = 96 };

typedef struct {
    uint64_t time;
    unsigned generation;
    unsigned side;
    float band_low, freq_low, band_high, freq_high;
    float gain;
} DsEvent;

typedef struct {
    float amplitude, target, step, frequency, phase;
    unsigned remaining;
} DsTone;

typedef struct {
    AudioUnit unit;
    AudioDeviceID device;
    _Atomic unsigned head, tail, generation;
    DsEvent events[DS_QUEUE];
    DsTone tone[2][2];
    unsigned renderGeneration;   /* audio thread only */
    uint64_t delay;
    bool initialized;
} DsUsb;

static DsUsb ds;
static _Atomic bool ds_open;
static _Atomic bool ds_on = true;
static _Atomic bool ds_read;
static uint64_t ds_retry_at, ds_check_at;

static float ds_gain = 1.0f;
static pthread_mutex_t ds_submit_lock = PTHREAD_MUTEX_INITIALIZER;   /* the game calls runtime_rumble_set from its own threads */   /* written by the rumble queue's caller, read by the audio thread */

static void ds_reset(DsUsb *d) { atomic_fetch_add(&d->generation, 1); }

static void ds_submit(unsigned side, float al, float fl, float ah, float fh, float gain) {
    DsUsb *d = &ds;
    pthread_mutex_lock(&ds_submit_lock);
    unsigned head = atomic_load_explicit(&d->head, memory_order_relaxed);
    if (head - atomic_load_explicit(&d->tail, memory_order_acquire) >= DS_QUEUE) {
        ds_reset(d);   /* a stale sustained vibration must not get stuck */
        pthread_mutex_unlock(&ds_submit_lock);
        return;
    }
    DsEvent *e = &d->events[head % DS_QUEUE];
    *e = (DsEvent){clock_gettime_nsec_np(CLOCK_UPTIME_RAW), atomic_load(&d->generation), side, al, fl, ah, fh, gain};
    atomic_store_explicit(&d->head, head + 1, memory_order_release);
    pthread_mutex_unlock(&ds_submit_lock);
}

static void ds_tone_set(DsTone *t, float a, float f) {
    if (f > 0) t->frequency = f;
    if (t->target == a) return;
    t->target = a;
    t->step = (a - t->amplitude) / DS_RAMP;
    t->remaining = DS_RAMP;
}

static void ds_apply(DsUsb *d, const DsEvent *e) {
    /* Both bands share one headroom: scaling them together keeps their balance. */
    float lo = e->band_low * e->gain * 0.5f, hi = e->band_high * e->gain * 0.5f;
    ds_tone_set(&d->tone[e->side][0], lo, e->freq_low);
    ds_tone_set(&d->tone[e->side][1], hi, e->freq_high);
}

static float ds_sample(DsTone *t) {
    if (t->remaining) {
        t->amplitude += t->step;
        if (!--t->remaining) t->amplitude = t->target;
    }
    if (!t->amplitude) return 0;
    float value = t->amplitude * sinf(t->phase);
    t->phase += t->frequency * (2.f * (float)M_PI / DS_RATE);
    if (t->phase >= 2.f * (float)M_PI) t->phase -= 2.f * (float)M_PI;
    return value;
}

static void ds_pcm(DsUsb *d, float *out, unsigned frames, uint64_t start) {
    unsigned head = atomic_load_explicit(&d->head, memory_order_acquire);
    unsigned generation = atomic_load(&d->generation);
    if (d->renderGeneration != generation) {
        memset(d->tone, 0, sizeof d->tone);
        d->renderGeneration = generation;
    }
    unsigned tail = atomic_load_explicit(&d->tail, memory_order_relaxed);
    for (unsigned i = 0; i < frames; ++i) {
        uint64_t time = start + (uint64_t)i * 1000000000ull / DS_RATE;
        while (tail != head) {
            const DsEvent *e = &d->events[tail % DS_QUEUE];
            if (e->generation == generation) {
                if (e->time > time) break;
                ds_apply(d, e);
            }
            ++tail;
        }
        *out++ = 0;   /* channel 0: the speaker stays the system's */
        *out++ = 0;   /* channel 1 */
        *out++ = ds_sample(&d->tone[0][0]) + ds_sample(&d->tone[0][1]);
        *out++ = ds_sample(&d->tone[1][0]) + ds_sample(&d->tone[1][1]);
    }
    atomic_store_explicit(&d->tail, tail, memory_order_release);
}

static OSStatus ds_render(void *ctx, AudioUnitRenderActionFlags *flags, const AudioTimeStamp *time,
                          UInt32 bus, UInt32 frames, AudioBufferList *data) {
    (void)flags; (void)time; (void)bus;
    DsUsb *d = ctx;
    if (data->mNumberBuffers != 1 || data->mBuffers[0].mNumberChannels != DS_CHANNELS ||
        data->mBuffers[0].mDataByteSize < frames * DS_CHANNELS * sizeof(float) || !data->mBuffers[0].mData) {
        for (UInt32 i = 0; i < data->mNumberBuffers; ++i)
            if (data->mBuffers[i].mData) memset(data->mBuffers[i].mData, 0, data->mBuffers[i].mDataByteSize);
        return kAudioUnitErr_FormatNotSupported;
    }
    uint64_t now = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
    ds_pcm(d, data->mBuffers[0].mData, frames, now > d->delay ? now - d->delay : 0);
    return noErr;
}

static OSStatus ds_property(AudioObjectID id, AudioObjectPropertySelector selector,
                            AudioObjectPropertyScope scope, UInt32 *size, void *value) {
    AudioObjectPropertyAddress a = {selector, scope, kAudioObjectPropertyElementMain};
    return AudioObjectGetPropertyData(id, &a, 0, NULL, size, value);
}

static CFStringRef ds_string(AudioDeviceID id, AudioObjectPropertySelector selector) CF_RETURNS_RETAINED {
    CFStringRef string = NULL;
    UInt32 size = sizeof(CFStringRef);
    return ds_property(id, selector, kAudioObjectPropertyScopeGlobal, &size, &string) == noErr ? string : NULL;
}

static bool ds_four_outputs(AudioDeviceID id) {
    AudioObjectPropertyAddress a = {kAudioDevicePropertyStreamConfiguration, kAudioDevicePropertyScopeOutput,
                                    kAudioObjectPropertyElementMain};
    UInt32 size = 0;
    if (AudioObjectGetPropertyDataSize(id, &a, 0, NULL, &size) || !size) return false;
    AudioBufferList *buffers = calloc(1, size);
    if (!buffers) return false;
    UInt32 n = 0;
    if (!AudioObjectGetPropertyData(id, &a, 0, NULL, &size, buffers))
        for (UInt32 i = 0; i < buffers->mNumberBuffers; ++i) n += buffers->mBuffers[i].mNumberChannels;
    free(buffers);
    return n == DS_CHANNELS;
}

/* The DualSense's four-channel USB endpoint: by transport, channel count, maker and name, so another
   computer, another controller of the same model or another cable/receiver all work without a saved ID. */
static AudioDeviceID ds_device(void) {
    AudioObjectPropertyAddress a = {kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal,
                                    kAudioObjectPropertyElementMain};
    UInt32 size = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &a, 0, NULL, &size) || !size) return 0;
    AudioDeviceID *ids = malloc(size);
    if (!ids) return 0;
    AudioDeviceID result = 0;
    int matches = 0;
    if (!AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, 0, NULL, &size, ids)) {
        for (UInt32 i = 0; i < size / sizeof(*ids); ++i) {
            AudioDeviceID id = ids[i];
            UInt32 transport = 0, n = sizeof(transport);
            if (ds_property(id, kAudioDevicePropertyTransportType, kAudioObjectPropertyScopeGlobal, &n, &transport) ||
                transport != kAudioDeviceTransportTypeUSB || !ds_four_outputs(id)) continue;
            NSString *name = CFBridgingRelease(ds_string(id, kAudioObjectPropertyName));
            NSString *maker = CFBridgingRelease(ds_string(id, kAudioObjectPropertyManufacturer));
            bool sony = maker && [maker localizedCaseInsensitiveContainsString:@"Sony"];
            bool dualsense = name && [name localizedCaseInsensitiveContainsString:@"DualSense"];
            if (!dualsense && !(sony && name && [name localizedCaseInsensitiveContainsString:@"Wireless Controller"])) continue;
            matches++;
            if (!result) result = id;
        }
    }
    free(ids);
    if (matches > 1) fprintf(stderr, "[rumble] %d DualSense audio endpoints: using the first\n", matches);
    return result;
}

/* Clear the emulated-rumble flags of the controller at the same USB location as the audio endpoint. */
static IOReturn ds_enable_pcm(AudioDeviceID audio) {
    NSString *uid = CFBridgingRelease(ds_string(audio, kAudioDevicePropertyDeviceUID));
    NSArray<NSString *> *parts = [uid componentsSeparatedByString:@":"];
    if (parts.count < 5 || ![parts[0] isEqualToString:@"AppleUSBAudioEngine"]) return kIOReturnNotFound;
    unsigned location = 0;
    if (![[NSScanner scannerWithString:parts[parts.count - 2]] scanHexInt:&location] || !location) return kIOReturnNotFound;
    CFMutableDictionaryRef match = IOServiceMatching(kIOHIDDeviceKey);
    if (!match) return kIOReturnNotFound;
    CFDictionarySetValue(match, CFSTR(kIOHIDVendorIDKey), (__bridge CFNumberRef)@0x054c);
    io_iterator_t it = 0;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &it)) return kIOReturnNotFound;
    IOReturn result = kIOReturnNotFound;
    for (io_service_t service; (service = IOIteratorNext(it)); IOObjectRelease(service)) {
        IOHIDDeviceRef dev = IOHIDDeviceCreate(kCFAllocatorDefault, service);
        if (!dev) continue;
        unsigned pid = [(__bridge NSNumber *)IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDProductIDKey)) unsignedIntValue];
        unsigned loc = [(__bridge NSNumber *)IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDLocationIDKey)) unsignedIntValue];
        if ((pid == 0x0ce6 || pid == 0x0df2) && loc == location) {
            result = IOHIDDeviceOpen(dev, kIOHIDOptionsTypeNone);
            if (!result) {
                uint8_t report[48] = {2};
                result = IOHIDDeviceSetReport(dev, kIOHIDReportTypeOutput, report[0], report, sizeof(report));
                IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
            }
        }
        CFRelease(dev);
        if (!result) { IOObjectRelease(service); break; }
    }
    IOObjectRelease(it);
    return result;
}

static void ds_close(DsUsb *d) {
    if (d->unit) {
        AudioOutputUnitStop(d->unit);
        if (d->initialized) AudioUnitUninitialize(d->unit);
        AudioComponentInstanceDispose(d->unit);
    }
    *d = (DsUsb){0};
    atomic_store(&ds_open, false);
}

static OSStatus ds_open_route(DsUsb *d, AudioDeviceID id) {
    if (!id) return kAudioHardwareBadDeviceError;
    AudioComponentDescription desc = {kAudioUnitType_Output, kAudioUnitSubType_HALOutput, kAudioUnitManufacturer_Apple, 0, 0};
    AudioComponent component = AudioComponentFindNext(NULL, &desc);
    if (!component) return kAudioUnitErr_FailedInitialization;
    OSStatus err = AudioComponentInstanceNew(component, &d->unit);
    if (err) goto fail;
    UInt32 off = 0, on = 1, frames = 0, bytes = sizeof(frames);
    err = AudioUnitSetProperty(d->unit, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, 1, &off, sizeof(off));
    if (!err) err = AudioUnitSetProperty(d->unit, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &on, sizeof(on));
    if (!err) err = AudioUnitSetProperty(d->unit, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &id, sizeof(id));
    if (!err) err = ds_property(id, kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal, &bytes, &frames);
    d->delay = 2000000 + (uint64_t)frames * 1000000000ull / DS_RATE;
    AudioStreamBasicDescription format = {DS_RATE, kAudioFormatLinearPCM, kAudioFormatFlagsNativeFloatPacked,
        DS_CHANNELS * sizeof(float), 1, DS_CHANNELS * sizeof(float), DS_CHANNELS, 32, 0};
    if (!err) err = AudioUnitSetProperty(d->unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &format, sizeof(format));
    SInt32 map[DS_CHANNELS] = {0, 1, 2, 3};
    if (!err) err = AudioUnitSetProperty(d->unit, kAudioOutputUnitProperty_ChannelMap, kAudioUnitScope_Output, 0, map, sizeof(map));
    AURenderCallbackStruct cb = {ds_render, d};
    if (!err) err = AudioUnitSetProperty(d->unit, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &cb, sizeof(cb));
    if (!err) { err = AudioUnitInitialize(d->unit); d->initialized = err == noErr; }
    if (!err) err = AudioOutputUnitStart(d->unit);
    if (!err) { d->device = id; return noErr; }
fail:
    ds_close(d);
    return err;
}

/* rumbleQueue: open the route when a DualSense endpoint shows up, close it when it goes away. */
void ds_rumble_tick(void) {
    if (!ds_on) return;
    uint64_t now = runtime_now_ns();
    if (atomic_load(&ds_open)) {
        if (now < ds_check_at) return;
        ds_check_at = now + 2000000000ull;
        AudioDeviceID id = ds_device();
        if (!id || id != ds.device) { ds_close(&ds); }
        return;
    }
    if (now < ds_retry_at) return;
    ds_retry_at = now + 2000000000ull;
    AudioDeviceID id = ds_device();
    if (!id) return;
    OSStatus err = ds_open_route(&ds, id);
    if (err) { fprintf(stderr, "[rumble] DualSense audio haptics unavailable (%d); Core Haptics stays in charge\n", (int)err); return; }
    IOReturn hid = ds_enable_pcm(id);
    atomic_store(&ds_open, true);
    fprintf(stderr, "[rumble] DualSense audio haptics: %s, 48000 Hz, grips on channels 2 and 3 (HID rumble mode %s)\n",
            [[[NSString alloc] initWithFormat:@"%u", (unsigned)id] UTF8String], hid == kIOReturnSuccess ? "cleared" : "left alone");
}

bool ds_rumble_on(void) {
    if (!atomic_load(&ds_read)) {
        char word[32] = {0};
        if (runtime_setting_word("dualsense-rumble.txt", word, sizeof word) &&
            (strcmp(word, "off") == 0 || strcmp(word, "no") == 0 || strcmp(word, "0") == 0))
            atomic_store(&ds_on, false);
        atomic_store(&ds_read, true);
        atexit(ds_rumble_shutdown);
        if (!atomic_load(&ds_on)) fprintf(stderr, "[rumble] dualsense-rumble.txt: off; Core Haptics only\n");
    }
    return atomic_load(&ds_on);
}

void ds_rumble_set(unsigned side, float al, float fl, float ah, float fh, float gain) {
    pthread_mutex_lock(&ds_submit_lock);
    ds_gain = gain > 0 ? gain : 1.0f;
    pthread_mutex_unlock(&ds_submit_lock);
    if (!atomic_load(&ds_open)) return;
    ds_submit(side, al, fl, ah, fh, ds_gain);
}

bool ds_rumble_active(void) { return atomic_load(&ds_open); }

void ds_rumble_quiet(void) {
    if (!atomic_load(&ds_open)) return;
    ds_submit(0, 0, 160, 0, 320, ds_gain);
    ds_submit(1, 0, 160, 0, 320, ds_gain);
}

void ds_rumble_shutdown(void) {
    if (ds.unit) ds_close(&ds);
}
#endif /* TARGET_OS_OSX */
