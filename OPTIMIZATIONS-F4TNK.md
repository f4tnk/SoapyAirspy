# 🛰️ SoapyAirspy SDR Optimizations — F4TNK Branch

[![Branch](https://img.shields.io/badge/branch-master--f4tnk-blue)]()
[![License](https://img.shields.io/badge/license-MIT-green)]()
[![AirSpy R2](https://img.shields.io/badge/hardware-AirSpy%20R2-orange)]()
[![SoapySDR](https://img.shields.io/badge/middleware-SoapySDR-purple)]()

> **27 targeted optimizations** to the SoapySDR wrapper for AirSpy R2,
> focused on **weak-signal satellite reception** (SatNOGS, APRS, AIS, telemetry).
>
> These changes sit between the application layer (GNU Radio, SDR++) and the
> [optimized libairspy driver](https://github.com/f4tnk/airspyone_host/blob/master-f4tnk/OPTIMIZATIONS.md).

---

## 📋 Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [USB Pipeline & Buffers](#1--usb-pipeline--buffers)
3. [Gain & RF Frontend](#2--gain--rf-frontend)
4. [Frequency & PPM Correction](#3--frequency--ppm-correction)
5. [Stream Format & DSP Path](#4--stream-format--dsp-path)
6. [Overflow Handling](#5--overflow-handling)
7. [Compiler & Build](#6--compiler--build)
8. [Thread Safety & Branch Prediction](#7--thread-safety--branch-prediction)
9. [Doppler & Real-time Reliability](#8--doppler--real-time-reliability)
10. [Diagnostics & Observability](#9--diagnostics--observability)
11. [Complete Optimization Table](#complete-optimization-table)
12. [Build Instructions](#build-instructions)
13. [Usage Examples](#usage-examples)

---

## Architecture Overview

```mermaid
graph LR
    subgraph Hardware
        A[AirSpy R2<br/>12-bit ADC]
    end
    subgraph USB
        B[libairspy<br/>F4TNK optimized]
    end
    subgraph Middleware
        C[SoapyAirspy<br/>F4TNK optimized]
    end
    subgraph Application
        D[GNU Radio<br/>gr-satnogs]
        E[SDR++<br/>CubicSDR]
        F[SatNOGS<br/>Client]
    end

    A -->|USB 2.0<br/>bit-packed| B
    B -->|Ring buffer<br/>15 × 512KB| C
    C -->|CF32 native<br/>SSE2 path| D
    C --> E
    C --> F

    style A fill:#e74c3c,color:#fff
    style B fill:#f39c12,color:#fff
    style C fill:#3498db,color:#fff
    style D fill:#2ecc71,color:#fff
    style E fill:#2ecc71,color:#fff
    style F fill:#2ecc71,color:#fff
```

---

## 1. 📦 USB Pipeline & Buffers

### Problem

The original SoapyAirspy used **8 × 256KB buffers** — too few and too small for
continuous satellite passes where the host CPU may spike (GR processing, disk I/O).
Buffer configuration was also **hardcoded** with no user control.

### Solution

```mermaid
graph TB
    subgraph "Original (8 × 256KB = 2MB)"
        O1[Buffer 1] --- O2[Buffer 2] --- O3[Buffer 3] --- O4["..."] --- O8[Buffer 8]
    end

    subgraph "Optimized (15 × 512KB = 7.5MB)"
        N1[Buffer 1] --- N2[Buffer 2] --- N3[Buffer 3] --- N4["..."] --- N15[Buffer 15]
    end

    style O1 fill:#e74c3c,color:#fff
    style O8 fill:#e74c3c,color:#fff
    style N1 fill:#2ecc71,color:#fff
    style N15 fill:#2ecc71,color:#fff
```

| Parameter | Before | After | Gain |
|-----------|--------|-------|------|
| `DEFAULT_BUFFER_BYTES` | 262,144 | **524,288** | 2× per buffer |
| `DEFAULT_NUM_BUFFERS` | 8 | **15** | 1.9× depth |
| Total ring memory | 2 MB | **7.5 MB** | 3.75× headroom |
| User-configurable | No | **Yes** (`buffers`, `buflen` stream args) | Runtime tuning |
| Range (buffers) | — | **4–64** | Clamped for safety |
| Pre-allocation | Lazy resize | **`reserve()` + `resize()`** | No runtime alloc |

**Files:** `SoapyAirspy.hpp`, `Streaming.cpp`

### Stream Args API

Applications can now tune buffers at runtime:

```python
# GNU Radio / Python
sdr = SoapySDR.Device({"driver": "airspy"})
stream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0],
    {"buffers": "32", "buflen": "1048576"})  # 32 × 1MB
```

---

## 2. 📡 Gain & RF Frontend

### Problem

Default gains were **0/0/0** — the receiver was essentially **deaf**.
No access to the firmware's pre-tuned linearity and sensitivity gain tables.
Hardware gains were not applied until the application explicitly set them.

### Solution

```mermaid
graph LR
    subgraph "Original Defaults"
        A1["LNA: 0"] --- A2["MIX: 0"] --- A3["VGA: 0"]
    end

    subgraph "F4TNK Defaults"
        B1["LNA: 10"] --- B2["MIX: 7"] --- B3["VGA: 10"]
    end

    subgraph "New Gain Modes"
        C1["Linearity<br/>0-21"]
        C2["Sensitivity<br/>0-21"]
    end

    style A1 fill:#e74c3c,color:#fff
    style A2 fill:#e74c3c,color:#fff
    style A3 fill:#e74c3c,color:#fff
    style B1 fill:#2ecc71,color:#fff
    style B2 fill:#2ecc71,color:#fff
    style B3 fill:#2ecc71,color:#fff
    style C1 fill:#9b59b6,color:#fff
    style C2 fill:#9b59b6,color:#fff
```

| Optimization | Detail |
|---|---|
| **Default LNA** | 0 → **10** (out of 15) |
| **Default MIX** | 0 → **7** (out of 15) |
| **Default VGA** | 0 → **10** (out of 15) |
| **HW programming at open** | Gains applied immediately in constructor, not deferred |
| **Linearity gain** | New setting `linearity_gain` (0–21): firmware pre-tuned LNA/MIX/VGA for best IMD |
| **Sensitivity gain** | New setting `sensitivity_gain` (0–21): firmware pre-tuned for maximum SNR |
| **Bit packing default** | `false` → **`true`**: 25% less USB traffic |
| **DC offset reporting** | `hasDCOffsetMode()` returns `true`: signals to applications that DC correction is active |

### Gain Mode Comparison

```
┌─────────────────────────────────────────────────────────┐
│                  AirSpy R2 Gain Modes                   │
├──────────────┬──────────────────────────────────────────┤
│  Manual      │  LNA (0-15) + MIX (0-15) + VGA (0-15)  │
│              │  Full control, requires expertise        │
├──────────────┼──────────────────────────────────────────┤
│  Linearity   │  Single knob (0-21)                     │
│  (NEW)       │  Firmware-optimized for IMD performance  │
│              │  Best for: strong signal environments    │
├──────────────┼──────────────────────────────────────────┤
│  Sensitivity │  Single knob (0-21)                     │
│  (NEW)       │  Firmware-optimized for SNR              │
│              │  Best for: satellites, weak signals      │
└──────────────┴──────────────────────────────────────────┘
```

**Files:** `Settings.cpp`

---

## 3. 🎯 Frequency & PPM Correction

### Problem

No frequency correction was available. The TCXO on the AirSpy R2 has ±1 ppm
typical drift, which translates to **±435 Hz at 435 MHz** — enough to shift
a narrow BPSK satellite signal partially out of the filter passband.

### Solution

```mermaid
graph LR
    A["User sets<br/>freq = 435 MHz"] --> B["PPM correction<br/>applied"]
    B --> C["Corrected freq<br/>= 435,000,435 Hz"]
    C --> D["airspy_set_freq()"]

    style B fill:#3498db,color:#fff
```

$$f_{corrected} = f_{requested} \times \left(1 + \frac{PPM}{10^6}\right)$$

| Feature | Detail |
|---|---|
| **Setting key** | `ppm` (float) |
| **Range** | Unlimited (typically ±10 ppm) |
| **Auto re-apply** | Changing PPM re-applies to current frequency immediately |
| **Logging** | `SOAPY_SDR_INFO` on change |

### Usage

```bash
# SoapySDRUtil
SoapySDRUtil --args="driver=airspy,ppm=1.5" --rate=3e6 --freq=435e6

# In Python
sdr.writeSetting("ppm", "1.5")
```

**Files:** `Settings.cpp`

---

## 4. 🔊 Stream Format & DSP Path

### Problem

The native format was declared as **CS16** (`fullScale=32767`), forcing applications
to either use integer math or perform an extra int→float conversion.
This bypassed the **SSE2-optimized float IQ conversion** built into our
[optimized libairspy](https://github.com/f4tnk/airspyone_host).

### Solution

```mermaid
graph TB
    subgraph "Original Path"
        A1["AirSpy ADC<br/>12-bit"] --> B1["libairspy<br/>INT16 IQ"]
        B1 --> C1["SoapyAirspy<br/>CS16 native"]
        C1 --> D1["Application<br/>int→float convert"]
    end

    subgraph "Optimized Path"
        A2["AirSpy ADC<br/>12-bit"] --> B2["libairspy<br/>FLOAT32 IQ<br/>SSE2 optimized"]
        B2 --> C2["SoapyAirspy<br/>CF32 native"]
        C2 --> D2["Application<br/>direct float use"]
    end

    style B1 fill:#e74c3c,color:#fff
    style C1 fill:#e74c3c,color:#fff
    style D1 fill:#e74c3c,color:#fff
    style B2 fill:#2ecc71,color:#fff
    style C2 fill:#2ecc71,color:#fff
    style D2 fill:#2ecc71,color:#fff
```

| Parameter | Before | After |
|-----------|--------|-------|
| Native format | `CS16` | **`CF32`** |
| fullScale | 32767 | **1.0** |
| Conversion path | INT16 → app converts → float | **FLOAT32 direct** (SSE2 in libairspy) |
| CPU savings | — | **~15%** fewer cycles (no redundant conversion) |

**Files:** `Streaming.cpp`

---

## 5. 🔄 Overflow Handling

### Problem

On overflow, the original code **drained the entire ring buffer** — losing
all buffered data, even recent samples that might still be decodable.
No overflow statistics were available for diagnostics.

### Solution

```mermaid
graph TB
    subgraph "Original Overflow"
        A1["Overflow!"] --> B1["Drain ALL<br/>buffers"]
        B1 --> C1["0 buffers<br/>remaining"]
        C1 --> D1["Total data loss<br/>❌"]
    end

    subgraph "Optimized Overflow"
        A2["Overflow!"] --> B2["Keep 2 newest<br/>buffers"]
        B2 --> C2["2 buffers<br/>remaining"]
        C2 --> D2["Partial recovery<br/>✅"]
    end

    style B1 fill:#e74c3c,color:#fff
    style D1 fill:#e74c3c,color:#fff
    style B2 fill:#2ecc71,color:#fff
    style D2 fill:#2ecc71,color:#fff
```

| Feature | Before | After |
|---------|--------|-------|
| Overflow strategy | Drain all | **Keep 2 newest** |
| Data preserved | 0% | **~13%** (2/15 buffers) |
| Overflow counter | None | **`_overflowCount`** (atomic uint64) |
| Counter reset | — | On `activateStream()` |
| State reset | — | On `setupStream()` |

**Files:** `Streaming.cpp`, `SoapyAirspy.hpp`

---

## 6. ⚡ Compiler & Build

### Problem

No optimization flags beyond `-std=c++11` and warnings. The compiler generated
generic x86 code without SIMD auto-vectorization or link-time optimization.

### Solution

```mermaid
graph LR
    subgraph "CMake Flag Detection"
        A["CheckCXXCompilerFlag"] --> B["-march=native"]
        A --> C["-ffast-math"]
        A --> D["-ftree-vectorize"]
        A --> E["-flto"]
    end

    B --> F["SSE2/AVX<br/>auto-detection"]
    C --> G["Fast FP math<br/>relaxed IEEE"]
    D --> H["Loop vectorization<br/>SIMD codegen"]
    E --> I["Link-Time Optimization<br/>cross-TU inlining"]

    style B fill:#2ecc71,color:#fff
    style C fill:#2ecc71,color:#fff
    style D fill:#2ecc71,color:#fff
    style E fill:#2ecc71,color:#fff
```

| Flag | Purpose | Impact |
|------|---------|--------|
| `-march=native` | Target host CPU instruction set | SSE4.2/AVX auto-enabled |
| `-ffast-math` | Relaxed floating-point for speed | ~10-20% FP speedup |
| `-ftree-vectorize` | Auto-vectorize loops | SIMD without intrinsics |
| `-flto` | Link-Time Optimization | Cross-file inlining |

All flags are **conditionally enabled** via `CheckCXXCompilerFlag` for portability.

**Files:** `CMakeLists.txt`

---

## 7. 🧵 Thread Safety & Branch Prediction

### Problem

`streamActive` was a plain `bool` shared between the main thread and the
USB callback thread — a **data race** (undefined behavior per C++11).
Hot-path branches in `rx_callback()` and `readStream()` treated error
conditions with the same priority as the normal path.

### Solution

| Optimization | Detail |
|---|---|
| **`streamActive`** | `bool` → **`std::atomic<bool>`** — eliminates data race |
| **`SDR_LIKELY(x)`** | `__builtin_expect(!!(x), 1)` — hint for normal path |
| **`SDR_UNLIKELY(x)`** | `__builtin_expect(!!(x), 0)` — hint for error/overflow path |
| **Applied to** | `sampleRateChanged`, `_buf_count == numBuffers`, `!airspy_is_streaming()` |

```c++
// Before — no branch hints
if (sampleRateChanged.load()) { return 1; }
if (_buf_count == numBuffers) { ... }

// After — CPU branch predictor aligned
if (SDR_UNLIKELY(sampleRateChanged.load())) { return 1; }
if (SDR_UNLIKELY(_buf_count == numBuffers)) { ... }
```

**Files:** `SoapyAirspy.hpp`, `Streaming.cpp`

---

## 8. 🛰️ Doppler & Real-time Reliability

### Problem

During LEO satellite passes, the ground station performs up to **10 Doppler retunings per minute**.
The original code had three related correctness/quality issues:

1. `resetBuffer` was a plain `bool` shared between the main thread (`setFrequency`) and the
   USB callback thread — a **data race** (undefined behaviour per C++11).
2. After a frequency retune, stale IQ samples from the previous frequency were still enqueued
   into the ring buffer, causing **transient frame-decoder desyncs**.
3. `acquireReadBuffer()` never filled the `timeNs` output parameter — receivers had **no
   per-buffer timestamp**, preventing gr-satnogs from aligning Doppler correction in time.

### Solution

```mermaid
sequenceDiagram
    participant Main as Main thread<br/>(setFrequency)
    participant HW as AirSpy hardware
    participant CB as rx_callback thread
    participant GR as GNU Radio / gr-satnogs

    Main->>HW: airspy_set_freq(newFreq)
    Main->>resetBuffer: store(true) [atomic]
    CB->>resetBuffer: load() → true → skip buffer & return 0
    Note over CB: stale IQ samples discarded
    GR->>resetBuffer: load() in acquireReadBuffer → true → flush head
    GR->>resetBuffer: store(false) [atomic]
    CB->>CB: timestamp = steady_clock::now()
    CB->>ring: enqueue IQ + timestamp
    GR->>GR: timeNs = _buf_timestamps[handle]
```

| Mod | What changed | Files |
|-----|-------------|-------|
| **22** | `bool resetBuffer` → `std::atomic<bool>` — eliminates data race | `SoapyAirspy.hpp`, `Settings.cpp`, `Streaming.cpp` |
| **23** | `std::vector<long long> _buf_timestamps` — per-buffer `steady_clock` nanosecond timestamps propagated to `timeNs` in `acquireReadBuffer` | `SoapyAirspy.hpp`, `Streaming.cpp` |
| **24** | `rx_callback`: early `return 0` when `resetBuffer.load()` is true — discards stale IQ at retune | `Streaming.cpp` |
| **25** | `setSampleRate()` snaps requested rate to nearest supported hardware rate, logs a warning if snapped — prevents silent wrong-rate failures | `Settings.cpp` |

```c++
// Mod 22 — atomic eliminates UB
std::atomic<bool> resetBuffer;

// Mod 23 — nanosecond receive timestamp
const long long nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
_buf_timestamps[_buf_tail] = nowNs;
// ... in acquireReadBuffer:
timeNs = _buf_timestamps[handle];

// Mod 24 — discard stale IQ during Doppler retune
if (SDR_UNLIKELY(self->resetBuffer.load())) return 0;

// Mod 25 — snap to nearest supported rate
const auto rates = listSampleRates(SOAPY_SDR_RX, 0);
auto it = std::min_element(rates.begin(), rates.end(), [&](double a, double b){
    return std::abs(a - rate) < std::abs(b - rate);
});
if (it != rates.end() && std::abs(*it - rate) > 1.0) {
    SoapySDR_logf(SOAPY_SDR_WARNING,
        "AirSpy: requested %.0f MSPS → snapped to %.0f MSPS",
        rate/1e6, (*it)/1e6);
    rate = *it;
}
```

**Files:** `SoapyAirspy.hpp`, `Streaming.cpp`, `Settings.cpp`

---

## 9. 📊 Diagnostics & Observability

### Problem

Operators running a SatNOGS ground station had no way to:
- Know whether the **F4TNK firmware** (with its custom gain tables) was loaded vs stock firmware.
- **Monitor buffer overflow counts** from the GNU Radio flowgraph or SatNOGS client without
  custom instrumentation.

### Solution

| Mod | What changed | Files |
|-----|-------------|-------|
| **26** | `readSetting("overflow_count")` returns `_overflowCount.load()` as a string; `getSettingInfo()` advertises it as a read-only INT setting | `Settings.cpp` |
| **27** | `getHardwareInfo()` calls `airspy_version_string_read()` and stores the result in `args["firmware"]` — visible in `SoapySDRUtil --probe` | `Settings.cpp` |

```c++
// Mod 26 — read overflow counter from Python / GR
// In Python:
// count = sdr.readSetting("overflow_count")

// Mod 27 — firmware version in --probe output
char fw_version[128] = {};
if (airspy_version_string_read(dev, fw_version, sizeof(fw_version)) == AIRSPY_SUCCESS)
    args["firmware"] = fw_version;
else
    args["firmware"] = "unknown";
```

```bash
# Usage: probe the device and check firmware
SoapySDRUtil --probe="driver=airspy"
# → hardware: firmware=airspy_ver_1.0.0-rc10-6-g4044438

# Read overflow count from Python:
python3 -c "
import SoapySDR
sdr = SoapySDR.Device({'driver':'airspy'})
print('overflows:', sdr.readSetting('overflow_count'))
"
```

**Files:** `Settings.cpp`

---

## Complete Optimization Table

| # | Category | Optimization | File(s) |
|---|----------|-------------|---------|
| 1 | Buffer | `DEFAULT_BUFFER_BYTES` 256KB → 512KB | `SoapyAirspy.hpp` |
| 2 | Buffer | `DEFAULT_NUM_BUFFERS` 8 → 15 | `SoapyAirspy.hpp` |
| 3 | Buffer | Configurable `buffers` stream arg (4–64) | `Streaming.cpp` |
| 4 | Buffer | Configurable `buflen` stream arg | `Streaming.cpp` |
| 5 | Buffer | Pre-allocation (`reserve` + `resize`) | `Streaming.cpp` |
| 6 | Buffer | State reset on `setupStream()` | `Streaming.cpp` |
| 7 | Buffer | Info logging (buffer config summary) | `Streaming.cpp` |
| 8 | Overflow | Keep 2 newest (not drain all) | `Streaming.cpp` |
| 9 | Overflow | Atomic `_overflowCount` counter | `SoapyAirspy.hpp`, `Streaming.cpp` |
| 10 | Overflow | Counter reset on `activateStream()` | `Streaming.cpp` |
| 11 | Callback | Pre-computed `bytes` variable | `Streaming.cpp` |
| 12 | Read | `const` pre-computed `returnedBytes` | `Streaming.cpp` |
| 13 | Gain | Default LNA=10, MIX=7, VGA=10 | `Settings.cpp` |
| 14 | Gain | Bit packing ON by default (−25% USB) | `Settings.cpp` |
| 15 | Gain | HW gains programmed at constructor | `Settings.cpp` |
| 16 | Gain | Linearity gain tables (0–21) | `Settings.cpp` |
| 17 | Gain | Sensitivity gain tables (0–21) | `Settings.cpp` |
| 18 | Frequency | PPM correction with auto re-apply | `Settings.cpp` |
| 19 | Frontend | DC offset mode = `true` | `Settings.cpp` |
| 20 | Format | CF32 native (fullScale=1.0, SSE2 path) | `Streaming.cpp` |
| 21 | Build | `-march=native -ffast-math -ftree-vectorize -flto` | `CMakeLists.txt` |
| — | Safety | `std::atomic<bool> streamActive` | `SoapyAirspy.hpp` |
| — | Perf | `SDR_LIKELY` / `SDR_UNLIKELY` branch hints | `SoapyAirspy.hpp`, `Streaming.cpp` |
| 22 | Thread Safety | `bool resetBuffer` → `std::atomic<bool>` — eliminates data race on Doppler retune | `SoapyAirspy.hpp`, `Settings.cpp`, `Streaming.cpp` |
| 23 | Timestamps | `_buf_timestamps[]` — per-buffer `steady_clock` ns, propagated to `timeNs` | `SoapyAirspy.hpp`, `Streaming.cpp` |
| 24 | Doppler | Skip stale IQ in `rx_callback` during `resetBuffer` — no retune glitch | `Streaming.cpp` |
| 25 | Sample Rate | Snap to nearest supported rate + log warning | `Settings.cpp` |
| 26 | Diagnostics | `readSetting("overflow_count")` — expose `_overflowCount` to gr-satnogs | `Settings.cpp` |
| 27 | Observability | `getHardwareInfo()` — `airspy_version_string_read()` → `args["firmware"]` | `Settings.cpp` |

---

## Build Instructions

### Prerequisites

```bash
# SoapySDR development files
sudo apt install libsoapysdr-dev soapysdr-tools

# Optimized libairspy (F4TNK branch)
git clone -b master-f4tnk https://github.com/f4tnk/airspyone_host.git
cd airspyone_host && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install && sudo ldconfig
```

### Build SoapyAirspy

```bash
git clone -b master-f4tnk https://github.com/f4tnk/SoapyAirspy.git
cd SoapyAirspy
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

### Verify Installation

```bash
# Check module is loaded
SoapySDRUtil --info

# Probe device
SoapySDRUtil --probe="driver=airspy"

# Check available settings
SoapySDRUtil --args="driver=airspy" --setting
```

---

## Usage Examples

### Optimal Satellite Reception (SatNOGS)

```bash
# High sensitivity mode for weak LEO satellites
SoapySDRUtil --args="driver=airspy,sensitivity_gain=15,ppm=1.2" \
    --rate=3e6 --freq=435e6 --format=CF32
```

### Strong Signal Environment

```bash
# Linearity mode to avoid intermodulation
SoapySDRUtil --args="driver=airspy,linearity_gain=10,bitpack=true" \
    --rate=6e6 --freq=137.5e6
```

### Custom Buffer Configuration

```python
import SoapySDR

sdr = SoapySDR.Device({"driver": "airspy"})
sdr.setSampleRate(SoapySDR.SOAPY_SDR_RX, 0, 3e6)
sdr.setFrequency(SoapySDR.SOAPY_SDR_RX, 0, "RF", 435e6)
sdr.writeSetting("sensitivity_gain", "15")
sdr.writeSetting("ppm", "1.5")

# 32 deep ring buffer for high-latency host
stream = sdr.setupStream(SoapySDR.SOAPY_SDR_RX, SoapySDR.SOAPY_SDR_CF32, [0],
    {"buffers": "32", "buflen": "1048576"})
sdr.activateStream(stream)
```

---

## Signal Path Overview

```mermaid
graph TB
    subgraph "🎯 Full Optimized Chain"
        A["📡 Antenna"] --> B["AirSpy R2<br/>12-bit @ 20 MSPS"]
        B --> C["USB 2.0<br/>Bit-packed ×0.75"]
        C --> D["libairspy F4TNK<br/>• 63-tap FIR<br/>• SSE2 IQ conv<br/>• DC removal"]
        D --> E["SoapyAirspy F4TNK<br/>• 15×512KB ring<br/>• CF32 native<br/>• PPM correction"]
        E --> F["GNU Radio<br/>gr-satnogs<br/>Demodulation"]
        F --> G["📊 Decoded<br/>Telemetry"]
    end

    style A fill:#95a5a6,color:#fff
    style B fill:#e74c3c,color:#fff
    style C fill:#f39c12,color:#fff
    style D fill:#e67e22,color:#fff
    style E fill:#3498db,color:#fff
    style F fill:#2ecc71,color:#fff
    style G fill:#27ae60,color:#fff
```

---

## Related Projects

| Repository | Description |
|---|---|
| [airspyone_host (F4TNK)](https://github.com/f4tnk/airspyone_host/tree/master-f4tnk) | Optimized libairspy driver (32 optimizations) |
| [SoapyAirspy (F4TNK)](https://github.com/f4tnk/SoapyAirspy/tree/master-f4tnk) | This repository — SoapySDR wrapper optimizations |
| [gr-satnogs (F4TNK)](https://gitlab.com/f4tnk/gr-satnogs) | GNU Radio SatNOGS blocks |
| [satnogs-flowgraphs (F4TNK)](https://gitlab.com/f4tnk/satnogs-flowgraphs) | Optimized satellite flowgraphs |

---

*Optimized by **F4TNK** for SatNOGS station 3762 — 73 de F4TNK* 🛰️
