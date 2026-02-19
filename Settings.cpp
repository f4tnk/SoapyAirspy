/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Charles J. Cliffe

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyAirspy.hpp"
#include <cinttypes>
#include <cstdio>

SoapyAirspy::SoapyAirspy(const SoapySDR::Kwargs &args)
{
    sampleRate = 3000000;
    centerFrequency = 0;

    numBuffers = DEFAULT_NUM_BUFFERS;

    agcMode = false;
    rfBias = false;
    bitPack = true;

    bufferedElems = 0;
    // Mod 22: resetBuffer is std::atomic<bool> — use store() for explicit atomic write
    resetBuffer.store(false);

    streamActive = false;
    sampleRateChanged.store(false);
    _overflowCount.store(0);

    dev = nullptr;

    // Optimized default gains for weak-signal reception (satellites, APRS, AIS)
    lnaGain = 10;
    mixerGain = 7;
    vgaGain = 10;
    linearityGain = 0;
    sensitivityGain = 0;
    ppmCorrection = 0.0;

    dev = nullptr;
    std::stringstream serialstr;
    serialstr.str("");

    if (args.count("serial") != 0)
    {
        try {
            serial = std::stoull(args.at("serial"), nullptr, 16);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("serial is not a hex number");
        } catch (const std::out_of_range &) {
            throw std::runtime_error("serial value of out range");
        }
        serialstr << std::hex << serial;
        if (airspy_open_sn(&dev, serial) != AIRSPY_SUCCESS) {
            throw std::runtime_error("Unable to open AirSpy device with serial " + serialstr.str());
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Found AirSpy device: serial = %" PRIx64, serial);
    }
    else
    {
        if (airspy_open(&dev) != AIRSPY_SUCCESS) {
            throw std::runtime_error("Unable to open AirSpy device");
        }
    }

    //program hardware with optimized defaults first
    airspy_set_lna_gain(dev, lnaGain);
    airspy_set_mixer_gain(dev, mixerGain);
    airspy_set_vga_gain(dev, vgaGain);
    airspy_set_packing(dev, bitPack ? 1 : 0);

    //apply arguments to settings — may override the defaults above
    for (const auto &info : this->getSettingInfo())
    {
        const auto it = args.find(info.key);
        if (it != args.end()) this->writeSetting(it->first, it->second);
    }

    //log firmware version + available sample rates at init
    char fw_ver[40] = {};
    if (airspy_version_string_read(dev, fw_ver, sizeof(fw_ver)) == AIRSPY_SUCCESS)
        fprintf(stderr, "SoapyAirspy | firmware: %s\n", fw_ver);

    const auto rates = this->listSampleRates(SOAPY_SDR_RX, 0);
    std::string ratesStr;
    for (size_t i = 0; i < rates.size(); i++) {
        if (i > 0) ratesStr += ", ";
        ratesStr += std::to_string((unsigned)(rates[i] / 1e6)) + " MSPS";
    }
    fprintf(stderr, "SoapyAirspy | sample rates: %s\n", ratesStr.c_str());

    //log initial defaults (will be overridden by SatNOGS setGain calls)
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AirSpy defaults: LNA=%d MIX=%d VGA=%d packing=%s bias=%s",
        lnaGain, mixerGain, vgaGain, bitPack ? "on" : "off", rfBias ? "on" : "off");

    //log GPSDO / SI5351C clock reference status at init
    {
        uint8_t reg0 = 0xFF, reg15 = 0;
        const bool r0_ok = (airspy_si5351c_read(dev, 0, &reg0) == AIRSPY_SUCCESS);
        const bool r15_ok = (airspy_si5351c_read(dev, 15, &reg15) == AIRSPY_SUCCESS);

        if (r0_ok && r15_ok) {
            const bool gpsdo_signal = !(reg0 & 0x10);  // LOS_CLKIN: 0=present
            const bool pll_a_lock = !(reg0 & 0x20);    // LOL_A: 0=locked
            const bool pll_b_lock = !(reg0 & 0x40);    // LOL_B: 0=locked
            const bool sys_init = !!(reg0 & 0x80);     // SYS_INIT: 1=initializing
            const bool plla_clkin = !!(reg15 & 0x04);   // PLLA_SRC: 1=CLKIN
            const bool pllb_clkin = !!(reg15 & 0x08);   // PLLB_SRC: 1=CLKIN

            fprintf(stderr, "SoapyAirspy | SI5351C status: reg0=0x%02X reg15=0x%02X\n", reg0, reg15);
            fprintf(stderr, "SoapyAirspy | 🔧 Clock source: PLL_A=%s, PLL_B=%s\n",
                plla_clkin ? "CLKIN (GPSDO)" : "XTAL (internal)",
                pllb_clkin ? "CLKIN (GPSDO)" : "XTAL (internal)");

            if (gpsdo_signal) {
                fprintf(stderr, "SoapyAirspy | ✅ GPSDO: Signal detected on CLKIN\n");
            } else {
                fprintf(stderr, "SoapyAirspy | ⚠️  GPSDO: No signal on CLKIN (LOS_CLKIN=1)\n");
            }
            fprintf(stderr, "SoapyAirspy | 🔒 PLL Lock: A=%s B=%s%s\n",
                pll_a_lock ? "LOCKED" : "UNLOCKED",
                pll_b_lock ? "LOCKED" : "UNLOCKED",
                sys_init ? " (SYS_INIT in progress)" : "");
        } else {
            fprintf(stderr, "SoapyAirspy | ⚠️  SI5351C register read failed (no GPSDO diagnostics)\n");
        }
    }
}

SoapyAirspy::~SoapyAirspy(void)
{
    airspy_close(dev);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyAirspy::getDriverKey(void) const
{
    return "Airspy";
}

std::string SoapyAirspy::getHardwareKey(void) const
{
    return "Airspy";
}

SoapySDR::Kwargs SoapyAirspy::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    std::stringstream serialstr;
    serialstr.str("");
    serialstr << std::hex << serial;
    args["serial"] = serialstr.str();

    // Mod 27: expose firmware version string for F4TNK detection in SatNOGS
    // airspy_version_string_read returns e.g. "AirSpy NOS 88e4bd6 2026-02-18"
    char fw_version[40] = {};
    if (airspy_version_string_read(dev, fw_version, sizeof(fw_version)) == AIRSPY_SUCCESS)
        args["firmware"] = std::string(fw_version);
    else
        args["firmware"] = "unknown";

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyAirspy::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    return antennas;
}

void SoapyAirspy::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    // TODO
}

std::string SoapyAirspy::getAntenna(const int direction, const size_t channel) const
{
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyAirspy::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return true;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> results;

    results.push_back("LNA");
    results.push_back("MIX");
    results.push_back("VGA");

    return results;
}

bool SoapyAirspy::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapyAirspy::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    agcMode = automatic;

    airspy_set_lna_agc(dev, agcMode?1:0);
    airspy_set_mixer_agc(dev, agcMode?1:0);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AGC: %s", automatic ? "Automatic" : "Manual");
}

bool SoapyAirspy::getGainMode(const int direction, const size_t channel) const
{
    return agcMode;
}

void SoapyAirspy::setGain(const int direction, const size_t channel, const double value)
{
    //set the overall gain by distributing it across available gain elements
    //OR delete this function to use SoapySDR's default gain distribution algorithm...
    SoapySDR::Device::setGain(direction, channel, value);
}

void SoapyAirspy::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    if (name == "LNA")
    {
        lnaGain = uint8_t(value);
        airspy_set_lna_gain(dev, lnaGain);
    }
    else if (name == "MIX")
    {
        mixerGain = uint8_t(value);
        airspy_set_mixer_gain(dev, mixerGain);
    }
    else if (name == "VGA")
    {
        vgaGain = uint8_t(value);
        airspy_set_vga_gain(dev, vgaGain);
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AirSpy setGain: %s=%d", name.c_str(), (int)value);
}

double SoapyAirspy::getGain(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA")
    {
        return lnaGain;
    }
    else if (name == "MIX")
    {
        return mixerGain;
    }
    else if (name == "VGA")
    {
        return vgaGain;
    }

    return 0;
}

SoapySDR::Range SoapyAirspy::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "LNA" || name == "MIX" || name == "VGA") {
        return SoapySDR::Range(0, 15);
    }

    return SoapySDR::Range(0, 15);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyAirspy::setFrequency(
        const int direction,
        const size_t channel,
        const std::string &name,
        const double frequency,
        const SoapySDR::Kwargs &args)
{
    if (name == "RF")
    {
        centerFrequency = (uint32_t) frequency;
        // Mod 22: atomic store — safe from USB callback thread
        resetBuffer.store(true);
        //apply PPM correction to compensate crystal oscillator drift
        const uint32_t corrected = (uint32_t)(frequency * (1.0 + ppmCorrection / 1e6));
        fprintf(stderr, "SoapyAirspy | tune: %.6f MHz (PPM: %+.1f)\n",
            corrected / 1e6, ppmCorrection);
        airspy_set_freq(dev, corrected);
    }
}

double SoapyAirspy::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "RF")
    {
        return (double) centerFrequency;
    }

    return 0;
}

std::vector<std::string> SoapyAirspy::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return names;
}

SoapySDR::RangeList SoapyAirspy::getFrequencyRange(
        const int direction,
        const size_t channel,
        const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        results.push_back(SoapySDR::Range(24000000, 1800000000));
    }
    return results;
}

SoapySDR::ArgInfoList SoapyAirspy::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyAirspy::setSampleRate(const int direction, const size_t channel, const double rate)
{
    // Mod 25: snap requested rate to nearest hardware-supported value
    // prevents silent failures when gr-satnogs requests e.g. 2.4 MSPS
    const auto supported = this->listSampleRates(direction, channel);
    double snapped = rate;
    if (!supported.empty())
    {
        snapped = supported[0];
        double bestDiff = std::abs(rate - supported[0]);
        for (const auto r : supported)
        {
            const double diff = std::abs(rate - r);
            if (diff < bestDiff) { bestDiff = diff; snapped = r; }
        }
        if (snapped != rate)
            SoapySDR_logf(SOAPY_SDR_WARNING,
                "SoapyAirspy: requested rate %.0f Hz not supported, snapped to %.0f Hz",
                rate, snapped);
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting sample rate: %.0f", snapped);

    if (sampleRate != (uint32_t)snapped) {
        sampleRate = (uint32_t)snapped;
        // Mod 22: atomic store
        resetBuffer.store(true);
        sampleRateChanged.store(true);
    }
}

double SoapyAirspy::getSampleRate(const int direction, const size_t channel) const
{
    return sampleRate;
}

std::vector<double> SoapyAirspy::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> results;

    uint32_t numRates;
	airspy_get_samplerates(dev, &numRates, 0);

	std::vector<uint32_t> samplerates;
    samplerates.resize(numRates);

	airspy_get_samplerates(dev, samplerates.data(), numRates);

	for (auto i: samplerates) {
        results.push_back(i);
	}

    return results;
}

void SoapyAirspy::setBandwidth(const int direction, const size_t channel, const double bw)
{
    SoapySDR::Device::setBandwidth(direction, channel, bw);
}

double SoapyAirspy::getBandwidth(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getBandwidth(direction, channel);
}

std::vector<double> SoapyAirspy::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> results;

    return results;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyAirspy::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    // Bias-T
    SoapySDR::ArgInfo biasOffsetArg;
    biasOffsetArg.key = "biastee";
    biasOffsetArg.value = "false";
    biasOffsetArg.name = "Bias tee";
    biasOffsetArg.description = "Enable the 4.5v DC Bias tee to power SpyVerter / LNA / etc. via antenna connection.";
    biasOffsetArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(biasOffsetArg);

    // bitpack — enabled by default for 25% USB bandwidth savings
    SoapySDR::ArgInfo bitPackingArg;
    bitPackingArg.key = "bitpack";
    bitPackingArg.value = "true";
    bitPackingArg.name = "Bit Pack";
    bitPackingArg.description = "Pack 4x 12-bit samples into 3x 16-bit words (25% less USB traffic). Enabled by default.";
    bitPackingArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(bitPackingArg);

    // PPM frequency correction
    SoapySDR::ArgInfo ppmArg;
    ppmArg.key = "ppm";
    ppmArg.value = "0.0";
    ppmArg.name = "PPM Correction";
    ppmArg.description = "Crystal oscillator frequency correction in parts per million. Compensates for TCXO drift.";
    ppmArg.type = SoapySDR::ArgInfo::FLOAT;
    setArgs.push_back(ppmArg);

    // Linearity gain mode
    SoapySDR::ArgInfo linGainArg;
    linGainArg.key = "linearity_gain";
    linGainArg.value = "0";
    linGainArg.name = "Linearity Gain";
    linGainArg.description = "Pre-tuned gain table optimized for linearity (0-21). Sets LNA/MIX/VGA internally for best IMD performance.";
    linGainArg.type = SoapySDR::ArgInfo::INT;
    linGainArg.range = SoapySDR::Range(0, 21);
    setArgs.push_back(linGainArg);

    // Sensitivity gain mode
    SoapySDR::ArgInfo sensGainArg;
    sensGainArg.key = "sensitivity_gain";
    sensGainArg.value = "0";
    sensGainArg.name = "Sensitivity Gain";
    sensGainArg.description = "Pre-tuned gain table optimized for sensitivity/SNR (0-21). Best for weak satellite and telemetry signals.";
    sensGainArg.type = SoapySDR::ArgInfo::INT;
    sensGainArg.range = SoapySDR::Range(0, 21);
    setArgs.push_back(sensGainArg);

    // Mod 26: overflow_count — read-only diagnostic (writable key ignored)
    SoapySDR::ArgInfo overflowArg;
    overflowArg.key = "overflow_count";
    overflowArg.value = "0";
    overflowArg.name = "Overflow Count";
    overflowArg.description = "Number of USB buffer overflow events since last activateStream(). Poll to detect frame loss in gr-satnogs.";
    overflowArg.type = SoapySDR::ArgInfo::INT;
    setArgs.push_back(overflowArg);

    return setArgs;
}

void SoapyAirspy::writeSetting(const std::string &key, const std::string &value)
{
    if (key == "biastee") {
        bool enable = (value == "true");
        rfBias = enable;
        airspy_set_rf_bias(dev, enable);
    }
    else if (key == "bitpack") {
        bool enable = (value == "true");
        bitPack = enable;
        airspy_set_packing(dev, enable ? 1 : 0);
    }
    else if (key == "ppm") {
        try { ppmCorrection = std::stod(value); }
        catch (...) { ppmCorrection = 0.0; }
        //re-apply current frequency with new correction
        if (centerFrequency > 0) {
            const uint32_t corrected = (uint32_t)(centerFrequency * (1.0 + ppmCorrection / 1e6));
            airspy_set_freq(dev, corrected);
        }
        fprintf(stderr, "SoapyAirspy | PPM correction: %.2f ppm\n", ppmCorrection);
    }
    else if (key == "linearity_gain") {
        try { linearityGain = uint8_t(std::stoi(value)); }
        catch (...) { linearityGain = 0; }
        if (linearityGain > 21) linearityGain = 21;
        airspy_set_linearity_gain(dev, linearityGain);
        fprintf(stderr, "SoapyAirspy | linearity gain: %d\n", linearityGain);
    }
    else if (key == "sensitivity_gain") {
        try { sensitivityGain = uint8_t(std::stoi(value)); }
        catch (...) { sensitivityGain = 0; }
        if (sensitivityGain > 21) sensitivityGain = 21;
        airspy_set_sensitivity_gain(dev, sensitivityGain);
        fprintf(stderr, "SoapyAirspy | sensitivity gain: %d\n", sensitivityGain);
    }
}

std::string SoapyAirspy::readSetting(const std::string &key) const
{
    if (key == "biastee") {
        return rfBias ? "true" : "false";
    }
    if (key == "bitpack") {
        return bitPack ? "true" : "false";
    }
    if (key == "ppm") {
        return std::to_string(ppmCorrection);
    }
    if (key == "linearity_gain") {
        return std::to_string(linearityGain);
    }
    if (key == "sensitivity_gain") {
        return std::to_string(sensitivityGain);
    }
    // Mod 26: expose overflow counter via SoapySDR settings API
    // gr-satnogs / GNU Radio flowgraphs can poll this to detect buffer loss
    if (key == "overflow_count") {
        return std::to_string(_overflowCount.load());
    }
    return "";
}

/*******************************************************************
 * Clock Source API — F4TNK GPSDO support
 * SI5351C register map:
 *   Reg 0  — Device Status: bit4=LOS_CLKIN, bit5=LOL_A, bit6=LOL_B, bit7=SYS_INIT
 *   Reg 15 — PLL Input Source: bit2=PLLA_SRC, bit3=PLLB_SRC (0=XTAL, 1=CLKIN)
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listClockSources(void) const
{
    return {"internal", "external"};
}

void SoapyAirspy::setClockSource(const std::string &source)
{
    // SI5351C PLL source is configured by firmware at boot.
    // Changing it at runtime requires reprogramming PLL registers
    // which would glitch the ADC clock — log warning only.
    if (source == "external") {
        SoapySDR_logf(SOAPY_SDR_WARNING,
            "GPSDO external clock source must be configured in firmware (SI5351C reg 15). "
            "Connect GPSDO to CLKIN before power-on.");
    }
}

std::string SoapyAirspy::getClockSource(void) const
{
    uint8_t reg15 = 0;
    if (airspy_si5351c_read(dev, 15, &reg15) != AIRSPY_SUCCESS) {
        return "unknown";
    }
    // Bit 2: PLLA_SRC — 0=XTAL (internal), 1=CLKIN (external/GPSDO)
    return (reg15 & 0x04) ? "external" : "internal";
}

/*******************************************************************
 * Sensor API — F4TNK GPSDO + PLL diagnostics
 * Reads SI5351C Device Status Register (reg 0) via USB I2C bridge:
 *   Bit 7: SYS_INIT  (1 = system initializing)
 *   Bit 6: LOL_B     (1 = PLL B lost lock)
 *   Bit 5: LOL_A     (1 = PLL A lost lock)
 *   Bit 4: LOS_CLKIN (1 = no signal on CLKIN pin → no GPSDO)
 *   Bit 1:0: REVID   (chip revision)
 ******************************************************************/

std::vector<std::string> SoapyAirspy::listSensors(void) const
{
    return {"gpsdo_locked", "pll_a_locked", "pll_b_locked", "clock_source", "si5351c_status"};
}

SoapySDR::ArgInfo SoapyAirspy::getSensorInfo(const std::string &key) const
{
    SoapySDR::ArgInfo info;
    info.key = key;

    if (key == "gpsdo_locked") {
        info.name = "GPSDO Locked";
        info.description = "SI5351C CLKIN signal present (GPSDO 10MHz reference detected). "
                           "Reads LOS_CLKIN bit from Device Status Register 0.";
        info.type = SoapySDR::ArgInfo::BOOL;
    }
    else if (key == "pll_a_locked") {
        info.name = "PLL A Locked";
        info.description = "SI5351C PLL A lock status. Loss of lock indicates reference instability.";
        info.type = SoapySDR::ArgInfo::BOOL;
    }
    else if (key == "pll_b_locked") {
        info.name = "PLL B Locked";
        info.description = "SI5351C PLL B lock status. PLL B generates the ADC sampling clock.";
        info.type = SoapySDR::ArgInfo::BOOL;
    }
    else if (key == "clock_source") {
        info.name = "Clock Source";
        info.description = "Active PLL reference source: XTAL (internal 25MHz) or CLKIN (external GPSDO).";
        info.type = SoapySDR::ArgInfo::STRING;
    }
    else if (key == "si5351c_status") {
        info.name = "SI5351C Status";
        info.description = "Raw SI5351C Device Status Register 0 value (hex). "
                           "Bits: [7]=SYS_INIT [6]=LOL_B [5]=LOL_A [4]=LOS_CLKIN [1:0]=REVID";
        info.type = SoapySDR::ArgInfo::STRING;
    }

    return info;
}

std::string SoapyAirspy::readSensor(const std::string &key) const
{
    uint8_t reg0 = 0xFF;  // default = all errors
    const bool reg0_ok = (airspy_si5351c_read(dev, 0, &reg0) == AIRSPY_SUCCESS);

    if (key == "gpsdo_locked") {
        if (!reg0_ok) return "false";
        // LOS_CLKIN is bit 4: 0 = CLKIN signal present (GPSDO locked), 1 = lost
        return (reg0 & 0x10) ? "false" : "true";
    }
    else if (key == "pll_a_locked") {
        if (!reg0_ok) return "false";
        // LOL_A is bit 5: 0 = locked, 1 = lost lock
        return (reg0 & 0x20) ? "false" : "true";
    }
    else if (key == "pll_b_locked") {
        if (!reg0_ok) return "false";
        // LOL_B is bit 6: 0 = locked, 1 = lost lock
        return (reg0 & 0x40) ? "false" : "true";
    }
    else if (key == "clock_source") {
        uint8_t reg15 = 0;
        if (airspy_si5351c_read(dev, 15, &reg15) != AIRSPY_SUCCESS) return "unknown";
        return (reg15 & 0x04) ? "CLKIN (GPSDO)" : "XTAL (internal)";
    }
    else if (key == "si5351c_status") {
        if (!reg0_ok) return "read_error";
        char buf[16];
        snprintf(buf, sizeof(buf), "0x%02X", reg0);
        return std::string(buf);
    }

    return "";
}
