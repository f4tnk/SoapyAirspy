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
        SoapySDR_logf(SOAPY_SDR_INFO, "AirSpy firmware: %s", fw_ver);

    const auto rates = this->listSampleRates(SOAPY_SDR_RX, 0);
    std::string ratesStr;
    for (size_t i = 0; i < rates.size(); i++) {
        if (i > 0) ratesStr += ", ";
        ratesStr += std::to_string((unsigned)(rates[i] / 1e6)) + " MSPS";
    }
    SoapySDR_logf(SOAPY_SDR_INFO, "AirSpy sample rates: %s", ratesStr.c_str());

    //log initial defaults (will be overridden by SatNOGS setGain calls)
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AirSpy defaults: LNA=%d MIX=%d VGA=%d packing=%s bias=%s",
        lnaGain, mixerGain, vgaGain, bitPack ? "on" : "off", rfBias ? "on" : "off");
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
        SoapySDR_logf(SOAPY_SDR_INFO, "AirSpy tune: %.6f MHz (PPM: %+.1f)",
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
        SoapySDR_logf(SOAPY_SDR_INFO, "PPM correction set to %.2f", ppmCorrection);
    }
    else if (key == "linearity_gain") {
        try { linearityGain = uint8_t(std::stoi(value)); }
        catch (...) { linearityGain = 0; }
        if (linearityGain > 21) linearityGain = 21;
        airspy_set_linearity_gain(dev, linearityGain);
        SoapySDR_logf(SOAPY_SDR_INFO, "Linearity gain set to %d", linearityGain);
    }
    else if (key == "sensitivity_gain") {
        try { sensitivityGain = uint8_t(std::stoi(value)); }
        catch (...) { sensitivityGain = 0; }
        if (sensitivityGain > 21) sensitivityGain = 21;
        airspy_set_sensitivity_gain(dev, sensitivityGain);
        SoapySDR_logf(SOAPY_SDR_INFO, "Sensitivity gain set to %d", sensitivityGain);
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
