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
#include <SoapySDR/Logger.hpp>
#include <cstdio>
#include <SoapySDR/Formats.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy
#include <chrono>   // Mod 23: per-buffer timestamps


std::vector<std::string> SoapyAirspy::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;

    // formats.push_back("CS8");
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);

    return formats;
}

std::string SoapyAirspy::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
     fullScale = 1.0;
     return SOAPY_SDR_CF32;
}

SoapySDR::ArgInfoList SoapyAirspy::getStreamArgsInfo(const int direction, const size_t channel) const {
    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo buffersArg;
    buffersArg.key = "buffers";
    buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
    buffersArg.name = "Buffer Count";
    buffersArg.description = "Number of async ring buffers for the producer-consumer queue.";
    buffersArg.units = "";
    buffersArg.type = SoapySDR::ArgInfo::INT;
    streamArgs.push_back(buffersArg);

    SoapySDR::ArgInfo bufLenArg;
    bufLenArg.key = "buflen";
    bufLenArg.value = std::to_string(DEFAULT_BUFFER_BYTES);
    bufLenArg.name = "Buffer Size";
    bufLenArg.description = "Size of each async buffer in bytes. Larger values reduce USB overhead.";
    bufLenArg.units = "bytes";
    bufLenArg.type = SoapySDR::ArgInfo::INT;
    streamArgs.push_back(bufLenArg);

    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

static int _rx_callback(airspy_transfer *t)
{
    //printf("_rx_callback\n");
    SoapyAirspy *self = (SoapyAirspy *)t->ctx;
    return self->rx_callback(t);
}

int SoapyAirspy::rx_callback(airspy_transfer *t)
{
    if (SDR_UNLIKELY(sampleRateChanged.load())) {
        return 1;
    }

    // Mod 24: skip new samples while main thread is draining stale data
    // after a frequency retune (Doppler event). Avoids IQ glitch.
    if (SDR_UNLIKELY(resetBuffer.load())) {
        return 0;
    }

    //overflow condition: the caller is not reading fast enough
    if (SDR_UNLIKELY(_buf_count == numBuffers))
    {
        _overflowEvent = true;
        _overflowCount++;
        return 0;
    }

    // Mod 23: stamp receive time before memcpy (monotonic clock, no wall-time drift)
    const long long nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    const size_t bytes = t->sample_count * bytesPerSample;

    //copy into the buffer queue
    auto &buff = _buffs[_buf_tail];
    buff.resize(bytes);
    std::memcpy(buff.data(), t->samples, bytes);
    _buf_timestamps[_buf_tail] = nowNs;

    //increment the tail pointer
    _buf_tail = (_buf_tail + 1) % numBuffers;

    //increment buffers available under lock
    //to avoid race in acquireReadBuffer wait
    {
        std::lock_guard<std::mutex> lock(_buf_mutex);
        _buf_count++;
    }

    //notify readStream()
    _buf_cond.notify_one();

    return 0;
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyAirspy::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args)
{
    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    airspy_sample_type asFormat = AIRSPY_SAMPLE_INT16_IQ;

    //check the format
    if (format == SOAPY_SDR_CF32) {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
        asFormat = AIRSPY_SAMPLE_FLOAT32_IQ;
    } else if (format == SOAPY_SDR_CS16) {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
        asFormat = AIRSPY_SAMPLE_INT16_IQ;
    } else {
        throw std::runtime_error(
                "setupStream invalid format '" + format
                        + "' -- Only CS16 and CF32 are supported by SoapyAirspy module.");
    }

    airspy_set_sample_type(dev, asFormat);
    sampleRateChanged.store(true);

    bytesPerSample = SoapySDR::formatToSize(format);

    //parse stream args for buffer configuration
    size_t reqBufferBytes = DEFAULT_BUFFER_BYTES;
    if (args.count("buffers") != 0) {
        try { numBuffers = std::stoul(args.at("buffers")); }
        catch (...) { numBuffers = DEFAULT_NUM_BUFFERS; }
        if (numBuffers < 4) numBuffers = 4;
        if (numBuffers > 64) numBuffers = 64;
    }
    if (args.count("buflen") != 0) {
        try { reqBufferBytes = std::stoul(args.at("buflen")); }
        catch (...) { reqBufferBytes = DEFAULT_BUFFER_BYTES; }
    }

    //We get this many complex samples over the bus.
    //AirSpy delivers sample_count as number of I/Q pairs.
    //reqBufferBytes / 4 gives sample count for both CS16 (4 bytes/sample)
    //and CF32 (the actual buffer memory is bufferLength * bytesPerSample).
    bufferLength = reqBufferBytes / 4;

    //clear async fifo counts
    _buf_tail = 0;
    _buf_count = 0;
    _buf_head = 0;
    _overflowEvent = false;
    _overflowCount = 0;

    //allocate buffers — pre-allocate to avoid runtime resizing
    _buffs.resize(numBuffers);
    for (auto &buff : _buffs) buff.reserve(bufferLength * bytesPerSample);
    for (auto &buff : _buffs) buff.resize(bufferLength * bytesPerSample);
    // Mod 23: allocate parallel timestamp array
    _buf_timestamps.assign(numBuffers, 0LL);

    fprintf(stderr, "SoapyAirspy | stream: %zu buffers \xc3\x97 %u samples (%u bytes each), format=%s\n",
        numBuffers, bufferLength, bufferLength * (unsigned)bytesPerSample, format.c_str());

    return (SoapySDR::Stream *) this;
}

void SoapyAirspy::closeStream(SoapySDR::Stream *stream)
{
    _buffs.clear();
}

size_t SoapyAirspy::getStreamMTU(SoapySDR::Stream *stream) const
{
    return bufferLength;
}

int SoapyAirspy::activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems)
{
    if (flags != 0) {
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    
    resetBuffer.store(true);
    bufferedElems = 0;
    _overflowCount.store(0);
    // Mod 23: reset timestamps
    std::fill(_buf_timestamps.begin(), _buf_timestamps.end(), 0LL);
    
    if (sampleRateChanged.load()) {
        int ret = airspy_set_samplerate(dev, sampleRate);
        if (ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_samplerate(%u) failed: %d", sampleRate, ret);
        }
        sampleRateChanged.store(false);
    }
    int ret = airspy_start_rx(dev, &_rx_callback, (void *) this);
    if (ret != AIRSPY_SUCCESS) {
        SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_start_rx() failed: %d", ret);
        return SOAPY_SDR_STREAM_ERROR;
    }
    streamActive.store(true);

    //log final SatNOGS configuration at stream start — this is THE authoritative config
    char fw_ver[40] = {};
    airspy_version_string_read(dev, fw_ver, sizeof(fw_ver));
    if (sensitivityGain > 0)
        fprintf(stderr,
            "SoapyAirspy | streaming: %.6f MHz @ %.1f MSPS | sensitivity=%d | packing=%s bias=%s PPM=%+.1f | %s\n",
            centerFrequency / 1e6, sampleRate / 1e6, sensitivityGain,
            bitPack ? "on" : "off", rfBias ? "on" : "off", ppmCorrection, fw_ver);
    else if (linearityGain > 0)
        fprintf(stderr,
            "SoapyAirspy | streaming: %.6f MHz @ %.1f MSPS | linearity=%d | packing=%s bias=%s PPM=%+.1f | %s\n",
            centerFrequency / 1e6, sampleRate / 1e6, linearityGain,
            bitPack ? "on" : "off", rfBias ? "on" : "off", ppmCorrection, fw_ver);
    else
        fprintf(stderr,
            "SoapyAirspy | streaming: %.6f MHz @ %.1f MSPS | LNA=%d MIX=%d VGA=%d | packing=%s bias=%s PPM=%+.1f | %s\n",
            centerFrequency / 1e6, sampleRate / 1e6, lnaGain, mixerGain, vgaGain,
            bitPack ? "on" : "off", rfBias ? "on" : "off", ppmCorrection, fw_ver);

    return 0;
}

int SoapyAirspy::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

    airspy_stop_rx(dev);
    
    streamActive = false;

    const auto overflows = _overflowCount.load();
    if (overflows > 0)
        SoapySDR_logf(SOAPY_SDR_WARNING, "SoapyAirspy | session ended: %zu USB overflow(s) detected", overflows);
    else
        fprintf(stderr, "SoapyAirspy | session ended: 0 overflows\n");
    
    return 0;
}

int SoapyAirspy::readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs)
{    
    if (SDR_UNLIKELY(!airspy_is_streaming(dev))) {
        return 0;
    }
    
    if (SDR_UNLIKELY(sampleRateChanged.load())) {
        airspy_stop_rx(dev);
        int ret = airspy_set_samplerate(dev, sampleRate);
        if (ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_set_samplerate(%u) failed during readStream: %d", sampleRate, ret);
        }
        ret = airspy_start_rx(dev, &_rx_callback, (void *) this);
        if (ret != AIRSPY_SUCCESS) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "airspy_start_rx() failed during readStream: %d", ret);
            return SOAPY_SDR_STREAM_ERROR;
        }
        sampleRateChanged.store(false);
    }

    //this is the user's buffer for channel 0
    char *buff0 = (char *)buffs[0];

    //are elements left in the buffer? if not, do a new read.
    if (bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
        if (ret < 0) return ret;
        bufferedElems = ret;
    }

    const size_t returnedElems = std::min(bufferedElems, numElems);
    const size_t returnedBytes = returnedElems * bytesPerSample;

    //copy into user's buff0
    std::memcpy(buff0, _currentBuff, returnedBytes);
    
    //bump variables for next call into readStream
    bufferedElems -= returnedElems;
    _currentBuff += returnedBytes;

    //return number of elements written to buff0
    if (bufferedElems != 0) flags |= SOAPY_SDR_MORE_FRAGMENTS;
    else this->releaseReadBuffer(stream, _currentHandle);
    return returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapyAirspy::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return _buffs.size();
}

int SoapyAirspy::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    buffs[0] = (void *)_buffs[handle].data();
    return 0;
}

int SoapyAirspy::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    //reset is issued by various settings
    //to drain old data out of the queue
    // Mod 22: use atomic load() for thread-safe check
    if (resetBuffer.load())
    {
        //drain all buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        // Mod 24: allow rx_callback to resume writing after drain
        resetBuffer.store(false);
        _overflowEvent = false;
    }

    //handle overflow from the rx callback thread
    //keep only the 2 newest buffers to minimize data loss
    if (_overflowEvent)
    {
        const size_t cnt = _buf_count.load();
        if (cnt > 2) {
            const size_t toDrain = cnt - 2;
            _buf_head = (_buf_head + toDrain) % numBuffers;
            _buf_count -= toDrain;
        }
        _overflowEvent = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
    }

    //wait for a buffer to become available
    if (_buf_count == 0)
    {
        std::unique_lock <std::mutex> lock(_buf_mutex);
        _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]{return _buf_count != 0;});
        if (_buf_count == 0) return SOAPY_SDR_TIMEOUT;
    }

    //extract handle and buffer
    handle = _buf_head;
    _buf_head = (_buf_head + 1) % numBuffers;
    buffs[0] = (void *)_buffs[handle].data();
    // Mod 23: return precise receive timestamp to caller (gr-satnogs Doppler)
    timeNs = _buf_timestamps[handle];
    flags = 0;

    //return number available
    return _buffs[handle].size() / bytesPerSample;
}

void SoapyAirspy::releaseReadBuffer(
    SoapySDR::Stream *stream,
    const size_t handle)
{
    //TODO this wont handle out of order releases
    _buf_count--;
}
