// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

#include "iris_formats.hpp"
#include <SoapySDR/Formats.hpp>
#include <stdexcept>
#include <iostream>
#include <cstring> //memcpy

/***************************************************
 * All supported format combinations
 * given hardware limitations and converters below
 **************************************************/
void resolveFormats(
    const size_t numChannels,
    const std::string &localFormat,
    const std::string &reqWireFmt,
    StreamFormat &localFormatOut,
    std::string &remoteFormatOut,
    size_t &bytesPerElement)
{
    localFormatOut = SF_UNDEFINED;

    //automatic conversions and wire type selections for the specified target formats
    if (numChannels == 2 and localFormat == SOAPY_SDR_CF32) localFormatOut = SF_CF32_x2_WIRE48;
    if (numChannels == 2 and localFormat == SOAPY_SDR_CS16) localFormatOut = SF_CS16_x2_WIRE48;
    if (numChannels == 2 and localFormat == SOAPY_SDR_CS12) localFormatOut = SF_CS12_x2_WIRE48;
    if (numChannels == 1 and localFormat == SOAPY_SDR_CF32) localFormatOut = SF_CF32_x1_WIRE24;
    if (numChannels == 1 and localFormat == SOAPY_SDR_CS16) localFormatOut = SF_CS16_x1_WIRE24;
    if (numChannels == 1 and localFormat == SOAPY_SDR_CS12) localFormatOut = SF_CS12_x1_WIRE24;
    if (numChannels == 1 and localFormat == SOAPY_SDR_CS8 ) localFormatOut = SF_CS8_x1_WIRE16;

    //formats for specific requested wire formats
    if (numChannels == 1 and localFormat == SOAPY_SDR_CF32 and reqWireFmt == SOAPY_SDR_CS8 ) localFormatOut = SF_CF32_x1_WIRE16;
    if (numChannels == 1 and localFormat == SOAPY_SDR_CS16 and reqWireFmt == SOAPY_SDR_CS8 ) localFormatOut = SF_CS16_x1_WIRE16;
    if (numChannels == 2 and localFormat == SOAPY_SDR_CS16 and reqWireFmt == SOAPY_SDR_CS16) localFormatOut = SF_CS16_x2_WIRE64;
    if (numChannels == 2 and localFormat == SOAPY_SDR_CF32 and reqWireFmt == SOAPY_SDR_CS16) localFormatOut = SF_CF32_x2_WIRE64;
    if (numChannels == 1 and localFormat == SOAPY_SDR_CS16 and reqWireFmt == SOAPY_SDR_CS16) localFormatOut = SF_CS16_x1_WIRE32;

    switch(localFormatOut)
    {
    case SF_UNDEFINED: break;
    case SF_CS8_x1_WIRE16 : remoteFormatOut = SOAPY_SDR_CS8; break;
    case SF_CS16_x1_WIRE16: remoteFormatOut = SOAPY_SDR_CS8; break;
    case SF_CF32_x1_WIRE16: remoteFormatOut = SOAPY_SDR_CS8; break;
    case SF_CS12_x1_WIRE24: remoteFormatOut = SOAPY_SDR_CS12; break;
    case SF_CS16_x1_WIRE24: remoteFormatOut = SOAPY_SDR_CS12; break;
    case SF_CF32_x1_WIRE24: remoteFormatOut = SOAPY_SDR_CS12; break;
    case SF_CS16_x1_WIRE32: remoteFormatOut = SOAPY_SDR_CS16; break;
    case SF_CS12_x2_WIRE48: remoteFormatOut = SOAPY_SDR_CS12; break;
    case SF_CS16_x2_WIRE48: remoteFormatOut = SOAPY_SDR_CS12; break;
    case SF_CF32_x2_WIRE48: remoteFormatOut = SOAPY_SDR_CS12; break;
    case SF_CS16_x2_WIRE64: remoteFormatOut = SOAPY_SDR_CS16; break;
    case SF_CF32_x2_WIRE64: remoteFormatOut = SOAPY_SDR_CS16; break;
    }

    if (remoteFormatOut.empty())
        throw std::runtime_error("Unsupported format and channel combination: " + localFormat);

    if (not reqWireFmt.empty() and reqWireFmt != remoteFormatOut)
        throw std::runtime_error("Unsupported wire format and channel combination: " + reqWireFmt);

    bytesPerElement = numChannels*SoapySDR::formatToSize(remoteFormatOut);
}

/***************************************************
 * 12-bit helpers
 **************************************************/
inline void unpack24(uint8_t *in, int16_t &i, int16_t &q)
{
    uint16_t part0 = uint16_t(in[0]);
    uint16_t part1 = uint16_t(in[1]);
    uint16_t part2 = uint16_t(in[2]);
    i = int16_t((part1 << 12) | (part0 << 4));
    q = int16_t((part2 << 8) | (part1 & 0xf0));
}

inline void pack24(uint16_t i, uint16_t q, uint8_t *out)
{
    out[0] = uint8_t(i >> 4);
    out[1] = uint8_t((q & 0xf0)|(i >> 12));
    out[2] = uint8_t(q >> 8);
}

/***************************************************
 * SF_CF32_xx_WIRE48
 **************************************************/
static void wire48_to_cf32x2(const void *inBuff, void * const *outBuffs, const size_t num)
{
    auto scale = float(1.0)/float(1 << 15);
    auto in = (uint8_t *)inBuff;
    auto out0 = (float *)outBuffs[0];
    auto out1 = (float *)outBuffs[1];
    for (size_t j = 0; j < num; j++)
    {
        int16_t i0, q0, i1, q1;
        unpack24(in+0, i0, q0);
        unpack24(in+3, i1, q1);
        in += 6;
        *(out0++) = float(i0)*scale;
        *(out0++) = float(q0)*scale;
        *(out1++) = float(i1)*scale;
        *(out1++) = float(q1)*scale;
    }
}

static void cf32x2_to_wire48(const void * const *inBuffs, void *outBuff, const size_t num)
{
    auto scale = float(1 << 15);
    auto in0 = (float *)inBuffs[0];
    auto in1 = (float *)inBuffs[1];
    auto out = (uint8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        auto i0 = int16_t(*(in0++)*scale);
        auto q0 = int16_t(*(in0++)*scale);
        auto i1 = int16_t(*(in1++)*scale);
        auto q1 = int16_t(*(in1++)*scale);
        pack24(i0, q0, out+0);
        pack24(i1, q1, out+3);
        out += 6;
    }
}

static void wire48_to_cf32x1(const void *inBuff, void * const *outBuffs, const size_t num)
{
    static constexpr auto scale = float(1.0)/float(1 << 15);
    auto in = (uint8_t *)inBuff;
    auto out0 = (float *)outBuffs[0];
    for (size_t j = 0; j < num; j++)
    {
        int16_t i0, q0;
        unpack24(in+0, i0, q0);
        in += 3;
        *(out0++) = float(i0)*scale;
        *(out0++) = float(q0)*scale;
    }
}

static void cf32x1_to_wire48(const void * const *inBuffs, void *outBuff, const size_t num)
{
    static constexpr auto scale = float(1 << 15);
    auto in0 = (float *)inBuffs[0];
    auto out = (uint8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        auto i0 = int16_t(*(in0++)*scale);
        auto q0 = int16_t(*(in0++)*scale);
        pack24(i0, q0, out+0);
        out += 3;
    }
}

/***************************************************
 * SF_CS16_xx_WIRE48
 **************************************************/
static void wire48_to_cs16x2(const void *inBuff, void * const *outBuffs, const size_t num)
{
    auto in = (uint8_t *)inBuff;
    auto out0 = (int16_t *)outBuffs[0];
    auto out1 = (int16_t *)outBuffs[1];
    for (size_t j = 0; j < num; j++)
    {
        int16_t i0, q0, i1, q1;
        unpack24(in+0, i0, q0);
        unpack24(in+3, i1, q1);
        in += 6;
        *(out0++) = i0;
        *(out0++) = q0;
        *(out1++) = i1;
        *(out1++) = q1;
    }
}

static void cs16x2_to_wire48(const void * const *inBuffs, void *outBuff, const size_t num)
{
    auto in0 = (int16_t *)inBuffs[0];
    auto in1 = (int16_t *)inBuffs[1];
    auto out = (uint8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        auto i0 = *(in0++);
        auto q0 = *(in0++);
        auto i1 = *(in1++);
        auto q1 = *(in1++);
        pack24(i0, q0, out+0);
        pack24(i1, q1, out+3);
        out += 6;
    }
}

static void wire48_to_cs16x1(const void *inBuff, void * const *outBuffs, const size_t num)
{
    auto in = (uint8_t *)inBuff;
    auto out0 = (int16_t *)outBuffs[0];
    for (size_t j = 0; j < num; j++)
    {
        int16_t i0, q0;
        unpack24(in+0, i0, q0);
        in += 3;
        *(out0++) = i0;
        *(out0++) = q0;
    }
}

static void cs16x1_to_wire48(const void * const *inBuffs, void *outBuff, const size_t num)
{
    auto in0 = (int16_t *)inBuffs[0];
    auto out = (uint8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        auto i0 = *(in0++);
        auto q0 = *(in0++);
        pack24(i0, q0, out+0);
        out += 3;
    }
}

/***************************************************
 * SF_CS16_x2_WIRE64
 **************************************************/
static void wire64_to_cs16x2(const void *inBuff, void * const *outBuffs, const size_t num)
{
    auto in = (uint32_t *)inBuff;
    auto out0 = (uint32_t *)outBuffs[0];
    auto out1 = (uint32_t *)outBuffs[1];
    for (size_t j = 0; j < num; j++)
    {
        *(out0++) = *(in++);
        *(out1++) = *(in++);
    }
}

static void cs16x2_to_wire64(const void * const *inBuffs, void *outBuff, const size_t num)
{
    auto in0 = (uint32_t *)inBuffs[0];
    auto in1 = (uint32_t *)inBuffs[1];
    auto out = (uint32_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        *(out++) = *(in0++);
        *(out++) = *(in1++);
    }
}

/***************************************************
 * SF_CF32_x2_WIRE64
 **************************************************/
static void wire64_to_cf32x2(const void *inBuff, void * const *outBuffs, const size_t num)
{
    static constexpr auto scale = float(1.0)/float(1 << 15);
    auto in = (int16_t *)inBuff;
    auto out0 = (float *)outBuffs[0];
    auto out1 = (float *)outBuffs[1];
    for (size_t j = 0; j < num; j++)
    {
        *(out0++) = float(*(in++))*scale;
        *(out0++) = float(*(in++))*scale;
        *(out1++) = float(*(in++))*scale;
        *(out1++) = float(*(in++))*scale;
    }
}

static void cf32x2_to_wire64(const void * const *inBuffs, void *outBuff, const size_t num)
{
    static constexpr auto scale = float(1 << 15);
    auto in0 = (float *)inBuffs[0];
    auto in1 = (float *)inBuffs[1];
    auto out = (uint16_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        *(out++) = int16_t(*(in0++)*scale);
        *(out++) = int16_t(*(in0++)*scale);
        *(out++) = int16_t(*(in1++)*scale);
        *(out++) = int16_t(*(in1++)*scale);
    }
}

/***************************************************
 * SF_CS12_x2_WIRE48
 **************************************************/
static void wire48_to_cs12x2(const void *inBuff, void * const *outBuffs, const size_t num)
{
    auto in = (uint8_t *)inBuff;
    auto out0 = (uint8_t *)outBuffs[0];
    auto out1 = (uint8_t *)outBuffs[1];
    for (size_t j = 0; j < num; j++)
    {
        *(out0++) = *(in++);
        *(out0++) = *(in++);
        *(out0++) = *(in++);
        *(out1++) = *(in++);
        *(out1++) = *(in++);
        *(out1++) = *(in++);
    }
}

static void cs12x2_to_wire48(const void * const *inBuffs, void *outBuff, const size_t num)
{
    auto in0 = (uint8_t *)inBuffs[0];
    auto in1 = (uint8_t *)inBuffs[1];
    auto out = (uint8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        *(out++) = *(in0++);
        *(out++) = *(in0++);
        *(out++) = *(in0++);
        *(out++) = *(in1++);
        *(out++) = *(in1++);
        *(out++) = *(in1++);
    }
}

/***************************************************
 * SF_CS16_x1_WIRE16
 **************************************************/
static void wire16_to_cs16x1(const void *inBuff, void * const *outBuffs, const size_t num)
{
    auto in = (int8_t *)inBuff;
    auto out0 = (int16_t *)outBuffs[0];
    for (size_t j = 0; j < num; j++)
    {
        *(out0++) = int16_t(*(in++));
        *(out0++) = int16_t(*(in++));
    }
}

static void cs16x1_to_wire16(const void * const *inBuffs, void *outBuff, const size_t num)
{
    auto in0 = (int16_t *)inBuffs[0];
    auto out = (int8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        *(out++) = int8_t(*(in0++));
        *(out++) = int8_t(*(in0++));
    }
}

/***************************************************
 * SF_CF32_x1_WIRE16
 **************************************************/
static void wire16_to_cf32x1(const void *inBuff, void * const *outBuffs, const size_t num)
{
    static constexpr auto scale = float(1.0)/float(1 << 7);
    auto in = (int8_t *)inBuff;
    auto out0 = (float *)outBuffs[0];
    for (size_t j = 0; j < num; j++)
    {
        *(out0++) = float(*(in++))*scale;
        *(out0++) = float(*(in++))*scale;
    }
}

static void cf32x1_to_wire16(const void * const *inBuffs, void *outBuff, const size_t num)
{
    static constexpr auto scale = float(1 << 7);
    auto in0 = (float *)inBuffs[0];
    auto out = (int8_t *)outBuff;
    for (size_t j = 0; j < num; j++)
    {
        *(out++) = int8_t(*(in0++)*scale);
        *(out++) = int8_t(*(in0++)*scale);
    }
}

/***************************************************
 * Conversion dispatchers
 **************************************************/
void convertToHost(
    const StreamFormat format,
    const void *inBuff,
    void * const *outBuffs,
    const size_t numSamples)
{
    switch(format)
    {
    case SF_UNDEFINED: break;
    case SF_CS8_x1_WIRE16 : std::memcpy(outBuffs[0], inBuff, numSamples*2); return;
    case SF_CS16_x1_WIRE16: return wire16_to_cs16x1(inBuff, outBuffs, numSamples);
    case SF_CF32_x1_WIRE16: return wire16_to_cf32x1(inBuff, outBuffs, numSamples);
    case SF_CS12_x1_WIRE24: std::memcpy(outBuffs[0], inBuff, numSamples*3); return;
    case SF_CS16_x1_WIRE24: return wire48_to_cs16x1(inBuff, outBuffs, numSamples);
    case SF_CF32_x1_WIRE24: return wire48_to_cf32x1(inBuff, outBuffs, numSamples);
    case SF_CS16_x1_WIRE32: std::memcpy(outBuffs[0], inBuff, numSamples*4); return;
    case SF_CS12_x2_WIRE48: return wire48_to_cs12x2(inBuff, outBuffs, numSamples);
    case SF_CS16_x2_WIRE48: return wire48_to_cs16x2(inBuff, outBuffs, numSamples);
    case SF_CF32_x2_WIRE48: return wire48_to_cf32x2(inBuff, outBuffs, numSamples);
    case SF_CS16_x2_WIRE64: return wire64_to_cs16x2(inBuff, outBuffs, numSamples);
    case SF_CF32_x2_WIRE64: return wire64_to_cf32x2(inBuff, outBuffs, numSamples);
    }
}

void convertToWire(
    const StreamFormat format,
    const void * const *inBuffs,
    void *outBuff,
    const size_t numSamples)
{
    switch(format)
    {
    case SF_UNDEFINED: break;
    case SF_CS8_x1_WIRE16 : std::memcpy(outBuff, inBuffs[0], numSamples*2); return;
    case SF_CS16_x1_WIRE16: return cs16x1_to_wire16(inBuffs, outBuff, numSamples);
    case SF_CF32_x1_WIRE16: return cf32x1_to_wire16(inBuffs, outBuff, numSamples);
    case SF_CS12_x1_WIRE24: std::memcpy(outBuff, inBuffs[0], numSamples*3); return;
    case SF_CS16_x1_WIRE24: return cs16x1_to_wire48(inBuffs, outBuff, numSamples);
    case SF_CF32_x1_WIRE24: return cf32x1_to_wire48(inBuffs, outBuff, numSamples);
    case SF_CS16_x1_WIRE32: std::memcpy(outBuff, inBuffs[0], numSamples*4); return;
    case SF_CS12_x2_WIRE48: return cs12x2_to_wire48(inBuffs, outBuff, numSamples);
    case SF_CS16_x2_WIRE48: return cs16x2_to_wire48(inBuffs, outBuff, numSamples);
    case SF_CF32_x2_WIRE48: return cf32x2_to_wire48(inBuffs, outBuff, numSamples);
    case SF_CS16_x2_WIRE64: return cs16x2_to_wire64(inBuffs, outBuff, numSamples);
    case SF_CF32_x2_WIRE64: return cf32x2_to_wire64(inBuffs, outBuff, numSamples);
    }
}
