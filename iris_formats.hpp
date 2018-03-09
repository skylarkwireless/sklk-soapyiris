// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

#pragma once
#include <cstddef>
#include <string>

enum StreamFormat
{
    SF_UNDEFINED,
    SF_CS8_x1_WIRE16,
    SF_CS16_x1_WIRE16,
    SF_CF32_x1_WIRE16,
    SF_CS12_x1_WIRE24,
    SF_CS16_x1_WIRE24,
    SF_CF32_x1_WIRE24,
    SF_CS16_x1_WIRE32,
    SF_CS12_x2_WIRE48,
    SF_CS16_x2_WIRE48,
    SF_CF32_x2_WIRE48,
    SF_CS16_x2_WIRE64,
    SF_CF32_x2_WIRE64,
};

void resolveFormats(
    const size_t numChannels,
    const std::string &localFormat,
    const std::string &requestedWireFormat,
    StreamFormat &localFormatOut,
    std::string &remoteFormatOut,
    size_t &bytesPerElement);

void convertToHost(
    const StreamFormat format,
    const void *inBuff,
    void * const *outBuffs,
    const size_t numSamples);

void convertToWire(
    const StreamFormat format,
    const void * const *inBuffs,
    void *outBuff,
    const size_t numSamples);
