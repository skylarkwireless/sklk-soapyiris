// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

#include <iostream>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Time.hpp>
#include <cstdlib>
#include <cstddef>
#include <chrono>
#include <string>
#include <cstdint>
#include <complex>
#include <csignal>

static sig_atomic_t loopDone = false;
void sigIntHandler(const int)
{
    loopDone = true;
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " [argsString] [rate] [numCh]" << std::endl;
        return EXIT_FAILURE;
    }

    std::string argStr(argv[1]);
    double rate = std::stod(argv[2]);
    size_t numCh = std::stoul(argv[3]);

    //load channels
    std::vector<size_t> channels;
    if (numCh == 1) channels = {0};
    else if (numCh == 2) channels = {0, 1};
    else
    {
        std::cerr << "Error! Supported number of channels 1 or 2" << std::endl;
        return EXIT_FAILURE;
    }

    auto device = SoapySDR::Device::make(argStr);
    if (device == nullptr)
    {
        std::cerr << "No device!" << std::endl;
        return EXIT_FAILURE;
    }

    //use the RX only antenna, and TRX for tx
    for (auto ch : channels) device->setAntenna(SOAPY_SDR_RX, ch, "RX");

    std::cout << "setting samples rates to " << rate/1e6 << " Msps..." << std::endl;
    for (auto ch : channels)
    {
        device->setSampleRate(SOAPY_SDR_RX, ch, rate);
        device->setSampleRate(SOAPY_SDR_TX, ch, rate);
    }

    std::cout << "create streams..." << std::endl;
    auto rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels);
    auto txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels);

    const size_t NUM_SAMPS = size_t(rate/1e3); //1ms
    std::cout << "Loops will operate on chunks of " << NUM_SAMPS << " samples" << std::endl;

    device->setHardwareTime(0); //clear HW time for easy debugging

    const long long txTimeDelta = 4*1e6; //4 milliseconds (in units of nanoseconds)
    std::cout << "Tx time delta " << SoapySDR::timeNsToTicks(txTimeDelta, rate) << " ticks" << std::endl;

    device->activateStream(rxStream);
    device->activateStream(txStream);

    long long numOverflows(0);
    long long numUnderflows(0);
    long long numTimeErrors(0);
    long long totalRxSamples(0);
    long long totalTxSamples(0);
    long long numIterations(0);

    std::vector<std::complex<int16_t>> buff(NUM_SAMPS);
    std::vector<std::complex<int16_t>> buff2(NUM_SAMPS);
    std::vector<void *> buffs(2);
    std::cout << "Press Ctrl+C to end loop" << std::flush;
    signal(SIGINT, sigIntHandler);
    bool exitLoop = false;
    while (not exitLoop)
    {
        exitLoop = true;

        ///////////////////////////////////////////////////////////
        // receiver loop
        ///////////////////////////////////////////////////////////
        int flags(0);
        long long timeNs(0);
        long long txTimeNs(0);
        buffs[0] = buff.data();
        buffs[1] = buff.data();
        int r = device->readStream(rxStream, buffs.data(), NUM_SAMPS, flags, timeNs);
        if (r == SOAPY_SDR_OVERFLOW or (r > 0 and (flags & SOAPY_SDR_END_ABRUPT) != 0))
        {
            numOverflows++;
            continue; //start this over
        }
        else if (r < 0)
        {
            std::cerr << "unexpected readStream error " << SoapySDR::errToStr(r) << std::endl;
            goto cleanup;
        }
        else if (size_t(r) != NUM_SAMPS)
        {
            std::cerr << "unexpected readStream return r != NUM_SAMPS " << r << std::endl;
            goto cleanup;
        }
        else
        {
            txTimeNs = timeNs + txTimeDelta; //first time used for tx
            totalRxSamples += r;
        }

        ///////////////////////////////////////////////////////////
        // transmit loop
        ///////////////////////////////////////////////////////////
        //std::cout << "tx at " << SoapySDR::timeNsToTicks(txTimeNs, rate) << std::endl;
        flags = SOAPY_SDR_HAS_TIME;
        if (exitLoop) flags |= SOAPY_SDR_END_BURST; //end burst on last 
        unsigned cnt = 0;
        for (size_t i =0;i < buff.size();i++)
        {
            buff[i] = std::complex<int16_t>((cnt+0)<<4, (cnt+1)<<4);
            buff2[i] = std::complex<int16_t>((cnt+2)<<4, (cnt+3)<<4);
            cnt += 4;
        }
        buffs[0] = buff.data();
        buffs[1] = buff2.data();
        r = device->writeStream(txStream, buffs.data(), NUM_SAMPS, flags, txTimeNs);
        if (r < 0)
        {
            std::cerr << "unexpected writeStream error " << SoapySDR::errToStr(r) << std::endl;
            goto cleanup;
        }
        else if (size_t(r) != NUM_SAMPS)
        {
            std::cerr << "unexpected writeStream return r != NUM_SAMPS " << r << std::endl;
            goto cleanup;
        }
        else
        {
            flags = 0;
            totalTxSamples += r;
        }

        ///////////////////////////////////////////////////////////
        // tx status
        ///////////////////////////////////////////////////////////
        while (true)
        {
            size_t chanMask;
            long timeoutUs(0); //non blocking unless last iteration
            if (exitLoop) timeoutUs = long(1e5);
            int r = device->readStreamStatus(txStream, chanMask, flags, timeNs, timeoutUs);
            if (r == SOAPY_SDR_TIMEOUT) break;
            else if (r == 0) {} //just flags
            else if (r == SOAPY_SDR_UNDERFLOW) numUnderflows++;
            else if (r == SOAPY_SDR_TIME_ERROR) numTimeErrors++;
            else if (r == SOAPY_SDR_CORRUPTION) {} //TODO count it
            else
            {
                std::cerr << "unexpected readStreamStatus error " << SoapySDR::errToStr(r) << std::endl;
                goto cleanup;
            }

            //probably not keeping up, flush rx
            if (r == SOAPY_SDR_TIME_ERROR)
            {
                buffs[0] = buff.data();
                buffs[1] = buff.data();
                while (device->readStream(rxStream, buffs.data(), NUM_SAMPS, flags, timeNs, 0) != SOAPY_SDR_TIMEOUT);
            }
        }
        if ((numIterations++%100) == 0) std::cerr << '.' << std::flush;
    }

    cleanup:

    device->deactivateStream(rxStream);
    device->deactivateStream(txStream);

    std::cout << "\nsummary..." << std::endl;
    if (numOverflows != 0) std::cout << "numOverflows\t" << numOverflows << std::endl;
    if (numUnderflows != 0) std::cout << "numUnderflows\t" << numUnderflows << std::endl;
    if (numTimeErrors != 0) std::cout << "numTimeErrors\t" << numTimeErrors << std::endl;
    if (totalRxSamples != 0) std::cout << "totalRxSamples\t" << totalRxSamples << std::endl;
    if (totalTxSamples != 0) std::cout << "totalTxSamples\t" << totalTxSamples << std::endl;
    if (numIterations != 0) std::cout << "numIterations\t" << numIterations << std::endl;
    std::cout << std::endl;

    std::cout << "cleanup..." << std::endl;
    device->closeStream(rxStream);
    device->closeStream(txStream);
    SoapySDR::Device::unmake(device);

    return EXIT_SUCCESS;
}
