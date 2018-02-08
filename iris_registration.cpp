// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//-------------------------------------------------------------
//-- Device discovery and factory
//-------------------------------------------------------------

#include "iris_device.hpp"
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

static SoapySDR::Kwargs modifyArgs(SoapySDR::Kwargs args, const long timeoutUs = -1)
{
    //using soapyremote for settings, set typical driver filters for iris here
    args["show"] = "1"; //needed to enable discovery on iris-arm remote module
    args["driver"] = "remote";
    args["remote:driver"] = "iris-arm";
    if (timeoutUs != -1) args["remote:timeout"] = std::to_string(timeoutUs);
    return args;
}

static bool hasRemoteSupportInstalled(void)
{
    const auto factories = SoapySDR::Registry::listFindFunctions();
    return factories.find("remote") != factories.end();
}

/***********************************************************************
 * Find available devices
 **********************************************************************/
static std::vector<SoapySDR::Kwargs> findIrisLocal(const SoapySDR::Kwargs &hint)
{
    auto result = SoapySDR::Device::enumerate(modifyArgs(hint));
    if (result.empty() and hint.count("driver") and hint.at("driver") == "iris" and not hasRemoteSupportInstalled())
    {
        SoapySDR::log(SOAPY_SDR_ERROR, "Missing SoapyRemote support module required by SoapyIris!");
    }
    return result;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *makeIrisLocal(const SoapySDR::Kwargs &args)
{
    if (args.count("remote") == 0)
    {
        if (not hasRemoteSupportInstalled())
            throw std::runtime_error("Missing SoapyRemote support module required by SoapyIris!");
        else
            throw std::runtime_error("No Iris found on local network!");
    }
    return new SoapyIrisLocal(modifyArgs(args));
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerIrisLocal("iris", &findIrisLocal, &makeIrisLocal, SOAPY_SDR_ABI_VERSION);
