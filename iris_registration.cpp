// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//-------------------------------------------------------------
//-- Device discovery and factory
//-------------------------------------------------------------

#include "iris_device.hpp"
#include <SoapySDR/Registry.hpp>

static SoapySDR::Kwargs modifyArgs(SoapySDR::Kwargs args)
{
    //using soapyremote for settings, set typical driver filters for iris here
    args["show"] = "1"; //needed to enable discovery on iris-arm remote module
    args["driver"] = "remote";
    args["remote:driver"] = "iris-arm";
    return args;
}

/***********************************************************************
 * Find available devices
 **********************************************************************/
static std::vector<SoapySDR::Kwargs> findIrisLocal(const SoapySDR::Kwargs &hint)
{
    return SoapySDR::Device::enumerate(modifyArgs(hint));
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *makeIrisLocal(const SoapySDR::Kwargs &args)
{
    if (args.count("remote") == 0) throw std::runtime_error("No Iris found on local network!");
    return new SoapyIrisLocal(modifyArgs(args));
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerIrisLocal("iris", &findIrisLocal, &makeIrisLocal, SOAPY_SDR_ABI_VERSION);
