// Copyright (c) 2017-2018 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//-------------------------------------------------------------
//-- Device constructor and settings
//-------------------------------------------------------------

#include "iris_device.hpp"
#include <SoapySDR/Logger.hpp>
#include <regex>
#include <iostream>

struct VersionParser
{
    VersionParser(const std::string &s):
        version(s.empty()?"<unknown>":s)
    {
        //regex is broken in gcc < 4.9, ignore the version parsing
        #if defined(__GNUC__) && __GNUC__ <= 4 && __GNUC_MINOR__ < 9
        return;
        #endif
        std::smatch matches;
        static const std::regex e("^(\\d+\\.\\d+)\\.(\\d+)\\.(\\d+)-(.*)$");
        try
        {
            if (not std::regex_match(s, matches, e)) return;
            major = matches[1];
            minor = matches[2];
            patch = matches[3];
            extra = matches[4];
            dirty = s.find("-dirty") != std::string::npos;
            series = major + "." + minor;
        }
        catch(...)
        {
            SoapySDR::logf(SOAPY_SDR_WARNING, "VersionParser failed to parse: %s", s.c_str());
        }
    }

    std::string version, major, minor, patch, extra;
    std::string series; //compatibility indicator
    bool dirty;
};

/*******************************************************************
 * Constructor
 ******************************************************************/

SoapyIrisLocal::SoapyIrisLocal(const SoapySDR::Kwargs &args):
    _remoteURL(args.at("remote")),
    _remote(SoapySDR::Device::make(args)),
    _adcClockRate(0.0),
    _dacClockRate(0.0)
{
    auto hwInfo = this->getHardwareInfo();
    auto driverVer = VersionParser(DRIVER_VERSION);
    auto fwVer = VersionParser(hwInfo["firmware"]);
    auto fpgaVer = VersionParser(hwInfo["fpga"]);

    //check the driver version for series match, and dirty bit
    if (driverVer.series != fwVer.series)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR,
            "Firmware version mismatch! Expected %s but found firmware with version %s",
            driverVer.series.c_str(), fwVer.version.c_str());
    }
    else if (fwVer.dirty)
    {
        SoapySDR::logf(SOAPY_SDR_WARNING,
            "Development image: firmware has dirty bit set!");
    }

    //check the fpga version for series match, and dirty bit
    if (driverVer.series != fpgaVer.series)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR,
            "FPGA version mismatch! Expected %s but found FPGA with version %s",
            driverVer.series.c_str(), fpgaVer.version.c_str());
    }
    else if (fpgaVer.dirty)
    {
        SoapySDR::logf(SOAPY_SDR_WARNING,
            "Development image: FPGA image has dirty bit set!");
    }
}

SoapyIrisLocal::~SoapyIrisLocal(void)
{
    if (_remote != nullptr) SoapySDR::Device::unmake(_remote);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyIrisLocal::getDriverKey(void) const
{
    return _remote->getDriverKey();
}

std::string SoapyIrisLocal::getHardwareKey(void) const
{
    return _remote->getHardwareKey();
}

SoapySDR::Kwargs SoapyIrisLocal::getHardwareInfo(void) const
{
    auto info = _remote->getHardwareInfo();
    info["driver"] = DRIVER_VERSION;
    return info;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

void SoapyIrisLocal::setFrontendMapping(const int direction, const std::string &mapping)
{
    return _remote->setFrontendMapping(direction, mapping);
}

std::string SoapyIrisLocal::getFrontendMapping(const int direction) const
{
    return _remote->getFrontendMapping(direction);
}

size_t SoapyIrisLocal::getNumChannels(const int direction) const
{
    return _remote->getNumChannels(direction);
}

SoapySDR::Kwargs SoapyIrisLocal::getChannelInfo(const int direction, const size_t channel) const
{
    return _remote->getChannelInfo(direction, channel);
}

bool SoapyIrisLocal::getFullDuplex(const int direction, const size_t channel) const
{
    return _remote->getFullDuplex(direction, channel);
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listAntennas(const int direction, const size_t channel) const
{
    return _remote->listAntennas(direction, channel);
}

void SoapyIrisLocal::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    return _remote->setAntenna(direction, channel, name);
}

std::string SoapyIrisLocal::getAntenna(const int direction, const size_t channel) const
{
    return _remote->getAntenna(direction, channel);
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyIrisLocal::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return _remote->hasDCOffsetMode(direction, channel);
}

void SoapyIrisLocal::setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
{
    return _remote->setDCOffsetMode(direction, channel, automatic);
}

bool SoapyIrisLocal::getDCOffsetMode(const int direction, const size_t channel) const
{
    return _remote->getDCOffsetMode(direction, channel);
}

bool SoapyIrisLocal::hasDCOffset(const int direction, const size_t channel) const
{
    return _remote->hasDCOffset(direction, channel);
}

void SoapyIrisLocal::setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset)
{
    return _remote->setDCOffset(direction, channel, offset);
}

std::complex<double> SoapyIrisLocal::getDCOffset(const int direction, const size_t channel) const
{
    return _remote->getDCOffset(direction, channel);
}

bool SoapyIrisLocal::hasIQBalance(const int direction, const size_t channel) const
{
    return _remote->hasIQBalance(direction, channel);
}

void SoapyIrisLocal::setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance)
{
    return _remote->setIQBalance(direction, channel, balance);
}

std::complex<double> SoapyIrisLocal::getIQBalance(const int direction, const size_t channel) const
{
    return _remote->getIQBalance(direction, channel);
}

bool SoapyIrisLocal::hasFrequencyCorrection(const int direction, const size_t channel) const
{
    return _remote->hasFrequencyCorrection(direction, channel);
}

void SoapyIrisLocal::setFrequencyCorrection(const int direction, const size_t channel, const double value)
{
    return _remote->setFrequencyCorrection(direction, channel, value);
}

double SoapyIrisLocal::getFrequencyCorrection(const int direction, const size_t channel) const
{
    return _remote->getFrequencyCorrection(direction, channel);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listGains(const int direction, const size_t channel) const
{
    return _remote->listGains(direction, channel);
}

bool SoapyIrisLocal::hasGainMode(const int direction, const size_t channel) const
{
    return _remote->hasGainMode(direction, channel);
}

void SoapyIrisLocal::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    return _remote->setGainMode(direction, channel, automatic);
}

bool SoapyIrisLocal::getGainMode(const int direction, const size_t channel) const
{
    return _remote->getGainMode(direction, channel);
}

void SoapyIrisLocal::setGain(const int direction, const size_t channel, const double value)
{
    return _remote->setGain(direction, channel, value);
}

void SoapyIrisLocal::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    return _remote->setGain(direction, channel, name, value);
}

double SoapyIrisLocal::getGain(const int direction, const size_t channel) const
{
    return _remote->getGain(direction, channel);
}

double SoapyIrisLocal::getGain(const int direction, const size_t channel, const std::string &name) const
{
    return _remote->getGain(direction, channel, name);
}

SoapySDR::Range SoapyIrisLocal::getGainRange(const int direction, const size_t channel) const
{
    return _remote->getGainRange(direction, channel);
}

SoapySDR::Range SoapyIrisLocal::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    return _remote->getGainRange(direction, channel, name);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyIrisLocal::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args)
{
    return _remote->setFrequency(direction, channel, frequency, args);
}

void SoapyIrisLocal::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args)
{
    return _remote->setFrequency(direction, channel, name, frequency, args);
}

double SoapyIrisLocal::getFrequency(const int direction, const size_t channel) const
{
    return _remote->getFrequency(direction, channel);
}

double SoapyIrisLocal::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    return _remote->getFrequency(direction, channel, name);
}

std::vector<std::string> SoapyIrisLocal::listFrequencies(const int direction, const size_t channel) const
{
    return _remote->listFrequencies(direction, channel);
}

SoapySDR::RangeList SoapyIrisLocal::getFrequencyRange(const int direction, const size_t channel) const
{
    return _remote->getFrequencyRange(direction, channel);
}

SoapySDR::RangeList SoapyIrisLocal::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const
{
    return _remote->getFrequencyRange(direction, channel, name);
}

SoapySDR::ArgInfoList SoapyIrisLocal::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    return _remote->getFrequencyArgsInfo(direction, channel);
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyIrisLocal::setSampleRate(const int direction, const size_t channel, const double rate)
{
    _remote->setSampleRate(direction, channel, rate);
    if (direction == SOAPY_SDR_RX) _adcClockRate = rate*2;
    if (direction == SOAPY_SDR_TX) _dacClockRate = rate*2;
}

double SoapyIrisLocal::getSampleRate(const int direction, const size_t channel) const
{
    return _remote->getSampleRate(direction, channel);
}

std::vector<double> SoapyIrisLocal::listSampleRates(const int direction, const size_t channel) const
{
    return _remote->listSampleRates(direction, channel);
}

SoapySDR::RangeList SoapyIrisLocal::getSampleRateRange(const int direction, const size_t channel) const
{
    return _remote->getSampleRateRange(direction, channel);
}

/*******************************************************************
 * Bandwidth API
 ******************************************************************/

void SoapyIrisLocal::setBandwidth(const int direction, const size_t channel, const double bw)
{
    return _remote->setBandwidth(direction, channel, bw);
}

double SoapyIrisLocal::getBandwidth(const int direction, const size_t channel) const
{
    return _remote->getBandwidth(direction, channel);
}

std::vector<double> SoapyIrisLocal::listBandwidths(const int direction, const size_t channel) const
{
    return _remote->listBandwidths(direction, channel);
}

SoapySDR::RangeList SoapyIrisLocal::getBandwidthRange(const int direction, const size_t channel) const
{
    return _remote->getBandwidthRange(direction, channel);
}

/*******************************************************************
 * Clocking API
 ******************************************************************/

void SoapyIrisLocal::setMasterClockRate(const double rate)
{
    return _remote->setMasterClockRate(rate);
}

double SoapyIrisLocal::getMasterClockRate(void) const
{
    return _remote->getMasterClockRate();
}

SoapySDR::RangeList SoapyIrisLocal::getMasterClockRates(void) const
{
    return _remote->getMasterClockRates();
}

std::vector<std::string> SoapyIrisLocal::listClockSources(void) const
{
    return _remote->listClockSources();
}

void SoapyIrisLocal::setClockSource(const std::string &source)
{
    return _remote->setClockSource(source);
}

std::string SoapyIrisLocal::getClockSource(void) const
{
    return _remote->getClockSource();
}

/*******************************************************************
 * Time API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listTimeSources(void) const
{
    return _remote->listTimeSources();
}

void SoapyIrisLocal::setTimeSource(const std::string &source)
{
    return _remote->setTimeSource(source);
}

std::string SoapyIrisLocal::getTimeSource(void) const
{
    return _remote->getTimeSource();
}

bool SoapyIrisLocal::hasHardwareTime(const std::string &what) const
{
    return _remote->hasHardwareTime(what);
}

long long SoapyIrisLocal::getHardwareTime(const std::string &what) const
{
    return _remote->getHardwareTime(what);
}

void SoapyIrisLocal::setHardwareTime(const long long timeNs, const std::string &what)
{
    return _remote->setHardwareTime(timeNs, what);
}

void SoapyIrisLocal::setCommandTime(const long long timeNs, const std::string &what)
{
    return _remote->setCommandTime(timeNs, what);
}

/*******************************************************************
 * Sensor API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listSensors(void) const
{
    return _remote->listSensors();
}

SoapySDR::ArgInfo SoapyIrisLocal::getSensorInfo(const std::string &name) const
{
    return _remote->getSensorInfo(name);
}

std::string SoapyIrisLocal::readSensor(const std::string &name) const
{
    return _remote->readSensor(name);
}

std::vector<std::string> SoapyIrisLocal::listSensors(const int direction, const size_t channel) const
{
    return _remote->listSensors(direction, channel);
}

SoapySDR::ArgInfo SoapyIrisLocal::getSensorInfo(const int direction, const size_t channel, const std::string &name) const
{
    return _remote->getSensorInfo(direction, channel, name);
}

std::string SoapyIrisLocal::readSensor(const int direction, const size_t channel, const std::string &name) const
{
    return _remote->readSensor(direction, channel, name);
}

/*******************************************************************
 * Register API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listRegisterInterfaces(void) const
{
    return _remote->listRegisterInterfaces();
}

void SoapyIrisLocal::writeRegister(const std::string &name, const unsigned addr, const unsigned value)
{
    return _remote->writeRegister(name, addr, value);
}

unsigned SoapyIrisLocal::readRegister(const std::string &name, const unsigned addr) const
{
    return _remote->readRegister(name, addr);
}

void SoapyIrisLocal::writeRegister(const unsigned addr, const unsigned value)
{
    return _remote->writeRegister(addr, value);
}

unsigned SoapyIrisLocal::readRegister(const unsigned addr) const
{
    return _remote->readRegister(addr);
}

void SoapyIrisLocal::writeRegisters(const std::string &name, const unsigned addr, const std::vector<unsigned> &value)
{
    return _remote->writeRegisters(name, addr, value);
}

std::vector<unsigned> SoapyIrisLocal::readRegisters(const std::string &name, const unsigned addr, const size_t length) const
{
    return _remote->readRegisters(name, addr, length);
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyIrisLocal::getSettingInfo(void) const
{
    return _remote->getSettingInfo();
}

void SoapyIrisLocal::writeSetting(const std::string &key, const std::string &value)
{
    return _remote->writeSetting(key, value);
}

std::string SoapyIrisLocal::readSetting(const std::string &key) const
{
    return _remote->readSetting(key);
}

SoapySDR::ArgInfoList SoapyIrisLocal::getSettingInfo(const int direction, const size_t channel) const
{
    return _remote->getSettingInfo(direction, channel);
}

void SoapyIrisLocal::writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value)
{
    return _remote->writeSetting(direction, channel, key, value);
}

std::string SoapyIrisLocal::readSetting(const int direction, const size_t channel, const std::string &key) const
{
    return _remote->readSetting(direction, channel, key);
}

/*******************************************************************
 * GPIO API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listGPIOBanks(void) const
{
    return _remote->listGPIOBanks();
}

void SoapyIrisLocal::writeGPIO(const std::string &bank, const unsigned value)
{
    return _remote->writeGPIO(bank, value);
}

void SoapyIrisLocal::writeGPIO(const std::string &bank, const unsigned value, const unsigned mask)
{
    return _remote->writeGPIO(bank, value, mask);
}

unsigned SoapyIrisLocal::readGPIO(const std::string &bank) const
{
    return _remote->readGPIO(bank);
}

void SoapyIrisLocal::writeGPIODir(const std::string &bank, const unsigned dir)
{
    return _remote->writeGPIODir(bank, dir);
}

void SoapyIrisLocal::writeGPIODir(const std::string &bank, const unsigned dir, const unsigned mask)
{
    return _remote->writeGPIODir(bank, dir, mask);
}

unsigned SoapyIrisLocal::readGPIODir(const std::string &bank) const
{
    return _remote->readGPIODir(bank);
}

/*******************************************************************
 * I2C API
 ******************************************************************/

void SoapyIrisLocal::writeI2C(const int addr, const std::string &data)
{
    return _remote->writeI2C(addr, data);
}

std::string SoapyIrisLocal::readI2C(const int addr, const size_t numBytes)
{
    return _remote->readI2C(addr, numBytes);
}

/*******************************************************************
 * SPI API
 ******************************************************************/

unsigned SoapyIrisLocal::transactSPI(const int addr, const unsigned data, const size_t numBits)
{
    return _remote->transactSPI(addr, data, numBits);
}

/*******************************************************************
 * UART API
 ******************************************************************/

std::vector<std::string> SoapyIrisLocal::listUARTs(void) const
{
    return _remote->listUARTs();
}

void SoapyIrisLocal::writeUART(const std::string &which, const std::string &data)
{
    return _remote->writeUART(which, data);
}

std::string SoapyIrisLocal::readUART(const std::string &which, const long timeoutUs) const
{
    return _remote->readUART(which, timeoutUs);
}
