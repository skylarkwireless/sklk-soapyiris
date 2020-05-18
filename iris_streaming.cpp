// Copyright (c) 2017-2019 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//-------------------------------------------------------------
//-- Streaming over IP/UDP implementation
//-------------------------------------------------------------

#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include "iris_device.hpp"
#include "SoapySocketDefs.hpp"
#include <SoapyRPCSocket.hpp>
#include <SoapyURLUtils.hpp>
#include <ThreadPrioHelper.hpp>
#include "iris_formats.hpp"
#include <iostream>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <deque>
#include <future>
#include <algorithm> //find
#include <condition_variable>

#define MAX_TX_STATUS_DEPTH 64

#define RX_SOCKET_BUFFER_BYTES 50*1024*1024 //arbitrary and large PC buffer size

#define ETHERNET_MTU 1500 //L2 MTU without the 14-byte eth header
#define ROUTE_HDR_SIZE 16 //128-bit transfer for routing header
#define PADDED_ETH_HDR_SIZE 16 //14 bytes + 2 bytes padding (holds size in bytes)
#define IPv6_UDP_SIZE (40 + 8) //40 bytes of IPv6 + 8 bytes of UDP header
#define TWBW_HDR_SIZE (sizeof(uint64_t)*4) //4 transfers at 64-bits width

void sockAddrInterfaceLookup(const sockaddr *sa, std::string &ethName, unsigned long long &mac64, int &scopeId);

/*******************************************************************
 * Thread prio is a good idea with sockets
 ******************************************************************/
struct ThreadPrioInit
{
    ThreadPrioInit(const double prio, const char *what)
    {
        auto result = setThreadPrio(prio);
        if (result.empty()) return;
        SoapySDR::logf(SOAPY_SDR_WARNING, "Could not set thread priority %.1f in %s: %s", prio, what, result.c_str());
    }
};

#define THREAD_PRIO(prio) static thread_local ThreadPrioInit __prio(prio, __FUNCTION__)

/*******************************************************************
 * Stream data
 ******************************************************************/
struct StreamStatusEntry
{
    StreamStatusEntry(const int ret = 0):
        ret(ret), flags(0), timeTicks(0){}
    int ret;
    int flags;
    long long timeTicks;
};

struct IrisLocalStream
{
    SoapySDR::Stream *remoteStream;
    sklk_SoapyRPCSocket sock;
    int direction;
    unsigned routeEndpoints;
    StreamFormat format; //!< requested stream format
    uint64_t buff[1024];

    size_t bytesPerElement;
    size_t numHostChannels;
    size_t hostFormatSize;
    size_t mtuElements;

    //read channel partial
    size_t readHandle;
    size_t readElemsLeft;
    size_t readOffset;

    //burst tracking
    long long tickCount;
    bool inBurst;
    bool burstUsesTime;
    long long packetCount;

    //tx sequence tracking
    std::atomic<uint16_t> nextSeqSend;
    std::atomic<uint16_t> lastSeqRecv;
    uint16_t windowSize;
    std::mutex mutex;
    std::condition_variable cond;
    std::thread thread;
    std::atomic<bool> running;
    std::deque<StreamStatusEntry> queue;
    void statusLoop(void);

    //async activate support
    std::shared_future<int> async;
    bool syncActivate;
};

/*******************************************************************
 * Stream config
 ******************************************************************/
std::vector<std::string> SoapyIrisLocal::getStreamFormats(const int /*direction*/, const size_t /*channel*/) const
{
    //formats supported by local read/write stream conversions
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
}

std::string SoapyIrisLocal::getNativeStreamFormat(const int /*direction*/, const size_t /*channel*/, double &fullScale) const
{
    fullScale = (1 << 11)-1;
    return SOAPY_SDR_CS12;
}

SoapySDR::ArgInfoList SoapyIrisLocal::getStreamArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList infos;

    {
        SoapySDR::ArgInfo info;
        info.key = "WIRE";
        info.name = "Stream wire format";
        info.type = SoapySDR::ArgInfo::STRING;
        info.value = "";
        info.description = "Specify a specific wire format for the stream.";
        info.options = {SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
        info.optionNames = {"Complex int16", "Complex int12", "Complex int8"};
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "MTU";
        info.name = "Ethernet MTU in bytes";
        info.type = SoapySDR::ArgInfo::INT;
        info.value = std::to_string(ETHERNET_MTU);
        info.description = "Configure a larger MTU for jumbo packets.";
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "SO_PRIORITY";
        info.name = "Socket send priority";
        info.type = SoapySDR::ArgInfo::INT;
        info.value = std::to_string(0);
        info.range = SoapySDR::Range(0, 10, 1);
        info.description = "Increase the priority of transmit data";
        infos.push_back(info);
    }

    //use remote infos that come from the driver itself
    //and filter out remote: from soapy remote (not applicable)
    for (const auto &info : _remote->getStreamArgsInfo(direction, channel))
    {
        if (info.key.find("remote:") == std::string::npos) infos.push_back(info);
    }

    return infos;
}

SoapySDR::Stream *SoapyIrisLocal::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &_channels,
    const SoapySDR::Kwargs &_args)
{
    //check the stream protocol for compatibility
    std::string streamProt("twbw32");
    try{streamProt = _remote->readSetting("STREAM_PROTOCOL");}
    catch (...){}
    if (streamProt != "twbw64") throw std::runtime_error(
        "Iris::setupStream: Stream protocol mismatch!"
        "Expected protocol twbw64, but firmware supports " + streamProt);

    std::unique_ptr<IrisLocalStream> data(new IrisLocalStream);
    std::vector<size_t> channels(_channels);
    if (channels.empty()) channels.push_back(0);

    //format configuration settings
    std::string remoteFormat;
    const auto &requestedWireFormat = _args.count("WIRE")?_args.at("WIRE"):"";
    resolveFormats(channels.size(), format, requestedWireFormat, data->format, remoteFormat, data->bytesPerElement);

    //query remote iris endpoint configuration
    auto remoteIPv6Addr       = _remote->readSetting("ETH0_IPv6_ADDR");
    const auto remoteServPort = _remote->readSetting("UDP_SERVICE_PORT");
    const auto rfTxFifoDepth = std::stoul(_remote->readSetting("RF_TX_FIFO_DEPTH"));
    if (remoteIPv6Addr.empty()) throw std::runtime_error("Iris::setupStream: Failed to query Iris IPv6 address");
    if (remoteServPort.empty()) throw std::runtime_error("Iris::setupStream: Failed to query Iris UDP service port");

    //ipv6 mac and scope for the remote socket
    std::string ethName;
    unsigned long long localMac64(0);
    int localScopeId(-1);
    {
        sklk_SoapyRPCSocket junkSock; junkSock.connect(_remoteURL);
        SoapyURL url(junkSock.getsockname());
        SockAddrData addr; auto err = url.toSockAddr(addr);
        sockAddrInterfaceLookup(addr.addr(), ethName, localMac64, localScopeId);
        if (ethName.empty()) throw std::runtime_error("Iris::setupStream: Failed to determine ethernet device name for " + url.getNode());
        if (localMac64 == 0) throw std::runtime_error("Iris::setupStream: Failed to lookup network hardware address for " + ethName);
        if (localScopeId == -1) throw std::runtime_error("Iris::setupStream: Failed to discover the IPv6 scope ID\n"
                                                         "  (Does interface='" + ethName + "' have an IPv6 address)?");
        SoapySDR::logf(SOAPY_SDR_INFO, "Using local ethernet interface: %s", ethName.c_str());
    }

    //get the scope id to get the remote ipv6 address with the local scope id
    const auto percentPos = remoteIPv6Addr.find_last_of('%');
    if (percentPos != std::string::npos)
    {
        remoteIPv6Addr = remoteIPv6Addr.substr(0, percentPos+1) + std::to_string(localScopeId);
    }

    data->direction = direction;
    data->readHandle = ~0;
    data->readElemsLeft = 0;
    data->readOffset = 0;
    data->tickCount = 0;
    data->inBurst = false;
    data->burstUsesTime = false;
    data->packetCount = 0;
    const size_t mtu = _args.count("MTU")?std::stoul(_args.at("MTU")):ETHERNET_MTU;
    const size_t mtuPayloadBytes = mtu - IPv6_UDP_SIZE - TWBW_HDR_SIZE;
    data->mtuElements = mtuPayloadBytes/data->bytesPerElement;
    data->numHostChannels = channels.size();
    data->hostFormatSize = SoapySDR::formatToSize(format);
    data->nextSeqSend = 0;
    data->lastSeqRecv = 0;
    const auto txFifoDepthBytes = rfTxFifoDepth*16;
    //what we actually buffer in the stream fifo...
    const size_t mtuLayer2Bytes = IPv6_UDP_SIZE + TWBW_HDR_SIZE + data->mtuElements*data->bytesPerElement;
    const size_t mtuBufferedBytes = ROUTE_HDR_SIZE + PADDED_ETH_HDR_SIZE + mtuLayer2Bytes;
    data->windowSize = txFifoDepthBytes/mtuBufferedBytes;

    //true by default, async can be useful, but it might cause a race w/ trigger and activate
    data->syncActivate = true;
    if (_args.count("SYNC_ACTIVATE") != 0) data->syncActivate = _args.at("SYNC_ACTIVATE") == "true";

    const SoapyURL bindURL("udp", "::", "0");
    int ret = data->sock.bind(bindURL.toString());
    if (ret != 0) throw std::runtime_error("Iris::setupStream: Failed to bind to " + bindURL.toString() + ": " + data->sock.lastErrorMsg());
    const SoapyURL connectURL("udp", remoteIPv6Addr, remoteServPort);
    ret = data->sock.connect(connectURL.toString());
    if (ret != 0) throw std::runtime_error("Iris::setupStream: Failed to connect to " + connectURL.toString() + ": " + data->sock.lastErrorMsg());

    //lookup the local mac address to program the framer
    SoapyURL localEp(data->sock.getsockname());

    //pass arguments within the args to program the framer
    SoapySDR::Kwargs args(_args);
    args["iris:eth_dst"] = std::to_string(localMac64);
    args["iris:ip6_dst"] = localEp.getNode();
    args["iris:udp_dst"] = localEp.getService();
    args["iris:mtu"] = std::to_string(data->mtuElements);
    SoapySDR::logf(SOAPY_SDR_INFO, "mtu %d bytes -> %d samples X %d channels, %d bytes per element",
        int(mtu), int(data->mtuElements), int(data->numHostChannels), int(data->bytesPerElement));

    //is the bypass mode supported for hardware acceleration?
    bool tryBypassMode(false);
    for (const auto &streamArg : _remote->getStreamArgsInfo(direction, channels.front()))
    {
        if (streamArg.key != "remote:prot") continue;
        auto it = std::find(streamArg.options.begin(), streamArg.options.end(), "none");
        tryBypassMode = it != streamArg.options.end();
    }

    //and is it also available on the server?
    //just check for the existence of remote:version
    //since older versions did not have support
    if (_remote->getHardwareInfo().count("remote:version") == 0) tryBypassMode = false;

    //try to setup the stream, bypassing the software streams for hardware acceleration
    if (tryBypassMode) try
    {
        args["remote:prot"] = "none";
        data->remoteStream = _remote->setupStream(direction, remoteFormat, channels, args);
    }
    catch(...){tryBypassMode = false;}
    //not working? fall back to old-type setup without the stream bypass support
    if (not tryBypassMode)
    {
        args.erase("remote:prot");
        data->remoteStream = _remote->setupStream(direction, remoteFormat, channels, args);
    }

    //if the rx stream was left running, stop it and drain the fifo
    if (direction == SOAPY_SDR_RX)
    {
        _remote->deactivateStream(data->remoteStream, 0, 0);
        while (data->sock.selectRecv(50000))
            data->sock.recv(data->buff, sizeof(data->buff));
    }

    if (direction == SOAPY_SDR_TX)
    {
        //route info used to route packets through hub
        data->routeEndpoints = 0x0;
        try{data->routeEndpoints = std::stoul(_remote->readSetting("ROUTE_ENDPOINTS"));}
        catch (...){SoapySDR::logf(SOAPY_SDR_ERROR, "Failed to query route endpoints");}
        if (data->routeEndpoints != 0) SoapySDR::logf(SOAPY_SDR_INFO,
            "Tx route: gateway[%.2x] -> dest[%.2x]",
            data->routeEndpoints & 0xff, data->routeEndpoints >> 8);

        data->running = true;
        data->thread = std::thread(&IrisLocalStream::statusLoop, data.get());
    }

    if (_args.count("SO_PRIORITY"))
    {
        ret = data->sock.setPriority(std::stoi(_args.at("SO_PRIORITY")));
        if (ret == -1) SoapySDR::logf(SOAPY_SDR_WARNING,
            "Failed to set socket priority: %s", data->sock.lastErrorMsg());
    }

    //set tx socket buffer size to match the buffering in the iris
    //set rx buffering size to be arbitrarily large for socket buffer
    size_t buffSize = (direction == SOAPY_SDR_RX)?RX_SOCKET_BUFFER_BYTES:txFifoDepthBytes;
    ret = data->sock.setBuffSize(direction == SOAPY_SDR_RX, buffSize);
    if (ret == -1) SoapySDR::logf(SOAPY_SDR_WARNING,
        "Failed to resize socket buffer to %d kib: %s", buffSize/1024, data->sock.lastErrorMsg());
    else
    {
        const size_t actualSize = data->sock.getBuffSize(direction == SOAPY_SDR_RX);
        if (actualSize < buffSize) SoapySDR::logf(SOAPY_SDR_WARNING,
            "Failed to resize socket buffer to %d kib, actual %d kib", buffSize/1024, actualSize/1024);
    }

    return (SoapySDR::Stream *)data.release();
}

void SoapyIrisLocal::closeStream(SoapySDR::Stream *stream)
{
    auto data = (IrisLocalStream *)stream;
    if (data->direction == SOAPY_SDR_TX)
    {
        data->running = false;
        data->thread.join();
    }
    _remote->closeStream(data->remoteStream);
    data->sock.close();
    delete data;
}

size_t SoapyIrisLocal::getStreamMTU(SoapySDR::Stream *stream) const
{
    auto data = (IrisLocalStream *)stream;
    return data->mtuElements;
}

int SoapyIrisLocal::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t numElems)
{
    auto data = (IrisLocalStream *)stream;
    if (data->syncActivate) return _remote->activateStream(data->remoteStream, flags, timeNs, numElems);
    data->async = std::async(std::launch::async, &SoapySDR::Device::activateStream, _remote, data->remoteStream, flags, timeNs, numElems);
    return 0;
}

int SoapyIrisLocal::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs)
{
    auto data = (IrisLocalStream *)stream;
    if (data->syncActivate) return _remote->deactivateStream(data->remoteStream, flags, timeNs);
    data->async = std::async(std::launch::async, &SoapySDR::Device::deactivateStream, _remote, data->remoteStream, flags, timeNs);
    return 0;
}

int SoapyIrisLocal::readStream(
    SoapySDR::Stream *stream,
    void * const *buffs,
    const size_t numElems,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    auto data = (IrisLocalStream *)stream;

    const bool onePkt = (flags & SOAPY_SDR_ONE_PACKET) != 0;
    bool eop = false;
    const int flags_in = flags;
    flags = 0; //clear

    size_t numRecv = 0;
    do
    {
        int flags_i(0);
        long long timeNs_i(0);

        //direct buffer call, there is no remainder left
        if (data->readElemsLeft == 0)
        {
            const void *buff[1];
            int ret = this->acquireReadBuffer(stream, data->readHandle, buff, flags_i, timeNs_i, timeoutUs);
            //timeout after some successful sends, leave loop
            if (ret == SOAPY_SDR_TIMEOUT and numRecv != 0) break;
            if (ret < 0) return ret;
            data->readOffset = size_t(buff[0]);
            data->readElemsLeft = size_t(ret);
            if (_tddMode and (flags_in & (1 << 29)) == 0/*reuse fft flag to shut this off*/)
            {
                unsigned sample_count =(unsigned)(((uint64_t)timeNs_i) & 0xFFFF);
                if (numRecv == 0 and sample_count != 0)
                {
                    SoapySDR::log(SOAPY_SDR_SSI, "D");
                }
                //std::cout << "received so far " << std::dec << sample_count + (unsigned)ret << std::endl;
                if (sample_count + (unsigned)ret >= numElems)
                    flags_i |= SOAPY_SDR_END_BURST;
            }
        }

        //always put the time in from the internally tracked tick rate
        //we do this for both new buffer handles which have good ticks
        //and for remainder buffers which get the tick interpolation
        flags_i |= SOAPY_SDR_HAS_TIME;
        if (!_tddMode)
            timeNs_i = this->ticksToTimeNs(data->tickCount, _adcClockRate);

        //convert the buffer
        void *buffsOffset[2];
        const size_t bytesOffset = numRecv*data->hostFormatSize;
        for (size_t i = 0; i < data->numHostChannels; i++) buffsOffset[i] = reinterpret_cast<void *>(size_t(buffs[i]) + bytesOffset);
        size_t numSamples = std::min(numElems-numRecv, data->readElemsLeft);
        convertToHost(data->format, (const void *)data->readOffset, buffsOffset, numSamples);

        //next internal tick count
        data->tickCount += numSamples;

        //used entire buffer, release
        if ((data->readElemsLeft -= numSamples) == 0)
        {
            this->releaseReadBuffer(stream, data->readHandle);
        }

        //increment pointers for next
        else
        {
            data->readOffset += numSamples*data->bytesPerElement;
        }

        eop = onePkt or (flags_i & (SOAPY_SDR_END_BURST | SOAPY_SDR_ONE_PACKET)) != 0;
        flags |= flags_i; //total set of any burst or time flags
        if (numRecv == 0) timeNs = timeNs_i; 
        numRecv += numSamples;
    } while (numRecv != numElems and not eop);

    //ended with fragments?
    if (data->readElemsLeft != 0)
    { 
        if (_tddMode)
            data->readElemsLeft = 0;
        else 
            flags |= SOAPY_SDR_MORE_FRAGMENTS;
    }
    return numRecv;
}

int SoapyIrisLocal::writeStream(
    SoapySDR::Stream *stream,
    const void * const *buffs,
    const size_t numElems,
    int &flags,
    const long long timeNs,
    const long timeoutUs)
{
    auto data = (IrisLocalStream *)stream;

    const bool onePkt = (flags & SOAPY_SDR_ONE_PACKET) != 0;

    size_t numSent = 0;
    do
    {
        //acquire a new handle
        size_t handle;
        void *buff[1];
        int ret = this->acquireWriteBuffer(stream, handle, buff, timeoutUs);

        //timeout after some successful sends, leave loop
        if (ret == SOAPY_SDR_TIMEOUT and numSent != 0) break;

        //return error if present
        if (ret < 0) return ret;

        //only end burst if the last sample can be released
        const size_t numLeft = numElems-numSent;
        const size_t numSamples = std::min<size_t>(ret, numLeft);
        int flags_i = (numSent+numSamples == numElems)?flags:(flags & ~(SOAPY_SDR_END_BURST));

        //convert the samples
        const void *buffsOffset[2];
        const size_t bytesOffset = numSent*data->hostFormatSize;
        for (size_t i = 0; i < data->numHostChannels; i++) buffsOffset[i] = reinterpret_cast<const void *>(size_t(buffs[i]) + bytesOffset);
        convertToWire(data->format, buffsOffset, buff[0], numSamples);

        //release the buffer to send the samples
        this->releaseWriteBuffer(stream, handle, numSamples, flags_i, timeNs);
        flags &= ~(SOAPY_SDR_HAS_TIME | SOAPY_SDR_WAIT_TRIGGER); //only valid on the first release
        numSent += numSamples;

    } while (numSent != numElems and not onePkt);

    return numSent;
}

void IrisLocalStream::statusLoop(void)
{
    THREAD_PRIO(0.7);
    while (running)
    {
        if (not this->sock.selectRecv(100000)) continue;

        uint64_t buff[16];
        int ret = this->sock.recv(buff, sizeof(buff));
        if (ret < 0) //socket error, end loop
        {
            {
                std::lock_guard<std::mutex> lock(this->mutex);
                this->queue.emplace_back(SOAPY_SDR_STREAM_ERROR);
            }
            this->cond.notify_all();
            return;
        }

        /*out_status_end_burst, out_status_underflow, out_status_time_late, out_status_has_time, out_status_valid, sequence_terror, sequence_tvalid, //86:80
        sequence_tdata, //79:64
        out_status_time}; //63:0*/
        StreamStatusEntry entry;
        entry.timeTicks = buff[0];
        const unsigned short sequence = (buff[1] & 0xffff);
        const bool hasSequence  = (buff[1] & (1 << 16)) != 0;
        const bool seqError     = hasSequence and (buff[1] & (1 << 17)) != 0;
        const bool hasStatus    = (buff[1] & (1 << 18)) != 0;
        const bool hasTime      = hasStatus and (buff[1] & (1 << 19)) != 0;
        const bool timeError    = hasStatus and (buff[1] & (1 << 20)) != 0;
        const bool underflow    = hasStatus and (buff[1] & (1 << 21)) != 0;
        const bool burstEnd     = hasStatus and (buff[1] & (1 << 22)) != 0;
        const bool overflow     = hasStatus and (buff[1] & (1 << 23)) != 0;
        /*std::cout << "got stat ret = " << std::dec << ret << std::endl;
        std::cout << "buff[0] " << std::hex << buff[0] << std::endl;
        std::cout << "buff[1] " << std::hex << buff[1] << std::endl;
        std::cout << "hasSequence " << hasSequence << std::endl;
        std::cout << "sequence " << sequence << std::endl;
        std::cout << "seqError " << seqError << std::endl;
        std::cout << "hasStatus " << hasStatus << std::endl;
        std::cout << "hasTime " << hasTime << std::endl;
        std::cout << "timeError " << timeError << std::endl;
        std::cout << "underflow " << underflow << std::endl;
        std::cout << "burstEnd " << burstEnd << std::endl;//*/

        //every status message contains a sequence
        //hasSequence just tells us it was a requested event
        this->lastSeqRecv = sequence;

        //error indicators
        if (hasTime) entry.flags |= SOAPY_SDR_HAS_TIME;
        if (burstEnd) entry.flags |= SOAPY_SDR_END_BURST;
        if (timeError) entry.ret = SOAPY_SDR_TIME_ERROR;
        if (underflow) entry.ret = SOAPY_SDR_UNDERFLOW;
        if (overflow) entry.ret = SOAPY_SDR_OVERFLOW;
        if (seqError) entry.ret = SOAPY_SDR_CORRUPTION;

        //enqueue status messages
        if (hasStatus or seqError)
        {
            if (underflow) SoapySDR::log(SOAPY_SDR_SSI, "U");
            if (overflow) SoapySDR::log(SOAPY_SDR_SSI, "O");
            if (timeError) SoapySDR::log(SOAPY_SDR_SSI, "T");
            if (seqError) SoapySDR::log(SOAPY_SDR_SSI, "S");
            std::lock_guard<std::mutex> lock(this->mutex);
            //constrain max queue size (if user isnt reading stream status)
            if (this->queue.size() > MAX_TX_STATUS_DEPTH) this->queue.pop_front();
            this->queue.push_back(entry);
        }

        //notify any waiters for sequence or status message
        this->cond.notify_all();
    }
}

int SoapyIrisLocal::readStreamStatus(
    SoapySDR::Stream *stream,
    size_t &chanMask,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    auto data = (IrisLocalStream *)stream;
    if (data->direction == SOAPY_SDR_RX) return SOAPY_SDR_NOT_SUPPORTED;

    //wait for an entry to become available
    std::unique_lock<std::mutex> lock(data->mutex);
    if (not data->cond.wait_for(lock,
        std::chrono::microseconds(timeoutUs),
        [data]{return not data->queue.empty();})) return SOAPY_SDR_TIMEOUT;

    //copy queue entry into the output fields
    auto entry = data->queue.front();
    data->queue.pop_front();
    chanMask = (data->numHostChannels == 2)?0x3:0x1;
    flags = entry.flags;
    timeNs = _tddMode?entry.timeTicks:this->ticksToTimeNs(entry.timeTicks, _dacClockRate);
    return entry.ret;
}

size_t SoapyIrisLocal::getNumDirectAccessBuffers(SoapySDR::Stream *)
{
    return 1; //single local buffer
}

int SoapyIrisLocal::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t /*handle*/, void **buffs)
{
    auto data = (IrisLocalStream *)stream;
    buffs[0] = data->buff + 2;
    return 0;
}

int SoapyIrisLocal::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    THREAD_PRIO(1.0);
    auto data = (IrisLocalStream *)stream;

    if (not data->sock.selectRecv(timeoutUs)) return SOAPY_SDR_TIMEOUT;
    handle = 0; //always 0, its just one buffer
    this->getDirectAccessBufferAddrs(stream, handle, (void**)buffs);

    int ret = data->sock.recv(data->buff, sizeof(data->buff));
    if (ret < 0) return SOAPY_SDR_STREAM_ERROR;

    //unpacker logic for twbw_rx_framer64
    const auto *hdr64 = data->buff;
    //std::cout << "===========================================\n";
    //std::cout << "read udp ret = " << std::dec << ret << std::endl;
    //std::cout << "hdr0 " << std::hex << hdr64[0] << std::endl;
    //std::cout << "hdr1 " << std::hex << hdr64[1] << std::endl;
    buffs[0] = (void *)(hdr64+2); //payload start
    const bool hasTime = (hdr64[0] & (1 << 31)) != 0;
    const bool timeError = (hdr64[0] & (1 << 30)) != 0;
    const bool overflow = (hdr64[0] & (1 << 29)) != 0;
    const bool isBurst = (hdr64[0] & (1 << 28)) != 0;
    const bool isTrigger = (hdr64[0] & (1 << 26)) != 0;
    const long long timeTicks = (long long)hdr64[1];
    const size_t burstCount = size_t(hdr64[0] & 0xffff) + 1;
    const size_t payloadBytes = size_t(ret)-(sizeof(uint64_t)*2);
    size_t numSamps = payloadBytes/data->bytesPerElement;

    //or end of burst but not totally full due to packing
    bool burstEnd = false;
    if (isBurst and burstCount <= numSamps and ((numSamps-burstCount)*data->bytesPerElement) < sizeof(uint64_t))
    {
        numSamps = burstCount;
        burstEnd = true;
    }

    flags = 0;
    ret = 0;

    //detect gaps in a burst due to drops
    if (!_tddMode && data->inBurst && data->tickCount != timeTicks)
    {
        flags |= SOAPY_SDR_END_ABRUPT;
        SoapySDR::log(SOAPY_SDR_SSI, "D");
        //std::cout << "\nDBG overflow " << data->packetCount << "\n\t"
        //    << "expected: " << data->tickCount << ", but got " << timeTicks << std::endl;
    }

    //gather time even if its not valid
    timeNs = _tddMode ? timeTicks : this->ticksToTimeNs(timeTicks, _adcClockRate);
    data->tickCount = timeTicks;

    //error indicators
    if (overflow) flags |= SOAPY_SDR_END_ABRUPT;
    if (hasTime) flags |= SOAPY_SDR_HAS_TIME; //always has time
    if (burstEnd) flags |= SOAPY_SDR_END_BURST;
    if (isTrigger) flags |= SOAPY_SDR_WAIT_TRIGGER;

    //a bad time was specified in the command packet
    else if (timeError)
    {
        SoapySDR::log(SOAPY_SDR_SSI, "L");
        ret = SOAPY_SDR_TIME_ERROR;
    }

    //otherwise the error was an overflow
    else if (overflow)
    {
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        ret = SOAPY_SDR_OVERFLOW;
    }

    //restart streaming when error in continuous mode
    if (ret != 0 and not isBurst)
    {
        //not implemented (and it probably wont backup anyway)
    }

    //release on error
    if (ret != 0)
    {
        releaseReadBuffer(stream, handle);
        return ret;
    }

    data->inBurst = !burstEnd;
    data->packetCount++;
    return numSamps;
}

void SoapyIrisLocal::releaseReadBuffer(SoapySDR::Stream *, const size_t)
{
    return; //nothing to do, buffer is reused, no ring present at this level
}

int SoapyIrisLocal::acquireWriteBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    void **buffs,
    const long timeoutUs)
{
    THREAD_PRIO(1.0);
    auto data = (IrisLocalStream *)stream;

    //ran out of sequences, wait for response
    auto ready = [data]{return uint16_t(data->nextSeqSend-data->lastSeqRecv) < data->windowSize;};
    if (not _tddMode and not ready()) //first check without locking, we only lock when backing up completely
    {
        std::unique_lock<std::mutex> lock(data->mutex);
        if (not data->cond.wait_for(lock, std::chrono::microseconds(timeoutUs), ready)) return SOAPY_SDR_TIMEOUT;
    }

    handle = 0; //always 0, its just one buffer
    this->getDirectAccessBufferAddrs(stream, handle, buffs);
    return data->mtuElements;
}

void SoapyIrisLocal::releaseWriteBuffer(
    SoapySDR::Stream *stream,
    const size_t /*handle*/,
    const size_t numElems,
    int &flags,
    const long long timeNs)
{
    auto data = (IrisLocalStream *)stream;

    //pack the header
    //void *payload;
    size_t len = 0;
    bool hasTime((flags & SOAPY_SDR_HAS_TIME) != 0);
    const long long expectedTickCount = data->tickCount;
    if (hasTime) data->tickCount = _tddMode?timeNs:this->timeNsToTicks(timeNs, _dacClockRate);
    else if (data->inBurst and data->burstUsesTime) hasTime = true;
    const bool burstEnd((flags & SOAPY_SDR_END_BURST) != 0);
    const bool trigger((flags & SOAPY_SDR_WAIT_TRIGGER) != 0);

    //sanity checks for burst values
    if (data->inBurst and !data->burstUsesTime and hasTime)
    {
        SoapySDR::logf(SOAPY_SDR_WARNING, "Got a timestamp in a data burst that began without a timestamp!");
    }
    if (!_tddMode and data->inBurst and hasTime and data->burstUsesTime and expectedTickCount != data->tickCount)
    {
        SoapySDR::logf(SOAPY_SDR_WARNING, "Discontinuous timestamp in a burst: expected=%lld, actual=%lld", expectedTickCount, data->tickCount);
    }

    //request sequence packets once in a while with this metric
    const bool seqRequest = (data->nextSeqSend)%(data->windowSize/8) == 0 and not _tddMode;

    //packer logic for twbw_tx_deframer64
    auto *hdr64 = data->buff;
    hdr64[0] = (uint64_t(numElems-1) & 0xffff) |
               (uint64_t(flags & 0xffff0000))  | //re-purpose upper bits of flags
               ((uint64_t(data->nextSeqSend++) & 0xffff) << 32) |
               (uint64_t(data->routeEndpoints) << 48);
    if (hasTime)    hdr64[0] |= (uint64_t(1) << 31);
    if (burstEnd)   hdr64[0] |= (uint64_t(1) << 28);
    if (trigger)    hdr64[0] |= (uint64_t(1) << 26);
    if (seqRequest) hdr64[0] |= (uint64_t(1) << 25);
    hdr64[1] = data->tickCount;
    len = (sizeof(uint64_t)*2) + numElems*data->bytesPerElement;
    //std::cout << "hdr64[0] " << std::hex << hdr64[0] << std::endl;

    int ret = data->sock.send(data->buff, len);
    if (ret != int(len)) SoapySDR::logf(SOAPY_SDR_ERROR,
        "releaseWriteBuffer() sock send(%d) failed: %d", int(len), ret);
    else
    {
        if (!data->inBurst) data->burstUsesTime = hasTime;
        data->inBurst = !burstEnd;
        data->packetCount++;
        data->tickCount += numElems;
    }
}
