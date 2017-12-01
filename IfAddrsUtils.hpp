// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//----------------------------------------------------------
//-- additional socket utilities using ifaddrs
//----------------------------------------------------------

#include "SoapyURLUtils.hpp"

std::string sockAddrToEthName(const SockAddrData &sa);

long long ethNameToHwAddr64(const std::string &name);

int ethNameToIpv6ScopeId(const std::string &name);
