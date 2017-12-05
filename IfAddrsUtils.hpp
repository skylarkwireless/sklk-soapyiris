// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//----------------------------------------------------------
//-- additional socket utilities using ifaddrs
//----------------------------------------------------------

#include "SoapyURLUtils.hpp"
#include <string>

void sockAddrInterfaceLookup(const SockAddrData &sa, std::string &ethName, long long &mac64, int &scopeId);
