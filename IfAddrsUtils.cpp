// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//----------------------------------------------------------
//-- additional socket utilities using getifaddrs
//----------------------------------------------------------

#include "SoapySocketDefs.hpp"
#include <sys/types.h>
#include <cstring>
#include <iostream>
#include <map>

//mac address platform specific
#ifdef __APPLE__
#  include <net/if_dl.h>
#  define AF_PACKET AF_LINK
#  define sockaddr_ll sockaddr_dl
#  define LLHALEN(s) ((s)->sdl_alen)
#else
#  include <linux/if_packet.h>
#  define LLADDR(s) ((s)->sll_addr)
#  define LLHALEN(s) ((s)->sll_halen)
#endif

void sockAddrInterfaceLookup(const sockaddr *sa, std::string &ethName, unsigned long long &mac64, int &scopeId)
{
    struct ifaddrs *ifaddr;
    if (getifaddrs(&ifaddr) == -1) return;

    std::map<std::string, std::map<int, sockaddr *>> ethToFamilyToSa;

    for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL) continue;
        const bool fa_match = (ifa->ifa_addr->sa_family == sa->sa_family);

        if (fa_match and ifa->ifa_addr->sa_family == AF_INET and
            reinterpret_cast<const struct sockaddr_in *>(sa)->sin_addr.s_addr ==
            reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr)->sin_addr.s_addr) ethName = ifa->ifa_name;

        if (fa_match and ifa->ifa_addr->sa_family == AF_INET6 and std::memcmp(
            reinterpret_cast<const struct sockaddr_in6 *>(sa)->sin6_addr.s6_addr,
            reinterpret_cast<const struct sockaddr_in6 *>(ifa->ifa_addr)->sin6_addr.s6_addr,
            sizeof(in6_addr)) ==0) ethName = ifa->ifa_name;

        ethToFamilyToSa[ifa->ifa_name][ifa->ifa_addr->sa_family] = ifa->ifa_addr;
    }

    if (not ethName.empty())
    {
        const auto &ethAddrs = ethToFamilyToSa.at(ethName);
        const auto &macAddr = ethAddrs.find(AF_PACKET);
        if (macAddr != ethAddrs.end())
        {
            const auto p = LLADDR((struct sockaddr_ll*)macAddr->second);
            const auto n = LLHALEN((struct sockaddr_ll*)macAddr->second);
            for (int i = n-1; i >= 0; i--) mac64 = (mac64 << 8) | uint8_t(p[i]);
        }
        const auto &ipV6Addr = ethAddrs.find(AF_INET6);
        if (ipV6Addr != ethAddrs.end())
        {
            scopeId = reinterpret_cast<const struct sockaddr_in6 *>(ipV6Addr->second)->sin6_scope_id;
        }
    }
    freeifaddrs(ifaddr);
}
