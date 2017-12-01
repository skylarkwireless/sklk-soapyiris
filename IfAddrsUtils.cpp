// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//----------------------------------------------------------
//-- additional socket utilities using ifaddrs
//----------------------------------------------------------

#include "IfAddrsUtils.hpp"
#include "SoapySocketDefs.hpp"
#include <sys/types.h>
#include <ifaddrs.h>
#include <linux/if_packet.h>
#include <cstring>
#include <iostream>

std::string sockAddrToEthName(const SockAddrData &sa)
{
    std::string name;
    struct ifaddrs *ifaddr;
    if (getifaddrs(&ifaddr) == -1) return name;

    for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL) continue;
        if (ifa->ifa_addr->sa_family != sa.addr()->sa_family) continue;

        if (ifa->ifa_addr->sa_family == AF_INET &&
            reinterpret_cast<const struct sockaddr_in *>(sa.addr())->sin_addr.s_addr ==
            reinterpret_cast<const struct sockaddr_in *>(ifa->ifa_addr)->sin_addr.s_addr) name = ifa->ifa_name;

        if (ifa->ifa_addr->sa_family == AF_INET6 && std::memcmp(
            reinterpret_cast<const struct sockaddr_in6 *>(sa.addr())->sin6_addr.s6_addr,
            reinterpret_cast<const struct sockaddr_in6 *>(ifa->ifa_addr)->sin6_addr.s6_addr,
            sizeof(in6_addr)) ==0) name = ifa->ifa_name;
    }

    freeifaddrs(ifaddr);
    return name;
}

long long ethNameToHwAddr64(const std::string &name)
{
    uint64_t mac64(0);
    struct ifaddrs *ifaddr;
    if (getifaddrs(&ifaddr) == -1) return mac64;

    for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL) continue;
        if (ifa->ifa_addr->sa_family != AF_PACKET) continue;
        if (ifa->ifa_name != name) continue;
        struct sockaddr_ll *s = (struct sockaddr_ll*)ifa->ifa_addr;
        for (int i = sizeof(s->sll_addr)-1; i >= 0; i--)
        {
            mac64 = (mac64 << 8) | s->sll_addr[i];
        }
    }

    freeifaddrs(ifaddr);
    return mac64;
}

int ethNameToIpv6ScopeId(const std::string &name)
{
    int scopeId(-1);
    struct ifaddrs *ifaddr;
    if (getifaddrs(&ifaddr) == -1) return scopeId;

    for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL) continue;
        if (ifa->ifa_addr->sa_family != AF_INET6) continue;
        if (ifa->ifa_name != name) continue;
        scopeId = reinterpret_cast<const struct sockaddr_in6 *>(ifa->ifa_addr)->sin6_scope_id;
    }

    freeifaddrs(ifaddr);
    return scopeId;
}
