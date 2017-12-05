// Copyright (c) 2017 Skylark Wireless LLC
// SPDX-License-Identifier: BSD-3-Clause

//----------------------------------------------------------
//-- additional socket utilities using ifaddrs
//----------------------------------------------------------

#include "IfAddrsUtils.hpp"
#include "SoapySocketDefs.hpp"
#include <iostream>
#include <vector>
#include <map>

#include <winsock2.h>
#include <iphlpapi.h>
#include <stdio.h>
#include <stdlib.h>

// Link with Iphlpapi.lib
#pragma comment(lib, "IPHLPAPI.lib")

#define WORKING_BUFFER_SIZE 15000
#define MAX_TRIES 3

#define MALLOC(x) HeapAlloc(GetProcessHeap(), 0, (x))
#define FREE(x) HeapFree(GetProcessHeap(), 0, (x))

void sockAddrInterfaceLookup(const SockAddrData &sa, std::string &ethName, long long &mac64, int &scopeId)
{
    std::map<std::string, std::vector<BYTE>> ethToMac;
    std::map<std::string, DWORD> ethToIpV6Index;

    //nothing but simplicity https://msdn.microsoft.com/en-us/library/aa365915.aspx
    DWORD dwSize = 0;
    DWORD dwRetVal = 0;

    unsigned int i = 0;

    // Set the flags to pass to GetAdaptersAddresses
    ULONG flags = GAA_FLAG_INCLUDE_PREFIX;

    // default to unspecified address family (both)
    ULONG family = AF_UNSPEC;

    LPVOID lpMsgBuf = NULL;

    PIP_ADAPTER_ADDRESSES pAddresses = NULL;
    ULONG outBufLen = 0;
    ULONG Iterations = 0;

    PIP_ADAPTER_ADDRESSES pCurrAddresses = NULL;

    // Allocate a 15 KB buffer to start with.
    outBufLen = WORKING_BUFFER_SIZE;

    do {

        pAddresses = (IP_ADAPTER_ADDRESSES *) MALLOC(outBufLen);
        if (pAddresses == NULL) {
            printf
                ("Memory allocation failed for IP_ADAPTER_ADDRESSES struct\n");
            return;
        }

        dwRetVal =
            GetAdaptersAddresses(family, flags, NULL, pAddresses, &outBufLen);

        if (dwRetVal == ERROR_BUFFER_OVERFLOW) {
            FREE(pAddresses);
            pAddresses = NULL;
        } else {
            break;
        }

        Iterations++;

    } while ((dwRetVal == ERROR_BUFFER_OVERFLOW) && (Iterations < MAX_TRIES));

    if (dwRetVal == NO_ERROR) {
        // If successful, output some information from the data we received
        pCurrAddresses = pAddresses;
        while (pCurrAddresses) {
            for (auto addr_i = pCurrAddresses->FirstUnicastAddress; addr_i != NULL; addr_i = addr_i->Next)
            {
                const auto a = addr_i->Address.lpSockaddr;
                const bool fa_match = (a->sa_family == sa.addr()->sa_family);

                if (fa_match and a->sa_family == AF_INET and
                    reinterpret_cast<const struct sockaddr_in *>(sa.addr())->sin_addr.s_addr ==
                    reinterpret_cast<const struct sockaddr_in *>(a)->sin_addr.s_addr) ethName = pCurrAddresses->AdapterName;

                if (fa_match and a->sa_family == AF_INET6 and std::memcmp(
                    reinterpret_cast<const struct sockaddr_in6 *>(sa.addr())->sin6_addr.s6_addr,
                    reinterpret_cast<const struct sockaddr_in6 *>(a)->sin6_addr.s6_addr,
                    sizeof(in6_addr)) ==0) ethName = pCurrAddresses->AdapterName;

                ethToIpV6Index[pCurrAddresses->AdapterName] = pCurrAddresses->Ipv6IfIndex;
            }
            ethToMac[pCurrAddresses->AdapterName] = std::vector<BYTE>(pCurrAddresses->PhysicalAddress,
                pCurrAddresses->PhysicalAddress+pCurrAddresses->PhysicalAddressLength);

            pCurrAddresses = pCurrAddresses->Next;
        }
    } else {
        printf("Call to GetAdaptersAddresses failed with error: %d\n",
               dwRetVal);
        if (dwRetVal == ERROR_NO_DATA)
            printf("\tNo addresses were found for the requested parameters\n");
        else {

            if (FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                    FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, 
                    NULL, dwRetVal, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),   
                    // Default language
                    (LPTSTR) & lpMsgBuf, 0, NULL)) {
                printf("\tError: %s", (const char *)lpMsgBuf);
                LocalFree(lpMsgBuf);
                if (pAddresses)
                    FREE(pAddresses);
                return;
            }
        }
    }

    if (not ethName.empty())
    {
        const auto &ipv6Index = ethToIpV6Index.find(ethName);
        if (ipv6Index != ethToIpV6Index.end()) scopeId = ipv6Index->second;

        const auto &macAddr = ethToMac.find(ethName);
        if (macAddr != ethToMac.end())
        {
            for (int i = int(macAddr->second.size())-1; i >= 0; i--)
            {
                mac64 = (mac64 << 8) | macAddr->second[i];
            }
        }
    }

    if (pAddresses) {
        FREE(pAddresses);
    }
}
