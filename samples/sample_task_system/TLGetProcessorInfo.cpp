/*
MIT License

Copyright (c) 2019 Advanced Micro Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "TLCommon.h"

#ifdef WIN32
#include <stdio.h>
#include <intrin.h>
#include <windows.h>
#endif

namespace AMD
{
#ifdef WIN32
// Based on: https://github.com/GPUOpen-LibrariesAndSDKs/cpu-core-counts/blob/master/windows/ThreadCount-Win7.cpp
/**********************************************************************
MIT License

Copyright(c) 2017 GPUOpen Libraries & SDKs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**********************************************************************/

    // This advice is specific to AMD processors and is not general guidance for all processor manufacturers.
    //
    // GetLogicalProcessorInformationEx requires Win7 or later
    // based on https://msdn.microsoft.com/en-us/library/windows/desktop/dd405488(v=vs.85).aspx

    int countSetBits(uint64_t bitMask)
    {
        return (int)_mm_popcnt_u64(bitMask);
    }

    void getProcessorCount(DWORD& cores, DWORD& logical) {
        cores = logical = 0;
        char* buffer = NULL;
        DWORD len = 0;
        if (FALSE == GetLogicalProcessorInformationEx(RelationAll, (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)buffer, &len)) {
            if (GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
                buffer = (char*)malloc(len);
                if (GetLogicalProcessorInformationEx(RelationAll, (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)buffer, &len)) {
                    char* ptr = buffer;
                    while (ptr < buffer + len) {
                        PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX pi = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)ptr;
                        if (pi->Relationship == RelationProcessorCore) {
                            cores++;
                            for (size_t g = 0; g < pi->Processor.GroupCount; ++g) {
                                logical += countSetBits(pi->Processor.GroupMask[g].Mask);
                            }
                        }
                        ptr += pi->Size;
                    }
                }
                free(buffer);
            }
        }
    }

    char* getCpuidVendor(char* vendor) {
        int data[4];
        __cpuid(data, 0);
        *reinterpret_cast<int*>(vendor) = data[1];
        *reinterpret_cast<int*>(vendor + 4) = data[3];
        *reinterpret_cast<int*>(vendor + 8) = data[2];
        vendor[12] = 0;
        return vendor;
    }

    int getCpuidFamily() {
        int data[4];
        __cpuid(data, 1);
        int family = ((data[0] >> 8) & 0x0F);
        int extendedFamily = (data[0] >> 20) & 0xFF;
        int displayFamily = (family != 0x0F) ? family : (extendedFamily + family);
        return displayFamily;
    }

    // This advice is specific to AMD processors and is
    // not general guidance for all processor
    // manufacturers. Remember to profile!
    DWORD getDefaultThreadCount() {
        DWORD cores, logical;
        getProcessorCount(cores, logical);
        DWORD count = logical;
        char vendor[13];
        getCpuidVendor(vendor);
        if (0 == strcmp(vendor, "AuthenticAMD")) {
            if (0x15 == getCpuidFamily()) {
                // AMD "Bulldozer" family microarchitecture
                count = logical;
            }
            else {
                count = cores;
            }
        }
        return count;
    }
#endif

    int32_t TLGetProcessorInfo(int32_t* pNumPhysicalCores, int32_t* pNumLogicalCores)
    {
        DWORD cores, logical;
        getProcessorCount(cores, logical);

        *pNumPhysicalCores = cores;
        *pNumLogicalCores = logical;

        return 0;
    }
}
