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

#pragma once

#include "FEMFXCommonInternal.h"

namespace AMD
{
    // Keeps list of free ids for recycling
    struct FmFreeIds
    {
        uint* freeIdsArray;
        uint numFreeIds;
        uint maxIds;
    };

    static inline void FmInitFreeIds(FmFreeIds* freeIds)
    {
        freeIds->freeIdsArray = NULL;
        freeIds->numFreeIds = 0;
        freeIds->maxIds = 0;
    }

    // Init free ids list with provided memory for elements.
    static inline void FmInitFreeIds(FmFreeIds* freeIds, uint* freeIdsArray, uint maxIds, uint rbFlag = 0)
    {
        freeIds->freeIdsArray = freeIdsArray;
        freeIds->numFreeIds = maxIds;
        freeIds->maxIds = maxIds;
        for (uint i = 0; i < maxIds; i++)
        {
            freeIds->freeIdsArray[i] = (maxIds - 1 - i) | rbFlag;
        }
    }

    // Reset free ids list.
    static inline void FmResetFreeIds(FmFreeIds* freeIds, uint rbFlag = 0)
    {
        uint maxIds = freeIds->maxIds;
        freeIds->numFreeIds = maxIds;
        for (uint i = 0; i < maxIds; i++)
        {
            freeIds->freeIdsArray[i] = (maxIds - 1 - i) | rbFlag;
        }
    }

    // Return a free id, or FM_INVALID_ID if none
    static FM_FORCE_INLINE uint FmReserveId(FmFreeIds* freeIds)
    {
        if (freeIds->numFreeIds == 0)
        {
            return FM_INVALID_ID;
        }
        uint id = freeIds->freeIdsArray[freeIds->numFreeIds - 1];
        freeIds->numFreeIds--;
        return id;
    }

    // Remove an id from map, returning it to free list
    static FM_FORCE_INLINE void FmReleaseId(FmFreeIds* freeIds, uint id)
    {
        freeIds->freeIdsArray[freeIds->numFreeIds] = id;
        freeIds->numFreeIds++;
    }

}