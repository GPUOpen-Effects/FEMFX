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

//---------------------------------------------------------------------------------------
// API for platform atomic operations
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"

#ifdef WIN32
#include <Windows.h>
#endif

namespace AMD
{
    // Atomic operation interface

#ifdef WIN32
    // Atomically increment and return new value
    static FM_FORCE_INLINE uint FmAtomicIncrement(uint* pValue)
    {
        return InterlockedIncrement((volatile unsigned long *)pValue);
    }

    // Atomically decrement and return new value
    static FM_FORCE_INLINE uint FmAtomicDecrement(uint* pValue)
    {
        return InterlockedDecrement((volatile unsigned long *)pValue);
    }

    // Atomically add to *pValue and return result
    static FM_FORCE_INLINE uint FmAtomicAdd(uint* pValue, uint addedValue)
    {
        return (uint)InterlockedAdd((volatile LONG *)pValue, (LONG)addedValue);
    }

    // Atomically subtract from *pValue and return result
    static FM_FORCE_INLINE uint FmAtomicSub(uint* pValue, uint subtractedValue)
    {
        return (uint)InterlockedAdd((volatile LONG *)pValue, -(LONG)subtractedValue);
    }

    // Atomically OR with *pValue and return result
    static FM_FORCE_INLINE uint FmAtomicOr(uint* pValue, uint orValue)
    {
        return (uint)InterlockedOr((volatile LONG *)pValue, (LONG)orValue);
    }

    // Atomically replace *pValue with newValue if currently equal to compareValue, and return the initial value
    static FM_FORCE_INLINE uint FmAtomicCompareExchange(uint* pValue, uint newValue, uint compareValue)
    {
        return InterlockedCompareExchange((volatile unsigned long *)pValue, newValue, compareValue);
    }

    // Atomically read current *pValue
    static FM_FORCE_INLINE uint FmAtomicRead(uint* pValue)
    {
        uint value = *(volatile unsigned long*)pValue;

        // Ensure subsequent loads ordered after read of value
        _mm_lfence();

        return value;
    }

    // Atomically write to *pValue
    static FM_FORCE_INLINE uint FmAtomicWrite(uint* pValue, uint newValue)
    {
        return InterlockedExchange((volatile unsigned long *)pValue, newValue);
    }
#endif

    // Atomically compute max
    void FmAtomicMax(uint* pValue, uint newValue);
}