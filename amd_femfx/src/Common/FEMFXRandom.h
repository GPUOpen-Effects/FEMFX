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
// Random number generator with non-global state to allow determinism with
// multi-threading
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"

namespace AMD
{
    // FNV-1a hash function as described in https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
    static FM_FORCE_INLINE uint FmProcessByteFNV1a(uint value, uint8_t byteValue)
    {
        return (value ^ (uint)byteValue) * 0x1000193;
    }

    static FM_FORCE_INLINE uint FmComputeHashFNV1a(uint value)
    {
        uint result = 0x811C9DC5;
        result = FmProcessByteFNV1a(result, (uint8_t)(value));
        result = FmProcessByteFNV1a(result, (uint8_t)(value >> 8));
        result = FmProcessByteFNV1a(result, (uint8_t)(value >> 16));
        result = FmProcessByteFNV1a(result, (uint8_t)(value >> 24));
        return result;
    }

    static FM_FORCE_INLINE uint FmComputeHash(uint a)
    {
        return FmComputeHashFNV1a(a);
    }

    struct FmRandomState
    {
        uint state;

        FM_FORCE_INLINE FmRandomState(uint seed = 0)
        {
            state = seed;
        }

        FM_FORCE_INLINE void Init(uint seed)
        {
            state = seed;
        }

        FM_FORCE_INLINE uint RandomUint()
        {
            state = FmComputeHash(state);
            return state;
        }

        FM_FORCE_INLINE float RandomFloatZeroToOne()
        {
            state = FmComputeHash(state);
            return (float)((double)state / (double)0xFFFFFFFF);
        }

        FM_FORCE_INLINE float RandomFloatMinusOneToOne()
        {
            return RandomFloatZeroToOne() * 2.0f - 1.0f;
        }
    };

    // Shuffle array of indices for permutations
    static inline void FmShuffleIndices(uint* indices, uint numIndices, FmRandomState& randomState)
    {
        for (int i = (int)numIndices - 1; i > 0; i--)
        {
            // Swap value i with random earlier value
            uint j = randomState.RandomUint() % (uint)i;
            uint tmp = indices[j];
            indices[j] = indices[i];
            indices[i] = tmp;
        }
    }

    // Init array of indices for permutations
    static inline void FmInitIndices(uint* indices, uint numIndices)
    {
        for (uint i = 0; i < numIndices; i++)
        {
            indices[i] = i;
        }
    }
}
