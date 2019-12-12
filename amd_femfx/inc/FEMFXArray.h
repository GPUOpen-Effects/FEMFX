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
// Resizable array, has incomplete API but is just for limited uses in the lib.
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"
#include <type_traits>

namespace AMD
{
    template< class T, int N = 1 >
    class FmArray
    {
        T* pElems;
        T firstElems[N];
        uint32_t numElems;
        uint32_t maxElems;

    public:
        FM_CLASS_NEW_DELETE(FmArray)

        inline FmArray()
        {
            FM_STATIC_ASSERT(std::is_trivially_constructible<T>::value && std::is_trivially_destructible<T>::value);
            pElems = firstElems;
            numElems = 0;
            maxElems = N;
        }

        inline FmArray(const FmArray& other)
        {
            pElems = firstElems;
            numElems = 0;
            maxElems = N;
            *this = other;
        }

        inline ~FmArray()
        {
            Clear();
        }

        inline FmArray& operator =(const FmArray& other)
        {
            Clear();
            uint otherNumElems = other.numElems;
            Reserve(otherNumElems);
            for (uint i = 0; i < otherNumElems; i++)
            {
                Add(other[i]);
            }
            return *this;
        }

        inline T* GetData()
        {
            return pElems;
        }

        inline uint32_t GetNumElems() const
        {
            return numElems;
        }

        inline T& operator[] (uint32_t i)
        {
            return pElems[i];
        }

        inline const T& operator[] (uint32_t i) const
        {
            return pElems[i];
        }

        inline void Add(const T& inElem)
        {
            if (numElems >= maxElems)
            {
                T* pOldElems = pElems;
                uint32_t newMaxElems = maxElems == 0 ? 1 : maxElems * 2;
                pElems = (T*)FmAlignedMalloc(sizeof(T)*newMaxElems, FM_ALIGN_OF(T));
                for (uint32_t elemIdx = 0; elemIdx < numElems; elemIdx++)
                {
                    pElems[elemIdx] = pOldElems[elemIdx];
                }
                if (pOldElems != firstElems)
                {
                    FmAlignedFree(pOldElems);
                }
                maxElems = newMaxElems;
            }
            pElems[numElems] = inElem;
            numElems++;
        }

        inline void Reserve(uint totalNumElems)
        {
            if (totalNumElems > maxElems)
            {
                T* pOldElems = pElems;
                uint32_t newMaxElems = totalNumElems;
                pElems = (T*)FmAlignedMalloc(sizeof(T) * newMaxElems, FM_ALIGN_OF(T));
                for (uint32_t elemIdx = 0; elemIdx < numElems; elemIdx++)
                {
                    pElems[elemIdx] = pOldElems[elemIdx];
                }
                if (pOldElems != firstElems)
                {
                    FmAlignedFree(pOldElems);
                }
                maxElems = newMaxElems;
            }
        }

        inline void Clear()
        {
            if (pElems != firstElems)
            {
                FmAlignedFree(pElems);
                pElems = firstElems;
            }
            numElems = 0;
            maxElems = N;
        }
    };



}