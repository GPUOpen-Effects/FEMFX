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

#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#ifdef _WIN32
#include <Windows.h>
#include <malloc.h>
#include <process.h>
#include <intrin.h>
#endif

#ifdef _WIN32
#define TL_FORCE_INLINE __forceinline
#define TL_ALIGN(x) __declspec(align(x))
#define TL_ALIGN_END(x)
#define TL_ALIGN_OF(x) __alignof(x)
#define TL_ALIGNED_MALLOC(size, alignment) _aligned_malloc(size, alignment)
#define TL_ALIGNED_FREE(ptr) _aligned_free(ptr)
#define TL_THREAD_LOCAL_STORAGE __declspec(thread)
#if _DEBUG
#define TL_ASSERT(x) if (!(x)) { fprintf(stderr, "TL assert failed: " #x "\n"); assert(0); }
#else
#define TL_ASSERT(x)
#endif
#else
#error "Not implemented"
#define TL_FORCE_INLINE inline
#define TL_ALIGN(x)
#define TL_ALIGN_END(x)
#define TL_ALIGN_OF(x)
#define TL_ALIGNED_MALLOC(size, alignment)
#define TL_ALIGNED_FREE(ptr)
#define TL_THREAD_LOCAL_STORAGE
#define TL_ASSERT(x)
#endif

#define TL_CLASS_NEW_DELETE(T)\
    inline void* operator new(size_t size)\
    {\
        return TL_ALIGNED_MALLOC(size, TL_ALIGN_OF(T));\
    }\
    inline void operator delete(void* ptr)\
    {\
        TL_ALIGNED_FREE(ptr);\
    }\
    inline void* operator new [](size_t size)\
    {\
        return TL_ALIGNED_MALLOC(size, TL_ALIGN_OF(T));\
    }\
    inline void operator delete[] (void* ptr)\
    {\
        TL_ALIGNED_FREE(ptr);\
    }

namespace AMD
{
#ifdef _WIN32
    typedef HANDLE TLThread;
    typedef uint32_t(__stdcall *TLJoinableThreadFunc)(void *);

    static inline void TLCreateJoinableThread(TLThread* thread, TLJoinableThreadFunc func, void* argList, size_t stackSize, const char* threadName)
    {
        (void)threadName;
        uint32_t threadAddr;
        *thread = (HANDLE)_beginthreadex(NULL, (int)stackSize, func, argList, 0, &threadAddr);
    }

    static inline void TLExitJoinableThread()
    {
        _endthreadex(0);
    }

    static inline void TLJoinThread(TLThread thread)
    {
        WaitForSingleObject(thread, INFINITE);
        CloseHandle(thread);
    }

    // Pause for spin-waits
    static TL_FORCE_INLINE void TLPause()
    {
        _mm_pause();
    }

    // Load fence; place after load of signal value and before loading data associated with signal
    static TL_FORCE_INLINE void TLLoadFence()
    {
        _mm_lfence();
    }

    // Store fence; place before store of signal value and after storing data associated with signal
    static TL_FORCE_INLINE void TLStoreFence()
    {
        _mm_sfence();
    }

    static TL_FORCE_INLINE void TLYield()
    {
        Sleep(0);
    }
#else
#error "Not implemented"
#endif

    // Get a system time value
#ifdef WIN32
    class TLTimer
    {
        LARGE_INTEGER freq;
        LARGE_INTEGER startTime;

    public:
        TLTimer()
        {
            QueryPerformanceFrequency(&freq);
            startTime.QuadPart = 0;
        }

        TL_FORCE_INLINE void SetStartTime()
        {
            QueryPerformanceCounter(&startTime);
        }

        TL_FORCE_INLINE int64_t GetElapsedMicroseconds()
        {
            LARGE_INTEGER endTime, elapsedTime;
            QueryPerformanceCounter(&endTime);

            elapsedTime.QuadPart = endTime.QuadPart - startTime.QuadPart;
            elapsedTime.QuadPart *= 1000000;
            elapsedTime.QuadPart /= freq.QuadPart;  // microseconds resolution
            return (int64_t)elapsedTime.QuadPart;
        }

        TL_FORCE_INLINE bool HasElapsedMicroseconds(int64_t microseconds)
        {
            LARGE_INTEGER endTime, elapsedTime;
            QueryPerformanceCounter(&endTime);

            elapsedTime.QuadPart = endTime.QuadPart - startTime.QuadPart;
            elapsedTime.QuadPart *= 1000000;

            return (elapsedTime.QuadPart >= microseconds * freq.QuadPart);
        }
    };
#else
#error "Not implemented"
#endif

    // Atomic operations
    struct TLAtomicInt
    {
        TL_ALIGN(64) int32_t val TL_ALIGN_END(64);  // size/alignment of cache block
    };

    struct TLAtomicInt64
    {
        TL_ALIGN(64) int64_t val TL_ALIGN_END(64);  // size/alignment of cache block
    };

#ifdef _WIN32
    static TL_FORCE_INLINE int32_t TLAtomicIncrement(int32_t* pValue)
    {
        return InterlockedIncrement((volatile unsigned long *)pValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicIncrement64(int64_t* pValue)
    {
        return InterlockedIncrement64((volatile LONG64 *)pValue);
    }

    static TL_FORCE_INLINE int32_t TLAtomicDecrement(int32_t* pValue)
    {
        return InterlockedDecrement((volatile unsigned long *)pValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicDecrement64(int64_t* pValue)
    {
        return InterlockedDecrement64((volatile LONG64 *)pValue);
    }

    // Atomically add to *pValue and return result
    static TL_FORCE_INLINE int32_t TLAtomicAdd(int32_t* pValue, int addedValue)
    {
        return InterlockedAdd((volatile long *)pValue, addedValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicAdd64(int64_t* pValue, int64_t addedValue)
    {
        return InterlockedAdd64((volatile LONG64 *)pValue, addedValue);
    }

    // Atomically OR with *pValue and return result
    static TL_FORCE_INLINE int32_t TLAtomicOr(int32_t* pValue, int32_t orValue)
    {
        return InterlockedOr((volatile long *)pValue, orValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicOr64(int64_t* pValue, int64_t orValue)
    {
        return InterlockedOr64((volatile long long *)pValue, orValue);
    }

    // Atomically AND with *pValue and return result
    static TL_FORCE_INLINE int32_t TLAtomicAnd(int32_t* pValue, int32_t andValue)
    {
        return InterlockedAnd((volatile long *)pValue, andValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicAnd64(int64_t* pValue, int64_t andValue)
    {
        return InterlockedAnd64((volatile long long *)pValue, andValue);
    }

    // Atomically replace *pValue with newValue if currently equal to compareValue, and return the initial value
    static TL_FORCE_INLINE int32_t TLAtomicCompareExchange(int32_t* pValue, int32_t newValue, int32_t compareValue)
    {
        return InterlockedCompareExchange((volatile unsigned long *)pValue, newValue, compareValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicCompareExchange64(int64_t* pValue, int64_t newValue, int64_t compareValue)
    {
        return InterlockedCompareExchange64((volatile LONG64 *)pValue, newValue, compareValue);
    }

    // Atomically read current *pValue, with load fence
    static TL_FORCE_INLINE int32_t TLAtomicRead(int32_t* pValue)
    {
        int32_t value = *(volatile unsigned long *)pValue;

        // Ensure subsequent loads ordered after read of value
        _mm_lfence();

        return value;
    }
    static TL_FORCE_INLINE int64_t TLAtomicRead64(int64_t* pValue)
    {
        int64_t value = *(volatile LONG64 *)pValue;

        // Ensure subsequent loads ordered after read of value
        _mm_lfence();

        return value;
    }

    // Atomically write to *pValue and return previous value
    static TL_FORCE_INLINE int32_t TLAtomicWrite(int32_t* pValue, int32_t newValue)
    {
        return InterlockedExchange((volatile unsigned long *)pValue, newValue);
    }
    static TL_FORCE_INLINE int64_t TLAtomicWrite64(int64_t* pValue, int64_t newValue)
    {
        return InterlockedExchange64((volatile LONG64 *)pValue, newValue);
    }

    // From "Use Best Practices with Spin Locks" by Kenneth Mitchell
    static TL_FORCE_INLINE void TLLock(int32_t* pl)
    {
        const int32_t LOCK_IS_TAKEN = 1;
        while ((LOCK_IS_TAKEN == TLAtomicRead(pl)) || (LOCK_IS_TAKEN == TLAtomicWrite(pl, LOCK_IS_TAKEN)))
        {
            TLPause();
        }
    }

    static TL_FORCE_INLINE void TLUnlock(int32_t* pl)
    {
        TLAtomicWrite(pl, 0);
    }
#else
#error "Not implemented"
#endif
}