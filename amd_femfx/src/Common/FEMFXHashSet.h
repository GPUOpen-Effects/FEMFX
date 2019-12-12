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
// Template for basic hash set, only supporting insertions
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXRandom.h"

namespace AMD
{
    /*
    struct ExampleElement
    {
        uint key;
        float value;
    };

    struct ExampleSetFuncs
    {
        // Set key reserved to indicate unused element 
        static void SetInvalidKey(uint& key)
        {
            key = -1;
        }

        // Check if key is valid
        static bool IsValidKey(const uint& key)
        {
            return (key != -1);
        }

        // Test two keys for equality
        static bool KeysEqual(const uint& keyA, const uint& keyB)
        {
            return (keyA == keyB);
        }

        // Compute hash of key
        static uint HashFunc(const uint& key)
        {
            return FmComputeHash(key);
        }

        // Initialize value when key/value pair inserted
        static void InitValue(ExampleElement& elem)
        {
            elem.value = 0.0f;
        }
    };
    */

    // Template for basic hash set, only supporting insertions.
    template< class KeyValue, class SetFuncs >
    struct FmHashSet
    {
        KeyValue* elements;
        uint numElements;
        uint maxElements;
    };

    // Init hash set with provided memory for elements and initialize keys to invalid.
    template< class KeyValue, class SetFuncs >
    void FmInitHashSet(FmHashSet<KeyValue, SetFuncs>* hashSet, typename KeyValue* pElements, uint maxElements)
    {
        hashSet->elements = pElements;
        hashSet->maxElements = maxElements;
        hashSet->numElements = 0;

        for (uint i = 0; i < maxElements; i++)
        {
            SetFuncs::SetInvalidKey(hashSet->elements[i].key);
        }
    }

    // Return slot where key found, or may be inserted.
    template< class KeyValue, class SetFuncs, class Key >
    uint FmFindElementSlot(const FmHashSet<KeyValue, SetFuncs>& hashSet, const Key& key)
    {
        uint maxElements = hashSet.maxElements;
        uint idx = SetFuncs::HashFunc(key) % maxElements;

        FM_ASSERT(hashSet.numElements < maxElements);

        uint steps = 0;
        while (
            SetFuncs::IsValidKey(hashSet.elements[idx].key) &&
            !SetFuncs::KeysEqual(hashSet.elements[idx].key, key) &&
            steps < maxElements)
        {
            idx = (idx + 1) % maxElements;
            steps++;
        }

        return idx;
    }

    // Insert element and return pointer; set whether key was found in set.
    // NULL element signifies set full.
    template< class KeyValue, class SetFuncs, class Key >
    KeyValue* FmInsertElement(bool* foundInSet, FmHashSet<KeyValue, SetFuncs>* hashSet, const Key& key)
    {
        FM_ASSERT(hashSet->numElements <= hashSet->maxElements);

        // Find existing or new slot
        uint elemIdx = FmFindElementSlot(*hashSet, key);

        KeyValue& foundElement = hashSet->elements[elemIdx];

        if (SetFuncs::KeysEqual(foundElement.key, key))
        {
            // Found element, return
            *foundInSet = true;
            return &foundElement;
        }
        else if (!SetFuncs::IsValidKey(foundElement.key))
        {
            // Not in set, init new element
            hashSet->numElements++;

            foundElement.key = key;
            SetFuncs::InitValue(foundElement);
            *foundInSet = false;
            return &foundElement;
        }
        else
        {
            // Hash table full, set element to NULL
            *foundInSet = false;
            return NULL;
        }
    }

    // Hash set with max size that can be increased, by adding more tables rather than resizing.
    // Tables share same array of elements, so can iterate through array to find all entries.
    // Avoids rebuilding but lookups more expensive.
    template< class KeyValue, class SetFuncs >
    struct FmExpandableHashSet
    {
        FmHashSet< KeyValue, SetFuncs >* sets;
        uint                             numSets;
        uint                             maxSets;

        KeyValue* elements;
        uint      numElementsUsed;
        uint      maxElements;
    };

    // Insert element
    // slotIdx is index in set elements array.
    // Returns false if load greater than half.
    template< class KeyValue, class SetFuncs >
    bool FmInsertElement(FmHashSet< KeyValue, SetFuncs >* set, uint slotIdx, KeyValue& element)
    {
        if (set->numElements >= set->maxElements / 2)
        {
            return false;
        }

        set->elements[slotIdx] = element;
        set->numElements++;
        return true;
    }

    template< class KeyValue, class SetFuncs >
    KeyValue* FmInsertElement(FmExpandableHashSet< KeyValue, SetFuncs > * expandableSet, uint key)
    {
        FM_ASSERT(expandableSet->numElementsUsed <= expandableSet->maxElements);

        uint numSets = expandableSet->numSets;

        // Search through sets for key
        uint elemIdx = FM_INVALID_ID;
        for (uint setIdx = 0; setIdx < numSets; setIdx++)
        {
            FmHashSet< KeyValue, SetFuncs >& set = expandableSet->sets[setIdx];

            elemIdx = FmFindElementSlot(set, key);

            if (set.elements[elemIdx].key == key)
            {
                return &set.elements[elemIdx];
            }
        }

        // Not found, insert new element in the last set
        KeyValue element;
        element.key = key;
        SetFuncs::InitValue(element);

        FmHashSet< KeyValue, SetFuncs >& lastSet = expandableSet->sets[numSets - 1];
        if (FmInsertElement(&lastSet, elemIdx, element))
        {
            return &lastSet.elements[elemIdx];
        }

        // Load too great, create new set
        if (expandableSet->numSets >= expandableSet->maxSets)
        {
            FM_ASSERT(0);
            return NULL;
        }

        uint numElements = FmMinUint(lastSet.maxElements * 2, expandableSet->maxElements - expandableSet->numElementsUsed);

        FmHashSet< KeyValue, SetFuncs >& newSet = expandableSet->sets[numSets];
        FmInitHashSet(&newSet, &expandableSet->elements[expandableSet->numElementsUsed], numElements);

        expandableSet->numElementsUsed += numElements;
        expandableSet->numSets++;

        // Insert element in new set
        elemIdx = FmFindElementSlot(newSet, key);
        if (FmInsertElement(&newSet, elemIdx, element))
        {
            return &newSet.elements[elemIdx];
        }
        else
        {
            FM_ASSERT(0);
            return NULL;
        }
    }

    template< class KeyValue, class SetFuncs >
    uint FmGetNumEntries(const FmExpandableHashSet< KeyValue, SetFuncs >& expandableSet)
    {
        uint numSets = expandableSet.numSets;
        uint totalEntries = 0;
        for (uint setIdx = 0; setIdx < numSets; setIdx++)
        {
            const FmHashSet< KeyValue, SetFuncs >& set = expandableSet.sets[setIdx];
            totalEntries += set.numElements;
        }
        return totalEntries;
    }

    static inline void FmGetExpandableHashSetSizes(uint* outMaxElements, uint* outMaxSets, uint numElements)
    {
        uint maxElements = numElements * 2;
        *outMaxElements = maxElements;
        *outMaxSets = FmIntLog2(maxElements) + 1;
    }

    template< class KeyValue, class SetFuncs >
    size_t FmGetExpandableHashSetNumBytes(uint numElements)
    {
        uint maxElements, maxSets;
        FmGetExpandableHashSetSizes(&maxElements, &maxSets, numElements);

        return sizeof(FmExpandableHashSet) + sizeof(KeyValue) * maxElements + sizeof(FmHashSet< KeyValue, SetFuncs >) * maxSets;
    }


}
