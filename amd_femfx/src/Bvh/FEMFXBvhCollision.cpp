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

#include "FEMFXBvhCollision.h"
#include "FEMFXTriCcd.h"

namespace AMD
{
    void FmBvhSelfCcdRecursive(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmBvh& bvh, int nodeIndexA, int nodeIndexB, float timestep)
    {
        const FmBvhNode& nodeA = bvh.nodes[nodeIndexA];
        const FmBvhNode& nodeB = bvh.nodes[nodeIndexB];
        const FmAabb& boxA = bvh.nodes[nodeIndexA].box;
        const FmAabb& boxB = bvh.nodes[nodeIndexB].box;
        bool nodeAIsLeaf = nodeA.left == nodeA.right;
        bool nodeBIsLeaf = nodeB.left == nodeB.right;
        float impactTime;

        if ((nodeIndexA == nodeIndexB) || FmAabbCcd(impactTime, boxA, boxB, timestep))
        {
            if (nodeAIsLeaf && nodeBIsLeaf)
            {               
                leafPairCallback(userData, (uint)nodeA.left, (uint)nodeB.left);
            }
            else if (nodeAIsLeaf)
            {
                FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeIndexA, nodeB.left, timestep);
                FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeIndexA, nodeB.right, timestep);
            }
            else if (nodeBIsLeaf)
            {
                FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.left, nodeIndexB, timestep);
                FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.right, nodeIndexB, timestep);
            }
            else
            {
                if (nodeIndexA == nodeIndexB)
                {
                    // If the same tree node, test LL, LR, and RR pairs (not RL) to find unique pairs
                    FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.left, nodeB.left, timestep);
                    FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.left, nodeB.right, timestep);
                    FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.right, nodeB.right, timestep);
                }
                else
                {
                    // If different tree nodes, use normal binary recursion
                    float sizeA = lengthSqr(boxA.pmax - boxA.pmin);
                    float sizeB = lengthSqr(boxB.pmax - boxB.pmin);

                    // Split the larger node
                    if (sizeA > sizeB)
                    {
                        FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.left, nodeIndexB, timestep);
                        FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeA.right, nodeIndexB, timestep);
                    }
                    else
                    {
                        FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeIndexA, nodeB.left, timestep);
                        FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, nodeIndexA, nodeB.right, timestep);
                    }
                }
            }
        }
    }

    void FmBvhSelfCcd(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmBvh& bvh, float timestep)
    {
        if (bvh.numPrims > 0)
        {
            FmBvhSelfCcdRecursive(leafPairCallback, userData, bvh, 0, 0, timestep);
        }
    }

    void FmBvhPairCcdRecursive(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmBvh& bvhA, const FmBvh& bvhB, int nodeIndexA, int nodeIndexB, float timestep)
    {
        const FmBvhNode& nodeA = bvhA.nodes[nodeIndexA];
        const FmBvhNode& nodeB = bvhB.nodes[nodeIndexB];
        const FmAabb& boxA = bvhA.nodes[nodeIndexA].box;
        const FmAabb& boxB = bvhB.nodes[nodeIndexB].box;
        bool nodeAIsLeaf = nodeA.left == nodeA.right;
        bool nodeBIsLeaf = nodeB.left == nodeB.right;
        float impactTime;

        if (FmAabbCcd(impactTime, boxA, boxB, timestep))
        {
            if (nodeAIsLeaf && nodeBIsLeaf)
            {
                leafPairCallback(userData, (uint)nodeA.left, (uint)nodeB.left);
            }
            else if (nodeAIsLeaf)
            {
                FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeIndexA, nodeB.left, timestep);
                FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeIndexA, nodeB.right, timestep);
            }
            else if (nodeBIsLeaf)
            {
                FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeA.left, nodeIndexB, timestep);
                FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeA.right, nodeIndexB, timestep);
            }
            else
            {
                float sizeA = lengthSqr(boxA.pmax - boxA.pmin);
                float sizeB = lengthSqr(boxB.pmax - boxB.pmin);

                // Split the larger node
                if (sizeA > sizeB)
                {
                    FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeA.left, nodeIndexB, timestep);
                    FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeA.right, nodeIndexB, timestep);
                }
                else
                {
                    FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeIndexA, nodeB.left, timestep);
                    FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, nodeIndexA, nodeB.right, timestep);
                }
            }
        }
    }

    void FmBvhPairCcd(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmBvh& bvhA, const FmBvh& bvhB, float timestep)
    {
        if (bvhA.numPrims > 0 && bvhB.numPrims > 0)
        {
            FmBvhPairCcdRecursive(leafPairCallback, userData, bvhA, bvhB, 0, 0, timestep);
        }
    }

    void FmObjectBvhCcdRecursive(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmAabb& objectBvA, int objectIdA, const FmBvh& bvh, int nodeIndexB, float timestep)
    {
        const FmBvhNode& nodeB = bvh.nodes[nodeIndexB];
        const FmAabb& boxA = objectBvA;
        const FmAabb& boxB = bvh.nodes[nodeIndexB].box;
        bool nodeBIsLeaf = nodeB.left == nodeB.right;
        float impactTime;

        if (FmAabbCcd(impactTime, boxA, boxB, timestep))
        {
            if (nodeBIsLeaf)
            {
                uint objectIdB = (uint)nodeB.left;
                leafPairCallback(userData, objectIdA, objectIdB);
            }
            else
            {
                // Recurse on children
                FmObjectBvhCcdRecursive(leafPairCallback, userData, objectBvA, objectIdA, bvh, nodeB.left, timestep);
                FmObjectBvhCcdRecursive(leafPairCallback, userData, objectBvA, objectIdA, bvh, nodeB.right, timestep);
            }
        }
    }

    void FmObjectBvhCcd(FmBvhLeafPairCallback leafPairCallback, void* userData, const FmAabb& objectBvA, uint objectIdA, const FmBvh& bvh, float timestep)
    {
        if (bvh.numPrims > 0)
        {
            FmObjectBvhCcdRecursive(leafPairCallback, userData, objectBvA, objectIdA, bvh, 0, timestep);
        }
    }
}
