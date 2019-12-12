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
// Example code for applying an explosion pressure wave to FEM object
//---------------------------------------------------------------------------------------

#include "ExplosionForce.h"

namespace AMD
{

void ApplyExplosionForce(FmScene* scene, FmTetMesh* tetMesh, 
    const FmVector3& origin, float shockwavePressure0, float shockwaveArea0,
    float speed, float timestep, float timeSinceDetonated)
{
    float innerRadius = speed * timeSinceDetonated;
    float outerRadius = speed * (timeSinceDetonated + timestep);

    float minDist = sqrtf(shockwaveArea0 / (4.0f * 3.14159265f));

    uint numExteriorFaces = FmGetNumExteriorFaces(*tetMesh);
    for (uint extFaceId = 0; extFaceId < numExteriorFaces; extFaceId++)
    {
        uint tetId, faceId;
        FmGetExteriorFace(&tetId, &faceId, *tetMesh, extFaceId);

        FmTetVertIds tetVertIds = FmGetTetVertIds(*tetMesh, tetId);
        FmFaceVertIds faceVertIds;
        FmGetFaceVertIds(&faceVertIds, faceId, tetVertIds);

        FmVector3 pos0 = FmGetVertPosition(*tetMesh, faceVertIds.ids[0]);
        FmVector3 pos1 = FmGetVertPosition(*tetMesh, faceVertIds.ids[1]);
        FmVector3 pos2 = FmGetVertPosition(*tetMesh, faceVertIds.ids[2]);

        FmVector3 faceCenter = (pos0 + pos1 + pos2) / 3.0f;

        float lenSqr;
        FmVector3 explosionDir = FmNormalize(&lenSqr, faceCenter - origin);
        float distance = sqrtf(lenSqr);

        FmVector3 faceDir = normalize(cross(pos1 - pos0, pos2 - pos0));

        if (distance >= innerRadius && distance < outerRadius && dot(explosionDir, faceDir) < 0.0f)
        {
            FmVector3 xDir = normalize(FmOrthogonalVector(explosionDir));
            FmVector3 yDir = cross(xDir, explosionDir);

            pos0 -= faceCenter;
            pos1 -= faceCenter;
            pos2 -= faceCenter;
            float p0x = dot(pos0, xDir);
            float p0y = dot(pos0, yDir);
            float p1x = dot(pos1, xDir);
            float p1y = dot(pos1, yDir);
            float p2x = dot(pos2, xDir);
            float p2y = dot(pos2, yDir);
            float ax = p1x - p0x;
            float ay = p1y - p0y;
            float bx = p2x - p0x;
            float by = p2y - p0y;
            float faceArea = ax * by - ay * bx;

            float falloff;
            if (distance < minDist)
            {
                falloff = 1.0f;
            }
            else
            {
                float shockwaveArea = 4.0f * 3.14159265f * distance * distance;
                falloff = shockwaveArea0 / shockwaveArea;
            }

            float pressure = shockwavePressure0 * falloff;
            FmVector3 vertForce = explosionDir * pressure * faceArea * (1.0f / 3.0f);
            //FmVector3 vertForce = -faceDir * pressure * faceArea * (1.0f / 3.0f);

            FmAddForceToVert(scene, tetMesh, faceVertIds.ids[0], vertForce);
            FmAddForceToVert(scene, tetMesh, faceVertIds.ids[1], vertForce);
            FmAddForceToVert(scene, tetMesh, faceVertIds.ids[2], vertForce);
        }
    }
}

}
