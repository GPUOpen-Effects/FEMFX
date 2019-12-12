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

#include <math.h>

// Basic drop-in file for viewer camera parameters and control.
// Supports Orbit and Fly modes
// Pass in input values as deltas or current values.  Current values are used relative to a base value.

class ViewerCamera
{
private:
    // Some included math to avoid dependencies
    struct Vector3
    {
        float x, y, z;
        inline Vector3() {}
        inline Vector3(float inX, float inY, float inZ) { x = inX; y = inY; z = inZ; }
        inline Vector3 operator + (const Vector3& otherVec) { return Vector3(x + otherVec.x, y + otherVec.y, z + otherVec.z); }
        inline Vector3 operator - (const Vector3& otherVec) { return Vector3(x - otherVec.x, y - otherVec.y, z - otherVec.z); }
        inline Vector3 operator - () { return Vector3(-x, -y, -z); }
        inline Vector3 operator * (float scale) { return Vector3(x*scale, y*scale, z*scale); }
    };
    inline float squaredLength(const Vector3& vec) { return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z; }
    inline Vector3 normalize(const Vector3& vec)
    {
        float lengthSquared, lengthInverse;
        lengthSquared = squaredLength(vec);
        lengthInverse = 1.0f / sqrtf(lengthSquared);
        return Vector3(vec.x * lengthInverse, vec.y * lengthInverse, vec.z * lengthInverse);
    }
    inline Vector3 cross(const Vector3& vecA, const Vector3& vecB)
    {
        return Vector3(
            vecA.y * vecB.z - vecA.z * vecB.y,
            vecA.z * vecB.x - vecA.x * vecB.z,
            vecA.x * vecB.y - vecA.y * vecB.x);
    }
    inline float dot(const Vector3& vecA, const Vector3& vecB)
    {
        return vecA.x * vecB.x + vecA.y * vecB.y + vecA.z * vecB.z;
    }
    struct Matrix3
    {
        Vector3 col0, col1, col2;
        inline Matrix3() {}
        inline Matrix3(const Vector3& inCol0, const Vector3& inCol1, const Vector3& inCol2)
        {
            col0 = inCol0;
            col1 = inCol1;
            col2 = inCol2;
        }
        inline Vector3 operator * (const Vector3& vec) { return col0*vec.x + col1*vec.y + col2*vec.z; }
        static inline Matrix3 rotation(const Vector3& unitAxis, float radians)
        {
            float x = unitAxis.x;
            float y = unitAxis.y;
            float z = unitAxis.z;
            float xy = x * y;
            float yz = y * z;
            float zx = z * x;
            float sn = sinf(radians);
            float cs = cosf(radians);
            float onesubcs = (1.0f - cs);
            return Matrix3(
                Vector3(x*x*onesubcs + cs, xy*onesubcs + z*sn, zx*onesubcs - y*sn),
                Vector3(xy*onesubcs - z*sn, y*y*onesubcs + cs, yz*onesubcs + x*sn),
                Vector3(zx*onesubcs + y*sn, yz*onesubcs - x*sn, z*z*onesubcs + cs));
        }
    };
    inline Matrix3 transpose(const Matrix3& mat)
    {
        return Matrix3(
            Vector3(mat.col0.x, mat.col1.x, mat.col2.x),
            Vector3(mat.col0.y, mat.col1.y, mat.col2.y),
            Vector3(mat.col0.z, mat.col1.z, mat.col2.z)
        );
    }

public:
    Vector3 mLookAtPos;    // Center of orbit rotation
    Vector3 mUpDir;        // Up direction for azimuth rotation
    Vector3 mBackDir;      // Direction to eye when azimuth and elevation zero

    float   mAzimuth;        // Azimuth, elevation, radius of orbit
    float   mElevation;
    float   mRadius;

    Matrix3 mViewRotation;   // Updated from above params
    Vector3 mViewPosition;   // (eye pos)

    // Scale factors on input
    float mAzimuthInputScale;
    float mElevationInputScale;
    float mRadiusInputScale;
    float mRadiusExpansion;
    float mLookAtXInputScale;  // x, y, z are local to the current view
    float mLookAtYInputScale;
    float mLookAtZInputScale;

    // Values saved for relative input
    float mAzimuthStartValue;
    float mAzimuthStartInput;
    float mElevationStartValue;
    float mElevationStartInput;
    float mRadiusStartValue;
    float mRadiusStartInput;
    Vector3 mLookAtPosStart;
    float mLookAtXStartInput;
    float mLookAtYStartInput;
    float mLookAtZStartInput;

    inline ViewerCamera()
    {
        mLookAtPos = Vector3(0.0f, 0.0f, 0.0f);
        mUpDir = Vector3(0.0f, 1.0f, 0.0f);
        mBackDir = Vector3(0.0f, 0.0f, 1.0f);

        mAzimuth = 0.0f;
        mElevation = 0.0f;
        mRadius = 1.0f;

        UpdateViewTransform();

        mAzimuthInputScale = 1.0f;
        mElevationInputScale = 1.0f;
        mRadiusInputScale = 1.0f;
        mRadiusExpansion = 1.1f;

        mLookAtXInputScale = 1.0f;
        mLookAtYInputScale = 1.0f;
        mLookAtZInputScale = 1.0f;

        mAzimuthStartValue = 0.0f;
        mAzimuthStartInput = 0.0f;

        mElevationStartValue = 0.0f;
        mElevationStartInput = 0.0f;

        mRadiusStartValue = 0.0f;
        mRadiusStartInput = 0.0f;

        mLookAtPosStart = Vector3(0.0f, 0.0f, 0.0f);
        mLookAtXStartInput = 0.0f;
        mLookAtYStartInput = 0.0f;
        mLookAtZStartInput = 0.0f;
    }

    // Get current parameters
    inline float GetAzimuth() const { return mAzimuth; }
    inline float GetElevation() const { return mElevation; }
    inline float GetRadius() const { return mRadius; }
    inline float GetLookAtX() const { return mLookAtPos.x; }
    inline float GetLookAtY() const { return mLookAtPos.y; }
    inline float GetLookAtZ() const { return mLookAtPos.z; }
    inline float GetUpDirX() const { return mUpDir.x; }
    inline float GetUpDirY() const { return mUpDir.y; }
    inline float GetUpDirZ() const { return mUpDir.z; }

    // Set current parameters
    inline void SetAzimuth(float inValue) { mAzimuth = inValue; }
    inline void SetElevation(float inValue) { mElevation = inValue; }
    inline void SetRadius(float inValue) { mRadius = inValue; }
    inline void SetLookAtX(float inValue) { mLookAtPos.x = inValue; }
    inline void SetLookAtY(float inValue) { mLookAtPos.y = inValue; }
    inline void SetLookAtZ(float inValue) { mLookAtPos.z = inValue; }
    inline void SetUpDirX(float inValue) { mUpDir.x = inValue; }
    inline void SetUpDirY(float inValue) { mUpDir.y = inValue; }
    inline void SetUpDirZ(float inValue) { mUpDir.z = inValue; }

    inline void SetAzimuthInputScale(float inputScale) { mAzimuthInputScale = inputScale; }
    inline void SetElevationInputScale(float inputScale) { mElevationInputScale = inputScale; }
    inline void SetRadiusInputScale(float inputScale) { mRadiusInputScale = inputScale; }
    inline void SetRadiusExpansion(float expansion) { mRadiusExpansion = expansion; }
    inline void SetLookAtXInputScale(float inputScale) { mLookAtXInputScale = inputScale; }
    inline void SetLookAtYInputScale(float inputScale) { mLookAtYInputScale = inputScale; }
    inline void SetLookAtZInputScale(float inputScale) { mLookAtZInputScale = inputScale; }

    inline void AzimuthDragStart(float controllerInput) { mAzimuthStartInput = controllerInput;  mAzimuthStartValue = mAzimuth; }
    inline void AzimuthDragCurrent(float controllerInput) { mAzimuth = mAzimuthStartValue + (controllerInput - mAzimuthStartInput)*mAzimuthInputScale; }
    inline void AzimuthDelta(float delta) { mAzimuth += delta * mAzimuthInputScale; }

    inline void ElevationDragStart(float controllerInput) { mElevationStartInput = controllerInput;  mElevationStartValue = mElevation; }
    inline void ElevationDragCurrent(float controllerInput) { mElevation = mElevationStartValue + (controllerInput - mElevationStartInput)*mElevationInputScale; }
    inline void ElevationDelta(float delta) { mElevation += delta * mElevationInputScale; }

    inline void RadiusDragStart(float controllerInput) { mRadiusStartInput = controllerInput;  mRadiusStartValue = mRadius; }
    inline void RadiusDragCurrent(float controllerInput) { mRadius = mRadiusStartValue * powf(mRadiusExpansion, (controllerInput - mRadiusStartInput)*mRadiusInputScale); }
    inline void RadiusDelta(float delta) { mRadius = mRadius * powf(mRadiusExpansion, delta * mRadiusInputScale); }
    //inline void RadiusDragCurrent(float controllerInput) { mRadius = mRadiusStartValue + (controllerInput - mRadiusStartInput)*mRadiusInputScale; }
    //inline void RadiusDelta(float delta) { mRadius += delta * mRadiusInputScale; }

    inline void LookAtDragStart(float inputX, float inputY, float inputZ)
    {
        mLookAtPosStart = mLookAtPos;
        mLookAtXStartInput = inputX;
        mLookAtYStartInput = inputY;
        mLookAtZStartInput = inputZ;
    }
    inline void LookAtDragCurrent(float inputX, float inputY, float inputZ)
    {
        mLookAtPos = mLookAtPosStart +
            mViewRotation.col0 * (inputX - mLookAtXStartInput)*mLookAtXInputScale +
            mViewRotation.col1 * (inputY - mLookAtYStartInput)*mLookAtYInputScale +
            mViewRotation.col2 * (inputZ - mLookAtZStartInput)*mLookAtZInputScale;
    }

    inline void LookAtDelta(float deltaX, float deltaY, float deltaZ)
    {
        mLookAtPos = mLookAtPos +
            mViewRotation.col0 * deltaX * mLookAtXInputScale +
            mViewRotation.col1 * deltaY * mLookAtYInputScale +
            mViewRotation.col2 * deltaZ * mLookAtZInputScale;
    }

public:
    inline void UpdateViewTransform()
    {
        Vector3 yVector = normalize(mUpDir);
        Vector3 xVector = cross(mUpDir, mBackDir);
        Vector3 zVector = normalize(cross(xVector, yVector));
        xVector = cross(yVector, zVector);

        Matrix3 azimuthRotation = Matrix3::rotation(yVector, mAzimuth);
        xVector = azimuthRotation * xVector;
        zVector = azimuthRotation * zVector;

        Matrix3 elevationRotation = Matrix3::rotation(xVector, mElevation);
        yVector = elevationRotation * yVector;
        zVector = elevationRotation * zVector;

        mViewRotation.col0 = xVector;
        mViewRotation.col1 = yVector;
        mViewRotation.col2 = zVector;
        mViewPosition = mLookAtPos + zVector * mRadius;
    }

    // Produces view matrix data compatible with both DirectX or OpenGL conventions
    inline void GetViewMatrix(float* viewMatrix)
    {
        UpdateViewTransform();

        Matrix3 worldToViewRotation = transpose(mViewRotation);
        Vector3 worldToViewTranslation = -(worldToViewRotation*mViewPosition);

        viewMatrix[0] = worldToViewRotation.col0.x;
        viewMatrix[1] = worldToViewRotation.col0.y;
        viewMatrix[2] = worldToViewRotation.col0.z;
        viewMatrix[3] = 0.0f;
        viewMatrix[4] = worldToViewRotation.col1.x;
        viewMatrix[5] = worldToViewRotation.col1.y;
        viewMatrix[6] = worldToViewRotation.col1.z;
        viewMatrix[7] = 0.0f;
        viewMatrix[8] = worldToViewRotation.col2.x;
        viewMatrix[9] = worldToViewRotation.col2.y;
        viewMatrix[10] = worldToViewRotation.col2.z;
        viewMatrix[11] = 0.0f;
        viewMatrix[12] = worldToViewTranslation.x;
        viewMatrix[13] = worldToViewTranslation.y;
        viewMatrix[14] = worldToViewTranslation.z;
        viewMatrix[15] = 1.0f;
    }

    // Produces view matrix data compatible with both DirectX or OpenGL conventions
    inline void GetViewMatrixTranspose(float* viewMatrixT)
    {
        float viewMatrix[16];
        GetViewMatrix(viewMatrix);

        viewMatrixT[0] = viewMatrix[0];
        viewMatrixT[1] = viewMatrix[4];
        viewMatrixT[2] = viewMatrix[8];
        viewMatrixT[3] = viewMatrix[12];
        viewMatrixT[4] = viewMatrix[1];
        viewMatrixT[5] = viewMatrix[5];
        viewMatrixT[6] = viewMatrix[9];
        viewMatrixT[7] = viewMatrix[13];
        viewMatrixT[8] = viewMatrix[2];
        viewMatrixT[9] = viewMatrix[6];
        viewMatrixT[10] = viewMatrix[10];
        viewMatrixT[11] = viewMatrix[14];
        viewMatrixT[12] = viewMatrix[3];
        viewMatrixT[13] = viewMatrix[7];
        viewMatrixT[14] = viewMatrix[11];
        viewMatrixT[15] = viewMatrix[15];
    }

    // Produces projection matrix data compatible with both DirectX or OpenGL conventions
    inline void GetProjectionMatrix(float *projectionMatrix, float fovyRadians, float aspect, float zNear, float zFar)
    {
        static const float halfPi = 1.570796327f;
        float f = tanf(halfPi - 0.5f*fovyRadians);
        float rangeInverse = 1.0f / (zNear - zFar);
        projectionMatrix[0] = f / aspect;
        projectionMatrix[1] = 0.0f;
        projectionMatrix[2] = 0.0f;
        projectionMatrix[3] = 0.0f;
        projectionMatrix[4] = 0.0f;
        projectionMatrix[5] = f;
        projectionMatrix[6] = 0.0f;
        projectionMatrix[7] = 0.0f;
        projectionMatrix[8] = 0.0f;
        projectionMatrix[9] = 0.0f;
        projectionMatrix[10] = (zNear + zFar)*rangeInverse;
        projectionMatrix[11] = -1.0f;
        projectionMatrix[12] = 0.0f;
        projectionMatrix[13] = 0.0f;
        projectionMatrix[14] = zNear*zFar*rangeInverse*2.0f;
        projectionMatrix[15] = 0.0f;
    }
};
