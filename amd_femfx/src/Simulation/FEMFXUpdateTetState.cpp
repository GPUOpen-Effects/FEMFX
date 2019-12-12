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

#include "AMD_FEMFX.h"
#include "FEMFXAsyncThreading.h"
#include "FEMFXParallelFor.h"
#include "FEMFXSoaTetMath.h"
#include "FEMFXSoaSvd3x3.h"
#include "FEMFXScene.h"
#include "FEMFXUpdateTetState.h"

#define FM_UPDATE_TET_BATCH_SIZE 64

namespace AMD
{
    // Input data required for updating tet state: tet rotation, stiffness, plasticity, and testing fracture
    struct FmUpdateTetStateInput
    {
        // Deformed tet positions; translated by center of mass for numerical benefit
        FmVector3 deformedPosition0;
        FmVector3 deformedPosition1;
        FmVector3 deformedPosition2;
        FmVector3 deformedPosition3;

        // Rest tet positions
        FmVector3 restPosition0;
        FmVector3 restPosition1;
        FmVector3 restPosition2;
        FmVector3 restPosition3;

        // Rest tet positions used for stress calculation; from original tet mesh or virtual rest positions after plastic deformation
        FmVector3 stressRestPosition0;
        FmVector3 stressRestPosition1;
        FmVector3 stressRestPosition2;
        FmVector3 stressRestPosition3;

        FmTetShapeParams shapeParams;         // Shape params based on original rest positions
        FmTetShapeParams stressShapeParams;   // Shape params based on plastic deformed rest positions, for computing elastic stress

        // Matrices from plasticity state
        FmMatrix3 plasticDeformationMatrix;
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
        FmMatrix3 plasticTetRelRotation;
#endif

        // Copied material parameters
        float        youngsModulus;
        float        poissonsRatio;
        float        fractureStressThreshold;
        float        plasticYieldThreshold;
        float        plasticCreep;
        float        plasticMin;
        float        plasticMax;
        bool         preserveVolume;

        bool         testFracture;       // Based on tet properties, set if need to test fracture
        bool         updatePlasticity;   // Based on tet properties, set if need to update plasticity
    };

    // Output tet state which must be stored into mesh
    struct FmUpdateTetStateOutput
    {
        FmMatrix3 tetRotation; // Tet rotation computed from original rest positions
        FmQuat    tetQuat;     // Rotation converted to quaternion for average at vertices

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        FmTetRotatedStiffnessMatrix rotatedStiffnessMat; // Tet stiffness matrix
#endif

        // Updated plasticity state
        FmMatrix3 plasticDeformationMatrix;
        FmTetShapeParams plasticShapeParams;
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
        FmMatrix3 plasticTetRelRotation;
#endif

        // Stress measure for fracture
        FmVector3 maxStressDirection;
        float maxStressEigenvalue;

        // Strain magnitude
        float strainMag;

        // Copied from input
        float fractureStressThreshold;
        bool testFracture;
        bool updatePlasticity;
    };

    static FM_FORCE_INLINE void FmAddToQuat(FmQuat* dst, const FmQuat& src)
    {
        if (dot(*dst, src) > 0.0f)
        {
            *dst += src;
        }
        else
        {
            *dst += -src;
        }
    }

#if FM_SOA_TET_MATH
    // Input data required for updating tet state: tet rotation, stiffness, plasticity, and testing fracture
    template<class T>
    struct FmSoaUpdateTetStateInput
    {
        // Deformed tet positions; translated by center of mass for numerical benefit
        typename T::SoaVector3 deformedPosition0;
        typename T::SoaVector3 deformedPosition1;
        typename T::SoaVector3 deformedPosition2;
        typename T::SoaVector3 deformedPosition3;

        // Rest tet positions
        typename T::SoaVector3 restPosition0;
        typename T::SoaVector3 restPosition1;
        typename T::SoaVector3 restPosition2;
        typename T::SoaVector3 restPosition3;

        // Rest tet positions used for stress calculation; from original tet mesh or virtual rest positions after plastic deformation
        typename T::SoaVector3 stressRestPosition0;
        typename T::SoaVector3 stressRestPosition1;
        typename T::SoaVector3 stressRestPosition2;
        typename T::SoaVector3 stressRestPosition3;

        FmSoaTetShapeParams<T> shapeParams;         // Shape params based on original rest positions
        FmSoaTetShapeParams<T> stressShapeParams;   // Shape params based on plastic deformed rest positions, for computing elastic stress

        // Matrices from plasticity state
        typename T::SoaMatrix3 plasticDeformationMatrix;
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
        typename T::SoaMatrix3 plasticTetRelRotation;
#endif

        // Copied material parameters
        typename T::SoaFloat youngsModulus;
        typename T::SoaFloat poissonsRatio;
        typename T::SoaFloat fractureStressThreshold;
        typename T::SoaFloat plasticYieldThreshold;
        typename T::SoaFloat plasticCreep;
        typename T::SoaFloat plasticMin;
        typename T::SoaFloat plasticMax;
        typename T::SoaBool  preserveVolume;

        typename T::SoaBool  testFracture;       // Based on tet properties, set if need to test fracture
        typename T::SoaBool  updatePlasticity;   // Based on tet properties, set if need to test/update plasticity
    };

    // Output tet state which must be stored into mesh
    template<class T>
    struct FmSoaUpdateTetStateOutput
    {
        typename T::SoaMatrix3 tetRotation; // Tet rotation computed from original rest positions

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        typename FmSoaTetRotatedStiffnessMatrix<T> rotatedStiffnessMat; // Tet stiffness matrix
#endif

        // Updated plasticity state
        typename T::SoaMatrix3 plasticDeformationMatrix;
        typename FmSoaTetShapeParams<T> plasticShapeParams;
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
        typename T::SoaMatrix3 plasticTetRelRotation;
#endif

        // Stress measure for fracture
        typename T::SoaVector3 maxStressDirection;
        typename T::SoaFloat maxStressEigenvalue;

        // Deformation value
        typename T::SoaFloat strainMag;

        // Copied from input
        typename T::SoaFloat fractureStressThreshold;
        typename T::SoaBool testFracture;
        typename T::SoaBool updatePlasticity;
    };

    template<class SoaTypes>
    struct FmUpdateTetStateBatch
    {
        uint baseTetIdx;
        uint numTets; // number of tets batched

        FmSoaUpdateTetStateInput<SoaTypes> input;
        FmSoaUpdateTetStateOutput<SoaTypes> output;

        FM_ALIGN(16) FmVector3 aosDeformedPosition0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosDeformedPosition1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosDeformedPosition2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosDeformedPosition3[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestPosition0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestPosition1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestPosition2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestPosition3[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestPosition0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestPosition1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestPosition2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestPosition3[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosShapeParamsBaryMatrixCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosShapeParamsBaryMatrixCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosShapeParamsBaryMatrixCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosShapeParamsBaryMatrixCol3[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector4 aosStressShapeParamsBaryMatrixCol3[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticDeformationMatrixCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticDeformationMatrixCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticDeformationMatrixCol2[SoaTypes::width] FM_ALIGN_END(16);
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
        FM_ALIGN(16) FmVector3 aosPlasticTetRelRotationCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticTetRelRotationCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticTetRelRotationCol2[SoaTypes::width] FM_ALIGN_END(16);
#endif

        FmUpdateTetStateBatch()
        {
            baseTetIdx = 0;
            numTets = 0;

            for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
            {
                aosDeformedPosition0[sliceIdx] = FmInitVector3(0.0f);
                aosDeformedPosition1[sliceIdx] = FmInitVector3(0.0f);
                aosDeformedPosition2[sliceIdx] = FmInitVector3(0.0f);
                aosDeformedPosition3[sliceIdx] = FmInitVector3(0.0f);
                aosRestPosition0[sliceIdx] = FmInitVector3(0.0f);
                aosRestPosition1[sliceIdx] = FmInitVector3(0.0f);
                aosRestPosition2[sliceIdx] = FmInitVector3(0.0f);
                aosRestPosition3[sliceIdx] = FmInitVector3(0.0f);
                aosStressRestPosition0[sliceIdx] = FmInitVector3(0.0f);
                aosStressRestPosition1[sliceIdx] = FmInitVector3(0.0f);
                aosStressRestPosition2[sliceIdx] = FmInitVector3(0.0f);
                aosStressRestPosition3[sliceIdx] = FmInitVector3(0.0f);
                aosShapeParamsBaryMatrixCol0[sliceIdx] = FmInitVector4(0.0f);
                aosShapeParamsBaryMatrixCol1[sliceIdx] = FmInitVector4(0.0f);
                aosShapeParamsBaryMatrixCol2[sliceIdx] = FmInitVector4(0.0f);
                aosShapeParamsBaryMatrixCol3[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol0[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol1[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol2[sliceIdx] = FmInitVector4(0.0f);
                aosStressShapeParamsBaryMatrixCol3[sliceIdx] = FmInitVector4(0.0f);
                aosPlasticDeformationMatrixCol0[sliceIdx] = FmInitVector3(0.0f);
                aosPlasticDeformationMatrixCol1[sliceIdx] = FmInitVector3(0.0f);
                aosPlasticDeformationMatrixCol2[sliceIdx] = FmInitVector3(0.0f);
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
                aosPlasticTetRelRotationCol0[sliceIdx] = FmInitVector3(0.0f);
                aosPlasticTetRelRotationCol1[sliceIdx] = FmInitVector3(0.0f);
                aosPlasticTetRelRotationCol2[sliceIdx] = FmInitVector3(0.0f);
#endif
            }

            input.shapeParams.det = SoaTypes::SoaFloat(0.0f);
            input.stressShapeParams.det = SoaTypes::SoaFloat(0.0f);
            input.youngsModulus = SoaTypes::SoaFloat(0.0f);
            input.poissonsRatio = SoaTypes::SoaFloat(0.0f);
            input.fractureStressThreshold = SoaTypes::SoaFloat(0.0f);
            input.plasticYieldThreshold = SoaTypes::SoaFloat(0.0f);
            input.plasticCreep = SoaTypes::SoaFloat(0.0f);
            input.plasticMin = SoaTypes::SoaFloat(0.0f);
            input.plasticMax = SoaTypes::SoaFloat(0.0f);
            input.preserveVolume = SoaTypes::SoaFloat(0.0f);
            input.testFracture = SoaTypes::SoaFloat(0.0f);
            input.updatePlasticity = SoaTypes::SoaFloat(0.0f);
        }

        void SetBatchSlice(
            uint idx,
            const FmUpdateTetStateInput& inputSlice,
            bool meshSupportsPlasticity)
        {
            aosDeformedPosition0[idx] = inputSlice.deformedPosition0;
            aosDeformedPosition1[idx] = inputSlice.deformedPosition1;
            aosDeformedPosition2[idx] = inputSlice.deformedPosition2;
            aosDeformedPosition3[idx] = inputSlice.deformedPosition3;
            aosRestPosition0[idx] = inputSlice.restPosition0;
            aosRestPosition1[idx] = inputSlice.restPosition1;
            aosRestPosition2[idx] = inputSlice.restPosition2;
            aosRestPosition3[idx] = inputSlice.restPosition3;
            aosStressRestPosition0[idx] = inputSlice.stressRestPosition0;
            aosStressRestPosition1[idx] = inputSlice.stressRestPosition1;
            aosStressRestPosition2[idx] = inputSlice.stressRestPosition2;
            aosStressRestPosition3[idx] = inputSlice.stressRestPosition3;
            aosShapeParamsBaryMatrixCol0[idx] = inputSlice.shapeParams.baryMatrix.col0;
            aosShapeParamsBaryMatrixCol1[idx] = inputSlice.shapeParams.baryMatrix.col1;
            aosShapeParamsBaryMatrixCol2[idx] = inputSlice.shapeParams.baryMatrix.col2;
            aosShapeParamsBaryMatrixCol3[idx] = inputSlice.shapeParams.baryMatrix.col3;
            aosStressShapeParamsBaryMatrixCol0[idx] = inputSlice.stressShapeParams.baryMatrix.col0;
            aosStressShapeParamsBaryMatrixCol1[idx] = inputSlice.stressShapeParams.baryMatrix.col1;
            aosStressShapeParamsBaryMatrixCol2[idx] = inputSlice.stressShapeParams.baryMatrix.col2;
            aosStressShapeParamsBaryMatrixCol3[idx] = inputSlice.stressShapeParams.baryMatrix.col3;

            input.shapeParams.det.setSlice(idx, inputSlice.shapeParams.det);
            input.stressShapeParams.det.setSlice(idx, inputSlice.stressShapeParams.det);

            if (meshSupportsPlasticity)
            {
                aosPlasticDeformationMatrixCol0[idx] = inputSlice.plasticDeformationMatrix.col0;
                aosPlasticDeformationMatrixCol1[idx] = inputSlice.plasticDeformationMatrix.col1;
                aosPlasticDeformationMatrixCol2[idx] = inputSlice.plasticDeformationMatrix.col2;
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
                aosPlasticTetRelRotationCol0[idx] = inputSlice.plasticTetRelRotation.col0;
                aosPlasticTetRelRotationCol1[idx] = inputSlice.plasticTetRelRotation.col1;
                aosPlasticTetRelRotationCol2[idx] = inputSlice.plasticTetRelRotation.col2;
#endif
            }

            input.youngsModulus.setSlice(idx, inputSlice.youngsModulus);
            input.poissonsRatio.setSlice(idx, inputSlice.poissonsRatio);

            if (inputSlice.testFracture)
            {
                input.fractureStressThreshold.setSlice(idx, inputSlice.fractureStressThreshold);
            }

            if (inputSlice.updatePlasticity)
            {
                input.plasticYieldThreshold.setSlice(idx, inputSlice.plasticYieldThreshold);
                input.plasticCreep.setSlice(idx, inputSlice.plasticCreep);
                input.plasticMin.setSlice(idx, inputSlice.plasticMin);
                input.plasticMax.setSlice(idx, inputSlice.plasticMax);
                input.preserveVolume.setSlice(idx, inputSlice.preserveVolume);
            }

            input.testFracture.setSlice(idx, inputSlice.testFracture);
            input.updatePlasticity.setSlice(idx, inputSlice.updatePlasticity);
        }

        void GetBatchSlice(
            FmUpdateTetStateOutput* outputSlice,
            uint idx,
            bool meshSupportsPlasticity,
            bool computingStrainMag)
        {
            outputSlice->tetRotation = FmGetSlice<SoaTypes>(output.tetRotation, idx);

#if !FM_MATRIX_ASSEMBLY_BY_TETS
            FmGetSlice<SoaTypes>(&outputSlice->rotatedStiffnessMat, output.rotatedStiffnessMat, idx);
#endif

            if (meshSupportsPlasticity)
            {
                outputSlice->plasticDeformationMatrix = FmGetSlice<SoaTypes>(output.plasticDeformationMatrix, idx);
                FmGetSlice<SoaTypes>(&outputSlice->plasticShapeParams, output.plasticShapeParams, idx);
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
                outputSlice->plasticTetRelRotation = FmGetSlice<SoaTypes>(output.plasticTetRelRotation, idx);
#endif
            }

            // Stress measure for fracture
            outputSlice->maxStressDirection = FmGetSlice<SoaTypes>(output.maxStressDirection, idx);
            outputSlice->maxStressEigenvalue = output.maxStressEigenvalue.getSlice(idx);

            if (computingStrainMag)
            {
                outputSlice->strainMag = output.strainMag.getSlice(idx);
            }

            outputSlice->fractureStressThreshold = output.fractureStressThreshold.getSlice(idx);
            outputSlice->testFracture = output.testFracture.getSlice(idx);
            outputSlice->updatePlasticity = output.updatePlasticity.getSlice(idx);
        }

        void ConvertAosToSoa(bool meshSupportsPlasticity)
        {
            FmSetSoaFromAlignedAos(&input.deformedPosition0, aosDeformedPosition0);
            FmSetSoaFromAlignedAos(&input.deformedPosition1, aosDeformedPosition1);
            FmSetSoaFromAlignedAos(&input.deformedPosition2, aosDeformedPosition2);
            FmSetSoaFromAlignedAos(&input.deformedPosition3, aosDeformedPosition3);

            FmSetSoaFromAlignedAos(&input.restPosition0, aosRestPosition0);
            FmSetSoaFromAlignedAos(&input.restPosition1, aosRestPosition1);
            FmSetSoaFromAlignedAos(&input.restPosition2, aosRestPosition2);
            FmSetSoaFromAlignedAos(&input.restPosition3, aosRestPosition3);

            FmSetSoaFromAlignedAos(&input.stressRestPosition0, aosStressRestPosition0);
            FmSetSoaFromAlignedAos(&input.stressRestPosition1, aosStressRestPosition1);
            FmSetSoaFromAlignedAos(&input.stressRestPosition2, aosStressRestPosition2);
            FmSetSoaFromAlignedAos(&input.stressRestPosition3, aosStressRestPosition3);

            FmSetSoaFromAlignedAos(&input.shapeParams.baryMatrix.col0, aosShapeParamsBaryMatrixCol0);
            FmSetSoaFromAlignedAos(&input.shapeParams.baryMatrix.col1, aosShapeParamsBaryMatrixCol1);
            FmSetSoaFromAlignedAos(&input.shapeParams.baryMatrix.col2, aosShapeParamsBaryMatrixCol2);
            FmSetSoaFromAlignedAos(&input.shapeParams.baryMatrix.col3, aosShapeParamsBaryMatrixCol3);

            FmSetSoaFromAlignedAos(&input.stressShapeParams.baryMatrix.col0, aosStressShapeParamsBaryMatrixCol0);
            FmSetSoaFromAlignedAos(&input.stressShapeParams.baryMatrix.col1, aosStressShapeParamsBaryMatrixCol1);
            FmSetSoaFromAlignedAos(&input.stressShapeParams.baryMatrix.col2, aosStressShapeParamsBaryMatrixCol2);
            FmSetSoaFromAlignedAos(&input.stressShapeParams.baryMatrix.col3, aosStressShapeParamsBaryMatrixCol3);

            if (meshSupportsPlasticity)
            {
                FmSetSoaFromAlignedAos(&input.plasticDeformationMatrix.col0, aosPlasticDeformationMatrixCol0);
                FmSetSoaFromAlignedAos(&input.plasticDeformationMatrix.col1, aosPlasticDeformationMatrixCol1);
                FmSetSoaFromAlignedAos(&input.plasticDeformationMatrix.col2, aosPlasticDeformationMatrixCol2);

#if FM_COMPUTE_PLASTIC_REL_ROTATION            
                FmSetSoaFromAlignedAos(&input.plasticTetRelRotation.col0, aosPlasticTetRelRotationCol0);
                FmSetSoaFromAlignedAos(&input.plasticTetRelRotation.col1, aosPlasticTetRelRotationCol1);
                FmSetSoaFromAlignedAos(&input.plasticTetRelRotation.col2, aosPlasticTetRelRotationCol2);
#endif
            }
        }
    };
#endif

    // Initialize FmUpdateTetStateInput for tet
    static FM_FORCE_INLINE void FmGatherUpdateTetStateInput(
        FmUpdateTetStateInput* tetUpdateInput,
        const FmTetMesh& tetMesh, uint tetId,
        const FmVector3& centerOfMass,
        bool testingFracture,
        bool updatingPlasticity)
    {
        const FmTetShapeParams& tetShapeParams = tetMesh.tetsShapeParams[tetId];
        FmTetStressMaterialParams tetStressMaterialParams = tetMesh.tetsStressMaterialParams[tetId];
        FmTetPlasticityMaterialParams tetPlasticityMaterialParams = tetMesh.tetsPlasticityMaterialParams ? tetMesh.tetsPlasticityMaterialParams[tetId] : FmTetPlasticityMaterialParams();
        FmTetFractureMaterialParams tetFractureMaterialParams = tetMesh.tetsFractureMaterialParams ? tetMesh.tetsFractureMaterialParams[tetId] : FmTetFractureMaterialParams();
        uint16_t tetFlags = tetMesh.tetsFlags[tetId];

        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        uint vId0 = tetVertIds.ids[0];
        uint vId1 = tetVertIds.ids[1];
        uint vId2 = tetVertIds.ids[2];
        uint vId3 = tetVertIds.ids[3];

        FM_ASSERT(vId0 < tetMesh.numVerts && vId1 < tetMesh.numVerts && vId2 < tetMesh.numVerts && vId3 < tetMesh.numVerts);

        // Compute tet rotation based on deformed positions
        FmVector3 deformedPosition0 = tetMesh.vertsPos[vId0] - centerOfMass;  // Offset positions closer to origin
        FmVector3 deformedPosition1 = tetMesh.vertsPos[vId1] - centerOfMass;
        FmVector3 deformedPosition2 = tetMesh.vertsPos[vId2] - centerOfMass;
        FmVector3 deformedPosition3 = tetMesh.vertsPos[vId3] - centerOfMass;

        FmVector3 restPosition0 = tetMesh.vertsRestPos[vId0];
        FmVector3 restPosition1 = tetMesh.vertsRestPos[vId1];
        FmVector3 restPosition2 = tetMesh.vertsRestPos[vId2];
        FmVector3 restPosition3 = tetMesh.vertsRestPos[vId3];

        // Get tet rest positions.
        // For plastic deformation, modify rest positions with current plastic deformation
        FmVector3 stressRestPosition0, stressRestPosition1, stressRestPosition2, stressRestPosition3;
        FmTetShapeParams stressShapeParams;

        bool meshSupportsPlasticity = (tetMesh.tetsPlasticity != NULL);

        if (meshSupportsPlasticity)
        {
            const FmTetPlasticityState& plasticityState = tetMesh.tetsPlasticity[tetId];

            // Shift the rest positions according to the plastic deformation. Also update tet matrices that depend on these positions.
            plasticityState.plasticShapeParams.ComputeRestPositions(&stressRestPosition0, &stressRestPosition1, &stressRestPosition2, &stressRestPosition3);

            stressShapeParams = plasticityState.plasticShapeParams;

            tetUpdateInput->plasticDeformationMatrix = plasticityState.plasticDeformationMatrix;
#if FM_COMPUTE_PLASTIC_REL_ROTATION
            tetUpdateInput->plasticTetRelRotation = plasticityState.plasticTetRelRotation;
#endif
        }
        else
        {
            stressRestPosition0 = restPosition0;
            stressRestPosition1 = restPosition1;
            stressRestPosition2 = restPosition2;
            stressRestPosition3 = restPosition3;

            stressShapeParams = tetShapeParams;
        }

        bool meshSupportsFracture = (tetMesh.tetsToFracture != NULL);

        bool tetTestFracture = testingFracture
            && meshSupportsFracture
            && !FM_IS_SET(tetFlags, FM_TET_FLAG_KINEMATIC)
            && !FM_ALL_SET(tetFlags,
                FM_TET_FLAG_FACE0_FRACTURE_DISABLED |
                FM_TET_FLAG_FACE1_FRACTURE_DISABLED |
                FM_TET_FLAG_FACE2_FRACTURE_DISABLED |
                FM_TET_FLAG_FACE3_FRACTURE_DISABLED);

        bool tetUpdatePlasticity = updatingPlasticity
            && meshSupportsPlasticity
            && tetPlasticityMaterialParams.plasticCreep > 0.0f
            && FM_NOT_SET(tetFlags, FM_TET_FLAG_PLASTICITY_DISABLED);

        tetUpdateInput->deformedPosition0 = deformedPosition0;
        tetUpdateInput->deformedPosition1 = deformedPosition1;
        tetUpdateInput->deformedPosition2 = deformedPosition2;
        tetUpdateInput->deformedPosition3 = deformedPosition3;

        tetUpdateInput->restPosition0 = restPosition0;
        tetUpdateInput->restPosition1 = restPosition1;
        tetUpdateInput->restPosition2 = restPosition2;
        tetUpdateInput->restPosition3 = restPosition3;

        tetUpdateInput->stressRestPosition0 = stressRestPosition0;
        tetUpdateInput->stressRestPosition1 = stressRestPosition1;
        tetUpdateInput->stressRestPosition2 = stressRestPosition2;
        tetUpdateInput->stressRestPosition3 = stressRestPosition3;

        tetUpdateInput->shapeParams = tetShapeParams;
        tetUpdateInput->stressShapeParams = stressShapeParams;

        tetUpdateInput->youngsModulus = tetStressMaterialParams.youngsModulus;
        tetUpdateInput->poissonsRatio = tetStressMaterialParams.poissonsRatio;

        tetUpdateInput->fractureStressThreshold = tetFractureMaterialParams.fractureStressThreshold;

        if (tetUpdatePlasticity)
        {
            tetUpdateInput->plasticYieldThreshold = tetPlasticityMaterialParams.plasticYieldThreshold;
            tetUpdateInput->plasticCreep = tetPlasticityMaterialParams.plasticCreep;
            tetUpdateInput->plasticMin = tetPlasticityMaterialParams.plasticMin;
            tetUpdateInput->plasticMax = tetPlasticityMaterialParams.plasticMax;
            tetUpdateInput->preserveVolume = ((tetFlags & FM_TET_FLAG_VOLUME_PRESERVING_PLASTICITY) != 0);
        }

        tetUpdateInput->testFracture = tetTestFracture;
        tetUpdateInput->updatePlasticity = tetUpdatePlasticity;
    }

    // Update state of the multiplicative plasticity model.
    // If return is true, outputs updated plasticDeformationMatrix and shape params.
    // References:
    // - Irving et al., "Invertible Finite Elements For Robust Simulation of Large Deformation"
    // - Bargteil et al., "A Finite Element Method for Animating Large Viscoplastic Flow"
    static FM_FORCE_INLINE bool FmUpdateTetPlasticity(
#if FM_COMPUTE_PLASTIC_REL_ROTATION
        FmMatrix3* outPlasticTetRelRotation,
#endif
        FmMatrix3* outPlasticDeformationMatrix,
        FmTetShapeParams* outPlasticShapeParams,
        const FmMatrix3& inPlasticDeformationMatrix,
        const FmTetShapeParams& inPlasticShapeParams,
        const FmMatrix3& tetRotationInv,
        const FmVector3& restPosition0,
        const FmVector3& restPosition1,
        const FmVector3& restPosition2,
        const FmVector3& restPosition3,
        const FmVector3& deformedPosition0,
        const FmVector3& deformedPosition1,
        const FmVector3& deformedPosition2,
        const FmVector3& deformedPosition3,
        const FmVector3& stress0,
        const FmVector3& stress1,
        float plasticYieldThreshold, float plasticCreep, float plasticMin, float plasticMax, bool preserveVolume)
    {
        (void)tetRotationInv;
        FmMatrix3 plasticDeformationMatrix = inPlasticDeformationMatrix;
        FmTetShapeParams plasticShapeParams = inPlasticShapeParams;

        float elasticStressMag = sqrtf(lengthSqr(stress0) + lengthSqr(stress1));
        float yieldStress = plasticYieldThreshold;

        bool updateNeeded = false;

        if (elasticStressMag > yieldStress)
        {
            FmMatrix3 restVolumeMatrixInv = inverse(FmComputeTetVolumeMatrix(restPosition0, restPosition1, restPosition2, restPosition3));

            // Compute deformation gradient (decomposed into SVD), after applying plastic deformation offset
            FmMatrix3 U, V;
            FmVector3 Fhat;
            FmComputeTetElasticDeformationGradient(&U, &Fhat, &V,
                deformedPosition0,
                deformedPosition1,
                deformedPosition2,
                deformedPosition3,
                restVolumeMatrixInv, inverse(plasticDeformationMatrix));

            FmVector3 elasticDeformation = Fhat;

            const float deformationThreshold = 0.01f;
            if (elasticDeformation.x > deformationThreshold       // Do nothing for low or inverted elastic deformation
                && elasticDeformation.y > deformationThreshold
                && elasticDeformation.z > deformationThreshold)
            {
                updateNeeded = true;

                float creep = plasticCreep;
                float minDeformation = FmMaxFloat(plasticMin, deformationThreshold);
                float maxDeformation = plasticMax;

                if (preserveVolume)
                {
                    // Scale the elastic deformation such that determinant is 1.0, which represents a volume preserving deformation.
                    float elasticDet = elasticDeformation.x * elasticDeformation.y * elasticDeformation.z;
                    elasticDeformation = elasticDeformation * powf(elasticDet, -(1.0f / 3.0f));
                }

                // Exponentiate elastic deformation to compute a contribution to plastic deformation
                float plasticPower = FmMinFloat(creep * (elasticStressMag - yieldStress) / elasticStressMag, 1.0f);

                FmVector3 Fphat;
                Fphat.x = powf(elasticDeformation.x, plasticPower);
                Fphat.y = powf(elasticDeformation.y, plasticPower);
                Fphat.z = powf(elasticDeformation.z, plasticPower);

                FmMatrix3 contribution = mul(mul(V, FmMatrix3::scale(Fphat)), transpose(V));

                plasticDeformationMatrix = mul(contribution, plasticDeformationMatrix);

                // Limit total plastic deformation by limiting singular values
                FmVector3 totalPlasticDeformation;
                FmSvd3x3(&U, &totalPlasticDeformation, &V, plasticDeformationMatrix);

                totalPlasticDeformation.x = FmMaxFloat(totalPlasticDeformation.x, minDeformation);
                totalPlasticDeformation.y = FmMaxFloat(totalPlasticDeformation.y, minDeformation);
                totalPlasticDeformation.z = FmMaxFloat(totalPlasticDeformation.z, minDeformation);

                totalPlasticDeformation.x = FmMinFloat(totalPlasticDeformation.x, maxDeformation);
                totalPlasticDeformation.y = FmMinFloat(totalPlasticDeformation.y, maxDeformation);
                totalPlasticDeformation.z = FmMinFloat(totalPlasticDeformation.z, maxDeformation);

                if (preserveVolume)
                {
                    // Resize total to preserve volume
                    float plasticDet = totalPlasticDeformation.x * totalPlasticDeformation.y * totalPlasticDeformation.z;
                    totalPlasticDeformation = totalPlasticDeformation * powf(plasticDet, -(1.0f / 3.0f));
                }

                plasticDeformationMatrix = mul(mul(V, FmMatrix3::scale(totalPlasticDeformation)), transpose(V));

                // Adjust stress and strain matrices to match the displaced rest positions.  This has seemed to fix some false 
                // motion issues seen in some plastically deformed objects when they aren't pinned down by kinematic vertices.
                // However the change in aspect ratios can have a negative effect on the system matrix conditioning.
                // Note the plastic min and max values give a way to limit that change.
                FmMatrix3 offsets = mul(plasticDeformationMatrix, inverse(restVolumeMatrixInv));
                FmVector3 plasticRestPosition3 = FmVector3(0.0f);
                FmVector3 plasticRestPosition0 = offsets.col0;
                FmVector3 plasticRestPosition1 = offsets.col1;
                FmVector3 plasticRestPosition2 = offsets.col2;

                FmComputeShapeParams(&plasticShapeParams, plasticRestPosition0, plasticRestPosition1, plasticRestPosition2, plasticRestPosition3);

                *outPlasticDeformationMatrix = plasticDeformationMatrix;
                *outPlasticShapeParams = plasticShapeParams;

#if FM_COMPUTE_PLASTIC_REL_ROTATION     
                // Compute rotation based on plastic rest positions, and relative transform to be applied to regular rotation
                FmMatrix3 plasticTetRotation = FmComputeTetRotationPolarDecompSvd(
                    deformedPosition0,
                    deformedPosition1,
                    deformedPosition2,
                    deformedPosition3,
                    plasticShapeParams.baryMatrix);

                FmMatrix3 plasticTetRelRotation = mul(plasticTetRotation, tetRotationInv);

                *outPlasticTetRelRotation = plasticTetRelRotation;
#endif
            }
        }

        return updateNeeded;
    }

    // Update state of the multiplicative plasticity model.
    // If return is true, outputs updated plasticDeformationMatrix and shape params.
    // References:
    // - Irving et al., "Invertible Finite Elements For Robust Simulation of Large Deformation"
    // - Bargteil et al., "A Finite Element Method for Animating Large Viscoplastic Flow"
    template<class T>
    typename T::SoaBool FmUpdateTetPlasticity(
#if FM_COMPUTE_PLASTIC_REL_ROTATION
        typename T::SoaMatrix3* outPlasticTetRelRotation,
#endif
        typename T::SoaMatrix3* outPlasticDeformationMatrix,
        FmSoaTetShapeParams<T>* outPlasticShapeParams,
        const typename T::SoaMatrix3& inPlasticDeformationMatrix,
        const FmSoaTetShapeParams<T>& inPlasticShapeParams,
        const typename T::SoaMatrix3& tetRotationInv,
        const typename T::SoaVector3& restPosition0,
        const typename T::SoaVector3& restPosition1,
        const typename T::SoaVector3& restPosition2,
        const typename T::SoaVector3& restPosition3,
        const typename T::SoaVector3& deformedPosition0,
        const typename T::SoaVector3& deformedPosition1,
        const typename T::SoaVector3& deformedPosition2,
        const typename T::SoaVector3& deformedPosition3,
        const typename T::SoaVector3& stress0,
        const typename T::SoaVector3& stress1,
        typename T::SoaFloat plasticYieldThreshold, typename T::SoaFloat plasticCreep, typename T::SoaFloat plasticMin, typename T::SoaFloat plasticMax, typename T::SoaBool preserveVolume)
    {
        (void)tetRotationInv;
        typename T::SoaMatrix3 plasticDeformationMatrix = inPlasticDeformationMatrix;
        FmSoaTetShapeParams<T> plasticShapeParams = inPlasticShapeParams;

        typename T::SoaFloat elasticStressMag = sqrtf(lengthSqr(stress0) + lengthSqr(stress1));
        typename T::SoaFloat yieldStress = plasticYieldThreshold;

        typename T::SoaBool stressOverYield = elasticStressMag > yieldStress;
        typename T::SoaBool updateNeeded = stressOverYield;

        if (any(updateNeeded))
        {
            typename T::SoaMatrix3 restVolumeMatrixInv = inverse(FmComputeTetVolumeMatrix<T>(restPosition0, restPosition1, restPosition2, restPosition3));

            // Compute deformation gradient (decomposed into SVD), after applying plastic deformation offset
            typename T::SoaMatrix3 U, V;
            typename T::SoaVector3 Fhat;
            FmComputeTetElasticDeformationGradient<T>(&U, &Fhat, &V,
                deformedPosition0,
                deformedPosition1,
                deformedPosition2,
                deformedPosition3,
                restVolumeMatrixInv, inverse(plasticDeformationMatrix));

            typename T::SoaVector3 elasticDeformation = Fhat;

            const typename T::SoaFloat deformationThreshold = 0.01f;

            typename T::SoaBool overThreshold = 
                (elasticDeformation.x > deformationThreshold)
                & (elasticDeformation.y > deformationThreshold)
                & (elasticDeformation.z > deformationThreshold);

            updateNeeded = updateNeeded & overThreshold;

            if (any(updateNeeded))       // Do nothing for low or inverted elastic deformation
            {
                typename T::SoaFloat creep = plasticCreep;
                typename T::SoaFloat minDeformation = max(plasticMin, deformationThreshold);
                typename T::SoaFloat maxDeformation = plasticMax;

                preserveVolume = preserveVolume & updateNeeded;
                if (any(preserveVolume))
                {
                    // Scale the elastic deformation such that determinant is 1.0, which represents a volume preserving deformation.
                    typename T::SoaFloat elasticDet = elasticDeformation.x * elasticDeformation.y * elasticDeformation.z;
                    elasticDeformation = select(elasticDeformation, elasticDeformation * powf(elasticDet, -(1.0f / 3.0f)), preserveVolume);
                }

                // Exponentiate elastic deformation to compute a contribution to plastic deformation
                typename T::SoaFloat plasticPower = min(creep * (elasticStressMag - yieldStress) / elasticStressMag, 1.0f);

                typename T::SoaVector3 Fphat;
                Fphat.x = powf(elasticDeformation.x, plasticPower);
                Fphat.y = powf(elasticDeformation.y, plasticPower);
                Fphat.z = powf(elasticDeformation.z, plasticPower);

                typename T::SoaMatrix3 contribution = mul(mul(V, typename T::SoaMatrix3::scale(Fphat)), transpose(V));

                plasticDeformationMatrix = mul(contribution, plasticDeformationMatrix);

                // Limit total plastic deformation by limiting singular values
                typename T::SoaVector3 totalPlasticDeformation;
                FmSvd3x3<T>(&U, &totalPlasticDeformation, &V, plasticDeformationMatrix);

                totalPlasticDeformation.x = max(totalPlasticDeformation.x, minDeformation);
                totalPlasticDeformation.y = max(totalPlasticDeformation.y, minDeformation);
                totalPlasticDeformation.z = max(totalPlasticDeformation.z, minDeformation);

                totalPlasticDeformation.x = min(totalPlasticDeformation.x, maxDeformation);
                totalPlasticDeformation.y = min(totalPlasticDeformation.y, maxDeformation);
                totalPlasticDeformation.z = min(totalPlasticDeformation.z, maxDeformation);

                if (any(preserveVolume))
                {
                    // Resize total to preserve volume
                    typename T::SoaFloat plasticDet = totalPlasticDeformation.x * totalPlasticDeformation.y * totalPlasticDeformation.z;
                    totalPlasticDeformation = select(totalPlasticDeformation, totalPlasticDeformation * powf(plasticDet, -(1.0f / 3.0f)), preserveVolume);
                }

                plasticDeformationMatrix = mul(mul(V, typename T::SoaMatrix3::scale(totalPlasticDeformation)), transpose(V));

                // Adjust stress and strain matrices to match the displaced rest positions.  This has seemed to fix some false 
                // motion issues seen in some plastically deformed objects when they aren't pinned down by kinematic vertices.
                // However the change in aspect ratios can have a negative effect on the system matrix conditioning.
                // Note the plastic min and max values give a way to limit that change.
                typename T::SoaMatrix3 offsets = mul(plasticDeformationMatrix, inverse(restVolumeMatrixInv));
                typename T::SoaVector3 plasticRestPosition3 = typename T::SoaVector3(0.0f);
                typename T::SoaVector3 plasticRestPosition0 = offsets.col0;
                typename T::SoaVector3 plasticRestPosition1 = offsets.col1;
                typename T::SoaVector3 plasticRestPosition2 = offsets.col2;

                FmComputeShapeParams<T>(&plasticShapeParams, plasticRestPosition0, plasticRestPosition1, plasticRestPosition2, plasticRestPosition3);

                *outPlasticDeformationMatrix = plasticDeformationMatrix;
                *outPlasticShapeParams = plasticShapeParams;

#if FM_COMPUTE_PLASTIC_REL_ROTATION     
                // Compute rotation based on plastic rest positions, and relative transform to be applied to regular rotation
                typename T::SoaMatrix3 plasticTetRotation = FmComputeTetRotationPolarDecompSvd<T>(
                    deformedPosition0,
                    deformedPosition1,
                    deformedPosition2,
                    deformedPosition3,
                    plasticShapeParams.baryMatrix);

                typename T::SoaMatrix3 plasticTetRelRotation = mul(plasticTetRotation, tetRotationInv);

                *outPlasticTetRelRotation = plasticTetRelRotation;
#endif
            }
        }

        return updateNeeded;
    }

    void FmComputeUpdateTetStateOutput(
        FmUpdateTetStateOutput* tetUpdateOutput,
        const FmUpdateTetStateInput& tetUpdateInput,
        bool meshHasPlasticity, bool computingTetDeformation)
    {
        (void)meshHasPlasticity;

        tetUpdateOutput->fractureStressThreshold = tetUpdateInput.fractureStressThreshold;
        tetUpdateOutput->testFracture = tetUpdateInput.testFracture;
        tetUpdateOutput->updatePlasticity = tetUpdateInput.updatePlasticity;

        FmMatrix3 tetRotation = FmComputeTetRotationPolarDecompSvd(
            tetUpdateInput.deformedPosition0,
            tetUpdateInput.deformedPosition1,
            tetUpdateInput.deformedPosition2,
            tetUpdateInput.deformedPosition3,
            tetUpdateInput.shapeParams.baryMatrix);

        tetUpdateOutput->tetRotation = tetRotation;

        FmTetStressMatrix stressMat;

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        FmTetStiffnessMatrix stiffnessMat;
        FmComputeTetStressAndStiffnessMatrix(&stressMat, &stiffnessMat,
            tetUpdateInput.stressShapeParams.baryMatrix.col0,
            tetUpdateInput.stressShapeParams.baryMatrix.col1,
            tetUpdateInput.stressShapeParams.baryMatrix.col2,
            tetUpdateInput.stressShapeParams.GetVolume(),
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#else
        FmComputeTetStressMatrix(&stressMat,
            tetUpdateInput.stressShapeParams.baryMatrix.col0,
            tetUpdateInput.stressShapeParams.baryMatrix.col1,
            tetUpdateInput.stressShapeParams.baryMatrix.col2,
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#endif

#if FM_COMPUTE_PLASTIC_REL_ROTATION || !FM_MATRIX_ASSEMBLY_BY_TETS
        // Tet rotation for stress calculation, which depends on plastic deformation
        FmMatrix3 stressTetRotation;

#if FM_COMPUTE_PLASTIC_REL_ROTATION
        if (meshHasPlasticity)
        {
            // Modify the tet rotation to represent the rotation from modified rest positions.
            stressTetRotation = mul(tetUpdateInput.plasticTetRelRotation, tetRotation);
        }
        else
#endif
        {
            stressTetRotation = tetRotation;
        }
#endif

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        // Compute rotated stiffness for matrix assembly
        FmComputeRotatedStiffnessMatrix(&tetUpdateOutput->rotatedStiffnessMat, stiffnessMat,
            tetUpdateInput.stressRestPosition0,
            tetUpdateInput.stressRestPosition1,
            tetUpdateInput.stressRestPosition2,
            tetUpdateInput.stressRestPosition3,
            stressTetRotation);
#endif

        if (computingTetDeformation || tetUpdateInput.testFracture || tetUpdateInput.updatePlasticity)
        {
            FmMatrix3 tetRotationInv = transpose(tetRotation);

#if FM_COMPUTE_PLASTIC_REL_ROTATION     
            // Compute the offsets needed for stress calculation.
            // Use post-plasticity rotation to more accurately compute elastic stress.
            FmMatrix3 stressTetRotationInv = transpose(stressTetRotation);
#else
            FmMatrix3 stressTetRotationInv = tetRotationInv;
#endif


            if (computingTetDeformation)
            {
                // Compute deformation offsets for strain calculation.
                // Use the original rest positions for total deformation.
                FmVector3 unrotatedOffset0 = mul(tetRotationInv, tetUpdateInput.deformedPosition0) - tetUpdateInput.restPosition0;
                FmVector3 unrotatedOffset1 = mul(tetRotationInv, tetUpdateInput.deformedPosition1) - tetUpdateInput.restPosition1;
                FmVector3 unrotatedOffset2 = mul(tetRotationInv, tetUpdateInput.deformedPosition2) - tetUpdateInput.restPosition2;
                FmVector3 unrotatedOffset3 = mul(tetRotationInv, tetUpdateInput.deformedPosition3) - tetUpdateInput.restPosition3;

                FmVector3 strain[2];
                FmComputeTetStrain(strain, tetUpdateInput.shapeParams, unrotatedOffset0, unrotatedOffset1, unrotatedOffset2, unrotatedOffset3);
                float strainMag = sqrtf(lengthSqr(strain[0]) + lengthSqr(strain[1]));

                tetUpdateOutput->strainMag = strainMag;
            }
               
            if (tetUpdateInput.testFracture || tetUpdateInput.updatePlasticity)
            {
                // Compute elastic stress
                FmVector3 unrotatedOffset0 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition0) - tetUpdateInput.stressRestPosition0;
                FmVector3 unrotatedOffset1 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition1) - tetUpdateInput.stressRestPosition1;
                FmVector3 unrotatedOffset2 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition2) - tetUpdateInput.stressRestPosition2;
                FmVector3 unrotatedOffset3 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition3) - tetUpdateInput.stressRestPosition3;

                FmVector3 stress0 =
                    mul(stressMat.EBe00, unrotatedOffset0) +
                    mul(stressMat.EBe01, unrotatedOffset1) +
                    mul(stressMat.EBe02, unrotatedOffset2) +
                    mul(stressMat.EBe03, unrotatedOffset3);
                FmVector3 stress1 =
                    mul(stressMat.EBe10, unrotatedOffset0) +
                    mul(stressMat.EBe11, unrotatedOffset1) +
                    mul(stressMat.EBe12, unrotatedOffset2) +
                    mul(stressMat.EBe13, unrotatedOffset3);

                if (tetUpdateInput.testFracture)
                {
                    // Create symmetric 3x3 matrix from stress components
                    float sxx = stress0.x;
                    float syy = stress0.y;
                    float szz = stress0.z;
                    float syz = stress1.x;
                    float szx = stress1.y;
                    float sxy = stress1.z;

                    FmMatrix3 stress3(
                        FmInitVector3(sxx, sxy, szx),
                        FmInitVector3(sxy, syy, syz),
                        FmInitVector3(szx, syz, szz));

                    // primary direction of stress is eigenvector for largest eigenvalue
                    FmVector3 eigenvals;
                    FmMatrix3 eigenvecs;
                    FmEigenSymm3x3CyclicJacobi(&eigenvals, &eigenvecs, stress3);

                    tetUpdateOutput->maxStressDirection = eigenvecs.col0;
                    tetUpdateOutput->maxStressEigenvalue = fabsf(eigenvals.x);
                }

                if (tetUpdateInput.updatePlasticity)
                {
                    bool updateNeeded = 
                        FmUpdateTetPlasticity(
#if FM_COMPUTE_PLASTIC_REL_ROTATION
                            &tetUpdateOutput->plasticTetRelRotation,
#endif
                            &tetUpdateOutput->plasticDeformationMatrix, &tetUpdateOutput->plasticShapeParams,
                            tetUpdateInput.plasticDeformationMatrix,
                            tetUpdateInput.stressShapeParams,  // = FmTetPlasticityState::plasticShapeParams
                            tetRotationInv,
                            tetUpdateInput.restPosition0,
                            tetUpdateInput.restPosition1, 
                            tetUpdateInput.restPosition2, 
                            tetUpdateInput.restPosition3,
                            tetUpdateInput.deformedPosition0,
                            tetUpdateInput.deformedPosition1,
                            tetUpdateInput.deformedPosition2,
                            tetUpdateInput.deformedPosition3,
                            stress0,
                            stress1,
                            tetUpdateInput.plasticYieldThreshold, tetUpdateInput.plasticCreep, tetUpdateInput.plasticMin, tetUpdateInput.plasticMax, tetUpdateInput.preserveVolume);

                    tetUpdateOutput->updatePlasticity = updateNeeded;
                }
            }
        }
    }

#if FM_SOA_TET_MATH
    template<class T>
    void FmComputeUpdateTetStateOutput(
        FmSoaUpdateTetStateOutput<T>* tetUpdateOutput,
        const FmSoaUpdateTetStateInput<T>& tetUpdateInput,
        bool meshHasPlasticity, bool computingTetDeformation)
    {
        (void)meshHasPlasticity;

        tetUpdateOutput->fractureStressThreshold = tetUpdateInput.fractureStressThreshold;
        tetUpdateOutput->testFracture = tetUpdateInput.testFracture;
        tetUpdateOutput->updatePlasticity = tetUpdateInput.updatePlasticity;

        typename T::SoaMatrix3 tetRotation = FmComputeTetRotationPolarDecompSvd<T>(
            tetUpdateInput.deformedPosition0,
            tetUpdateInput.deformedPosition1,
            tetUpdateInput.deformedPosition2,
            tetUpdateInput.deformedPosition3,
            tetUpdateInput.shapeParams.baryMatrix);

        tetUpdateOutput->tetRotation = tetRotation;

        FmSoaTetStressMatrix<T> stressMat;

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        FmSoaTetStiffnessMatrix<T> stiffnessMat;
        FmComputeTetStressAndStiffnessMatrix(&stressMat, &stiffnessMat,
            tetUpdateInput.stressShapeParams.baryMatrix.col0,
            tetUpdateInput.stressShapeParams.baryMatrix.col1,
            tetUpdateInput.stressShapeParams.baryMatrix.col2,
            tetUpdateInput.stressShapeParams.GetVolume(),
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#else
        FmComputeTetStressMatrix(&stressMat,
            tetUpdateInput.stressShapeParams.baryMatrix.col0,
            tetUpdateInput.stressShapeParams.baryMatrix.col1,
            tetUpdateInput.stressShapeParams.baryMatrix.col2,
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#endif

#if FM_COMPUTE_PLASTIC_REL_ROTATION || !FM_MATRIX_ASSEMBLY_BY_TETS
        // Tet rotation for stress calculation, which depends on plastic deformation
        typename T::SoaMatrix3 stressTetRotation;
        
#if FM_COMPUTE_PLASTIC_REL_ROTATION
        if (meshHasPlasticity)
        {
            // Modify the tet rotation to represent the rotation from modified rest positions.
            stressTetRotation = mul(tetUpdateInput.plasticTetRelRotation, tetRotation);
        }
        else
#endif
        {
            stressTetRotation = tetRotation;
        }
#endif

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        // Compute rotated stiffness for matrix assembly
        FmComputeRotatedStiffnessMatrix(&tetUpdateOutput->rotatedStiffnessMat, stiffnessMat,
            tetUpdateInput.stressRestPosition0,
            tetUpdateInput.stressRestPosition1,
            tetUpdateInput.stressRestPosition2,
            tetUpdateInput.stressRestPosition3,
            stressTetRotation);
#endif

        if (computingTetDeformation || any(tetUpdateInput.testFracture | tetUpdateInput.updatePlasticity))
        {
            typename T::SoaMatrix3 tetRotationInv = transpose(tetRotation);

#if FM_COMPUTE_PLASTIC_REL_ROTATION     
            // Compute the offsets needed for stress calculation.
            // Use post-plasticity rotation to more accurately compute elastic stress.
            typename T::SoaMatrix3 stressTetRotationInv = transpose(stressTetRotation);
#else
            typename T::SoaMatrix3 stressTetRotationInv = tetRotationInv;
#endif

            if (computingTetDeformation)
            {
                // Compute deformation offsets for strain calculation.
                // Use the original rest positions for total deformation.
                typename T::SoaVector3 unrotatedOffset0 = mul(tetRotationInv, tetUpdateInput.deformedPosition0) - tetUpdateInput.restPosition0;
                typename T::SoaVector3 unrotatedOffset1 = mul(tetRotationInv, tetUpdateInput.deformedPosition1) - tetUpdateInput.restPosition1;
                typename T::SoaVector3 unrotatedOffset2 = mul(tetRotationInv, tetUpdateInput.deformedPosition2) - tetUpdateInput.restPosition2;
                typename T::SoaVector3 unrotatedOffset3 = mul(tetRotationInv, tetUpdateInput.deformedPosition3) - tetUpdateInput.restPosition3;

                typename T::SoaVector3 strain[2];
                FmComputeTetStrain<T>(strain, tetUpdateInput.shapeParams, unrotatedOffset0, unrotatedOffset1, unrotatedOffset2, unrotatedOffset3);
                typename T::SoaFloat tetDeformation = sqrtf(lengthSqr(strain[0]) + lengthSqr(strain[1]));

                tetUpdateOutput->strainMag = tetDeformation;
            }

            if (any(tetUpdateInput.testFracture | tetUpdateInput.updatePlasticity))
            {
                // Compute elastic stress
                typename T::SoaVector3 unrotatedOffset0 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition0) - tetUpdateInput.stressRestPosition0;
                typename T::SoaVector3 unrotatedOffset1 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition1) - tetUpdateInput.stressRestPosition1;
                typename T::SoaVector3 unrotatedOffset2 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition2) - tetUpdateInput.stressRestPosition2;
                typename T::SoaVector3 unrotatedOffset3 = mul(stressTetRotationInv, tetUpdateInput.deformedPosition3) - tetUpdateInput.stressRestPosition3;

                typename T::SoaVector3 stress0 =
                    mul(stressMat.EBe00, unrotatedOffset0) +
                    mul(stressMat.EBe01, unrotatedOffset1) +
                    mul(stressMat.EBe02, unrotatedOffset2) +
                    mul(stressMat.EBe03, unrotatedOffset3);
                typename T::SoaVector3 stress1 =
                    mul(stressMat.EBe10, unrotatedOffset0) +
                    mul(stressMat.EBe11, unrotatedOffset1) +
                    mul(stressMat.EBe12, unrotatedOffset2) +
                    mul(stressMat.EBe13, unrotatedOffset3);

                if (any(tetUpdateInput.testFracture))
                {
                    // Create symmetric 3x3 matrix from stress components
                    typename T::SoaFloat sxx = stress0.x;
                    typename T::SoaFloat syy = stress0.y;
                    typename T::SoaFloat szz = stress0.z;
                    typename T::SoaFloat syz = stress1.x;
                    typename T::SoaFloat szx = stress1.y;
                    typename T::SoaFloat sxy = stress1.z;

                    typename T::SoaMatrix3 stress3(
                        FmInitVector3<T>(sxx, sxy, szx),
                        FmInitVector3<T>(sxy, syy, syz),
                        FmInitVector3<T>(szx, syz, szz));

                    // primary direction of stress is eigenvector for largest eigenvalue
                    typename T::SoaVector3 eigenvals;
                    typename T::SoaMatrix3 eigenvecs;
                    FmEigenSymm3x3CyclicJacobi<T>(&eigenvals, &eigenvecs, stress3);

                    tetUpdateOutput->maxStressDirection = eigenvecs.col0;
                    tetUpdateOutput->maxStressEigenvalue = fabsf(eigenvals.x);
                }

                if (any(tetUpdateInput.updatePlasticity))
                {
                    typename T::SoaBool updateNeeded = 
                        FmUpdateTetPlasticity(
#if FM_COMPUTE_PLASTIC_REL_ROTATION
                            &tetUpdateOutput->plasticTetRelRotation,
#endif
                            &tetUpdateOutput->plasticDeformationMatrix, &tetUpdateOutput->plasticShapeParams,
                            tetUpdateInput.plasticDeformationMatrix,
                            tetUpdateInput.stressShapeParams,  // = typename T::SoaTetPlasticityState::plasticShapeParams
                            tetRotationInv,
                            tetUpdateInput.restPosition0,
                            tetUpdateInput.restPosition1,
                            tetUpdateInput.restPosition2,
                            tetUpdateInput.restPosition3,
                            tetUpdateInput.deformedPosition0,
                            tetUpdateInput.deformedPosition1,
                            tetUpdateInput.deformedPosition2,
                            tetUpdateInput.deformedPosition3,
                            stress0,
                            stress1,
                            tetUpdateInput.plasticYieldThreshold, tetUpdateInput.plasticCreep, tetUpdateInput.plasticMin, tetUpdateInput.plasticMax, tetUpdateInput.preserveVolume);

                    tetUpdateOutput->updatePlasticity = tetUpdateInput.updatePlasticity & updateNeeded;
                }
            }
        }
    }
#endif

    bool FmStoreUpdatedTetState(FmTetMesh* tetMesh, uint tetId,
        const FmUpdateTetStateOutput& updatedState,
        bool computingStrainMag)
    {
        FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

        // Store tet rotation
        tetMesh->tetsRotation[tetId] = updatedState.tetRotation;

        // Store strain mag
        if (computingStrainMag)
        {
            tetMesh->tetsStrainMag[tetId] = updatedState.strainMag;
        }

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        tetMesh->tetsStiffness[tetId].rotatedStiffnessMat = updatedState.rotatedStiffnessMat;
#endif

        if (updatedState.updatePlasticity)
        {
            FmTetPlasticityState& plasticityState = tetMesh->tetsPlasticity[tetId];

            plasticityState.plasticDeformationMatrix = updatedState.plasticDeformationMatrix;
            plasticityState.plasticShapeParams = updatedState.plasticShapeParams;
#if FM_COMPUTE_PLASTIC_REL_ROTATION            
            plasticityState.plasticTetRelRotation = updatedState.plasticTetRelRotation;
#endif
        }

        bool removedKinematicFlag = false;

        if (updatedState.testFracture)
        {
            if (updatedState.maxStressEigenvalue > updatedState.fractureStressThreshold)
            {
                uint tetToFractureIndex = FmAtomicIncrement(&tetMesh->numTetsToFracture.val) - 1;
                FmTetToFracture& tetToFracture = tetMesh->tetsToFracture[tetToFractureIndex];
                tetToFracture.tetId = tetId;
                tetToFracture.fractureDirection = updatedState.maxStressDirection; // direction in element's rest orientation
            }

            // Break kinematic flags based on stress threshold, if allowed.
            // Multiple threads may update same vertex.
            if (updatedState.maxStressEigenvalue > tetMesh->removeKinematicStressThreshold)
            {
                for (uint cornerIdx = 0; cornerIdx < 4; cornerIdx++)
                {
                    uint vIdx = tetVertIds.ids[cornerIdx];

                    if (FM_ALL_SET(tetMesh->vertsFlags[vIdx], FM_VERT_FLAG_KINEMATIC_REMOVABLE | FM_VERT_FLAG_KINEMATIC))
                    {
                        tetMesh->vertsFlags[vIdx] &= ~FM_VERT_FLAG_KINEMATIC;
                        removedKinematicFlag = true;
                    }
                }
            }
        }

        return removedKinematicFlag;
    }

    // Accumulate tet quaternion and strain values in vertices
    void FmAccumulateTetQuatAndStrainMag(FmTetMesh* tetMesh, bool computingStrainMag)
    {
        uint numTets = tetMesh->numTets;
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            float tetStrainMag = tetMesh->tetsStrainMag[tetId];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];
            uint vId0 = tetVertIds.ids[0];
            uint vId1 = tetVertIds.ids[1];
            uint vId2 = tetVertIds.ids[2];
            uint vId3 = tetVertIds.ids[3];

            FmQuat tetQuat = FmInitQuat(tetMesh->tetsRotation[tetId]);

            // Accumulate quaternions for average vertex orientation
            FmAddToQuat(&tetMesh->vertsTetValues[vId0].tetQuatSum, tetQuat);
            FmAddToQuat(&tetMesh->vertsTetValues[vId1].tetQuatSum, tetQuat);
            FmAddToQuat(&tetMesh->vertsTetValues[vId2].tetQuatSum, tetQuat);
            FmAddToQuat(&tetMesh->vertsTetValues[vId3].tetQuatSum, tetQuat);

            if (computingStrainMag)
            {
                tetMesh->vertsTetValues[vId0].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId0].numIncidentTets);
                tetMesh->vertsTetValues[vId1].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId1].numIncidentTets);
                tetMesh->vertsTetValues[vId2].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId2].numIncidentTets);
                tetMesh->vertsTetValues[vId3].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId3].numIncidentTets);

                tetMesh->vertsTetValues[vId0].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId0].tetStrainMagMax, tetMesh->vertsTetValues[vId0].tetStrainMagAvg);
                tetMesh->vertsTetValues[vId1].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId1].tetStrainMagMax, tetMesh->vertsTetValues[vId1].tetStrainMagAvg);
                tetMesh->vertsTetValues[vId2].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId2].tetStrainMagMax, tetMesh->vertsTetValues[vId2].tetStrainMagAvg);
                tetMesh->vertsTetValues[vId3].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId3].tetStrainMagMax, tetMesh->vertsTetValues[vId3].tetStrainMagAvg);
            }
        }
    }

    // Update a range of tets from beginIdx to endIdx.
    // runFracture = mesh supports fracture and fracture processing should be done.
    // updatePlasticity = mesh supports plasticity and plasticity processing should be done.
    void FmUpdateTetsRange(
        bool* outRemovedKinematicFlag,
        uint* outMaxUnconstrainedSolveIterations,
        FmTetMesh* tetMesh, uint beginIdx, uint endIdx,
        const FmVector3& centerOfMass,
        bool runFracture, bool updatePlasticity, bool meshSupportsPlasticity, bool computingStrainMag)
    {
        bool removedKinematicFlag = false;

        // Compute max iterations over tetrahedra
        uint maxUnconstrainedSolveIterations = 0;

#if FM_SOA_TET_MATH
        FmUpdateTetStateBatch<FmSoaTypes> tetUpdateBatch;
        tetUpdateBatch.baseTetIdx = beginIdx;
#endif

        // For all tets compute rotation, update stiffness matrices
        // Optionally update plasticity state, test fracture and collect fracturing tets
        for (uint tetId = beginIdx; tetId < endIdx; tetId++)
        {
            uint tetMaxUnconstrainedSolveIterations = tetMesh->tetsMaxUnconstrainedSolveIterations[tetId];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

            maxUnconstrainedSolveIterations = FmMaxUint(maxUnconstrainedSolveIterations, tetMaxUnconstrainedSolveIterations);

            // Collect input data from tet
            FmUpdateTetStateInput tetUpdateInput;
            FmGatherUpdateTetStateInput(&tetUpdateInput, *tetMesh, tetId, centerOfMass, runFracture, updatePlasticity);

            // Compute tet rotations, stress, stiffness matrices
#if FM_SOA_TET_MATH
            uint batchNumTets = tetUpdateBatch.numTets;

            if (batchNumTets == FmSoaTypes::width)
            {
                tetUpdateBatch.ConvertAosToSoa(meshSupportsPlasticity);
                FmComputeUpdateTetStateOutput<FmSoaTypes>(&tetUpdateBatch.output, tetUpdateBatch.input, meshSupportsPlasticity, computingStrainMag);

                uint baseTetIdx = tetUpdateBatch.baseTetIdx;

                for (uint batchIdx = 0; batchIdx < batchNumTets; batchIdx++)
                {
                    FmUpdateTetStateOutput tetUpdateOutput;
                    tetUpdateBatch.GetBatchSlice(&tetUpdateOutput, batchIdx, meshSupportsPlasticity, computingStrainMag);

                    FmStoreUpdatedTetState(tetMesh, baseTetIdx + batchIdx, tetUpdateOutput, computingStrainMag);
                }

                tetUpdateBatch.baseTetIdx = tetId;
                tetUpdateBatch.numTets = 0;
            }

            tetUpdateBatch.SetBatchSlice(tetUpdateBatch.numTets, tetUpdateInput, meshSupportsPlasticity);
            tetUpdateBatch.numTets++;
#else
            FmUpdateTetStateOutput tetUpdateOutput;
            FmComputeUpdateTetStateOutput(&tetUpdateOutput, tetUpdateInput, meshSupportsPlasticity, computingStrainMag);

            FmStoreUpdatedTetState(tetMesh, tetId, tetUpdateOutput, computingStrainMag);
#endif
        }

#if FM_SOA_TET_MATH
        uint batchNumTets = tetUpdateBatch.numTets;
        if (batchNumTets > 0)
        {
            tetUpdateBatch.ConvertAosToSoa(meshSupportsPlasticity);
            FmComputeUpdateTetStateOutput<FmSoaTypes>(&tetUpdateBatch.output, tetUpdateBatch.input, meshSupportsPlasticity, computingStrainMag);

            uint baseTetIdx = tetUpdateBatch.baseTetIdx;

            for (uint batchIdx = 0; batchIdx < batchNumTets; batchIdx++)
            {
                FmUpdateTetStateOutput tetUpdateOutput;
                tetUpdateBatch.GetBatchSlice(&tetUpdateOutput, batchIdx, meshSupportsPlasticity, computingStrainMag);

                FmStoreUpdatedTetState(tetMesh, baseTetIdx + batchIdx, tetUpdateOutput, computingStrainMag);
            }
        }
#endif

        *outRemovedKinematicFlag = removedKinematicFlag;
        *outMaxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;
    }

    class FmTaskDataUpdateTetState : public FmAsyncTaskData
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataUpdateTetState)

        FmAtomicUint removedKinematicFlag;
        FmAtomicUint maxUnconstrainedSolveIterations;

        FmScene* scene;
        FmTetMesh* tetMesh;
        FmVector3 centerOfMass;
        uint numTets;
        uint numTetTasks;

        bool runFracture;
        bool updatePlasticity;
        bool meshSupportsPlasticity;
        bool computingStrainMag;

        FmTaskDataUpdateTetState(
            FmScene* inScene, FmTetMesh* inTetMesh, const FmVector3& inCenterOfMass,
            uint inNumTets, uint inNumTetTasks,
            bool inRunFracture, bool inUpdatePlasticity, bool inMeshSupportsPlasticity, bool inComputingDeformation)
        {
            FmAtomicWrite(&removedKinematicFlag.val, 0);
            FmAtomicWrite(&maxUnconstrainedSolveIterations.val, 0);
            scene = inScene;
            tetMesh = inTetMesh;
            centerOfMass = inCenterOfMass;
            numTets = inNumTets;
            numTetTasks = inNumTetTasks;
            runFracture = inRunFracture;
            updatePlasticity = inUpdatePlasticity;
            meshSupportsPlasticity = inMeshSupportsPlasticity;
            computingStrainMag = inComputingDeformation;
        }
    };

    FM_WRAPPED_TASK_FUNC(FmTaskFuncUpdateTetsBatch)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT(UPDATE_TET_STATE);

        FmTaskDataUpdateTetState* taskData = (FmTaskDataUpdateTetState *)inTaskData;

        FmTetMesh* tetMesh = taskData->tetMesh;
        FmVector3 centerOfMass = taskData->centerOfMass;
        bool runFracture = taskData->runFracture;
        bool updatePlasticity = taskData->updatePlasticity;
        bool meshSupportsPlasticity = taskData->meshSupportsPlasticity;
        bool computingStrainMag = taskData->computingStrainMag;

        uint beginIdx, endIdx;
        FmGetIndexRange(&beginIdx, &endIdx, (uint)inTaskBeginIndex, FM_UPDATE_TET_BATCH_SIZE, taskData->numTets);

        bool removedKinematicFlag;
        uint maxUnconstrainedSolveIterations;

        FmUpdateTetsRange(&removedKinematicFlag, &maxUnconstrainedSolveIterations, tetMesh, beginIdx, endIdx, centerOfMass, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

        if (removedKinematicFlag)
        {
            FmAtomicWrite(&taskData->removedKinematicFlag.val, 1);
        }

        FmAtomicMax(&taskData->maxUnconstrainedSolveIterations.val, maxUnconstrainedSolveIterations);

        // Mark progress in tet mesh; call FmTaskFuncFinishMeshUpdateFracture() on last task, which will delete taskData
        taskData->progress.TaskIsFinished();
    }

    void FmTaskFuncFinishUpdateTetsAndFracture(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FM_TRACE_SCOPED_EVENT(FINISH_UPDATE_TETS_FRACTURE);

        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateTetState* taskData = (FmTaskDataUpdateTetState *)inTaskData;

        FmScene* scene = taskData->scene;
        FmTetMesh* tetMesh = taskData->tetMesh;
        bool runFracture = taskData->runFracture;
        bool removedKinematicFlag = (FmAtomicRead(&taskData->removedKinematicFlag.val) != 0);
        bool computingStrainMag = taskData->computingStrainMag;
        uint maxUnconstrainedSolveIterations = FmAtomicRead(&taskData->maxUnconstrainedSolveIterations.val);

        if (removedKinematicFlag)
        {
            tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
        }
        tetMesh->maxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;

        FmAccumulateTetQuatAndStrainMag(tetMesh, computingStrainMag);

        if (runFracture)
        {
            FmFractureMesh(scene, tetMesh);
        }

        // Mark progress in island
        taskData->parentTaskData->progress.TaskIsFinished(taskData->parentTaskData);

        delete taskData;
    }

    // Compute tet rotation and rotated stiffness matrix.
    // Compute plastic deformation state change.
    // Create list of tets with sufficient stress for fracture.
    // Set mesh's unconstrained solver iterations as max of tet settings.
    void FmUpdateTetStateAndFracture(FmScene* scene, FmTetMesh* tetMesh, 
        bool runFracture, bool updatePlasticity,
        FmAsyncTaskData* parentTaskData)
    {
        uint numTets = tetMesh->numTets;

        // Clear fracture state
        // Keep existing crack tips to continue fracture from them
        FmAtomicWrite(&tetMesh->numTetsToFracture.val, 0);
        tetMesh->flags &= ~(FM_OBJECT_FLAG_REMOVED_KINEMATIC | FM_OBJECT_FLAG_HIT_MAX_VERTS | FM_OBJECT_FLAG_HIT_MAX_EXTERIOR_FACES);

        FmVector3 centerOfMass = tetMesh->centerOfMass;

        bool meshSupportsFracture = (tetMesh->tetsToFracture != NULL);
        bool meshSupportsPlasticity = (tetMesh->tetsPlasticity != NULL);

        // Set runFracture only if fracture supported on this mesh and scene pointer valid
        runFracture = runFracture && meshSupportsFracture && (scene != NULL);

        bool computingStrainMag = FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_COMPUTE_TET_STRAIN_MAG);
        
        uint numUpdateTetTasks = FmGetNumTasks(tetMesh->numTets, FM_UPDATE_TET_BATCH_SIZE); 

#if FM_ASYNC_THREADING
        if (parentTaskData)
        {
            if (numTets > 128)
            {
                // If enough tets, parallelize using a parallel for
                FmTaskDataUpdateTetState* taskData = new FmTaskDataUpdateTetState(
                    scene, tetMesh, centerOfMass, numTets, numUpdateTetTasks, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

                taskData->progress.Init(numUpdateTetTasks, FmTaskFuncFinishUpdateTetsAndFracture, taskData);
                taskData->parentTaskData = parentTaskData;

                // Using runLoop=true since this is reached within a loop over meshes and is not a tail call
                FmParallelForAsync("UpdateTetsState", FM_TASK_AND_WRAPPED_TASK_ARGS(FmTaskFuncUpdateTetsBatch), NULL, taskData, numUpdateTetTasks, scene->taskSystemCallbacks.SubmitAsyncTask, scene->params.numThreads, true);
            }
            else
            {
                // Update tets and record the completion of task/mesh in global progress
                bool removedKinematicFlag;
                uint maxUnconstrainedSolveIterations;

                FmUpdateTetsRange(&removedKinematicFlag, &maxUnconstrainedSolveIterations, tetMesh, 0, numTets, centerOfMass, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

                if (removedKinematicFlag)
                {
                    tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
                }
                tetMesh->maxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;

                FmAccumulateTetQuatAndStrainMag(tetMesh, computingStrainMag);

                if (runFracture)
                {
                    FmFractureMesh(scene, tetMesh);
                }

                parentTaskData->progress.TaskIsFinished(parentTaskData);
            }
        }
        else
#endif
        {
            bool removedKinematicFlag;
            uint maxUnconstrainedSolveIterations;

            FmUpdateTetsRange(&removedKinematicFlag, &maxUnconstrainedSolveIterations, tetMesh, 0, numTets, centerOfMass, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

            if (removedKinematicFlag)
            {
                tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
            }
            tetMesh->maxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;

            FmAccumulateTetQuatAndStrainMag(tetMesh, computingStrainMag);

            if (runFracture)
            {
                FmFractureMesh(scene, tetMesh);
            }
        }
    }
}
