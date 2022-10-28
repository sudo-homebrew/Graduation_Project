/* Copyright 2019-2020 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for Adaptive Octomap.
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomapcodegen/octomapcodegen_api.hpp"
#include "octomapcodegen/octomapcodegen.hpp"
#else
#include "octomapcodegen_api.hpp"
#include "octomapcodegen.hpp"
#endif

void* octomapInitialize_real64(real64_T resolution) {
     return static_cast<void*>(new nav::octomapcodegen(resolution));    
}
    
void octomapCleanup_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    delete tempMap;
}

EXTERN_C OCTOMAP_CODEGEN_API uint32_T octomapGetNumLeafNodes_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapGetNumLeafNodes();
}

EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetResolution_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapGetResolution();
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapSetResolution_real64(void* map, real64_T resolution) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapSetResolution(resolution);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapGetMapDimensions_real64(void* map, real64_T* dimensions) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapGetMapDimensions(dimensions);    
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapUpdateNodeBoolean_real64(void* map, real64_T* xyz, boolean_T occupied, boolean_T lazyEval) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapUpdateNodeBoolean(xyz, occupied, lazyEval);    
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapUpdateNodeDouble_real64(void* map, real64_T* xyz, real64_T probUpdate, boolean_T lazyEval) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapUpdateNodeDouble(xyz, probUpdate, lazyEval);    
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapSetNodeValue_real64(void* map, real64_T* xyz, real64_T prob, boolean_T lazyEval) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapSetNodeValue(xyz, prob, lazyEval);    
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapGetOccupancy_real64(void* map, real64_T* xyz, uint32_T nRows, real64_T* occupancyValues) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapGetOccupancy(xyz, nRows, occupancyValues);        
}

EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapInsertPointCloud_real64(void* map, real64_T* origin,
                          real64_T* points,
                          uint32_T nPoints,
                          real64_T maxRange,
                          boolean_T lazyEval,
                          boolean_T discretize) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->insertPointCloudImpl_real64(origin, points, nPoints, maxRange, lazyEval, discretize);
}

EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapInsertPointCloud_real32(void* map, real64_T* origin,
                          real32_T* points,
                          uint32_T nPoints,                          
                          real64_T maxRange,
                          boolean_T lazyEval,
                          boolean_T discretize) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->insertPointCloudImpl_real32(origin, points, nPoints, maxRange, lazyEval, discretize);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapUpdateInternalOccupancy_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapUpdateInternalOccupancy();
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapPrune_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapPrune();
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapToMaxLikelihood_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapToMaxLikelihood();
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapExtractVisualizationData_real64(void* map, real64_T maxDepth, real64_T* outData) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapExtractVisualizationData(maxDepth, outData);    
}

EXTERN_C OCTOMAP_CODEGEN_API uint32_T octomapMmemoryUsage_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapMmemoryUsage();
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapInflate_real64(void* map, real64_T radius, real64_T occThreshold) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);    
    tempMap->octomapInflate(radius, occThreshold);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapSetClampingThreshold_real64(void* map, real64_T clampingThresMin, real64_T clampingThresMax) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapSetClampingThreshold(clampingThresMin, clampingThresMax);
}

EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapReadBinary_real64(void* map, const char* filename) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapReadBinary(filename);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapWriteBinary_real64(void* map, const char* filename) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapWriteBinary(filename);
}

EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapRead_real64(void* map, const char* filename) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapRead(filename);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapWrite_real64(void* map, const char* filename) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapWrite(filename);
}

EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetProbHit_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapGetProbHit();
}

EXTERN_C OCTOMAP_CODEGEN_API real64_T octomapGetProbMiss_real64(void* map) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapGetProbMiss();
}

EXTERN_C OCTOMAP_CODEGEN_API uint32_T octomapGetSizeSerializationData_real64(void* map){
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapGetSizeSerializationData();
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapSerialization_real64(void* map, char* data) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapSerialization(data);
}

EXTERN_C OCTOMAP_CODEGEN_API boolean_T octomapDeserialization_real64(void* map, char* data, uint32_T n) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    return tempMap->octomapDeserialization(data, n);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapDeserializationFullROSMsgData_real64(void* map, real64_T resolution, char* data, uint32_T n) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapDeserializationFullROSMsgData(resolution, data, n);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapDeserializationBinaryROSMsgData_real64(void* map, real64_T resolution, char* data, uint32_T n) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapDeserializationBinaryROSMsgData(resolution, data, n);
}

EXTERN_C OCTOMAP_CODEGEN_API void octomapGetRayIntersection_real64(
        void* map,
        real64_T* ptStart,
        real64_T* ptDirections,
        uint32_T nPtDirection,
        real64_T occupiedThreshold,
        boolean_T ignoreUnknownCells,
        real64_T maxRange,
        real64_T* outData) {
    nav::octomapcodegen* tempMap = static_cast<nav::octomapcodegen*>(map);
    tempMap->octomapGetRayIntersection(ptStart, ptDirections, nPtDirection, occupiedThreshold, ignoreUnknownCells, maxRange, outData);    
}
