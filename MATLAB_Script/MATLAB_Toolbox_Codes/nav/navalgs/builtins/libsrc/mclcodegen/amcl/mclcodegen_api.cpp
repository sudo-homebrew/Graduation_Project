/* Copyright 2019-2020 The MathWorks, Inc. */

/**
 * @file
 * External C-API interfaces for Adaptive Monte Carlo Localization (AMCL).
 * To fully support code generation, note that this file needs to be fully
 * compliant with the C89/C90 (ANSI) standard.
 */

#include <math.h> // For mathematical constants

#ifdef BUILDING_LIBMWMCLCODEGEN
#include "amcl/map/amcl_map.h"
#include "amcl/pf/amcl_pf.h"
#include "amcl/pf/amcl_pf_vector.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include "mclcodegen/mclcodegen_api.hpp"
#include "mclcodegen/mclcodegen_data.hpp"
#else
#include "amcl_map.h"
#include "amcl_pf.h"
#include "amcl_pf_vector.h"
#include "amcl_odom.h"
#include "amcl_laser.h"
#include "mclcodegen_api.hpp"
#include "mclcodegen_data.hpp"
#endif

using namespace amcl;
using namespace nav;


static double wrapTo2Pi(double theta) {
    bool positive = theta > 0;
    double thetaWrap = fmod(theta, (2 * M_PI));
    if (positive && thetaWrap == 0) {
        return 2 * M_PI;
    } else {
        return thetaWrap;
    }
}

static double wrapToPi(double theta) {
    return (wrapTo2Pi(theta + M_PI) - M_PI);
}

static double angdiff(double a, double b) {
    double diff = a - b;
    return wrapToPi(diff);
}

static pf_vector_t globalPoseGenerator(void* arg) {
    GlobalSamplingData* gsData = static_cast<GlobalSamplingData*>(arg);
    pf_vector_t sample = pf_vector_zero();

    unsigned int randomIndex = static_cast<unsigned int>(
        gsData->_zeroToOne(gsData->_randGenerator) * gsData->_freeSpaceIndices.size());

    sample.v[0] = MAP_WXGX(gsData->_mapPtr, gsData->_freeSpaceIndices[randomIndex].first);
    sample.v[1] = MAP_WYGY(gsData->_mapPtr, gsData->_freeSpaceIndices[randomIndex].second);
    sample.v[2] = gsData->_zeroToOne(gsData->_randGenerator) * 2 * M_PI - M_PI;
    return sample;
}


EXTERN_C MCL_CODEGEN_API void* mclInitializeWithSeed_real64(real64_T seed) {
    unsigned int intSeed = static_cast<unsigned int>(seed);

    // Global seed for particle filter use. Defined in amcl_pf.c
    pf_global_seed = intSeed;

    // Seed for initialization with mean and covariance
    // intSeed is guaranteed to be >= 0, since it's an unsigned int
    pf_pdf_reset_seed(intSeed);
    
    // Create new MCL data storage object and return it
    auto mcl = new MCLData(intSeed);
    return static_cast<void*>(mcl);
}

EXTERN_C MCL_CODEGEN_API void mclCleanup_real64(void* mclObj) {

	auto mcl = static_cast<MCLData*>(mclObj);

    if (mcl == nullptr)
            return;

    if (mcl->_map != nullptr) {
        map_free(mcl->_map);
        mcl->_map = nullptr;
    }
    if (mcl->_particleFilter != nullptr) {
        pf_free(mcl->_particleFilter);
        mcl->_particleFilter = nullptr;
    }

    mcl->_sensorModel.reset(nullptr);

    delete mcl;
}

EXTERN_C MCL_CODEGEN_API void mclSetMotionModel_real64(void* mclObj, const real64_T* motionParams) {

	auto mcl = static_cast<MCLData*>(mclObj);

    odom_model_t motionModelType = ODOM_MODEL_DIFF;
        mcl->_motionModel.SetModel(motionModelType, motionParams[0], motionParams[1],
                                    motionParams[2],
                          motionParams[3]);
}

EXTERN_C MCL_CODEGEN_API void mclSetOccupancyGrid_real64(void* mclObj,
                                                         real64_T width,
                                                         real64_T height,
                                                         real64_T resolution,
                                                         real64_T originx,
                                                         real64_T originy,
                                                         const real64_T* gridCells) {

	auto mcl = static_cast<MCLData*>(mclObj);

    mcl->_map = map_alloc();
    mcl->_map->size_x = static_cast<int>(width);
    mcl->_map->size_y = static_cast<int>(height);
    mcl->_map->scale = resolution;
    mcl->_map->origin_x = originx;
    mcl->_map->origin_y = originy;

    if (mcl->_testHookForMemory) {
        mcl->_map->cells = nullptr;
    } else {
        mcl->_map->cells = static_cast<map_cell_t*>(malloc(
            sizeof(map_cell_t) * static_cast<size_t>(mcl->_map->size_x * mcl->_map->size_y)));
    }

    if (mcl->_map->cells == nullptr) {
        throw 1; // nav::navalgs::mclInternal::NotEnoughMemoryMap
    }

    for (int i = 0; i < mcl->_map->size_x * mcl->_map->size_y; i++) {
        if (gridCells[i] == 0) {
            mcl->_map->cells[i].occ_state = -1;
        } else if (gridCells[i] == 1) {
            mcl->_map->cells[i].occ_state = 1;
        } else {
            mcl->_map->cells[i].occ_state = 0;
        }
    }

    // Cache free space indices
    mcl->_globalLocalizationData._mapPtr = mcl->_map;
    mcl->_globalLocalizationData._freeSpaceIndices.resize(0);
    for (int i = 0; i < mcl->_map->size_x; i++) {
        for (int j = 0; j < mcl->_map->size_y; j++)
            if (mcl->_map->cells[MAP_INDEX(mcl->_map, i, j)].occ_state == -1) {
                mcl->_globalLocalizationData._freeSpaceIndices.push_back(std::make_pair(i, j));
            }
    }

    if (mcl->_globalLocalizationData._freeSpaceIndices.size() == 0) {
        throw 2; // nav::navalgs::mclInternal::NoFreeIndices
    }
}

EXTERN_C MCL_CODEGEN_API void mclSetSensorModel_real64(void* mclObj,
                                                       real64_T max_beams,
                                                       real64_T z_hit,
                                                       real64_T z_rand,
                                                       real64_T sigma_hit,
                                                       real64_T laser_likelihood_max_dist,
                                                       const real64_T* sensorLimits,
                                                       const real64_T* sensorPoseParams) {

	auto mcl = static_cast<MCLData*>(mclObj);

    mcl->_sensorModel.reset(new AMCLLaser(static_cast<size_t>(max_beams), mcl->_map));
        mcl->_sensorModel->SetModelLikelihoodField(z_hit, z_rand, sigma_hit,
                                                    laser_likelihood_max_dist);

    mcl->_minSensorRange = sensorLimits[0];
        mcl->_maxSensorRange = sensorLimits[1];

    pf_vector_t sensorPose;
    sensorPose.v[0] = sensorPoseParams[0];
    sensorPose.v[1] = sensorPoseParams[1];
    sensorPose.v[2] = sensorPoseParams[2];
    mcl->_sensorModel->SetLaserPose(sensorPose);
}

EXTERN_C MCL_CODEGEN_API void mclInitializePf_real64(void* mclObj,
                                                     real64_T minParticles,
                                                     real64_T maxParticles,
                                                     real64_T alphaSlow,
                                                     real64_T alphaFast,
                                                     real64_T kldErr,
                                                     real64_T kldZ) {
    auto mcl = static_cast<MCLData*>(mclObj);

	size_t maxParticlesInt = static_cast<size_t>(maxParticles);

    // Based on how the memory is allocated in pf_alloc function, check if that much memory is
    // available.
    std::unique_ptr<pf_sample_t[]> memChkParticlesSet1(new (std::nothrow)
                                                           pf_sample_t[maxParticlesInt]);
    std::unique_ptr<pf_kdtree_node_t[]> memChkKdtreeNodesSet1(
        new (std::nothrow) pf_kdtree_node_t[3 * maxParticlesInt]);
    std::unique_ptr<pf_cluster_t[]> memChkClustersSet1(new (std::nothrow)
                                                           pf_cluster_t[maxParticlesInt]);
    std::unique_ptr<pf_sample_t[]> memChkParticlesSet2(new (std::nothrow)
                                                           pf_sample_t[maxParticlesInt]);
    std::unique_ptr<pf_kdtree_node_t[]> memChkKdtreeNodesSet2(
        new (std::nothrow) pf_kdtree_node_t[3 * maxParticlesInt]);
    std::unique_ptr<pf_cluster_t[]> memChkClustersSet2(new (std::nothrow)
                                                           pf_cluster_t[maxParticlesInt]);

    bool isEnoughMemory = memChkParticlesSet2 != nullptr && memChkKdtreeNodesSet2 != nullptr &&
                          memChkClustersSet2 != nullptr && memChkParticlesSet1 != nullptr &&
                          memChkKdtreeNodesSet1 != nullptr && memChkClustersSet1 != nullptr;

    if (mcl->_testHookForMemory) {
        isEnoughMemory = false;
    }

    // Free all memory
    memChkParticlesSet1.reset(nullptr);
    memChkKdtreeNodesSet1.reset(nullptr);
    memChkClustersSet1.reset(nullptr);
    memChkParticlesSet2.reset(nullptr);
    memChkKdtreeNodesSet2.reset(nullptr);
    memChkClustersSet2.reset(nullptr);

    // Throw exception if not enough memory
    if (!isEnoughMemory) {
        throw 1; // nav::navalgs::mclInternal::NotEnoughMemory
    }
    
    if (mcl->_particleFilter != nullptr) {
        pf_free(mcl->_particleFilter);
        mcl->_particleFilter = nullptr;
    }

   mcl->_particleFilter =
        pf_alloc(static_cast<int>(minParticles), static_cast<int>(maxParticles), alphaSlow,
                 alphaFast, static_cast<pf_init_model_fn_t>(globalPoseGenerator),
                 static_cast<void*>(&mcl->_globalLocalizationData));
    mcl->_particleFilter->pop_err = kldErr;
   mcl->_particleFilter->pop_z = kldZ;
}

EXTERN_C MCL_CODEGEN_API void mclSetInitialPose_real64(void* mclObj,
                                                       const real64_T* poseParams,
                                                       const real64_T* covParams) {

    auto mcl = static_cast<MCLData*>(mclObj);

	if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    pf_vector_t mean = pf_vector_zero();
    mean.v[0] = poseParams[0];
    mean.v[1] = poseParams[1];
    mean.v[2] = poseParams[2];

    pf_matrix_t covariance = pf_matrix_zero();
    int z = 0;
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            covariance.m[i][j] = covParams[z];
            z++;
        }
    }

    pf_init(mcl->_particleFilter, mean, covariance);
    mcl->_isPFInitialized = false;
}

EXTERN_C MCL_CODEGEN_API void mclUpdate_real64(void* mclObj,
                                               real64_T rangeCount,
                                               const real64_T* rangesParams,
                                               const real64_T* anglesParams,
                                               const real64_T* poseParams) {
	mclPredict_real64(mclObj, poseParams);
    mclCorrect_real64(mclObj, rangeCount, rangesParams, anglesParams);
    mclResample_real64(mclObj);
}


// Expects 12 doubles
EXTERN_C MCL_CODEGEN_API void mclGetHypothesis_real64(void* mclObj, real64_T* maxWeightHyp) {
    auto mcl = static_cast<MCLData*>(mclObj);

	if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    pf_sample_set_t* set;
    pf_cluster_t* cluster;

    if (mcl->_isPFUpdated && !mcl->_isResampled) {
        pf_update_without_resample(mcl->_particleFilter);
        set = mcl->_particleFilter->sets + (mcl->_particleFilter->current_set + 1) % 2;
    } else {
        set = mcl->_particleFilter->sets + mcl->_particleFilter->current_set;
    }


    double maxWeight = 0.0;
    size_t maxWeightHypIndex = 0;

    // Multiple hypothesis are computed for each cluster of particles.
    // Find hypothesis with maximum weight and return the same.
    std::vector<MCLHypothesis> hypothesis;
    hypothesis.resize(static_cast<size_t>(set->cluster_count));
    for (size_t i = 0; i < static_cast<size_t>(set->cluster_count); i++) {

        if (i >= static_cast<size_t>(set->cluster_count)) {
            throw 2; // nav::navalgs::mclInternal::NoClusterStat
        }
        cluster = set->clusters + i;

        hypothesis[i]._weight = cluster->weight;
        hypothesis[i]._poseMean = cluster->mean;
        hypothesis[i]._poseCovariance = cluster->cov;

        if (hypothesis[i]._weight > maxWeight) {
            maxWeight = hypothesis[i]._weight;
            maxWeightHypIndex = i;
        }
    }

    if (maxWeight > 0.0) {
        maxWeightHyp[0] = (hypothesis[maxWeightHypIndex]._poseMean.v[0]);
        maxWeightHyp[1] = (hypothesis[maxWeightHypIndex]._poseMean.v[1]);
        maxWeightHyp[2] = (hypothesis[maxWeightHypIndex]._poseMean.v[2]);
        size_t k = 3;
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                maxWeightHyp[k] = (hypothesis[maxWeightHypIndex]._poseCovariance.m[i][j]);
                k++;
            }
        }
    } else {
        throw 3; // nav::navalgs::mclInternal::MaxWeightNotPositive
    }
}

EXTERN_C MCL_CODEGEN_API void mclSetParticles_real64(void* mclObj,
                                                     real64_T* particles,
                                                     real64_T* weights) {

	auto mcl = static_cast<MCLData*>(mclObj);

    if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    pf_sample_set_t* set = mcl->_particleFilter->sets + mcl->_particleFilter->current_set;

    int j = 0;
    for (int i = 0; i < set->sample_count; i++) {
        set->samples[i].pose.v[0] = *(particles + j);
        set->samples[i].pose.v[1] = *(particles + j + 1);
        set->samples[i].pose.v[2] = *(particles + j + 2);
        set->samples[i].weight = *(weights + i);
        j += 3;
    }
}

EXTERN_C MCL_CODEGEN_API uint32_T mclGetNumParticles_real64(void* mclObj) {
    auto mcl = static_cast<MCLData*>(mclObj);

    if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }
    
    pf_sample_set_t* set = mcl->_particleFilter->sets + mcl->_particleFilter->current_set;
    return static_cast<uint32_T>(set->sample_count);
}

EXTERN_C MCL_CODEGEN_API void mclGetParticles_real64(void* mclObj, real64_T* particleInfo) {

	auto mcl = static_cast<MCLData*>(mclObj);

    if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    pf_sample_set_t* set = mcl->_particleFilter->sets + mcl->_particleFilter->current_set;
    std::unique_ptr<pf_sample_t[]> memChkParticles;

    if (mcl->_testHookForMemory) {
        memChkParticles = nullptr;
    } else {
        memChkParticles.reset(new (std::nothrow)
                                  pf_sample_t[static_cast<size_t>(set->sample_count)]);
    }

    if (memChkParticles == nullptr) {
        throw 2; // nav::navalgs::mclInternal::NotEnoughMemory
    }
    memChkParticles.reset(nullptr);

    for (int i = 0; i < set->sample_count; i++) {
        particleInfo[i * 4] = set->samples[i].pose.v[0];
        particleInfo[i * 4 + 1] = set->samples[i].pose.v[1];
        particleInfo[i * 4 + 2] = set->samples[i].pose.v[2];
        particleInfo[i * 4 + 3] = set->samples[i].weight;
    }
}



EXTERN_C MCL_CODEGEN_API void mclPredict_real64(void* mclObj, const real64_T* poseParams) {
    
	auto mcl = static_cast<MCLData*>(mclObj);

	if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    pf_vector_t pose = pf_vector_zero();
    pf_vector_t delta = pf_vector_zero();
    pose.v[0] = poseParams[0];
    pose.v[1] = poseParams[1];
    pose.v[2] = poseParams[2];

    if (mcl->_isPFInitialized) {
        // Compute change in pose

        delta.v[0] = pose.v[0] - mcl->_odometryPose.v[0];
        delta.v[1] = pose.v[1] - mcl->_odometryPose.v[1];
        delta.v[2] = angdiff(pose.v[2], mcl->_odometryPose.v[2]);

        // Update only if motion threshold is met
        mcl->_updateLikelihood = fabs(delta.v[0]) > mcl->_xThreshold ||
                            fabs(delta.v[1]) > mcl->_yThreshold ||
                            fabs(delta.v[2]) > mcl->_thetaThreshold;
    }

    if (mcl->_isPFInitialized && mcl->_updateLikelihood) {
        AMCLOdomData odata;
        odata.pose = pose;
        odata.delta = delta;

        // Motion update
        mcl->_motionModel.UpdateAction(mcl->_particleFilter, &odata);

        // Pose at last filter update
        mcl->_odometryPose = pose;
    } else {
        mcl->_isPFUpdated = false;
        mcl->_isResampled = false;
    }

    if (!mcl->_isPFInitialized) {
        // Pose at last filter update
        mcl->_odometryPose = pose;

        // Filter is now initialized
        mcl->_isPFInitialized = true;

        // Should update sensor data
        mcl->_updateLikelihood = true;

        mcl->_resampleCount = 0;
    }
}

EXTERN_C MCL_CODEGEN_API void mclCorrect_real64(void* mclObj,
                                                real64_T rangeCount,
                                                const real64_T* rangesParams,
                                                const real64_T* anglesParams) {

	auto mcl = static_cast<MCLData*>(mclObj);

    if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    if (mcl->_sensorModel == nullptr) {
        throw 2; // nav::navalgs::mclInternal::AssignSensorModel
    }

    // Return if likelihood is not to be updated
    if (!mcl->_updateLikelihood) {
        return;
    }

    AMCLLaserData laserScan;
    laserScan.range_count = static_cast<int>(rangeCount);
    laserScan.range_max = mcl->_maxSensorRange;
    laserScan.sensor = mcl->_sensorModel.get();

    laserScan.ranges = new double[static_cast<size_t>(laserScan.range_count)][2];
    for (int i = 0; i < laserScan.range_count; i++) {

        if (rangesParams[i] < mcl->_minSensorRange) {
            laserScan.ranges[i][0] = mcl->_maxSensorRange;
        } else {
            laserScan.ranges[i][0] = rangesParams[i];
        }

        laserScan.ranges[i][1] = anglesParams[i];
    }

    // Measurement update
    mcl->_sensorModel->UpdateSensor(mcl->_particleFilter, &laserScan);

    mcl->_isPFUpdated = true;
    mcl->_updateLikelihood = false;
}

EXTERN_C MCL_CODEGEN_API void mclResample_real64(void* mclObj) {
    auto mcl = static_cast<MCLData*>(mclObj);
    mcl->_isResampled = false;

    if (mcl->_isPFUpdated) {
        bool triggerResampling = (++mcl->_resampleCount % mcl->_resampleInterval) == 0;

        if (triggerResampling) {
            pf_update_resample(mcl->_particleFilter);
            mcl->_isResampled = true;
        }
    }
}

EXTERN_C MCL_CODEGEN_API void mclSetResamplingInterval_real64(void* mclObj, real64_T interval) {
    auto mcl = static_cast<MCLData*>(mclObj);
    mcl->_resampleInterval = static_cast<uint32_T>(interval);
}

EXTERN_C MCL_CODEGEN_API real64_T mclGetResamplingInterval_real64(void* mclObj) {
    auto mcl = static_cast<MCLData*>(mclObj);
    return static_cast<real64_T>(mcl->_resampleInterval);
}

EXTERN_C MCL_CODEGEN_API real64_T mclGetOccupancyWorld_real64(void* mclObj,
                                                              real64_T x,
                                                              real64_T y) {
    
	auto mcl = static_cast<MCLData*>(mclObj);

	if (mcl->_map == nullptr) {
        throw 1; // nav::navalgs::mclInternal::AssignMap
    }

    double occ = -2.0;

    int i = static_cast<int>(MAP_GXWX(mcl->_map, x));
    int j = static_cast<int>(MAP_GYWY(mcl->_map, y));

    if (MAP_VALID(mcl->_map, i, j)) {
        occ = mcl->_map->cells[static_cast<int>(MAP_INDEX(mcl->_map, i, j))].occ_state;
    }

    return occ;
}

EXTERN_C MCL_CODEGEN_API void mclSetUpdateThresholds_real64(void* mclObj,
                                                            real64_T x,
                                                            real64_T y,
                                                            real64_T theta) {
    auto mcl = static_cast<MCLData*>(mclObj);
    mcl->_xThreshold = x;
    mcl->_yThreshold = y;
    mcl->_thetaThreshold = theta;
}

EXTERN_C MCL_CODEGEN_API void mclGetUpdateThresholds_real64(void* mclObj,
                                                            real64_T* x,
                                                            real64_T* y,
                                                            real64_T* theta) {
    auto mcl = static_cast<MCLData*>(mclObj);
    *x = mcl->_xThreshold;
    *y = mcl->_yThreshold;
    *theta = mcl->_thetaThreshold;
}

EXTERN_C MCL_CODEGEN_API boolean_T mclIsUpdated_real64(void* mclObj) {
    auto mcl = static_cast<MCLData*>(mclObj);
    return mcl->_isPFUpdated;
}

EXTERN_C MCL_CODEGEN_API boolean_T mclIsResampled_real64(void* mclObj) {
    auto mcl = static_cast<MCLData*>(mclObj);
    return mcl->_isResampled;
}

EXTERN_C MCL_CODEGEN_API void mclGlobalLocalization_real64(void* mclObj) {
    auto mcl = static_cast<MCLData*>(mclObj);

    if (mcl->_particleFilter == nullptr) {
        throw 1; // nav::navalgs::mclInternal::InitializePF
    }

    if (mcl->_map == nullptr) {
        throw 2; // nav::navalgs::mclInternal::AssignMap
    }

    // Initialize pf for global localization
    pf_init_model(mcl->_particleFilter, static_cast<pf_init_model_fn_t>(globalPoseGenerator),
                  static_cast<void*>(&mcl->_globalLocalizationData));
    mcl->_isPFInitialized = false;
}

EXTERN_C MCL_CODEGEN_API void mclSetTestHookForMemory_real64(void* mclObj, boolean_T value) {
    auto mcl = static_cast<MCLData*>(mclObj);
    mcl->_testHookForMemory = value;
}
