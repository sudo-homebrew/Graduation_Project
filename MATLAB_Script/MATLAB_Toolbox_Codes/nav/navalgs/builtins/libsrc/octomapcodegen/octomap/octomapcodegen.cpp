/* Copyright 2019-2021 The MathWorks, Inc. */

#ifdef BUILDING_LIBMWOCTOMAPCODEGEN
#include "octomap/OcTree.h"
#include "octomap/octomap_utils.h"
#include "octomapcodegen/octomapcodegen.hpp"
#else
#include "octomapcodegen.hpp"
#include "OcTree.h"
#include "octomap_utils.h"
#endif

#include <sstream>
#include <cmath>

namespace nav {
    
    octomapcodegen::octomapcodegen(real64_T resolution): _map(new octomap::OcTree(resolution)) {
    }
    octomapcodegen::~octomapcodegen(){
    }
    
    real64_T octomapcodegen::octomapGetResolution(){
        return _map->getResolution();
    }
    
    void octomapcodegen::octomapSetResolution(real64_T resolution){
        _map->setResolution(resolution);
    }
    
    uint32_T octomapcodegen::octomapGetNumLeafNodes(){
        return static_cast<uint32_T>(_map->getNumLeafNodes());
    }
    
    void octomapcodegen::octomapGetMapDimensions(real64_T* dimensions) {
        double xMin, yMin, zMin;
        double xMax, yMax, zMax;
        
        _map->getMetricMin(xMin, yMin, zMin);
        _map->getMetricMax(xMax, yMax, zMax);
        
        dimensions[0] = xMin;
        dimensions[1] = yMin;
        dimensions[2] = zMin;
        
        dimensions[3] = xMax;
        dimensions[4] = yMax;
        dimensions[5] = zMax;
    }
    
    void octomapcodegen::octomapUpdateNodeBoolean(real64_T* xyz, boolean_T occupied, boolean_T lazyEval) {
        real32_T logOddsUpdate = octomap::logodds(_map->getProbMiss());
        if (occupied) {
            logOddsUpdate = octomap::logodds(_map->getProbHit());
        }
        _map->updateNode(xyz[0], xyz[1], xyz[2], logOddsUpdate, lazyEval);
    }
    
    void octomapcodegen::octomapUpdateNodeDouble(real64_T* xyz, real64_T probUpdate, boolean_T lazyEval) {
        real32_T logOddsUpdate = octomap::logodds(probUpdate);
        _map->updateNode(xyz[0], xyz[1], xyz[2], logOddsUpdate, lazyEval);
    }
    
    void octomapcodegen::octomapSetNodeValue(real64_T* xyz, real64_T prob, boolean_T lazyEval) {
        real32_T logOdds = octomap::logodds(prob);
        _map->setNodeValue(xyz[0], xyz[1], xyz[2], logOdds, lazyEval);
    }
    
    void octomapcodegen::octomapGetOccupancy(real64_T* xyz, uint32_T nRows, real64_T* occupancyValues) {
        // For each row find the occupancy value and return it
        for (uint32_T i = 0; i < nRows; ++i) {
            // Since MATLAB stores matrices in column-major, data elements in a row are separated
            // by "rows" elements
            octomap::OcTreeNode* node{_map->search(xyz[i], xyz[nRows + i], xyz[2 * nRows + i])};
            if (node != nullptr) {
                occupancyValues[i] = node->getOccupancy();
            } else {
                occupancyValues[i] = 0.5;
            }
        }
    }

    void octomapcodegen::octomapUpdateInternalOccupancy() {
        _map->updateInnerOccupancy();
    }
    
    void octomapcodegen::octomapPrune() {
        _map->prune();
    }
    
    void octomapcodegen::octomapToMaxLikelihood() {
        _map->toMaxLikelihood();
    }        
    
    void octomapcodegen::octomapExtractVisualizationData(real64_T maxDepth, real64_T* outData) {
        uint32_T numLeaves{octomapGetNumLeafNodes()};
        uint32_T i{0};
        for (auto it = _map->begin_leafs(static_cast<unsigned char>(maxDepth)), end = _map->end_leafs();
        it != end; ++it) {
            outData[i] = it.getX();
            outData[numLeaves + i] = it.getY();
            outData[2 * numLeaves + i] = it.getZ();
            outData[3 * numLeaves + i] = it.getSize();
            outData[4 * numLeaves + i] = it->getOccupancy();
            outData[5 * numLeaves + i] = it.getDepth();
            ++i;
        }
    }
    
    uint32_T octomapcodegen::octomapMmemoryUsage() {
        return static_cast<uint32_T>(_map->memoryUsage());
    }        

    void octomapcodegen::octomapInflate(real64_T radius, real64_T occThreshold) {
        // get origin points and occupancy values for every node in the tree
        uint32_T numLeaves{0};
        uint32_T i = 0;

        // expand collapsed occupied nodes until all occupied leaves are at maximum depth
        std::vector<octomap::OcTreeNode*> collapsedOccupiedNodes;
        do {
            collapsedOccupiedNodes.clear();
            // collect all occupied nodes
            for (auto it = _map->begin_leafs(0); it != _map->end_leafs(); ++it) {
                if (_map->isNodeOccupied(*it) && it.getDepth() < _map->getTreeDepth()) {
                    collapsedOccupiedNodes.push_back(&(*it));
                }
            }
            // expand these occupied nodes
            for (auto it = collapsedOccupiedNodes.begin(); it != collapsedOccupiedNodes.end(); ++it) {
                _map->expandNode(*it);
            }
        } while (collapsedOccupiedNodes.size() > 0);

        numLeaves = octomapGetNumLeafNodes();

        // Create numLeaves x 4 matrix (x,y,z,value)
        std::vector<std::vector<real64_T>> pointDataVector(numLeaves, std::vector<real64_T>(4, 0));
        i = 0; // counter for adding leaf data to the pointDataVector
        for (auto it = _map->begin_leafs(0), // 0 for max depth -> all the way down
                end = _map->end_leafs(); it != end; ++it) {
            pointDataVector[i][0] = it.getX();
            pointDataVector[i][1] = it.getY();
            pointDataVector[i][2] = it.getZ();
            pointDataVector[i][3] = it->getOccupancy();
            ++i;
        }

        // make a cube for everyone to use
        real64_T res = _map->getResolution();
        // Number of tree nodes in the length of a cube side = incoming radius * 1/(least resolution of
        // one node) Need to double it to make it a diameter Need it to be odd for calculations to make
        // sense
        uint32_T NodeSideLength = static_cast<uint32_T>(2.0 * (radius / res)) + 1;
        uint32_T nodesInCube = static_cast<uint32_T>(pow(NodeSideLength, 3.0));
        std::vector<std::vector<real64_T>> cubeDataVector(nodesInCube, std::vector<double>(3, 0));

        std::map<std::vector<double>, real64_T> uniqueNodesMap;

        // go through list of points
        for (i = 0; i < numLeaves; i++) {
            // if point's occupancy is >= occupancy threshold, make a cube for it and insert that cube.
            // else, skip it
            real64_T pointOccupancy = pointDataVector[i][3];
            if (pointOccupancy >= occThreshold) {

                // fill cube with poses, better than allocating and de-allocating a cube every time
                // iterate through cube levels
                for (uint32_T level_iterator = 0; level_iterator < NodeSideLength; level_iterator++) {
                    // iterate through rows in a level. this is painful there must be a better way. but
                    // i want SOMETHING
                    for (uint32_T row_iterator = 0; row_iterator < NodeSideLength; row_iterator++) {
                        // iterate through columns in a row
                        for (uint32_T col_iterator = 0; col_iterator < NodeSideLength; col_iterator++) {
                            // x-coord
                            uint32_T coordIdx =
                                    static_cast<uint32_T>((level_iterator * pow(NodeSideLength, 2.0) +
                                    row_iterator * (NodeSideLength) + col_iterator));
                            double offset =
                                    (-(static_cast<double>(NodeSideLength) - 1.0) / 2.0 + col_iterator) *
                                    res;
                            cubeDataVector[coordIdx][0] = pointDataVector[i][0] + offset;

                            // y-coord
                            offset =
                                    (-(static_cast<double>(NodeSideLength) - 1.0) / 2.0 + row_iterator) *
                                    res;
                            cubeDataVector[coordIdx][1] = pointDataVector[i][1] + offset;

                            // z-coord
                            offset =
                                    (-(static_cast<double>(NodeSideLength) - 1.0) / 2.0 + level_iterator) *
                                    res;
                            cubeDataVector[coordIdx][2] = pointDataVector[i][2] + offset;

                        } // done filling row
                    } // done filling level
                } // done filling cube

                //  find unique nodes              
                for (uint32_T ins_iterator = 0; ins_iterator < nodesInCube; ins_iterator++) {       

                    std::vector<double> nodeCenter(cubeDataVector[ins_iterator]);
                    
                    // If the nodeCenter does not exist then create an entry for it in the uniqueNodesMap
                    // (if exists) Check if the nodeCenter's occupancy is less that pointOccupancy --> then only update
                    if (uniqueNodesMap.find(nodeCenter) == uniqueNodesMap.end() || uniqueNodesMap[nodeCenter] < pointOccupancy) {
                        uniqueNodesMap[nodeCenter] = pointOccupancy;
                    }
                } // end of finding unique nodes
            } // end of if-statement for whether you're gonna put in a cube or not
        } // end of loop that goes through list of points

        // updateNode(...) call on the unique cubes only
        for (auto iter_unique=uniqueNodesMap.begin(); iter_unique != uniqueNodesMap.end(); ++ iter_unique) {
            
            // carefully insert cube using updateNode 
            // if current occupancy of the point is greater than what you'd be replacing it with
            // (pointOccupancy),  don't replace it!!!  also remember that updateNode using the
            // log-odds value and not the probability

            // Extract information from the std::map
            std::vector<double> nodeCenter(iter_unique->first);
            real64_T nodeOccupancy(iter_unique->second);
            real64_T occ(0.0);

            octomap::point3d cubeCenter(static_cast<float>(nodeCenter[0]),
                                        static_cast<float>(nodeCenter[1]),
                                        static_cast<float>(nodeCenter[2]));

            // Search for the cube center
            octomap::OcTreeNode* node{_map->search(cubeCenter)};

            // If node exists find the occupancy
            if (node != nullptr) {
                occ = node->getOccupancy();
            }

            if (occ < nodeOccupancy) {
                real32_T pointLogOdds = octomap::logodds(nodeOccupancy);
                _map->updateNode(cubeCenter, pointLogOdds, true); // lazyEval = true bc we're doing a LOT of updates and
                // we should just smooth out the tree afterwards
            }
        }

        _map->updateInnerOccupancy();

        // As expandNode() was used to expand collapsed occupied nodes, call prune() to collapse nodes where possible.
        _map->prune();
    }
    
    void octomapcodegen::octomapSetClampingThreshold(real64_T clampingThresMin, real64_T clampingThresMax) {
        _map->setClampingThresMin(clampingThresMin);
        _map->setClampingThresMax(clampingThresMax);
    }
    
    boolean_T octomapcodegen::octomapReadBinary(const char* name) {
        std::string filename = name;
        std::ifstream binary_infile(filename.c_str(), std::ios_base::binary);
        return readBinaryImpl(binary_infile);
    }
    
    void octomapcodegen::octomapWriteBinary(const char* name) {
        std::string filename = name;
        std::ofstream binary_outfile(filename.c_str(), std::ios_base::binary);
        writeBinaryImpl(binary_outfile);
        binary_outfile.close();
    }
    
    boolean_T octomapcodegen::octomapRead(const char* name) {
        std::string filename = name;
        std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
        return readImpl(file);
    }
    
    void octomapcodegen::octomapWrite(const char* name) {
        std::string filename = name;
        std::ofstream file(filename.c_str(), std::ios_base::out | std::ios_base::binary);
        writeImpl(file);
        file.close();
    }
    
    real64_T octomapcodegen::octomapGetProbHit() {
        return _map->getProbHit();
    }
    
    real64_T octomapcodegen::octomapGetProbMiss() {
        return _map->getProbMiss();
    }
    
    uint32_T octomapcodegen::octomapGetSizeSerializationData(){
        std::stringstream datastream;
        writeImpl(datastream);
        
        std::string s = datastream.str();
        
        return static_cast<uint32_T>(s.size());
    }
    
    void octomapcodegen::octomapSerialization(char* charArrayRef) {
        // write to in-memory buffer  
        std::stringstream datastream;
        writeImpl(datastream);
        
        std::string s = datastream.str();
        
        size_t n = s.size();       
        
        // copying the contents of the
        // string to char array
        for(size_t i = 0; i < n; ++i){
            charArrayRef[i] = s[i];
        }
    }
    
    boolean_T octomapcodegen::octomapDeserialization(char* data, uint32_T n) {
        OneShotReadBuf buf(data, n);
        std::istream reader(&buf);
        
        // read from buffer
        return readImpl(reader);
    }
    
    void octomapcodegen::octomapDeserializationFullROSMsgData(real64_T resolution, char* data, uint32_T n) {
        OneShotReadBuf buf(data, n);
        std::istream reader(&buf);
        
        // read from buffer
        auto newOctree = new octomap::OcTree(resolution);
        newOctree->readData(reader);
        _map.reset(newOctree);
    }
    
    void octomapcodegen::octomapDeserializationBinaryROSMsgData(real64_T resolution, char* data, uint32_T n) {
        OneShotReadBuf buf(data, n);
        std::istream reader(&buf);
        
        // read from buffer
        auto newOctree = new octomap::OcTree(resolution);
        newOctree->readBinaryData(reader);
        _map.reset(newOctree);
    }    
    
    void octomapcodegen::octomapGetRayIntersection(
            real64_T* ptStart,
            real64_T* ptDirections,
            uint32_T nPtDirection,
            real64_T occupiedThreshold,
            boolean_T ignoreUnknownCells,
            real64_T maxRange,
            real64_T* outData) {
        // set the occupancy threshold so that raycast can return correct result
        // that matches the MATLAB occupancy threshold definition
        _map->setOccupancyThres(occupiedThreshold);
        
        octomap::point3d origin{
            static_cast<float>(ptStart[0]),
                    static_cast<float>(ptStart[1]),
                    static_cast<float>(ptStart[2])};
                    
        for (uint32_T idx = 0; idx < nPtDirection; idx++)
        {
            // iterate through each ray
            octomap::point3d direction{
                static_cast<float>(ptDirections[idx]),
                        static_cast<float>(ptDirections[idx + nPtDirection]),
                        static_cast<float>(ptDirections[idx + 2 * nPtDirection]) };

            // first perform raycast to find the collision cell
            octomap::point3d output;
            bool collide = _map->castRay(origin, direction, output, ignoreUnknownCells, maxRange);

            // find the occupancy status of the collision cell
            auto node = _map->search(output);
            if (node != nullptr) {
                outData[idx] = node->getOccupancy();
            }
            else {
                outData[idx] = 0.5;
            }

            if (collide || (!ignoreUnknownCells && !node))
            {
                // find the intersecting point on the collision cell
                octomap::point3d intersectionPt;
                _map->getRayIntersection(origin, direction, output, intersectionPt);

                outData[idx + nPtDirection] = static_cast<double>(intersectionPt.x());
                outData[idx + 2 * nPtDirection] = static_cast<double>(intersectionPt.y());
                outData[idx + 3 * nPtDirection] = static_cast<double>(intersectionPt.z());
            }
            else
            {
                // when last cell is free, unknown or at boundary
                if (maxRange < 0)
                {
                    maxRange = 2*_map->getResolution()*std::pow(2, _map->getTreeDepth());
                }

                // return the end point at max range
                auto endPt = direction.normalize()*static_cast<float>(maxRange) + origin;
                outData[idx + nPtDirection] = static_cast<double>(endPt.x());
                outData[idx + 2 * nPtDirection] = static_cast<double>(endPt.y());
                outData[idx + 3 * nPtDirection] = static_cast<double>(endPt.z());
            }
        }
    }
        
    template <typename TfloatPoint>
        bool octomapcodegen::insertPointCloudImpl(double const* origin,
        TfloatPoint const* points,
        uint32_T rows,
        double maxRange,
        bool lazyEval,
        bool discretize) {
            // Define frame origin position
            octomath::Vector3 trans{
                static_cast<float>(origin[0]), // float x
                        static_cast<float>(origin[1]), // float y
                        static_cast<float>(origin[2])  // float z
            };

            // Define frame origin orientation
            octomath::Quaternion rot{
                static_cast<float>(origin[3]), // float quat_w
                        static_cast<float>(origin[4]), // float quat_x
                        static_cast<float>(origin[5]), // float quat_y
                        static_cast<float>(origin[6])  // float quat_z

            };

            // Define frame origin as 6-DOF pose
            octomath::Pose6D frameOrigin{trans, rot};

            // Place the sensor origin at the origin of the frame
            octomap::point3d sensorOrigin{float(0), float(0), float(0)};

            octomap::Pointcloud ptCloud;
            bool isNotFiniteErrorThrown(false);

            // Processing one row at a time, i.e. casting only 3 values in a single
            // row of the points matrix, this is much more efficient in terms of
            // memory consumption compared to casting the whole points mxArray into
            // double in MATLAB code

            for (uint32_T i = 0; i < rows; ++i) {
                // Ensure the [x,y,z] to be added to the list of pointCloud is finite (i.e., non Nan or not
                // Inf) NOTE: Octomap throws std::cerr when any of the [x,y,z] is nan/inf and does ignore
                // these points during insertion operation. Hence, using this IF check it is ensured that
                // the std::cerr is not witnessed by the customer. The kind of warning being suppressed:
                // WARNING: coordinates ( (98.2262 81.5177 0) -> (-nan(ind) -nan(ind) -nan(ind))) out of
                // bounds in computeRayKey

                if (std::isfinite(points[i]) && isfinite(points[rows + i]) && isfinite(points[2 * rows + i])) {
                    ptCloud.push_back(static_cast<float>(points[i]), static_cast<float>(points[rows + i]),
                            static_cast<float>(points[2 * rows + i]));
                } else {
                    if (!isNotFiniteErrorThrown) {
                        isNotFiniteErrorThrown = true;
                        return isNotFiniteErrorThrown;
                    }
                }
            }
        _map->insertPointCloud(ptCloud, sensorOrigin, frameOrigin, maxRange, lazyEval, discretize);
        return isNotFiniteErrorThrown;
    }
    
    bool octomapcodegen::insertPointCloudImpl_real64(real64_T* origin,
            real64_T* points,
            uint32_T nPoints,
            real64_T maxRange,
            boolean_T lazyEval,
            boolean_T discretize){
        return insertPointCloudImpl(origin, points, nPoints, maxRange, lazyEval, discretize);
    }
    
    bool octomapcodegen::insertPointCloudImpl_real32(real64_T* origin,
            real32_T* points,
            uint32_T nPoints,
            real64_T maxRange,
            boolean_T lazyEval,
            boolean_T discretize){
        return insertPointCloudImpl(origin, points, nPoints, maxRange, lazyEval, discretize);
    }
    
    // read from istream (binary)
    bool octomapcodegen::readBinaryImpl(std::istream& buf) {

        bool filePathStatus = true;
        std::unique_ptr<octomap::OcTree> newMap{ new octomap::OcTree(1) };
        newMap->setClampingThresMax(_map->getClampingThresMax());
        newMap->setClampingThresMin(_map->getClampingThresMin());
        if (newMap->readBinary(buf))
        {
            _map = std::move(newMap);
        }
        else
        {
            filePathStatus = false;
        }
        return filePathStatus;
    }

    // write to ostream (binary)
    void octomapcodegen::writeBinaryImpl(std::ostream& buf) {
        _map->writeBinary(buf);
    }

    // write to ostream
    void octomapcodegen::writeImpl(std::ostream& buf) {
        _map->write(buf);
    }

    // read from istream
    bool octomapcodegen::readImpl(std::istream& buf) {

        bool filePathStatus = true;
        auto newMap = octomap::AbstractOcTree::read(buf);
        if (newMap)
        {
            auto newOcTree = dynamic_cast<octomap::OcTree*>(newMap);
            _map.reset(newOcTree);
        }
        else
        {
            filePathStatus = false;
        }
        return filePathStatus;
    }
} // namespace nav
