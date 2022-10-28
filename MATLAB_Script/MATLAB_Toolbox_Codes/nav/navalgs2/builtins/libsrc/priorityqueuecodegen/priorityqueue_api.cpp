/* Copyright 2019-2021 The MathWorks, Inc. */
/**
 *@brief PriorityQueueImpl codegen C interface.
 *
 */


#ifdef BUILDING_LIBMWPRIORITYQUEUECODEGEN
#include "priorityqueuecodegen/priorityqueue_impl.hpp"
#include "priorityqueuecodegen/priorityqueue_api.hpp"
#else
#include "priorityqueue_impl.hpp"
#include "priorityqueue_api.hpp"
#endif

/// nav::PriorityQueueImpl
 
EXTERN_C PRIORITYQUEUE_CODEGEN_API void* priorityqueuecodegen_constructPQ(const real64_T dim, const real64_T primeIdx) {
    int dimension = static_cast<int>(dim);
    int primeIndex = static_cast<int>(primeIdx);
    // returns the pointer for PQ implementation (min heap)
    return static_cast<void*>(new nav::PriorityQueueImpl(dimension, primeIndex));
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_push(void* objPtr, const real64_T* nodeData) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    return static_cast<real64_T>(pqObjPtr->push(nodeData));
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_top(void* objPtr, real64_T* nodeData, real64_T* nodeId) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    nav::Node node = pqObjPtr->top();

    // data marshaling
    *nodeId = static_cast<real64_T>(node.first);
    for (size_t k = 0; k < node.second.size(); k++) {
        nodeData[k] = node.second[k];
    }
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_pop(void* objPtr) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    pqObjPtr->pop();
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_size(void* objPtr) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    return static_cast<real64_T>(pqObjPtr->size());
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API boolean_T priorityqueuecodegen_isEmpty(void* objPtr) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    return pqObjPtr->isEmpty();
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_getDataDim(void* objPtr) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    return static_cast<real64_T>(pqObjPtr->getDataDim());
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_destructPQ(void* objPtr) {
    nav::PriorityQueueImpl* pqObjPtr = static_cast<nav::PriorityQueueImpl*>(objPtr);
    if (pqObjPtr != nullptr) {
        delete pqObjPtr;
    }
}

/// nav::NodeMap

EXTERN_C PRIORITYQUEUE_CODEGEN_API void* priorityqueuecodegen_constructNodeMap(const real64_T dim) {
    int dimension = static_cast<int>(dim);
    return static_cast<void *>(new nav::NodeMap(dimension));
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_nodemap_insertNode(void* objPtr,
                                                                                    const real64_T* newNodeData, 
                                                                                    const real64_T parentId) {
    nav::NodeMap* nodeMapObjPtr = static_cast<nav::NodeMap*>(objPtr);
    int pid = static_cast<int>(parentId);
    return static_cast<real64_T>(nodeMapObjPtr->insertNode(newNodeData, pid));
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_nodemap_getNodeData(void* objPtr,
                                                                                    const real64_T nodeId,
                                                                                    real64_T* data,
                                                                                    real64_T* parentId) {
    nav::NodeMap* nodeMapObjPtr = static_cast<nav::NodeMap*>(objPtr);
    int nid = static_cast<int>(nodeId);
    int pid;
    std::vector<real64_T> tmp = nodeMapObjPtr->getNodeData(nid, pid);
    *parentId = static_cast<real64_T>(pid);
    // data marshalling
    size_t k = 0;
    for (real64_T& d : tmp) {
        data[k++] = d;
    }
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_nodemap_traceBack(void* objPtr,
                                                                               const real64_T idx,
                                                                               real64_T* pathNodeData,
                                                                               real64_T* numPathNodes) {
    nav::NodeMap* nodeMapObjPtr = static_cast<nav::NodeMap*>(objPtr);
    std::vector<std::vector<double>> pth = nodeMapObjPtr->traceBack(static_cast<int>(idx));
    *numPathNodes = static_cast<real64_T>(pth.size());
    // data marshaling
    size_t k = 0;
    std::vector<std::vector<double>>::iterator it = pth.begin();
    for (; it != pth.end(); ++it ) {
        for (size_t j = 0; j < it->size(); j++) {
            pathNodeData[k++] = (*it)[j];
        }
    }
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_nodemap_getDataDim(void* objPtr) {
    nav::NodeMap* nodeMapObjPtr = static_cast<nav::NodeMap*>(objPtr);
    return static_cast<real64_T>(nodeMapObjPtr->getDataDim());
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_nodemap_getNumNodes(void* objPtr) {
    nav::NodeMap* nodeMapObjPtr = static_cast<nav::NodeMap*>(objPtr);
    return static_cast<real64_T>(nodeMapObjPtr->getNumNodes());
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_destructNodeMap(void* objPtr) {
    nav::NodeMap* nodeMapObjPtr = static_cast<nav::NodeMap*>(objPtr);
    if (nodeMapObjPtr != nullptr) {
        delete nodeMapObjPtr;
    }
}


/// nav::SimpleMap

EXTERN_C PRIORITYQUEUE_CODEGEN_API void* priorityqueuecodegen_constructSimpleMap(const real64_T dim) {
    int dimension = static_cast<int>(dim);
    return static_cast<void *>(new nav::SimpleMap(dimension));
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_destructSimpleMap(void* objPtr) {
    nav::SimpleMap* mapObjPtr = static_cast<nav::SimpleMap*>(objPtr);
    if (mapObjPtr != nullptr) {
        delete mapObjPtr;
    }
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_simplemap_getDataDim(void* objPtr) {
    nav::SimpleMap* mapObjPtr = static_cast<nav::SimpleMap*>(objPtr);
    return static_cast<real64_T>(mapObjPtr->getDataDim());
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API real64_T priorityqueuecodegen_simplemap_getSize(void* objPtr) {
    nav::SimpleMap* mapObjPtr = static_cast<nav::SimpleMap*>(objPtr);
    return static_cast<real64_T>(mapObjPtr->getSize());
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_simplemap_getData(void* objPtr,
                                                                               const real64_T nodeId,
                                                                               real64_T* data) {
    nav::SimpleMap* mapObjPtr = static_cast<nav::SimpleMap*>(objPtr);
    int nid = static_cast<int>(nodeId);
    std::vector<real64_T> tmp = mapObjPtr->getData(nid);
    // data marshalling
    size_t k = 0;
    for (real64_T& d : tmp) {
        data[k++] = d;
    }
}

EXTERN_C PRIORITYQUEUE_CODEGEN_API void priorityqueuecodegen_simplemap_insertData(void* objPtr,
                                                                                     const real64_T id,
                                                                                     const real64_T* newData ) {
    nav::SimpleMap* mapObjPtr = static_cast<nav::SimpleMap*>(objPtr);
    int nid = static_cast<int>(id);
    mapObjPtr->insertData(nid, newData);
}