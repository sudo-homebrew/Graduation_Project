// Copyright 2021 The MathWorks, Inc.
#ifndef PARAMETER_HELPER_H
#define PARAMETER_HELPER_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <regex>

#define UNUSED_PARAM(x)

class MATLABROSParameter {
    public:
    MATLABROSParameter(){
        nodePtr_ = std::make_shared<ros::NodeHandle>();
    };

    /**
    * Get available parameter for a given key.
    * @param key Index of the ROS parameter
    * @param pEntry Destination char pointer to store the parameter string
    */
    void getAvailableParameter(int key, char *pEntry) {
        if(paramNames_.size() > 0) {
            const char *pStr = paramNames_[key].c_str();
            size_t paramNameLen = getParameterLength(key);
            strncpy(pEntry, pStr, paramNameLen);
            pEntry[paramNameLen] = 0;
        } else {
            pEntry[0] = 0;
        }
    }

    /**
    * Sort the ROS Parameters present in paramNames_ container.
    */
    void sortParameters(){
        if(paramNames_.size() > 0) {
            std::sort(paramNames_.begin(), paramNames_.end());
        }
    }

    /**
    * Get parameter length for a given key.
    * @param key Index of the ROS parameter
    */
    int getParameterLength(int key) {
        return paramNames_[key].size();
    }

    /**
    * Resolve a graph resource name into a fully qualified graph resource name 
    * @param name Source graph resource name.
    * @param resolvedNamespace Destination char pointer to store fully qualified graph
    * resource name
    */
    void resolveName(std::string name, char* resolvedNamespace) {
        std::string resolvedName = ros::names::resolve(name);
        if(name.size() > 0) {
            const char *pStr = resolvedName.c_str();
            size_t resolvedNameLength = strlen(pStr);
            strncpy(resolvedNamespace, pStr, resolvedNameLength);
            resolvedNamespace[resolvedNameLength] = 0;
        } else {
            resolvedNamespace[0] = 0;
        }
    }

    /**
    * Get the total number of ROS Parameters available on parameter server 
    */
    uint32_t getNumOfROSParameters(){
        if(nodePtr_->getParamNames(paramNames_)){
            return static_cast<uint32_t>(paramNames_.size());
        } else {
            return 0;
        }
    }

    /**
    * Set a ROS Parameter for primitive datatypes using Node Handle.
    * @param paramName The name of the ROS parameter
    * @param value Parameter value that should be set.
    */
    template <typename CppParamType>
    void setParameter(std::string paramName, const CppParamType value){
        nodePtr_->setParam(paramName, value);
    }

   /**
    * Set a ROS Parameter for MATLAB homogeneous cell arrays of primitive types
    * except strings using Node Handle.
    * @param paramName The name of the ROS parameter
    * @param dataPtr Pointer to an array that should be set
    * @param arrLength The number of elements in the dataPtr array that 
    * should be written
    */
    template <typename CppParamType>
    void setArrayParameter(std::string paramName,
                           CppParamType* dataPtr,
                           uint32_t arrLength){
        if(dataPtr != nullptr) {
            std::vector<CppParamType> data(dataPtr, dataPtr + arrLength);
            nodePtr_->setParam(paramName, data);
        } else {
            std::vector<CppParamType> data{};
            nodePtr_->setParam(paramName, data);
        }
    }

    /**
    * Helper method to store strings which are part of cell array in a container.
    * @param paramName The name of the ROS parameter
    * @param data Parameter string value that should be set.
    */
    void setStringValues(std::string paramName,
                         std::string data){
        if(!data.empty()) {
            stringVecParamValues_.push_back(data);
        } else {
            stringVecParamValues_.push_back("");
        }
    }

    /**
    * Set a ROS Parameter for MATLAB homogeneous cell arrays of strings using Node Handle.
    * @param paramName The name of the ROS parameter
    */
    void setStringArrayParameter(std::string paramName){
        nodePtr_->setParam(paramName, stringVecParamValues_);
    }

    /**
    * Get the value for a named parameter from the parameter server.
    * @param paramName The name of the ROS parameter
    * @param paramValue Pointer to data variable. The retrieved parameter value will be
    * written to this location
    * @param status Pointer to initialized status flag. If there is no parameter available,
    * false will be written to this variable else true
    */
    template <typename CppParamType>
    bool getParameter(std::string paramName, CppParamType* paramValue){
        bool status = false;
        if(paramValue != nullptr) {
            status = nodePtr_->getParam(paramName, *paramValue);
        }
        return status;
    }

    /**
    * Get the string value for a named parameter from the parameter server.
    * @param paramName The name of the ROS parameter
    * @param paramValue Pointer to data variable. The retrieved parameter value will be
    * written to this location
    * @return status flag. If there is no parameter available, false will be 
    * written to this variable else true
    */
    bool getStringParameter(std::string paramName, char* paramValue){
        bool status = false;
        if(paramValue != nullptr) {
            std::string pStringValue(paramValue);
            status = nodePtr_->getParam(paramName, pStringValue);
            if(status) {
                const char *pStr = pStringValue.c_str();
                size_t paramValueLength = strlen(pStr);
                strncpy(paramValue, pStr, paramValueLength);
                paramValue[paramValueLength] = 0;
            } else {
                paramValue[0] = 0;
            }
        }
        return status;
    }

    /**
    * Check whether the ROS parameter exists.
    * @param key The name of the ROS parameter
    */
    bool hasParam(const char* key) {
        return nodePtr_->hasParam(key);
    }

    /**
    * Check whether the ROS parameter name is a valid graph resource name
    * @param key The name of the ROS parameter
    */
    bool isValidPattern(const char* key) {
        // Check the pattern, ROS graph name can start with '~' followed 
        // by '/' and then any of case-insensitive characters followed by
        // word character in repeated way. '^' indicates start of string, '$'
        // indicates end of string. 
        const char *pattern = "^[\\~\\/A-Za-z][\\w\\/_]*$";
        const std::regex re(pattern);
        return std::regex_match(key, re);
    }

    /**
    * Delete a ROS Parameter
    * @param key The name of the ROS parameter to be deleted
    */
    bool deleteParam(const char* key) {
        bool res = true;
        std::string paramName{key};
        if (paramName.compare("/") == 0) {
            // Not allowed to delete the root of the tree as it must always
            // have a value. The equivalent command is setting the root to an
            // empty dictionary
            std::map<std::string, bool> emptyMap;
            nodePtr_->setParam(paramName, emptyMap);
        } else {
            res = nodePtr_->deleteParam(paramName);
        }
        return res;
    }

    private:
    std::vector<std::string> paramNames_{};
    std::vector<std::string> stringVecParamValues_{};
    std::shared_ptr<ros::NodeHandle> nodePtr_;
};


#endif
