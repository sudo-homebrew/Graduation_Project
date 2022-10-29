classdef PointCloudInterface < handle
%PointCloud2 Custom MATLAB implementation of sensor_msgs/PointCloud2 type
%   This class handles ROS point clouds and provides conversion
%   functions to MATLAB-compatible data structures.
%
%   PointCloud2 properties:
%      PreserveStructureOnRead - Determines if read methods preserve structure
%
%   PointCloud2 methods:
%      readXYZ            - Return 3D point coordinates (x-y-z)
%      readRGB            - Return per-point color information
%      readAllFieldNames  - Return all available field names
%      readField          - Read an arbitrary point field

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

    properties (Abstract)
        PreserveStructureOnRead
        Width
        Height
        PointStep
        Data
    end

    methods (Abstract)
        fieldNames = readAllFieldNames(obj)
    end

    methods (Abstract, Access = {...
        ?ros.msg.sensor_msgs.internal.PointCloudInterface, ...
        ?ros.msg.sensor_msgs.internal.PointCloud2Reader, ...
        ?matlab.unittest.TestCase})
        fieldIdx = getFieldNameIndex(obj, fieldName);
        offset = getFieldOffset(obj, fieldIdx);
        datatype = getFieldDatatype(obj, fieldIdx);
        count = getFieldCount(obj, fieldIdx);
    end

end
