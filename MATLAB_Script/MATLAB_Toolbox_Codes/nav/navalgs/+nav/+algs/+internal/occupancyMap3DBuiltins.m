classdef occupancyMap3DBuiltins < handle
%This class is for internal use only. It may be removed in the future.

%occupancyMap3DBuiltins Interface to builtins used for OccupancyMap3D
%
%   This class is a collection of functions used for interfacing with the
%   Octomap 3P library. Its main purpose is to dispatch function calls 
%   correctly when executed in MATLAB or code generation. During MATLAB 
%   execution, we call the existing MCOS C++ class. During code generation 
%   we use a codegen-compatible version.
%
%   See also nav.algs.internal.coder.occupancyMap3DBuildable

% Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties
        %MCOSObj - MCOS interface object to octomap
        %   This is only used during MATLAB execution.
        MCOSObj = []
        
        %Octomap - Opaque C++ object
        %   This is only used during code generation.
        Octomap
    end    

    methods
        function obj = occupancyMap3DBuiltins(resolution)
        %MonteCarloLocalizationBuiltins Constructor

            if coder.target('MATLAB')
                % Create MCOS class in MATLAB
                obj.MCOSObj = nav.algs.internal.OctomapWrapper(resolution);
            else
                % Generate code through external dependency
                obj.Octomap = nav.algs.internal.coder.occupancyMap3DBuildable(resolution);
            end
        end
        
        function delete(obj)
            if ~isempty(obj.MCOSObj)
                delete(obj.MCOSObj);
            end
        end
    end

    methods
        function setClampingThreshold(obj, clampingThresMin, clampingThresMax)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setClampingThreshold(clampingThresMin, clampingThresMax);
            else
                % Generate code through external dependency
                obj.Octomap.setClampingThreshold(clampingThresMin, clampingThresMax);
            end
        end
        
        function res = Resolution(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                res = obj.MCOSObj.Resolution;
            else
                % Generate code through external dependency
                res = obj.Octomap.getResolution();
            end
        end
        
        function occ = getOccupancy(obj, pos)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                occ = obj.MCOSObj.getOccupancy(pos);
            else
                % Generate code through external dependency
                occ = obj.Octomap.getOccupancy(pos);
            end
        end
        
        function str = serialization(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                str = obj.MCOSObj.serialization();
            else
                % Generate code through external dependency
                str = obj.Octomap.serialization();
                
            end
        end
        
        function deserialization(obj, pData)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.deserialization(pData);
            else
                % Generate code through external dependency
                obj.Octomap.deserialization(pData);
            end
        end
        
        function setNodeValue(obj, xyz, prob, lazyEval)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.setNodeValue(xyz, prob, lazyEval);
            else
                % Generate code through external dependency
                obj.Octomap.setNodeValue(xyz, prob, lazyEval);
            end
        end
        
        function updateNodeBoolean(obj, xyz, occupied, lazyEval)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.updateNodeBoolean(xyz, occupied, lazyEval);
            else
                % Generate code through external dependency
                obj.Octomap.updateNodeBoolean(xyz, occupied, lazyEval);
            end
        end
        
        function updateNodeDouble(obj, xyz, probUpdate, lazyEval)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.updateNodeDouble(xyz, probUpdate, lazyEval);
            else
                % Generate code through external dependency
                obj.Octomap.updateNodeDouble(xyz, probUpdate, lazyEval);
            end
        end
        
        function inflate(obj, inflationRadius, occupiedThreshold)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.inflate(inflationRadius, occupiedThreshold);
            else
                % Generate code through external dependency
                obj.Octomap.inflate(inflationRadius, occupiedThreshold);
            end
        end
        
        function insertPointCloud(obj, origin, points, maxRange, lazyEval, discretize)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.insertPointCloud(origin, points, maxRange, lazyEval, discretize);
            else
                % Generate code through external dependency
                obj.Octomap.insertPointCloud(origin, points, maxRange, lazyEval, discretize);
            end
        end
        
        function out =  getRayIntersection(obj, ptStart, ptDirections, occupiedThreshold, ignoreUnknownCells, maxRange)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                out = obj.MCOSObj.getRayIntersection(ptStart, ptDirections, occupiedThreshold, ignoreUnknownCells, maxRange);
            else
                % Generate code through external dependency
                out = obj.Octomap.getRayIntersection(ptStart, ptDirections, occupiedThreshold, ignoreUnknownCells, maxRange);
            end
        end
        
        function visData = extractVisualizationData(obj, maxDepth)
            % Call MCOS method in MATLAB
            % NOTE: occupancyMap3D.show does not support codegen, hence use
            % the MCOSObj always
            visData = obj.MCOSObj.extractVisualizationData(maxDepth);
        end
        
        function read(obj, filename)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.read(filename);
            else
                % Generate code through external dependency
                obj.Octomap.read(filename);
            end
        end
        
        function readBinary(obj, filename)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.readBinary(filename);
            else
                % Generate code through external dependency
                obj.Octomap.readBinary(filename);
            end
        end
        
        function write(obj, filename)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.write(filename);
            else
                % Generate code through external dependency
                obj.Octomap.write(filename);
            end
        end
        
        function writeBinary(obj, filename)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.writeBinary(filename);
            else
                % Generate code through external dependency
                obj.Octomap.writeBinary(filename);
            end
        end
        
        function sz = memoryUsage(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                sz = obj.MCOSObj.memoryUsage();
            else
                % Generate code through external dependency
                sz = obj.Octomap.memoryUsage();
            end
        end
        
        function dims = getMapDimensions(obj)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                dims = obj.MCOSObj.getMapDimensions();
            else
                % Generate code through external dependency
                dims = obj.Octomap.getMapDimensions();
            end
        end
        
        function deserializationBinaryROSMsgData(obj, res, data)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.deserializationBinaryROSMsgData(res, data);
            else
                % Generate code through external dependency
                obj.Octomap.deserializationBinaryROSMsgData(res, data);
            end
        end
        
        function deserializationFullROSMsgData(obj, res, data)
            if coder.target('MATLAB')
                % Call MCOS method in MATLAB
                obj.MCOSObj.deserializationFullROSMsgData(res, data);
            else
                % Generate code through external dependency
                obj.Octomap.deserializationFullROSMsgData(res, data);
            end
        end
        
    end
end
