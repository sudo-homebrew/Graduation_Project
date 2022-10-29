classdef GridSensorDataProjector < fusion.internal.SensorConfigurationManager
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    % This class provides an interface to project sensorData on a grid. 
    %
    % obj = fusion.internal.GridSensorDataProjector(Name,Value);
    % 
    % properties:
    % GridLength        - Dimensions of the grid in X direction (meters)
    % GridWidth         - Dimensions of the grid in Y direction (meters)
    % GridResolution    - Number of cells per meter in X and Y direction
    % GridOriginInLocal - Location of left corner of grid in local
    %                     coordinates.
    
    % measMap = obj(sensorData, time); if HasSensorConfigurationInput is
    % false.
    % measMap = obj(sensorData, sensorConfigs, time) if
    % HasSensorConfigurationInput is true.
    %
    % measMap is the measurement evidence map 
    
    % No input/property validation is done here, inputs must be validated
    % by the parent class.
    
    %#codegen
    
    % Grid properties
    properties (Nontunable)
        % GridLength Dimension of the grid in x-direction of the local
        % coordinate.
        % Specify the length as a positive scalar value describing
        % the length of the 2-D grid.
        %
        % Default: 100
        GridLength = 100;
        
        % GridWidth Dimension of the grid in y-direction of the local
        % coordinate.
        % Specify the width as a positive scalar value describing
        % the width of the 2-D grid.
        %
        % Default: 100
        GridWidth = 100;
        
        % GridResolution Resolution of the grid.
        % Specify the resolution of the grid as a positive scalar
        % describing number of cells per meter of the grid in both x and y
        % direction.
        %
        % Default: 1
        GridResolution = 1;
        
        %  GridOriginInLocal Location of the grid in local coordinates
        %  A vector defining the [X Y] location of the bottom-left
        %  corner of the grid, relative to the local frame.
        % Default: [-50 50]
        GridOriginInLocal = [-50 -50];
    end
    
    properties (Nontunable, Access = {?fusion.internal.GridSensorDataProjector, ?trackerGridRFS, ?matlab.unittest.TestCase})
        UseGPU (1, 1) logical = false
        HasOcclusion (1, 1) logical = true;
    end
    
    properties(Nontunable, Access = {?fusion.internal.GridSensorDataProjector,...
            ?trackerGridRFS,...
            ?matlab.unittest.TestCase})   
        ClassToUse
    end
    
    properties (Access = {?fusion.internal.GridSensorDataProjector,?matlab.unittest.TestCase})
        % A function handle which must be capable of performing the
        % following operation
        % xyLocal = fcn(sensorData, sensorConfig);
        %
        SensorDataProjectionFcn
    end
    
    properties (Access = {?fusion.internal.GridSensorDataProjector,...
            ?trackerGridRFS,...
            ?matlab.unittest.TestCase})
        pMeasurementEvidenceMap
    end
    
    methods
        function obj = GridSensorDataProjector(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function validatePropertiesImpl(obj)
            % Validate sensor configurations validity for sensor data
            % projection.
            for i = 1:numel(obj.pSensorConfigurations)
                thisConfig = obj.pSensorConfigurations{i};
                coder.internal.assert((size(thisConfig.SensorLimits,1)>= 2),'fusion:GridTracker:InvalidSensorLimits');
            end
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            setupImpl@fusion.internal.SensorConfigurationManager(obj, varargin{:});
            
            if ~coder.internal.is_defined(obj.ClassToUse)
                sensorData = varargin{1};
                obj.ClassToUse = class(sensorData(1).Measurement);
            end
            
            obj.pMeasurementEvidenceMap = fusion.internal.MeasurementEvidenceMap(...
                obj.GridLength,obj.GridWidth,obj.GridResolution,obj.GridOriginInLocal,...
                'UseGPU',obj.UseGPU,...
                'ClassToUse',obj.ClassToUse);
            
            obj.pMeasurementEvidenceMap.PositionParameters = obj.SensorConfigurations{1}.SensorTransformParameters(2:end);
            
            obj.SensorDataProjectionFcn = @fusion.internal.GridMeasurementBuiltins.SensorDataToLocalGrid;
        end
        
        function map = stepImpl(obj, sensorData, varargin)
            % step(obj, sensorData, time);
            % step(obj, sensorData, configs, time);
            
            stepImpl@fusion.internal.SensorConfigurationManager(obj, varargin{:});
            
            % Nullify the map to forget previous time-step
            nullify(obj.pMeasurementEvidenceMap);
            
            % Handle to the map
            map = obj.pMeasurementEvidenceMap;
            
            for i = 1:numel(sensorData)
                data = sensorData(i);
                
                % Project sensor data on x,y grid
                sensorConfig = getConfig(obj, data);
                
                xy = obj.SensorDataProjectionFcn(data, sensorConfig);
                
                % Compute sensor parameters
                [sensorPos,Pd,maxRange,fov,hasOcc] = parseConfig(obj,sensorConfig);
                
                % Insert data on the grid
                insertData(map,xy,sensorPos,Pd,maxRange,fov,hasOcc);
            end
            
            % Provide position parameters on the map. All configs carry
            % this information, just take from the first one
            config = obj.SensorConfigurations{1};
            map.PositionParameters = config.SensorTransformParameters(2:end);
        end
        
        function [sensorPos, Pd, maxRange, fov, hasOcc] = parseConfig(obj,config)
            % Parse the configuration of the sensor to provide inputs to
            % the projection

            % Transformation from sensor to local
            transParams = config.SensorTransformParameters(1);

            % Convert to origin and orientation
            [isRect,origin,~,orient,hasAz,hasEl,~,hasRange] = matlabshared.tracking.internal.fusion.parseMeasurementParameters(transParams,mfilename,obj.ClassToUse);
            ypr = eulerd(quaternion(orient,'rotmat','point'),'ZYX','point');

            % pose = [x y theta]
            sensorPos = [origin(1:2);ypr(1)];

            % Pd
            Pd = cast(config.DetectionProbability,obj.ClassToUse);

            if size(config.SensorLimits,1) == 2
                maxRange = config.SensorLimits(2,2);
            else
                if isfield(config.SensorTransformParameters,'Frame')
                    coder.internal.assert(~isRect,'fusion:GridTracker:InvalidFrameType');
                else
                    hasAz = true;
                    hasEl = false;
                    hasRange = true;
                end
                % Max range of the sensor
                coder.internal.assert(hasAz,'fusion:GridTracker:MissingAzimuthLimits');
                coder.internal.assert(hasRange,'fusion:GridTracker:MissingRangeLimits');
                rngIdx = 2 + hasEl;
                maxRange = config.SensorLimits(rngIdx,2);
            end

            % Field of view
            fov = cast(config.SensorLimits(1,:),obj.ClassToUse);

            % Use occlusion for projection of sensor data
            hasOcc = obj.HasOcclusion;
        end
    end
    
    methods (Access = protected)
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@fusion.internal.SensorConfigurationManager(obj);
            s.UseGPU = obj.UseGPU;
            s.ClassToUse = obj.ClassToUse;
            s.HasOcclusion = obj.HasOcclusion;
            if isLocked(obj)
                s.pMeasurementEvidenceMap = clone(obj.pMeasurementEvidenceMap);
                s.SensorDataProjectionFcn = obj.SensorDataProjectionFcn;
            end
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            obj.UseGPU = s.UseGPU;
            obj.HasOcclusion = s.HasOcclusion;
            obj.ClassToUse = s.ClassToUse;
            if wasLocked
                obj.pMeasurementEvidenceMap = s.pMeasurementEvidenceMap;
                obj.SensorDataProjectionFcn = s.SensorDataProjectionFcn;
            end
            loadObjectImpl@fusion.internal.SensorConfigurationManager(obj, s, wasLocked) 
        end
        
        function newObj = cloneImpl(obj)
            newObj = cloneImpl@fusion.internal.SensorConfigurationManager(obj);
            if coder.internal.is_defined(obj.pMeasurementEvidenceMap)
                newObj.pMeasurementEvidenceMap = clone(obj.pMeasurementEvidenceMap);
                newObj.SensorDataProjectionFcn = obj.SensorDataProjectionFcn;
            end
        end
        
        function resetImpl(obj)
            nullify(obj.pMeasurementEvidenceMap);
            resetImpl@matlab.System(obj);
        end
    end
    
    methods (Access = protected)
        function numIn = getNumInputsImpl(obj)
            numIn = 2 + obj.HasSensorConfigurationsInput;
        end
    end
    
end
