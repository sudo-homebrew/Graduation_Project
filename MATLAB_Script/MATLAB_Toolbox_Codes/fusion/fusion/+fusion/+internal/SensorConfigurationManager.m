classdef SensorConfigurationManager < matlab.System
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    % This class provides a way to manage the configuration of the sensors
    % provided as trackingSensorConfiguration objects.
    %
    % obj = fusion.internal.SensorConfigurationManager(Name,value)
    %
    % properties:
    % SensorConfigurations        - Configurations of the sensor
    % MaxNumSensors               - Maximum number of sensors
    % HasSensorConfigurationInput - A flag to control if sensor
    %                               configurations are updated with time.
    %
    %
    % step(obj) if HasSensorConfigurationInput is false.
    % 
    % step(obj, config) if HasSensorConfigurationInput is true.
    %
    % config = getConfig(obj, sensorData) gets the configuration of the
    % sensor reporting sensorData. sensorData is a struct with field
    % SensorIndex.
    %
    
    %#codegen
    
    properties (Dependent)
        SensorConfigurations
    end
    
    properties (Nontunable)
        MaxNumSensors = 20;
    end
    
    properties (Nontunable)
        HasSensorConfigurationsInput (1, 1) logical = false;
    end
    
    properties (Access = {?fusion.internal.SensorConfigurationManager,?matlab.unittest.TestCase})
        pSensorConfigurations
        pSensorIndices
    end
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            coder.internal.assert(coder.internal.is_defined(obj.pSensorConfigurations),'fusion:GridTracker:MustDefineBeforeStep','SensorConfigurations');
        end
        
        function stepImpl(obj, varargin)
            if obj.HasSensorConfigurationsInput
                configs = varargin{1};
                syncSensorConfigurations(obj,configs);
            end
        end
        
        function syncSensorConfigurations(obj, configs)
            for i = 1:numel(configs)
                if iscell(configs)
                    thisConfig = configs{i};
                else
                    thisConfig = configs(i);
                end
                id = thisConfig.SensorIndex;
                internalConfig = getConfigFromSensorIndex(obj,id);
                sync(internalConfig,thisConfig);
            end
        end
        
        function config = getConfigFromSensorIndex(obj, sensorIdx)
            internalIndex = obj.pSensorIndices == sensorIdx;
            coder.internal.assert(any(internalIndex),'fusion:GridTracker:unknownSensorIDConfig', sensorIdx);
            if coder.target('MATLAB')
                config = obj.pSensorConfigurations{internalIndex};
            else
                internalIndexLoc = find(internalIndex);
                config = obj.pSensorConfigurations{internalIndexLoc(1)};
            end
        end
    end
    
    methods
        function obj = SensorConfigurationManager(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods
        function config = getConfig(obj, sensorData)
            sensorIndex = sensorData.SensorIndex;
            config = getConfigFromSensorIndex(obj, sensorIndex);
            coder.internal.assert(config.IsValidTime,'fusion:GridTracker:notActiveSensor');
        end
    end
    
    methods
        function set.SensorConfigurations(obj,val)
            coder.internal.assert(isa(val,'cell') || isa(val,'fusion.internal.AbstractTrackingSensorConfiguration'),...
                'fusion:GridTracker:invalidConfigType',class(val));
            
            % Don't have a data type here yet. Use a safe data type
            n = numel(val);
            coder.internal.assert(n<=obj.MaxNumSensors,'fusion:GridTracker:SensorsGreaterThanMax');
            sensorIndex = zeros(n,1,'uint32');
            isSet = coder.internal.is_defined(obj.pSensorConfigurations);
            coder.internal.assert(~isSet,'fusion:GridTracker:SensorConfigSet');
            configs = cell(n,1);
            for i = 1:numel(val)
                if iscell(val)
                    coder.internal.assert(isa(val{i},'fusion.internal.AbstractTrackingSensorConfiguration'),...
                        'fusion:GridTracker:invalidConfigType',class(val{i}));
                    sensorIndex(i) = val{i}.SensorIndex;
                    configs{i} = clone(val{i});
                else
                    sensorIndex(i) = val(i).SensorIndex;
                    configs{i} = clone(val(i));
                end
            end
            obj.pSensorConfigurations = configs;
            coder.internal.assert(numel(unique(sensorIndex)) == numel(val),'fusion:GridTracker:ExpectedUniqueSensors');
            obj.pSensorIndices = sensorIndex;
        end
        
        function val = get.SensorConfigurations(obj)
            val = obj.pSensorConfigurations;
        end
    end
    
    methods (Access = protected)
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            if coder.internal.is_defined(obj.pSensorConfigurations)
                n = numel(obj.pSensorConfigurations);
                s.pSensorConfigurations = cell(n,1);
                for i = 1:n
                    s.pSensorConfigurations{i} = clone(obj.pSensorConfigurations{i});
                end
                s.pSensorIndices = obj.pSensorIndices;
            end
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            if isfield(s,'pSensorConfigurations')
                obj.pSensorConfigurations = s.pSensorConfigurations;
                obj.pSensorIndices = s.pSensorIndices;
            end
            loadObjectImpl@matlab.System(obj, s, wasLocked);
        end
        
        function newObj = cloneImpl(obj)
            newObj = cloneImpl@matlab.System(obj);
            if coder.internal.is_defined(obj.pSensorConfigurations)
                n = numel(obj.pSensorConfigurations);
                newObj.pSensorConfigurations = cell(n,1);
                for i = 1:n
                    newObj.pSensorConfigurations{i} = clone(obj.pSensorConfigurations{i});
                end
                newObj.pSensorIndices = obj.pSensorIndices;
            end
        end
    end
end
