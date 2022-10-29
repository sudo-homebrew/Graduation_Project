classdef (Abstract,Hidden) ScenarioReaderBase < matlabshared.tracking.internal.SimulinkBusUtilities

    % ScenarioReaderBase Base class for scenario reader block.
    %
    % This is an internal class and may be removed or modified in a future
    % release.

    % Copyright 2021 The MathWorks, Inc.

    properties(Nontunable,Abstract)
        % ScenarioSource Source of scenario
        ScenarioSource
    end

    properties(Nontunable)
        % ScenarioFileName Scenario Designer session file name
        ScenarioFileName = '';

        % WorkspaceVariableName Workspace variable name
        WorkspaceVariableName = '';

        % Source of platform pose
        PlatformPoseSource = 'Scenario'

        % SampleTime Sample time (s)
        SampleTime(1,1) {mustBeGreaterThan(SampleTime,0)} = 0.1;

        % CoordinateSystem Coordinate system to report platform poses
        CoordinateSystem = 'Cartesian';

        % RNGSetting Random number generation setting
        RNGSetting = 'Repeatable'

        % InitialSeed Initial seed
        InitialSeed(1,1) {mustBeInteger,mustBeNonnegative} = 0
    end

    properties (Nontunable)

        % IncludePlatformProfiles Include profiles information with
        % platforms
        IncludePlatformProfiles (1, 1) logical = false;

        % EnableCoverageConfigOutput Enables coverage configuration output
        EnableCoverageConfigOutput (1, 1) logical = false;

        % EnableDetectionsOutput Enables detections output
        EnableDetectionsOutput (1, 1) logical = false;

        % EnableSensorConfigOutput Enables sensor configuration in output
        EnableSensorConfigOutput (1, 1) logical = false;

        % EnableEmissionsOutput Enable emissions output
        EnableEmissionsOutput (1, 1) logical = false;

        % EnableEmitterConfigOutput Enables emitters output
        EnableEmitterConfigOutput (1, 1) logical = false;

        % EnableOcclusion Enable occlusion in signal transmission while
        % propagating Emissions.
        EnableOcclusion (1, 1) logical = true;
    end


    properties(Nontunable)
        % BusName2Source Source of coverage configuration bus name
        BusName2Source = 'Auto'
        % BusName2 Specify coverage configuration bus name
        BusName2 = char.empty(1,0)

        % BusName3Source Source of detection bus name
        BusName3Source = 'Auto'
        % BusName3 Specify detection bus name
        BusName3 = char.empty(1,0)

        % BusName4Source Source of sensor configuration bus name
        BusName4Source = 'Auto'
        % BusName4 Specify sensor configuration bus name
        BusName4 = char.empty(1,0)

        % BusName5Source Source of emission bus name
        BusName5Source = 'Auto'
        % BusName5 Specify emission bus name
        BusName5 = char.empty(1,0)

        % BusName6Source Source of emitter configuration bus name
        BusName6Source = 'Auto'

        % BusName6 Specify emitter configuration bus name
        BusName6 = char.empty(1,0)

        % MaxPlatformsSource Source of maximum number of platforms
        MaxPlatformsSource = 'Auto';

        % MaxNumPlatforms Maximum number of platforms
        MaxNumPlatforms(1,1) {mustBeInteger,mustBeNonnegative} = 50;

        % MaxSensorsSource Source of maximum number of radar sensors
        MaxSensorsSource = 'Auto';

        % MaxNumSensors Maximum number of radar sensors
        MaxNumSensors(1,1) {mustBeInteger,mustBeNonnegative} = 10;

        % MaxEmittersSource Source of maximum number of emitters
        MaxEmittersSource = 'Auto';

        % MaxNumEmitters Maximum number of emitters
        MaxNumEmitters(1,1) {mustBeInteger,mustBeNonnegative} = 10;

        % MaxDetectionsSource Source of maximum number of detections
        MaxDetectionsSource = 'Auto';

        % MaxNumDetections Maximum number of detections
        MaxNumDetections(1,1) {mustBeInteger,mustBeNonnegative} = 50;

        % MaxEmissionsSource Source of maximum number of emissions
        MaxEmissionsSource = 'Auto';

        % MaxNumEmissions Maximum number of emissions
        MaxNumEmissions(1,1) {mustBeInteger,mustBeNonnegative} = 50;
    end

    properties (Abstract,Hidden,Dependent, Access = protected)
        % A Logical array to select from different output options.
        % This property is hidden and used internally.
        pOutputSelector
    end


    properties(Hidden,Constant)
        PlatformPoseSourceSet = matlab.system.StringSet({'Scenario','Input port'});
        RNGSettingSet = matlab.system.StringSet({'Repeatable','Not repeatable','Specify seed'})
        CoordinateSystemSet = matlab.system.StringSet({'Cartesian','Geodetic'});

        BusName2SourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName3SourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName4SourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName5SourceSet = matlab.system.StringSet({'Auto','Property'});
        BusName6SourceSet = matlab.system.StringSet({'Auto','Property'});

        MaxPlatformsSourceSet = matlab.system.StringSet({'Auto','Property'});
        MaxSensorsSourceSet = matlab.system.StringSet({'Auto','Property'});
        MaxEmittersSourceSet = matlab.system.StringSet({'Auto','Property'});
        MaxDetectionsSourceSet = matlab.system.StringSet({'Auto','Property'});
        MaxEmissionsSourceSet = matlab.system.StringSet({'Auto','Property'});
    end

    properties(Nontunable, Abstract, Access = protected)
        % pScenarioRecording Scenario recording object. This property has
        % to be implemented by the derived class.
        pScenarioRecording
    end

    properties (Constant, Abstract, Access = protected)
        % pNumRecordingOutputParams Number of output arguments in a
        % scenario recording. This property has to be implemented by derived
        % class.
        pNumRecordingOutputParams
    end

    properties(Access = protected)
        pProfiles
        pSampleStructs
        pMaxNumDetections
        pMaxNumPlatforms
        pMaxNumEmissions
        pMaxNumEmitters
        pNumMaxCovCon
        pMaxNumSensors
        pCurrentStep;
    end

    methods
        function obj = ScenarioReaderBase(varargin)
            setProperties(obj,nargin,varargin{:})
        end

        % Bus name validation for additional buses (first is handled by
        % SimulinkBusUtilities)
        function val = get.BusName2(obj)
            val = obj.BusName2;
            val = getBusName(obj,val,2);
        end
        function set.BusName2(obj,val)
            validateBusName(obj,val,'BusName2')
            obj.BusName2 = val;
        end
        function val = get.BusName3(obj)
            val = obj.BusName3;
            val = getBusName(obj,val,3);
        end
        function set.BusName3(obj,val)
            validateBusName(obj,val,'BusName3')
            obj.BusName3 = val;
        end
        function val = get.BusName4(obj)
            val = obj.BusName4;
            val = getBusName(obj,val,4);
        end
        function set.BusName4(obj,val)
            validateBusName(obj,val,'BusName4')
            obj.BusName4 = val;
        end
        function val = get.BusName5(obj)
            val = obj.BusName5;
            val = getBusName(obj,val,5);
        end
        function set.BusName5(obj,val)
            validateBusName(obj,val,'BusName5')
            obj.BusName5 = val;
        end
        function val = get.BusName6(obj)
            val = obj.BusName6;
            val = getBusName(obj,val,6);
        end
        function set.BusName6(obj,val)
            validateBusName(obj,val,'BusName6')
            obj.BusName6 = val;
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            %Reset the state of scenario recording. 
            reset(obj.pScenarioRecording);
            %Set simulation start time as current time
            obj.pScenarioRecording.CurrentTime=str2double(get_param(bdroot,'StartTime'));
        end

        function varargout = stepImpl(obj,varargin)
            %Check if scenario recording is finished. Raise an error if
            %scenario recording is finished and simulation is still running.
            assert(~isDone(obj.pScenarioRecording),message('fusion:simulink:ScenarioReaderBase:ScenarioEndMessage'));
       
            recordedData = cell (1,obj.pNumRecordingOutputParams);
            %Read one step data from recording.
            [recordedData{:}] = read(obj.pScenarioRecording);

            if strcmpi(obj.PlatformPoseSource,'Input port')
                %Platforms are managed externally in a closed loop
                %simulation.
                managedPlatformPoses = varargin{1}.Platforms(1:varargin{1}.NumPlatforms);
                recordedData{2} = updateManagedPlatforms(obj,recordedData{2},managedPlatformPoses);
            end
            varargout = formatBusOutput(obj,recordedData{:});
        end

        function busOutput = formatBusOutput(obj,varargin)
            % Copy data collected form scenario into a Simulink
            % compatible bus structure.
            
            [SimulationTime,Poses,CoverageConfig,Detections,...
                SensorConfigurations,~,Emissions,...
                EmitterConfigurations,~] =varargin{:};
            
            %Pose
            busIdx=1;
            busOutput{busIdx} = sendToBus(obj,Poses,busIdx);
            busIdx = busIdx+1;
            %CoverageConfig
            busOutput{busIdx} = sendToBus(obj,CoverageConfig,busIdx);
            busIdx = busIdx+1;
            %Detections
            busOutput{busIdx} = sendToBus(obj,Detections,busIdx);
            busIdx = busIdx+1;
            %SensorConfigurations
            busOutput{busIdx} = sendToBus(obj,SensorConfigurations,busIdx);
            busIdx = busIdx+1;
            %Emissions
            busOutput{busIdx} = sendToBus(obj,Emissions,busIdx);
            busIdx = busIdx+1;
            %EmitterConfigurations
            busOutput{busIdx} =  sendToBus(obj,EmitterConfigurations,busIdx);
            
            %Insert Time at second position in output, this way first 2
            %outputs will always be available, and rest of the outputs can be
            %enabled and disabled as required.
            busOutput = {busOutput{1},SimulationTime,busOutput{2:end}};

        end

        function [out, argsToBus] = defaultOutput(obj,busIdx,varargin)
            %Default output which will be placed on a bus.
            out = struct.empty();
            switch busIdx
                case 1 %Pose
                    out = obj.pSampleStructs.Poses;
                case 2 %CoverageConfig
                    if obj.EnableCoverageConfigOutput
                        out = obj.pSampleStructs.CoverageConfig;
                    end
                case 3 %Detections
                    if obj.EnableDetectionsOutput
                        %Default output for detections is a cell array of detcetions.          
                        out = {obj.pSampleStructs.Detections};
                    end
                case 4 %SensorConfigurations
                    if obj.EnableSensorConfigOutput
                        out = obj.pSampleStructs.SensorConfigurations;
                    end
                case 5 %Emissions
                    if obj.EnableEmissionsOutput
                       %Default output for emissions is a cell array of emissions.          
                        out = {obj.pSampleStructs.Emissions};
                    end
                case 6 %EmitterConfigurations
                    if obj.EnableEmitterConfigOutput
                        out = obj.pSampleStructs.EmitterConfigurations;
                    end
            end
            argsToBus = {};
        end

        function y = sendToBus(obj,outObj,varargin)
            %Create a bus structure with data extracted from scenario. 
            busIdx = varargin{1};
            y = struct.empty();
            switch busIdx
                case 1 %Pose
                    [pos,num] = convert2BusStruct(obj,outObj,busIdx);
                    y = struct('Platforms',pos,'NumPlatforms',num);

                case 2 %CoverageConfig
                    if obj.EnableCoverageConfigOutput
                        [conf,num] =convert2BusStruct(obj,outObj,busIdx);
                        y = struct('Configurations',conf,'NumConfigurations',num);

                    end
                case 3 %Detections
                    if obj.EnableDetectionsOutput
                        [det,num] =convert2BusStruct(obj,outObj,busIdx);
                        y = struct('Detections',det,'NumDetections',num);
                    end
                case 4 %SensorConfigurations
                    if obj.EnableSensorConfigOutput
                        [conf,num] =convert2BusStruct(obj,outObj,busIdx);
                        y = struct('Configurations',conf,'NumConfigurations',num);
                    end
                case 5 %Emissions
                    if obj.EnableEmissionsOutput
                        [emissions,num] =convert2BusStruct(obj,outObj,busIdx);
                        y = struct('Emissions',emissions,'NumEmissions',num);
                    end
                case 6 %EmitterConfigurations
                    if obj.EnableEmitterConfigOutput
                        [conf,num] =convert2BusStruct(obj,outObj,busIdx);
                        y = struct('Configurations',conf,'NumConfigurations',num);
                    end
            end
        end

        function scenarioRecording = recordScenario(~,scenario,IEmitters,ISensors,HOcclusion,coord,rngSeed)
            %Record the scenario. 
            scenarioRecording = record(scenario,'rotmat','RecordingFormat', ...
                'Recording','IncludeEmitters',IEmitters,'IncludeSensors',ISensors,...
                'HasOcclusion',HOcclusion,'CoordinateSystem',coord,"InitialSeed",rngSeed);
        end

        function poses = updateManagedPlatforms(~,poses,managedPlatformPoses)
            %Update poses of the platforms as received in input.                        
            for i = 1:numel(managedPlatformPoses)
                %Find index of the platform to be updated.
                idx = [poses.PlatformID] == managedPlatformPoses(i).PlatformID;
                %A platform with PlatformID same as PlatformID received in input must
                %be present in the scenario, if not raise an error. It is
                %always ensured that scenario will have unique PlatformIDs. 
                if any(idx)
                    fields = fieldnames(managedPlatformPoses(i));
                    for j = 1:numel(fields)
                        poses(idx).(fields{j}) = managedPlatformPoses(i).(fields{j});
                    end
                else
                   % No matching PlatformID found, raise an error.
                   error(message('fusion:simulink:ScenarioReaderBase:InvalidPlatfromIDInput',managedPlatformPoses(i).PlatformID));
                end
            end
        end

        function Poses = addProfilesWithPlatformPoses(obj,Poses)
            % Add profile information along with platform poses.
            
            %Get profile information 
            platformProfiles = obj.pProfiles;
            
            for i = 1 : numel(Poses)
                %Find index of the platform to be updated. It is ensured
                %that a platform with unique PlatformID is always
                %available.
                idx = [platformProfiles.PlatformID] == Poses(i).PlatformID;
                if any(idx)
                    %Add Dimensions field form profile with poses. 
                    Poses(i).Dimensions = platformProfiles(idx).Dimensions;                                    
                end
            end
        end

        function storeSampleStructs(obj,recordedData)
            %Structures for output buses are captured from the recorded
            %data. When an optional output is selected form the block and
            %its corresponding data is not generated by the scenario,
            %internally stored structures are used for bus creation and a
            %warning is generated to notify the user regarding
            %unavailability of selected data.

            %Time
            obj.pSampleStructs.SimulationTime = recordedData(1).SimulationTime;
            %Poses
            storeSamplePoseStruct(obj,recordedData);
            %Detection
            storeSampleDetectionStruct(obj,recordedData);
            % Sensor config
            storeSampleSensorConfigStruct(obj,recordedData);
            %Emission
            storeSampleEmissionStruct(obj,recordedData);
            %Emitter config
            storeSampleEmitterConfigStruct(obj,recordedData);
            %Coverage config
            storeSampleCovConfStruct(obj,recordedData);
            
            %Maximum number of coverage configurations on a bus will be equal
            %to sum of maximum number of sensors and emitters.
            obj.pNumMaxCovCon = obj.pMaxNumSensors + obj.pMaxNumEmitters;

        end

        function storeSampleEmitterConfigStruct(obj,recordedData)
            %Store sample EmitterConfigurations structure.
            
            %Check if recorded data has EmitterConfigurations field and
            %EmitterConfigurations array has data in it.
            if ~isfield(recordedData,'EmitterConfigurations') || ...
                    ( isfield(recordedData,'EmitterConfigurations') && ...
                    all(arrayfun(@isempty,[recordedData.EmitterConfigurations]),'all'))
                %Scenario contains EmitterConfigurations.
                if obj.EnableEmitterConfigOutput
                    %If user has selected EmitterConfigurations output,
                    %generate a warning that data does not exist.
                    warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','Emitter configurations','Emitters'));
                end
                %When scenario does not generate EmitterConfigurations and
                %user has selected the output,set sample structure
                %manually so that a bus definition can be generated. 
                %A valid structure is required to generate bus definition.                 
                obj.pSampleStructs.EmitterConfigurations = fusion.simulink.internal.getSampleStruct('EmitterConfig');
                obj.pMaxNumEmitters = obj.MaxNumEmitters;
            else
                %Scenario contains EmitterConfigurations.
                %Store sample structure from recorded data.                
                obj.pSampleStructs.EmitterConfigurations = recordedData(1).EmitterConfigurations(1);
                
                %Find maximum number of MeasurementParameters in
                %EmitterConfigurations.
                maxNumMeasParams = max(arrayfun(@(x)numel(x.MeasurementParameters),[recordedData.EmitterConfigurations]),[],'all');
                %On a bus MeasurementParameters structure will be of fixed
                %size, initializing the structure with maxNumMeasParams
                %elements.
                measParams=repmat(obj.pSampleStructs.EmitterConfigurations.MeasurementParameters(1),maxNumMeasParams,1);
                obj.pSampleStructs.EmitterConfigurations.MeasurementParameters = copyMeasurementParams(obj,measParams);
                %Zero out values in the structure.
                obj.pSampleStructs.EmitterConfigurations = matlabshared.tracking.internal.nullifyStruct(obj.pSampleStructs.EmitterConfigurations);
                
                if strcmpi(obj.MaxEmittersSource,'Auto')
                    %When source is Auto, get maximum number of emitters
                    %from scenario. This is required to generate a fixed size bus.
                    obj.pMaxNumEmitters = max(cellfun(@numel,{recordedData.EmitterConfigurations}));
                else
                    %When it is defined by user, use that
                    obj.pMaxNumEmitters = obj.MaxNumEmitters;
                end
            end
        end

        function storeSampleEmissionStruct(obj,recordedData)
            %Store sample Emissions structure.
            
            %Check if recorded data has Emissions field and 
            %Emissions array has data in it.
            if ~isfield(recordedData,'Emissions') || ...
                    ( isfield(recordedData,'Emissions') && ...
                    all(cellfun(@isempty,{recordedData.Emissions})))
                %Scenario does not contain Emissions.
                if obj.EnableEmissionsOutput
                    %If user has selected Emissions output,
                    %generate a warning that data does not exist.
                    warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','Emissions','Emitters'));
                end
                %Store sample structure manually.
                obj.pSampleStructs.Emissions = fusion.simulink.internal.getSampleStruct('Emission');
                obj.pMaxNumEmissions = obj.MaxNumEmissions;
            else
                %Scenario contains Emissions.
                %Store sample emission structure from scenario.
                obj.pSampleStructs.Emissions = matlabshared.tracking.internal.nullifyStruct(objEmission2Struct(obj,recordedData(1).Emissions(1)));
               
                if strcmpi(obj.MaxEmissionsSource,'Auto')
                    %When source is Auto, get maximum number of emissions
                    %from scenario.
                    obj.pMaxNumEmissions = max(cellfun(@numel,{recordedData.Emissions}));
                else
                    %When it is defined by user, use that
                    obj.pMaxNumEmissions = obj.MaxNumEmissions;
                end
            end
        end

        function storeSampleSensorConfigStruct(obj,recordedData)
            %Store sample SensorConfigurations structure.
            
            %Check if recorded data has SensorConfigurations field and 
            %SensorConfigurations array has data in it.            
            if ~isfield(recordedData,'SensorConfigurations') || ...
                    (isfield(recordedData,'SensorConfigurations') && ...
                    all(arrayfun(@isempty,[recordedData.SensorConfigurations]),'all'))
                %Scenario does not contain SensorConfigurations.
                if obj.EnableSensorConfigOutput
                    %If user has selected SensorConfigurations output
                    %generate a warning.
                    warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','Sensor configurations','Sensor'));
                end
                %Store sample structure manually
                obj.pSampleStructs.SensorConfigurations = fusion.simulink.internal.getSampleStruct('SensorConfig');
                obj.pMaxNumSensors = obj.MaxNumSensors;
            else
                %Scenario contains SensorConfigurations.
                %Store sample structure from scenario.
                obj.pSampleStructs.SensorConfigurations = recordedData(1).SensorConfigurations(1);
                %Find maximum number of MeasurementParameters in SensorConfigurations
                maxNumMeasParams = max(arrayfun(@(x)numel(x.MeasurementParameters),[recordedData.SensorConfigurations]),[],'all');
                %On a bus MeasurementParameters structure will be of fixed
                %size, initializing the structure with maxNumMeasParams
                %elements.
                measParams = repmat(obj.pSampleStructs.SensorConfigurations.MeasurementParameters(1),maxNumMeasParams,1);
                obj.pSampleStructs.SensorConfigurations.MeasurementParameters = copyMeasurementParams(obj,measParams);
                %Zero out values in the structure.
                obj.pSampleStructs.SensorConfigurations = matlabshared.tracking.internal.nullifyStruct(obj.pSampleStructs.SensorConfigurations);

                if strcmpi(obj.MaxSensorsSource,'Auto')
                    %When source is Auto, get maximum number of sensors
                    %from scenario.
                    obj.pMaxNumSensors = max(cellfun(@numel,{recordedData.SensorConfigurations}));
                else
                    %When it is defined by user, use that
                    obj.pMaxNumSensors = obj.MaxNumSensors;
                end
            end
        end

        function storeSampleDetectionStruct(obj,recordedData)
            %Store sample Detections structure.
            
            %Check if recorded data has Detections field and 
            %Detections cell array has data in it.  
            if ~isfield(recordedData,'Detections') || ...
                    (isfield(recordedData,'Detections') && ...
                    all(cellfun(@isempty,{recordedData.Detections})))
                %Scenario does not generate detections.
                if obj.EnableDetectionsOutput
                    %If user has selected Detections output,
                    %generate a warning that data does not exist.
                    warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','Detections','Detecting sensor'));
                end
                %Set sample structure manually.
                obj.pSampleStructs.Detections = fusion.simulink.internal.getSampleStruct('Detections');
                obj.pMaxNumDetections = obj.MaxNumDetections;
            else
                %Scenario generates detections.
                %Set sample structure from scenario.
                obj.pSampleStructs.Detections = getDetectionTemplateStruct(obj,recordedData);
                                
                if strcmpi(obj.MaxDetectionsSource,'Auto')
                    %When source is Auto, get maximum number of Detections
                    %from scenario data.
                    obj.pMaxNumDetections = max(cellfun(@numel,{recordedData.Detections}));
                else
                    %When it is defined by user, use that
                    obj.pMaxNumDetections = obj.MaxNumDetections;
                end
            end
        end

        function sampleDetection = getDetectionTemplateStruct(~,recordedData)

            % Generate a detection template from all the available
            % detections.
            %
            % The template detection 'sampleDetection' will contain fields from all the detections
            % collected form the scenario.
            %
            % The length of sampleDetection's Measurement field is the maximum length,
            % L, taken from the N objectDetection collected from the
            % scenario. The MeasurementNoise is an L-by-L matrix. These
            % fields are zero padded when needed.
            %
            % The scalar structs in the ObjectAttributes and
            % MeasurementParameters of detections contains all of the
            % fields.
            
            %Initialize parameters
            maxMeasurementSize = 0;
            maxMeasurementNoiseSize = 0;
            maxNumMeasParams = 0;
            objAttrib = struct;           

            %Since we have detections from all the sensors in a single cell
            %array, we need to go through all the
            %available detections to find out all the fields and their
            %sizes to generate output detection template.
            
            %First get the default detection struct and modify it according
            %to the detections from scenario.
            sampleDetection = fusion.simulink.internal.getSampleStruct('Detections');
            
            dets = {recordedData.Detections};
            
            for i = 1: numel(dets)
                %Iterate over all the detections
                if ~isempty(dets{i})
                    for j = 1:numel(dets{i})
                        %Find maximum length of Measurement.
                        maxMeasurementSize = max(maxMeasurementSize,length(dets{i}{j}.Measurement));
                        %Find maximum length of MeasurementNoise
                        maxMeasurementNoiseSize = max(maxMeasurementNoiseSize,length(dets{i}{j}.MeasurementNoise));
                        if isprop(dets{i}{j}, 'MeasurementParameters')
                            %Find maximum size of MeasurementParameters.
                            maxNumMeasParams = max(maxNumMeasParams,length(dets{i}{j}.MeasurementParameters));
                        end

                        if isprop(dets{i}{j}, 'ObjectAttributes')
                            %Store ObjectAttributes in template structure
                            for k = 1 : numel(dets{i}{j}.ObjectAttributes) %ObjectAttributes is always a cell
                                fields = fieldnames(dets{i}{j}.ObjectAttributes{k});
                                for thisField = 1:numel(fields)
                                    %if a field from ObjectAttributes is not
                                    %present in objAttrib, add it.
                                    if ~isfield(objAttrib(k),fields{thisField})
                                        objAttrib(k).(fields{thisField}) = dets{i}{j}.ObjectAttributes{k}.(fields{thisField});
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            %update sampleDetection.            
            % Set Measurement to be of maximum size.  
            sampleDetection.Measurement = zeros(maxMeasurementSize,1);
            %Set Measurement noise to be of maximum size.
            sampleDetection.MeasurementNoise = zeros(maxMeasurementNoiseSize,maxMeasurementNoiseSize);
            %Set measurement parameters.
            sampleDetection.MeasurementParameters = repmat(fusion.simulink.internal.getSampleStruct('MeasurementParameters'),maxNumMeasParams,1);
            %Set ObjectAttributes
            sampleDetection.ObjectAttributes = objAttrib;
            %Zero out values in the structure.
            sampleDetection = matlabshared.tracking.internal.nullifyStruct(sampleDetection);
        end

        function storeSampleCovConfStruct(obj,recordedData)
            %Store sample CoverageConfig structure.
            
            %Check if recorded data has CoverageConfig field and 
            %CoverageConfig array has data in it. 
            if ~isfield(recordedData,'CoverageConfig') || ...
                    (isfield(recordedData,'CoverageConfig') && ...
                    all(arrayfun(@isempty,[recordedData.CoverageConfig]),'all'))
                %Scenario does not have CoverageConfig data.
                if obj.EnableCoverageConfigOutput
                    %If user has selected CoverageConfig output,
                    %generate a warning that data does not exist.
                    warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','Coverage configurations','Sensors or Emitters'));
                end
                %Set sample structure manually.
                obj.pSampleStructs.CoverageConfig = fusion.simulink.internal.getSampleStruct('CoverageConfig');
            else
                %Scenario contains CoverageConfig data.
                %Set sample structure from scenario.
                obj.pSampleStructs.CoverageConfig = matlabshared.tracking.internal.nullifyStruct(recordedData(1).CoverageConfig(1));
                %Orientation is always a 3x3 rotation matrix in Simulink.
                obj.pSampleStructs.CoverageConfig.Orientation = eye(3);
                % ScanLimits is a 2-by-2 matrix [minAz, maxAz; minEl,maxEl]
                obj.pSampleStructs.CoverageConfig.ScanLimits = NaN(2,2);
                % LookAngle is a 2x1 vector [azimuth; elevation]
                obj.pSampleStructs.CoverageConfig.LookAngle = NaN(2,1);                               
            end
        end

        function storeSamplePoseStruct(obj,recordedData)
            %Store sample platform pose structure.
            
            %Check if recorded data has Poses field and 
            %Poses array has data in it
            if all(cellfun(@isempty,{recordedData.Poses}))
                %Scenario does not contain platform poses. 
                %Generate a warning that data does not exist.
                warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','Platforms Pose','Platforms'));
                %Store sample pose structure manually.
                obj.pSampleStructs.Poses = fusion.simulink.internal.getSampleStruct('Poses');
                obj.pMaxNumPlatforms = obj.MaxNumPlatforms;
            else
               %Scenario contains platform poses.
               %Set sample structure from recorded data.
                obj.pSampleStructs.Poses = recordedData(1).Poses(1);
                if obj.IncludePlatformProfiles
                    % Add profiles information.
                    obj.pSampleStructs.Poses.Dimensions = obj.pProfiles(1).Dimensions;
                end
                obj.pSampleStructs.Poses = matlabshared.tracking.internal.nullifyStruct(obj.pSampleStructs.Poses);
                if strcmpi(obj.MaxPlatformsSource,'Auto')
                    %When source is Auto, get maximum number of platforms
                    %from scenario data.
                    obj.pMaxNumPlatforms = max(cellfun(@numel,{recordedData.Poses}));
                else
                    %When it is defined by user, use that
                    obj.pMaxNumPlatforms = obj.MaxNumPlatforms;
                end
            end
        end

        function stEmission= objEmission2Struct(obj,objEmission,stEmission)
            % Convert radarEmission or sonarEmission object into a struct.
            %
            % When stEmission is provided, copy data from objEmission to
            % stEmission and when it is not provided, create an empty
            % structure and copy data into that.
                        
            if nargin < 3
                stEmission = struct.empty();
            end
            
            for i = 1:numel(objEmission)
                %Iterate over all the emissions in objEmission and copy.
                emission = getDataFromCellorStructArray(obj,objEmission,i);
                fields = fieldnames(emission);
                for j = 1:numel(fields)
                    if strcmpi(fields{j},'Orientation')
                        if isa(emission.Orientation,'quaternion')
                            stEmission(i).Orientation = rotmat(emission.Orientation,'Frame');
                        else
                            stEmission(i).Orientation = emission.Orientation;
                        end
                    else
                        stEmission(i).(fields{j}) = emission.(fields{j});
                    end
                end
                %In Simulink workflow, for Radar and Sonar emissions a
                %common structure is used.
                if isa(emission,'radarEmission')
                    stEmission(i).SourceLevel = NaN(1);
                    stEmission(i).TargetStrength = NaN(1);
                elseif isa(emission,'sonarEmission ')
                    stEmission(i).EIRP = NaN(1);
                    stEmission(i).RCS = NaN(1);
                end
            end
        end

        function detStruct = copyDetectionsInStruct(obj,objDet,detStruct)
            % Copy detections from objDet to detStruct.
            for i = 1:numel(objDet)
                %Get detections from a cell or structure array.
                det = objDet{i};
                fields = fieldnames(det);
                for j = 1:numel(fields)
                    if strcmpi(fields{j},'MeasurementParameters')
                        %Copy MeasurementParameters;
                        detStruct(i).MeasurementParameters = copyMeasurementParams(obj,det.MeasurementParameters);
                    elseif strcmpi(fields{j},'ObjectAttributes')
                        %Copy ObjectAttributes
                        detStruct(i).ObjectAttributes = copyObjectAttributesInStruct(obj,det.ObjectAttributes);
                    elseif strcmpi(fields{j},'Measurement') || strcmpi(fields{j},'MeasurementNoise')
                        %copy Measurement and MeasurementNoise at sepcific
                        %indexes in detStruct.
                        %detStruct will have the maximum size for
                        %Measurement and MeasurementNoise with zero 
                        ind = getLogicalIndices(det.(fields{j}), obj.pSampleStructs.Detections.(fields{j}));
                        detStruct(i).(fields{j})(ind) = det.(fields{j});
                    else
                        detStruct(i).(fields{j}) = det.(fields{j});
                    end
                end
            end
        end
        function objAttribSt = copyObjectAttributesInStruct(obj,objAttribs,objAttribSt)
            % Copy ObjectAttributes into a struct.
            %
            % When objAttribSt is provided, copy data from objAttribs to
            % objAttribSt and when it is not provided, create an empty
            % structure and copy data into that.
            num = numel(objAttribs);
            if nargin<3
                objAttribSt = repmat(struct,num,1);
            end
            for k = 1:num
                objAtrib = getDataFromCellorStructArray(obj,objAttribs,k);
                fields = fieldnames(objAtrib);
                for l = 1: numel(fields)
                    objAttribSt(k).(fields{l}) = objAtrib.(fields{l});
                end
            end
        end
        function out = getDataFromCellorStructArray(~,data,idx)
            %Get data from a cell array or structure array at index idx.
            if iscell(data)
                %data is a cell array
                out = data{idx};
            else
                %data is a struct array
                out = data(idx);
            end
        end
        function outMeasParams = copyMeasurementParams(obj,inMeasParams,outMeasParams)
            % copyMeasurementParameters into a struct.
            %
            % When outMeasParams is provided, copy data from inMeasParams to
            % outMeasParams and when it is not provided, create an empty
            % structure and copy data into that.
            num = numel(inMeasParams);
            if nargin<3
                outMeasParams = repmat(struct,num,1);
            end
            for i = 1:num
                measParams = getDataFromCellorStructArray(obj,inMeasParams,i);
                fields = fieldnames(measParams);
                for j = 1:numel(fields)
                    if strcmpi(fields{j},'Frame')
                        outMeasParams(i).Frame = fusionCoordinateFrameType(measParams.Frame);
                    else
                        outMeasParams(i).(fields{j}) = measParams.(fields{j});
                    end
                end
            end
        end

        function rngSeed = getRNGSeed(obj,varargin)
            %Get RNG seed based on RNGSetting specified on the block.
            if strcmpi(obj.RNGSetting,'Not repeatable')
                %When RNGSetting is 'Not repeatable'use a new random seed.
                rngSeed = rand;
            elseif strcmpi(obj.RNGSetting,'Repeatable')
                %When RNGSetting is 'Repeatable' 
                if isequal(nargin,1)
                    %Previous seed does not exist, use a new random seed.
                    rngSeed = rand;
                else
                    %Previous seed exists, use seed from previous run.
                    previousSeed = varargin{1};
                    rngSeed=previousSeed;
                end
            else
                %RNGSetting is 'Specify seed', use seed specified from block mask.
                rngSeed = obj.InitialSeed;
            end
        end

        function [yOut,num] = convert2BusStruct(obj,dataObj,busIdx)
            
            % Convert scenario data into format that is compatible with
            % bus structure. when extracting data from scenario, it is not
            % compatible with the bus structure and needs to be converted.
            %
            % This function accepts data and bus index as input and returns
            % 'yOut' as a structure array of maximum length and 'num' as
            % number of valid indexes in 'yOut'.
            
            num = numel(dataObj);
            switch busIdx
                case 1 %Pose
                    sampleData = obj.pSampleStructs.Poses;
                    yOut = repmat(sampleData,obj.pMaxNumPlatforms,1);
                    yOut = copyParams(obj,dataObj,yOut);
                    if obj.IncludePlatformProfiles
                        yOut = addProfilesWithPlatformPoses(obj,yOut);
                    end

                case 2 %CoverageConfig
                    sampleData = obj.pSampleStructs.CoverageConfig;
                    yOut = repmat(sampleData,obj.pNumMaxCovCon,1);
                    yOut = copyParams(obj,dataObj,yOut);

                case 3 %Detections
                    sampleData = obj.pSampleStructs.Detections;
                    yOut = repmat(sampleData,obj.pMaxNumDetections,1);
                    yOut = copyDetectionsInStruct(obj,dataObj,yOut);

                case 4 %SensorConfigurations
                    sampleData = obj.pSampleStructs.SensorConfigurations;
                    yOut = repmat(sampleData,obj.pMaxNumSensors,1);
                    yOut = copyParams(obj,dataObj,yOut);

                case 5 %Emissions
                    sampleData = obj.pSampleStructs.Emissions;
                    yOut = repmat(sampleData,obj.pMaxNumEmissions,1);
                    yOut = objEmission2Struct(obj,dataObj,yOut);

                case 6 %EmitterConfigurations
                    sampleData = obj.pSampleStructs.EmitterConfigurations;
                    yOut = repmat(sampleData,obj.pMaxNumEmitters,1);
                    yOut = copyParams(obj,dataObj,yOut);
            end

            function out = copyParams(obj,in,out)
                % Copy parameters from a structure array 'in' to 'out'.
                % Here 'in' represents data extracted from scenario and
                % 'out' represent a structure which is compatible with
                % Simulink Bus.
                
                for i = 1:numel(in)
                    fields = fieldnames(in(i));
                    for j = 1:numel(fields)
                        if strcmpi(fields{j},'Frame')
                            out(i).Frame = fusionCoordinateFrameType(in(i).Frame);
                        elseif strcmpi(fields{j},'Orientation') && isa(in(i).(fields{j}),'quaternion')
                            out(i).Orientation = rotmat(in(i).Orientation,'Frame');
                        elseif strcmpi(fields{j},'FieldOfView')
                            if isequal(numel(in(i).FieldOfView),2)
                                %FieldOfView is a 2x1 vector.
                                out(i).FieldOfView(1:2,1) = in(i).FieldOfView;
                            else
                                %FieldOfView is a 2x2 matrix
                                out(i).FieldOfView = in(i).FieldOfView;
                            end
                        elseif strcmpi(fields{j},'ScanLimits')
                            if isequal(numel(in(i).ScanLimits),2)
                                %sensor or emitter can only scan in the
                                %azimuth direction and  is 1-by-2 row
                                %vector [minAz, maxAz] in degrees.
                                out(i).ScanLimits(1,1:2) = in(i).ScanLimits;
                            else
                                %sensor or emitter can scan in both azimuth
                                %and elevation direction.
                                % ScanLimits is a 2-by-2 matrix
                                out(i).ScanLimits = in(i).ScanLimits;
                            end
                        elseif strcmpi(fields{j},'LookAngle')
                            if isscalar(in(i).LookAngle)
                                %sensor or emitter can scan only in the
                                %azimuth direction.
                                out(i).LookAngle(1) = in(i).LookAngle;
                            else
                                %sensor or emitter can scan in both azimuth
                                %and elevation directions.
                                % LookAngle is a 2x1 vector.
                                out(i).LookAngle = in(i).LookAngle;
                            end
                        elseif strcmpi(fields{j},'MeasurementParameters')
                            out(i).MeasurementParameters = copyMeasurementParams(obj,...
                                in(i).MeasurementParameters,out(i).MeasurementParameters);
                        else
                            out(i).(fields{j}) = in(i).(fields{j});
                        end
                    end
                end
            end

        end
        
        function checkSensorOrEmitterUpdateRateCompatibility(obj,platforms,checkSensors,checkEmitters)
            %Check if update rate of a sensors or emitters present in
            %scenario is compatible with the sample time of the block. The
            %function checks if sample time of the sensor or emitter is an
            %integer multiple of block's sample time. If not, a warning is
            %issued.
            SmallTimeValue = 1e-4;
            for i = 1:numel(platforms)
                %Check Sensor sample time
                if checkSensors
                    for j = 1:numel(platforms{i}.Sensors)
                        sampleTime = 1/platforms{i}.Sensors{j}.UpdateRate;
                        val = mod(sampleTime,obj.SampleTime);
                        if val > SmallTimeValue
                            warning(message('fusion:simulink:ScenarioReaderBase:IncompatibleUpdateRate', ...                               
                                num2str(platforms{i}.Sensors{j}.UpdateRate), ...
                                num2str(platforms{i}.PlatformID), ...
                                num2str(platforms{i}.Sensors{j}.SensorIndex), ...
                                num2str(obj.SampleTime)));
                        end
                    end
                end
                %Check Emitter sample time
                if checkEmitters
                    for j = 1:numel(platforms{i}.Emitters)
                        sampleTime = 1/platforms{i}.Emitters{j}.UpdateRate;
                        val = mod(sampleTime,obj.SampleTime);
                        if val > SmallTimeValue
                            warning(message('fusion:simulink:ScenarioReaderBase:IncompatibleUpdateRate', ...
                                num2str(platforms{i}.Emitters{j}.UpdateRate), ...
                                num2str(platforms{i}.PlatformID), ...
                                num2str(platforms{i}.Emitters{j}.EmitterIndex), ...
                                num2str(obj.SampleTime)));
                        end
                    end
                end
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
            %Datatype for simulation time output is double and rest all
            %outputs are buses.
            numBusOutputs = sum(obj.pOutputSelector)-1;
            busDataType = cell(1,numBusOutputs);
            [busDataType{:}] = getOutputDataTypeImpl@matlabshared.tracking.internal.SimulinkBusUtilities(obj);
            %2nd output port Simulation time is a double. Insert it at second position.
            varargout ={busDataType{1},'double',busDataType{2:end}};
        end

        function flag = isInactivePropertyImpl(obj, prop)
            flag = isInactivePropertyImpl@matlabshared.tracking.internal.SimulinkBusUtilities(obj,prop);
            
            % Sensor related properties are visible only when
            % EnableCoverageConfigOutput or EnableSensorConfigOutput is
            % true. MaxNumSensors determines the bus size for coverage
            % config and sensor config.
            if any(strcmpi(prop,{'MaxSensorsSource','MaxNumSensors'...
                    'BusName4','BusName4Sources'}))
                if ~(obj.EnableCoverageConfigOutput || obj.EnableSensorConfigOutput)
                    flag=true;
                end
            end
            
            % Emitter related properties are visible only when
            % CoverageConfig output or EmitterConfig output is selected.
            % MaxNumEmitters determines the bus size for coverage
            % config and emitter config.
            if any(strcmpi(prop,{'MaxEmittersSource','MaxNumEmitters'...
                    'BusName6','BusName6Source'}))
                if  ~(obj.EnableEmitterConfigOutput || obj.EnableCoverageConfigOutput)
                    flag = true;
                end
            end
            
            %EnableOcclusion property is visible only when emission output
            %is selected.
            if ~obj.EnableEmissionsOutput && strcmpi(prop,'EnableOcclusion')
                flag=true;
            end

            %InitialSeed property is visible when RNGSetting is 'Specify
            %seed'.
            if ~strcmpi(obj.RNGSetting,'Specify seed')  && strcmpi(prop,'InitialSeed')
                flag = true;
            end
            if ~obj.EnableCoverageConfigOutput && startsWith(prop,'BusName2')
                flag = true;
            end
            if ~obj.EnableSensorConfigOutput && startsWith(prop,'BusName4')
                flag = true;
            end
            if ~obj.EnableEmitterConfigOutput && startsWith(prop,'BusName6')
                flag = true;
            end     
                        
            %Bus related properties are visible only when
            %the respective output is selected from the block.
            if ~obj.EnableEmissionsOutput && any(strcmpi(prop,{'MaxEmissionsSource',...
                    'MaxNumEmissions','BusName5','BusName5Source'}))
                flag = true;
            end
            if ~obj.EnableDetectionsOutput && any(strcmpi(prop,{'MaxDetectionsSource',...
                    'MaxNumDetections','BusName3','BusName3Source'}))
                flag = true;
            end   
            
            if strcmpi(obj.MaxPlatformsSource,'Auto') && strcmpi(prop,'MaxNumPlatforms')
                flag = true;
            end
            if strcmpi(obj.MaxDetectionsSource,'Auto') && strcmpi(prop,'MaxNumDetections')
                flag = true;
            end
            if strcmpi(obj.MaxEmissionsSource,'Auto') && strcmpi(prop,'MaxNumEmissions')
                flag = true;
            end
            if strcmpi(obj.MaxEmittersSource,'Auto') && strcmpi(prop,'MaxNumEmitters')
                flag = true;
            end
            if strcmpi(obj.MaxSensorsSource,'Auto') && strcmpi(prop,'MaxNumSensors')
                flag = true;
            end
        end

        function s = saveObjectImpl(obj)
            %save private and protected properties.

            s = saveObjectImpl@matlabshared.tracking.internal.SimulinkBusUtilities(obj);
            if isLocked(obj)
                s.pProfiles = obj.pProfiles;
                s.pSampleStructs = obj.pSampleStructs;
                s.pMaxNumDetections = obj.pMaxNumDetections;
                s.pMaxNumPlatforms = obj.pMaxNumPlatforms;
                s.pMaxNumEmissions = obj.pMaxNumEmissions;
                s.pMaxNumSensors = obj.pMaxNumSensors;
                s.pMaxNumEmitters = obj.pMaxNumEmitters;
                s.pNumMaxCovCon = obj.pNumMaxCovCon;
                s.pScenarioRecording  = obj.pScenarioRecording;
                s.pCurrentStep = obj.pScenarioRecording.CurrentStep;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            %Load private and protected properties.
            if wasLocked
                obj.pProfiles = s.pProfiles;
                obj.pSampleStructs = s.pSampleStructs;
                obj.pMaxNumDetections = s.pMaxNumDetections;
                obj.pMaxNumPlatforms = s.pMaxNumPlatforms;
                obj.pMaxNumEmissions = s.pMaxNumEmissions;
                obj.pMaxNumSensors = s.pMaxNumSensors;
                obj.pMaxNumEmitters = s.pMaxNumEmitters;
                obj.pNumMaxCovCon = s.pNumMaxCovCon;
                obj.pScenarioRecording  = s.pScenarioRecording;
                obj.pCurrentStep = s.pCurrentStep;
                obj.pScenarioRecording.CurrentStep = s.pCurrentStep;
            end
            loadObjectImpl@matlabshared.tracking.internal.SimulinkBusUtilities(obj,s,wasLocked);
        end

        function checkForEarthCentricScenario(obj,scenario)
            % Error if scenario is not Earth-centered and CoordinateSystem
            % is selected as 'Geodetic'
            if ~scenario.IsEarthCentered
                assert(~strcmpi(obj.CoordinateSystem,'Geodetic'), ...
                    message('fusion:simulink:ScenarioReaderBase:NotAnEarthcenteredScenario'));
            end
        end

        function out = getOutputNamesImpl(~)
            %Output names for the block
            out = {getString(message('fusion:simulink:ScenarioReaderBase:PlatformsOutput')),...
                getString(message('fusion:simulink:ScenarioReaderBase:SimTimeOutput'))...
                getString(message('fusion:simulink:ScenarioReaderBase:CovConOutput')),...
                getString(message('fusion:simulink:ScenarioReaderBase:DetectionOutput')),...
                getString(message('fusion:simulink:ScenarioReaderBase:SensorConfOutput')),...
                getString(message('fusion:simulink:ScenarioReaderBase:EmissionOutput')),...
                getString(message('fusion:simulink:ScenarioReaderBase:EmitterConfOutput')), ...
                };
        end

        function sts = getSampleTimeImpl(obj)
            %Set sample time for the block.
            sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.SampleTime);
        end

        function varargout = getInputNamesImpl(~)
            %Input names for the block
            varargout{1} = getString(message('fusion:simulink:ScenarioReaderBase:PlatformPoseInput'));
        end

        function flag = isInputSizeMutableImpl(~,~)
            flag = false;
        end

        function varargout = isOutputComplexImpl(~)
            [varargout{1:nargout}] = deal(false);
        end

        function varargout = isOutputFixedSizeImpl(obj)
            [FixdSizeOutput{1:numel(obj.pOutputSelector)}] = deal(true);
            varargout = FixdSizeOutput(obj.pOutputSelector);
        end

        function varargout = getOutputSizeImpl(obj)
            [outputSize{1:numel(obj.pOutputSelector)}] = deal([1 1]);
            varargout = outputSize(obj.pOutputSelector);
        end

        function numOutput = getNumOutputsImpl(obj)
            numOutput = sum(obj.pOutputSelector);
        end

        function validateInputsImpl(obj,varargin)
            %Validate input to the block.
            if strcmpi(obj.PlatformPoseSource,'Input port')
                %Error if input is not a struct.
                assert(isstruct(varargin{1}) , ...
                    message('fusion:simulink:ScenarioReaderBase:InvalidPlatformPoseInput'));
                assert(isfield(varargin{1},'Platforms') ...
                    && isfield(varargin{1},'NumPlatforms')  , ...
                    message('fusion:simulink:ScenarioReaderBase:InvalidPlatfromFields'));                                
            end
        end

        function num = getNumInputsImpl(obj)
            num=0;
            if strcmpi(obj.PlatformPoseSource,'Input port')
                num = 1;
            end
        end
    end
end

%Following functions are used while copying Measurement and
%MeasurementNoise in a detection structure. In the output bus structure,
%both of these parameters have maximum sizes and are zero padded wherever
%required. This is similar to the logic in DetectionConcatenation block.
function indOut = getLogicalIndices(x, y)

% Returns the logical indices to each element in x for in the larger array
% y, where x is a smaller array which is wholly contained by (a subset of)
% y. This enables on the first size(x) values in y to be set to the values
% in x, for example:
%   ind = getLogicalIndices(x, y);
%   y(ind) = x;

szy = size(y);

sz = getMinSize(x, y);
ind = reshape(tagIndicies(zeros(szy), sz),szy);

indOut = false(szy);
numOut = prod(szy);
indOut(1:numOut) = indOut(1:numOut) | ind(1:numOut)==numel(szy);
end

function z = tagIndicies(x, sz, depth)
% Tags a subset, sz, of active indices in the array x by incrementing
% their values by 1. This is a helper function for getLogicalIndices
if nargin<3
    depth = 1;
end
if depth~=ndims(x)
    y = tagIndicies(shiftdim(x,1),circshift(sz,-1),depth+1);
else
    y = x;
end
y(1:sz(1),:) = y(1:sz(1),:)+1;
if depth>1
    z = shiftdim(y,ndims(x)-1);
else
    z = y;
end
end

function [sz,scalarVal] = getMinSize(in1, in2)
% Return the minimum size and maximum dimension between inputs in1 and in2,
% sz. Returns a scalar value that is representative of the data stored in
% in1 and in2, scalarVal.

maxDims = max(ndims(in1),ndims(in2));
minDims = min(ndims(in1),ndims(in2));
sz = inf(1,maxDims);

sz1 = size(in1);
sz(1:minDims) = min(sz(1:minDims),sz1(1:minDims));

sz2 = size(in2);
sz(1:minDims) = min(sz(1:minDims),sz2(1:minDims));

sz(isinf(sz)) = 1;

if nargout<2
    return
end
if ~isempty(in1)
    scalarVal = in1(1);
elseif ~isempty(in2)
    scalarVal = in2(1);
else
    scalarVal = in1;
end
end

