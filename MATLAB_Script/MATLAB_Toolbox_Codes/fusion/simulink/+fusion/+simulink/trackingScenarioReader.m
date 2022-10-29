classdef(StrictDefaults, Hidden) trackingScenarioReader < fusion.simulink.internal.ScenarioReaderBase

    % trackingScenarioReader The block reads a trackingScenario object or a
    % Tracking Scenario Designer session file and outputs platform poses
    % and simulation time. You can configure the block to optionally output
    % detections, point clouds, emissions, sensor and emitter
    % configurations, and sensor coverages from the scenario.
    %
    % See also: trackingScenario , trackingScenarioDesigner

    % Copyright 2021 The MathWorks, Inc.

    properties(Nontunable)
        % ScenarioSource Source of scenario
        ScenarioSource = 'trackingScenario';
    end

    properties (Nontunable)
        % EnablePointCloudsOutput Point clouds
        EnablePointCloudsOutput (1, 1) logical = false;
    end

    properties(Nontunable)
        % BusName7Source Source of point cloud bus name
        BusName7Source = 'Auto'

        % BusName7 Specify point cloud bus name
        BusName7 = char.empty(1,0)

        % MaxLidarSensorsSource Source of maximum number of lidar sensors
        MaxLidarSensorsSource = 'Auto';

        % MaxNumLidarSensors Maximum number of lidar sensors
        MaxNumLidarSensors(1,1) {mustBeInteger,mustBeNonnegative} = 10;
    end

    properties(Hidden,Constant)
        BusName7SourceSet = matlab.system.StringSet({'Auto','Property'});
        MaxLidarSensorsSourceSet = matlab.system.StringSet({'Auto','Property'});
        ScenarioSourceSet = matlab.system.StringSet({'trackingScenario','Tracking Scenario Designer session file'});
    end

    properties(Constant, Access=protected)
        %   pBusPrefix A string that captures the base output bus name An
        %   output bus name are created by the object. It will have the
        %   name given here, appended by the number of tracks. Additional
        %   sub buses are created as well.
        pBusPrefix = {'BusPlatforms','BusCovConf','BusDetection',...
            'BusSensorConf','BusEmission','BusEmitterConf','BusPointCloud'};
    end

    properties (Hidden,Dependent, Access = protected)
        % A Logical array to select from different output options.
        % This property is hidden and used internally.
        pOutputSelector
    end

    properties(Nontunable, Access = protected)
        pScenarioRecording        
        pMaxNumLidarSensors;
    end

    properties(Constant, Access = protected)
        % pNumRecordingOutputParams Number of output arguments from
        % scenario recording.
        pNumRecordingOutputParams = 11;

        % pPointCloudBusIdx Index of point cloud bus
        pPointCloudBusIdx = 7;
    end

    methods
        function obj = trackingScenarioReader(varargin)
            setProperties(obj,nargin,varargin{:})
        end

        function val = get.pOutputSelector(obj)
            % First and second outputs are Platform pose and time. Both are
            % always available.
            val = [true,true,obj.EnableCoverageConfigOutput, ...
                obj.EnableDetectionsOutput,obj.EnableSensorConfigOutput, ...
                obj.EnableEmissionsOutput,obj.EnableEmitterConfigOutput, ...
                obj.EnablePointCloudsOutput];
        end

        function val = get.BusName7(obj)
            val = obj.BusName7;
            val = getBusName(obj,val,7);
        end

        function set.BusName7(obj,val)
            validateBusName(obj,val,'BusName7')
            obj.BusName7 = val;
        end

    end

    methods(Access = protected)
        function busOutput = formatBusOutput(obj,varargin)
            % Copy data collected form scenario into a Simulink
            % compatible bus structure.

            out = formatBusOutput@fusion.simulink.internal.ScenarioReaderBase(obj,varargin{:});

            %Extracting point cloud data from scenario
            PointClouds = varargin{10};
            PointCloudClusters = varargin{11};

            %Format lidar data on bus
            lidarData = sendToBus(obj,{PointClouds,PointCloudClusters},obj.pPointCloudBusIdx);

            %Point cloud is the last output from the block, appending it at
            %the end.
            busOutput = [out,lidarData];
            %Select requested output data.
            busOutput = busOutput(obj.pOutputSelector);
        end

        function validatePropertiesImpl(obj)
            %When source of platform pose is Input, optional outputs are
            %not available from the block.
            if strcmpi(obj.PlatformPoseSource ,'Input port')
                assert(~(obj.EnableCoverageConfigOutput || ...
                    obj.EnableDetectionsOutput || obj.EnableEmissionsOutput || ...
                    obj.EnableSensorConfigOutput || ...
                    obj.EnableEmitterConfigOutput || obj.EnablePointCloudsOutput), ...
                    message('fusion:simulink:trackingScenarioReader:InvalidSettingForPlatformPoseSource'));
            end
            % Workspace variable name or session file name are empty by
            % default. Validating scenario and generating data from
            % scenario only if these fields are not empty.
            if  (strcmpi(obj.ScenarioSource,'trackingScenario') && ~isempty(obj.WorkspaceVariableName)) || ...
                    (strcmpi(obj.ScenarioSource,'Tracking Scenario Designer session file') && ~isempty(obj.ScenarioFileName))
                validateScenarioAndGenerateData(obj);
            end
        end

        function validateScenarioAndGenerateData(obj)
            %Validates scenario
            if strcmpi(obj.ScenarioSource,'Tracking Scenario Designer session file')
                fullFileName = fusion.simulink.trackingScenarioReader.getFullScenarioFileName(obj.ScenarioFileName);
                % Error if specified scenario file does not exist.
                assert(isfile(fullFileName), ...
                    message('fusion:simulink:trackingScenarioReader:InvalidTSDSessionFileName',fullFileName));
            else
                % Scenario source is trackingScenario. Error if specified
                % workspace variable name is not valid.
                assert(isvarname(obj.WorkspaceVariableName), ...
                    message('fusion:simulink:trackingScenarioReader:InvalidScenarioVariableName',obj.WorkspaceVariableName));
            end
            %Generate simulation data from scenario.
            generateScenarioDataForSimulink(obj);
        end

        function [out, argsToBus] = defaultOutput(obj,busIdx,varargin)
            %Default output which will be placed on a bus.
            if isempty(obj.pSampleStructs)
                %At this point we need the bus structure to be ready.
                %If sample structures are empty, we must save sample
                %structures first, so that bus definitions can be
                %generated.
                validateScenarioAndGenerateData(obj);
            end

            out = struct.empty();
            if isequal(busIdx,obj.pPointCloudBusIdx)
                %LidarPointCloud
                if obj.EnablePointCloudsOutput
                    %Defaut output is a cell array of point clouds and
                    %point cloud clusters extracted from scenario.
                    out = {{obj.pSampleStructs.PointClouds.Points},{obj.pSampleStructs.PointClouds.Clusters}};
                end
                argsToBus = {};
            else
                [out, argsToBus] = defaultOutput@fusion.simulink.internal.ScenarioReaderBase(obj,busIdx,varargin{:});
            end
        end

        function y = sendToBus(obj,outObj,varargin)
            %Create bus structures for output ports.
            busIdx = varargin{1};
            y = struct.empty();
            if isequal(busIdx,obj.pPointCloudBusIdx) %LidarPointClouds
                if obj.EnablePointCloudsOutput
                    [lidarData,num] = convert2BusStruct(obj,outObj,busIdx);
                    y = struct('PointClouds',lidarData,'NumPointClouds',num);
                end
            else
                y = sendToBus@fusion.simulink.internal.ScenarioReaderBase(obj,outObj,varargin{:});
            end
        end

        function Poses = addProfilesWithPlatformPoses(obj,Poses)
            % Add profile information along with platform poses.
            Poses = addProfilesWithPlatformPoses@fusion.simulink.internal.ScenarioReaderBase(obj,Poses);
            % A lidar sensor always needs Mesh data, add Mesh Data with
            % profiles.
            platformProfiles = obj.pProfiles;
            for i = 1 : numel(Poses)
                %Find index of the platform to be updated. It is ensured
                %that a platform with unique PlatformID is always
                %available.
                idx = [platformProfiles.PlatformID] == Poses(i).PlatformID;
                if any(idx)
                    %Add mesh data with profiles
                    % Mesh data will be of fixed size on a bus. 'Vertices' and
                    % 'Faces' are matrices of maximum size. 'NumVertices' and
                    % 'NumFaces' are number of valid vertices and Faces in
                    % 'Vertices' and 'Faces' respectively.
                    % Data will be padded with NaNs wherever necessary.

                    NumVertices = size(platformProfiles(idx).Mesh.Vertices,1);
                    Poses(i).Mesh.NumVertices = NumVertices ;
                    Poses(i).Mesh.Vertices(1:NumVertices,:) = platformProfiles(idx).Mesh.Vertices;
                    NumFaces = size(platformProfiles(idx).Mesh.Faces,1);
                    Poses(i).Mesh.NumFaces = NumFaces;
                    Poses(i).Mesh.Faces(1:NumFaces,:) = platformProfiles(idx).Mesh.Faces;
                end
            end
        end

        function storeSampleStructs(obj,recordedData)
            % Store sample structures from recorded scenario data to
            % generate bus definitions. Some of the parameters have
            % slightly different structure in Simulink workflow as compared
            % to MATLAB. Modify sample structures in this function as
            % necessary.

            storeSampleStructs@fusion.simulink.internal.ScenarioReaderBase(obj,recordedData);

            if obj.IncludePlatformProfiles
                % Store sample mesh data.
                platformProfiles = obj.pProfiles;
                maxNumVertices = max(arrayfun(@(x)size(x.Vertices,1),[platformProfiles.Mesh]));
                maxNumFaces = max(arrayfun(@(x)size(x.Faces,1),[platformProfiles.Mesh]));
                obj.pSampleStructs.Poses.Mesh.NumVertices = maxNumVertices;
                obj.pSampleStructs.Poses.Mesh.Vertices = NaN(maxNumVertices,3);
                obj.pSampleStructs.Poses.Mesh.NumFaces = maxNumFaces;
                obj.pSampleStructs.Poses.Mesh.Faces = NaN(maxNumFaces,3);
            end


            %Parameter FieldOfView has different type for radar and lidar
            %sensors. In case of lidar its 2x2 matrix and in case of radar
            %it is a 2x1 vector. Since bus is fixed, we always use a 2x2
            %matrix and pad it with NaNs whenever required.
            obj.pSampleStructs.CoverageConfig.FieldOfView = NaN(2,2);
            obj.pSampleStructs.SensorConfigurations.FieldOfView = NaN(2,2);


            %Store point cloud information
            if ~isfield(recordedData,'PointClouds') || ...
                    ( isfield(recordedData,'PointClouds') && ...
                    all(cellfun(@isempty,{recordedData.PointClouds})))
                if obj.EnablePointCloudsOutput
                    %Warn user if point cloud output is enabled and
                    %scenario does not generate point cloud data.
                    warning(message('fusion:simulink:ScenarioReaderBase:EmptyDataWarning','PointClouds','Lidar sensor'));
                end

                % If scenario does not generate PointCloud data and user
                % still selects it as an output, sample structure is
                % populated from internally store structures otherwise bus
                % utility generates hard error for empty structs.
                obj.pSampleStructs.PointClouds = fusion.simulink.internal.getSampleStruct('PointClouds');
                obj.pMaxNumLidarSensors = obj.MaxNumLidarSensors;
            else
                LidarPointCloudDataSize = cellfun(@(x)size(x,1),recordedData(1).PointClouds,'UniformOutput',false);
                %The bus structure is generated as per maximum number of
                %points generated by lidar sensors present in scenario.
                numPoints=max([LidarPointCloudDataSize{:}]);
                obj.pSampleStructs.PointClouds = fusion.simulink.internal.getSampleStruct('PointClouds',numPoints);

                if strcmpi(obj.MaxLidarSensorsSource,'Auto')
                    %When source is Auto, get maximum number of lidar
                    %sensors from scenario. Maximum number of lidar sensors
                    %is equal to the maximum number of point clouds
                    %available in the scenario. This is required to
                    %generate a fixed size bus.
                    obj.pMaxNumLidarSensors = max(cellfun(@numel,{recordedData.PointClouds}));
                else
                    %When it is defined by user, use that
                    obj.pMaxNumLidarSensors = obj.MaxNumLidarSensors;
                end
            end
        end

        function s = saveObjectImpl(obj)
            %Save private and protected properties.
            s = saveObjectImpl@fusion.simulink.internal.ScenarioReaderBase(obj);
            if isLocked(obj)
                s.pMaxNumLidarSensors = obj.pMaxNumLidarSensors;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            %Load private and protected properties.
            if wasLocked
                obj.pMaxNumLidarSensors = s.pMaxNumLidarSensors;
            end
            loadObjectImpl@fusion.simulink.internal.ScenarioReaderBase(obj,s,wasLocked);
        end

        function varargout = getOutputNamesImpl(obj)
            %Get output port names
            outputNames = getOutputNamesImpl@fusion.simulink.internal.ScenarioReaderBase(obj);
            pointCloudOutputName = getString(message('fusion:simulink:trackingScenarioReader:PointCloudOutput'));
            %Point cloud is the last output from the block, append at last.
            outputNames = [outputNames,pointCloudOutputName];
            varargout = outputNames(obj.pOutputSelector);
        end

        function flag = isInactivePropertyImpl(obj, prop)

            flag = isInactivePropertyImpl@fusion.simulink.internal.ScenarioReaderBase(obj,prop);

            % CoordinateSystem is available only when source of scenario is
            % trackingScenario. Earth-centered scenarios are not supported
            % with TSD App. WorkspaceVariableName is hidden if
            % ScenarioSource is not trackingScenario.
            if ~strcmpi(obj.ScenarioSource,'trackingScenario') && ...
                    any(strcmpi(prop,{'CoordinateSystem','WorkspaceVariableName'}))
                flag=true;
            end

            %ScenarioFileName property is hidden if ScenarioSource is
            %trackingScenario
            if strcmpi(obj.ScenarioSource,'trackingScenario') && strcmpi(prop,'ScenarioFileName')
                flag = true;
            end

            %Lidar related properties are hidden if point cloud output is
            %not enabled.
            if ~obj.EnablePointCloudsOutput
                if any(strcmpi(prop,{'MaxLidarSensorsSource','MaxNumLidarSensors',...
                        'BusName7','BusName7Source'}))
                    flag = true;
                end
            end

            %MaxNumLidarSensors is hiddedn if MaxLidarSensorsSource is Auto
            if strcmpi(obj.MaxLidarSensorsSource,'Auto') && strcmpi(prop,'MaxNumLidarSensors')
                flag = true;
            end

            %Random number generation settings are available only when
            %sensor or emitter data is selected as output.
            if any(strcmpi(prop,{'RNGSetting','InitialSeed'})) && ~(obj.EnableDetectionsOutput...
                    || obj.EnableEmissionsOutput || obj.EnablePointCloudsOutput)
                flag = true;
            end

            % When source of PlatformPose is not scenario, optional outputs
            % are not available from the block.
            if ~strcmpi(obj.PlatformPoseSource,'Scenario') && ...
                    any((strcmpi(prop,...
                    {'EnableCoverageConfigOutput','EnableDetectionsOutput',...
                    'EnableSensorConfigOutput','EnableEmissionsOutput', ...
                    'EnableEmitterConfigOutput','EnablePointCloudsOutput', ...
                    'EnablePointCloudClustersOutput','RNGSetting'})))
                flag = true;
            end
        end

        function icon = getIconImpl(obj)
            % Generate icon for theblock
            if strcmpi(obj.ScenarioSource,'trackingScenario')
                scenarioName = obj.WorkspaceVariableName;
            else
                %Tracking Scenario Designer session file
                [~,scenarioName,~] = fileparts(obj.ScenarioFileName);
            end
            icon = getString(message('fusion:block:scenarioReaderTitle'));
            % Appending scenario name with block's text icon.
            icon = [icon,sprintf('\n\n[ %s ]',scenarioName)];
        end

        function generateDataFromTrackingScenario(obj,scenarioName,scenario,blkHandle,sampleTime)
            %Generate simulation data form a tracking scenario object.
            %Model stop time is used as scenario StopTime and UpdateRate is
            %derived from block's sample time. Scenario is recorded and
            %then data is replayed as per Simulink time.

            stopTime = 10; % The default sample time to use in case of error
            % If a finite stop time is provided in the model, use that.
            try
                modelH = bdroot(blkHandle);
                stopTime = double(evalinGlobalScope(modelH,get_param(modelH,'StopTime')));
            catch E %#ok<NASGU>
            end
            
            %Check if sensor or emitter UpdateRate is compatible with
            %block's SampleTime.
            checkSensorOrEmitterUpdateRateCompatibility(obj,scenario.Platforms, ...
                obj.EnableDetectionsOutput | obj.EnablePointCloudsOutput, ... %Check sensors when detection or point cloud output is enabled.
                obj.EnableEmissionsOutput ... %check emitters when emission output is enabled.
                );

            %Get the scenario data from store.
            sd = fusion.simulink.trackingScenarioReader.getScenarioData(scenarioName);

            %Check if we need to re-generate scenario data
            regenerate = checkIfScenarioDataNeedsToBeRegenerated(obj,sd,sampleTime,stopTime);

            if regenerate
                % Data needs to be regenerated, we must generate and set
                % it.
                restart(scenario);

                scenario.StopTime = stopTime; %Model stop time.
                scenario.UpdateRate = 1/sampleTime;  %Update rate is in Hz

                %Check if emitters should be included in recording.
                IEmitters = obj.EnableEmissionsOutput | obj.EnableEmitterConfigOutput ...
                    |obj.EnableCoverageConfigOutput;

                %Check if sensors should be included in recording.
                ISensors = obj.EnableCoverageConfigOutput | obj.EnableDetectionsOutput ...
                    | obj.EnableSensorConfigOutput ...
                    | obj.EnablePointCloudsOutput;

                %Occlusion in Emission propagation.
                HOcclusion =  obj.EnableOcclusion;

                % Set coordinate system to report platform poses.
                coord = 'Cartesian';
                if scenario.IsEarthCentered && ~strcmpi(obj.CoordinateSystem, coord)
                    %Geodetic coordinate supported only when scenario is
                    %earth centred.
                    coord = obj.CoordinateSystem;
                end

                % Get random seed to be used for recording based on random
                % number generation setting specified on the block.
                if isempty(sd)
                    % scenario data does not exist. Get new seed.
                    rngSeed = getRNGSeed(obj);
                else
                    % scenario data exist but needs to be regenerated. If
                    % random number generation setting is repeatable, use
                    % previously used seed.
                    rngSeed = getRNGSeed(obj,sd.InitialSeed);
                end


                % We always output unorganized point cloud output from the
                % block which is also default for a lidar sensor. Setting
                % HasOrganizedOutput property as false on lidar sensors.
                for i = 1:numel(scenario.Platforms)
                    for j = 1:numel(scenario.Platforms{i}.Sensors)
                        if isa(scenario.Platforms{i}.Sensors{j},'monostaticLidarSensor')
                            scenario.Platforms{i}.Sensors{j}.HasOrganizedOutput = false;
                        end
                    end
                end

                % Display a message on Simulink diagnostic viewer to inform
                % the user, if recording the scenario takes more time. The
                % message is displayed only if recording takes more then 10
                % seconds.
                ud.scenarioName = scenarioName;
                t = startTimer(modelH,blkHandle,ud);

                %Record the scenario.
                scenarioData.Recording = recordScenario(obj,scenario,IEmitters,ISensors,HOcclusion,coord,rngSeed);

                %Once scenario is recorded, delete the timer.
                if ~isempty(t)
                    deleteTimer(t);
                end

                %Storing profiles.
                profiles = platformProfiles(scenario);
                % platformProfiles function does not provide Mesh data,
                % adding it manually.
                for i = 1:numel(scenario.Platforms)
                    idx = [profiles.PlatformID] == scenario.Platforms{i}.PlatformID;
                    profiles(idx).Mesh.NumVertices = size(scenario.Platforms{i}.Mesh.Vertices,1);
                    profiles(idx).Mesh.Vertices = scenario.Platforms{i}.Mesh.Vertices;
                    profiles(idx).Mesh.NumFaces = size(scenario.Platforms{i}.Mesh.Faces,1);
                    profiles(idx).Mesh.Faces = scenario.Platforms{i}.Mesh.Faces;
                end
                scenarioData.Profiles = profiles;

                % Storing some meta data about recording. This meta data is
                % required to check if we need to generate the data again
                % next time.
                scenarioData.SampleTime = sampleTime;
                scenarioData.StopTime = stopTime;
                scenarioData.EnableOcclusion = HOcclusion;
                scenarioData.CoordinateSystem = coord;
                scenarioData.EmitterIncluded = IEmitters;
                scenarioData.SensorIncluded = ISensors;
                scenarioData.InitialSeed = rngSeed;

                %Set scenario data in the store.
                fusion.simulink.trackingScenarioReader.setScenarioData(scenarioName,scenarioData);
            end
            %Get the scenario data from the store
            cachedData = fusion.simulink.trackingScenarioReader.getScenarioData(scenarioName);
            %Set the trackingScenarioRecording object which will be used to
            %retrieve data at each time step.
            obj.pScenarioRecording = cachedData.Recording;
            %Set platform profiles
            obj.pProfiles = cachedData.Profiles;
            
            %Clear previously stored sample structures.
            obj.pSampleStructs=[];
            
            %Store sample structures from recorded data to generate bus
            %definitions.           
            storeSampleStructs(obj,cachedData.Recording.RecordedData);
        end

        function generateScenarioDataForSimulink(obj)
            %Generate scenario data from a trackingScenario object or a TSD
            %session file and store it.
            blkHandle = gcbh;
            %Get the model handle to look for scenario variable in
            %modelWorkspace.
            modelH = bdroot(blkHandle);

            if strcmpi(obj.ScenarioSource,'Tracking Scenario Designer session file')
                %Find fullfile name of the session file.
                fullFileName = fusion.simulink.trackingScenarioReader.getFullScenarioFileName(obj.ScenarioFileName);
                %Extract a trackingScenario object from Tracking Scenario Designer session file.
                scenario = fusion.simulink.trackingScenarioReader.getScenarioFromSessionFile(fullFileName);
                %Generate scenario data from trackingScenario.
                generateDataFromTrackingScenario(obj,obj.ScenarioFileName,scenario,blkHandle,obj.SampleTime);
            else
                % ScenarioSource is trackingScenario
                %
                %Load scenario variable from ModelWorkspace, base workspace
                %or data dictionary.
                scenario = fusion.simulink.trackingScenarioReader.getScenarioObjectFromWorkspace(obj.WorkspaceVariableName,modelH);
                % Now we have the scenario variable, validate if correct
                % coordinate system is selected from block mask to report
                % platform poses.
                checkForEarthCentricScenario(obj,scenario);
                %Generate scenario data from trackingScenario.
                generateDataFromTrackingScenario(obj,obj.WorkspaceVariableName,scenario,blkHandle,obj.SampleTime);
            end
        end


        function regenrate = checkIfScenarioDataNeedsToBeRegenerated(obj,existingData,sampleTime,stopTime)
            % Check if scenario data needs to be regenerated. Function
            % returns true if scenario needs to be recorded and false when
            % existing data can be reused.

            % Scenario is recorded once and stored in a persistent data
            % store. It is re-recorded only if one of following is true: 
            % 1. Model stop time exceeds previously recorded scenario's StopTime.
            % 2. SampleTime is changed from previously recorded scenario's sampleTime.
            % 3. Emitter data is requested from the block and it was not recorded earlier.
            % 4. Sensor data is requested from the block and it was recorded earlier.
            % 5. Random number generator seed is changed.
            % 6. Scenario was not recorded earlier. Data does not exist.
            % 7. Occlusion status is changed.
            % 8. Coordinate system to report platform pose is changed.

            if ~isempty(existingData)
                % scenario data exists.

                % If stop time in the model exceeds the scenario stop time,
                % then regenerate the scenario Data.
                stopTimeExceedsScenarioTime = stopTime > round(existingData.StopTime,6); % rounding StopTime up to 6 decimal places(microseconds).
                %If sample time is changed regenerate scenario Data.
                sampleTimeChanged = sampleTime ~= round(existingData.SampleTime,6);

                IEmitters = obj.EnableEmissionsOutput | obj.EnableEmitterConfigOutput ...
                    | obj.EnableCoverageConfigOutput;
                ISensors = obj.EnableCoverageConfigOutput | obj.EnableDetectionsOutput ...
                    | obj.EnableSensorConfigOutput  ...
                    | obj.EnablePointCloudsOutput;

                occlusionStatusChanged = ~isequal(obj.EnableOcclusion,existingData.EnableOcclusion);
                CoordinateSystemChanged = ~isequal(obj.CoordinateSystem,existingData.CoordinateSystem);

                RNGSeedChnaged=false;
                if strcmpi(obj.RNGSetting,'Not repeatable') || ...
                        (strcmpi(obj.RNGSetting,'Specify seed') && ~isequal(obj.InitialSeed,existingData.InitialSeed))
                    %Regenerate if RNG setting is Not repeatable or initial
                    %seed is changed.
                    RNGSeedChnaged=true;
                end

                if IEmitters
                    %Regenerate if emitter data was not included earlier or
                    %RNGSeed is changed.
                    IEmitters = IEmitters  & (~existingData.EmitterIncluded | RNGSeedChnaged);
                end
                if ISensors
                    %Regenerate if sensor data was not included earlier or
                    %RNGSeed is changed.
                    ISensors = ISensors & (~existingData.SensorIncluded | RNGSeedChnaged);
                end

                % Regenerate scenario data if any of following is true.
                regenrate = stopTimeExceedsScenarioTime | sampleTimeChanged | ...
                    occlusionStatusChanged | CoordinateSystemChanged | IEmitters | ISensors;
            else
                % Empty scenario data means that we never created the
                % scenario data for this scenario. We must create it.
                regenrate=true;
            end

        end
    end

    methods (Access=protected)
        function [yOut,num] = convert2BusStruct(obj,dataObj,busIdx)
            % Convert scenario data into format that is compatible with bus
            % structure, when extracting data from scenario, it is not
            % compatible with the bus structure and needs to be converted.
            % Function accepts data and bux index as input and returns
            % 'yOut' as a structure array of maximum length and 'num' as
            % number of valid indexes in 'yOut'.

            %Lidar Point Clouds
            if isequal(busIdx,obj.pPointCloudBusIdx)
                %get sample lidar data.
                sampleData = obj.pSampleStructs.PointClouds;
                %copy sample data.
                yOut = repmat(sampleData,obj.pMaxNumLidarSensors,1);
                
                
                num = numel(dataObj{1});
                %When input is lidar sensor data, dataObj is a cell array
                %of point clouds and point cloudclusters at a given time
                %step.
                for i =1:num
                    %Iterate over all the point clouds and point cloud
                    %clusters and copy data into the output structure.

                    %The output contains matrices 'Points' and 'Clusters'
                    %of maximum length, NumPoints indicates number of valid
                    %indexes in both the matrices.
                    yOut(i).NumPoints = size(dataObj{1}{i},1);

                    %Points is a P-by-3 matrix of scalars, where P is the
                    %product of the numbers of elevation and azimuth
                    %channels.
                    % First element is dataObj is always point clouds.
                    yOut(i).Points(1:yOut(i).NumPoints,:) = dataObj{1}{i};

                    % Clusters is P-by-2 matrix of scalars, where P is the
                    % product of the numbers of elevation and azimuth
                    % channels. For each column of the matrix, the first
                    % element represents the PlatformID of the target
                    % generating the point whereas the second element
                    % represents the ClassID of the target.

                    % Second element is dataObj is always point clouds
                    % clusters.
                    yOut(i).Clusters(1:yOut(i).NumPoints,:) = dataObj{2}{i};
                end
            else
                [yOut,num]=convert2BusStruct@fusion.simulink.internal.ScenarioReaderBase(obj,dataObj,busIdx);
            end
        end
    end

    methods(Static)
        function isEnabled = isBrowseEnabled(hblk)
            % Return true if browse button should be enabled on block mask
            isEnabled = false;
            if isnumeric(hblk)
                %Enable when ScenarioSource is Tracking Scenario Designer session file
                isEnabled = strcmpi(get_param(hblk,'ScenarioSource'), 'Tracking Scenario Designer session file');
                isEnabled = isEnabled && ~matlab.system.display.Action.isSystemLocked(hblk);
            end
        end

        function isEnabled = isRefreshEnabled(hblk)
            % Return true if refresh button should be enabled on block mask
            isEnabled = ~matlab.system.display.Action.isSystemLocked(hblk);
        end
    end

    methods (Hidden)
        function browseButtonCallback(~, h)
            %Opens a file browser to select a Tracking Scenario Designer
            %session file.
            %
            %This function gets called wherever browse button is clicked
            %from the block mask.

            %Set a title for file browser.
            title = getString(message('fusion:simulink:trackingScenarioReader:FileBrowserTitle'));
            %Set file specification.
            spec = {'*.mat', getString(message('fusion:simulink:trackingScenarioReader:SessionFileSpecification'))};

            %Open a file browser and get file name and path from it.
            [file, path] = uigetfile(spec, title);

            if file
                % Use the exact path to prevent cd changing.
                file = fullfile(path, file);
                blk = h.SystemHandle;
                %Set ScenarioFileName on the block mask.
                set_param(blk, 'ScenarioFileName', file);
            end
        end

        function refreshButtonCallback(obj, h)
            % When refresh scenario data button is pressed from block mask,
            % clear scenario data from data store and regenerate.
            blk = h.SystemHandle;
            if strcmpi(obj.ScenarioSource, 'trackingScenario')
                scenarioName = get_param(blk,'WorkspaceVariableName');
            elseif strcmpi(obj.ScenarioSource, 'Tracking Scenario Designer session file')
                scenarioName = get_param(blk,'ScenarioFileName');
            end
            if ~isempty(scenarioName)
                %clear existing scenario data from data store.
                fusion.simulink.trackingScenarioReader.clearScenarioData(scenarioName);
                %Regenerate fresh data.
                generateScenarioDataForSimulink(obj);
            end
        end
    end

    methods(Static)
        function scenario = getTrackingScenarioObject(blkHandle)
            % A static method to get the trackingScenario object from the
            % block
            %
            % This function can be used as an internal API to get a
            % trackingScenario object directly form the block.

            scenarioSource =   get_param(blkHandle,'ScenarioSource');
            modelH = bdroot(blkHandle);
            if strcmpi(scenarioSource, 'Tracking Scenario Designer session file')
                %Extract trackingScneario object from Tracking Scenario Designer session file.
                scenario = fusion.simulink.trackingScenarioReader.getScenarioFromSessionFile...
                    (get_param(blkHandle,'ScenarioFileName'));
            else
                % scenarioSource is trackingScenario
                % Get trackingScenario object from workspace.
                scenario = fusion.simulink.trackingScenarioReader.getScenarioObjectFromWorkspace ...
                    (get_param(blkHandle,'WorkspaceVariableName'),modelH);
            end
        end

        function profiles = getPlatformProfiles(blkHandle)
            % A static method to get the platform profiles from the block
            %
            % This function can be used as an internal API to get 
            % platform profiles directly from the block.

            scenarioSource =   get_param(blkHandle,'ScenarioSource');
            if strcmpi(scenarioSource, 'Tracking Scenario Designer session file')
                scenarioName=get_param(blkHandle,'ScenarioFileName');
            else
                %scenarioSource is trackingScenario
                scenarioName =  get_param(blkHandle,'WorkspaceVariableName');
            end
            % Check if we have generated scenario data.
            cachedData = fusion.simulink.trackingScenarioReader.getScenarioData(scenarioName);
            if isempty(cachedData)
                %Scenario data does not exist, get trackingScenario object
                %and extract profiles from it.
                scenario = fusion.simulink.trackingScenarioReader.getTrackingScenarioObject(blkHandle);
                profiles = platformProfiles(scenario);
                %Adding Mesh Data with profiles.
                for i = 1:numel(scenario.Platforms)
                    %Iterate over all the platforms in the scenario and add
                    %Mesh data with profiles.
                    idx = [profiles.PlatformID] == scenario.Platforms{i}.PlatformID;
                    profiles(idx).Mesh = scenario.Platforms{i}.Mesh;
                end
            else
                %Scenario data exists, get profiles from recorded data.
                profiles = cachedData.Profiles;
            end
        end
    end

    methods(Static, Access = protected)
        function scenario = getScenarioObjectFromWorkspace(WorkspaceVariableName,modelH)
            %Get a trackingScenario object from a variable
            %WorkspaceVariableName.
            %
            %The function looks for the variable in model workspace, base
            %workspace and data dictionaries.

            try
                % First, check if the variable is in the model workspace.
                mdlWks = get_param(modelH,'ModelWorkspace');
                if mdlWks.hasVariable(WorkspaceVariableName)
                    scene = mdlWks.getVariable(WorkspaceVariableName);
                else
                    % Look into the global scope of the model. This
                    % includes MATLAB workspace and data dictionary.
                    scene = evalinGlobalScope(modelH,WorkspaceVariableName);
                end
            catch E
                msg = message('fusion:simulink:ScenarioReaderBase:UndefindedScenarioVariable',WorkspaceVariableName);
                throwAsCaller(MException(msg));
            end
            if isa(scene,'trackingScenario')
                % scene is a valid trackingScenario object. clone the
                % workspace scenario object and so that we do not modify
                % the workspace variable.
                scenario = clone(scene);
            else
                %Generate an error if scene is not a valid trackingScenario
                %object.
                error(message('fusion:simulink:trackingScenarioReader:InvalidScenarioVariableName',WorkspaceVariableName));
            end
        end

        function scenario = getScenarioFromSessionFile(ScenarioFileName)
            %Extract a trackingScenaio object from a Tracking Scenario Designer session file.
            try
                %Load the mat file into a variable.
                scenarioData = load(ScenarioFileName,'-mat');
               
                %Generate trackingScenario object from session file.
                simMode = 'detections'; %generate trackingScenario with sensors.
                scenario =  scenarioData.data.DataModel.generateScenario(simMode);
            catch E
                msg =  message('fusion:simulink:trackingScenarioReader:InvalidTSDSessionFileName',ScenarioFileName);
                throwAsCaller(MException(msg));
            end
            %Generate an error if the loaded scenario is not a
            %trackingScenario object.
            assert(isa(scenario,'trackingScenario'),message('fusion:simulink:trackingScenarioReader:InvalidTSDSessionFileName',ScenarioFileName));
        end

        function fullName = getFullScenarioFileName(rawName)
            %Get file name along with file path.
            if isempty(rawName)
                fullName = rawName;
            else
                % Check if the rawname can be converted into a full file
                % name.  This happens in the file is on the path or PWD
                fullName = which(rawName);
                if isempty(fullName)
                    % If empty, it is not on the path or PWD.  Try to use
                    % the passed value.
                    fullName = rawName;
                end
            end                      
        end

        function setScenarioData(scenarioName,scenarioData)
            %Set scenario data into the data store.
            fusion.simulink.internal.setGetScenarioData(scenarioName,scenarioData);
        end

        function clearScenarioData(scenarioName)
            %Clear scenario data for scenarioName from the data store.
            fusion.simulink.internal.setGetScenarioData(scenarioName,'clear');
        end

        function scenarioData = getScenarioData(scenarioName)
            %Get scenario data from data store.
            scenarioData = fusion.simulink.internal.setGetScenarioData(scenarioName);
        end
    end

    methods(Static, Access = protected)
        function header = getHeaderImpl
            %Mask header title and summary.
            header = matlab.system.display.Header(...
                'Title', getString(message('fusion:block:scenarioReaderTitle')), ...
                'Text', getString(message('fusion:block:scenarioReaderDesc')));
        end

        function simMode = getSimulateUsingImpl  
            % Simulation mode 'Interpreted execution' is supported. 
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            %Hide the Simulate Using dropdown from the block mask.
            flag = false;
        end

        function groups = getPropertyGroupsImpl
            %Block mask design
            ScenarioSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','ScenarioSection', ...
                {'ScenarioSource','WorkspaceVariableName','ScenarioFileName','PlatformPoseSource',...
                'SampleTime','CoordinateSystem' });

            GroundTruthSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','GroundTruthSection', ...
                {'IncludePlatformProfiles'});

            SensorSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','SensorSection', ...
                {'EnableDetectionsOutput', 'EnablePointCloudsOutput', ...
                'EnableEmissionsOutput','EnableOcclusion'});


            ConfigSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','ConfigSection', ...
                {'EnableCoverageConfigOutput','EnableSensorConfigOutput',...
                'EnableEmitterConfigOutput'});


            RNGSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','RNGSection', ...
                {'RNGSetting','InitialSeed'});

            browse = matlab.system.display.Action(@(h,obj) browseButtonCallback(obj, h), ...
                'Label',getString(message('fusion:simulink:trackingScenarioReader:BrowseButtonLabel')), ...
                'Placement','PlatformPoseSource','Alignment','right');
            matlab.system.display.internal.setCallbacks(browse, ...
                'IsEnabledFcn', @fusion.simulink.trackingScenarioReader.isBrowseEnabled);

            refresh = matlab.system.display.Action(@(h,obj) refreshButtonCallback(obj, h), ...
                'Label',getString(message('fusion:simulink:trackingScenarioReader:RefreshButtonLabel')), ...
                'Placement','PlatformPoseSource','Alignment','right');
            matlab.system.display.internal.setCallbacks(refresh, ...
                'IsEnabledFcn', @fusion.simulink.trackingScenarioReader.isRefreshEnabled);

            ScenarioSection.Actions = [browse,refresh];


            PlatformBusSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','PlatformBusSection', ...
                {'BusNameSource','BusName',...
                'MaxPlatformsSource','MaxNumPlatforms'});


            PointCloudBusSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','PointCloudBusSection', ...
                {'BusName7Source','BusName7',...
                'MaxLidarSensorsSource','MaxNumLidarSensors'});


            EmitterBusSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','EmitterBusSection', ...
                {'BusName5Source','BusName5',...
                'MaxEmissionsSource','MaxNumEmissions'});

            DetectionsBusSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','DetectionsBusSection', ...
                {'BusName3Source','BusName3',...
                'MaxDetectionsSource','MaxNumDetections'});

            ConfigBusSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackingScenarioReader','ConfigBusSection', ...
                {'BusName2Source','BusName2', ...
                'BusName4Source','BusName4', ...   
                'BusName6Source','BusName6', ...
                'MaxSensorsSource','MaxNumSensors', ...
                'MaxEmittersSource','MaxNumEmitters'
                });

            ScenarioGroup = matlab.system.display.SectionGroup(...
                'Title',getString(message('fusion:simulink:trackingScenarioReader:TitleScenarioGroup')), ...
                'Sections',ScenarioSection);

            OutputSettingsGroup = matlab.system.display.SectionGroup(...
                'Title',getString(message('fusion:simulink:trackingScenarioReader:TitleOutputSettingsGroup')), ...
                'Sections',[GroundTruthSection,SensorSection,...
                ConfigSection,RNGSection]);

            PortSettingGroup = matlab.system.display.SectionGroup(...
                'Title',getString(message('fusion:simulink:trackingScenarioReader:TitlePortSettingGroup')), ...
                'Sections',[PlatformBusSection,DetectionsBusSection,...
                PointCloudBusSection,EmitterBusSection,ConfigBusSection]);

            groups = [ScenarioGroup,OutputSettingsGroup,PortSettingGroup];

        end
    end
end

function t = startTimer(modelH,blkHandle,ud)
%Start a timer and call recordingTimerCallback after timeout.
%recordingTimerCallback function displays a message on Simulink diagonostic
%viewer.
t = [];
if ~isempty(modelH) && strcmp(get_param(modelH,'SimulationStatus'),'stopped')
    try
        maskObj = Simulink.Mask.get(blkHandle);
        dlg = maskObj.getDialogHandle();
    catch E %#ok<NASGU>
        dlg = [];
    end
    if ~isempty(dlg)
        ud.ModelName = get_param(modelH,'Name');
        t = timer('StartDelay', 10, ... %timeout of 10 seconds
            'Tag', 'GeneratingTrackingScenarioData', ...
            'UserData', ud, ... %data to send with callback
            'TimerFcn', @recordingTimerCallback);
        start(t); %start the timer
    end
end
end

function recordingTimerCallback(t, ~)
% Display a message on the diagnostic viewer indicating that we are
% generating data.
sldiagviewer.createStage(getString(message('fusion:block:scenarioReaderTitle')),'ModelName', t.UserData.ModelName);
sldiagviewer.reportInfo(getString(message('fusion:simulink:trackingScenarioReader:RecordingTimerCallbackMessage',t.UserData.scenarioName)));
slmsgviewer.Instance().show(); % Open the diagnostic viewer
end

function deleteTimer(t)
% Delete the timer object
try
    if strcmp(t.Running, 'on')
        stop(t);
    end
    delete(t);
catch ME %#ok<NASGU>
    % DO NOTHING
end
end