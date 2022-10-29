classdef trackingScenarioRecording < handle
 % trackingScenarioRecording   A tracking scenario recording
 % TSR = trackingScenarioRecording(RECORDEDDATA) returns TSR, a
 % trackingScenarioRecording object. RECORDEDDATA is a struct with the same
 % fields as the output of the record method of trackingScenario.
 % 
 % TSR = trackingScenarioRecording(...,'Name', value) allows you to
 % specify additional properties as name-value pairs.
 %
 % read method syntax:
 %   [TIME, POSES, COVCON, DETS, SENCON, SENPLS, EMM, EMMCONF, EMMPLS] = read(TSR) 
 %   returns the following data:
 %      TIME     Simulation timestamp, as a scalar.
 %      POSES    Platform poses, as an array of struct.
 %      COVCON   Coverage configuration, as a struct array.
 %      DETS     Detections, as a cell array of objectDetection objects.
 %      SENCON   Sensor configurations, as an array of struct.
 %      SENPLS   Sensor platform IDs, as a numeric array.
 %      EMM      Emissions, as a cell array of radarEmission and
 %               sonarEmission objects.
 %      EMMCONF  Emitter configurations, as an array of struct.
 %      EMMPLS   Emitter platform IDs, as a numeric array.
 %
 % trackingScenarioRecording properties:
 %   RecordedData - The data stored in the recording
 %   CurrentTime  - Latest timestamp read
 %   CurrentStep  - Latest step read
 %
 % trackingScenarioRecording methods:
 %   isDone  - Return true when the read reaches the recoding end
 %   read    - Get the next set of recorded data and advance the recording
 %   reset   - Reset the read to the beginning of the recording
 %
 % % Example: Run a recorded scenario
 % % Load a recorded scenario called recordedData from a file
 % load recordedScenario recordedData 
 %
 % % Construct a scenario reader using the loaded recording1
 % recording = trackingScenarioRecording(recordedData);
 %
 % % Construct a theater plot to display the recorded data
 % tp = theaterPlot('AxesUnits', ["km" "km" "km"], 'XLimits',[-50 50]*1e3,...
 %    'YLimits',[-50 50]*1e3,'ZLimits', [-20 20]*1e3);
 % to = platformPlotter(tp,'DisplayName','Tower','Marker','d');
 % pp = platformPlotter(tp,'DisplayName','Targets');
 % dp = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','black');
 % cp = coveragePlotter(tp,'DisplayName','Radar Beam');
 %
 % scanBuffer = {};
 % while ~isDone(recording)
 %     % Step the reader to read the next frame of data
 %     [simTime,poses,covcon,dets,senconfig] = read(recording);
 %     scanBuffer = [scanBuffer;dets]; %#ok<AGROW>
 %     plotPlatform(to,poses(1).Position);
 %     plotPlatform(pp,reshape([poses(2:4).Position]',3,[])');
 %     plotCoverage(cp,covcon);
 %     if ~isempty(dets)
 %        plotDetection(dp,cell2mat(cellfun(@(c) c.Measurement(:)', scanBuffer, 'UniformOutput', false)));
 %     end
 %
 %     % Clear the buffer when a 360 degree scan is complete
 %     if senconfig.IsScanDone
 %         scanBuffer = {};
 %         dp.clearData;
 %     end
 % end
 %
 % See also: trackingScenario/record, radarEmission, sonarEmission,
 % objectDetection, monteCarloRun, coverageConfig
 
 %   Copyright 2019 The MathWorks, Inc.

    properties
        %RecordedData    The recorded data stored in the recording
        % Specify the recorded data struct. This property must be set on
        % construction. See trackingScenario/record to see which fields are
        % expected.
        RecordedData
    end
    properties(Dependent)
        % CurrentTime  Timestamp of the latest read data 
        % You may specify the CurrentTime. In the next call to read, the
        % method resumes reading from the first recorded data step that has
        % SimulationTime > CurrentTime
        %
        % Default: 0
        CurrentTime
        
        % CurrentStep  Step index of the latest read data
        % You may specify the CurrentStep. In the next call to read, the
        % method resumes reading from the first recorded data step.
        %
        % Default: 0
        CurrentStep
    end    

    % Pre-computed constants
    properties(Access = private)
        pNumSteps
        pStopTime
        pRecordedData % The part of the RecordedData used by the reader
        pCurrentStep
        pCurrentTime
        pIsFirstRead = true;
        pExistingFields;
    end

    methods
        function obj = trackingScenarioRecording(varargin)
            % Support name-value pair arguments when constructing object
            p = inputParser;
            addRequired(p, 'RecordedData');
            addParameter(p, 'CurrentTime', 0);
            addParameter(p, 'CurrentStep', 0);
            parse(p, varargin{:});
            obj.RecordedData = p.Results.RecordedData;
            obj.CurrentTime  = p.Results.CurrentTime;
            obj.CurrentStep  = p.Results.CurrentStep;
        end
        
        function set.RecordedData(obj,value)
            validateattributes(value,{'struct'},{'nonempty'},mfilename,'RecordedData')
            obj.RecordedData = value;
        end
        
        function set.CurrentTime(obj,value)
            validateattributes(value,{'numeric'},...
                {'real','nonsparse','nonnegative','finite','scalar'},...
                mfilename,'CurrentTime')
            if obj.pIsFirstRead
                setup(obj)
            end
            obj.pCurrentStep = find([obj.pRecordedData.SimulationTime] >= value, 1, 'first') - 1;
            if isempty(obj.pCurrentStep)
                error(message('fusion:trackingScenarioRecording:invalidTime','SimulationTime'))
            end
            obj.pCurrentTime = value;
        end
        
        function value = get.CurrentTime(obj)
            value = obj.pCurrentTime;
        end
        
        function set.CurrentStep(obj,value)
            validateattributes(value,{'numeric'},...
                {'real','nonsparse','nonnegative','finite','scalar','integer'},...
                mfilename,'RecordedData')
            if obj.pIsFirstRead
                setup(obj)
            end
            if value > obj.pNumSteps
                error(message('fusion:trackingScenarioRecording:invalidStep','RecordedData'))
            end
            obj.pCurrentStep = value;
            obj.pCurrentTime = obj.pRecordedData(max([1,obj.pCurrentStep])).SimulationTime;
        end
        
        function value = get.CurrentStep(obj)
            value = obj.pCurrentStep;
        end
        
        function [simTime,poses,covcon,detections,sensorConfigurations, ...
                sensorPlatformIDs,emissions,emmiterConfigurations, ...
                emitterPlatformIDs,ptClouds,ptCloudClusters] = read(obj)
            %READ Read the next step from a trackingScenarioRecording
            % [TIME, POSES, COVCON, DETS, SENCON, SENPLS, EMM, EMMCONF, EMMPLS,PC,PCCLUS] = read(TSR)
            % returns the following data:
            %   TIME     Simulation timestamp, as a scalar.
            %   POSES    Platform poses, as an array of struct.
            %   COVCON   Coverage configurations, as an array of struct.
            %   DETS     Detections, as a cell array of objectDetection objects.
            %   SENCON   Sensor configurations, as an array of struct.
            %   SENPLS   Sensor platform IDs, as a numeric array.
            %   EMM      Emissions, as a cell array of radarEmission and
            %            sonarEmission objects.
            %   EMMCONF  Emitter configurations, as an array of struct.
            %   EMMPLS   Emitter platform IDs, as a numeric array.
            %   PC       Point clouds, as an array of pointCloud objects
            %   PCCLUS   Ground truth clusters of each point cloud
            
            validateattributes(obj,{mfilename},{'scalar'},mfilename,'read')
            
            if obj.pIsFirstRead
                setup(obj)
            end
            
            if ~isDone(obj)
                obj.pCurrentStep = obj.pCurrentStep + 1;
            else
                error(message('fusion:trackingScenarioRecording:recordingEnded','isDone'));
            end
            
            recordedData = obj.pRecordedData;
            currentRecordedData = recordedData(obj.CurrentStep);
            
            % SimulationTime and Poses always exist
            simTime = currentRecordedData.SimulationTime;
            obj.pCurrentTime = simTime;
            poses = currentRecordedData.Poses;
            
            % The following may or may not exist
            covcon = readOptionalField(obj,currentRecordedData,'CoverageConfig');
            detections = readOptionalField(obj,currentRecordedData,'Detections');
            sensorConfigurations = readOptionalField(obj,currentRecordedData,'SensorConfigurations');
            sensorPlatformIDs = readOptionalField(obj,currentRecordedData,'SensorPlatformIDs');
            emissions = readOptionalField(obj,currentRecordedData,'Emissions');
            emmiterConfigurations = readOptionalField(obj,currentRecordedData,'EmitterConfigurations');
            emitterPlatformIDs = readOptionalField(obj,currentRecordedData,'EmitterPlatformIDs');
            ptClouds = readOptionalField(obj,currentRecordedData,'PointClouds');
            ptCloudClusters = readOptionalField(obj,currentRecordedData,'PointCloudClusters');
        end
        
        function reset(obj)
            %RESET  Reset the read to the beginning of the recording
            % RESET(obj) resets the read to the beginning of the recording
            validateattributes(obj,{mfilename},{'scalar'},mfilename,'read')
            obj.pCurrentTime = obj.RecordedData(1).SimulationTime;
            obj.pCurrentStep = 0;
        end

        function status = isDone(obj)
            %isDone  Return true when the read reaches the recoding end
            % isDone(obj) returns true if end of data has been reached.
            % Use isDone to check if the end of the recording has been
            % reached before reading the next step in the recording.
            validateattributes(obj,{mfilename},{'scalar'},mfilename,'read')
            
            if obj.pIsFirstRead
                setup(obj)
            end
            status = (obj.pCurrentStep > obj.pNumSteps - 1);
        end
    end

    methods(Access = protected)
        function setup(obj)
            % Find the struct that is actually a RecordedData file we can read
            setupRecordedData(obj)
            
            % Perform one-time calculations, such as computing constants
            obj.pNumSteps = numel(obj.pRecordedData);
            obj.pStopTime = obj.pRecordedData(end).SimulationTime;
            
            obj.pCurrentStep = 1;
            obj.pCurrentTime = obj.pRecordedData(1).SimulationTime;
            obj.pIsFirstRead = false;
            obj.pExistingFields = struct(...
                'Detections', isfield(obj.pRecordedData, 'Detections'), ...
                'SensorConfigurations', isfield(obj.pRecordedData, 'SensorConfigurations'), ...
                'SensorPlatformIDs', isfield(obj.pRecordedData, 'SensorPlatformIDs'), ...
                'Emissions', isfield(obj.pRecordedData, 'Emissions'), ...
                'EmitterConfigurations', isfield(obj.pRecordedData, 'EmitterConfigurations'), ...
                'EmitterPlatformIDs', isfield(obj.pRecordedData, 'EmitterPlatformIDs'), ...
                'CoverageConfig', isfield(obj.pRecordedData, 'CoverageConfig'),...
                'PointClouds',isfield(obj.pRecordedData,'PointClouds'),...
                'PointCloudClusters',isfield(obj.pRecordedData,'PointCloudClusters'));
        end
        
        function setupRecordedData(obj)
            % Look for the data expected in the RecordedData
            if containsRecordedDataFormat(obj, obj.RecordedData)
                validateattributes([obj.RecordedData.SimulationTime], {'numeric'}, ...
                    {'real','increasing','finite'},mfilename,'RecordedData.SimulationTime');
                obj.pRecordedData = obj.RecordedData;
            else
                error(message('fusion:trackingScenarioRecording:mustContainFields','''SimulationTime'' and ''Poses'''))
            end
        end
        
        function tf = containsRecordedDataFormat(~, this)
            tf = isstruct(this) && all(isfield(this, {'SimulationTime', 'Poses'}));
        end
        
        function value = readOptionalField(obj, data, fieldName)
            if obj.pExistingFields.(fieldName)
                value = data.(fieldName);
            else
                value = [];
            end
        end
    end
    
    methods (Static)
        function obj = loadobj(s)
            obj = s;
            % Loading a recording saved before R2020b and read once.
            if ~obj.pIsFirstRead && ~isfield(obj.pExistingFields,'PointClouds')
                obj.pExistingFields.PointClouds = false;
                obj.pExistingFields.PointCloudClusters = false;
            end
        end
    end
end