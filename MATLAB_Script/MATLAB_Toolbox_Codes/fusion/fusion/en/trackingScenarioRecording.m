classdef trackingScenarioRecording< handle
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

    methods
        function out=trackingScenarioRecording
            % Support name-value pair arguments when constructing object
        end

        function out=containsRecordedDataFormat(~) %#ok<STOUT>
        end

        function out=isDone(~) %#ok<STOUT>
            %isDone  Return true when the read reaches the recoding end
            % isDone(obj) returns true if end of data has been reached.
            % Use isDone to check if the end of the recording has been
            % reached before reading the next step in the recording.
        end

        function out=loadobj(~) %#ok<STOUT>
        end

        function out=read(~) %#ok<STOUT>
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
        end

        function out=readOptionalField(~) %#ok<STOUT>
        end

        function out=reset(~) %#ok<STOUT>
            %RESET  Reset the read to the beginning of the recording
            % RESET(obj) resets the read to the beginning of the recording
        end

        function out=setup(~) %#ok<STOUT>
            % Find the struct that is actually a RecordedData file we can read
        end

        function out=setupRecordedData(~) %#ok<STOUT>
            % Look for the data expected in the RecordedData
        end

    end
    properties
        % CurrentStep  Step index of the latest read data
        % You may specify the CurrentStep. In the next call to read, the
        % method resumes reading from the first recorded data step.
        %
        % Default: 0
        CurrentStep;

        % CurrentTime  Timestamp of the latest read data 
        % You may specify the CurrentTime. In the next call to read, the
        % method resumes reading from the first recorded data step that has
        % SimulationTime > CurrentTime
        %
        % Default: 0
        CurrentTime;

        %RecordedData    The recorded data stored in the recording
        % Specify the recorded data struct. This property must be set on
        % construction. See trackingScenario/record to see which fields are
        % expected.
        RecordedData;

    end
end
