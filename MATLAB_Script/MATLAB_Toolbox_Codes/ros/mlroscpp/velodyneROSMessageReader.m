%velodyneROSMessageReader Read Velodyne ROS messages
%
%   VELOREADER = velodyneROSMessageReader(MSGS, DEVICEMODEL) constructs a
%   Velodyne ROS message reader object, VELOREADER, that can read in point
%   cloud data from VelodyneScan ROS messages. MSGS is a cell array of
%   VelodyneScan ROS message objects or structs. DEVICEMODEL is character
%   vector or a string specifying the model of the Velodyne Lidar used for
%   capturing the ROS messages. Possible values for DEVICEMODEL are
%   'VLP16', 'VLP32C', 'HDL32E', and 'HDL64E'.
%
%   VELOREADER = velodyneROSMessageReader(..., Name, Value) specifies
%   additional name-value pair arguments described below:
%
%   'CalibrationFile'  Name of an XML file containing Velodyne Lidar
%                      laser calibration data. If no calibration file is
%                      specified, a default calibration file with data
%                      obtained from Velodyne device manuals is chosen.
%
%                      Default: ''
%
%   velodyneROSMessageReader properties:
%      CurrentTime       - Time of current point cloud in seconds
%      VelodyneMessages  - Raw Velodyne ROS messages (read only)
%      DeviceModel       - Name of the device model (read only)
%      CalibrationFile   - Name of the Velodyne calibration XML file (read only)
%      NumberOfFrames    - Number of point cloud frames in the messages (read only)
%      Duration          - Total duration of the messages in seconds (read only)
%      StartTime         - Time of first point cloud in seconds (read only)
%      EndTime           - Time of last point cloud in seconds (read only)
%      Timestamps        - Timestamps for each point cloud frame (read only)
%
%   velodyneROSMessageReader methods:
%      readFrame    - Read point cloud frame from messages
%      hasFrame     - Determine if another point cloud is available
%      reset        - Reset to the beginning of the ROS messages
%
%   Notes
%   -----
%   - Providing an incorrect device model may result in improperly
%     calibrated point clouds.
%   - Not providing a calibration file can lead to inaccurate results.
%   - StartTime and EndTime are reported from top of the hour of messages
%     recording and are not absolute times. For instance, if the messages
%     are recorded for 7 minutes from 1:58 PM to 2:05 PM, then StartTime
%     would be 58*60 (i.e. 3480 seconds) and EndTime would be
%     StartTime + 7*60 (i.e. 3900 seconds).
%
%   Example: Extract and display velodyne ROS messages
%      % Open example and load velodyne messages.
%      openExample('ros/WorkWithVelodyneROSMessagesExample')
%      load('lidarData_ConstructionRoad.mat')
%
%      % Create velodyneROSMessageReader object.
%      veloReader = velodyneROSMessageReader(msgs,"HDL32E");
%
%      % Extract point clouds and access position of the points in point clouds.
%      timeDuration = veloReader.StartTime + seconds(1);
%      ptCloudObj = readFrame(veloReader,timeDuration);
%      ptCloudLoc = ptCloudObj.Location;
%
%      % Reset current time marker to beginning of data.
%      reset(veloReader)
%
%      % Create the point cloud player.
%      player = pcplayer([-60 60],[-60 60],[-20 20]);
%
%      % Begin reading data from a specific point in time.
%      veloReader.CurrentTime = veloReader.StartTime + seconds(0.3);
%
%      % Display stream of point clouds until point clouds are not available.
%      while(hasFrame(veloReader) && player.isOpen())
%          ptCloudObj = readFrame(veloReader);
%          view(player,ptCloudObj.Location,ptCloudObj.Intensity);
%          pause(0.1);
%      end
%
%   See also velodyneROSMessageReader/readFrame, velodyneROSMessageReader/hasFrame,
%            pcplayer, pointCloud.

% Copyright 2020-2021 The MathWorks, Inc.

classdef velodyneROSMessageReader < handle & matlab.mixin.SetGet ...
        & matlab.mixin.Copyable

    properties (GetAccess = 'public', SetAccess = 'private')
        %VelodyneMessages - Raw Velodyne ROS messages
        %   Raw Velodyne messages read directly from input VelodyneScan messages.
        VelodyneMessages

        %DeviceModel - Name of the device model
        DeviceModel

        %CalibrationFile - Name of the Velodyne calibration XML file
        %   Name of the XML file containing Velodyne laser
        %   lidar calibration data.
        CalibrationFile

        %NumberOfFrames    - Number of point cloud frames in the messages
        %   Total number of point clouds in the input messages.
        NumberOfFrames

        %Duration - Total duration of the messages in seconds
        %   Total length of the input messages in seconds represented
        %   as duration object.
        Duration

        %StartTime - Time of first point cloud in seconds
        %   Time of first point cloud in seconds,
        %   represented as duration object.
        StartTime

        %EndTime - Time of last point cloud in seconds
        %   Time of last point cloud in seconds,
        %   represented as duration object.
        EndTime

        %Timestamps - Timestamps for each point cloud frame
        %   Vector of duration objects representing the capture
        %   start times of all the point cloud frames
        Timestamps
    end

    properties (GetAccess = 'public', SetAccess = 'public', Dependent)

        %CurrentTime - Time of current point cloud in seconds
        %   Time of the current point cloud being read in
        %   seconds as duration object. The CurrentTime range is between
        %   StartTime and EndTime.
        CurrentTime
    end

    properties (Access = 'private')

        %LaserCalibrationData Laser calibration data loaded from XML file.
        LaserCalibrationData

        %UserCurrentTimeFlag Flag to determine if CurrentTime property
        %   is set by user.
        UserCurrentTimeFlag

        %CurrentTimeInternal Same as CurrentTime but used for internal
        %   purpose.
        CurrentTimeInternal

        %Version Version Number used for backward compatibility.
        Version = 1.0;
    end

    properties (Access = 'private', Transient)

        %VelodyneROSMessageReaderObj Internal object for reading Velodyne
        %   ROS Messages.
        VelodyneROSMessageReaderObj
    end

    methods
        % Get CurrentTime property value
        function value = get.CurrentTime(this)
            value = this.CurrentTimeInternal;
        end
        % Set CurrentTime property value
        function set.CurrentTime(this, value)
        % Check for datatype validity.
            validateattributes(value, {'duration'}, ...
                               {'nonempty', 'scalar', 'finite'}, 'set', 'CurrentTime');

            % Check for value finiteness and acceptable limits.
            if (value < this.StartTime || value > this.EndTime)
                error(message('ros:mlros:velodyne:invalidCurrentTime', ...
                              char(this.StartTime), char(this.EndTime)));
            end
            this.UserCurrentTimeFlag = true;
            this.CurrentTimeInternal = value;
        end
    end

    methods (Access = 'public')
        % Constructor
        function this = velodyneROSMessageReader(velodyneMessages, deviceModel, varargin)
            paramsStruct = iParseAndValidateInputs(velodyneMessages, deviceModel, varargin{:});

            this.VelodyneMessages       = paramsStruct.VelodyneMessages;
            this.DeviceModel            = paramsStruct.DeviceModel;
            this.CalibrationFile        = paramsStruct.CalibrationFile;

            % Setup calibration data
            [laserCalibrationData, distanceResolution] = iRetrieveCalibrationData( ...
                this.CalibrationFile, this.DeviceModel);

            this.LaserCalibrationData = laserCalibrationData;

            % Create the ROS message reader object
            this.VelodyneROSMessageReaderObj = roscpp.velodyne.internal.VelodyneROSMessageReader();

            openFcnArgs.LaserCalibrations               = this.LaserCalibrationData;
            openFcnArgs.DistanceResolution              = distanceResolution;
            openFcnArgs.SkipPartialFrames               = paramsStruct.SkipPartialFrames; % skipPartialFrames is for internal usage only.

            timeStruct = load(this.VelodyneROSMessageReaderObj, ...
                              this.VelodyneMessages, openFcnArgs, this.DeviceModel);

            % Fill class properties returned from mex call.
            if(~isempty(timeStruct))
                this.NumberOfFrames = timeStruct.NumberOfFrames;
                this.StartTime      = seconds(timeStruct.StartTime);
                this.EndTime        = seconds(timeStruct.EndTime);
                this.Duration       = seconds(timeStruct.Duration);
                this.Timestamps     = seconds(timeStruct.TimestampsVector);
            end

            this.CurrentTimeInternal = this.StartTime;
            this.UserCurrentTimeFlag = false;
        end


        function ptCloudObj = readFrame(this, varargin)
        %readFrame Read point cloud frame from messages
        %
        %   PTCLOUD = readFrame(VELOREADER) reads the next available
        %   point cloud in sequence. PTCLOUD is a pointCloud object
        %   with the Location specifying the XYZ coordinates of points
        %   (expressed in meters) and Intensity specifying the
        %   intensities of respective points. Both Location and Intensity
        %   are properties of the pointCloud object. The VELOREADER object
        %   keeps track of the last read point cloud for future calls
        %   to readFrame.
        %
        %   PTCLOUD = readFrame(VELOREADER, FRAMENUMBER) reads a point
        %   cloud with the specific frame number from the ROS messages. The
        %   numeric value, FRAMENUMBER, should be a valid positive
        %   number and not more than VELOREADER.NumberOfFrames.
        %
        %   PTCLOUD = readFrame(VELOREADER, FRAMETIME) reads the first
        %   point cloud recorded at or after the given FRAMETIME in
        %   seconds. FRAMETIME should be of duration type.
        %
        %   See also velodyneROSMessageReader, velodyneROSMessageReader/hasFrame,
        %             pcplayer, pointCloud.

            narginchk(1,2);

            if nargin>1

                if isduration(varargin{1})

                    durationToSeek = varargin{1};

                    % Validate frameTime
                    validateattributes(durationToSeek, {'duration'}, ...
                                       {'nonempty', 'scalar', 'finite'}, 'readFrame', ...
                                       'frameTime');

                    % Check for valid duration values.
                    if (durationToSeek < this.StartTime || durationToSeek > this.EndTime)
                        error(message( 'ros:mlros:velodyne:invalidTimeDuration', ...
                                       char(this.StartTime), char(this.EndTime)));
                    end

                    % Convert to double, and remove start time
                    durationToSeekSeconds = double(seconds(durationToSeek - this.StartTime));

                    % Call builtin function with duration in seconds.
                    [xyziPoints, intensity, currentTimestamp, rangeData] = readPointCloud(...
                        this.VelodyneROSMessageReaderObj, this.VelodyneMessages, durationToSeekSeconds);

                elseif isnumeric(varargin{1})

                    frameNum = varargin{1};

                    % Validate frameNumber
                    validateattributes(frameNum, {'numeric'}, ...
                                       {'nonempty', 'scalar', 'finite', 'integer', 'real', 'positive', '<=', this.NumberOfFrames}, ...
                                       'readFrame', 'frameNumber');

                    % Convert to int32 and zero-based indexing
                    frameNumber = int32(frameNum - 1);

                    % Call builtin function with frame number.
                    [xyziPoints, intensity, currentTimestamp, rangeData] = readPointCloud(...
                        this.VelodyneROSMessageReaderObj, this.VelodyneMessages, frameNumber);
                else
                    % The type of varargin{1} is invalid. Throw an error
                    validateattributes(varargin{1}, {'numeric', 'duration'}, ...
                                       {}, 'readFrame', 'frameNumber or frameTime');
                end

            else

                % Default builtin call if optional arguments not provided.
                if this.UserCurrentTimeFlag
                    % If CurrentTime is set by user, use it to retrieve
                    % point cloud.
                    durationToSeekSeconds = double(seconds(this.CurrentTimeInternal - this.StartTime));

                    [xyziPoints, intensity, currentTimestamp, rangeData] = readPointCloud(...
                        this.VelodyneROSMessageReaderObj, this.VelodyneMessages, durationToSeekSeconds);
                else
                    % If user does not provide CurrentTime, read next point
                    % cloud in sequence.
                    if hasFrame(this)
                        [xyziPoints, intensity, currentTimestamp, rangeData] = readPointCloud(...
                            this.VelodyneROSMessageReaderObj, this.VelodyneMessages, int32(-1));
                    else
                        error(message('ros:mlros:velodyne:endOfMessage'));
                    end
                end
            end

            if isempty(xyziPoints)
                % Return an empty pointCloud
                ptCloudObj = pointCloud(zeros(0, 0, 3, 'like', xyziPoints));

                % Reset the reader
                this.CurrentTimeInternal = this.StartTime + seconds(0);
            else
                % Create pointCloud object from xyziPoints, intensity and
                % rangData
                ptCloudObj  = makePointCloud(this, xyziPoints, intensity, rangeData);
                % Update current time
                this.CurrentTimeInternal = this.StartTime + seconds(currentTimestamp);
            end

            this.UserCurrentTimeFlag = false;
        end

        % Check if another point cloud is available to read
        function flag = hasFrame(this)
        %hasFrame Determine if another point cloud is available
        %
        %   FLAG = hasFrame(VELOREADER) returns TRUE if there is another
        %   point cloud available to read from the ROS messages. Otherwise,
        %   the function returns false.
        %
        %   See also  velodyneROSMessageReader, velodyneROSMessageReader/readFrame,
        %             pcplayer, pointCloud.

        % Check if timestamp of last frame requested is less than the
        % EndTime of the ROS messages.
        % Timestamps in the Velodyne packet are reported in
        % microseconds. If the difference between EndTime and
        % CurrentTimeInterval is less than 1 microsecond (i.e. 1e-6),
        % consider them to be close enough to report reaching last
        % frame, i.e, end of the messages reached and next frame
        % unavailable.

            flag = abs(seconds(this.EndTime) - seconds(this.CurrentTimeInternal)) >= 1e-6;
        end

        % Reset CurrentTime to StartTime
        function reset(this)
        %reset Reset to the beginning of the ROS messages
        %
        %   reset(VELOREADER) resets the status of the VELOREADER object
        %   to the beginning of the ROS messages.
        %
        %   See also  velodyneROSMessageReader, velodyneROSMessageReader/readFrame,
        %             pcplayer, pointCloud.

        % Set CurrentTime property to StartTime.
            this.CurrentTime = this.StartTime;
        end
    end

    methods (Hidden)
        function delete(this)
            close(this.VelodyneROSMessageReaderObj);
            % Invalidate class properties
            this.VelodyneMessages     = [];
            this.CalibrationFile      = [];
        end
    end

    methods(Access = 'protected')
        function copyObj = copyElement(this)
        % Override copyElement method
            if(~isempty(this.CalibrationFile))
                copyObj = velodyneROSMessageReader(this.VelodyneMessages, this.DeviceModel, 'CalibrationFile', this.CalibrationFile);
            else
                copyObj = velodyneROSMessageReader(this.VelodyneMessages, this.DeviceModel);
            end
            copyObj.CurrentTime = this.CurrentTime;
        end
    end

    methods(Hidden)
        function s = saveobj(this)
        % save properties into struct
            s.VelodyneMessages = this.VelodyneMessages;
            s.DeviceModel      = this.DeviceModel;
            s.CalibrationFile  = this.CalibrationFile;
            s.Version          = this.Version;
            s.CurrentTime      = this.CurrentTime;
        end
    end

    methods (Static, Hidden)
        function this = loadobj(s)
        % Load Object
            currentTime = s.CurrentTime;
            if(~isempty(s.CalibrationFile))
                this = velodyneROSMessageReader(s.VelodyneMessages, s.DeviceModel, 'CalibrationFile', s.CalibrationFile);
            else
                this = velodyneROSMessageReader(s.VelodyneMessages, s.DeviceModel);
            end
            this.CurrentTime = currentTime;
        end
    end
    
    methods (Static)
        function validDeviceModels = validDeviceModels()
            validDeviceModels = {'VLP16', 'VLP32C', 'HDL32E', 'HDL64E'};
        end
    end

    methods(Access = 'private')
        % Create pointCloud object
        function ptCloudObj = makePointCloud (this, xyziPoints, intensity, rangeData)
            persistent sortedVerticalAngleIndices;

            if(isempty(sortedVerticalAngleIndices) || length(sortedVerticalAngleIndices)~= size(this.LaserCalibrationData,1))
                % Laser vertical angles are stored in 2nd column of
                % LaserCalibrationData.
                laserVerticalAngles = this.LaserCalibrationData(:, 2);

                % Sort the vertical angles and obtain indices to sorted
                % vertical angles.
                [~, sortedVerticalAngleIndices] = sort(laserVerticalAngles, 'descend');
            end


            % Create pointCloud object from xyziPoints, with points
            % sorted according to the laser vertical angles.
            ptCloudObj  = pointCloud(xyziPoints(sortedVerticalAngleIndices, :, :), ...
                                     'Intensity', intensity(sortedVerticalAngleIndices, :));

            % range, pitch, yaw
            ptCloudObj.RangeData = rangeData(sortedVerticalAngleIndices, :, :);
        end
    end
end

% Input parsing and validation
function paramsStruct = iParseAndValidateInputs(velodyneMessages, deviceModel, varargin)

% Validate velodyneMessages
    validateattributes(velodyneMessages,{'cell','struct','ros.msggen.velodyne_msgs.VelodyneScan'},{'nonempty'},'velodyneROSMessageReader','velodyneMessages');

    % Validate device model
    deviceModel = iValidateDeviceModel(deviceModel);

    % Parse name-value inputs
    p = inputParser;
    p.FunctionName = 'velodyneROSMessageReader';

    defaultCalibrationFile = iGetDefaultCalibrationFile(deviceModel);
    addParameter(p, 'CalibrationFile', defaultCalibrationFile);
    addParameter(p, 'SkipPartialFrames', true);    % Note that this parameter is undocumented

    parse(p, varargin{:});

    paramsStruct = p.Results;

    % Check that calibration file exists
    paramsStruct.CalibrationFile   = iValidateCalibrationFile(paramsStruct.CalibrationFile);
    paramsStruct.SkipPartialFrames = iValidateSkipPartialFrames(paramsStruct.SkipPartialFrames);
    paramsStruct.DeviceModel    = deviceModel;
    % Covert VelodyneMessages to cell array with all structs for each cell
    if isa(velodyneMessages, 'ros.msggen.velodyne_msgs.VelodyneScan')
        paramsStruct.VelodyneMessages = arrayfun(@(x) {toStruct(x)}, velodyneMessages);
    elseif isstruct(velodyneMessages)
        paramsStruct.VelodyneMessages = arrayfun(@(x) {x}, velodyneMessages);
    elseif isa(velodyneMessages{1,1}, 'ros.msggen.velodyne_msgs.VelodyneScan')
        paramsStruct.VelodyneMessages = cellfun(@(x) {toStruct(x)}, velodyneMessages);
    else
        paramsStruct.VelodyneMessages = velodyneMessages;
    end

    % Check that one element in VelodyneMessages contains Packets field,
    % and that there is a Data field in Packets to make sure the input
    % message is valid.
    if (~isfield(paramsStruct.VelodyneMessages{1,1},'Packets'))
        error(message('ros:mlros:velodyne:invalidArgTypeForROSMessage'));
    end
    if (isempty(paramsStruct.VelodyneMessages{1,1}.Packets))
        error(message('ros:mlros:velodyne:emptyROSMessage'));
    end
    if (~isfield(paramsStruct.VelodyneMessages{1,1}.Packets,'Data'))
        error(message('ros:mlros:velodyne:invalidArgTypeForROSMessage'));
    end
end

function deviceModel = iValidateDeviceModel(deviceModel)

    validDeviceModels = velodyneROSMessageReader.validDeviceModels;
    deviceModel = validatestring(deviceModel, validDeviceModels, ...
                                 'velodyneROSMessageReader', 'deviceModel');
end

function defaultCalibrationFile = iGetDefaultCalibrationFile(deviceModel)

    defaultCalibrationFile = fullfile(matlabroot, 'toolbox', 'shared', ...
                                      'pointclouds', 'utilities', 'velodyneFileReaderConfiguration', ...
                                      [char(deviceModel) '.xml']);
end

function calibrationFile = iValidateCalibrationFile(calibrationFile)

    validateattributes(calibrationFile, {'char', 'string'}, ...
                       {'nonempty', 'scalartext'}, 'velodyneROSMessageReader', 'CalibrationFile');

    calibrationFile = iCheckFile(calibrationFile, '.xml', 'Calibration XML');
end

function skipPartialFrames = iValidateSkipPartialFrames(skipPartialFrames)

    validateattributes(skipPartialFrames, {'logical', 'numeric'}, {'binary'}, ...
                       'velodyneROSMessageReader', 'SkipPartialFrames');
end

function [laserCalibrationData, distanceResolution] = iRetrieveCalibrationData(calibrationFile, deviceModel)

% Read Calibration data from XML file.
    [laserCalibrationData, distanceResolution]  = iGetVelodyneCorrectionsFromXML(calibrationFile);

    % Number of lasers based on Velodyne device model.
    if strcmpi('HDL32E', deviceModel) || strcmpi('VLP32C', deviceModel)
        numLasersOfDeviceModel = 32;
    elseif strcmpi('HDL64E', deviceModel)
        numLasersOfDeviceModel = 64;
    elseif strcmpi('VLS128', deviceModel)
        numLasersOfDeviceModel = 128;
    else
        numLasersOfDeviceModel = 16;
    end

    % Validate laser count in calibration data with number of
    % lasers for given device model.
    if size(laserCalibrationData, 1) ~= numLasersOfDeviceModel
        error(message(...
            'ros:mlros:velodyne:invalidCalibrationFileIncorrectEnabledLaserCountForDeviceModel', ...
            deviceModel, numLasersOfDeviceModel));
    end
end

function fileName = iCheckFile(fileName, expectedExtension, fileType)

% Check the file extension
    [~,~,fileExtension] = fileparts(fileName);

    if ~strcmpi(fileExtension, expectedExtension)
        error(message('ros:mlros:velodyne:invalidFileType', fileType, ...
                      expectedExtension, fileExtension));
    end

    % Check if the file exists
    [fid, msg] = fopen(fileName, 'r');

    if fid == -1
        error(message('ros:mlros:velodyne:fileOpenFailed', fileType, ...
                      fileName, msg));
    else
        % Get full path name of file
        fileName = fopen(fid);
        fclose(fid);
    end
end

% helper function to load laser calibration data from XML file
function [laserCorrections, distLSB] = iGetVelodyneCorrectionsFromXML(xmlFile)

    % Parse the provided Velodyne laser calibration XML file.
    try
        xmlParser = matlab.io.xml.dom.Parser;
        xmlParser.Configuration.AllowDoctype=true;
        domNode = xmlParser.parseFile(xmlFile);
    catch ex
        newEx = MException(message('ros:mlros:velodyne:calibrationFileReadError'));
        throw(newEx.addCause(ex))
    end
    % Get distLSB_ value
    distLSB = single(0.2);
    distLSBNode = domNode.getElementsByTagName('distLSB_');
    isDistLSBTagFound = distLSBNode.getLength() > 0;
    if(isDistLSBTagFound)
        distLSB = single(iGetTagValue(domNode,'distLSB_'));
    end

    % Count number of lasers enabled
    enabled = domNode.getElementsByTagName('enabled_');
    isEnabledTagMissing = enabled.getLength() <= 0;
    if(isEnabledTagMissing)
        error(message('ros:mlros:velodyne:invalidCalibrationFileTagNotFound', ...
                      'enabled_'));
    end
    enabledCount = 0;
    for k = 0 : enabled.getLength-1
        enabledItem   = enabled.item(k);
        enabledItemId = enabledItem.getElementsByTagName('item');
        for i= 0 : enabledItemId.getLength()-1
            if(str2double(enabledItemId.item(i).getTextContent) > 0)
                enabledCount = enabledCount+1;
            end
        end
    end

    isValidLaserCount = (enabledCount == 16 || enabledCount == 32 || enabledCount == 64 || enabledCount == 128);
    if(enabledCount <= 0 || ~isValidLaserCount)
        error(message('ros:mlros:velodyne:invalidCalibrationFileIncorrectEnabledLaserCount'));
    end
    % Read laser corrections
    px = domNode.getElementsByTagName('px');

    isPxTagMissing = (px.getLength() <= 0);
    if(isPxTagMissing)
        error(message('ros:mlros:velodyne:invalidCalibrationFileTagNotFound', 'px'));
    end
    isPxTagCountLessThanEnabledCount = (px.getLength() < enabledCount);
    if(isPxTagCountLessThanEnabledCount)
        error(message('ros:mlros:velodyne:invalidCalibrationFileMissingCalibrationValues'));
    end
    laserCorrections = zeros(enabledCount, 9, 'single');
    % Read the following tag values into laserCorrections array.
    %   <rotCorrection_>-5.3328056</rotCorrection_>
    %   <vertCorrection_>-7.2988362</vertCorrection_>
    %   <distCorrection_>111</distCorrection_>
    %   <distCorrectionX_>118</distCorrectionX_>
    %   <distCorrectionY_>118</distCorrectionY_>
    %   <vertOffsetCorrection_>19.736338</vertOffsetCorrection_>
    %   <horizOffsetCorrection_>2.5999999</horizOffsetCorrection_>
    %   <focalDistance_>0</focalDistance_>
    %   <focalSlope_>0</focalSlope_>

    for k = 0 : enabledCount-1
        pxItem = px.item(k);
        if(pxItem.getLength() <= 0)
            error(message('ros:mlros:velodyne:invalidCalibrationFileTagNotFound', 'px'));
        end
        laserCorrections(k+1, 1) = iGetTagValue(pxItem, 'rotCorrection_');
        laserCorrections(k+1, 2) = iGetTagValue(pxItem, 'vertCorrection_');
        laserCorrections(k+1, 3) = iGetTagValue(pxItem, 'distCorrection_');
        laserCorrections(k+1, 4) = iGetTagValue(pxItem, 'distCorrectionX_');
        laserCorrections(k+1, 5) = iGetTagValue(pxItem, 'distCorrectionY_');
        laserCorrections(k+1, 6) = iGetTagValue(pxItem, 'vertOffsetCorrection_');
        laserCorrections(k+1, 7) = iGetTagValue(pxItem, 'horizOffsetCorrection_');
        laserCorrections(k+1, 8) = iGetTagValue(pxItem, 'focalDistance_');
        laserCorrections(k+1, 9) = iGetTagValue(pxItem, 'focalSlope_');
    end
end

% helper function to parse a numeric tag value for a given XML tag
function tagValue = iGetTagValue(pxItem, tagName)
% Utility function to parse, validate and return tag
% value for a given tag in XML file.
    pxItemId = pxItem.getElementsByTagName(tagName);
    if(pxItemId.getLength() > 0)
        tagValue = str2double(pxItemId.item(0).getTextContent);
        if(isnan(tagValue))
            error(message('ros:mlros:velodyne:invalidCalibrationFileNonnumericTagValue', tagName));
        end
    else
        error(message('ros:mlros:velodyne:invalidCalibrationFileTagNotFound', tagName));
    end

end
