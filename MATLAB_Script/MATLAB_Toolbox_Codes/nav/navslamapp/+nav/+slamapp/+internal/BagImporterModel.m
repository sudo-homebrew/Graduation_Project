classdef BagImporterModel < handle & ...
        matlab.mixin.SetGet & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper & ...
        robotics.appscore.internal.mixin.SaveLoadTools
    %This class is for internal use only. It may be removed in the future.

    %BagImporterModel This class encapsulates the Model portion of the
    %   Model/View/Controller design pattern used in the SLAM app for
    %   importing sensor data from a rosbag

    % Copyright 2018-2020 The MathWorks, Inc.

    properties
        %Scans
        Scans

        %ScanTimes
        ScanTimes

        %Ts
        Ts

        %PosesXY
        PosesXY
    end

    properties
        %LidarRange
        LidarRange

        %LidarFrame
        LidarFrame
    end

    properties (Transient)
        %ScanFcn
        ScanFcn
    end

    properties
        %ScanFcnIndex Indicating which ScanFcn is currently used
        ScanFcnIndex
    end

    properties
        % bag selections

        %BagSelection
        BagSelection

        %ScanBagSelection
        ScanBagSelection

        %TfBagSelection
        TfBagSelection

    end

    properties

        % info from the bag

        %BagName
        BagName

        %AvailableScanTopics Laser scan topics
        AvailableScanTopics

        %AvailableOdomTopics TF topics
        AvailableOdomTopics

        %AvailableOdomOptions User-facing Odom options in the dropdown
        AvailableOdomOptions

        %NumScanMessages
        NumScanMessages

        %NumOdomMessages
        NumOdomMessages

        %ScanStartTime
        ScanStartTime

        %ScanEndTime
        ScanEndTime

        %ScanStartTimeOffset
        ScanStartTimeOffset

        %AvailableTFFrames
        AvailableTFFrames

        %AvailablePercentagesToKeep
        AvailablePercentagesToKeep


        %Applied Applied toolstrip settings
        Applied

    end

    properties (SetObservable)
        %Tentative Tentative toolstrip settings (not yet applied)
        Tentative

        %InfoStrings
        InfoStrings
    end


    properties (SetObservable, Transient)
        %TabRefreshTrigger
        TabRefreshTrigger = 1
    end


    events
    BagImporterModel_RequestUpdateBagImporterTab
    BagImporterModel_MaxLidarRangeEstimateAvailable
end

methods
    function obj = BagImporterModel()
    %BagImporterModel Constructor
        obj.AvailablePercentagesToKeep = {'100'; '50'; '33.3'; '25'; '20'; '12.5'; '10'; '5'; '1'};
        obj.MsgIDPrefix = 'nav:navslamapp:bagimportermodel';
    end

    function [isSuccess, errorTag, errorID, fileSize, startTime, endTime, availableTopics] = checkBag(obj, bagName)
    %checkBag
        isSuccess = true;
        errorTag = [];
        errorID = [];
        fileSize = -1;
        startTime = datetime.empty;
        endTime = datetime.empty;
        availableTopics = {};

        fprintf('%s\n', bagName);
        try
            bagSelect = ros.Bag.parse(bagName);
        catch ME
            fprintf('%s\n', ME.identifier);
            isSuccess = false;
            errorTag = 'ReadBag';
            errorID = ME.identifier;
            return;
        end

        obj.BagSelection = bagSelect;
        T = bagSelect.AvailableTopics;
        scanTopicTable = T(T.MessageType == 'sensor_msgs/LaserScan', :);
        odomTopicTable = T(T.MessageType == ros.internal.Settings.Tf2MessageType | T.MessageType == ros.internal.Settings.TfMessageType, :);

        obj.AvailableScanTopics = scanTopicTable.Properties.RowNames;
        obj.AvailableOdomTopics = odomTopicTable.Properties.RowNames;

        if isempty(obj.AvailableScanTopics)
            isSuccess = false;
            errorTag = 'NoScanTopic';
            return;
        end

        if ~isempty(obj.AvailableOdomTopics)
            obj.AvailableOdomOptions = {'N/A'; obj.retrieveMsg('UseTF')};
        else
            obj.AvailableOdomOptions = {'N/A'};
        end
        obj.NumOdomMessages = 0;

        % retrieving scans from the first available scan and odom topics
        obj.ScanBagSelection = bagSelect.select('Topic', obj.AvailableScanTopics{1});
        obj.NumScanMessages = obj.ScanBagSelection.NumMessages;
        obj.ScanStartTimeOffset = obj.ScanBagSelection.StartTime;
        obj.ScanStartTime = 0;
        obj.ScanEndTime = obj.ScanBagSelection.EndTime - obj.ScanStartTimeOffset;

        obj.Applied.ScanStartTime = obj.ScanStartTime;
        obj.Applied.ScanEndTime = obj.ScanEndTime;

        %
        sampleScanStructCell = obj.ScanBagSelection.readMessages(1, 'DataFormat','struct');
        sampleScanStruct = sampleScanStructCell{1};
        obj.LidarRange = [sampleScanStruct.RangeMin, sampleScanStruct.RangeMax];
        obj.LidarFrame = sampleScanStruct.Header.FrameId;
        if strcmp(obj.LidarFrame(1), '/')
            obj.LidarFrame = obj.LidarFrame(2:end);
        end

        % tf
        obj.AvailableTFFrames = bagSelect.AvailableFrames;

        obj.Applied.ScanTopicIndex = 1;
        obj.Applied.OdomOptionIndex = 1;

        obj.Applied.TargetFrameIndex = -1;
        obj.Applied.SourceFrameIndex = -1;
        obj.Applied.PercentageToKeepIndex = 1;

        obj.Tentative = obj.Applied;

        D = dir(bagName);
        fileSize = -1;
        if ~isempty(D)
            fileSize = (D.bytes)/(1024^2);
        end
        startTime = datetime(bagSelect.StartTime, 'ConvertFrom', 'posixtime', 'TimeZone', 'America/New_York');
        endTime = datetime(bagSelect.EndTime, 'ConvertFrom', 'posixtime', 'TimeZone', 'America/New_York');
        availableTopics = bagSelect.AvailableTopics.Row;

        obj.BagName = bagName;
    end

    function [scan, scanTime] = getCurrentRawScan(obj, scanNumber)
    %getCurrentRawScan

        scanStructs = obj.ScanBagSelection.readMessages(scanNumber, 'DataFormat','struct');
        scanStruct = scanStructs{1};
        angles = linspace(scanStruct.AngleMin, scanStruct.AngleMax, length(scanStruct.Ranges));
        scan = lidarScan(scanStruct.Ranges, angles);

        % accessing the message recording time in the MessageList table
        scanTime = obj.ScanBagSelection.MessageList{scanNumber,1} - obj.ScanStartTimeOffset;

    end

    function [scan, scanTime] = getCurrentRawScan2(obj, scanNumber)
    %getCurrentRawScan2

        scan = [];
        scanTime = [];
        if numel(obj.Scans) > 0
            scan = obj.Scans{scanNumber};
            scanTime = obj.ScanTimes{scanNumber};
        end
    end

    function syncTimeSelection(obj, type, value)
    %syncTimeSelection
        if ~isnan(value)
            if value < obj.ScanStartTime
                value = obj.ScanStartTime;
            end

            if value > obj.ScanEndTime
                value = obj.ScanEndTime;
            end

            if type == 0
                obj.Tentative.ScanStartTime = value;
                if value > obj.Tentative.ScanEndTime
                    obj.Tentative.ScanEndTime = value;
                end
            end

            if type == 1
                obj.Tentative.ScanEndTime = value;
                if value < obj.Tentative.ScanStartTime
                    obj.Tentative.ScanStartTime = value;
                end
            end
        end
    end


    function [isSuccess, errorTag] = extractData(obj)
    %extractData
        isSuccess = true;
        errorTag = [];

        appliedBackup = obj.Applied;

        needToUpdateTimeSlider = false;
        if obj.Tentative.ScanTopicIndex ~= obj.Applied.ScanTopicIndex
            needToUpdateTimeSlider = true;
        end
        obj.Applied = obj.Tentative;

        % trigger toolstrip refresh
        obj.Tentative.PercentageToKeepIndex = obj.Tentative.PercentageToKeepIndex;

        scanTopic = obj.AvailableScanTopics(obj.Applied.ScanTopicIndex);
        if needToUpdateTimeSlider
            obj.ScanBagSelection = obj.BagSelection.select('Topic', scanTopic);

            obj.ScanStartTimeOffset = obj.ScanBagSelection.StartTime;
            obj.ScanStartTime = 0;
            obj.ScanEndTime = obj.ScanBagSelection.EndTime - obj.ScanStartTimeOffset;

            obj.Applied.ScanStartTime = obj.ScanStartTime;
            obj.Applied.ScanEndTime = obj.ScanEndTime;

            obj.Tentative = obj.Applied;
        else
            obj.ScanBagSelection = obj.BagSelection.select('Topic', scanTopic, ...
                                                           'Time', obj.ScanStartTimeOffset + [obj.Applied.ScanStartTime, obj.Applied.ScanEndTime]);
        end

        percent = str2double(obj.AvailablePercentagesToKeep( obj.Applied.PercentageToKeepIndex));
        skip = round(1/percent*100);

        indices = 1:skip:obj.ScanBagSelection.NumMessages;
        if length(indices) < 2
            isSuccess = false;
            errorTag = 'TooFewScans';
            % recover original applied values
            obj.Applied = appliedBackup;
            obj.Tentative = obj.Applied;
            return
        end

        scanMsgStructs = readMessages(obj.ScanBagSelection, indices, 'DataFormat', 'struct' );

        scanTimesRaw = obj.ScanBagSelection.MessageList{1:skip:obj.ScanBagSelection.NumMessages,1}; % use bag storage time instead of scan message header time

        numReadings = numel(scanMsgStructs{1}.Ranges);
        angles = scanMsgStructs{1}.AngleMin + (0:numReadings-1) * scanMsgStructs{1}.AngleIncrement;


        hasTfTopic = (obj.Applied.OdomOptionIndex > 1);

        % if the bag has tf topic, but the tf source and target frames are not identified
        if hasTfTopic && ( (obj.Applied.TargetFrameIndex < 1) || (obj.Applied.SourceFrameIndex < 1 ))
            % notify user and request tab view reset
            isSuccess = false;
            errorTag = 'TFFramesUnidentified';
            obj.Applied = appliedBackup;
            obj.Tentative = obj.Applied;
            return;
        end

        % back up scan related data before sync with TF data
        scansBackup = obj.Scans;
        scanTimesBackup = obj.ScanTimes;
        numScanMessagesBackup = obj.NumScanMessages;

        % clear containers
        obj.Scans = {};
        obj.Ts = {};
        obj.ScanTimes = {};
        obj.PosesXY = {};


        if hasTfTopic

            if isempty(obj.TfBagSelection)
                obj.TfBagSelection = obj.BagSelection.select('Topic', obj.AvailableOdomTopics);
            end

            k = 1;
            lidarFrame = obj.AvailableTFFrames{obj.Applied.TargetFrameIndex};
            fixedFrame = obj.AvailableTFFrames{obj.Applied.SourceFrameIndex};

            for i = 1:numel(scanMsgStructs)
                scanTime = scanTimesRaw(i);
                if canTransform(obj.TfBagSelection, fixedFrame, lidarFrame, rostime(scanTime))

                    ranges = scanMsgStructs{i}.Ranges;
                    obj.Scans{k} = lidarScan(double(ranges), double(angles));
                    obj.ScanTimes{k} = scanTime - obj.ScanStartTimeOffset;

                    tformMsg = getTransform(obj.TfBagSelection, fixedFrame, lidarFrame, rostime(scanTime));
                    [obj.Ts{k}, ~, obj.PosesXY{k}] = tformFromStampedTransformMsg(tformMsg);
                    k = k + 1;
                end
            end
            obj.NumScanMessages = k-1;
            obj.NumOdomMessages = k-1;
        else % if no tf is available
            for i = 1:numel(scanMsgStructs)
                scanTime = scanTimesRaw(i);
                ranges = scanMsgStructs{i}.Ranges;
                obj.Scans{i} = lidarScan(double(ranges), double(angles));
                obj.ScanTimes{i} = scanTime - obj.ScanStartTimeOffset;
            end
            obj.NumScanMessages = i;
            obj.NumOdomMessages = 0;
        end

        % if the bag has tf topic, and the target and source frames
        % both are present, but no transform can be found between
        % target and source frame
        if obj.NumScanMessages < 2
            isSuccess = false;
            errorTag = 'TFNoTransform';
            obj.Applied = appliedBackup;
            obj.Tentative = obj.Applied;

            obj.Scans = scansBackup;
            obj.ScanTimes = scanTimesBackup;
            obj.NumScanMessages = numScanMessagesBackup;

            obj.Ts = {};
            obj.PosesXY = {};
            obj.NumOdomMessages = 0;

            return;
        end

        import robotics.appscore.internal.eventdata.VectorEventData
        obj.notify('BagImporterModel_MaxLidarRangeEstimateAvailable', VectorEventData(obj.LidarRange) );


    end


    function rootFrameIndex = estimateTfRootFrame(obj, tfOptionIndex)
    %estimateTfRootFrame

        if tfOptionIndex == 1
            rootFrameIndex = -1;
            return
        end

        obj.TfBagSelection = obj.BagSelection.select('Topic', obj.AvailableOdomTopics);

        % analyze first 100 tf messages
        numTfMsgs = min(100, obj.TfBagSelection.NumMessages);
        tfMsgs = obj.TfBagSelection.readMessages(1:numTfMsgs, 'DataFormat', 'struct');

        G1 = digraph;
        G2 = graph;

        for i = 1:length(tfMsgs)
            ti = tfMsgs{i};
            childFrameIds = {ti.Transforms.ChildFrameId};
            frameIds = cellfun(@(x) x.FrameId, {ti.Transforms.Header}, 'UniformOutput', false);
            for j = 1:length(childFrameIds)
                G1 = addedge(G1, childFrameIds{j}, frameIds{j});
                G2 = addedge(G2, childFrameIds{j}, frameIds{j});
            end
        end
        G1 = G1.simplify;
        G2 = G2.simplify;

        % if G1 is a directed acyclic graph and G2 is connected
        if isdag(G1) && isempty(find(conncomp(G2)-1, 1))
            order = G1.toposort;
            frames = G1.Nodes{:,1};
            rootFrameName = frames{order(end)};
            % in some tf messages, the frame names start with '/'
            if strcmp(rootFrameName(1), '/')
                rootFrameName = rootFrameName(2:end);
            end
            [~, rootFrameIndex] = ismember(rootFrameName, obj.AvailableTFFrames);
        else
            rootFrameIndex = -1;
        end
    end

    function clean(obj)
    %clean Return the BagImporterModel object to a pristine state
        obj.Scans = [];
        obj.ScanTimes = [];
        obj.Ts = [];
        obj.PosesXY = [];

        obj.LidarRange = [];
        obj.LidarFrame = [];

        obj.BagSelection = [];
        obj.AvailableScanTopics = [];
        obj.AvailableOdomTopics = [];
        obj.AvailableOdomOptions = [];
        obj.NumScanMessages = [];
        obj.NumOdomMessages = [];
        obj.ScanBagSelection =[];
        obj.ScanStartTime = [];
        obj.ScanEndTime = [];
        obj.ScanStartTimeOffset = [];
        obj.AvailableTFFrames = [];
        obj.Applied = [];

        obj.TfBagSelection = [];

        obj.ScanFcnIndex = 1;

    end

    function [infoStruct, extraInfoStruct] = saveProperties(obj)
    %saveProperties

    % basic properties
        infoStruct = saveProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj);

        % observable properties
        extraInfoStruct.Tentative = obj.Tentative;
        extraInfoStruct.InfoStrings = obj.InfoStrings;

    end

    function loadProperties(obj, infoStruct1, infoStruct2)
    %loadProperties
        loadProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj, infoStruct1);

        % load additional properties to update importer tab content
        if ~isempty(infoStruct2.Tentative) % not all app sessions go through import stage
            obj.Tentative = infoStruct2.Tentative;
            obj.TabRefreshTrigger = 1;
            obj.InfoStrings = infoStruct2.InfoStrings;
        end
    end

    function assignScanRetrievalFcn(obj)
    %assignScanRetrievalFcn
        if obj.ScanFcnIndex == 2
            obj.ScanFcn = @(scanNumber) obj.getCurrentRawScan2(scanNumber);
        else
            obj.ScanFcn = @(scanNumber) obj.getCurrentRawScan(scanNumber);
        end
    end

    function set.ScanFcnIndex(obj, val)
    %set.ScanFcnIndex
        obj.ScanFcnIndex = val;
        obj.assignScanRetrievalFcn();
    end

end
end

function [T, poseVec, poseVecSmashed] = tformFromStampedTransformMsg( stampedTransformObj )
%tformFromStampedTransformMsg
    x = stampedTransformObj.Transform.Rotation.X;
    y = stampedTransformObj.Transform.Rotation.Y;
    z = stampedTransformObj.Transform.Rotation.Z;
    w = stampedTransformObj.Transform.Rotation.W;

    tx = stampedTransformObj.Transform.Translation.X;
    ty = stampedTransformObj.Transform.Translation.Y;
    tz = stampedTransformObj.Transform.Translation.Z;

    quat = [w x y z];
    translate = [tx ty tz];

    T = trvec2tform(translate)*quat2tform(quat);
    poseVec = [translate quat ];

    ypr = quat2eul(quat);
    poseVecSmashed = [translate(1) translate(2) ypr(1)]; % projected to xy plane
end
