classdef MapBuilderModel < handle & ...
        nav.algs.internal.InternalAccess & ...
        robotics.appscore.internal.mixin.SaveLoadTools
    %This class is for internal use only. It may be removed in the future.

    %MAPBUILDERMODEL Model to build a map with processed scan/odom data

    % Copyright 2018-2020 The MathWorks, Inc.

    properties
        %LidarSLAMProcessor
        LidarSLAMProcessor
    end

    properties
        %Scans
        Scans

        %NumScans
        NumScans

        %HasInitGuess
        HasInitGuess

        %RelativePoseEstimates
        RelativePoseEstimates

        %FrontScanIdx The user-facing index of the scan at the processing front
        FrontScanIdx

        %InboundMap From User-facing scan index to internal scan index
        InboundMap

        %OutboundMap From internal scan index to User-facing scan index
        OutboundMap

        %History Map building history results, indexed by scan IDs
        History

        %LCScanIdPairSelected Current loop closure scan pair showing
        LCScanIdPairSelected

        %ScanIdPairToShow  Next loop closure scan pair to show
        ScanIdPairToShow

        %AutoScanMatchingResults A containers.Map object
        AutoScanMatchingResults
    end

    properties % manual modification backlog

        %ModifiedEdges Saved as user facing scan ID pairs
        ModifiedEdges

        %ModifiedEdgeRelPoses Have the same number rows as ModifiedEdges
        ModifiedEdgeRelPoses

        %IgnoredEdges Saved as user facing scan ID pairs
        IgnoredEdges
    end

    properties % variables used by sync
               %ModifiedLCScanPairs
        ModifiedLCScanPairs

        %ModifiedLCRelPoses
        ModifiedLCRelPoses

        %IgnoredLCSanPairs
        IgnoredLCScanPairs

        %TrajScanPairs
        TrajScanPairs

        %TrajRelPoses
        TrajRelPoses
    end

    properties (SetObservable)
        %IsRunning Whether map building is in process
        IsRunning

        %IsPoseGraphDirty
        IsPoseGraphDirty

    end

    properties (Transient, SetObservable)

        %IsAppDirty Whether app state is changed w.r.t. saved session
        IsAppDirty
    end


    events
    MapBuilderModel_RequestRefreshBadge

end

methods
    function obj = MapBuilderModel()
    %MAPBUILDERMODEL Constructor
        obj.IsRunning = 0;
        obj.IsPoseGraphDirty = false;
        obj.AutoScanMatchingResults = containers.Map;
        obj.IsAppDirty = false;
    end

    function loadProcessedData(obj, scans, poses)
    %loadProcessedData
        obj.FrontScanIdx = 1;
        % Input scans are expected to be either Nx1 cell array or 1xN cell
        % array. But the stored scans are always expected to be 1xN by
        % match scans grid submap mexed function. So converting the input
        % scans to 1xN cell array format before storing.
        obj.Scans = scans(:)';

        if isempty(poses)
            obj.RelativePoseEstimates = [];
            obj.HasInitGuess = false;
        else
            % convert from odom data to relative poses
            relPoses = zeros(length(poses), 3);
            for i = 2:numel(poses)
                T0 = robotics.core.internal.SEHelpers.poseToTformSE2(poses{i-1});
                T1 = robotics.core.internal.SEHelpers.poseToTformSE2(poses{i});
                dT = robotics.core.internal.SEHelpers.tforminvSE2(T0)*T1;
                relPoses(i,:) = robotics.core.internal.SEHelpers.tformToPoseSE2(dT);
            end
            obj.RelativePoseEstimates = relPoses;
            obj.HasInitGuess = true;

        end
        obj.NumScans = length(scans);
    end



    function initialize(obj, settings)
    %initialize
        obj.History = {};
        obj.LCScanIdPairSelected = [];
        obj.ScanIdPairToShow = [];

        obj.LidarSLAMProcessor = lidarSLAM(settings.MapResolution, settings.LidarRange(2));
        obj.LidarSLAMProcessor.LoopClosureThreshold = settings.LoopClosureThreshold;
        obj.LidarSLAMProcessor.LoopClosureSearchRadius = settings.LoopClosureSearchRadius;
        obj.LidarSLAMProcessor.LoopClosureMaxAttempts = settings.LoopClosureMaxAttempts;
        obj.LidarSLAMProcessor.LoopClosureAutoRollback = settings.LoopClosureAutoRollback;
        obj.LidarSLAMProcessor.OptimizationInterval = settings.OptimizationInterval;
        obj.LidarSLAMProcessor.MovementThreshold = settings.MovementThreshold;

        obj.LidarSLAMProcessor.OptimizationFcn =  @(pg) optimizePoseGraph(pg, 'MaxIterations', settings.MaxIterations, ...
                                                          'MaxTime', settings.MaxTime, ...
                                                          'GradientTolerance', settings.GradientTolerance, ...
                                                          'FunctionTolerance', settings.FunctionTolerance, ...
                                                          'StepTolerance', settings.StepTolerance, ...
                                                          'FirstNodePose', settings.FirstNodePose);
        obj.trimScans(settings.LidarRange);
    end


    function toggleRunPause(obj)
    %toggleRunPause
        if obj.IsRunning
            obj.IsRunning = 0;
        else
            obj.IsRunning = 1;
        end
    end

    function trimScans(obj, rangeLimits)
    %trimScans Remove lidar ranges outside valid rangeLimits
    %   rangeLimits are specified through settings dialog
        for i = 1:obj.NumScans
            obj.Scans{i} = obj.Scans{i}.removeInvalidData('RangeLimits', rangeLimits);
        end
    end



    function [scanIDs, poses, lcEdges, isPoseGraphReoptimized] = step(obj)
    %step
        if obj.HasInitGuess

            if numel(obj.InboundMap) > 0
                prevScanIdx = obj.OutboundMap(end); % last updated user-facing scan Id
                relPose = obj.synthesizeRelativePose(prevScanIdx, obj.FrontScanIdx);
            else
                relPose = obj.RelativePoseEstimates(1,:); % doesn't matter
            end

            [isScanAccepted, loopClosureInfo, optimizationInfo] = obj.LidarSLAMProcessor.addScan(obj.Scans{obj.FrontScanIdx}, relPose); %
        else
            [isScanAccepted, loopClosureInfo, optimizationInfo] = obj.LidarSLAMProcessor.addScan(obj.Scans{obj.FrontScanIdx});
        end

        obj.InboundMap(obj.FrontScanIdx) = obj.LidarSLAMProcessor.PoseGraph.NumNodes;
        if isScanAccepted
            obj.OutboundMap(obj.LidarSLAMProcessor.PoseGraph.NumNodes) = obj.FrontScanIdx;
        end


        if isScanAccepted && obj.LidarSLAMProcessor.PoseGraph.NumEdges > 0
            scanIdPair = [obj.FrontScanIdx-1, obj.FrontScanIdx];
            trajEdge = [obj.InboundMap(scanIdPair(1)), obj.InboundMap(scanIdPair(2))];
            trajEdgeId = obj.LidarSLAMProcessor.PoseGraph.findEdgeID(trajEdge);

            if ~obj.AutoScanMatchingResults.isKey(num2str(scanIdPair)) % should only be filled once
                obj.AutoScanMatchingResults(num2str(scanIdPair)) = obj.LidarSLAMProcessor.PoseGraph.edgeConstraints(trajEdgeId);
            end

            if ~isempty(loopClosureInfo.EdgeIDs) && optimizationInfo.IsAccepted
                lcEdgeIds = loopClosureInfo.EdgeIDs;
                lcEdges = obj.LidarSLAMProcessor.PoseGraph.edges(lcEdgeIds);
                for j = 1:length(lcEdgeIds)
                    scanIdPair = [obj.OutboundMap(lcEdges(j,1)), obj.OutboundMap(lcEdges(j,2))];
                    obj.AutoScanMatchingResults(num2str(scanIdPair)) = obj.LidarSLAMProcessor.PoseGraph.edgeConstraints(lcEdgeIds(j));
                end
            end
        end

        scanIDs = obj.OutboundMap;

        poses = obj.LidarSLAMProcessor.PoseGraph.nodes;
        lcEdges = [];
        if ~isempty(obj.LidarSLAMProcessor.PoseGraph.LoopClosureEdgeIDs)
            lcEdges = obj.LidarSLAMProcessor.PoseGraph.edges(obj.LidarSLAMProcessor.PoseGraph.LoopClosureEdgeIDs);
        end
        s.ScanIDs = scanIDs;
        s.Poses = poses;
        s.LcEdges = lcEdges;
        s.IsScanAccepted = isScanAccepted;
        obj.History{obj.FrontScanIdx} = s;


        obj.FrontScanIdx = obj.FrontScanIdx + 1;
        isPoseGraphReoptimized = optimizationInfo.IsPerformed;

    end

    function [clearCanvas, refScan, currScan, relPose, userFacingScanIdPair] = retrieveIncrementalScanPairAndConstraint(obj, userFacingScanId)
    %retrieveIncrementalScanPairAndConstraint
        histData = obj.History{userFacingScanId};

        clearCanvas = false;
        relPose = [];
        refScan = [];
        currScan = [];
        userFacingScanIdPair = [];

        if histData.IsScanAccepted % alternatively, we can check (obj.OutboundMap(v) == userFacingScanId)
            v = obj.InboundMap(userFacingScanId);
            if v < 2
                clearCanvas = true;
                return;
            end
            edgeId = obj.LidarSLAMProcessor.PoseGraph.findEdgeID([v-1,v]);
            relPose = obj.LidarSLAMProcessor.PoseGraph.edgeConstraints(edgeId);
            refScan = obj.LidarSLAMProcessor.Scans{v-1};
            currScan = obj.LidarSLAMProcessor.Scans{v};
            userFacingScanIdPair = obj.OutboundMap(v-1:v);
        else
            clearCanvas = true;
        end

    end

    function [noLC, userFacingLCScanIDPairs] = getAvailableLoopClosureEdgesAssociatedWith(obj, userFacingScanId)
    %getAvailableLoopClosureEdgesAssociatedWith
        histData = obj.History{userFacingScanId};
        userFacingLCScanIDPairs  = [];
        noLC = false;
        if histData.IsScanAccepted % alternatively, we can check (obj.OutboundMap(v) == userFacingScanId)
            v = obj.InboundMap(userFacingScanId);
            if v < 2 % first node
                noLC = true;
                return;
            end
            edgeNodePairs = obj.LidarSLAMProcessor.PoseGraph.findEdgeNodePairs(v);
            if size(edgeNodePairs,1) == 1 % tip
                noLC = true;
                return;
            end

            % loop closures up until the current scan
            edgeNodePairs = edgeNodePairs(edgeNodePairs(:,1) ~= v-1, :);
            lcEdges = edgeNodePairs(edgeNodePairs(:,2) == v, :);

            if ~isempty(lcEdges)
                for i = 1:size(lcEdges,1)
                    userFacingLCScanIDPairs(i, :) = obj.OutboundMap(lcEdges(i,:)); %#ok<AGROW>
                end
            else
                noLC = true;
            end
        else
            noLC = true;
        end
    end

    function [refScan, currScan, relPose] = getLoopClosureScanPairAndRelPose(obj, userFacingScanIdPair)
    %getLoopClosureScanPairAndRelPose

        lcEdge = obj.InboundMap(userFacingScanIdPair);

        lcEdgeId = obj.LidarSLAMProcessor.PoseGraph.findEdgeID(lcEdge);

        relPose = obj.LidarSLAMProcessor.PoseGraph.edgeConstraints(lcEdgeId);
        refScan = obj.LidarSLAMProcessor.Scans{lcEdge(1)};
        currScan = obj.LidarSLAMProcessor.Scans{lcEdge(2)};

    end


    function updateIgnoredEdgeBacklog(obj, scanIdPair)
    %updateIgnoredEdgeBacklog
        if ~isempty(obj.IgnoredEdges)
            [found, loc] = ismember(scanIdPair, obj.IgnoredEdges, 'rows');
        else
            found = false;
        end

        if found
            obj.IgnoredEdges(loc,:) = scanIdPair;
        else
            obj.IgnoredEdges = [obj.IgnoredEdges; scanIdPair];
        end
        obj.IsPoseGraphDirty = true;

        edge = obj.InboundMap(scanIdPair);
        
        edgeId = obj.LidarSLAMProcessor.PoseGraph.findEdgeID(edge);
        obj.LidarSLAMProcessor.PoseGraph.updateEdgeConstraint(edgeId(1), [], zeros(1,6));
    end

    function updateManualModificationBacklog(obj, scanIdPair, relPose)
    %updateManualModificationBacklog
        if ~isempty(obj.ModifiedEdges)
            [found, loc] = ismember(scanIdPair, obj.ModifiedEdges, 'rows');
        else
            found = false;
        end

        if found
            obj.ModifiedEdges(loc,:) = scanIdPair;
            obj.ModifiedEdgeRelPoses(loc,:) = relPose;
        else
            obj.ModifiedEdges = [obj.ModifiedEdges; scanIdPair];
            obj.ModifiedEdgeRelPoses = [obj.ModifiedEdgeRelPoses; relPose];
        end
        obj.IsPoseGraphDirty = true;

        edge = obj.InboundMap(scanIdPair);
        assert(edge(1) < edge(2)); % avoid inverting the constraint
        edgeId = obj.LidarSLAMProcessor.PoseGraph.findEdgeID(edge);
        obj.LidarSLAMProcessor.PoseGraph.updateEdgeConstraint(edgeId(1), relPose, [1 0 0 1 0 1]);

        % if edge is a trajectory edge, also needs to update the submap
        lslam = obj.LidarSLAMProcessor;
        if edge(2) - edge(1) == 1
            numScansPerSubmap = lslam.NumScansPerSubmap;
            submapId = ceil(edge(2)/numScansPerSubmap);

            if submapId <= length(lslam.Submaps) % there is a chance the submap is not yet added
                maxRange = lslam.Submaps{submapId}.MaxRange;
                maxLevel = lslam.Submaps{submapId}.MaxLevel;
                resolution = lslam.Submaps{submapId}.Resolution;
                anchorIdx = round(numScansPerSubmap/2);

                indices = (submapId-1)*numScansPerSubmap+1:(submapId)*numScansPerSubmap;
                poses = lslam.PoseGraph.nodes(indices);

                lslam.Submaps{submapId} = nav.algs.internal.createSubmap(obj.Scans, indices, poses, anchorIdx, resolution, maxRange, maxLevel);
            end
        end

        import robotics.appscore.internal.eventdata.VectorEventData
        obj.notify('MapBuilderModel_RequestRefreshBadge', VectorEventData(max(scanIdPair)) );
    end


    function reoptimizePoseGraph(obj, syncStartId)
    %reoptimizePoseGraph

        optFcn = obj.LidarSLAMProcessor.OptimizationFcn;
        % update the initial guess to only consider trajectory edges
        [poseGraphUpdated, stats] = optFcn(obj.LidarSLAMProcessor.PoseGraph);

        obj.LidarSLAMProcessor.PoseGraph = poseGraphUpdated;

        % reset residual error history
        obj.LidarSLAMProcessor.ResidualErrorHistory = stats.ResidualError*ones(size(obj.LidarSLAMProcessor.ResidualErrorHistory));

        % reset map building history up to front
        for i = syncStartId:obj.FrontScanIdx-1
            idx = obj.InboundMap(i);
            poses = obj.LidarSLAMProcessor.PoseGraph.nodes(1:idx);
            obj.History{i}.Poses = poses;
        end
        % update the submap centers
        for j = 1:numel(obj.LidarSLAMProcessor.Submaps)
            nid = obj.LidarSLAMProcessor.Submaps{j}.AnchorScanIndex;
            p = obj.LidarSLAMProcessor.PoseGraph.nodes(nid);
            obj.LidarSLAMProcessor.Submaps{j}.Center = [p(1) p(2)];
        end

        obj.IsPoseGraphDirty = false;

    end

    function relPose = synthesizeRelativePose(obj, prevScanId, currScanId)
    %synthesizeRelativePose
        relPoses = obj.RelativePoseEstimates(prevScanId:currScanId,:);
        p0 = [0 0 0];
        for j = 2:size(relPoses,1)
            p0 = robotics.core.internal.SEHelpers.accumulatePoseSE2(p0, relPoses(j,:));
        end
        relPose = p0;
    end

    function initSync(obj, syncStartVal)
    %initSync

    % extract all the trajectory edges as user-facing scan pairs,
    % as they can be reused
        pg = obj.LidarSLAMProcessor.PoseGraph;
        obj.TrajScanPairs = [];
        obj.TrajRelPoses = [];
        for i = 2:pg.NumNodes
            trajEdgeId = pg.findEdgeID([i-1, i]);
            trajRelPose = pg.edgeConstraints(trajEdgeId);
            obj.TrajScanPairs = [obj.TrajScanPairs; [obj.OutboundMap(i-1), obj.OutboundMap(i)]];
            obj.TrajRelPoses = [obj.TrajRelPoses; trajRelPose];
        end

        modScanPairs = obj.ModifiedEdges;
        modRelPoses = obj.ModifiedEdgeRelPoses;
        obj.IgnoredLCScanPairs = obj.IgnoredEdges;

        mask = true(size(modScanPairs,1), 1);
        for i = 1:size(modScanPairs, 1)
            if obj.InboundMap(modScanPairs(i, 2)) - obj.InboundMap(modScanPairs(i, 1)) == 1 % i.e. a traj edge
                mask(i) = false;
            end
        end

        obj.ModifiedLCScanPairs = modScanPairs(mask, :);
        obj.ModifiedLCRelPoses = modRelPoses(mask, :);

        % partially reset lidar SLAM object
        obj.LidarSLAMProcessor.clearSLAMResultsAfter(obj.InboundMap(syncStartVal));

        % also need to revert the pose graph state to the one right
        % before the syncStartVal
        obj.LidarSLAMProcessor.PoseGraph.updateNodeEstimates(obj.History{syncStartVal-1}.Poses);

        % partially reset inbound/outbound map and history structs
        obj.OutboundMap = obj.OutboundMap(1:obj.InboundMap(syncStartVal-1));
        obj.InboundMap = obj.InboundMap(1:syncStartVal-1);
        obj.History = obj.History(1:syncStartVal-1);

    end


    function findCertifiedEdgesIfAvailable(obj, userFacingScanId)
    %findCertifiedEdgesIfAvailable

        lslam = obj.LidarSLAMProcessor;
        % check if the trajectory edge related to current scan has
        % been matched already
        prevAcceptedScanId = obj.OutboundMap(lslam.PoseGraph.NumNodes);
        if isempty(obj.TrajScanPairs)
            trajFound = false;
        else
            [trajFound, loc] = ismember([prevAcceptedScanId, userFacingScanId], obj.TrajScanPairs, 'rows');
        end

        if trajFound
            lslam.CertifiedTrajEdgePose = obj.TrajRelPoses(loc, :);
            lslam.CertifiedTrajEdgeInfoMat = [1 0 0 1 0 1];
        else
            lslam.CertifiedTrajEdgePose = [];
            lslam.CertifiedTrajEdgeInfoMat = [];
        end

        % check if there are ignored loop closures associated with
        % current scan
        lslam.LoopClosureBlacklistFromNodeIds = [];
        if isempty(obj.IgnoredLCScanPairs)
            igFound = false;
        else
            [igFound, loc] = ismember(userFacingScanId, obj.IgnoredLCScanPairs(:,2));
        end

        if igFound
            igScanPairsCurrent = obj.IgnoredLCScanPairs(loc, :);
            lslam.LoopClosureBlacklistFromNodeIds = obj.InboundMap(igScanPairsCurrent(:, 1));
        end


        % check if there are modified loop closures available with
        % the current scan
        lslam.LoopClosureFromNodeIds = [];
        lslam.CertifiedLoopClosureEdgePoses = [];
        lslam.CertifiedLoopClosureEdgeInfoMats = [];

        if isempty(obj.ModifiedLCScanPairs)
            lcFound = false;
        else
            [lcFound, loc] = ismember(userFacingScanId, obj.ModifiedLCScanPairs(:,2));
        end

        if lcFound
            lcScanPairsCurrent = obj.ModifiedLCScanPairs(loc, :);
            lcRelPosesCurrent = obj.ModifiedLCRelPoses(loc, :);

            lslam.LoopClosureFromNodeIds = obj.InboundMap(lcScanPairsCurrent(:,1));
            lslam.CertifiedLoopClosureEdgePoses = lcRelPosesCurrent;
            lslam.CertifiedLoopClosureEdgeInfoMats = repmat([1 0 0 1 0 1], size(lcRelPosesCurrent,1), 1);
        end
    end

    function map = getOccupancyGridMap(obj)
    %getOccupancyGridMap
        map = buildMap(obj.LidarSLAMProcessor.Scans, obj.LidarSLAMProcessor.PoseGraph.nodes, obj.LidarSLAMProcessor.MapResolution, obj.LidarSLAMProcessor.MaxLidarRange);
    end

    function clean(obj)
    %clean Reset MapBuilderModel object to pristine state
        obj.Scans = [];
        obj.NumScans = 0;
        obj.RelativePoseEstimates = [];

        obj.HasInitGuess = [];
        obj.LidarSLAMProcessor = [];
        obj.FrontScanIdx = 1;

        obj.InboundMap = [];
        obj.OutboundMap = [];

        obj.History = {};

        obj.LCScanIdPairSelected = [];
        obj.ScanIdPairToShow = [];

        obj.ModifiedEdges = [];
        obj.ModifiedEdgeRelPoses = [];
        obj.IgnoredEdges = [];

        obj.ModifiedLCScanPairs = [];
        obj.ModifiedLCRelPoses = [];
        obj.IgnoredLCScanPairs = [];
        obj.TrajScanPairs = [];
        obj.TrajRelPoses = [];

        obj.IsRunning = 0;
        obj.IsPoseGraphDirty = false;

        obj.AutoScanMatchingResults = containers.Map;

    end

    function cleanPartial(obj)
    %cleanPartial Only remove artifacts from last round of map building
        obj.LidarSLAMProcessor = [];
        obj.FrontScanIdx = 1;

        obj.InboundMap = [];
        obj.OutboundMap = [];

        obj.History = {};

        obj.LCScanIdPairSelected = [];
        obj.ScanIdPairToShow = [];

        obj.ModifiedEdges = [];
        obj.ModifiedEdgeRelPoses = [];
        obj.IgnoredEdges = [];

        obj.ModifiedLCScanPairs = [];
        obj.ModifiedLCRelPoses = [];
        obj.IgnoredLCScanPairs = [];
        obj.TrajScanPairs = [];
        obj.TrajRelPoses = [];

        obj.IsRunning = 0;
        obj.IsPoseGraphDirty = false;

        obj.AutoScanMatchingResults = containers.Map;
    end

    function [infoStruct, extraInfoStruct] = saveProperties(obj)
    %saveProperties

    % basic properties
        infoStruct = saveProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj);

        % observable properties
        extraInfoStruct.IsPoseGraphDirty = obj.IsPoseGraphDirty;
        extraInfoStruct.IsRunning = obj.IsRunning; % not used for deserialization
    end

    function loadProperties(obj, infoStruct1, infoStruct2)
    %loadProperties
        loadProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj, infoStruct1);

        % sync state
        obj.IsPoseGraphDirty = infoStruct2.IsPoseGraphDirty;

    end
end
end
