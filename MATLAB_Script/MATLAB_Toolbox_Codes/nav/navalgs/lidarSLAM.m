classdef (Sealed)lidarSLAM < nav.algs.internal.InternalAccess
%lidarSLAM Create a lidar SLAM object
%   lidarSLAM creates a lidar-based SLAM object. SLAM stands for
%   Simultaneous Localization And Mapping. It incrementally takes in
%   new lidar scans and attaches it to a node in the underlying pose
%   graph. The SLAM algorithm also searches for new loop closures and
%   re-optimizes the node poses in the pose graph as new scans are added.
%
%   SLAMOBJ = lidarSLAM creates a lidar SLAM object, SLAMOBJ. The
%   default occupancy grid map size is 20 cells per meter. The default
%   maximum range for each lidar scan is 8m.
%
%   SLAMOBJ = lidarSLAM(MAPRESOLUTION, MAXLIDARRANGE) creates a lidar
%   SLAM object, SLAMOBJ. The resolution of the occupancy grid map to be
%   created by the SLAM object is specified by MAPRESOLUTION as number
%   of cells per meter. The maximum range of each lidar scan is specified
%   by MAXLIDARRANGE in meters. These two inputs correspond to
%   SLAMOBJ's properties with the same names and cannot be modified
%   after the SLAMOBJ is constructed.
%
%   SLAMOBJ = lidarSLAM(MAPRESOLUTION, MAXLIDARRANGE, MAXNUMSCANS) 
%   specifies the upper bound on the number of accepted scans allowed in 
%   SLAMOBJ when generating code. This scan limit is only required when 
%   generating  code. During MATLAB execution MAXNUMSCANS limit is ignored.
%
%   lidarSLAM properties:
%      PoseGraph                - Underlying pose graph that connects all 
%                                 scans
%      MapResolution            - Resolution of the occupancy grid map
%      MaxLidarRange            - Maximum lidar range
%      OptimizationFcn          - Pose graph optimization function
%      LoopClosureThreshold     - Threshold for accepting loop closures
%      LoopClosureSearchRadius  - Radius in which loop closures are 
%                                 searched
%      LoopClosureMaxAttempts   - Number of attempts of finding loop 
%                                 closure for each added scan
%      LoopClosureAutoRollback  - Allow auto rollback of added loop closure
%      OptimizationInterval     - Number of new loop closures accumulated 
%                                 before re-optimization
%      MovementThreshold        - Minimum change in pose to trigger scan 
%                                 acceptance
%      ScanRegistrationMethod   - Scan registration method
%
%   lidarSLAM methods:
%      addScan              - Add a new scan to SLAM object
%      removeLoopClosures   - Remove loop closures from SLAM object
%      scansAndPoses        - Return scans and poses for given nodes
%      copy                 - Create a copy of the object
%      show                 - Show overlaid scans and robot trajectory
%
%   Example:
%      % Create a lidarSLAM object using the default branch and bound 
%      % scan registration method.
%      maxLidarRange = 5;
%      mapResolution1 = 40;
%      ldslmObj1 = lidarSLAM(mapResolution1, maxLidarRange);
%
%      % Create another lidarSLAM object using the phase correlation 
%      % scan registration method. Specify different map resolution when 
%      % using phase correlation method. This resolution is empirically
%      % chosen to achieve scan registration accuracy similar to branch and
%      % bound method.
%      mapResolution2 = 70;
%      ldslmObj2 = lidarSLAM(mapResolution2, maxLidarRange);
%      ldslmObj2.ScanRegistrationMethod = 'PhaseCorrelation';
%
%      % Create a pair of dummy laser scans
%      ranges = [6*ones(1, 100), 7*ones(1,100), 4*ones(1,100)];
%      angles = linspace(-pi/2, pi/2, 300);
%      scan1 = lidarScan(ranges, angles);
%      scan2 = transformScan(scan1, [0.5, 0.2, 1.1]);
%
%      % Add the scans to the lidar SLAM object
%      addScan(ldslmObj1, scan1);
%      addScan(ldslmObj1, scan2);
%      addScan(ldslmObj2, scan1);
%      addScan(ldslmObj2, scan2);
%
%      % Visualize the scans and robot trajectory
%      show(ldslmObj1);
%      figure;
%      show(ldslmObj2);
%
%   % Note: Image Processing Toolbox is required for using Phase 
%   % Correlation Method.
%
%   See also occupancyMap

%   Copyright 2017-2021 The MathWorks, Inc.

%#codegen

    properties (Constant, Access = {?nav.algs.internal.InternalAccess})
        % scan registration method names
        ScanRegistrationMethodNames = {'PhaseCorrelation',... phase correlation algorithm uses imregcorr from IPT
                               'BranchAndBound'... branch and bound algorithm uses matchScans and matchScansGrid from NAV
                           };
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})


        %NumScansSinceLastSubmap
        NumScansSinceLastSubmap

        %NumScansPerSubmap
        NumScansPerSubmap

        %NumNewLoopClosures Number of loop closures since last pose graph
        %   optimization
        NumNewLoopClosures

        %ResidualErrorHistory
        ResidualErrorHistory
        %NextResidualId Index of the next optimization output residual e.g.,
        %   till now if 5 times, residual computed by pose graph optimization is
        %   stored in ResidualErrorHistory then NextResidualId is 6. This property
        %   is needed due to codegen limitation
        NextResidualId

        %Scans
        Scans
        %MaxNumScans is the upper bound on number of accepted scans while generating code
        MaxNumScans
        %NextScanId is the index of the next scan e.g., if currently 100
        %   scans are inserted, then NextScanId is 101. This property is needed
        %   due to codegen limitation cell growing (cell{end+1}).
        NextScanId

        %Submaps
        Submaps
        %NextSubmapId Index of the next submap e.g., if 10 SubMaps are
        %   created till now then NextSubmapId is 11. This property is needed
        %   due to codegen limitation cell growing (cell{end+1}).
        NextSubmapId

        %IdentityInformationMatrix
        IdentityInformationMatrix

    end

    properties (SetAccess = {?nav.algs.internal.InternalAccess})

        %PoseGraph The underlying pose graph
        %   This pose graph connects all the added lidar scans.
        %
        %   Default: poseGraph
        PoseGraph

        %MapResolution The resolution of built map
        %
        %   Default: 20 (cells per meter)
        MapResolution

        %MaxLidarRange The maximum range of the lidar sensor
        %
        %   Default: 8 (meter)
        MaxLidarRange
    end

    properties

        %OptimizationFcn Pose graph optimization function
        %   The function must have the following signature:
        %   [UPDATEDPOSE, STAT] = optimizationFcn(POSEGRAPH) where
        %   POSEGRAPH is a poseGraph object, UPDATEDPOSE is an
        %   N-by-3 matrix containing updated poses for POSEGRAPH (N is the
        %   number of nodes in POSEGRAPH) listed in the order of node ID,
        %   and STAT is a struct and it must have a field called
        %   ResidualError, which is a positive scalar.
        %
        %   Default: optimizePoseGraph
        OptimizationFcn

        %LoopClosureThreshold Threshold for accepting a loop closure
        %   This threshold is applied to a score from the scan matching
        %   algorithm used to identify loop closures. Higher threshold
        %   in general correlates to a better match more, but that's NOT
        %   guaranteed.
        %
        %   Default: 100
        LoopClosureThreshold

        %LoopClosureSearchRadius Loop closure search radius for newly added scan
        %
        %   Default: 8 (meter)
        LoopClosureSearchRadius

        %LoopClosureMaxAttempts Number of attempts for finding loop closure for newly added scan
        %   The SLAM object will look for loop closures in different
        %   directions around current pose estimate of the newly added scan
        %   in different attempts.
        %
        %   Default: 1
        LoopClosureMaxAttempts

        %LoopClosureAutoRollback Allow auto rollback (reject) newly added loop closure edges
        %   Lidar SLAM object keeps track of its pose graph optimization
        %   residual error history. When new loop closures are added and
        %   the underlying pose graph is re-optimized, if a sudden spike
        %   in the residual error is detected, the most recently-added loop
        %   closure edges are automatically rejected (rolled back)
        %
        %   Default: true
        LoopClosureAutoRollback

        %OptimizationInterval Number of newly added loop closures before pose graph re-optimization is performed
        %
        %   Default: 1 (re-optimize for every added loop closure)
        OptimizationInterval

        %MovementThreshold Minimum change in pose to trigger scan acceptance
        %   This property is a 2-vector in the following form
        %   [translationThreshold rotationThreshold]
        %
        %   Default: [0 0] (always accepting)
        MovementThreshold

    end

    properties (Access = {?nav.algs.internal.InternalAccess})
        %UseCertifiedData A Boolean indicating whether to override the on-the-fly scan
        %   matching calculation with pre-existing (certified) results
        UseCertifiedData

        % when UseCertifiedData property is true, the following properties need to
        % be provided.

        %CertifiedTrajEdgePose
        CertifiedTrajEdgePose

        %CertifiedEdgeInfoMat
        CertifiedTrajEdgeInfoMat

        %LoopClosureFromNodeIds
        LoopClosureFromNodeIds

        %CertifiedLoopClosureEdgePoses
        CertifiedLoopClosureEdgePoses

        %CertifiedLoopClosureInfoMats
        CertifiedLoopClosureEdgeInfoMats

        %LoopClosureBlacklistFromNodeIds
        LoopClosureBlacklistFromNodeIds
        
        %GridSize
        GridSize
        
        %GridLines
        GridLines
        
        %LastGrid
        LastGrid
                
        %ScanRegistrationMethodInternal
        ScanRegistrationMethodInternal

        %SearchRangeWithGuess Incremental Scan Registration Translation and
        %   Rotation Search Range
        SearchRangeWithGuess
        
        %LastScanRegistrationMethod
        LastScanRegistrationMethod
    end
    
    properties (Dependent)
        %ScanRegistrationMethod Scan registration method to use in addScan
        %   method, specified as a character vector. The possible choices
        %   are 'BranchAndBound and 'PhaseCorrelation'.
        %
        %   Default: 'BranchAndBound' 
        ScanRegistrationMethod

        %TranslationSearchRange Incremental match translational search
        %   range, specified as a two-element vector of the form [x y] in
        %   meters. This property is only applicable when the
        %   ScanRegistrationMethod property is set to 'BranchAndBound'.
        %   These values define the search window around the initial
        %   translation estimate given in relPoseEst argument of addScan
        %   method. Set the value of this property to the maximum expected
        %   translation between consecutive accepted scans. This property
        %   is similar to the 'TranslationSearchRange' name-value pair
        %   argument in matchScansGrid function
        %
        %   Default: [maxLidarRange/2, maxLidarRange/2] 
        TranslationSearchRange

        %RotationSearchRange Incremental match rotational search range,
        %   specified as positive scalar in radians. This property is only
        %   applicable when the ScanRegistrationMethod property is set to
        %   'BranchAndBound'. This values define the search window around
        %   the initial rotation estimate given in relPoseEst argument of
        %   addScan method. Set the value of this property to the maximum
        %   expected rotation between consecutive accepted scans. This
        %   property is similar to the 'RotationSearchRange' name-value
        %   pair argument in matchScansGrid function.
        %
        %   Default: pi/2
        RotationSearchRange
    end

    methods
        function obj = lidarSLAM(mapResolution, maxLidarRange, maxNumScans)
        %lidarSLAM The constructor

            if coder.target('MATLAB')
                % For MATLAB execution the number of arguments can be 0-2
                narginchk(0, 4);

                % Pre allocation of Scans is not required for MATLAB execution
                maxNumScans = 0;
            else
                % For codegen users must specify expected upper limit on number
                % of scans.
                coder.internal.assert(nargin>2, 'nav:navalgs:lidarslam:MaxNumScansRequiredForCodegen');

                % Because of codegen limitation Scans should be pre allocated.
                % MaxNumScans is a required input.
                maxNumScans = robotics.internal.validation.validatePositiveNumericScalar(maxNumScans, 'lidarSLAM', 'MaxNumScans');
            end

            % Default values for mapResolution and maxLidarRange
            mapRes = 20;
            maxRange = 8;
            registrationMethod = 'BranchAndBound';

            if nargin >= 1
                mapRes = robotics.internal.validation.validatePositiveIntegerScalar(mapResolution, 'lidarSLAM', 'MapResolution');
            end

            if nargin >= 2
                maxRange = robotics.internal.validation.validatePositiveNumericScalar(maxLidarRange, 'lidarSLAM', 'MaxLidarRange');
            end

            obj.MaxLidarRange = maxRange;
            obj.MapResolution = mapRes;
            obj.MaxNumScans = maxNumScans;

            obj.NextScanId = 1;
            obj.PoseGraph = poseGraph('MaxNumEdges',obj.MaxNumScans,'MaxNumNodes',(obj.MaxNumScans+1));
            obj.OptimizationFcn = @optimizePoseGraph;

            obj.Scans = coder.nullcopy(cell(1,obj.MaxNumScans));
            if ~coder.target('MATLAB')
                % create dummy scan and pre-allocate Scans due to codegen limitation
                coder.varsize('rowVec', [1, inf], [0, 1]);
                coder.varsize('colVec', [inf, 1], [1, 0]);
                colVec = 0;
                % dummy lidarscan
                scan = lidarScan(colVec,colVec);
                for k = 1:obj.MaxNumScans
                    obj.Scans{k} = scan;
                end
            end


            obj.LoopClosureThreshold = 100;
            obj.LoopClosureSearchRadius = 8;
            obj.LoopClosureMaxAttempts = 1;
            obj.LoopClosureAutoRollback = true;
            % due to codegen limitation, define LoopClosureBlacklistFromNodeIds
            % as an 1-by-M variable-sized vector.
            rowVec = [];
            obj.LoopClosureBlacklistFromNodeIds = rowVec;
            obj.LoopClosureFromNodeIds = rowVec;

            obj.IdentityInformationMatrix = true;

            obj.NumScansSinceLastSubmap = 0;
            obj.NumScansPerSubmap = 7;
            obj.NumNewLoopClosures = 0;

            numSubMaps = floor(obj.MaxNumScans/obj.NumScansPerSubmap);
            obj.Submaps = coder.nullcopy(cell(1,numSubMaps));

            if (~coder.target('MATLAB'))&&(numSubMaps>0)
                % dummy submap
                smap = nav.algs.internal.createSubmap();
                % due to codegen limitation, pre-allocate Submaps
                for k = 1:numSubMaps
                    obj.Submaps{k} = smap;
                end
            end
            obj.NextSubmapId = 1;

            obj.OptimizationInterval = 1;
            obj.MovementThreshold = [0, 0];

            % computation histories
            obj.ResidualErrorHistory = zeros(1,obj.MaxNumScans);
            obj.NextResidualId = 1;


            % user certified data
            obj.UseCertifiedData = false;
            if ~coder.target('MATLAB')
                % create dummy variable size mats due to codegen limitation
                coder.varsize('mat1', [inf, 3], [1, 1]);
                coder.varsize('mat2', [inf, 6], [1, 1]);
            end
            mat1 = [1 0 1];
            mat2 = [1 0 0 1 0 1];
            obj.CertifiedTrajEdgePose = mat1;
            obj.CertifiedTrajEdgeInfoMat = mat2;
            obj.CertifiedLoopClosureEdgePoses = mat1;
            obj.CertifiedLoopClosureEdgeInfoMats = mat2;
            
            if strcmp(registrationMethod,'PhaseCorrelation')
                obj.ScanRegistrationMethodInternal = nav.algs.internal.ScanRegistrationMethod.PhaseCorrelation; %phase correlation
            else
                obj.ScanRegistrationMethodInternal = nav.algs.internal.ScanRegistrationMethod.BranchAndBound; %branch and bound
            end
            obj.LastScanRegistrationMethod = nav.algs.internal.ScanRegistrationMethod.BranchAndBound;
            gridLength = round(2*maxRange*mapRes);
            obj.GridSize = [gridLength,gridLength];
            % Calculate the edges of the bins
            obj.GridLines = linspace(-maxRange,maxRange,obj.GridSize(1)+1);
            % preallocate last grid for code-generation
            obj.LastGrid = false(obj.GridSize);

            obj.SearchRangeWithGuess = [maxRange/2,maxRange/2,pi/2];
        end


        function [isScanAccepted, loopClosureInfo, optimizationInfo ] = addScan(obj, currScan, relPoseEstimate)
        %addScan Add scan to lidar SLAM object
        %   addScan(SLAMOBJ, CURRSCAN) adds a current scan CURRSCAN to
        %   the lidar SLAM object. CURRSCAN is matched to the latest
        %   SCAN in SLAMOBJ. New loop closure detection and whole pose
        %   graph re-optimization may also be performed in the background
        %   as CURRSCAN is added, depending on the user settings.
        %
        %   addScan(SLAMOBJ, CURRSCAN, RELPOSEESTIMATE) allows the user
        %   to specify an estimated pose of CURRSCAN relative to the
        %   latest lidar scan pose in SLAMOBJ.
        %
        %   Note: RELPOSEESTIMATE is ignored if the scan registration
        %   method is set to phase correlation
        %
        %   [ISSCANACCEPTED, LOOPCLOSUREINFO, OPTIMIZATIONINFO] = addScan(SLAMOBJ, ___)
        %   reports the detailed results from addScan.
        %
        %      ISSCANACCEPTED - A Boolean indicating whether CURRSCAN
        %      is accepted.
        %
        %      LOOPCLOSUREINFO - A struct containing information about
        %      the loop closure added around CURRSCAN. This struct
        %      contains 3 fields:
        %      'EdgeIDs'        - The IDs of newly added loop closure edges
        %      'EdgeNodePairs'  - The node pairs corresponding to the newly
        %                         added loop closure edges
        %      'Scores'  - The loop closure score for each added loop
        %                  closure edges
        %      NOTE the loop closure edges listed in this struct might 
        %      no longer be present in the pose graph if the automatic
        %      loop closure rollback mechanism is turned on.
        %
        %      OPTIMIZATIONINFO - A struct containing information about
        %      the pose graph optimization performed while CURRSCAN is
        %      added. This struct contains 4 fields.
        %      'IsPerformed'         - Whether pose graph optimization
        %                              is performed when CURRSCAN is
        %                              added
        %      'IsAccepted'          - Whether the pose graph optimization
        %                              result is accepted
        %      'ResidualError'       - The residual error associated
        %                              with the pose graph optimization
        %      'LoopClosuresRemoved' - The list of IDs of loop closure
        %                              edges removed
        %
        %   Example:
        %      % Create a new lidarSLAM object
        %      maxLidarRange = 5;
        %      mapResolution = 40;
        %      lidarSlam = lidarSLAM(mapResolution, maxLidarRange);
        %
        %      % Create a pair of dummy laser scans
        %      ranges = [6*ones(1, 100), 7*ones(1,100), 4*ones(1,100)];
        %      angles = linspace(-pi/2, pi/2, 300);
        %      scan1 = lidarScan(ranges, angles);
        %      scan2 = transformScan(scan1, [0.5, 0.2, 1.1]);
        %      scan3 = transformScan(scan2, [0.5, 0.2, 1.1]);
        %
        %      % Add the scans to the lidar SLAM object
        %      lidarSlam.addScan(scan1);
        %      lidarSlam.addScan(scan2);
        %
        %      % If a good initial guess of the relative pose is available,
        %      % using lower search ranges allows addScan to run faster.
        %      lidarSlam.TranslationSearchRange = [1,1];
        %      lidarSlam.RotationSearchRange = 1.5; 
        %      % Search ranges are only applicable when an initial is
        %      % available. Here, Assume [0,0,0] as an initial guess since 
        %      % the expected motion is small.
        %      lidarSlam.addScan(scan3,[0,0,0]); 
        %
        %      % Visualize the scans and robot trajectory 
        %      lidarSlam.show

            narginchk(2,3);
            isScanAccepted = false;

            % incremental scan matching
            if (obj.NextScanId == 1)
                currGrid = obj.rasterizeScan(currScan);
                obj.Scans{1} = currScan; % and ignore the relative pose if it's provided
                obj.LastGrid = currGrid;
                obj.NextScanId = obj.NextScanId + 1;
                isScanAccepted = true;
                obj.NumScansSinceLastSubmap = obj.NumScansSinceLastSubmap + 1;

                % Because of codegen limitation all the output variables of
                % add scans should be initialized as variable sized
                % matrices while adding first scan.
                coder.varsize('varSizeMat', [inf, inf], [1, 1]);
                varSizeMat = [];
                loopClosureInfo.EdgeIDs = varSizeMat;
                loopClosureInfo.EdgeNodePairs = varSizeMat;
                loopClosureInfo.Scores = varSizeMat;

                optimizationInfo.IsPerformed = false;
                optimizationInfo.IsAccepted = false;
                optimizationInfo.ResidualError = varSizeMat;
                optimizationInfo.LoopClosuresRemoved = varSizeMat;
                return
            end

            maxLevel = 5;

            if obj.UseCertifiedData
                relPose = obj.CertifiedTrajEdgePose;
                infoMat = obj.CertifiedTrajEdgeInfoMat;

            else
                
                if (obj.ScanRegistrationMethodInternal == nav.algs.internal.ScanRegistrationMethod.PhaseCorrelation)
                    % imregcorr reports the transformation from the origin. Hence provide a
                    % referencing such that the sensor is at the center of the images
                    Rgrid = imref2d(size(obj.LastGrid));
                    offsetX = mean(Rgrid.XWorldLimits);
                    Rgrid.XWorldLimits = Rgrid.XWorldLimits - offsetX;
                    offsetY = mean(Rgrid.YWorldLimits);
                    Rgrid.YWorldLimits = Rgrid.YWorldLimits - offsetY;    
                    currGrid = obj.rasterizeScan(currScan);
                    % Find relative pose in image coordinates
                    if obj.LastScanRegistrationMethod == nav.algs.internal.ScanRegistrationMethod.BranchAndBound
                        % if the last scan registration method is branch
                        % and bound we need to compute grid from the last
                        % accepted scan
                        lastGrid = obj.rasterizeScan(obj.Scans{obj.NextScanId-1});
                        relPoseImage = imregcorr(currGrid, Rgrid, lastGrid, Rgrid);
                        obj.LastScanRegistrationMethod = nav.algs.internal.ScanRegistrationMethod.PhaseCorrelation;
                    else
                        % if the last scan registration method is phase
                        % correlation last accepted scan grid is expected
                        % to be stored in LastGrid
                        relPoseImage = imregcorr(currGrid, Rgrid, obj.LastGrid, Rgrid);
                    end
                    
                    % extract tform from rigid2d object. it stores the
                    % transpose of 3x3 transformation matrix in it property T
                    relPoseTform = relPoseImage.T';
                    % Convert translation to world coordinates
                    relPoseTform(1:2,3) = relPoseTform(1:2,3)./obj.MapResolution; 
                    relPose = robotics.core.internal.SEHelpers.tformToPoseSE2(relPoseTform);
                    infoMat = [1 0 0 1 0 1];
                    obj.LastGrid = currGrid;
                else
                    if obj.IdentityInformationMatrix && nargin < 3
                        % matchScansGrid computes covariance only when 2
                        % outputs are requested. While using identity
                        % information matrix covariance computation isn't
                        % necessary.
                        relPose = matchScansGrid(currScan, obj.Scans{obj.NextScanId-1}, ...
                            'MaxRange', obj.MaxLidarRange, 'Resolution', obj.MapResolution);

                    elseif obj.IdentityInformationMatrix && (nargin > 2)
                        relPoseEstimate = robotics.internal.validation.validateMobilePose(relPoseEstimate(:), 'addScan', 'relPoseEstimate');
                        relPose = matchScansGrid(currScan, obj.Scans{obj.NextScanId-1}, 'InitialPose', relPoseEstimate, ...
                            'MaxRange', obj.MaxLidarRange, 'Resolution', obj.MapResolution, ...
                            'TranslationSearchRange', obj.SearchRangeWithGuess(1:2), ...
                            'RotationSearchRange', obj.SearchRangeWithGuess(3) );
                    elseif ~obj.IdentityInformationMatrix && nargin < 3
                        [relPose, stats] = matchScansGrid(currScan, obj.Scans{obj.NextScanId-1}, ...
                            'MaxRange', obj.MaxLidarRange, 'Resolution', obj.MapResolution);
                    else
                        relPoseEstimate = robotics.internal.validation.validateMobilePose(relPoseEstimate(:), 'addScan', 'relPoseEstimate');
                        [relPose, stats] = matchScansGrid(currScan, obj.Scans{obj.NextScanId-1}, 'InitialPose', relPoseEstimate, ...
                            'MaxRange', obj.MaxLidarRange, 'Resolution', obj.MapResolution, ...
                            'TranslationSearchRange', obj.SearchRangeWithGuess(1:2), ...
                            'RotationSearchRange', obj.SearchRangeWithGuess(3) );
                    end
                    
                    relPoseRefined = matchScans(currScan,obj.Scans{obj.NextScanId-1}, 'initialPose', relPose);
                    if obj.IdentityInformationMatrix
                        infoMat = [1 0 0 1 0 1];
                    else
                        m = inv(stats.Covariance);
                        infoMat = robotics.core.internal.SEHelpers.serializeInformationMatrixSE2(m);
                    end
                    if norm(relPoseRefined(1:2) - relPose(1:2)) < 2*(1/obj.MapResolution) && abs(robotics.internal.wrapToPi(relPoseRefined(3)) - robotics.internal.wrapToPi(relPose(3))) < 0.05
                        relPose = relPoseRefined;
                    end
                end
            end


            % decide whether to accept or discard the scan
            if obj.relativePoseAcceptable(relPose)
                isScanAccepted = true;
                loopClosureInfo = obj.acceptRelPoseAndSearchForLoopClosures(currScan, relPose, infoMat);
            else
                loopClosureInfo.EdgeIDs = [];
                loopClosureInfo.EdgeNodePairs = [];
                loopClosureInfo.Scores = [];
            end

            % run graph optimization
            if obj.NumNewLoopClosures >= obj.OptimizationInterval
                optimizationInfo = obj.updatePoseGraph;
            else
                optimizationInfo.IsPerformed = false;
                optimizationInfo.IsAccepted = false;
                optimizationInfo.ResidualError = [];
                optimizationInfo.LoopClosuresRemoved = [];
            end

            % form submaps

            if obj.NumScansSinceLastSubmap >= obj.NumScansPerSubmap
                indices = (obj.PoseGraph.NumNodes - obj.NumScansPerSubmap+1) : obj.PoseGraph.NumNodes;
                poses = obj.PoseGraph.nodeEstimates(indices);
                anchorIndex = round(obj.NumScansPerSubmap/2);

                obj.Submaps{obj.NextSubmapId} = nav.algs.internal.createSubmap(obj.Scans, indices, poses, anchorIndex, obj.MapResolution, obj.MaxLidarRange, maxLevel);
                obj.NextSubmapId = obj.NextSubmapId + 1;
                obj.NumScansSinceLastSubmap = 0;

            end


        end

        function removeLoopClosures(obj, lcEdgeIDs)
        %removeLoopClosures Remove loop closures from underlying pose graph
        %   removeLoopClosure(SLAMOBJ) removes all the loop closures
        %   identified in SLAMOBJ
        %
        %   removeLoopClosure(SLAMOBJ, LCEDGEIDS) removes the loop
        %   closures specified by the loop closure IDs (LCEDGEIDS)
        %   in the SLAMOBJ's underlying pose graph.


            narginchk(1,2);
            if nargin == 1
                lcEdgeIDs = obj.PoseGraph.LoopClosureEdgeIDs;
            end
            obj.PoseGraph.removeEdges(lcEdgeIDs);

        end


        function [scans, poses] = scansAndPoses(obj, nodeIDs)
        %scansAndPoses Returns scans and poses corresponding to pose graph nodes
        %   [SCANS, POSES] = scansAndPoses(SLAMOBJ) returns all the
        %   scans SCANS and poses POSES in SLAMOBJ.
        %
        %   [SCANS, POSES] = scansAndPoses(SLAMOBJ, NODEIDS) returns
        %   SCANS and POSES corresponding to the specified NODEIDS.
        %   NODEIDS is a vector of positive integers.

            if nargin > 1
                validateattributes(nodeIDs, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.PoseGraph.NumNodes}, 'scansAndPoses', 'nodeIds');
            else
                nodeIDs = 1:obj.PoseGraph.NumNodes;
            end

            % For codegen access using () parenthesis is not supported
            scans = coder.nullcopy(cell(1,length(nodeIDs)));
            for k = 1:length(nodeIDs)
                scans{k} = obj.Scans{nodeIDs(k)};
            end
            poses = obj.PoseGraph.nodeEstimates(nodeIDs);

        end



        function newObj = copy(obj)
        %copy Creates a copy of the object
        %   NEWOBJ = COPY(OBJ) creates a deep copy of the
        %   lidar SLAM object with the same properties.
            if coder.target('MATLAB')
                newObj = lidarSLAM(obj.MapResolution, obj.MaxLidarRange);
            else
                newObj = lidarSLAM(obj.MapResolution, obj.MaxLidarRange, obj.MaxNumScans);
            end

            % internal settings
            newObj.OptimizationFcn = obj.OptimizationFcn;
            newObj.LoopClosureThreshold = obj.LoopClosureThreshold;
            newObj.LoopClosureSearchRadius = obj.LoopClosureSearchRadius;
            newObj.LoopClosureMaxAttempts = obj.LoopClosureMaxAttempts;
            newObj.LoopClosureAutoRollback = obj.LoopClosureAutoRollback;

            newObj.OptimizationInterval = obj.OptimizationInterval;
            newObj.MovementThreshold = obj.MovementThreshold;

            newObj.IdentityInformationMatrix = obj.IdentityInformationMatrix;

            newObj.NumScansPerSubmap = obj.NumScansPerSubmap;

            % internal data
            newObj.PoseGraph = copy(obj.PoseGraph);
            newObj.Scans = obj.Scans;    % cell array of value class objects
            newObj.Submaps = obj.Submaps; % cell array of value class objects

            newObj.NumScansSinceLastSubmap = obj.NumScansSinceLastSubmap;
            newObj.NumNewLoopClosures = obj.NumNewLoopClosures;

            newObj.ResidualErrorHistory = obj.ResidualErrorHistory;

            % internal properties
            newObj.NextScanId = obj.NextScanId;
            newObj.NextSubmapId = obj.NextSubmapId;
            newObj.NextResidualId = obj.NextResidualId;
            
            % internal properties useful for phase correlation scan registration method
            newObj.ScanRegistrationMethodInternal = obj.ScanRegistrationMethodInternal;
            newObj.LastScanRegistrationMethod = obj.LastScanRegistrationMethod;
            newObj.LastGrid = obj.LastGrid;
            newObj.GridSize = obj.GridSize;
            newObj.GridLines = obj.GridLines;
        end

        function ax = show(obj, varargin)
        %show Plot overlaid scans and trace of the lidar scan sensor
        %   show(SLAMOBJ) plots lidar scans in SLAMOBJ at their current
        %   estimated poses. The estimated trace of the lidar sensor is
        %   also shown.
        %
        %   AX = show(SLAMOBJ) returns the axes handle under which
        %   the scans are plotted.
        %
        %   show(___, Name, Value) provides additional options specified
        %   by one or more Name, Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1, Value1, ..., NameN, ValueN:
        %
        %   'Parent' - Handle of the axes in which the scans
        %              are to be rendered
        %
        %   'Poses'  - Display lidar sensor poses
        %              Possible values: {'on', 'off'}
        %              Default: 'on'
            
            % If called in code generation, throw incompatibility error
            coder.internal.errorIf(~coder.target('MATLAB'), 'nav:navalgs:lidarslam:CodegenNotSupported', 'show');
            
            parser = inputParser;
            parser.StructExpand = false;

            parser.addParameter('Parent', [], ...
                                @(x)robotics.internal.validation.validateAxesUIAxesHandle(x));
            parser.addParameter('Poses', 'on', ...
                                @(x)any(validatestring(x, {'on', 'off'})));
            parser.parse(varargin{:});

            ph = parser.Results.Parent;
            if strcmp(parser.Results.Poses, 'on')
                displayPoses = true;
            else
                displayPoses = false;
            end

            if isempty(ph)
                ax = newplot;
            else
                ax = newplot(ph);
            end

            if strcmp(ax.NextPlot, 'replace') % when hold is off
                cla(ax);
                axis(ax, 'equal');
                box(ax, 'on');
                grid(ax, 'on');
            end

            for i = 1:(obj.NextScanId-1)
                hold(ax, 'on');
                sc = transformScan(obj.Scans{i}.removeInvalidData('RangeLimits', [0.02 obj.MaxLidarRange]), obj.PoseGraph.nodeEstimates(i));
                scPoints = sc.Cartesian;
                plot(ax, scPoints(:,1), scPoints(:,2), '.', 'MarkerSize', 3, 'color', 'm');
            end
            if displayPoses
                nds = obj.PoseGraph.nodeEstimates;
                plot(ax, nds(:,1), nds(:,2), '.-', 'MarkerSize', 5, 'color', 'b');
            end
            hold(ax, 'off');
        end
    end


    methods
        function set.OptimizationFcn(obj, optFcn)
        %set.OptimizationFcn
            if coder.target('MATLAB')
                assert(isa(optFcn, 'function_handle')...
                       ||(iscell(optFcn) && isa(optFcn{1}, 'function_handle')),...
                       message('nav:navalgs:lidarslam:InvalidFunctionHandle'));
            else
                coder.internal.assert(isa(optFcn, 'function_handle')...
                                      ||(iscell(optFcn) && isa(optFcn{1}, 'function_handle')),...
                                      'nav:navalgs:lidarslam:InvalidFunctionHandle')
            end
            obj.OptimizationFcn = optFcn;
        end

        function set.LoopClosureThreshold(obj, lcThreshold)
        %set.LoopClosureThreshold
            lcThreshold = robotics.internal.validation.validatePositiveNumericScalar(lcThreshold, 'lidarSLAM', 'LoopClosureThreshold');
            obj.LoopClosureThreshold = lcThreshold;
        end

        function set.LoopClosureSearchRadius(obj, searchRadius)
        %set.LoopClosureSearchRadius
            searchRadius = robotics.internal.validation.validatePositiveNumericScalar(searchRadius, 'lidarSLAM', 'LoopClosureSearchRadius');
            obj.LoopClosureSearchRadius = searchRadius;
        end

        function set.OptimizationInterval(obj, interval)
        %set.OptimizationInterval
            interval = robotics.internal.validation.validatePositiveNumericScalar(interval, 'lidarSLAM', 'OptimizationInterval');
            obj.OptimizationInterval = double(interval);
        end

        function set.LoopClosureAutoRollback(obj, rollback)
        %set.LoopClosureAutoRollback
            rollback = robotics.internal.validation.validateLogical(rollback, 'lidarSLAM', 'LoopClosureAutoRollback');
            obj.LoopClosureAutoRollback = rollback;
        end

        function set.MovementThreshold(obj, threshold)
        %set.MovementThreshold
            validateattributes(threshold, {'numeric'}, {'nonempty', 'vector', 'numel', 2, 'real', ...
                                'nonnan', 'finite', 'nonnegative'}, 'lidarSLAM', 'MovementThreshold');
            obj.MovementThreshold =  double(threshold);
        end


        function set.LoopClosureMaxAttempts(obj, maxAttempts)
        %set.LoopClosureMaxAttempts
            validateattributes(maxAttempts, {'numeric'}, {'nonempty', 'scalar', 'integer', ...
                                'nonnan', 'finite', 'nonnegative'}, 'lidarSLAM', 'MovementThreshold');
            obj.LoopClosureMaxAttempts =  double(maxAttempts);
        end

        function set.IdentityInformationMatrix(obj, infoMatIsIdentity)
        %set.IdentityInformationMatrix
            infoMatIsIdentity = robotics.internal.validation.validateLogical(infoMatIsIdentity, 'lidarSLAM', 'IdentityInformationMatrix');
            obj.IdentityInformationMatrix = infoMatIsIdentity;
        end
        
        function scanRegistrationMethod = get.ScanRegistrationMethod(obj)
        %get.ScanRegistrationMethod
            if obj.ScanRegistrationMethodInternal > 0
                % return algorithm using algorithm id
                scanRegistrationMethod = obj.ScanRegistrationMethodNames{obj.ScanRegistrationMethodInternal};
            else
                %return branch and boud
                scanRegistrationMethod = obj.ScanRegistrationMethodNames{end};
            end
        end
        
        function set.ScanRegistrationMethod(obj,registrationMethod)
            %set.ScanRegistrationMethod
            scanRegistrationMethod = validatestring(registrationMethod, obj.ScanRegistrationMethodNames, 'lidarSLAM', 'ScanRegistrationMethod');
            if strcmp(scanRegistrationMethod,'PhaseCorrelation')
                obj.ScanRegistrationMethodInternal = nav.algs.internal.ScanRegistrationMethod.PhaseCorrelation; %phase correlation
            else
                obj.ScanRegistrationMethodInternal = nav.algs.internal.ScanRegistrationMethod.BranchAndBound; %branch and bound
            end
        end

        function trSearchRange = get.TranslationSearchRange(obj)
        %get.TranslationSearchRange
            trSearchRange = obj.SearchRangeWithGuess(1:2);
        end
        
        function set.TranslationSearchRange(obj,trSearchRange)
            %set.TranslationSearchRange
            validateattributes(trSearchRange, {'numeric'}, {'nonempty', 'real', ...
                        'nonnan', 'finite', 'vector', 'positive', 'numel', 2}, 'lidarSLAM', 'TranslationSearchRange');
            obj.SearchRangeWithGuess(1:2) = double([trSearchRange(1),trSearchRange(2)]);
        end

        function rotSearchRange = get.RotationSearchRange(obj)
        %get.RotationSearchRange
            rotSearchRange = obj.SearchRangeWithGuess(3);
        end
        
        function set.RotationSearchRange(obj,rotSearchRange)
            %set.RotationSearchRange
            rotSearchRange = robotics.internal.validation.validatePositiveNumericScalar(rotSearchRange, 'lidarSLAM', 'RotationSearchRange');
            obj.SearchRangeWithGuess(3) = rotSearchRange;
        end
    end


    methods (Access = {?nav.algs.internal.InternalAccess})
        function grid = rasterizeScan(obj, scan)
            %rasterizeScan convert 2D lidar scan into grid. Grid cell
            %   values are true when atleast one scan point lies within the
            %   cell boundaries. 
            
                xyPoints = scan.Cartesian;
                gridCount = zeros(obj.GridSize(1), obj.GridSize(2));

                % Calculate x-y indices of the points in scan
                xIndices = discretize(xyPoints(:,2), obj.GridLines);
                yIndices = discretize(xyPoints(:,1), obj.GridLines);
                numPoints = scan.Count;
                for i = 1:numPoints
    
                    xIdx = xIndices(i);
                    yIdx = yIndices(i);

                    if ~isnan(xIdx) && ~isnan(yIdx)
                        gridCount(xIdx, yIdx) = gridCount(xIdx, yIdx) + 1;
                    end
                end
                grid = gridCount>0;
        end
        
        function isAcceptable = relativePoseAcceptable(obj, relPose)
        %relativePoseAcceptable

            isAcceptable = false;

            if isempty(relPose)
                return;
            end

            if obj.UseCertifiedData
                isAcceptable = true;
                return;
            end


            % minimal change of pose is required between two scans
            if (norm(relPose(1:2)) > obj.MovementThreshold(1)) || ...
                    (abs(robotics.internal.wrapToPi(relPose(3)) )>obj.MovementThreshold(2))
                isAcceptable = true;
            end

        end

        function submapIDs = findNearbySubmaps(obj, mostRecentScanCenter)
        %findNearbySubmaps
            nsbmp = obj.NextSubmapId-1;
            centers = zeros(nsbmp - 1, 2);
            for i = 1:nsbmp - 1 % ignore the most recent submap
                centers(i,:) = obj.Submaps{i}.Center;
            end

            centerCandidates = zeros(nsbmp - 1,2);
            k = 1;
            for i = 1:nsbmp - 1
                d = norm(centers(i, :) - mostRecentScanCenter);
                if d < obj.LoopClosureSearchRadius
                    centerCandidates(k,:) = [d i];
                    k = k + 1;
                end
            end
            centerCandidates = centerCandidates(1:(k-1),:);

            if ~isempty(centerCandidates)
                centerCandidates = sortrows(centerCandidates);

                N = min(obj.LoopClosureMaxAttempts, size(centerCandidates,1));
                submapIDs = centerCandidates(1:N, 2)';
            else
                submapIDs = [];
            end
        end

        function loopClosureInfo = acceptRelPoseAndSearchForLoopClosures(obj, currScan, relPose, infoMat)
        %acceptRelPoseAndSearchForLoopClosures

            newLcEdgeIDs = [];
            newLcScores = [];

            obj.PoseGraph.addRelativePose(relPose, infoMat);

            obj.Scans{obj.NextScanId} = currScan;
            obj.NextScanId = obj.NextScanId + 1;
            obj.NumScansSinceLastSubmap = obj.NumScansSinceLastSubmap + 1;


            % if no pre-certified LCs
            if (~obj.UseCertifiedData) || (obj.UseCertifiedData && length(obj.LoopClosureFromNodeIds)<1)

                % check for potential loop closures based on proximity
                if (obj.NextSubmapId-1) > 1
                    scanPose = obj.PoseGraph.nodeEstimates(obj.PoseGraph.NumNodes);
                    nearbySubmapIDs = obj.findNearbySubmaps(scanPose(1:2));
                    if ~isempty(nearbySubmapIDs)
                        for k = 1:length(nearbySubmapIDs)
                            mid = nearbySubmapIDs(k);

                            % only compute scan matching if the current LC edge is not blacklisted
                            if (~obj.UseCertifiedData) || (obj.UseCertifiedData && ~obj.currentScanPairIsBlacklisted(mid))

                                % do the heave-lifting and match scan
                                % against submap on-the-fly
                                [newLcEdgeID, newLcScore] = obj.searchForLoopClosureAgainstSubmap(mid, currScan);

                                newLcEdgeIDs = [newLcEdgeIDs, newLcEdgeID]; %#ok<AGROW>
                                newLcScores = [newLcScores, newLcScore]; %#ok<AGROW>
                            end


                        end
                    end

                end

            else

                for j = 1:length(obj.LoopClosureFromNodeIds)

                    idx = obj.LoopClosureFromNodeIds(j);

                    if ~ismember(idx, obj.LoopClosureBlacklistFromNodeIds)  % just in case an edge is modified first then ignored
                        lcRelPose = obj.CertifiedLoopClosureEdgePoses(j, :);
                        infoMat = obj.CertifiedLoopClosureEdgeInfoMats(j, :);

                        [~, newLcEdgeID] = obj.PoseGraph.addRelativePose(lcRelPose, infoMat, idx, obj.PoseGraph.NumNodes);
                        obj.NumNewLoopClosures = obj.NumNewLoopClosures + 1;

                        newLcScore = -1; % indicating it's a manual override

                        newLcEdgeIDs = [newLcEdgeIDs, newLcEdgeID]; %#ok<AGROW>
                        newLcScores = [newLcScores, newLcScore]; %#ok<AGROW>
                    end
                end


            end


            loopClosureInfo.EdgeIDs = newLcEdgeIDs;
            loopClosureInfo.EdgeNodePairs = [];
            if ~isempty(newLcEdgeIDs)
                loopClosureInfo.EdgeNodePairs = obj.PoseGraph.edgeNodePairs(newLcEdgeIDs);
            end
            loopClosureInfo.Scores = newLcScores;
        end

        function optimizationInfo = updatePoseGraph(obj)
        %updatePoseGraph
            [poseGraphUpdated, stats] = obj.OptimizationFcn(obj.PoseGraph);

            optimizationInfo.IsPerformed = true;
            optimizationInfo.IsAccepted = true;
            optimizationInfo.ResidualError = stats.ResidualError;
            optimizationInfo.LoopClosuresRemoved = [];

            % pose graph optimization residual error jumps, roll
            % back the last batch of loop closure edges (but ONLY when the
            % LC edges added are not pre-certified)
            if  ((~obj.UseCertifiedData) || (obj.UseCertifiedData && length(obj.LoopClosureFromNodeIds)<1)) && ...
                    (obj.LoopClosureAutoRollback) && ...
                    (~(obj.NextResidualId==1)) && ...
                    ( (stats.ResidualError - mean(obj.ResidualErrorHistory(1:(obj.NextResidualId-1)))) > 5*(mean(obj.ResidualErrorHistory(1:(obj.NextResidualId-1)))) ) && (stats.ResidualError > 0.3)

                lcIDs = obj.PoseGraph.LoopClosureEdgeIDs;
                numLC = obj.PoseGraph.NumLoopClosureEdges;
                nn = obj.NumNewLoopClosures;
                loopClosureIDsToRemove = lcIDs(numLC-nn + 1 : numLC);
                obj.PoseGraph.removeEdges(loopClosureIDsToRemove);
                optimizationInfo.IsAccepted = false;
                optimizationInfo.LoopClosuresRemoved = loopClosureIDsToRemove;
                obj.NumNewLoopClosures = 0;
                return;
            end

            obj.PoseGraph.updatePoseGraph(poseGraphUpdated);
            %obj.PoseGraph.quickUpdateNodePoses(graphUpdated.nodeEstimates);
            obj.ResidualErrorHistory(obj.NextResidualId) = stats.ResidualError;
            obj.NextResidualId = obj.NextResidualId + 1;
            obj.NumNewLoopClosures = 0;

            %update the submap centers
            for j = 1:(obj.NextSubmapId - 1)
                nid = obj.Submaps{j}.AnchorScanIndex;
                p = obj.PoseGraph.nodeEstimates(nid);
                obj.Submaps{j}.Center = [p(1) p(2)];
            end
        end


        function [newLcEdgeID, newLcScore] = searchForLoopClosureAgainstSubmap(obj, submapId, currScan)
        %searchForLoopClosureAgainstSubmap Match currScan against the specified submap

            [lcRelPose, score, covariance] = nav.algs.internal.matchScansGridSubmap(currScan, obj.Submaps{submapId}, 0, [0 0 0], [0 0], 0);

            % lower the loop closure acceptance threshold when the scan points are more spread
            deltaLCScore = 0;
            if size(currScan.Ranges(currScan.Ranges > 0.4*obj.MaxLidarRange), 1) > 0.3*size(currScan.Ranges, 1)
                deltaLCScore = - 0.2*obj.LoopClosureThreshold;
            end

            if obj.IdentityInformationMatrix
                infoMat = [1 0 0 1 0 1];
            else
                m = inv(covariance);
                infoMat = robotics.core.internal.SEHelpers.serializeInformationMatrixSE2(m);
            end

            if score > obj.LoopClosureThreshold + deltaLCScore

                % refinement
                lcRelPoseRefined = matchScans(currScan, obj.Scans{obj.Submaps{submapId}.AnchorScanIndex}, 'InitialPose', lcRelPose);
                if norm(lcRelPoseRefined(1:2) - lcRelPose(1:2)) < 2*(1/obj.MapResolution) && abs(robotics.internal.wrapToPi(lcRelPoseRefined(3)) - robotics.internal.wrapToPi(lcRelPose(3))) < 0.05
                    lcRelPose = lcRelPoseRefined;
                end

                refNodeID = obj.Submaps{submapId}.AnchorScanIndex;
                [~, lcEdgeId] = obj.PoseGraph.addRelativePose(lcRelPose, infoMat, refNodeID, obj.PoseGraph.NumNodes);
                obj.NumNewLoopClosures = obj.NumNewLoopClosures + 1;

                newLcEdgeID = lcEdgeId;
                newLcScore =  score - deltaLCScore;

            else
                newLcEdgeID = [];
                newLcScore =  [];
            end

        end

        function clearSLAMResultsAfter(obj, id)
        %clearSLAMResultsAfter Clear SLAM result after scan id,
        %   including id.
        %   Range for id: 2 to numel(obj.Scans)

        % repopulate codegen related properties if they are not set.
            if isempty(obj.NextScanId)
                obj.NextScanId = length(obj.Scans) + 1;
                obj.NextSubmapId = length(obj.Submaps) + 1;
                obj.NextResidualId = length(obj.ResidualErrorHistory) + 1;
            end

            id = double(int32(id));
            assert((id > 1) && (id <=(obj.NextScanId-1)));

            % clean up pose graph
            obj.PoseGraph.clearAfter(id);

            % clean up Scans
            obj.Scans = obj.Scans(1:id-1);
            % Id of the next scan
            obj.NextScanId = id;

            % reset number of new loop closures
            obj.NumNewLoopClosures = mod(obj.PoseGraph.NumLoopClosureEdges, obj.OptimizationInterval);

            % clean up submaps
            lastSubmapId = floor((id-1)/obj.NumScansPerSubmap);
            obj.Submaps = obj.Submaps(1:lastSubmapId);
            % Id of the next submap to create
            obj.NextSubmapId = lastSubmapId + 1;
            obj.NumScansSinceLastSubmap = mod(id-1, obj.NumScansPerSubmap);

            % clean up error history
            numLC = obj.PoseGraph.NumLoopClosureEdges;
            numOpt = floor(numLC/obj.OptimizationInterval);
            obj.ResidualErrorHistory = obj.ResidualErrorHistory(1:numOpt);
            % index of the next residual
            obj.NextResidualId = numOpt + 1;

        end

        function bVal = currentScanPairHasMatchingSolution(obj, submapId)
        %currentScanPairHasMatchingSolution Check whether there is a
        %   pre-computed match between the current scan and the
        %   specified submap.
            bVal = false;
            fromNodeId = obj.Submaps{submapId}.AnchorScanIndex;
            if isempty(obj.LoopClosureFromNodeIds)
                return;
            end
            bVal = ismember(fromNodeId, obj.LoopClosureFromNodeIds);
        end

        function bVal = currentScanPairIsBlacklisted(obj, submapId)
        %currentScanPairIsBlacklisted Check whether the match between
        %   current scan and the specified submap has been blacklisted.
            bVal = false;
            fromNodeId = obj.Submaps{submapId}.AnchorScanIndex;
            if isempty(obj.LoopClosureBlacklistFromNodeIds)
                return;
            end
            bVal = ismember(fromNodeId, obj.LoopClosureBlacklistFromNodeIds);
        end

    end

    methods(Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
        % Let the coder know about non-tunable parameters
            props = {'MaxNumScans','IdentityInformationMatrix'};
        end
        
        function obj = loadobj(s)
        %loadobj Load saved lidarSLAM
            
            if (isa(s,'lidarSLAM'))
                obj = copy(s);
                % some new internal properties are introduced in 21a, if
                % ScanRegistrationMethod is not set, it's from previous
                % releases. since they didn't exist in previous releases
                % while loading mat files stored in previous releases newly
                % introduced properties are set to their default values.
                if isempty(obj.ScanRegistrationMethodInternal)
                    obj.ScanRegistrationMethodInternal = nav.algs.internal.ScanRegistrationMethod.BranchAndBound;
                    obj.LastScanRegistrationMethod = nav.algs.internal.ScanRegistrationMethod.BranchAndBound;
                    gridLength = round(2*obj.MaxLidarRange*obj.MapResolution);
                    obj.GridSize = [gridLength,gridLength];
                    % Calculate the edges of the bins
                    obj.GridLines = linspace(-obj.MaxLidarRange,obj.MaxLidarRange,obj.GridSize(1)+1);
                    obj.LastGrid = false(obj.GridSize);
                end
            end
        end
    end

end
