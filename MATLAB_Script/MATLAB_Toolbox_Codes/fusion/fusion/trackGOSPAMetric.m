classdef trackGOSPAMetric < fusion.internal.metrics.OSPABase
    % trackOSPAMetric Generalized Optimal Subpattern Assignment Metric (GOSPA)
    %  gospaObj = trackGOSPAMetric returns a System object capable of computing
    %  the generalized optimal subpattern assignment metric between a set of
    %  track objects and a set of known truth objects.
    %
    %  gospaObj = trackGOSPAMetric('Name', value) allows you to specify the
    %  properties for the GOSPA metric calculation using name-value pairs. See
    %  the list of properties below.
    %
    %  trackGOSPAMetric properties:
    %   CutoffDistance          - Maximum distance between track and truth (c)
    %   Order                   - Order of the GOSPA metric computation (p)
    %   Alpha                   - Alpha parameter of the GOSPA metric
    %   Distance                - Specifies the physical quantity for distance:
    %                               One of:
    %                              'posnees'   - the normalized estimation
    %                                            error squared (NEES) in track
    %                                            position  (default)
    %                              'velnees'   - track velocity NEES
    %                              'posabserr' - track position absolute error
    %                              'velabserr' - track velocity absolute error
    %                              'custom'    - custom distance function
    %
    %   MotionModel             - Specifies desired motion model.  One of:
    %                               'constvel'  - constant velocity (default)
    %                               'constacc'  - constant acceleration
    %                               'constturn' - constant turn
    %   SwitchingPenalty        - Penalty value for switching assignments
    %   DistanceFcn             - A custom function to compute the distance
    %                             between a track and a truth object
    %   TrackIdentifierFcn      - Handle to a function returning unique
    %                             track identifiers
    %   TruthIdentifierFcn      - Handle to a function returning unique
    %                             truth identifiers
    %   HasAssignmentInput      - A flag to determine if current assignment
    %                             is provided to step method
    %
    %   trackGOSPAMetric methods:
    %   step            - Computes the GOSPA metric
    %   release         - Allows property value and input characteristics change
    %   clone           - Creates a copy of the trackGOSPAMetric object
    %   isLocked        - Locked status (logical)
    %   reset           - Resets states of the trackGOSPAMetric
    %
    %   lgospa = step(gospaObj, tracks, truths) returns the GOSPA
    %   track-metric between the set of tracks and truth objects. It
    %   includes a switching penalty for tracks, whose value depends on the
    %   property SwitchingPenalty. It uses the global nearest neighbor
    %   (GNN) assignment at current and last step to compute the metric and
    %   switching penalty.
    %
    %   [lgospa, gospa, switching] = step(gospaObj, tracks, truths) also
    %   returns the two components which contribute to lgospa metric.
    %   lgospa is a p-th order sum of gospa and switching.
    %
    %   lgospa = (gospa^p + switching^p)^(1/p);
    %
    %   For using non-custom distance, the tracks and truths must satisfy
    %   the following: tracks must be an array of structs with fields or an
    %   object with properties:
    %       "State", "StateCovariance"
    %   truths must be an array of structs with fields or an object with
    %   properties:
    %       "Position", "Velocity"
    %
    %   [...] = step(gospaObj, tracks, truths, assignments) allows
    %   specifying the current assignment between tracks and truths. This
    %   syntax can be used when HasAssignment property of the object is set
    %   to true. assignments must be a N-by-2 array of integers, where the
    %   first column denotes the track identities and the second column
    %   denotes the truth identities.
    %
    %   [lgospa, gospa, switching, localization, missTarget, falseTracks] =
    %   step(...) also returns the different components of the gospa
    %   metric. These outputs are only returned when the Alpha property of
    %   the object is set to 2. When alpha is equal to 2, the gospa metric
    %   can be interpreted as a p-th ordered sum of the components:
    %
    %   gospa =(localization^p + missTarget^p + falseTracks^p)^1/p
    %
    %   System objects may be called directly like a function instead of using
    %   the step method. For example, y = step(obj) and y = obj() are
    %   equivalent.
    %
    %   % EXAMPLE: Examine the results of a system that tracked two targets
    %
    %   % load pre-recorded data
    %   load trackmetricex tracklog truthlog
    %
    %   % construct an object to calculate GOSPA metric
    %   tgm = TRACKGOSPAMETRIC('SwitchingPenalty',5);
    %
    %   % Create output variables
    %   lgospa = zeros(numel(tracklog),1);
    %   gospa = zeros(numel(tracklog),1);
    %   switching = zeros(numel(tracklog),1);
    %   localization = zeros(numel(tracklog),1);
    %   missTarget = zeros(numel(tracklog),1);
    %   falseTracks = zeros(numel(tracklog),1);
    %
    %   for i = 1:numel(tracklog)
    %       % extract the tracks and ground truth at ith step
    %       tracks = tracklog{i};
    %       truths = truthlog{i};
    %       [lgospa(i), gospa(i), switching(i), localization(i), missTarget(i), falseTracks(i)] = tgm(tracks, truths);
    %   end
    %
    %   % Plot output
    %   plot([lgospa gospa switching localization missTarget falseTracks]);
    %   legend('Labeled GOSPA', 'GOSPA', 'Switching Component', 'Localization Component', 'Missed Target Component', 'False Tracks Component');
    %
    %   See also: trackOSPAMetric, trackErrorMetrics,
    %   trackAssignmentMetrics, constvel, constacc, constturn
    
    % References:
    % [1] Rahmathullah, Abu Sajana, Ángel F. García-Fernández,
    % and Lennart Svensson. "Generalized optimal sub-pattern assignment
    % metric." 2017 20th International Conference on Information Fusion
    % (Fusion). IEEE, 2017.
    %
    % [2] Rahmathullah, Abu Sajana, Ángel F. García-Fernández, and Lennart
    % Svensson. "A metric on the space of finite sets of trajectories for
    % evaluation of multi-target tracking algorithms." arXiv preprint
    % arXiv:1605.01177 (2016).
    
    %   Copyright 2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Nontunable)
        % HasAssignmentInput A flag to enable assignment input to the step
        % method. When assignment input is provided, the gospa metric is
        % computed using the provided assignment between tracks and truths.
        % When assignment input is not provided, the metric is computed
        % using global nearest neighbor assignment between tracks and
        % truths.
        %
        % Default: false
        HasAssignmentInput = false;
    end
    
    properties (Nontunable)
        % Alpha Alpha for the GOSPA metric specified in the range (0 2]
        % The value of alpha determines the penalty assigned to the
        % cardinality mismatch between tracks and truth. Larger values of
        % alpha reduce the penalty. When alpha is equal to 1, the GOSPA
        % metric can be interpreted as unnormalized OSPA metric. When alpha
        % is equal to 2, the GOSPA metric can be interpreted as an ordered
        % sum of localization, missed targets and false tracks component.
        %
        % Default: 2
        Alpha = 2;
    end
    
    properties (Nontunable)
        % SwitchingPenalty Penalty value for switching of truth assignments
        % If a truth switches assignments at current step as compared to
        % the previous step, a penalty is added to the metric. If the truth
        % switches from being unassigned to assigned or vice-versa, a
        % half-switch penalty, 1/2*(SwitchingPenalty^p), is added. If a
        % truth switches assignment from one assigned track to another, a
        % full-switch penalty, SwitchingPenalty^p, is added. The switching
        % penalty is calculated by comparing assignments at the current
        % step and last step. If HasAssignmentInput property is set to
        % true, the assignments used for penalizing switching are current
        % assignment input and the assignment input from the last step. If
        % HasAssignmentInput property is set to false, the assignments used
        % for penalizing switching is the current global nearest
        % neighbor(GNN) assignment between tracks and truths and the GNN
        % assignment from the last step.
        %
        % Default: 0
        SwitchingPenalty = 0;
    end
    
    properties (Nontunable, Access = {?trackGOSPAMetric,?matlab.unittest.TestCase})
        pReturnComponents
    end
    
    methods
        function obj = trackGOSPAMetric(varargin)
            % obj = trackOSPAMetrics('Name', value)
            setProperties(obj, numel(varargin), varargin{:});
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj,tracks, truths)
            setupImpl@fusion.internal.metrics.OSPABase(obj, tracks, truths);
        end
        
        function [lgospa, gospa, switching, loc, missTruth, falseTracks] ...
                = stepImpl(obj, tracks, truths, varargin)
            % Meaningful error if number of outputs greater than 3 and
            % object cannot return components
            coder.internal.assert(nargout <=3 || obj.pReturnComponents,'fusion:trackGOSPAMetric:AlphaNotTwo');
            
            % Validate inputs
            validateOSPAInputs(obj, tracks, truths, varargin{:});
            
            % Retrieve GOSPA parameters from properties
            % a. m and n value for GOSPA computation
            numTracks = numel(tracks);
            numTruths = numel(truths);
            n = max(numTracks, numTruths);
            m = min(numTracks, numTruths);
            
            % b. order and cut-off distance
            p = obj.Order;
            c = obj.CutoffDistance;
            
            % c. alpha
            alpha = obj.Alpha;
            
            % Compute distance matrix
            dMatrix = getDistanceMatrix(obj, tracks, truths, false);
            
            % Assignment of track indices to truth indices
            indexedAssignment = getCurrentAssociations(obj, dMatrix, tracks, truths, varargin{:});
            
            % Localization OSPA
            ind = sub2ind([numTracks numTruths],indexedAssignment(:,1),indexedAssignment(:,2));
            dMatrixCol = dMatrix(:);
            locOspa = sum(dMatrixCol(ind))^(1/p);
            
            % Number of assignments
            numAssignments = size(indexedAssignment,1);
            
            % mismatch m GOSPA
            misM = (c^p/alpha*((alpha-1)*(m - numAssignments)))^(1/p);
            
            % mismatch n GOSPA
            misN = (c^p/alpha*(n - numAssignments))^(1/p);
            
            % Put together GOSPA
            gospaP = (locOspa^p + misM^p + misN^p); % gospa^p
            gospa = gospaP^(1/p);
            
            % Switching penalty
            switchingPenalty = computeSwitchingPenalty(obj, tracks, truths, indexedAssignment, varargin{:});
            
            lgospa = (gospaP + switchingPenalty^p)^(1/p);
            
            % Assemble outputs
            switching = switchingPenalty;
            
            % Return localization, missed target and false tracks ospa if alpha = 2
            if obj.pReturnComponents
                loc = locOspa;
                if m == numTruths
                    missTruth = misM;
                    falseTracks = misN;
                else
                    missTruth = misN;
                    falseTracks = misM;
                end
            end
        end
        
        function validatePropertiesImpl(obj)
            validatePropertiesImpl@fusion.internal.metrics.OSPABase(obj);
            validateGOSPAProperties(obj);
        end
        
        function validateGOSPAProperties(obj)
            obj.pReturnComponents = obj.Alpha == 2;
        end
        
        function num = getNumInputsImpl(obj)
            num = 2 + obj.HasAssignmentInput;
        end
        
        function groups = getPropertyGroups(~)
            groups = matlab.mixin.util.PropertyGroup(...
                {'CutoffDistance', ...
                'Order',...
                'Alpha',...
                'MotionModel',...
                'Distance',...
                'DistanceFcn',...
                'SwitchingPenalty',...
                'TrackIdentifierFcn',...
                'TruthIdentifierFcn',...
                'HasAssignmentInput'});
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            % Set private and protected properties
            loadObjectImpl@fusion.internal.metrics.OSPABase(obj,s,wasLocked);
            if wasLocked
                obj.pReturnComponents = s.pReturnComponents;
            end
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@fusion.internal.metrics.OSPABase(obj);
            if isLocked(obj)
                s.pReturnComponents = obj.pReturnComponents;
            end
        end
    end
    
    methods
        function set.SwitchingPenalty(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','scalar','nonnegative'}, mfilename,'SwitchingPenalty');
            obj.SwitchingPenalty = val;
        end
        
        function set.Alpha(obj, val)
            validateattributes(val,{'numeric'},{'real','finite','nonsparse','scalar','positive','<=',2},mfilename,'Alpha');
            obj.Alpha = val;
        end
    end
    
    methods (Access = {?trackGOSPAMetric,?matlab.unittest.TestCase})
        function sp = computeSwitchingPenalty(obj, tracks, truths, indexedAssignment, varargin)
            if obj.SwitchingPenalty > 0
                % Get identities of input tracks and truths
                [trackIDs, truthIDs] = getInputIdentities(obj, tracks, truths);
                
                % If assignment input is provided, simply use the input
                % provided by the user
                if obj.HasAssignmentInput
                    identifierAssignment = varargin{1};
                else % Convert GNN Assignment to identifier assignment
                    identifierAssignment = convertIndexToIdentifiers(obj, indexedAssignment, trackIDs, truthIDs);
                end
                
                currAssignment = identifierAssignment;
                
                % Unassigned truths
                unassignedTruths = setdiff(truthIDs,currAssignment(:,2),'stable');
                unassignments = [zeros(numel(unassignedTruths),1,obj.pDataType) unassignedTruths(:)];
                
                curr = [identifierAssignment;unassignments];
                last = obj.pLastAssignment;
                
                % Compute assigned tracks for common truths
                [~,iCur,iLast] = intersect(curr(:,2),last(:,2),'stable');
                
                % Previously assigned track
                prevAssignedTrack = last(iLast,1);
                
                % Currently assigned track
                currAssignedTrack = curr(iCur,1);
                
                % Has truth switched?
                unequalAssignment = prevAssignedTrack ~= currAssignedTrack;
                
                % Was switching from unassigned to track or track to track
                % or track to unassigned
                validPrevTrack = prevAssignedTrack > 0;
                validCurrTrack = currAssignedTrack > 0;
                
                numFullSwitches = sum(unequalAssignment & validPrevTrack & validCurrTrack);
                numHalfSwitches = sum(unequalAssignment & (~validPrevTrack | ~validCurrTrack));
                
                sp = obj.SwitchingPenalty*(numFullSwitches + 1/2*numHalfSwitches)^(1/obj.Order);
                
                % Update last assignment
                obj.pLastAssignment = cast(curr(:,1:2),obj.pDataType);
            else
                sp = cast(0,obj.pDataType);
            end
        end
        
        function indexedAssignments = getCurrentAssociations(obj, dMatrix, tracks, truths, varargin)
            if obj.HasAssignmentInput
                % Identifiers of input tracks and truths
                [trackIds, truthIds] = getInputIdentities(obj, tracks, truths);
                
                % This assignment has trackIDs on first column and truth IDs
                % on second column.
                identifierAssignments = varargin{1};
                
                % Convert identifier to indexed assignment
                indexedAssignments = convertIdentifierToIndices(obj, identifierAssignments, trackIds, truthIds);
            else
                % Compute assignments using GNN algorithm.
                indexedAssignments = assignjv(dMatrix, obj.CutoffDistance^obj.Order/2);
            end
        end
    end
end
