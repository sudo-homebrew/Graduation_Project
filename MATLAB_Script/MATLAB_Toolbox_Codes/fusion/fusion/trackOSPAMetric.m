classdef trackOSPAMetric < fusion.internal.metrics.OSPABase
% trackOSPAMetric Optimal Subpattern Assignment Metric (OSPA)
%  ospaObj = trackOSPAMetric returns a System object capable of computing
%  the optimal subpattern assignment metric (OSPA), Labelled OSPA (LOSPA)
%  and OSPA(2) or OSPA-on-OSPA metric. 
% 
%  ospaObj = trackOSPAMetric('Name', value) allows you to specify the
%  properties for the OSPA metric calculation using name-value pairs. See
%  the list of properties below. 
% 
%  To compute OSPA, set Metric to 'OSPA'. 
%  To compute LOSPA, set Metric to 'OSPA' and LabelingError to a positive value.
%  To compute OSPA(2), set Metric property as 'OSPA(2)'.
%
%  trackOSPAMetric properties:
%   Metric                  - Choose between OSPA and OSPA(2) metric
%   CutoffDistance          - Maximum distance between track and truth
%   Order                   - Order of the OSPA metric computation
%   LabelingError           - Penalty added per wrong assignment of
%                             each track to truth
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
%                               'singer'    - singer acceleration
%   DistanceFcn             - A custom function to compute the distance
%                             between a track and a truth object
%   TrackIdentifierFcn      - Handle to a function returning unique
%                             track identifiers
%   TruthIdentifierFcn      - Handle to a function returning unique
%                             truth identifiers
%   HasAssignmentInput      - A flag to determine if current known
%                             assignment is provided to step method
%   WindowLength            - Sliding window length for OSPA(2) metric*
%   WindowSumOrder          - Order of weighted-sum in the window*
%   WindowWeights           - Choice of weighting function*
%   WindowWeightExponent    - Exponent for automatic weight calculation*
%   CustomWindowWeights     - Custom weights in the window*
%
%   *These properties are only used for OSPA(2) metric computation. 
%
%   trackOSPAMetric methods:
%   step            - Computes the metric
%   release         - Allows property value and input characteristics change
%   clone           - Creates a copy of the trackOSPAMetric object
%   isLocked        - Locked status (logical)
%   reset           - Resets states of the trackOSPAMetric
%
%   ospaMetric = step(ospaObj, tracks, truths) returns the OSPA metric
%   between the set of tracks and truth objects. For using non-custom
%   distance, the tracks and truths must satisfy the following:
%   tracks must be an array of structs with fields or an object with
%   properties:
%       "State", "StateCovariance"
%   truths must be an array of structs with fields or an object with
%   properties:
%       "Position", "Velocity"
%
%   ospaMetric = step(ospaObj, tracks, truths, assignment) allows
%   specifying the "known" assignment between tracks and truth at the
%   current time step. For using "default" truth and track identifiers, the
%   tracks must be identified by "TrackID" and truths must be identified by
%   "PlatformID". You must specify HasAssignmentInput property as true to
%   use this signature. This syntax is only available when Metric is
%   specified as "OSPA"
%
%   [.., localizationOSPA] = step(...) also returns the localization
%   component of the metric.
%
%   [..., localizationOSPA, cardinalityOSPA] = step(...) also returns the
%   cardinality component of the metric.
%
%   [..., localizationOSPA, cardinalityOSPA, labelingOSPA] = step(...) also
%   returns the labeling error component of the metric. This syntax is only
%   available when Metric is specified as "OSPA". 
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
%   % construct an object to calculate OSPA metric
%   tom = TRACKOSPAMETRIC;
%
%   % Create output variable
%   ospa = zeros(numel(tracklog),1);
%   cardOspa = zeros(numel(tracklog),1);
%   locOspa = zeros(numel(tracklog),1);
%
%   for i = 1:numel(tracklog)
%       % extract the tracks and ground truth at ith step
%       tracks = tracklog{i};
%       truths = truthlog{i};
%       [ospa(i), locOspa(i), cardOspa(i)] = tom(tracks, truths);
%   end
%
%   % Plot output
%   plot([ospa locOspa cardOspa]);
%   legend('OSPA', 'Localization OSPA', 'Cardinality OSPA');
%
%   See also: trackErrorMetrics, trackAssignmentMetrics, trackGOSPAMetric,
%   constvel, constacc, constturn

%   Copyright 2019 The MathWorks, Inc.
    
    %#codegen   
    properties (Nontunable)
        % LabelingError Penalty for incorrect assignment of tracks to truth
        % Specify a real, finite and scalar value in the range of 0 to
        % CutoffDistance to specify the penalty added for optimal
        % (minimum-distance) of tracks different than the known assignment.
        % The known assignment can be provided as an input to the step
        % method. If the assignment is not provided as an input, the last
        % known assignment is assumed to be correct.
        %
        %   This property is only active when Metric is specified as 'OSPA'
        %
        %   Default: 0
        LabelingError = 0;
    end

    properties (Nontunable)
        % HasAssignmentInput A flag to enable assignment input to the step
        % When assignment input is provided, the input is compared with the
        % global nearest neighbor assignment (GNN) between current existing
        % tracks and truths to compute the labeling error component.
        % If HasAssignmentInput is false, the GNN assignment from last step
        % is used to compute the labeling error component.
        %
        % This property is only active when Metric is specified as 'OSPA'
        % Default: false
        HasAssignmentInput = false;
     end
    
     properties (Nontunable)
         % Metric Choice of metric 
         % Specify Metric as 'OSPA' or 'OSPA(2)'. When Metric is set to
         % 'OSPA', the object computes the traditional OSPA metric. When
         % Metric is set to 'OSPA(2)', the object computes the OSPA(2)
         % metric. 
         % 
         % Default: 'OSPA'
         Metric = 'OSPA'

         % WindowLength Window length for OSPA(2) metric
         % Specify a positive integer to specify the window length for
         % OSPA(2) metric.
         %
         % This property is active when Metric property is set to OSPA(2)
         %
         % Default: 100
         WindowLength = 100

         % WindowSumOrder Order of weighted sum in the history
         % Specify a positive number to represent the order of the distance
         % (q) sum in the window. For example, an order q for a window
         % length N produces the following summation:
         %
         % (w1*d1^q + w2*d2^q ... + wn*dn^q)^(1/q);
         %
         % where wi is the weight and di is the distance between track and
         % truth at ith position in the sliding window.
         %
         % This property is active when Metric property is set to OSPA(2)
         %
         % Default: 2
         WindowSumOrder = 2

         % WindowWeights Choose weighting function in the window
         % Specify WindowWeights as "auto" or "custom". When WindowWeights
         % is specified as "auto", weights are auto-calculated using the
         % WindowWeightExponent property.
         %
         % When WindowWeights is specified as "custom", specify
         % CustomWindowWeights as a vector of length WindowLength to
         % specify weights in the window.
         %
         % This property is active when Metric property is set to OSPA(2) 
         %
         % Default: 'auto'
         WindowWeights = 'auto';

         % WindowWeightExponent Specify a positive number to represent the
         % window weight exponent (r) for automatic weight calculation.
         % When WindowWeights is 'auto', the weights in the window are
         % calculated as:
         %
         %  w(t) = (N - k + t)^r; w(t) = w(t)/sum(w);
         %
         % where t is the time-step in the window ranging from k - N + 1 to
         % k. N is the WindowLength, k is the current time step.
         %
         % A value of r equal to 0 represents equal weights in the window.
         % A higher value of r puts more emphasis on recent data.
         % 
         % This property is active when Metric property is set to OSPA(2)
         % and when WindowWeights is specified as "auto"
         % 
         % Default: 1 
         WindowWeightExponent = 1
     end

     properties
         % CustomWindowWeights Specify a vector of size WindowLength to
         % represent custom weights in the window. 
         % 
         % This property is active when WindowWeights is specified as
         % "custom"
         % 
         % Default: []
         CustomWindowWeights
     end

     properties (Constant, Hidden)
         WindowWeightsSet = matlab.system.StringSet({'auto','custom'});
         MetricSet = matlab.system.StringSet({'OSPA','OSPA(2)'});
     end

     properties (Access = protected)
         pDistanceCalculator
     end

    methods
        function obj = trackOSPAMetric(varargin)
            % obj = trackOSPAMetrics('Name', value)
            setProperties(obj, numel(varargin), varargin{:});
        end
    end
    
    % Setters and getters
    methods        
        function set.LabelingError(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','scalar','nonnegative'}, mfilename,'LabelingError');
            obj.LabelingError = val;
        end

        function set.WindowLength(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','integer','scalar','positive'}, mfilename,'WindowLength');
            obj.WindowLength = val;
        end

        function set.WindowWeightExponent(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','scalar','nonnegative'}, mfilename,'WindowWeightExponent');
            obj.WindowWeightExponent = val;
        end

        function set.WindowSumOrder(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','scalar','positive'}, mfilename,'WindowSumOrder');
            obj.WindowSumOrder = val;
        end

        function set.CustomWindowWeights(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','positive'}, mfilename,'CustomWindowWeights');
            if coder.target('MATLAB')
                setCustomWeights(obj,val);
            else
                coder.internal.defer_inference('setCustomWeights',obj,val);
            end
            obj.CustomWindowWeights = val;
        end
    end

    methods (Access = protected)
        function setCustomWeights(obj, val)
            if isLocked(obj) && coder.internal.is_defined(obj.pDistanceCalculator)
                validateattributes(val,{'single','double'},{'real','finite','nonnegative','numel',obj.WindowLength},mfilename,'CustomWindowWeights')
                obj.pDistanceCalculator.Weights(:) = val;
            end
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj,tracks, truths, varargin)
            % Setup data type of the object
            setupImpl@fusion.internal.metrics.OSPABase(obj, tracks, truths);
            if strcmpi(obj.Metric,'OSPA(2)')
                obj.pDistanceCalculator = createOSPA2BaseDistanceCalculator(obj);
            end
        end
        
        function [totalOspa, locOspa, cardOspa, labelOspa] = stepImpl(obj, tracks, truths, assignments)
            if nargin == 3
                assignments = obj.pLastAssignment;
            end
            
            % Validate inputs
            validateOSPAInputs(obj, tracks, truths, assignments);
            
            % Get localization and cardinality components from base class
            % along with global nearest neighbor assignment results
            [locOspa, cardOspa, optimalAssignment] = stepImpl@fusion.internal.metrics.OSPABase(obj, tracks, truths);
            
            % Labeling OSPA 
            alpha = obj.LabelingError;
            
            % p and n values
            p = obj.Order;
            n = max(numel(tracks),numel(truths));
            
            if alpha > 0 && strcmpi(obj.Metric,'OSPA')
                [kTrackIDs, kTruthIDs] = getKnownAssignment(obj, assignments); % Known assignment
                [oTrackIDs, oTruthIDs] = getCurrentAssignment(obj, tracks, truths, optimalAssignment); % Optimal assignment
                 
                % Number of tracks which switched labels
                [~, ik, iO] = intersect(kTrackIDs, oTrackIDs,'stable');
                kLabels = kTruthIDs(ik);
                oLabels = oTruthIDs(iO);
                numWrong = sum(kLabels ~= oLabels & ~(isnan(kLabels) & isnan(oLabels)));
                if numWrong > 0
                    labelOspa = (1/n*alpha^p*numWrong)^(1/p);
                else
                    labelOspa = cast(0,obj.pDataType);
                end
            else
                labelOspa = cast(0,obj.pDataType);
            end
            
            % Put together OSPA
            totalOspa = (locOspa^p + cardOspa^p + labelOspa^p)^(1/p);
        end
        
        function [trackIDs, truthIDs] = getCurrentAssignment(obj, tracks, truths, optimalAssignment)
            % Get identities of tracks and truths
            inputTrackIDs = getTrackIdentities(obj, tracks);
            inputTruthIDs = getTruthIdentities(obj, truths);
            
            % Get current assignment of trackIDs to truthIDs
            trackIDs = inputTrackIDs(optimalAssignment(:,1));
            truthIDs = inputTruthIDs(optimalAssignment(:,2));
            
            % Store in last assignment
            obj.pLastAssignment = cast([trackIDs truthIDs],obj.pDataType);
        end
        
        function [trackIDs, truthIDs] = getKnownAssignment(obj, varargin)
            % Get assignment from input or from last stored assignment
            if nargin == 2
                assignments = varargin{1};
            else
                assignments = obj.pLastAssignment;
            end
            trackIDs = assignments(:,1);
            truthIDs = assignments(:,2);
        end
        
        function validatePropertiesImpl(obj)
            % Validate interdependent properties
            validatePropertiesImpl@fusion.internal.metrics.OSPABase(obj);
            validateOSPAProperties(obj);
            validateOSPA2Properties(obj);
        end

        function validateOSPAProperties(obj)                        
            % Validate labeling error is less than cut-off
            if strcmpi(obj.Metric,'OSPA')
                coder.internal.assert(obj.LabelingError <= obj.CutoffDistance,'fusion:trackOSPAMetric:labelingErrorLessThanCutoff');
            end
        end

        function validateOSPA2Properties(obj)
            % Validate OSPA2 properties
            if strcmpi(obj.Metric,'OSPA(2)')
                if strcmpi(obj.WindowWeights,'custom')
                    validateattributes(obj.CustomWindowWeights,{'single','double'},{'real','finite','nonnegative','numel',obj.WindowLength},mfilename,'CustomWindowWeights');
                end
            end
        end

        function tf = isInactivePropertyImpl(obj, prop)
            tf = isInactivePropertyImpl@fusion.internal.metrics.OSPABase(obj, prop);
            tf = tf || isInactiveOSPAProperty(obj,prop);
            tf = tf || isInactiveOSPA2Property(obj,prop);
        end

        function tf = isInactiveOSPAProperty(obj, prop)
            tf = strcmpi(obj.Metric,'OSPA(2)') && any(strcmpi(prop,{'HasAssignmentInput','LabelingError'}));
        end

        function tf = isInactiveOSPA2Property(obj,prop)
            tf = strcmpi(obj.Metric,'OSPA') && any(strcmpi(prop,{'WindowLength','WindowSumOrder','WindowWeightExponent','WindowWeights','CustomWindowWeights'}));
            tf = tf || strcmpi(obj.WindowWeights,'auto') && strcmpi(prop,'CustomWindowWeights');
            tf = tf || strcmpi(obj.WindowWeights,'custom') && strcmpi(prop,'WindowWeightExponent');
        end

        function out = getNumInputsImpl(obj)
            out = 2 + (strcmpi(obj.Metric,'OSPA') && obj.HasAssignmentInput);
        end

        function out = getNumOutputsImpl(obj)
            out = 3 + strcmpi(obj.Metric,'OSPA');
        end

        function w = getWeights(obj)
            if strcmpi(obj.WindowWeights,'auto')
                % Avoid overflow when calculating x^r/sum(x^r) when r is
                % large
                x = 1:obj.WindowLength;
                r = obj.WindowWeightExponent;
                logxr = r*log(x);
                logsumxr = fusion.internal.logsumexp(logxr);
                w = exp(logxr - logsumxr);
            else
                w = obj.CustomWindowWeights(:)';
                w = w/sum(w);
            end
        end
    end

    methods (Access = {?fusion.internal.metrics.OSPABase,?matlab.unittest.TestCase})
        function d = getDistanceMatrix(obj, tracks, truths)
            if strcmpi(obj.Metric,'OSPA(2)')
                d = obj.pDistanceCalculator(tracks, truths);
                % Raise to the order
                d = d.^(obj.Order);
            else
                d = getDistanceMatrix@fusion.internal.metrics.OSPABase(obj, tracks, truths);
            end
        end

        function calculator = createOSPA2BaseDistanceCalculator(obj)
            w = getWeights(obj);
            calculator = fusion.internal.metrics.OSPA2BaseDistanceCalculator('WindowLength',obj.WindowLength,...
                'CutoffDistance',obj.CutoffDistance,...
                'WindowSumOrder',obj.WindowSumOrder,...
                'Weights',w,...
                'TrackIdentifierFcn',obj.pTrackIdentifierFcn,...
                'TruthIdentifierFcn',obj.pTruthIdentifierFcn,...
                'DistanceFcn',obj.pDistanceFcn);
        end
    end
    
    %% Save/load/reset/release methods
    methods (Access = protected)       
        function groups = getPropertyGroups(~)
           ospaGroup = matlab.mixin.util.PropertyGroup(...
                {'Metric',...
                'CutoffDistance', ...
                'Order',...
                'MotionModel',...
                'Distance',...
                'DistanceFcn',...
                'LabelingError',...
                'TrackIdentifierFcn',...
                'TruthIdentifierFcn',...
                'HasAssignmentInput'});

           windowGroup = matlab.mixin.util.PropertyGroup(...
                {'WindowLength',...
                'WindowSumOrder',...
                'WindowWeights',...
                'CustomWindowWeights',...
                'WindowWeightExponent'});

           groups = [ospaGroup windowGroup];
        end

        function releaseImpl(obj)
            releaseImpl@fusion.internal.metrics.OSPABase(obj);
            if strcmpi(obj.Metric,'OSPA(2)')
                release(obj.pDistanceCalculator);
            end
        end

        function resetImpl(obj)
            resetImpl@fusion.internal.metrics.OSPABase(obj);
            if strcmpi(obj.Metric,'OSPA(2)')
                reset(obj.pDistanceCalculator);
            end
        end

        function s = saveObjectImpl(obj)
            s = saveObjectImpl@fusion.internal.metrics.OSPABase(obj);
            if isLocked(obj)
                s.pDistanceCalculator = obj.pDistanceCalculator;
            end
        end

        function loadObjectImpl(obj, s, wasLocked)
            loadObjectImpl@fusion.internal.metrics.OSPABase(obj, s, wasLocked);
            if wasLocked
                if isfield(s,'pDistanceCalculator')
                    obj.pDistanceCalculator = s.pDistanceCalculator;
                end
            end
        end
    end
end
