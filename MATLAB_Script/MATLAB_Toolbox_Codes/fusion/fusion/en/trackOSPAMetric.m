classdef trackOSPAMetric< fusion.internal.metrics.OSPABase
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
%   <a href="matlab:help matlab.System/reset   ">reset</a>           - Resets states of the trackOSPAMetric
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

    methods
        function out=trackOSPAMetric
            % obj = trackOSPAMetrics('Name', value)
        end

        function out=getCurrentAssignment(~) %#ok<STOUT>
            % Get identities of tracks and truths
        end

        function out=getKnownAssignment(~) %#ok<STOUT>
            % Get assignment from input or from last stored assignment
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=getWeights(~) %#ok<STOUT>
        end

        function out=isInactiveOSPA2Property(~) %#ok<STOUT>
        end

        function out=isInactiveOSPAProperty(~) %#ok<STOUT>
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
        end

        function out=setCustomWeights(~) %#ok<STOUT>
        end

        function out=setupImpl(~) %#ok<STOUT>
            % Setup data type of the object
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

        function out=validateOSPA2Properties(~) %#ok<STOUT>
            % Validate OSPA2 properties
        end

        function out=validateOSPAProperties(~) %#ok<STOUT>
            % Validate labeling error is less than cut-off
        end

        function out=validatePropertiesImpl(~) %#ok<STOUT>
            % Validate interdependent properties
        end

    end
    properties
        % CustomWindowWeights Specify a vector of size WindowLength to
        % represent custom weights in the window. 
        % 
        % This property is active when WindowWeights is specified as
        % "custom"
        % 
        % Default: []
        CustomWindowWeights;

        % HasAssignmentInput A flag to enable assignment input to the step
        % When assignment input is provided, the input is compared with the
        % global nearest neighbor assignment (GNN) between current existing
        % tracks and truths to compute the labeling error component.
        % If HasAssignmentInput is false, the GNN assignment from last step
        % is used to compute the labeling error component.
        %
        % This property is only active when Metric is specified as 'OSPA'
        % Default: false
        HasAssignmentInput;

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
        LabelingError;

        % Metric Choice of metric 
        % Specify Metric as 'OSPA' or 'OSPA(2)'. When Metric is set to
        % 'OSPA', the object computes the traditional OSPA metric. When
        % Metric is set to 'OSPA(2)', the object computes the OSPA(2)
        % metric. 
        % 
        % Default: 'OSPA'
        Metric;

        % WindowLength Window length for OSPA(2) metric
        % Specify a positive integer to specify the window length for
        % OSPA(2) metric.
        %
        % This property is active when Metric property is set to OSPA(2)
        %
        % Default: 100
        WindowLength;

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
        WindowSumOrder;

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
        WindowWeightExponent;

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
        WindowWeights;

        pDistanceCalculator;

    end
end
