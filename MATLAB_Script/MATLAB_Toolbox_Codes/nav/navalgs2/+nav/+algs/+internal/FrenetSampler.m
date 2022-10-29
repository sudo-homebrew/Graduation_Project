classdef FrenetSampler < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%FrenetSampler Sample trajectories from start to terminal states
%   FrenetSampler Sample lateral and longitudinal trajectories in Frenet
%   frame along the reference path and combine the trajectories in Cartesian
%   frame
%
%   FrenetSampler methods:
%       FrenetSampler                   - Constructor which takes in
%                                         reference path, terminal states
%                                         and weights for cost functions
%
%       generateTrajectories            - Generate trajectories from the
%                                         given Frenet state to all the
%                                         terminal states

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen
    properties
        %TimeResolution - Discretization time interval
        %   Interval at which time is incremented when discretizing the
        %   time parameterized frenet states
        TimeResolution

        %TerminalStates - Final states for trajectories
        %   PathPoints to which quintic polynomial trajectories are
        %   generated. Structure array with fields Longitudinal, Lateral
        %   and Time which are N-vector while fields Speed and Acceleration
        %   are scalar
        TerminalStates

        %ReferencePathObject - ReferencePath object used for transformation
        %   Reference path from which root point is extracted which is used
        %   for converting for the Frenet states and Cartesian states
        ReferencePathObject

        %CostFunction - User defined cost function
        %   Cost function which is evaluated on all the candidate
        %   trajectories and added to the total cost
        CostFunction

        %Weights - Coefficients multiplied to the costs
        %   A structure array containing weights which gets multiplied to
        %   the evaluated costs on each candidate trajectory and added
        %   together to compute the total cost
        Weights

        %NumSegments Longitudinal terminal state partitions
        %   Number of partitions of the longitudinal terminal states. This
        %   is used for generating intermediate longitudinal terminal
        %   states to which all lateral terminal states are combined with
        %   for generating more motion primitives to each terminal state.
        %   For example, setting the property to 2 creates two partitions
        %   between each longitudinal terminal state. Trajectories are
        %   generated to reach the intermediate longitudinal states with
        %   all the given lateral terminal states.
        %   Default: 1
        NumSegments = 1

        %DeviationOffset Offset of deviation from reference path
        %   Offset for the deviation from the reference path in the lateral
        %   direction. In the Frenet coordinate system, negative values
        %   indicate a deviation to the right (positive = left). Set this
        %   property to bias your solution to a certain turn direction when
        %   avoiding obstacles in the reference path.
        %   Default: 0
        DeviationOffset = 0
    end

    methods
        function obj = FrenetSampler(referencePathObject, terminalStates, weights, varargin)
        %FrenetSampler Constructor to construct Frenet sampler object
        %   FrenetSampler class object is used to compute trajectories
        %   between the given initial states and the terminal states

        % Validate all the input arguments
            validateattributes(referencePathObject, {'nav.algs.internal.FrenetReferencePath'},{'nonempty','scalar'},'FrenetSampler','ReferencePathObject');

            % Assign values to object properties
            obj.ReferencePathObject = referencePathObject;

            parser = robotics.core.internal.NameValueParser({'CostFunction','TimeResolution','NumSegments','DeviationOffset'}, {[],0.1,1,0});
            parse(parser, varargin{:});

            obj.TimeResolution = parameterValue(parser, 'TimeResolution');
            obj.TerminalStates = terminalStates;
            obj.Weights = weights;

            if(~isempty(parameterValue(parser, 'CostFunction')))
                obj.CostFunction = parameterValue(parser, 'CostFunction');
            end

            obj.NumSegments = parameterValue(parser, 'NumSegments');
            obj.DeviationOffset = parameterValue(parser, 'DeviationOffset');
        end

        function combinedTrajectories = generateTrajectories(obj, initFrenetStates)
        %generateTrajectories Generate trajectories from initFrenetStates
        %   Generates discretized Cartesian state trajectories from the
        %   given initFrenetStates to all the terminal states

            robotics.internal.validation.validateTrajectoryState(initFrenetStates,...
                                                              'generateTrajectories','initFrenetStates');

            % Convert input initFrenetStates to Frenet frame
            frenetStates = initFrenetStates';

            numlongitudinalTrajectories = size(obj.TerminalStates.Longitudinal,2) * size(obj.TerminalStates.Time,2);

            numLateralStates = size(obj.TerminalStates.Lateral,2);

            % From each N lateral states there will be N polynomials to all
            % other N intermediate lateral states
            numIntermediateLateralTrajectories = (numLateralStates + double(obj.NumSegments - 1 > 0)*numLateralStates^2);

            numlateralTrajectories = numIntermediateLateralTrajectories * numlongitudinalTrajectories;
            numTimeStates = size(obj.TerminalStates.Time,2);

            % Preallocate trajectory variables
            longitudinalTrajectories = repmat({nav.algs.internal.QuinticQuarticTrajectory([0 0 0],[1 1 1],1)},numlongitudinalTrajectories,1);
            lateralTrajectories = repmat({nav.algs.internal.QuinticQuarticTrajectory([0 0 0],[1 1 1],1)},numlateralTrajectories,1);

            % Iterate through all the longitudinal and time terminal states
            for longitudinalIndex = 1:size(obj.TerminalStates.Longitudinal,2)
                for timeIndex = 1:numTimeStates
                    % Compute time parameterized longitudinal trajectory
                    trajectoryIndex = timeIndex + (longitudinalIndex-1) * size(obj.TerminalStates.Time,2);
                    longitudinalTrajectories{trajectoryIndex} = ...
                        nav.algs.internal.QuinticQuarticTrajectory(frenetStates(1:3)', ...
                                                                   [obj.TerminalStates.Longitudinal(longitudinalIndex) + frenetStates(1); ...
                                        obj.TerminalStates.Speed; obj.TerminalStates.Acceleration]', ...
                                                                   obj.TerminalStates.Time(timeIndex));
                end
            end

            % For each initial state there are N goal states (both terminal
            % or intermediate) hence the total lateral trajectories are
            % product of N lateral state done number of cascade times plus
            % N trajectories from initial frenet state. i.e. N*N*N... C+1
            % times where C is NumSegments.
            numLateralTrajectories = numLateralStates^(obj.NumSegments);

            % Combining each lateral state with each longitudinal
            % trajectory and form a mapping of all such trajectories
            lateralPairs = zeros(numLateralTrajectories*numlongitudinalTrajectories,obj.NumSegments);

            for longTrajIndex = 1:numlongitudinalTrajectories

                longTraj = longitudinalTrajectories{longTrajIndex};
                sMax = longTraj.evaluate(0,longTraj.Variable) - frenetStates(1);
                sLength =  sMax/(obj.NumSegments);

                % Trajectories from start state to each intermediate state
                % or terminal state (total N)
                for i = 1:numLateralStates
                    lateralIdx = i + (longTrajIndex-1)*numIntermediateLateralTrajectories;
                    lateralTrajectories{lateralIdx} = nav.algs.internal.QuinticQuarticTrajectory(frenetStates(4:6)',[obj.TerminalStates.Lateral(i),0,0],sLength);
                end

                % Trajectories between intermediate states (total N^2)
                if numIntermediateLateralTrajectories ~= numLateralStates
                    % If num of intermediate trajectories are not equal to
                    % the num of lateral states then generate
                    % trajectories between intermediate lateral states.
                    for i = 1:numLateralStates
                        for j = 1:numLateralStates
                            lateralIdx = i*numLateralStates + j + (longTrajIndex-1)*numIntermediateLateralTrajectories;
                            lateralTrajectories{lateralIdx} = nav.algs.internal.QuinticQuarticTrajectory([obj.TerminalStates.Lateral(i),0,0],[obj.TerminalStates.Lateral(j),0,0],sLength);
                        end
                    end
                end

                % Combine all the lateral polynomials (N + N^2) as a
                % mapping required for forming trajectories between start
                % to terminal states
                for i=1:obj.NumSegments
                    if i==1
                        lateralPairs(numLateralTrajectories*(longTrajIndex-1)+1:numLateralTrajectories*longTrajIndex,i) = ...
                            reshape(repmat(1:1:numLateralStates,numLateralTrajectories/numLateralStates,1),[],1) + ...
                            numIntermediateLateralTrajectories*(longTrajIndex-1);
                    else
                        lateralPairs(numLateralTrajectories*(longTrajIndex-1)+1:numLateralTrajectories*longTrajIndex,i) = ...
                            repmat(reshape(repmat(1:1:numLateralStates^2,numLateralTrajectories/(numLateralStates^(i)),1),[],1),...
                                   numLateralStates^(i-2),1) + numLateralStates + numIntermediateLateralTrajectories*(longTrajIndex-1);
                    end
                end
            end

            % Combine lateral trajectory pairs with corresponding
            % longitudinal pairs to form a map containing indices of all
            % the trajectories from start to terminal states. First column
            % corresponds to the longitudinal polynomial indices while
            % subsequent columns corresponds to lateral polynomial indices
            cartesianTrajectoryMapping = [reshape(repmat(1:numlongitudinalTrajectories,numLateralTrajectories,1),[],1),lateralPairs];

            % Combine the lateral and longitudinal trajectories
            combinedTrajectories = obj.combineTrajectories(longitudinalTrajectories, lateralTrajectories, cartesianTrajectoryMapping);
        end

        function set.TerminalStates(obj,TerminalStates)
        %set.TerminalStates Setter for terminal states

            validateattributes(TerminalStates, {'struct'},{'nonempty','scalar'},'FrenetSampler','TerminalStates')

            validateattributes(TerminalStates.Speed, {'numeric'},{'nonempty','finite','real','scalar'},'FrenetSampler','TerminalStates.Speed');

            validateattributes(TerminalStates.Acceleration, {'numeric'},{'nonempty','finite','real','scalar'},'FrenetSampler','TerminalStates.Acceleration');

            coder.varsize('longitudinal','lateral','time')

            longitudinal = TerminalStates.Longitudinal;
            lateral = TerminalStates.Lateral;
            time = TerminalStates.Time;

            obj.TerminalStates = struct('Longitudinal',longitudinal,'Lateral',lateral,'Time',time,'Speed',TerminalStates.Speed,'Acceleration',TerminalStates.Acceleration);
        end

        function set.Weights(obj, Weights)
        %set.Weights Setter for Weights for cost functions
            validateattributes(Weights, {'struct'},{'nonempty','scalar'},'FrenetSampler','Weights')

            % Assign the property after all the fields are validated
            obj.Weights = Weights;
        end

        function set.CostFunction(obj, CostFunction)
        %set.CostFunction Setter for user defined cost function
            validateattributes(CostFunction, {'function_handle'},{'nonempty','scalar'},'FrenetSampler','CostFunction')

            % Check for number of input arguments for the given cost
            % function
            if(nargin(CostFunction) == 1)
                validateattributes(CostFunction(ones(1, 6)), {'numeric'},{'nonempty','finite','real','scalar'},'FrenetSampler','output of CostFunction')
                obj.CostFunction = CostFunction;
            else
                coder.internal.error('nav:navalgs:frenetsampler:CostFunctionIncorrectInputSize');
            end
        end

        function set.TimeResolution(obj, TimeResolution)
        %set.CostFunction Setter for timeResolution discretization

            validateattributes(TimeResolution, {'numeric'},{'finite','real','nonempty','positive'},'FrenetSampler','TimeResolution')
            obj.TimeResolution = TimeResolution;
        end
    end

    methods(Access=private)
        function combinedTrajectories = combineTrajectories(obj, longitudinalTrajectories, lateralTrajectories, cartesianTrajectoryMapping)
        %combineTrajectories Combine the Frenet frame trajectories
        %   Combine the lateral and longitudinal trajectories given in
        %   Frenet frame by discretizing them and then transforming
        %   them into states given in Cartesian frame

            totalCombinedTrajectories = size(cartesianTrajectoryMapping,1);

            combinedTrajectories = repmat(struct(...
                'Trajectory',[], 'Cost',nan, 'MaxAcceleration', inf, ...
                'MaxCurvature',inf,'Feasible',[-1 -1 -1 -1]), ...
                                          totalCombinedTrajectories,1);

            coder.varsize('combinedTrajectories(:).Trajectory');

            for trajectoryIndex = 1:size(cartesianTrajectoryMapping,1)

                currentLongitudinalTrajectory = longitudinalTrajectories{cartesianTrajectoryMapping(trajectoryIndex,1)};

                totalTrajectoryTime = currentLongitudinalTrajectory.Variable;
                timeSteps = linspace(0,totalTrajectoryTime,ceil(totalTrajectoryTime/obj.TimeResolution)+1);
                longitudinalStates = currentLongitudinalTrajectory.evaluate([0 1 2], timeSteps);

                % Check if trajectories exceed reference path (in case of
                % nan longitudinal state) otherwise transformations will
                % become invalid.
                if longitudinalStates(1,end) > obj.ReferencePathObject.Length
                    continue;
                else
                    % Calculate all the required variables from longitudinal
                    % trajectories
                    s = longitudinalStates(1,:);
                    sParam = s - s(1);
                    refPathPoints = obj.ReferencePathObject.interpolate(s');

                    % Longitudinal partition length
                    sLength =  sParam(end)/(size(cartesianTrajectoryMapping,2)-1);

                    % Intermediate longitudinal terminal states for equal
                    % partitions
                    if sLength == 0 || sParam(end) == 0
                        % If both start and end state are zero for velocity
                        % keep
                        sIntermediateIndex = [1,size(sParam,2)];
                    else
                        sIntermediate = 0:sLength:sParam(end);
                        % Compute closest indices corresponding to each
                        % longitudinal partition
                        sIntermediateIndex = ones(size(sIntermediate,2),1);
                        for i = 2:size(sIntermediate,2)
                            sIntermediateIndex(i) = find(sParam(sIntermediateIndex(i)+1:end) >= sIntermediate(i),1,'first') + sIntermediateIndex(i);
                        end
                    end

                    % Calculate lateral states using lateral trajectory
                    % polynomial maps defined in terms of S.
                    lateralStates = zeros(3,size(sParam,2));
                    for lateralSegmentIndex = 2:size(cartesianTrajectoryMapping,2)
                        currentLateralTrajectory = lateralTrajectories{cartesianTrajectoryMapping(trajectoryIndex,lateralSegmentIndex)};
                        sIndexes = sIntermediateIndex(lateralSegmentIndex-1):sIntermediateIndex(lateralSegmentIndex);
                        currentLateralSParam = sParam(sIndexes) - sLength*(lateralSegmentIndex-2);
                        lateralStates(:,sIndexes) = currentLateralTrajectory.evaluate([0 1 2], currentLateralSParam);
                    end

                    % Segregate all lateral and longitudinal Frenet states
                    s = longitudinalStates(1,:);
                    sDot = longitudinalStates(2,:);
                    sDotDot = longitudinalStates(3,:);
                    l = lateralStates(1,:);
                    lPrime = lateralStates(2,:);
                    lPrimePrime = lateralStates(3,:);

                    currentFrenetTrajectory = [s; sDot; sDotDot; l; lPrime; lPrimePrime]';

                    % Threshold the final state, which often contains residual 
                    % floating point error when the terminal state should result in
                    % zero-velocity and/or acceleration. See g2049087
                    m = abs(currentFrenetTrajectory(end,:)) < sqrt(eps);
                    currentFrenetTrajectory(end,m) = 0;

                    % Convert Frenet states to Cartesian
                    cartesianStates = matlabshared.planning.internal.CartesianFrenetConversions.frenet2Cartesian(refPathPoints, currentFrenetTrajectory);

                    currentCartesianTrajectory = [cartesianStates,timeSteps'];

                    % Assign the values to the required fields of combined
                    % trajectories
                    combinedTrajectories(trajectoryIndex).Trajectory = currentCartesianTrajectory;

                    combinedTrajectories(trajectoryIndex).MaxAcceleration = max(abs(currentCartesianTrajectory(:,6)));
                    combinedTrajectories(trajectoryIndex).MaxCurvature = max(abs(currentCartesianTrajectory(:,4)));

                    % Compute cost for the custom cost function
                    if ~isempty(obj.CostFunction)
                        customCost = obj.CostFunction(currentCartesianTrajectory);
                    else
                        customCost = 0;
                    end

                    % Compute the total cost of the combined trajectory
                    cost = nav.algs.internal.FrenetCostFunctions.evaluateTotalCost([currentFrenetTrajectory, timeSteps'], obj.DeviationOffset, obj.Weights) + customCost;

                    % Remove residuals by rounding off to order of sqrt(eps). See g2049087
                    roundOrder = ceil(abs(log(sqrt(eps))/log(10)));
                    cost = round(cost*10^roundOrder)/10^roundOrder;

                    combinedTrajectories(trajectoryIndex).Cost = cost;
                end
            end
        end
    end
end
