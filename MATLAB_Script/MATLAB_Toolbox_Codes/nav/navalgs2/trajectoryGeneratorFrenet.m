classdef trajectoryGeneratorFrenet < nav.algs.internal.InternalAccess
%trajectoryGeneratorFrenet Generate trajectories using reference path
%
%   The trajectoryGeneratorFrenet object generates trajectories using 4th
%   or 5th order polynomials, where each trajectory defines a motion that
%   drives an initial Frenet state to a terminal Frenet state over given 
%   time span.
%
%   Frenet states describe their position, velocity, and acceleration
%   relative to a static reference path, given as [S dS ddS L dL ddL]. S is
%   the arc length along the path and the derivatives, dS and ddS, are
%   relative to time. L is the deviation perpendicular to the reference 
%   path at S, and the derivatives are relative to the arc length.
%
%   CONNECTOR = trajectoryGeneratorFrenet(refPath) creates an object that
%   generates trajectories between initial and terminal states, relative
%   to a referencePathFrenet object. By default, points along generated 
%   trajectories are spaced every 0.1 seconds in time.
%
%   CONNECTOR = trajectoryGeneratorFrenet(___, 'TimeResolution', timeValue)
%   specifies the "TimeResolution" property when creating the object.
%
%   referenceTrajectoryGenerator Properties:
%       TimeResolution      - Discretization time interval
%       ReferencePath      	- Reference path for generated trajectories
%
%   trajectoryGeneratorFrenet Methods:
%       connect             - Connect initial and terminal Frenet states
%       copy                - Creates a deep copy of the object
%       createParallelState - Create states using Frenet and Global params
%
%   Example:
%
%       % Create a set of waypoints.
%       waypoints = [100 100; 150 200; 200 400; 0 0]
%
%       % Generate a reference path from the waypoints.
%       refPath = referencePathFrenet(waypoints);
%
%       % Create a trajectory generator object which generates trajectories
%       % whose points are sampled at 0.2s intervals.
%       connector = trajectoryGeneratorFrenet(refPath,'TimeResolution',0.2)
%
%       % Retrieve the first point along the path.
%       arcLen = 0;
%       pathStart = interpolate(refPath,arcLen);
%
%       % Form an initial boundary condition for trajectory by adding
%       % initial velocity and acceleration to the [x y theta kappa] point
%       % information.
%       vInit = 1;
%       aInit = 0.1;
%       initGlobalState = [pathStart(1:4) vInit aInit];
%
%       % Convert global state to Frenet state, [S0 dS0 ddS0 L0 dL0 ddL0].
%       initFrenetState = global2frenet(refPath, initGlobalState);
%       
%       % Define offsets in arclength and lateral deviation.
%       arcLenOffsets = [10; 20; 30];
%       lateralDeviation = [-5; 0; 5];
%       
%       % Construct a set of terminal states which are offset from the
%       % initial Frenet state and have zero acceleration and velocity.
%       termFrenetStates = zeros(3,6)
%       termFrenetStates(:,[1 4]) = initFrenetState([1 4]) + ...
%           [arcLenOffsets lateralDeviation];
%       
%       % Generate trajectories between the initial state and each terminal
%       % state over 1s.
%       timeSpanSingle = 1;
%       frenetTrajSet1 = connect(connector,initFrenetState, ...
%           termFrenetStates,timeSpanSingle)
%       
%       % Generate trajectories with differing time spans.
%       timeSpanMultiple = [1;2;3];
%       frenetTrajSet2 = connect(connector,initFrenetState, ...
%           termFrenetStates,timeSpanMultiple);
%       
%       % Connect different initial states to a single terminal state, each
%       % over a different span of time, and return global trajectories.
%       initFrenetStates = repelem(initFrenetState,6,1);
%       initFrenetStates(2:end,[1 4]) = 10*(rand(5,2)-.5);
%       timeSpans = 2+rand(6,1);
%       [frenetTrajSet3, globalTrajSet3] = connect(connector, ...
%           initFrenetStates,termFrenetStates(3,:),timeSpans);
%
%   See also referencePathFrenet, trajectoryOptimalFrenet

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties (Hidden,Constant)
        %DefaultTimeResolution Default value for TimeResolution
        DefaultTimeResolution = 0.1;
    end

    properties
        %ReferencePath Reference path for generated trajectories
        %
        %   A referencePathFrenet object used to map trajectories generated
        %   between Frenet states into the global coordinate system.
        ReferencePath
        
        %TimeResolution Discretization time interval
        TimeResolution = trajectoryGeneratorFrenet.DefaultTimeResolution;
    end

    methods
        function obj = trajectoryGeneratorFrenet(referencePath, varargin)
        %trajectoryGeneratorFrenet Generates alternate trajectories relative to a reference path
            narginchk(1,3);
            obj.ReferencePath = referencePath;
            
            % Parse optional arguments
            if nargin > 1
                validatestring(varargin{1},{'TimeResolution'},'trajectoryGeneratorFrenet');
                coder.internal.prefer_const(varargin{2});

                % Set optional properties
                obj.TimeResolution = varargin{2};
            end
        end
        
        function [frenetTrajectory, globalTrajectory] = connect(obj, initialState, terminalState, timeSpan)
        %connect Connect initial and terminal Frenet states
        %
        %   FRENETTRAJECTORY = connect(obj,INITIALSTATE,TERMINALSTATE,...
        %       TIMESPAN)
        %   connects initial states to terminal states over a span of time.
        %   INITIALSTATE and TERMINALSTATE are 6-column matrices of Frenet 
        %   states, [S dS ddS L dL ddL], and TIMESPAN is a scalar or vector
        %   of positive time spans over which the initial and terminal states
        %   are connected. 
        %
        %   This function supports generation of 1-to-N, N-to-1, and N-to-N
        %   (pairwise) trajectories based on the number of states and times.
        %   FRENETTRAJECTORY is an Nx1 struct array. Each element contains
        %   Trajectory, an Mx6 matrix of Frenet states, and Time, an Mx1
        %   vector of times at which each states occurs.
        %
        %   To calculate the longitudinal terminal state based on a 
        %   4th-order polynomial while satisfying all other boundary 
        %   conditions, specify the arc length, S, as NaN.
        %   
        %   [___,GLOBALTRAJECTORY] = connect(obj,INITIALSTATE,TERMINALSTATE,TIMESPAN)
        %   returns an optional output, GLOBALTRAJECTORY, an Nx1 struct array. 
        %   Each element contains Time, defined above, and Trajectory, an
        %   Mx6 matrix of global states, [x,y,theta,kappa,speed,acceleration].
        %
        %   Example:
        %         % Generate a reference path from a set of waypoints.
        %         refPath = referencePathFrenet([0 0; 50 20; 0 50; -10 0]);
        % 
        %         % Create a trajectory generator object.
        %         connector = trajectoryGeneratorFrenet(refPath);
        % 
        %         % Generate a 5 second trajectory between the path origin
        %         % and a point 30m down the path.
        %         initState = [ 0 0 0 0 0 0]; % [S ds ddS L dL ddL]
        %         termState = [30 0 0 0 0 0]; % [S ds ddS L dL ddL]
        %         trajFrenet = connect(connector,initState,termState,5);
        % 
        %         % Generate trajectories that traverse the same arc length, 
        %         % but deviate laterally from the reference path.
        %         termStateDeviated = termState + (-3:3)'*[0 0 0 1 0 0];
        %         [trajFrenet,trajGlobal] = connect(connector,initState,termStateDeviated,5);
        % 
        %         % Specify a terminal state with 5m arc length, 10m/s 
        %         % velocity, and 10m longitudinal offset. These boundary 
        %         % conditions are not realistic, and need adjustment in 
        %         % the next steps.
        %         initState = [0 0 0 0 0 0];
        %         unrealTermState = [5 10 0 10 0 0];
        %         [~,unrealTrajGlobal] = connect(connector,initState,unrealTermState,3);
        %
        %         % Display the trajectory and note how the vehicle backs
        %         % up before accelerating to the terminal state. 
        %         show(refPath);
        %         hold on;
        %         axis equal;
        %         plot(unrealTrajGlobal.Trajectory(:,1),unrealTrajGlobal.Trajectory(:,2),'b');
        % 
        %         % Remove the terminal longitudinal constraint.
        %         relaxedTermState = [NaN 10 0 10 0 0];
        %
        %         % Generate a new trajectory.
        %         [~, trajGlobalRelaxed] = connect(connector,initState,relaxedTermState,3);
        %         plot(trajGlobalRelaxed.Trajectory(:,1),trajGlobalRelaxed.Trajectory(:,2),'g');
        %         legend({'Waypoints','ReferencePath','Unrealistic Trajectory', ...
        %           'Relaxed 4th order trajectory'})
        %
        %       See also referencePathFrenet, trajectoryGeneratorFrenet
            
            narginchk(4,4);

            % Organize inputs into two sets of N-row state pairs
            [f0,f1,tF] = obj.parseTrajInputs(initialState,terminalState,timeSpan);
            
            % Generate trajectories in Frenet coordinates
            frenetTrajectory = obj.connectPairs(f0, f1, obj.TimeResolution, tF);
            
            if nargout == 2
                % Convert trajectories to global coordinates if requested
                globalTrajectory = frenetTrajectory;
                for i = 1:numel(frenetTrajectory)
                    globalTrajectory(i).Trajectory = obj.ReferencePath.frenet2global(frenetTrajectory(i).Trajectory);
                end
            end
        end
        
        function [globalState,frenetState,lateralTimeDerivatives] = ...
                createParallelState(obj,S,L,v,a,invertHeading)
        %createParallelState Create states using Frenet and Global params
        %
        %   [GLOBALSTATE,FRENETSTATE,LATERALTIMEDERIVATIVES] = ...
        %   createParallelState(PATH,S,L,V,A) takes arclength, lateral 
        %   deviation, body velocity and body acceleration (as N-by-1
        %   column vectors S, L, V, A, respectively), and returns global 
        %   and frenet states (as N-by-6 matrices GLOBALSTATE, FRENETSTATE,
        %   respectively) and the lateral time derivatives of the frenet 
        %   states (as N-by-3 matrix LATERALTIMEDERIVATIVES):
        %
        %       globalState = [x, y, theta, kappa, V, A]
        %
        %       frenetState = [S, dS/dt, ddS/dt^2, L, dL/dS, ddL/dS^2]
        %
        %       lateralTimeDerivatives = [dL/dt, ddL/dt^2, invertHeading]
        %
        %   The position of the state is determined by the distance from 
        %   the path origin along the curve (S) and the lateral deviation 
        %   (L) along the path's normal-vector at S. The first and second 
        %   derivatives are derived by transforming the global velocity and
        %   acceleration (V,A) to the Frenet frame located at S along the 
        %   path.
        %   
        %   By default, the orientation (theta) of the global state is 
        %   assumed to be equal to the path's tangent-angle at S.
        %
        %   [___] = createParallelState(PATH,S,L,V,A,INVERTHEADING) 
        %   Optionally accepts INVERTHEADING, an N-element column vector of
        %   binary-valued inputs, with (0) being default. A value of (1) 
        %   indicates that the state heading should be inverted.
        %
        %   See also referencePathFrenet.global2frenet

            % Validate inputs
            narginchk(5,6);
            validateattributes(S,{'numeric'},{'finite','column'},'createParallelState','s');
            n = numel(S);
            validateattributes(L,{'numeric'},{'finite','column','numel',n},'createParallelState','l');
            validateattributes(v,{'numeric'},{'finite','column','numel',n},'createParallelState','v');
            validateattributes(a,{'numeric'},{'finite','column','numel',n},'createParallelState','a');
            if nargin == 5
                invertHeading = false(n,1);
            else
                validateattributes(invertHeading,{'numeric','logical'},{'binary','column','numel',n},'createParallelState','invertHeading');
            end

            % Construct a set of Frenet states using the provided S,L and with
            % dummy longitudinal velocity/acceleration.
            frenetState = zeros(n,6);
            frenetState(:,1) = S;
            frenetState(:,4) = L;
            frenetState(:,2) = 1;
            
            % Convert dummy states to global
            globalState = obj.ReferencePath.frenet2global(frenetState);

            % Invert heading and curvature
            globalState(invertHeading,3) = robotics.internal.wrapToPi(globalState(invertHeading,3)+pi);
            globalState(invertHeading,4) = -globalState(invertHeading,4);

            % Populate velocity/acceleration
            globalState(:,5) = v;
            globalState(:,6) = a;

            % Convert back to Frenet
            [frenetState, lateralTimeDerivatives] = obj.ReferencePath.global2frenet(globalState, frenetState(:,1));
        end
        
        function cObj = copy(obj)
        %copy Creates a deep copy of the object
            cObj = trajectoryGeneratorFrenet(copy(obj.ReferencePath), ...
                'TimeResolution', obj.TimeResolution);
        end
        
        function set.ReferencePath(obj, refPathObj)
        %set.ReferencePath
            validateattributes(refPathObj, {'nav.algs.internal.FrenetReferencePath'}, {'scalar'},'trajectoryGeneratorFrenet','referencePath');
            obj.ReferencePath = refPathObj;
        end
        
        function set.TimeResolution(obj, resolution)
        %set.TimeResolution
            validateattributes(resolution,{'numeric'},{'scalar','real','positive','nonnan','finite'},'trajectoryGeneratorFrenet','TimeResolution');
            obj.TimeResolution = resolution;
        end
    end
    
    methods (Access = ?nav.algs.internal.InternalAccess, Static)
        function trajectories = connectPairs(f0,f1,dt,timeSpan)
        %connectPairs Connect N pairs of start/end states
            % Number of expected trajectories
            numTraj = size(f0,1);

            % Preallocate trajectory variables
            longitudinalTrajectories = repmat({nav.algs.internal.QuinticQuarticTrajectory([0 0 0],[1 1 1],1)},numTraj,1);
            lateralTrajectories = repmat({nav.algs.internal.QuinticQuarticTrajectory([0 0 0],[1 1 1],1)},numTraj,1);

            trajectories = trajectoryGeneratorFrenet.allocateTrajectories(dt,timeSpan);
            
            for trajIdx = 1:numTraj
                % Extract longitudinal boundary conditions
                sV0 = f0(trajIdx,1:3);
                sV1 = f1(trajIdx,1:3);
                tFinal = timeSpan(trajIdx);

                % Fit quintic/quartic polynomial
                longitudinalTrajectories{trajIdx} = ...
                    nav.algs.internal.QuinticQuarticTrajectory(sV0,sV1,tFinal);

                % Evaluate longitudinal trajectory at evenly spaced
                % intervals wrt time
                t = (dt:dt:dt*(ceil(tFinal/dt)+1))'-dt;
                sV = longitudinalTrajectories{trajIdx}.evaluate([0 1 2], t)';

                % Extract longitudinal boundary conditions
                dV0 = f0(trajIdx,4:6);
                dV1 = f1(trajIdx,4:6);
                dsMax = sV(end,1)-sV(1);
                
                % Fit quintic/quartic polynomial
                lateralTrajectories{trajIdx} = ...
                    nav.algs.internal.QuinticQuarticTrajectory(dV0,dV1,dsMax);

                % Evaluate lateral trajectory based on arclength
                ds = sV(:,1)-sV(1);
                dV = lateralTrajectories{trajIdx}.evaluate([0 1 2], ds)';

                % Combine and populate output struct
                trajectories(trajIdx).Trajectory = reshape([sV dV],[],6);
                trajectories(trajIdx).Times = t;
            end
        end
        
        function trajectories = allocateTrajectories(dt,timeSpan)
            % Find max timespan and number of trajectories
            coder.internal.prefer_const(timeSpan);
            maxTime = max(timeSpan);
            numTraj = numel(timeSpan);

            % Convert max time to max number of steps
            maxSteps = int32(ceil(maxTime/dt)+1);
            
            % Allocate placeholder values and apply upper bounds
            if coder.target('MATLAB')
            % Varsize on by default
                traj = [];
                t = [];
                trajectories = repmat(struct('Trajectory',traj,'Times',t), numTraj, 1);
            else
                if coder.internal.isConst(timeSpan)
                % Entire dimensions of output struct will be compile-time 
                % constant if nTraj is constant.
                    nTraj = numel(timeSpan);
                    trajs = cell(nTraj,1);
                    times = cell(nTraj,1);
                    coder.unroll(coder.internal.isConst(nTraj))
                    for i = 1:nTraj
                        n = ceil(timeSpan(i)/dt)+1;
                        trajs{i} = zeros(n,6);
                        times{i} = zeros(n,1);
                    end
                    trajectories = struct('Trajectory',trajs,'Times',times);
                else
                % Output trajectory dimensions are controlled by the user's
                % inputs. If DynamicMemoryAllocation is turned off, the
                % timeSpan and nTraj inputs provided by the user must be
                % compile-time constant or have known upper limits.
                    traj = [];
                    t = [];
                    if coder.internal.isConst(maxSteps)
                        coder.varsize('traj',[maxSteps, 6],[1 0]);
                        coder.varsize('t',[maxSteps, 1],[1 0]);
                    else
                        coder.varsize('traj',[inf, 6],[1 0]);
                        coder.varsize('t',[inf, 1],[1 0]);
                    end
                    trajectories = repmat(struct('Trajectory',traj,'Times',t), numTraj, 1);
                end
            end
        end

        function [x0,x1,tF] = parseTrajInputs(initState, termState, timeSpan)
        %parseTrajInputs Parses and organizes the inputs into two sets of N-row state pairs

            % Lock down size and span information immediately
            coder.internal.prefer_const(timeSpan);
            numInit = size(initState,1);
            numTerm = size(termState,1);
            numTime = numel(timeSpan);
            numTraj = max([numInit, numTerm, numTime]);

            % Validate inputs
            trajectoryGeneratorFrenet.validateConnect(initState, termState, timeSpan);

            % Attempt to expand inputs
            x0 = trajectoryGeneratorFrenet.expandInput(initState,numTraj,'States');
            x1 = trajectoryGeneratorFrenet.expandInput(termState,numTraj,'States');
            tF = trajectoryGeneratorFrenet.expandInput(timeSpan(:),numTraj,'Times');
        end
        
        function expandedInput = expandInput(input,numTraj,varType)
            n = size(input,1);
            coder.internal.assert(any(n == [1 numTraj]), ['nav:navalgs:trajectorygeneratorfrenet:MismatchedNum' varType]);
            if coder.internal.isConstTrue(n ~= 1)
            % This input cannot be resized, as it is already non-scalar and
            % has passed the validation size check.
                expandedInput = input;
            else
                expandedInput = repmat(input,numTraj/n,1);
            end
        end

        function validateConnect(initState, termState, time)
        %validateConnect Validate type/size of inputs to connect method
        %
        %   The first column of terminal state is allowed to be nan, 
        %   reducing boundary constraints by 1 and allowing us to use 4th
        %   order polynomial to satisfy velocity, acceleration, and initial
        %   position.
            
            % Verify attributes of initial state
            validateattributes(initState,{'numeric'},{'size',[nan 6],'finite','nonnan'},'connect','initState');
            
            % Verify size of terminal states
            assert(size(termState,2) == 6);
            % Verify terminal states form valid 4th/5th order boundary conditions
            validateattributes(termState(:,2:end),{'numeric'},{'nonnan','finite'},'connect','termState');
            
            % Verify the times are valid
            validateattributes(time,{'numeric'},{'finite','nonempty','vector','positive'},'connect','time');
        end
    end

    methods (Static, Hidden)
        function result = matlabCodegenSoftNontunableProperties(~)
        %matlabCodegenSoftNontunableProperties Mark properties as nontunable during codegen
        %
        % Marking properties as 'Nontunable' indicates to Coder that
        % the property should be made compile-time Constant.
            result = {'TimeResolution'};
        end
    end
end
