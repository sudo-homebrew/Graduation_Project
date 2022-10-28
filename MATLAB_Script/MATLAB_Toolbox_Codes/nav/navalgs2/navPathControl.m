classdef navPathControl < nav.algs.internal.InternalAccess
%navPathControl Path representing control-based kinematic trajectory
%
%   The navPathControl object stores paths that are typically created by
%   control-based path planners. Paths are represented by a sequence of
%   states, controls, durations, and targets. Paths represented by a 
%   navPathControl are associated with a specific state propagator. 
%
%   States and targets in the path belong to the propagator's state space.
%   Controls are the outputs of a controller which are used to update the
%   propagators kinematic system. Each control is applied for an associated
%   duration.
%
%   PATHOBJ = navPathControl(PROPAGATOR) creates a path object with the
%   given state propagator, PROPAGATOR, specified as an object.
%
%   PATHOBJ = navPathControl(PROPAGATOR, STATES, CONTROLS, TARGETS,
%   DURATIONS) initializes the path with a sequence of given STATES,
%   CONTROLS, TARGETS and DURATIONS. The number of STATES must be one
%   greater than the number of subsequent inputs.
%
%   Properties for class navPathControl:
% 
%      StatePropagator	- State propagator for path
%      States           - Series of states for path
%      Controls         - Series of controls for path
%      Durations        - Series of durations for path
%      TargetStates     - Series of targets for path
%      NumStates        - Number of state samples in path
%      NumSegments      - Number of segments in path
%
%   navPathControl methods:
%      append           - Add segments to end of path
%      copy             - Create deep copy of object
%      interpolate      - Interpolates path based on propagator step size
%      pathDuration     - Total elapsed duration along path
%
%   Example:
%
%     % Create a propagator
%     propagator = mobileRobotPropagator;
% 
%     % Construct a kinodynamic planner
%     planner = plannerControlRRT(propagator);
%
%     % Define start and goal locations
%     start = [0 0 0];
%     goal  = [10 10 pi/2];
%     
%     % Plan a path using the state propagator
%     pathObj = plan(planner,start,goal);
%     
%     % Check the total elapsed time spent on the path
%     len = pathDuration(pathObj);
%
%     % Interpolate the path object to the propagator's control step size
%     interpolate(pathObj);
%
%     % Plan a second path from the final path location back to the start
%     pathObj2 = plan(planner, pathObj.States(end,:), start);
%
%     % Extract the sequence of motions from the second path
%     Q = pathObj2.States(2:end,:);
%     U = pathObj2.Controls;
%     QTGT = pathObj2.TargetStates;
%     DUR = pathObj2.Durations;
%
%     % Append this sequence to the end of the first path
%     append(pathObj,Q,U,QTGT,DUR)
% 
%     % Interpolate the new segments in the original path
%     interpolate(pathObj);
%
%   See also pathmetrics, mobileRobotPropagator, nav.StatePropagator

%   Copyright 2021 The MathWorks, Inc.
    
    %#codegen

    properties (SetAccess = protected)
        %StatePropagator State propagator for path
        %
        %   The StatePropagator defines the configuration and control
        %   spaces. It provides implementations for state and control
        %   sampling, system propagation, and provides cost, distance, and
        %   goal-reached metrics specific to the planning problem.
        StatePropagator
        
        %States Series of states for path
        %   The States property contains a series of states sampled within
        %   the state space bounds representing a path for navigation.
        %   Each row is a single state with the number of columns being the
        %   number of state variables. You can only set this property during
        %   object construction or through the append function.
        %
        %   Default: zeros(0, StateSpace.NumStateVariables)
        %
        %   See also append.
        States
        
        %Controls Series of control inputs along path
        %   The controls property contains a series of control inputs used
        %   to propagate the system forward in time. Each row is a single 
        %   control with the number of columns being the number of control
        %   inputs to the kinematic system. You can only set this property 
        %   during object construction or through the append function.
        %
        %   Default: zeros(0, StatePropagator.NumControlOutput)
        %
        %   See also append.
        Controls
        
        %Durations Series of durations over which controls are applied
        %   The durations property contains a vector of time spans. 
        %   The number of durations must match the number of controls, with 
        %   each control input applied to the system for the corresponding
        %   duration. You can only set this property during object 
        %   construction or through the append function.
        %
        %   Default: zeros(0, 1)
        %
        %   See also append.
        Durations
        
        %TargetStates A series of target states used by the propagator
        %   The TargetStates property contains a series of target
        %   configurations available during propagation. Targets can be 
        %   used to generate intermediate controls during propagation. The 
        %   number of targets must match the number of controls, and each 
        %   row is a single state with the number of columns being the 
        %   number of state variables. You can only set this property 
        %   during object construction or through the append function.
        %
        %   Default: zeros(0, StateSpace.NumStateVariables)
        %
        %   See also append.
        TargetStates
        
        %NumStates Number of state samples in path
        %   The NumStates property is the number of samples for the 
        %   path and is equal to the number of rows in the States property.
        NumStates
        
        %NumSegments Number of segments in path
        %   The NumSegments property is equivalent to the number of path
        %   segments found in the path. This is equivalent to the number of
        %   rows in the Controls, Durations, TargetStates, and IsGoal
        %   properties.
        %
        %   The number of segments will always be one less than the number
        %   of states, except when there are no states, in which case the
        %   number of segments is also zero.
        NumSegments
    end
    
    properties (Access = ?nav.algs.internal.InternalAccess, Transient)
        LastCleanIdx = 0;
        
        LastInterpIdx = 0;
        
        DurationInternal = 0;
    end
    
    methods
        function obj = navPathControl(propagator,varargin)
            narginchk(1,6);
            validateattributes(propagator,{'nav.StatePropagator'},{'scalar'},'navPathControl','StatePropagator');    
            
            % Validate path inputs
            [q,u,qTgt,dur] = navPathControl.validatePath(propagator,'navPathControl',1,varargin{:});
            qSz = propagator.StateSpace.NumStateVariables;
            uSz = propagator.NumControlOutput;
            coder.varsize('q',[inf qSz]);
            coder.varsize('u',[inf uSz]);
            coder.varsize('qTgt',[inf qSz]);
            coder.varsize('dur',[inf 1]);
            
            % Populate object
            obj.StatePropagator = propagator;
            obj.States = q;
            obj.Controls = u;
            obj.Durations = dur;
            obj.TargetStates = qTgt;
        end
        
        function L = pathDuration(obj)
        %pathDuration Total elapsed duration along path
            if obj.LastCleanIdx ~= size(obj.States,1)
                obj.DurationInternal = sum(obj.Durations);
                obj.LastCleanIdx = numel(obj.Durations);
            end
            L = obj.DurationInternal;
        end
        
        function append(obj,varargin)
        %append Adds a path segment to end of current path
        %
        %   append(OBJ,STATE,CONTROL,DURATION,TARGET) Appends a sequence of
        %   states, controls, durations, and targets to the end of the
        %   current path. If the path is empty when append is called, this 
        %   method expects that there are one fewer CONTROL, DURATION, and 
        %   TARGET than STATE. If the path is not empty, then the number
        %   must be equal, with the assumption that initial control and 
        %   duration are applied to the last state currently stored in the 
        %   path. By default, the path assumes that all TARGET states are 
        %   not goal states.
        
            % Validate inputs
            if isempty(obj.States)
                rowDiff = 1;
            else
                rowDiff = 0;
            end
            [q,u,qTgt,dur] = obj.validatePath(obj.StatePropagator,'append',rowDiff,varargin{:});
            
            % Append new segment to path
            obj.States = [obj.States; q];
            obj.Controls = [obj.Controls; u];
            obj.Durations = [obj.Durations; dur];
            obj.TargetStates = [obj.TargetStates; qTgt];
        end
        
        function interpolate(obj)
        %interpolate Interpolates path based on propagator step size
        %
        %   interpolate(OBJ) Evaluates the path at the propagator's
        %   ControlStepSize and adds all intermediate points to the path.
        
            if obj.LastInterpIdx < size(obj.States,1)
                % Extract path information
                q      = obj.States;
                u      = obj.Controls;
                qTgt   = obj.TargetStates;
                dur    = obj.Durations;
                
                % Find place to begin
                startIdx = max(min(obj.LastInterpIdx,obj.LastCleanIdx),1);
                
                % Define new path information
                Q = q(1:startIdx,:);
                U = u(1:startIdx-1,:);
                QTGT = qTgt(1:startIdx-1,:);
                DUR = dur(1:startIdx-1,:);
                
                % Calculate number of steps between uninterpolated segment ends.
                dt = obj.StatePropagator.ControlStepSize;
                STEPS = floor((dur(startIdx:end)+dt/2)/dt)-1;
                
                for i = startIdx:size(q,1)-1
                    numStep = STEPS(i-startIdx+1);
                    
                    T = unique([dt:dt:dur(i) dur(i)])';
                    
                    iStates   = zeros(numel(T),size(q,2));
                    iCommands = zeros(numel(T),size(u,2));
                    durations = repmat(dt,numel(T),1);
                    
                    q0 = q(i,:);
                    u0 = u(i,:);
                    
                    for n = 1:numStep
                        [iQ,iC,~] = obj.StatePropagator.propagate(...
                        q0,u0,qTgt(i,:),1);
                        iStates(n,:) = iQ(end,:);
                        iCommands(n,:) = u0;
                        q0 = iQ(end,:);
                        u0 = iC(end,:);
                    end
                    
                    % Append end point to intermediate points
                    iStates(end,:)   = q(i+1,:);
                    iCommands(end,:) = u0;
                    durations(end) = dur(i)-dt*numStep;
                    
                    Q    = [Q; iStates]; %#ok<AGROW> 
                    U    = [U; iCommands]; %#ok<AGROW> 
                    DUR  = [DUR; durations]; %#ok<AGROW> 
                    QTGT = [QTGT; repmat(qTgt(i,:),size(iStates,1),1)]; %#ok<AGROW> 
                end
                
                obj.States    = Q;
                obj.Controls  = U;
                obj.TargetStates = QTGT;
                obj.Durations = DUR;
                obj.LastInterpIdx = size(obj.States,1);
                obj.LastCleanIdx = size(obj.States,1);
            end
        end
        
        function cObj = copy(obj)
        %copy Creates a deep copy of the navPathControl object
            sp = copy(obj.StatePropagator);
            cObj = navPathControl(sp,obj.States,obj.Controls,obj.TargetStates,...
                obj.Durations);
            cObj.LastCleanIdx = obj.LastCleanIdx;
            cObj.LastInterpIdx = obj.LastInterpIdx;
            cObj.DurationInternal = obj.DurationInternal;
        end
        
        function numState = get.NumStates(obj)
            numState = size(obj.States,1);
        end
        
        function numControl = get.NumSegments(obj)
            numControl = size(obj.Controls,1);
        end
    end
    
    methods (Hidden,Static)
        function validateInput(input, requiredSize, fcnName, varName)
        %validateInput Common input validator
            validateattributes(input,{'numeric','logical'},{'size',requiredSize},fcnName,varName);
        end
        
        function [q,u,qTgt,dur] = validatePath(propagator,fcnName,expOffset,q,u,qTgt,dur)
        %validatePath Validates path segment
            
            qSz = propagator.StateSpace.NumStateVariables;
            uSz = propagator.NumControlOutput;
            
            if nargin <= 4
                if nargin < 4
                    q = zeros(0,propagator.StateSpace.NumStateVariables);
                end
                
                u = zeros(0,propagator.NumControlOutput);
                dur = zeros(0,1);
                qTgt = repmat(q,0,1);
            end
            
            navPathControl.validateInput(q,[nan qSz],fcnName,'States');
            
            if (nargin == 4 && (expOffset == 0 || size(q,1) > 1) || nargin > 4)
                narginchk(7,7);
            end
            
            numExpected = max(size(q,1)-expOffset,0);
            navPathControl.validateInput(u,[numExpected uSz],fcnName,'Controls');
            navPathControl.validateInput(dur,[numExpected 1],fcnName,'Durations');
            navPathControl.validateInput(qTgt,[numExpected qSz],fcnName,'TargetStates');
        end
    end
end