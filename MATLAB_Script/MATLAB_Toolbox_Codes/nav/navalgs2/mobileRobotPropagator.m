classdef mobileRobotPropagator < nav.StatePropagator
%mobileRobotPropagator State propagator for wheeled robotic systems
%
%   The mobileRobotPropagator is a convenience class intended to introduce 
%   concepts behind the state-propagator and plannerControlRRT
%   infrastructure. This object provides several kinematic models,
%   propagators, control laws, integrators, and validators which can be
%   interchangebly combined to get up and running with plannerControlRRT.
%
%   Each internal component potentially has its own set of tunable
%   parameters, which can be viewed and adjusted using the SystemParameters
%   property.
%
%   For optimal planning performance, implement a state-propagator class 
%   whose model, distance-estimator, control-laws, etc... accurately 
%   describe the propagated system. A detailed example for such a system 
%   can be found by running the following:
%
%   >> openExample('nav/TractorTrailerPlanningUsingPlannerControlRRTExample')
%
%   PROPAGATOR = mobileRobotPropagator creates a default state-propagator
%   with bicycle kinematics, linear-pursuit control law, rk4 integrator,
%   euclidean distance estimator, and no validation environment.
%
%   PROPAGATOR = mobileRobotPropagator(SPACE,Name,Value) provides additional
%   options specified by one or more Name,Value pair arguments. You can 
%   specify several name-value pair arguments in any order as 
%   Name1,Value1,...,NameN,ValueN:
%
%       'KinematicModel'  - String or char-array defining the kinematic 
%                           model used during propagation. This property 
%                           can only be set during construction.
%
%                           Default: 'bicycle'
%
%        'ControlPolicy'  - String or char-array defining the control law 
%                           used during planning and propagation. This 
%                           property can only be set during construction.
%
%                           Default: 'linearpursuit'
%
%           'Integrator'  - String or char-array defining the integrator 
%                           used during propagation. This property can only
%                           be set during construction.
%
%                           Default: 'rungekutta4'
%
%    'DistanceEstimator'  - String or char-array defining the distance
%                           metric used to find nearest-neighbors. This 
%                           property can only be set during construction.
%
%                           Default: 'euclidean'
%
%          'Environment'  - A binaryOccupancyMap, occupancyMap, or 
%                           vehicleCostmap object used to validate states 
%                           during planning. If an environment is not 
%                           provided, the propagator will only check 
%                           whether states fall within the state-space's 
%                           StateBounds. This property can only be set
%                           during construction.
%
%                           Default: []
%
%      'ControlStepSize'  - A positive scalar which defines the amount of 
%                           time elapsed per control step.
%
%                           Default: 0.1 [s]
%
%         'GoalDistance'  - A positive scalar which defines the max 
%                           distance for two states to be considered 
%                           "equal".
%
%                           Default: 1 [m]
%
%      'MaxControlSteps'  - A positive integer-valued scalar which defines
%                           the max number of steps to update the system 
%                           during propagation.
%
%                           Default: 10
%
%   mobileRobotPropagator Properties:
% 
%        StateSpace             - A space representing state of system during planning
%        GoalDistance           - Max distance for two states to be considered "equal"
%        ControlStepSize        - Amount of time elapsed per control step
%        MaxControlSteps        - The max number of steps to update the system during propagation.
%        ControlLimits          - Limits for controls generated by top-level controller
%        NumControlOutput       - Number of variables in top-level control space
%        KinematicModel         - Kinematic model used to propagate the system
%        Integrator             - Integration technique used during state propagation
%        DistanceEstimator      - Distance metric used to approximate propagation cost
%        ControlPolicy          - Generates controls that propagate system towards target
%        Environment            - A representation of the planning environment
%
%   mobileRobotPropagator Methods:
%
%        copy                	- Create deep copy of the state propagator object
%        distance           	- Estimate cost of propagating to target state
%        propagate              - Propagate system 
%        propagateWhileValid 	- Propagate system and return valid motion
%        sampleControl          - Return control command and steps it should be applied
%        setup                  - A setup utility called at the start of planning
%
%   Example:
%
%     % Load map
%     load('exampleMaps')
%     map = occupancyMap(ternaryMap,10);
% 
%     % Create a propagator that uses bicycle model kinematics
%     propagator = mobileRobotPropagator('Environment',map);
% 
%     % Update state space bounds
%     propagator.StateSpace.StateBounds(1:2,:) = ...
%                           [map.XWorldLimits; map.YWorldLimits];
% 
%     % Create planner
%     planner = plannerControlRRT(propagator);
% 
%     % Define start and goal location
%     start = [10 15 0];
%     goal  = [40 30 0];
% 
%     % Plan a path
%     rng(0,'twister');
%     path = plan(planner,start,goal);
% 
%     % Visualize results
%     show(map);
%     hold on;
%     interpolate(path);
%     plot(start(1),start(2),'rx');
%     plot(goal(1),goal(2),'go');
%     plot(path.States(:,1),path.States(:,2),'b');
%
%   See also nav.StateSpace, plannerControlRRT

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties        
        %GoalDistance Max distance for two states to be considered "equal".
        GoalDistance = 1;
        
        %MaxControlSteps The max number of steps to update the system during propagation.
        MaxControlSteps = 10
        
        %SystemParameters Customizable parameters for kinematic system.
        SystemParameters
    end
    
    properties (Dependent)
        %ControlLimits Limits for controls generated by top-level controller
        ControlLimits
    end
    
    properties (SetAccess = 'immutable')
        %KinematicModel The propagator's kinematic model
        %
        %   'bicycle'       - A 2D bicycle model (default)
        %                       q = [x y theta]
        %                       u = [v psi]
        %
        %   'ackermann'     - A 2D Ackermann model
        %                       q = [x y theta psi]
        %                       u = [v psiDot]
        %                       
        %	NOTE: Ackermann model requires Robotics System Toolbox
        KinematicModel
        
        %Integrator The integration technique used during state propagation
        %
        %   'rungekutta4' (Default)
        %   'euler'
        Integrator
        
        %DistanceEstimator Distance metric used to approximate propagation cost
        %
        %   'euclidean'     - Standard euclidean distance function applied
        %                     to all elements of the state vector.
        %
        %                     (Default)
        %
        %   'dubins'        - Distance along a Dubins path that connects
        %                     two states in the SE2 subspace.
        %
        %   'reedsshepp'    - Distance along a ReedsShepp path that
        %                     connects two states in the SE2 subspace.
        DistanceEstimator
        
        %ControlPolicy Generates controls that propagate system towards target
        %
        %   The ControlPolicy generates control commands and durations that
        %   attempt to guide the kinematic system towards the target state.
        %   Policies include:
        %
        %   'linearpursuit'     - Samples a random velocity and calculates a
        %                         lookahead point along the xy vector pointing 
        %                         from q to qTgt. Calculates a steering input 
        %                         which propagates the vehicle towards the
        %                         lookahead point. This differs from the 
        %                         standard Pure Pursuit algorithm in that 
        %                         the lookahead path is not fixed.
        %   
        %                         	(Default)
        %
        %   'arcpursuit'        - Samples a random velocity and calculates a
        %                         lookahead point along an arc which is tangent
        %                         to qTgt and intersects with (qX,qY). A
        %                         steering input is then calculated which
        %                         propagates the vehicle towards the 
        %                         lookahead point.
        %
        %   'randomsamples'     - Draws a finite set of random control
        %                         samples from the kinematic model's control 
        %                         space and chooses the control that brings 
        %                         the system closest to the target.
        %
        %   NOTE: If the kinematic model is 'Ackermann', the arc and linear
        %         pursuit policies calculate the current steering error and
        %         convert this to a desired steer rate.
        ControlPolicy
        
        %Environment A representation of the planning environment
        %
        %   An object used for validation of discrete states along a
        %   propagated motion. Environment objects can be of type 
        %   occupancyMap, binaryOccupancyMap, or vehicleCostmap.
        %
        %   By default the Environment is empty, in which case states 
        %   will only be rejected if they fall outside the state space 
        %   bounds.
        Environment
    end
    
    properties (Access = ?nav.algs.internal.InternalAccess)
        %IntegratorInternal
        IntegratorInternal
        
        %KinematicModelInternal
        KinematicModelInternal
        
        %ControlPolicyInternal
        ControlPolicyInternal
        
        %StateValidatorInternal
        StateValidatorInternal
        
        %odeFcn
        odeFcn
        
        %SkipStateValidation
        SkipStateValidation = false
    end
    
    methods
        function obj = mobileRobotPropagator(varargin)
            % Parse inputs
            nvPairs = mobileRobotPropagator.parseInputs(varargin{:});
            
            % Create kinematic model
            model = mobileRobotPropagator.buildModel(nvPairs.KinematicModel, nvPairs.DistanceEstimator);
            
            % Define control information
            controlLimits = model.ControlLimits;
            numControlOutput = size(controlLimits,1);
            ss = model.StateSpace;
            
            % Create base class
            obj = obj@nav.StatePropagator(ss,nvPairs.ControlStepSize,numControlOutput);
            obj.GoalDistance = nvPairs.GoalDistance;
            obj.MaxControlSteps = nvPairs.MaxControlSteps;
            
            % Populate user-facing properties
            obj.KinematicModel = lower(nvPairs.KinematicModel);
            obj.Integrator = lower(nvPairs.Integrator);
            obj.DistanceEstimator = lower(nvPairs.DistanceEstimator);
            obj.ControlPolicy = lower(nvPairs.ControlPolicy);
            obj.Environment = nvPairs.Environment;
            
            % Construct and assign internal classes
            obj.KinematicModelInternal = model;
            obj.ControlPolicyInternal = mobileRobotPropagator.buildControlPolicy(nvPairs.ControlPolicy);
            obj.IntegratorInternal = mobileRobotPropagator.buildIntegrator(nvPairs.Integrator);
            
            % Create StateValidator
            obj.StateValidatorInternal = obj.buildValidator(obj.KinematicModelInternal,obj.Environment);
            
            % Setup propagator
            obj.setup();
        end
        
        function [iStates, iCommands, numStep] = propagate(obj, q0, u0, qTgt, maxSteps)
            if ~obj.SkipStateValidation
                % Validate inputs when not being called by planner
                qSz = obj.StateSpace.NumStateVariables;
                uSz = obj.NumControlOutput;
                obj.validateStateInputs('propagate',qSz,uSz,q0,u0,qTgt);
                validateattributes(maxSteps,{'numeric'}, ...
                    {'integer','numel',1,'nonnegative'},'propagate','maxSteps');
            end
            
            dt = obj.ControlStepSize;
            
            iStates   = nan(maxSteps,obj.StateSpace.NumStateVariables);
            iCommands = nan(maxSteps,obj.NumControlOutput);
            
            % Retrieve current state/command
            q = q0;
            u = u0;
            
            uMin = obj.KinematicModelInternal.ControlLimits(:,1)';
            uMax = obj.KinematicModelInternal.ControlLimits(:,2)';
            
            t = dt;
            i = 0;
            
            % Drive system 
            while i < maxSteps
                i = i+1;
                t = t+dt;
                
                % Store intermediate states and controls
                iStates(i,:) = obj.odeFcn(q,u,dt);
                % Generate next control command
                u = obj.ControlPolicyInternal.sampleNext(obj,q,u,qTgt);
                u = min(max(u,uMin),uMax);
                iCommands(i,:) = u;
                
                if obj.targetReached(iStates(i,:),qTgt,u)
                    iStates((i+1):end,:) = [];
                    iCommands((i+1):end,:) = [];
                    break;
                else
                    q = iStates(i,:);
                end
            end
            
            numStep = ones(i,1);
        end
        
        function [iStates, iCommands, numStep] = propagateWhileValid(obj, q0, u0, qTgt, maxSteps)
            [iStates, iCommands, numStep] = obj.propagate(q0,u0,qTgt,maxSteps);
            
            isValid = obj.StateValidatorInternal.isStateValid(iStates);
            lastValid = find(~isValid,1,'first');
            
            % Clear states lying beyond the final state
            if ~isempty(lastValid)
                idx = lastValid(1);
                iStates(idx:end,:) = [];
                iCommands(idx:end,:) = [];
                numStep(idx:end,:) = [];
            end
        end
        
        function h = distance(obj, q1, q2)
            narginchk(3,3)
            sz1 = size(q1,1);
            sz2 = size(q2,1);
            coder.internal.errorIf(sz1 ~= sz2 && (sz1 > 1 && sz2 > 1),'nav:navalgs:mobilerobotpropagator:MismatchStates');
            h = obj.StateSpace.distance(q1,q2);
        end
        
        function [u, numStep] = sampleControl(obj, q0, u0, qTgt)
            if ~obj.SkipStateValidation
                % Validate inputs when not being called by planner
                qSz = obj.StateSpace.NumStateVariables;
                uSz = obj.NumControlOutput;
                obj.validateStateInputs('sampleControl',qSz,uSz,q0,u0,qTgt);
            end
            [u, numStep] = obj.ControlPolicyInternal.sampleControl(obj,q0,u0,qTgt);
        end
        
        function [TF, dist] = targetReached(obj, q, qTgt, ~)
        %targetReached Default goal reached function
            dist = obj.StateSpace.distance(q,qTgt);
            TF = dist <= obj.GoalDistance;
        end
        
        function cObj = copy(obj)
        %copy Create a deep copy of the object
            names = fieldnames(mobileRobotPropagator.defaultNVPairs());
            inputs = {};
            for i = 1:numel(names)
                propVal = obj.(names{i});
                if isa(propVal,'handle')
                    inputs = {inputs{:} names{i} copy(propVal)};  
                else
                    inputs = {inputs{:} names{i} obj.(names{i})};  
                end
            end
            % Configure the type of the copied mobileRobotPropagator
            cObj = mobileRobotPropagator(inputs{:});
            
            % Update the internal property values
            cObj.StateValidatorInternal = copy(obj.StateValidatorInternal);
            ss = cObj.StateValidatorInternal.StateSpace;
            cObj.KinematicModelInternal.StateSpace = ss;
            cObj.StateSpace = ss;
            cObj.SystemParameters = obj.SystemParameters;
            setup(cObj);
        end
        
        function setup(obj)
            model = obj.KinematicModelInternal;
            derivFcn  = model.getDerivativeFcn();
            updateFcn = model.getUpdateFcn();
            obj.odeFcn = obj.IntegratorInternal.getIntegrator(derivFcn,updateFcn);
        end
        
        function set.ControlLimits(obj, bounds)
        %set.ControlLimits Setter for ControlLimits dependent property
            obj.KinematicModelInternal.ControlLimits = bounds;
        end

        function bounds = get.ControlLimits(obj)
        %get.ControlLimits Getter for ControlLimits dependent property
            bounds = obj.KinematicModelInternal.ControlLimits;
        end
        
        function paramStruct = get.SystemParameters(obj)
            paramStruct = struct();
            componentNames = {'KinematicModel', 'Integrator', 'ControlPolicy'};
            for i = 1:numel(componentNames)
                paramStruct.(componentNames{i}) = obj.([char(componentNames{i}) 'Internal']).getParameters();
            end
        end
        
        function set.SystemParameters(obj, paramStruct)
            validNames = {'KinematicModel', 'Integrator', 'ControlPolicy'};
            names = fieldnames(paramStruct);
            for i = 1:numel(names)
                validatestring(names{i},validNames,'mobileRobotPropagator','SystemParameters');
                obj.([char(names{i}) 'Internal']).setParameters(obj, paramStruct.(names{i}));
            end
        end
    end
    
    methods (Hidden)
        function tf = isequal(this,other)
            tf = compare(@isequal,this,other);
        end
        
        function tf = isequaln(this,other)
            tf = compare(@isequaln,this,other);
        end
    end
    
    methods (Access = protected)
        function tf = compare(f,this,other)
        %compare Check if two objects are equal
            compProps = [properties(this); 'IntegratorInternal'; 'KinematicModelInternal'; ...
                'ControlPolicyInternal'; 'StateValidatorInternal'; 'odeFcn'];
            tf = true;
            for i = 1:numel(compProps)
                p1 = this.(compProps{i});
                p2 = other.(compProps{i});
                if isa(p1,'function_handle')
                    tf = tf & f(func2str(p1),func2str(p2));
                else
                    tf = tf & f(p1,p2);
                end
            end
        end
    end
    
    methods (Static, Hidden)
        function nvPairs = defaultNVPairs()
            nvPairs = struct(...
                'ControlStepSize', 0.1, ...
                'GoalDistance', 1, ...
                'MaxControlSteps',10, ...
                'KinematicModel', 'bicycle', ...
                'DistanceEstimator', 'euclidean', ...
                'Integrator', 'rungekutta4', ...
                'ControlPolicy', 'linearpursuit', ...
                'Environment', []);
        end
        
        function nvPairs = parseInputs(varargin)
            % Retrieve default NV-pairs
            defaults = mobileRobotPropagator.defaultNVPairs();
            
            pstruct = coder.internal.parseParameterInputs(defaults,struct(),varargin{:});
            nvPairs = coder.internal.vararginToStruct(pstruct,defaults,varargin{:});
        end
        
        function model = buildModel(modelType, metric)
            metric = validatestring(metric,{'euclidean','dubins','reedsShepp'},'mobileRobotPropagator','DistanceEstimator');
            modelType = validatestring(modelType,{'bicycle','ackermann'},'mobileRobotPropagator','KinematicModel');
            switch metric
                case 'euclidean'
                    ss = stateSpaceSE2;
                case 'dubins'
                    ss = stateSpaceDubins;
                otherwise
                    ss = stateSpaceReedsShepp;
            end
            
            % Ackermann model not available in deployed mode
            coder.internal.errorIf(isdeployed && isequal(modelType,'ackermann'),'nav:navalgs:mobilerobotpropagator:NoAckermannMCC');
            ctor = str2func(['@nav.algs.internal.models.' modelType]);
            model = ctor(ss);
        end
        
        function policy = buildControlPolicy(policyName)
            policyName = validatestring(policyName,{'randomSamples','linearPursuit','arcPursuit'},'mobileRobotPropagator','ControlPolicy');
            switch policyName
                case 'randomSamples'
                    policy = nav.algs.internal.ControlPolicies.randomSamples;
                case 'linearPursuit'
                    policy = nav.algs.internal.ControlPolicies.linearPursuit;
                otherwise % arcPursuit
                    policy = nav.algs.internal.ControlPolicies.arcPursuit;
            end
        end
        
        function validator = buildValidator(model,environment)
            ss = model.StateSpace;
            if isempty(environment)
                validator = nav.algs.internal.dummyValidator(ss);
            else
                validateattributes(environment,...
                    {'occupancyMap','binaryOccupancyMap','vehicleCostmap'},...
                    {'scalar'},'mobileRobotPropagator','Environment');
                
                switch class(environment)
                    case {'occupancyMap','binaryOccupancyMap'}
                        validator = validatorOccupancyMap(ss,'Map',environment);
                    otherwise
                        vDims = environment.CollisionChecker.VehicleDimensions;
                        fOver = vDims.FrontOverhang;
                        rOver = vDims.RearOverhang;
                        vDims.Wheelbase = model.WheelBase;
                        vDims.Length = fOver+rOver+model.WheelBase;
                        environment.CollisionChecker.VehicleDimensions = vDims;
                        validator = validatorVehicleCostmap(ss,'Map',environment);
                        validator.ThetaIndex = 3;
                end
            end
        end
        
        function integrator = buildIntegrator(integratorName)
            name = validatestring(integratorName,{'euler','rungeKutta4'},'mobileRobotPropagator','Integrator');
            switch name
                case 'euler'
                    integrator = nav.algs.internal.integrators.euler;
                case 'rungeKutta4'
                    integrator = nav.algs.internal.integrators.rk4;
            end
        end
        
        function validateStateInputs(fcnName, qSz, uSz, q0, u0, qTgt)
            validateattributes(q0,{'numeric'},{'row','numel',qSz},fcnName,'q0');
            validateattributes(u0,{'numeric'},{'row','numel',uSz},fcnName,'u0');
            validateattributes(qTgt,{'numeric'},{'row','numel',qSz},fcnName,'qTgt');
        end
    end
end