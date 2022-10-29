classdef StatePropagator < nav.algs.internal.InternalAccess
%StatePropagator Create state propagator for use in control-based planning
%   StatePropagator is an interface for all state propagators used with
%   kino-dynamic planners. Derive from this class if you are defining a
%   propagator for your custom model and/or control system.
%   
%   This representation allows for control generation, state propagation,
%   cost estimation between states, and verifying whether two states are
%   equivalent.
%   
%   Controlled systems should follow the basic form of:
%
%       [u_i, steps] = fcn(q,u,qTgt);     -> Generate a control command
%       q_i = q+integrate(q,u_i,steps);   -> Propagate system using generated command
%
%   where:
%       q/q_i are Q-element row-vectors representing the system's state
%       u/u_i are U-element row-vectors capture the system's top-level control command
%
%   SP = nav.StatePropagator(stateSpace, stepSize, numControlOutput) creates 
%   a state propagator object, SP, responsible for propagating a kino-dynamic
%   system. The state space of the system is represented by stateSpace, a
%   child class of nav.StateSpace, stepSize, a positive scalar indicating
%   the time taken for a single propagation step, and numControlOutput, a
%   positive integer-valued scalar, which defines the number of control 
%   command variables output by the system's top-level controller.
%
%   StatePropagator properties:
%      ControlStepSize      - Amount of time elapsed per control step
%      StateSpace           - A space representing state of system during planning
%      NumControlOutput     - Number of variables in control command
%
%   StateSpace methods:
%      copy                	- Create deep copy of the state propagator object
%      distance           	- Estimate cost of propagating to target state
%      propagate            - Propagate system 
%      propagateWhileValid 	- Propagate system and return valid motion
%      sampleControl        - Return control command and stepsation it should be applied
%      setup                - A setup utility called at the start of planning
%
%   See also mobileRobotPropagator

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        %StateSpace A space representing state of system during planning
        %
        %   The StateSpace is responsible for representing a system's 
        %   configuration space. This should include all state information 
        %   related to the propagated system. Systems employing multi-layer
        %   cascade controllers may choose to append persistent low-level 
        %   control-state information directly to the state vector, whereas
        %   top-level control commands are managed directly by the
        %   StatePropagator.
        StateSpace
        
        %ControlStepSize Amount of time elapsed per control step
        ControlStepSize = 0.1
    end
    
    properties (SetAccess = immutable)
        %NumControlOutput Number of variables in control command
        NumControlOutput
    end
    
    methods
        function obj = StatePropagator(ss, stepSize, numControlOutput)
            obj.StateSpace = ss;
            obj.ControlStepSize = stepSize;
            
            validateattributes(numControlOutput,{'numeric'},{'scalar','positive','integer'},'StatePropagator','NumControlOutput');
            obj.NumControlOutput = numControlOutput;
        end
    end
    
    methods (Abstract)
        %distance Estimated cost of propagating to target state
        %
        %   H = distance(Q1, q2) returns the estimated distance between
        %   a set of initial states, Q1 as an N-by-S matrix, and a final 
        %   state, q2 as an S-element row vector, where N is the number of 
        %   states and S is the number of variables in the StateSpace.
        %
        %   For each {q1,q2} pair, this function should return the 
        %   estimated cost of propagating the system from q1 to q2.
        %   
        %   Note: Values returned by this function are used to find the 
        %   "nearest"-neighbor to a sampled target state.
        h = distance(Q1,q2)
        
        %propagateWhileValid Propagate system and return valid motion
        %
        %   Accepts an initial state and control command, and iteratively 
        %   propagates the system towards qTgt.
        %
        %   [q, u, steps] = propagateWhileValid(obj, q0, u0, qTgt, maxSteps)
        %   Propagate the system from the current state, q0, towards a
        %   target state, qTgt, starting with the initial control input u0 
        %   for up to a given number of steps, maxSteps.
        %
        %   NOTE: This function is responsible for validating the motion 
        %   between (q0,q)
        % 
        %   Returns:
        %       q            The final state of the system
        %       u            The control after final step has been reached
        %       steps        The total number of steps between (q0,q)
        %
        %   NOTE: If the propagation failed, or to skip adding the current 
        %         state to the tree, return q as empty.
        %
        %   [Q, U, STEPS] = propagateWhileValid(obj, q0, u0, qTgt, maxSteps)
        %   Propagate the system up to the given number of steps and optionally 
        %   return system information at intermediate steps along the 
        %   motion. All propagations should be validated and the function 
        %   should only return system information between q0 and the last 
        %   valid state.
        %
        %   At the end of each propagation step, i, the system should return:
        %       Q(i,:)       The current state of the system
        %       U(i,:)       The control input for step i+1
        %       STEPS(i)     The steps between i-1, i
        [Q, U, STEPS] = propagateWhileValid(obj, q0, u0, qTgt, maxSteps)
        
        %propagate Propagates the system without validation
        %   Accepts an initial state and control command, and iteratively 
        %   propagates the system towards qTgt.
        %
        %   [q, u, steps] = propagate(obj, q0, u0, qTgt, maxSteps)
        %   Propagate the system from the current state, q0, towards a
        %   target state, qTgt, starting with the initial control input u0 
        %   for up to a given number of steps, maxSteps.
        % 
        %   Returns:
        %       q            The final state of the system
        %       u            The control after final step has been reached
        %       steps        The total number of steps between (q0,q)
        %
        %   NOTE: To skip adding the current state to the tree, return q as empty.
        %
        %   [Q, U, STEPS] = propagate(obj, q0, u0, qTgt, maxSteps)
        %   Propagate the system up to the given number of steps and optionally 
        %   return system information at intermediate steps along the 
        %   motion.
        %
        %   At the end of each propagation step, i, the system should return:
        %       Q(i,:)       The current state of the system
        %       U(i,:)       The control input for step i+1
        %       STEPS(i)     The steps between i-1, i
        %
        %   See also propagateWhileValid
        [Q, U, STEPS] = propagate(obj, q0, u0, qTgt, maxSteps);
        
        %sampleControl Generate a control command and duration
        %
        %   [u, steps] = sampleControl(obj, q0, u0, qTgt) Takes
        %   in the current state and control command, q0, u0, respectively,
        %   and generates a new control command, u, and number of steps to
        %   propagate the system, towards the target state, qTgt.
        [u, steps] = sampleControl(obj, q0, u0, qTgt)
        
        %COPY Create deep copy of state space object
        copyObj = copy(obj)
        
        %setup A setup utility called at the start of planning
        %
        %   setup(OBJ) Used to perform initial configurations to the 
        %   StatePropagator object. This method is always called by the 
        %   planner at the beginning of the 'plan' method.
        setup(obj)
    end
    
    methods 
        function set.StateSpace(obj,ss)
            validateattributes(ss,{'nav.StateSpace'},{'scalar','nonempty'},'StatePropagator','StateSpace');
            
            obj.StateSpace = ss;
        end
        
        function set.ControlStepSize(obj,stepSize)
            validateattributes(stepSize,{'numeric'},{'scalar','positive'},'StatePropagator','ControlStepSize');
            obj.ControlStepSize = stepSize;
        end
    end
    
    methods(Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
        % Let the coder know about non-tunable parameters
            props = {'NumControlOutput'};
        end
    end
end
