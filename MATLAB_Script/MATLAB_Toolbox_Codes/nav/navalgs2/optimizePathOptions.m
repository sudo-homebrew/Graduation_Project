classdef optimizePathOptions < matlabshared.tracking.internal.CustomDisplay
%optimizePathOptions Create optimization options for optimizePath function
%
%    OPTIONS = optimizePathOptions creates a set of default optimization
%    options for optimizePath function.
%
%    Optimization options are grouped into four categories:
%        Trajectory Parameters - Specify the constraints for robot motion
%                                throughout the path.
%        Obstacle Parameters   - Specify the distances which dictate the
%                                influence of obstacle on the path.
%        Solver Parameters     - Specify the options for solver used to
%                                optimize the path.
%        Weights               - Specify the cost function weights.
%
%    Trajectory Parameters 
%        MaxPathStates - Maximum number of poses allowed in path
%        ReferenceDeltaTime     - Travel time to be maintained between two
%                                 consecutive poses
%        MinTurningRadius       - Optimizer aims to maintain turning radius
%                                 above Minimum Turning Radius, usually a
%                                 robot property
%        MaxVelocity            - Optimizer aims to maintain Velocity below
%                                 this value (Soft Limit)
%        MaxAngularVelocity     - Optimizer aims to maintain Angular 
%                                 Velocity below this value (Soft Limit)
%        MaxAcceleration        - Optimizer aims to maintain Acceleration
%                                 below this value (Soft Limit)
%        MaxAngularAcceleration - Optimizer aims to maintain Angular
%                                 Acceleration below this value
%                                 (Soft Limit)
%
%    Obstacle parameters 
%        ObstacleSafetyMargin      - Optimizer aims to keep distance of
%                                    poses to the obstacles at-least at
%                                    this value.
%        ObstacleCutOffDistance    - Optimizer ignores obstacles beyond
%                                    this distance for the obstacle
%                                    component of the cost function
%        ObstacleInclusionDistance - Optimizer includes all obstacles
%                                    within this distance for the obstacle
%                                    component of the cost function.
%
%    Note: Closest on left and right are considered for obstacles with
%    distance between ObstacleInclusionDistance and ObstacleCutOffDistance
%
%    Solver Parameters
%        NumIteration       - Number of times solver is invoked
%        MaxSolverIteration - Maximum number of iterations per solver
%                             invocation
%
%    Weights
%        WeightTime                - Weight of time component
%        WeightSmoothness          - Weight of Non-Holonomic motion
%                                    component
%        WeightMinTurningRadius    - Weight of component to comply with
%                                    Minimum Turning Radius
%        WeightVelocity            - Weight of Velocity component
%        WeightAngularVelocity     - Weight of Angular Velocity component
%        WeightAcceleration        - Weight of Acceleration component
%        WeightAngularAcceleration - Weight of Angular Acceleration
%                                    component
%        WeightObstacles           - Weight of component to maintain Safe
%                                    distance from obstacles.
%     
% See also optimizePath, plannerHybridAStar

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties
        %ReferenceDeltaTime Travel time between two consecutive poses
        %Travel time between two consecutive poses, specified as a positive
        %scalar in seconds. This parameter along with reference velocity
        %impacts the interpolation distance between poses. Increase this
        %value to have lesser number of poses and reduce it to have higher
        %number of poses in output path.
        %Default: 0.3
        ReferenceDeltaTime = 0.3;
             
        %MaxPathStates Maximum number of poses allowed in the path.
        %Maximum number of poses allowed in the path, specified as an
        %integer greater than or equal to 3.
        %Default: 200
        MaxPathStates = 200;
        
        %NumIteration Number of solver invocations
        %Number of solver invocations, specified as a positive integer.
        %This value also specifies the number of times interpolation occurs
        %during optimization.
        %Default: 4
        NumIteration = 4;
        
        %MaxSolverIteration Maximum number of iterations per solver invocation.
        %Maximum number of iterations per solver invocation, specified as a
        %positive integer.
        %Default: 15
        MaxSolverIteration = 15;
        
        %WeightTime Cost function weight for time
        %Cost function weight for time, specified as a nonnegative scalar.
        %To lower the travel time, increase this weight value.
        %Default: 10
        WeightTime = 10;
        
        %WeightSmoothness Cost function weight for nonholonomic motion
        %Cost function weight for nonholonomic motion, specified as a
        %nonnegative scalar. To obtain smoother path, increase this weight
        %value.
        %Default: 1000
        WeightSmoothness = 1000;
        
        %WeightMinTurningRadius Cost function weight for complying with minimum turning radius
        %Cost function weight for complying with minimum turning radius,
        %specified as a nonnegative scalar. To make the path maintain the
        %minimum turning radius, increase this weight value.
        %Default: 10
        WeightMinTurningRadius = 10;
        
        %WeightVelocity Cost function weight for velocity
        %Cost function weight for velocity, specified as a nonnegative
        %scalar. To maintain the velocity closer to MaxVelocity, increase
        %this weight value.
        %Default: 100
        WeightVelocity = 100;
        
        %WeightAngularVelocity Cost function weight for angular velocity
        %Cost function weight for angular velocity, specified as a
        %nonnegative scalar. To maintain the angular velocity closer to
        %MaxAngularVelocity, increase this weight value.
        %Default: 10
        WeightAngularVelocity = 10;    
        
        %WeightAcceleration Cost function weight for acceleration
        %Cost function weight for acceleration, specified as a nonnegative
        %scalar. To maintain the acceleration closer to MaxAcceleration,
        %increase this weight value.
        %Default: 10
        WeightAcceleration = 10;
        
        %WeightAngularAcceleration Cost function weight for angular acceleration
        %Cost function weight for angular acceleration, specified as a
        %nonnegative scalar. To maintain the angular acceleration closer to
        %MaxAngularAcceleration, increase this weight value.
        %Default: 10
        WeightAngularAcceleration = 10;
        
        %WeightObstacles Cost function weight for maintaining safe distance from obstacles
        %Cost function weight for maintaining safe distance from obstacles,
        %specified as a nonnegative scalar. To maintain the safe distance
        %from obstacles, increase this weight value.
        %Default: 50
        WeightObstacles = 50;
        
        %MinTurningRadius Minimum turning radius in path
        %Minimum turning radius in the path, specified as a positive scalar
        %in meters. Note that this is a soft constraint and may be ignored
        %based on WeightMinTurningRadius value
        %Default: 1
        MinTurningRadius = 1;

        %MaxVelocity Maximum velocity along path
        %Maximum velocity along the path, specified as a positive scalar in
        %meters per second. Note that this is a soft constraint and may be
        %ignored based on WeightVelocity value.
        %Default: 0.4
        MaxVelocity = 0.4;

        %MaxAngularVelocity Maximum angular velocity along path
        %Maximum angular velocity along the path, specified as a positive
        %scalar in radians per second. Note that this is a soft constraint
        %and may be ignored based on WeightAngularVelocity value.
        %Default: 0.3
        MaxAngularVelocity = 0.3;
        
        %MaxAcceleration Maximum acceleration along path
        %Maximum acceleration along the path, specified as a positive
        %scalar in meters per second squared. Note that this is a soft
        %constraint and may be ignored based on WeightAcceleration value.
        %Default: 0.5
        MaxAcceleration = 0.5;
        
        %MaxAngularAcceleration Maximum angular acceleration along path
        %Maximum angular acceleration along path, specified as a positive
        %scalar in radians per second squared. Note that this is a soft
        %constraint and may be ignored based on WeightAngularAcceleration
        %value.
        %Default: 0.5
        MaxAngularAcceleration = 0.5;
        
        %ObstacleSafetyMargin Safety distance from obstacles
        %Safety distance from the obstacles, specified as a positive scalar
        %in meters. Note that this is a soft constraint and may be ignored
        %based on WeightObstacles value
        %Default: 0.5
        ObstacleSafetyMargin = 0.5;
        
        %ObstacleCutOffDistance Obstacle cutoff distance
        %Obstacle cutoff distance, specified as a positive scalar in
        %meters. The path optimizer ignores obstacles beyond cutoff
        %distance.
        %Default: 2.5
        ObstacleCutOffDistance = 2.5;
        
        %ObstacleInclusionDistance Obstacle inclusion distance 
        %Obstacle inclusion distance, specified as a positive scalar in
        %meters. The path optimizer includes all obstacles within the
        %inclusion distance. Only closest obstacle on left and right
        %between inclusion and cutoff distance are considered.
        %Default: 0.75
        ObstacleInclusionDistance = 0.75;
    end
        
    methods
        
        function obj = optimizePathOptions()
            %Default constructor.
        end
        %% Setter functions for each property
        function set.ReferenceDeltaTime(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'ReferenceDeltaTime');
            obj.ReferenceDeltaTime = input;
        end
                
        function set.MaxPathStates(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','finite','integer','>=',3}  , 'optimizePathOptions', 'MaxPathStates');
            obj.MaxPathStates = input;
        end
        
        function set.NumIteration(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite','integer'} , 'optimizePathOptions', 'NumIteration');
            obj.NumIteration = input;
        end
        
        function set.MaxSolverIteration(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite','integer'} , 'optimizePathOptions', 'MaxSolverIteration');
            obj.MaxSolverIteration = input;
        end
        
        function set.WeightTime(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightTime');
            obj.WeightTime = input;
        end
        
        function set.WeightSmoothness(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightSmoothness');
            obj.WeightSmoothness = input;
        end
        
        function set.WeightMinTurningRadius(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightMinTurningRadius');
            obj.WeightMinTurningRadius = input;
        end
        
        function set.WeightVelocity(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightVelocity');
            obj.WeightVelocity = input;
        end
        
        function set.WeightAngularVelocity(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightAngularVelocity');
            obj.WeightAngularVelocity = input;
        end
        
        function set.WeightAcceleration(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightAcceleration');
            obj.WeightAcceleration = input;
        end
        
        function set.WeightAngularAcceleration(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightAngularAcceleration');
            obj.WeightAngularAcceleration = input;
        end
        
        function set.WeightObstacles(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','nonnegative','finite'} , 'optimizePathOptions', 'WeightObstacles');
            obj.WeightObstacles = input;
        end
        
        function set.MinTurningRadius(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'MinTurningRadius');
            obj.MinTurningRadius = input;
        end
        
        function set.MaxVelocity(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'MaxVelocity');
            obj.MaxVelocity = input;
        end
        
        function set.MaxAngularVelocity(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'MaxAngularVelocity');
            obj.MaxAngularVelocity = input;
        end
        
        function set.MaxAcceleration(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'MaxAcceleration');
            obj.MaxAcceleration = input;
        end
        
        function set.MaxAngularAcceleration(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'MaxAngularAcceleration');
            obj.MaxAngularAcceleration = input;
        end
        
        function set.ObstacleSafetyMargin(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'ObstacleSafetyMargin');
            obj.ObstacleSafetyMargin = input;
        end
        
        function set.ObstacleCutOffDistance(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'ObstacleCutOffDistance');
            obj.verifyCutOffLessThanInclusion(input,-1);
            obj.ObstacleCutOffDistance = input;
        end
        
        function set.ObstacleInclusionDistance(obj,input)
            validateattributes(input,...
                               {'double'},{'scalar','nonnan','nonempty','positive','finite'} , 'optimizePathOptions', 'ObstacleInclusionDistance');
            obj.verifyCutOffLessThanInclusion(-1,input);
            obj.ObstacleInclusionDistance = input;
        end
    end
    methods(Hidden)
        function verifyCutOffLessThanInclusion(obj,cutOff,inclusionF)
        % Function to verify if cut off factor is greater than inclusion
        % factor.
            if(cutOff == -1)
                cutOff = obj.ObstacleCutOffDistance;
            end
            if(inclusionF == -1)
                inclusionF = obj.ObstacleInclusionDistance;
            end
            coder.internal.errorIf(inclusionF > cutOff,...
               'nav:navalgs:optimizepath:CutoffLessThanInclusion');
        end

    end

    methods (Access = protected)
        
       %% Custom Display attributes
       function header = getHeader(obj)
       % Setting up header.
          if ~isscalar(obj)
             header = getHeader@matlab.mixin.CustomDisplay(obj);
          else
             headerStr = matlab.mixin.CustomDisplay.getClassNameForHeader(obj);             
             header = sprintf('%s\n',headerStr);
          end
       end
       function propgrp = getPropertyGroups(obj)
       % Formatting property display in groups with group name as bold.
          if ~isscalar(obj)
             propgrp = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
          else 
             propList3 = {'NumIteration','MaxSolverIteration'};
             gTitle3 = ['<strong>'...
                message('nav:navalgs:optimizepath:SolverParameters').getString() ...
                '</strong>'];
             
             propList2 = {'ObstacleSafetyMargin','ObstacleCutOffDistance',...
                 'ObstacleInclusionDistance'};
             gTitle2 = ['<strong>'...
                message('nav:navalgs:optimizepath:ObstacleParameters').getString() ...
                '</strong>'];
             
             propList1 = {'MaxPathStates', 'ReferenceDeltaTime', ...
                 'MinTurningRadius','MaxVelocity',...
                 'MaxAngularVelocity','MaxAcceleration',...
                 'MaxAngularAcceleration'};
             gTitle1 = ['<strong>'...
                message('nav:navalgs:optimizepath:TrajectoryParameters').getString() ...
                '</strong>'];
             
             propList4 = {'WeightTime','WeightSmoothness',...
                 'WeightMinTurningRadius','WeightVelocity',...
                 'WeightAngularVelocity','WeightAcceleration',...
                 'WeightAngularAcceleration','WeightObstacles'};
             gTitle4 = ['<strong>'...
                message('nav:navalgs:optimizepath:Weights').getString() ...
                '</strong>'];
             
             propgrp1 = matlab.mixin.util.PropertyGroup(propList1,gTitle1);             
             propgrp2 = matlab.mixin.util.PropertyGroup(propList2,gTitle2);
             propgrp3 = matlab.mixin.util.PropertyGroup(propList3,gTitle3);             
             propgrp4 = matlab.mixin.util.PropertyGroup(propList4,gTitle4);

             propgrp = [propgrp1 propgrp2 propgrp3 propgrp4];
             
          end
       end
    end

end
