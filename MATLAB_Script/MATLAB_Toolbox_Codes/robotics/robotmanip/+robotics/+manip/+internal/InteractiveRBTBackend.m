classdef InteractiveRBTBackend < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %INTERACTIVERBTBACKEND Model object for interactiveRigidBodyTree
    %   This class stores the RigidBodyTree properties and executes RBT
    %   tasks like IK, Configuration updates, and getTransform calls.

    %   Copyright 2019-2021 The MathWorks, Inc.
    
    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        
        %RigidBodyTree - Rigid body tree robot model
        %   Rigid body tree robot model, specified as a rigidBodyTree
        %   object that defines the inertial and kinematic properties of
        %   the manipulator. The RigidBodyTree property is defined at
        %   object construction and is read-only.
        RigidBodyTree
        
        %IKSolver - The inverse kinematics solver object
        IKSolver
        
        %GIKPoseTarget - Pose target constraint used for marker following
        %   The IKSolver uses the GIKPoseTarget constraint as the primary
        %   pose target constraint for interfacing with the interactive
        %   marker. When the marker is updated, the pose target is updated,
        %   and that target is used to compute the corresponding
        %   configuration via a generalized inverse kinematics solver.
        GIKPoseTarget
    end
    
    properties (Dependent, SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        %MarkerBodyPose - pose of rigid body associated with the marker
        MarkerBodyPose
    end
    
    % Properties that correspond to dependent properties on the main object
    properties (Access = ?robotics.manip.internal.InternalAccess)
        
        %Constraints - Cell array of constraints on inverse kinematics
        Constraints
        
        %MarkerBodyName - Associated rigid body name for interactive marker
        MarkerBodyName
        
        %MarkerControlMethod - Type of interactive control for body
        MarkerControlMethod
        
        %MarkerScaleFactor - A scalar indicating the relative scale of the marker
        MarkerScaleFactor
        
        %ShowMarker - Boolean flag indicating whether to show marker
        ShowMarker
        
        %Configuration - Current configuration of rigidBodyTree robot model
        Configuration
        
        %StoredConfigurations - Array of stored robot configurations
        StoredConfigurations
        
        %Frames - Character array indicating whether tree is plotted with frames on or off
        Frames
    end
    
    methods
        function obj = InteractiveRBTBackend(tree, ikSolver, ...
                solverPoseWeights, constraints, ...
                markerBodyName, markerControlMethod, markerScaleFactor, showMarker, ...
                config, storedConfigs, frameStatus)
            %InteractiveRBTBackend Object constructor
            
            % rigidBodyTree input validation
            obj.RigidBodyTree = tree;
            obj.IKSolver = ikSolver;
            obj.Constraints = constraints;
            obj.MarkerBodyName = markerBodyName;
            obj.MarkerControlMethod = markerControlMethod;
            obj.MarkerScaleFactor = markerScaleFactor;
            obj.ShowMarker = showMarker;
            obj.Configuration = config;
            obj.StoredConfigurations = storedConfigs;
            obj.Frames = frameStatus;
            
            % Initialize GIK Pose target
            obj.GIKPoseTarget = constraintPoseTarget(obj.MarkerBodyName);
            obj.GIKPoseTarget.Weights = solverPoseWeights;
        end
    end
        
    %% Dependent properties
    methods 
        function markerBodyPose = get.MarkerBodyPose(obj)
            %get.MarkerBodyPose Get method for MarkerBodyPose
            %   The pose of the rigid body associated with the marker is
            %   always computed using the current configuration of the
            %   associated rigidBodyTree.
            
            markerBodyPose = getTransform(obj.RigidBodyTree, obj.Configuration, obj.MarkerBodyName);
        end
    end
    
    %% Methods
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
        
        function updatePoseTarget(obj)
            %updatePoseTarget Update the GIKPoseTarget property
            %   Update the primary pose target constraint used to compute
            %   the rigidBodyTree configuration.
            
            poseTgt = constraintPoseTarget(obj.MarkerBodyName);
            poseTgt.TargetTransform = obj.MarkerBodyPose;
            poseTgt.Weights = obj.GIKPoseTarget.Weights;
            obj.GIKPoseTarget = poseTgt; 
        end
        
        function updateIKConstraints(obj)
            %updateIKConstraints Update GIK constraint types
            
            inputConstraintTypes = generateConstraintTypes(obj.Constraints);
            obj.IKSolver.release;
            obj.IKSolver.ConstraintInputs = [{'pose'} inputConstraintTypes];
        end
        
        function newConfig = updateIK(obj, currConfig, targetPose)
            %updateIK Update built-in position and orientation targets
            %   Set the target pose of the primary GIK PoseTarget
            %   constraint object to a specified pose using the current
            %   configuration as an initial guess.
            
            obj.GIKPoseTarget.TargetTransform = targetPose;
            
            % Solve GIK with applied constraints and update pose
            newConfig = obj.IKSolver(currConfig, obj.GIKPoseTarget, obj.Constraints{:});
            obj.Configuration = newConfig;
            
        end
    end
end

function cTypes = generateConstraintTypes(constraintTargets)
    %generateConstraintTypes Generate constraint types from a cell array of GIK constraint objects
    %   The GIK solver acknowledges constraints via two properties:
    %   the Constraints property, which contains a cell array of
    %   valid constraint objects, and the ConstraintInputs
    %   property, which is a corresponding cell array of strings
    %   indicating the constraint type. This function maps user-provided
    %   constraints to their associated type strings.
    
cTypes = cell(1, numel(constraintTargets));
for i = 1:numel(cTypes)
    switch class(constraintTargets{i})
        case 'constraintAiming'
            cTypes{i} = 'aiming';
        case 'constraintCartesianBounds'
            cTypes{i} = 'cartesian';
        case 'constraintJointBounds'
            cTypes{i} = 'joint';
        case 'constraintOrientationTarget'
            cTypes{i} = 'orientation';
        case 'constraintPositionTarget'
            cTypes{i} = 'position';
        case 'constraintPoseTarget'
            cTypes{i} = 'pose';
        case 'constraintRevoluteJoint'
            cTypes{i} = 'revolutejoint';
        case 'constraintPrismaticJoint'
            cTypes{i} = 'prismaticjoint';
        case 'constraintFixedJoint'
            cTypes{i} = 'fixedjoint';
    end
end

end

