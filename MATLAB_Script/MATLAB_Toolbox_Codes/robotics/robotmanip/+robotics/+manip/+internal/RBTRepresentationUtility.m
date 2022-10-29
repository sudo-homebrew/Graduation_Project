classdef RBTRepresentationUtility < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %RBTRepresentationUtility - Utility functions for using RigidBodyTrees
    % as function inputs for Simulink and Codegen
    
    %   Copyright 2017-2021 The MathWorks, Inc.
    
    %#codegen
    
    methods (Static)
        function rigidBodyTreeStruct = populateRigidBodyTreeStruct(tree, skipCollisionData)
            %populateRigidBodyTreeStruct Convert a tree to a RigidBodyTree struct to be used in Simulink and Codegen
            %   A structure generated with this utility can be converted 
            %   back to a robotics.manip.internal.RigidBodyTree object. 
            %   skipCollisionData is an optional logical argument that
            %   decides whether to include collision object data in the RBT
            %   struct. skipCollisionData is true by default.
            %
            %   For example, the following sequence creates a structure 
            %   using this utility, and then converts it back to the internal
            %   RigidBodyTree representation:
            %
            %   tree = robotics.manip.internal.robotplant.kuka_lbr_iiwa_14_R820;
            %   treeStruct = robotics.manip.internal.RBTRepresentationUtility.populateRigidBodyTreeStruct(tree);
            %   robotics.manip.internal.RigidBodyTree(treeStruct.NumBodies, treeStruct);
            
            narginchk(1, 2);
            if nargin == 1
                skipCollisionData = true;
            end
            treeInternal = tree.TreeInternal;
            
            if treeInternal.NumBodies > 0
                maxBodyNameLength = max(cellfun(@length,[treeInternal.BodyNames, treeInternal.BaseName]));
                maxJointNameLength = max(cellfun(@(body)length(body.Joint.Name),treeInternal.Bodies(1:treeInternal.NumBodies)));
            else
                maxBodyNameLength = length(treeInternal.BaseName);
                maxJointNameLength = 1;
            end       

            %Initialize collision struct
            emptyCollisionGeomStruct = robotics.manip.internal.CollisionGeomStruct.getEmptyCollisionGeomStruct;
            
            % Initialize Body structure
            emptyBodyStruct = struct('NameLength',0,...
                'Name',uint8(zeros(maxBodyNameLength,1)),...
                'ParentIndex',0,...
                'NumChildren',0,...
                'ChildrenIndices',zeros(treeInternal.MaxNumBodies,1),...
                'Mass',0,...
                'CenterOfMass',zeros(3,1),...
                'Inertia',zeros(3,3),...
                'SpatialInertia',zeros(6,6), ...
                'CollisionGeom',[]);
            
            % Initialize Joint structure
            emptyJointStruct = struct('Type',uint8(0),...
                'NameLength',0,...
                'Name',uint8(zeros(maxJointNameLength,1)),...
                'VelocityNumber',0,...
                'PositionNumber',0,...
                'MotionSubspace',zeros(6,6),...
                'JointAxis',zeros(3,1),...
                'PositionLimits',zeros(7,2),...
                'HomePosition',zeros(7,1),...
                'JointToParentTransform',zeros(4,4),...
                'ChildToJointTransform',zeros(4,4)); 
            
            emptyBodyStructArray = cell(treeInternal.NumBodies+1, 1);
            for i=1:treeInternal.NumBodies+1
                emptyBodyStructArray{i} = emptyBodyStruct;
            end
            
            % Initialize Rigid Body Tree structure
            rigidBodyTreeStruct = struct('NumBodies',0,...
                'MaxNumBodies', 0, ...
                'Gravity',[0; 0; 0],...
                'NumNonFixedBodies',0,...
                'PositionNumber',0,...
                'VelocityNumber',0,...
                'PositionDoFMap',zeros(treeInternal.MaxNumBodies,2),...
                'VelocityDoFMap',zeros(treeInternal.MaxNumBodies,2),...
                'MaxNameLength',0,...
                'MaxJointPositionNumber',0,...
                'DataFormat',1,...
                'JointPositionLimits',zeros(treeInternal.MaxNumBodies,2),...
                'Bodies',{emptyBodyStructArray},...
                'Joints',repmat(emptyJointStruct, treeInternal.NumBodies+1, 1));
            
            % Populate RigidBodyTree structure fields
            rigidBodyTreeStruct.MaxNumBodies = treeInternal.MaxNumBodies;
            rigidBodyTreeStruct.NumBodies = treeInternal.NumBodies;
            rigidBodyTreeStruct.Gravity(:) = treeInternal.Gravity(:);
            rigidBodyTreeStruct.NumNonFixedBodies = treeInternal.NumNonFixedBodies;
            rigidBodyTreeStruct.PositionNumber = treeInternal.PositionNumber;
            rigidBodyTreeStruct.VelocityNumber = treeInternal.VelocityNumber;
            rigidBodyTreeStruct.PositionDoFMap = treeInternal.PositionDoFMap;
            rigidBodyTreeStruct.VelocityDoFMap = treeInternal.VelocityDoFMap;
            rigidBodyTreeStruct.MaxNameLength = treeInternal.MaxNameLength;
            rigidBodyTreeStruct.MaxJointPositionNumber = treeInternal.MaxJointPositionNumber;
            switch treeInternal.DataFormat
                case 'row'
                    rigidBodyTreeStruct.DataFormat = treeInternal.DATA_FORMAT_ROW;
                case 'column'
                    rigidBodyTreeStruct.DataFormat = treeInternal.DATA_FORMAT_COLUMN;
                otherwise % case 'struct'
                    rigidBodyTreeStruct.DataFormat = treeInternal.DATA_FORMAT_STRUCT;
            end
            rigidBodyTreeStruct.JointPositionLimits = treeInternal.JointPositionLimits;
            
            % Populate Body and Joint structures
            rigidBodyTreeStruct.Bodies{1} = ...
                robotics.manip.internal.RBTRepresentationUtility.populateRigidBodyStruct(treeInternal.Base, emptyBodyStruct, emptyCollisionGeomStruct, skipCollisionData);
            for i = 1:treeInternal.NumBodies
                rigidBodyTreeStruct.Bodies{i+1} = ...
                    robotics.manip.internal.RBTRepresentationUtility.populateRigidBodyStruct(treeInternal.Bodies{i}, emptyBodyStruct, emptyCollisionGeomStruct, skipCollisionData);
                rigidBodyTreeStruct.Joints(i+1) = ...
                    robotics.manip.internal.RBTRepresentationUtility.populateJointStruct(treeInternal.Bodies{i}.Joint, emptyJointStruct);
            end
        end
        
        function rigidBodyStruct = populateRigidBodyStruct(body, emptyBodyStruct, emptyCollisionGeomStruct, skipCollisionData)
            %populateRigidBodyStruct Convert a body to a body struct
            %   Convert rigidBody to a rigidBody structure for use with 
            %   Simulink and Codegen. emptyBodyStruct is a bare bone 
            %   rigidBody struct that is populated and returned as 
            %   rigidBodyStruct. skipCollisionData is a logical flag that 
            %   decides whether to populate collision data or not. 
            rigidBodyStruct = emptyBodyStruct;
            
            rigidBodyStruct.ParentIndex = body.ParentIndex;
            rigidBodyStruct.NameLength = length(body.Name);
            rigidBodyStruct.Name(1:rigidBodyStruct.NameLength) = uint8(body.Name);
            rigidBodyStruct.NumChildren = nnz(body.ChildrenIndices);
            rigidBodyStruct.ChildrenIndices(1:numel(body.ChildrenIndices)) = ...
                body.ChildrenIndices(:);
            if body.Index > 0
                rigidBodyStruct.Mass = body.Mass;
                rigidBodyStruct.CenterOfMass(:) = body.CenterOfMass(:);
                rigidBodyStruct.Inertia(:) = body.InertiaInternal(:);
                rigidBodyStruct.SpatialInertia(:) = body.SpatialInertia(:);
            end

            if ~skipCollisionData
                rigidBodyStruct.CollisionGeom = repmat(emptyCollisionGeomStruct, 1, body.CollisionsInternal.Size);
                rigidBodyStruct.CollisionGeom = body.CollisionsInternal.extractStruct();
            end
            
        end
        
        function jointStruct = populateJointStruct(joint, emptyJointStruct)
            %populateJointStruct Convert joint to Joint structure
            %   Convert rigidBodyJoint to a rigidBodyJoint structure for
            %   use with Simulink and Codegen. emptyJointStruct is a bare
            %   bone rigidBodyJoint struct that gets populated and returned
            %   as jointStruct.
            jointStruct = emptyJointStruct;
            
            % Define joint type
            switch joint.Type
                case 'fixed'
                    jointStruct.Type = uint8(0);
                case 'revolute'
                    jointStruct.Type = uint8(1);
                otherwise % 'prismatic'
                    jointStruct.Type = uint8(2);
            end
            
            % Joint name is a character vector
            jointStruct.NameLength = length(joint.Name);
            jointStruct.Name(1:jointStruct.NameLength) = uint8(joint.Name);
            
            % Get joint velocity and position numbers
            jointStruct.VelocityNumber = joint.VelocityNumber;
            jointStruct.PositionNumber = joint.PositionNumber;
            
            % The following items are bounded by the maximum velocity and
            % position number, so a constant vector of that length is used.
            jointStruct.MotionSubspace(:, 1:joint.VelocityNumber) = ...
                joint.MotionSubspace(:, 1:joint.VelocityNumber);
            jointStruct.JointAxis(:) = joint.JointAxisInternal;
            jointStruct.PositionLimits(1:joint.PositionNumber,:) = ...
                joint.PositionLimits(1:joint.PositionNumber,:);
            jointStruct.HomePosition(1:joint.PositionNumber) = ...
                joint.HomePosition(1:joint.PositionNumber);
            
            % Get joint transforms
            jointStruct.JointToParentTransform = joint.JointToParentTransform;
            jointStruct.ChildToJointTransform = joint.ChildToJointTransform;
        end
    end
end
