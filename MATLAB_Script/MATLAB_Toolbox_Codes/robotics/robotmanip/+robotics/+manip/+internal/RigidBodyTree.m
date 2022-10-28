classdef RigidBodyTree < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %RIGIDBODYTREE Internal implementation for rigidBodyTree
    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    %#codegen
    
    
    properties(SetAccess = {?robotics.manip.internal.InternalAccess})
        %NumBodies Number of rigid bodies in the robot
        NumBodies
        
        %Base Base of the robot
        Base
    end
    
    properties(Access = ?robotics.manip.internal.InternalAccess)
        %FastVisualizationHelper Helper class for FastUpdate
        FastVisualizationHelper
    end
    
    properties (Dependent)
        %BodyNames Names of all rigid bodies in the robot
        BodyNames
        
        %BaseName Name of the base of the robot
        BaseName
    end
    
    properties
        %Gravity Gravitational Acceleration the robot experiences
        Gravity
    end
    
    properties (Access = {?robotics.manip.internal.InternalAccess})
        
        %Bodies
        Bodies
        
        %NumBodies Number of rigid bodies with non-fixed joint in the robot
        NumNonFixedBodies
        
        %PositionNumber Robot's position DoF
        PositionNumber
        
        %VelocityNumber Robot's velocity (real) DoF
        VelocityNumber
        
        %PositionDoFMap A map that keeps track of joint position
        %   indices in the configuration (q) vector
        PositionDoFMap
        
        %VelocityDoFMap A map that keeps track of joint velocity
        %   (acceleration) indices in the qdot vector
        VelocityDoFMap
        
        %MaxNumBodies Upper bound on number of bodies allowed in the tree.
        %   This is a non-tunable property and is useful when generating code.
        %   In MATLAB execution, MaxNumBodies is not required and is ignored
        %   if specified.
        MaxNumBodies
        
        %MaxNameLength Maximum length of body/joint name char vector
        MaxNameLength
        
        %MaxJointPositionNumber Maximum joint position dof
        MaxJointPositionNumber
        
        %VisualizationInfo
        VisualizationInfo
        
        %DataFormatInternal This property determines input and output data format
        %   for all the kinematics and dynamics functions.
        DataFormatInternal
    end
    
    properties (Dependent, Access = {?robotics.manip.internal.InternalAccess})
        
        %JointPositionLimits A nx2 matrix that collects all joint limits
        JointPositionLimits
        
        %ShowTag Tag to identify graphic objects
        ShowTag
        
        %EstimatedMaxReach Estimated max reach of the manipulator
        %   The max reach of a manipulator is the maximum of the three
        %   coordinates (absolute value) of the farthest possible
        %   point on the robot away from the base origin.
        %
        %   Default: 1 (meter)
        EstimatedMaxReach
        
        %IsMaxReachUpToDate A boolean indicator that shows whether the
        %   current EstimatedMaxReach is up-to-date
        IsMaxReachUpToDate
        
        %DataFormat
        DataFormat
    end
    
    properties (Constant, Hidden)
        DATA_FORMAT_ROW = uint8(0);
        DATA_FORMAT_COLUMN = uint8(1);
        DATA_FORMAT_STRUCT = uint8(2);
    end
    
    
    methods
        function obj = RigidBodyTree(maxNumBodies, input2)
            %RigidBodyTree Constructor
            %   OBJ = RigidBodyTree() Creates a default rigid body tree. This
            %   constructor is only useful in MATLAB and not in
            %   code-generation, because only in MATLAB can Bodies cell-array
            %   be dynamically extended. During code-generation a compile time
            %   constant defining the upper-bound on the capacity of the Bodies
            %   cell-array is required.
            %
            %   OBJ = RigidBodyTree(MAXNUMBODIES) Creates a tree with
            %   MAXNUMBODIES rigid bodies. In MATLAB, the invariant is
            %   obj.MaxNumBodies=max(obj.MaxNumBodies,obj.NumBodies), i.e.,
            %   with every addition of a new body the maximum number of bodies
            %   may increase. During code generation, however, it is required
            %   at compile time that we know the maximum number of bodies the
            %   Bodies cell-array can hold i.e., the maximum capacity of the
            %   Bodies cell-array.
            %
            %   OBJ = RigidBodyTree(MAXNUMBODIES, RIGIDBODYTREESTRUCT) Creates
            %   a tree from a struct RIGIDBODYTREESTRUCT that captures the
            %   properties of a rigid body tree in a struct-like format
            %   compatible with Simulink.
            
            if (nargin < 2) || (~isstruct(input2))
                if(nargin > 0)
                    obj.MaxNumBodies = maxNumBodies;
                else
                    obj.MaxNumBodies = 0;
                end
                if nargin > 1
                    dataFormat = input2;
                end
                if ~coder.target('MATLAB')
                    coder.internal.assert(nargin>0, 'robotics:robotmanip:rigidbodytree:MaxNumBodiesRequiredForCodegen');
                    coder.internal.assert(isnumeric(maxNumBodies)&& ...
                        isscalar(maxNumBodies)&& ...
                        ~isempty(maxNumBodies)&& ...
                        (maxNumBodies>0), 'robotics:robotmanip:rigidbodytree:InvalidMaxNumBodies');
                end
                
                if nargin > 1
                    obj.DataFormat = dataFormat;
                else
                    obj.DataFormat = 'struct';
                end
                
                obj.MaxNameLength = 200;
                obj.MaxJointPositionNumber = 7;
                
                obj.Base = robotics.manip.internal.RigidBody('base');
                obj.Base.Index = 0;
                
                obj.VisualizationInfo = robotics.manip.internal.VisualizationInfo();
                obj.IsMaxReachUpToDate = false;
                obj.Gravity = [0 0 0];
                
                obj.FastVisualizationHelper = robotics.manip.internal.FastVisualizationHelper;
                
                clearAllBodies(obj);
            else
                %Directly instantiate an internal RigidBodyTree from a
                %structure using the Simulink-compatible format
                rigidBodyTreeStruct = input2;
                
                %DataFormat will be that stored in the RBT struct
                dataFormats = {'row', 'column', 'struct'};
                obj.DataFormat = dataFormats{rigidBodyTreeStruct.DataFormat+1};
                obj.MaxNameLength = 200;
                obj.MaxJointPositionNumber = 7;
                
                obj.MaxNumBodies = maxNumBodies;
                obj.NumBodies = rigidBodyTreeStruct.NumBodies;
                
                defaultInitializeBodiesCellArray(obj);

                for i = coder.unroll(1: rigidBodyTreeStruct.NumBodies)  % unroll due to codegen limitation
                    obj.Bodies{i} = robotics.manip.internal.RigidBody(...
                        rigidBodyTreeStruct.Bodies{i+1}, ...
                        rigidBodyTreeStruct.Joints(i+1), ...
                        false);
                    obj.Bodies{i}.Index = i;
                end

                obj.Gravity = rigidBodyTreeStruct.Gravity(:);
                obj.NumNonFixedBodies = rigidBodyTreeStruct.NumNonFixedBodies;
                obj.PositionNumber = rigidBodyTreeStruct.PositionNumber;
                obj.VelocityNumber = rigidBodyTreeStruct.VelocityNumber;
                obj.PositionDoFMap = rigidBodyTreeStruct.PositionDoFMap;
                obj.VelocityDoFMap = rigidBodyTreeStruct.VelocityDoFMap;
                
                %Creating the base rigidBody using the struct version of
                %the rigidBody constructor
                obj.Base = robotics.manip.internal.RigidBody(rigidBodyTreeStruct.Bodies{1}, rigidBodyTreeStruct.Joints(1), true);
                obj.Base.Index = 0;
            end
        end
        
        
        
        function bodyhandle = getBody(obj, bodyname)
            %getBody Get robot's body handle by name
            
            if strcmp(obj.BaseName, bodyname)
                robotics.manip.internal.error('rigidbodytree:UseBaseProperty');
            end
            
            bid = validateInputBodyName(obj, bodyname);
            bodyhandle = obj.Bodies{bid};
            
        end
        
        
        function addBody(obj, bodyin, parentName)
            %addBody Add a body to the robot
            
            narginchk(3,3);
            
            % validate input arguments
            validateattributes(bodyin, {'robotics.manip.internal.RigidBody'}, ...
                {'scalar', 'nonempty'}, 'addBody','bodyin');
            
            bid = obj.findBodyIndexByName(bodyin.Name);
            
            % check body name collision
            if (bid > -1)
                robotics.manip.internal.error('rigidbodytree:BodyNameCollision', bodyin.Name);
            end
            
            % check if the designated parent exists in robot
            pid = validateInputBodyName(obj, parentName);
            
            % check joint name collision
            bid2 = obj.findBodyIndexByJointName(bodyin.Joint.Name);
            if (bid2 > 0)
                robotics.manip.internal.error('rigidbodytree:JointNameCollision', bodyin.Joint.Name);
            end
            
            % update indices
            % Note: for codegen to recognize that obj.Bodies{:}
            % keeps "ChildrenIndices" assigned it requires that we
            % have the RHS of "obj.Bodies{index} = ..." have
            % "ChildrenIndices" assigned.  To ensure this we assign
            % to body.ChildrenIndices before assigning it to
            % obj.Bodies.
            index = obj.NumBodies + 1;
            body = copy(bodyin);
            if(~coder.target('MATLAB'))
                body.ChildrenIndices = zeros(1,obj.MaxNumBodies);
            else
                % This change is required for backwards compatibility with
                % R2016b robot MAT files in exampleRobots2016b. This should be
                % removed if the test is refactored. Remove this branch once
                % g2594187 is resolved.
                body.ChildrenIndices = zeros(1,0);
            end
            obj.Bodies{index} = body;
            body.Index = index;
            body.ParentIndex = pid;
            
            if pid > 0
                parent = obj.Bodies{pid};
            else
                parent = obj.Base;
            end
            parent.ChildrenIndices(body.Index) = 1;
            
            % mark joint for InTree
            body.JointInternal.InTree = true;
            
            % update robot numbers
            obj.NumBodies = obj.NumBodies + 1;
            if(coder.target('MATLAB'))
                obj.MaxNumBodies = max(obj.MaxNumBodies, obj.NumBodies);
            end
            if ~strcmp(body.Joint.Type,'fixed')
                obj.NumNonFixedBodies = obj.NumNonFixedBodies + 1;
                obj.PositionDoFMap(body.Index,:) = [obj.PositionNumber+1, obj.PositionNumber+body.Joint.PositionNumber];
                obj.VelocityDoFMap(body.Index,:) = [obj.VelocityNumber+1, obj.VelocityNumber+body.Joint.VelocityNumber];
            else
                obj.PositionDoFMap(body.Index,:) = [0 -1];
                obj.VelocityDoFMap(body.Index,:) = [0 -1];
            end
            
            % DoF
            obj.PositionNumber = obj.PositionNumber + body.Joint.PositionNumber;
            obj.VelocityNumber = obj.VelocityNumber + body.Joint.VelocityNumber;
            
            obj.IsMaxReachUpToDate = false;
            
        end
        
        function nuIdsOut = findBodyIndicesInSubtree(obj, bid)
            %findBodyIndicesInSubtree Find IDs for all bodies in the subtree of
            %   body with index BID (not including BID).
            body = obj.Bodies{bid};
            nuIds = zeros(obj.NumBodies,1);
            k = 1;
            j = 1;
            % collect the downstream body IDs in BFS order
            cids = find(int32(body.ChildrenIndices));
            if numel(cids) > 0
                nuIds(k:k+numel(cids) -1) = cids;
                k = k + numel(cids);
            end
            while j < k
                tmp = find(int32(obj.Bodies{nuIds(j)}.ChildrenIndices));
                if numel(tmp) > 0
                    nuIds(k:k+numel(tmp)-1) = tmp;
                    k = k + numel(tmp);
                end
                j = j + 1;
            end
            nuIdsOut = nuIds(1:k-1);
        end
        
        function replaceJoint(obj, bodyname, joint)
            %replaceJoint Replace the joint of one of robot's body
            
            validateattributes(joint, {'rigidBodyJoint'}, {'scalar'}, ...
                'replaceJoint', 'joint');
            
            bid1 = validateInputBodyName(obj, bodyname);
            % error if attempting to switch joint for base
            if bid1 == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Joint');
            end
            
            % check for joint name duplication, if the joint's name happens
            % to be the same as the joint name of the body (with name
            % 'bodyname'), it's not considered joint name collision.
            bid2 = obj.findBodyIndexByJointName(joint.Name);
            if (bid2 > 0) && ~strcmp(bodyname, obj.Bodies{bid2}.Name)
                robotics.manip.internal.error('rigidbodytree:JointNameCollision', joint.Name);
            end
            
            body = obj.Bodies{bid1};
            oldJointIsFixed = strcmp(body.Joint.Type, 'fixed');
            newJointIsFixed = strcmp(joint.Type, 'fixed');
            pn1 = body.Joint.PositionNumber;
            vn1 = body.Joint.VelocityNumber;
            pn2 = joint.PositionNumber;
            vn2 = joint.VelocityNumber;
            
            % swap in new joint
            body.JointInternal = copy(joint);
            body.JointInternal.InTree = true; % mark InTree
            
            % update robot internal numbers
            obj.NumNonFixedBodies = obj.NumNonFixedBodies + double(oldJointIsFixed) - double(newJointIsFixed);
            obj.PositionNumber = obj.PositionNumber + pn2 - pn1;
            obj.VelocityNumber = obj.VelocityNumber + vn2 - vn1;

            % update DOF maps
            subtreeBodyIds = [bid1; obj.findBodyIndicesInSubtree(bid1)]';

            for i = 1:numel(subtreeBodyIds)
                id = subtreeBodyIds(i);
                if strcmp(obj.Bodies{id}.Joint.Type, 'fixed')
                    obj.PositionDoFMap(id,:) = [0,-1];
                    obj.VelocityDoFMap(id,:) = [0,-1];
                else
                    obj.PositionDoFMap(id, :) = obj.PositionDoFMap(id, :) + pn2 - pn1;
                    obj.VelocityDoFMap(id, :) = obj.VelocityDoFMap(id, :) + vn2 - vn1;
                end
            end

            obj.IsMaxReachUpToDate = false;
        end
        
        
        function replaceBody(obj, bodyname, newbody)
            %replaceBody Replace a body in the robot
            
            bid1 = validateInputBodyName(obj, bodyname);
            if bid1 == 0
                robotics.manip.internal.error('rigidbodytree:BaseNotReplaceable');
            end
            body = obj.Bodies{bid1};
            
            validateattributes(newbody, {'robotics.manip.internal.RigidBody'}, ...
                {'scalar', 'nonempty'},'replaceBody', 'newbody');
            
            % check for body name collision
            bid2 = obj.findBodyIndexByName(newbody.Name);
            
            
            if bid2 < 0 || ... % if the new body has a unique name
                    ( bid2 > 0 && strcmp(newbody.Name, bodyname) )% or if newbody name is the same as bodyname, it's not considered collision.
                % replace the joint
                obj.replaceJoint(body.Name, newbody.Joint);
                body.NameInternal = newbody.Name;
                
                % update other dynamics properties
                body.MassInternal = newbody.Mass;
                body.CenterOfMassInternal = newbody.CenterOfMass;
                body.Inertia = newbody.Inertia;
                if coder.target('matlab')
                    body.VisualsInternal = newbody.VisualsInternal;
                end
                body.CollisionsInternal = copy(newbody.CollisionsInternal);
            else
                robotics.manip.internal.error('rigidbodytree:BodyNameCollision', newbody.Name);
            end
            
            obj.IsMaxReachUpToDate = false;
            
        end
        
        
        
        function newrobot = removeBody(obj, bodyname)
            %removeBody Remove a body from robot
            
            % error if the body to be removed is base
            bid = validateInputBodyName(obj, bodyname);
            if bid == 0
                robotics.manip.internal.error('rigidbodytree:BaseNotRemovable');
            end
            
            % calling subtree, identify all the affected bodies
            if nargout > 0
                [bodyIndices, newrobot] = subtreeInternal(obj, bid);
            else
                bodyIndices = subtreeInternal(obj, bid);
            end
            
            % identify the remaining bodies
            mask = ones(1, obj.NumBodies, 'int32');
            for i = 1:length(bodyIndices)
                mask(bodyIndices(i)) = 0;
            end
            bodyIndicesToKeep = find(mask);
            
            % copy all the remaining bodies to a temporary robot
            if coder.target('MATLAB')
                tempBot = robotics.manip.internal.RigidBodyTree();
                tempBot.BaseName = obj.Base.Name;
                for i = 1:length(bodyIndicesToKeep)
                    bid = bodyIndicesToKeep(i);
                    body = obj.Bodies{bid};
                    pid = body.ParentIndex;
                    if pid > 0
                        parent = obj.Bodies{pid};
                    else
                        parent = obj.Base;
                    end
                    tempBot.addBody(body, parent.Name);
                end
            else
                tempBot = robotics.manip.internal.RigidBodyTree(obj.MaxNumBodies);
                tempBot.BaseName = obj.Base.Name;
                for i = coder.unroll(1:obj.MaxNumBodies)
                    if i <= length(bodyIndicesToKeep)
                        bid = bodyIndicesToKeep(i);
                        body = obj.Bodies{bid};
                        pid = body.ParentIndex;
                        if pid > 0
                            parent = obj.Bodies{pid};
                        else
                            parent = obj.Base;
                        end
                        tempBot.addBody(body, parent.Name);
                    end
                end
            end
            
            % clear the original robot
            clearAllBodies(obj);
            
            % re-add remaining bodies from temporary robot
            if coder.target('MATLAB')
                for i = 1:tempBot.NumBodies
                    body = tempBot.Bodies{i};
                    pid = body.ParentIndex;
                    if pid > 0
                        parent = tempBot.Bodies{pid};
                    else
                        parent = tempBot.Base;
                    end
                    obj.addBody(body, parent.Name);
                end
            else
                for i = coder.unroll(1:obj.MaxNumBodies) % original sequence
                    if i <= tempBot.NumBodies
                        body = tempBot.Bodies{i};
                        pid = body.ParentIndex;
                        if pid > 0
                            parent = tempBot.Bodies{pid};
                        else
                            parent = tempBot.Base;
                        end
                        obj.addBody(body, parent.Name);
                    end
                end
            end
            
            
        end
        
        
        function newrobot = subtree(obj, bodyname)
            %SUBTREE Get a subtree from robot
            
            if strcmp(bodyname, obj.Base.Name)
                newrobot = copy(obj);
            else
                bid = validateInputBodyName(obj, bodyname);
                [~, newrobot] = subtreeInternal(obj, bid);
            end
        end
        
        function newrobot = copy(obj, varargin)
            %COPY Copy the rigid body tree
            
            % A second input may be provided indicating the data format.
            % This allows a rigidBodyTree.TreeInternal to be
            % converted to one with a different data format in a codegen
            % compatible way. That TreeInternal can further be converted to
            % user-facing RBT with the constructor:
            % rigidBodyTree(treeInternal)
            narginchk(1,2)
            if nargin > 1
                dataFormat = varargin{1};
            else
                dataFormat = obj.DataFormat;
            end
            
            % Create a new robot and populate it
            newrobot = robotics.manip.internal.RigidBodyTree(obj.MaxNumBodies, dataFormat);
            newrobot.BaseName = obj.BaseName;
            if coder.target('matlab')
                newrobot.Base.VisualsInternal = obj.Base.VisualsInternal;
            end
            newrobot.Base.CollisionsInternal = copy(obj.Base.CollisionsInternal);
            newrobot.Gravity = obj.Gravity;
            
            % Do not copy obj.VisualizationInfo
            
            for i  = coder.unroll(1:obj.MaxNumBodies)
                if i<=obj.NumBodies
                    body = obj.Bodies{i};
                    pid = body.ParentIndex;
                    if pid > 0
                        parent = obj.Bodies{pid};
                    else
                        parent = obj.Base;
                    end
                    newrobot.addBody(body, parent.Name);
                end
            end
            
        end
        
        
        function addSubtree(obj, bodyname, subtreerobot)
            %addSubtree Add a subtree to current robot
            
            validateattributes(subtreerobot, {'robotics.manip.internal.RigidBodyTree'},...
                {'scalar'}, 'addSubtree', 'subtreerobot');
            
            bid = validateInputBodyName(obj, bodyname); %#ok<NASGU>
            
            
            % check name collision up front
            for i = 1:subtreerobot.NumBodies
                bn = subtreerobot.Bodies{i}.Name;
                if findBodyIndexByName(obj, bn) > -1
                    robotics.manip.internal.error('rigidbodytree:BodyNameCollision', bn);
                end
            end
            
            % add the subtree body by body
            for i = coder.unroll(1:subtreerobot.MaxNumBodies)
                if i <= subtreerobot.NumBodies
                    pid = subtreerobot.Bodies{i}.ParentIndex;
                    if pid > 0
                        pname = subtreerobot.Bodies{pid}.Name;
                    else
                        pname = bodyname;
                    end
                    
                    obj.addBody(subtreerobot.Bodies{i}, pname);
                end
            end
            
        end
        
        
        function q = homeJointPositions(obj)
            %homeJointPositions Return the home configuration for robot in
            %   plain vector
            
            
            q = zeros(obj.PositionNumber, 1);
            
            for i = 1:obj.NumBodies
                p = obj.PositionDoFMap(i,:);
                if p(1) <= p(2)
                    qi = obj.Bodies{i}.JointInternal.HomePosition;
                    q(p(1):p(2)) = qi(:);
                end
            end
            
        end
        
        function q = randomJointPositions(obj)
            %randomJointPositions Return a random configuration for robot in
            %   plain vector
            
            q = zeros(obj.PositionNumber, 1);
            
            for i = 1:obj.NumBodies
                p = obj.PositionDoFMap(i,:);
                if p(1) <= p(2)
                    qi = obj.Bodies{i}.JointInternal.randomPosition();
                    q(p(1):p(2)) = qi(:);
                end
            end
            
        end
        
        function Q = homeConfiguration(obj)
            %homeConfiguration Return the home configuration for robot
            
            if strcmp(obj.DataFormatInternal, 'struct')
                Q = repmat(struct('JointName','', 'JointPosition', 0), 1, obj.NumNonFixedBodies);
                coder.varsize('Q.JointName', [1, obj.MaxNameLength], [false, true]);
                coder.varsize('Q.JointPosition', [1, obj.MaxJointPositionNumber], [false, true]);
                
                k = 1;
                for i = 1:obj.NumBodies
                    body = obj.Bodies{i};
                    if ~strcmp(body.JointInternal.Type, 'fixed')
                        Q(k).JointName = body.JointInternal.Name;
                        Q(k).JointPosition = body.JointInternal.HomePosition;
                        k = k + 1;
                    end
                end
            else % plain vector
                Q_ = homeJointPositions(obj);
                Q = resultPostProcess(obj, Q_);
            end
        end
        
        
        function Q = randomConfiguration(obj)
            %randomConfiguration Return a random configuration for robot
            
            if strcmp(obj.DataFormatInternal, 'struct')
                Q = repmat(struct('JointName','', 'JointPosition', 0), 1, obj.NumNonFixedBodies);
                coder.varsize('Q.JointName', [1, obj.MaxNameLength], [false, true]);
                coder.varsize('Q.JointPosition', [1, obj.MaxJointPositionNumber], [false, true]);
                
                k = 1;
                for i = 1:obj.NumBodies
                    body = obj.Bodies{i};
                    if ~strcmp(body.JointInternal.Type, 'fixed')
                        Q(k).JointName = body.JointInternal.Name;
                        Q(k).JointPosition = body.Joint.randomPosition();
                        k = k + 1;
                    end
                end
            else % plain vector
                Q_ = randomJointPositions(obj);
                Q = resultPostProcess(obj, Q_);
            end
        end
        
        function Q = formatConfiguration(obj, q)
            %formatConfiguration Convert vector according to DataFormat
            if strcmp(obj.DataFormatInternal, 'struct')
                Q = repmat(struct('JointName','', 'JointPosition', 0), 1, obj.NumNonFixedBodies);
                coder.varsize('Q.JointName', [1, obj.MaxNameLength], [false, true]);
                coder.varsize('Q.JointPosition', [1, obj.MaxJointPositionNumber], [false, true]);
                
                k = 1;
                for i = 1:obj.NumBodies
                    body = obj.Bodies{i};
                    if ~strcmp(body.JointInternal.Type, 'fixed')
                        dofMap = obj.PositionDoFMap(body.Index,:);
                        Q(k).JointName = body.JointInternal.Name;
                        Q(k).JointPosition = q(dofMap(1):dofMap(2));
                        k = k + 1;
                    end
                end
            else % plain vector
                Q = resultPostProcess(obj, q);
            end
        end
        
        function posIndices = bodyIndicesToPositionIndices(obj, bodyIndices)
            %bodyIndicesToPositionIndices Return position indices for specified bodies.
            %   Returns the indices in the configuration vector for the
            %   joints that connect the specified bodies to the rigid body
            %   tree. The order of indices in posIndices corresponds to
            %   the order of bodyIndices. However, as fixed joints have
            %   no configuration variables, the length of posIndices and
            %   bodyIndices will not necessarily be the same.
            
            % Get the start-stop indices in the configuration vector for
            % each of the bodies in bodyIndices.
            positionMap = obj.PositionDoFMap(bodyIndices(bodyIndices~=0),:);
            posIndices = zeros(1,obj.PositionNumber);
            idxCount = 0;
            % All non-fixed joints have start-index <= stop-index
            for i = 1:size(positionMap,1)
                a = positionMap(i,:);
                numPositions = (a(2) - a(1)) + 1;
                if numPositions > 0
                    posIndices(idxCount + (1:numPositions)) = a(1):a(2);
                    idxCount = idxCount + numPositions;
                end
            end
            posIndices = posIndices(1:idxCount);
        end
        
        function [isColliding, separationDist, witnessPts] = ...
                checkSelfCollision(obj, tTree, baseTform, isExhaustive)
            %checkSelfCollision Utility to check robot's collision with itself
            %   Checks the self collision of the robot. The tTree is the
            %   transform tree with the transforms of all the bodies in the
            %   rigid body tree with respect to the base. The base's transform
            %   is specified explicitly in baseTform. Setting the isExhaustive
            %   flag to false will early return from the routine in case of a
            %   collision, and skip checking the remaining pairs.
            
            isColliding = false;
            numBodies = obj.NumBodies;
            separationDist = inf(numBodies + 1);
            witnessPts = inf(3 * (numBodies + 1), 2 * (numBodies + 1));
            
            %Populate the collision data for pairs with the bodies excluding the base
            for i = 1 : numBodies
                %If a collision is found, and the isExhaustive flag is false,
                %return from the routine.
                if(isColliding && ~isExhaustive)
                    return;
                end
                bodyI = obj.Bodies{i};
                tformI = tTree{i};
                bodyI.CollisionsInternal.setTransform(tformI);
                
                %Skip checking adjacent bodies, and the collision of
                %the body with itself.
                for j = i+2 : numBodies
                    bodyJ = obj.Bodies{j};
                    tformJ = tTree{j};
                    bodyJ.CollisionsInternal.setTransform(tformJ);
                    
                    [cState, sDist, wPts] = ...
                        checkCollision(...
                        bodyI.CollisionsInternal,...
                        bodyJ.CollisionsInternal);
                    
                    isColliding = isColliding || cState;
                    
                    %The witness points' column and row indices form an
                    %arithmetic progression where for rows the initial row
                    %index is 1 and the difference is 3, and for columns the
                    %initial column index is 1 and the difference is 2
                    witnessPts(...
                        1+(i-1)*3:3+(i-1)*3, 1+(j-1)*2:2+(j-1)*2) = wPts;
                    witnessPts(...
                        1+(j-1)*3:3+(j-1)*3, 1+(i-1)*2:2+(i-1)*2) = wPts;
                    
                    separationDist(i, j) = sDist;
                    separationDist(j, i) = sDist;
                end
            end
            
            %Populate the collision data for pairs with the base as the body
            %The base's separation distance or witness points from the other
            %bodies are stored in the last column or row index of separationDist
            %and witnessPts
            baseOutputIdx = numBodies+1;
            base = obj.Base;
            base.CollisionsInternal.setTransform(baseTform);
            
            %Ignore the collision of the base with the itself, and the first
            %body
            for j = 2 : numBodies
                if(isColliding && ~isExhaustive)
                    return;
                end
                bodyJ = obj.Bodies{j};
                tformJ = tTree{j};
                bodyJ.CollisionsInternal.setTransform(tformJ);
                
                [cState, sDist, wPts] = ...
                    checkCollision(base.CollisionsInternal, bodyJ.CollisionsInternal);
                
                isColliding = isColliding || cState;
                
                %The witness points' column and row indices form an
                %arithmetic progression where for rows the initial row
                %index is 1 and the difference is 3, and for columns the
                %initial column index is 1 and the difference is 2. For the
                %base, this is fixed to be at numBodies + 1.
                witnessPts(...
                    1+(baseOutputIdx-1)*3:3+(baseOutputIdx-1)*3, 1+(j-1)*2:2+(j-1)*2) = wPts;
                witnessPts(...
                    1+(j-1)*3:3+(j-1)*3, 1+(baseOutputIdx-1)*2:2+(baseOutputIdx-1)*2) = wPts;
                
                separationDist(baseOutputIdx, j) = sDist;
                separationDist(j, baseOutputIdx) = sDist;
            end
            
        end
        
        function [isColliding, separationDist, witnessPts] = ...
                checkWorldCollision(obj, tTree, baseTform, worldObjects, isExhaustive)
            %checkWorldCollision Utility to check robot's collision with world objects
            %   Checks the collision of the robot with the world objects
            %   (specified in worldObjects as a cell-array). The tTree is the
            %   transform tree with the transforms of all the bodies in the
            %   rigid body tree with respect to the base. The base's transform
            %   is specified explicitly in baseTform. Setting the isExhaustive
            %   flag to false will early return from the routine in case of a
            %   collision, and skip checking the remaining pairs.
            
            numWorldObjects = length(worldObjects);
            numBodies = obj.NumBodies;
            isColliding = false;
            separationDist = inf(numBodies + 1, numWorldObjects);
            witnessPts = inf(3 * (numBodies + 1), 2 * numWorldObjects);
            
            %Populate the collision data for body-world pairs excluding the base
            for i = 1 : numBodies
                %If a collision is found, and the isExhaustive flag is false,
                %return from the routine.
                if(isColliding && ~isExhaustive)
                    return;
                end
                body = obj.Bodies{i};
                tformI = tTree{i};
                body.CollisionsInternal.setTransform(tformI);
                %loop over the world objects
                for j = 1 : numWorldObjects
                    [cState, sDist, wPts] = ...
                        checkCollisionWithGeom(...
                        body.CollisionsInternal, worldObjects{j}, baseTform * worldObjects{j}.Pose);
                    isColliding = isColliding || cState;
                    
                    %The witness points' column and row indices form an
                    %arithmetic progression where for rows the initial row
                    %index is 1 and the difference is 3, and for columns the
                    %initial column index is 1 and the difference is 2
                    witnessPts(...
                        1+(i-1)*3:3+(i-1)*3, 1+(j-1)*2:2+(j-1)*2) = wPts;
                    separationDist(i, j) =  sDist;
                end
            end
            
            %Populate the collision data for body-world pairs with the base as
            %the body.  The base's separation distance or witness points from
            %the other world objects are stored in the last column or row index of
            %separationDist and witnessPts
            baseOutputIdx = numBodies+1;
            base = obj.Base;
            base.CollisionsInternal.setTransform(baseTform);
            
            for j = 1 : numWorldObjects
                if(isColliding && ~isExhaustive)
                    return;
                end
                [cState, sDist, wPts] = ...
                    checkCollisionWithGeom(...
                    base.CollisionsInternal, worldObjects{j}, baseTform * worldObjects{j}.Pose);
                
                isColliding = isColliding || cState;
                
                %The witness points' column and row indices form an
                %arithmetic progression where for rows the initial row
                %index is 1 and the difference is 3, and for columns the
                %initial column index is 1 and the difference is 2
                witnessPts(...
                    1+(baseOutputIdx-1)*3:3+(baseOutputIdx-1)*3, 1+(j-1)*2:2+(j-1)*2) = wPts;
                separationDist(baseOutputIdx, j) =  sDist;
            end
            
        end
        
        function [isColliding, separationDist, witnessPts] = ...
                checkCollision(obj, config, worldObjects, ignoreSelfCollision, isExhaustive)
            %checkCollision Checks collision of the rigid body tree.
            %   The internal function checks for self and world collision,
            %   separately, and based on the ignoreSelfCollision and isExhaustive
            %   flags combines the results.
            
            %Transform tree to obtain the poses of the bodies with respect to
            %the base
            tTree = obj.forwardKinematics(config);
            
            %This is the case of self-collision
            if(isempty(worldObjects))
                if(~ignoreSelfCollision)
                    baseTform = obj.getTransform(config, obj.BaseName);
                    [isColliding, separationDist, witnessPts] = ...
                        obj.checkSelfCollision(tTree, baseTform, isExhaustive);
                else
                    %This is a no operation case, as we are trying to check for
                    %self-collision, but the ignoreSelfCollision is true
                    isColliding = false;
                    separationDist = inf(obj.NumBodies + 1);
                    witnessPts = inf(3 * (obj.NumBodies + 1), 2 * (obj.NumBodies + 1));
                end
            else
                %This is the case of world-collision
                baseTform = obj.getTransform(config, obj.BaseName);
                %The case where we don't ignore self-collision
                if(~ignoreSelfCollision)
                    [isSelfColliding, selfSeparationDist, selfWitnessPts] = ...
                        obj.checkSelfCollision(tTree, baseTform, isExhaustive);
                    [isWorldColliding, worldSeparationDist, worldWitnessPts] = ...
                        obj.checkWorldCollision(tTree, baseTform, worldObjects, isExhaustive);
                    isColliding = [isSelfColliding, isWorldColliding];
                    separationDist = [selfSeparationDist, worldSeparationDist];
                    witnessPts = [selfWitnessPts, worldWitnessPts];
                else
                    %The case where we ignore self-collision and only output the
                    %results from world-collision
                    [isColliding, separationDist, witnessPts] = ...
                        obj.checkWorldCollision(tTree, baseTform, worldObjects, isExhaustive);
                end
            end
        end
        function T = getTransform(obj, Q, bodyName1, bodyName2)
            %getTransform Get the transform T that converts points originally
            %   expressed in 'bodyname' frame to another body frame (if no
            %   4th argument, it's base frame)
            
            narginchk(3,4);
            
            qvec = obj.validateConfiguration(Q);
            
            Ttree = obj.forwardKinematics(qvec);
            
            % 3-argument case: getTransform(ROBOT, Q, BODYNAME1)
            bid1 = validateInputBodyName(obj, bodyName1);
            if bid1 == 0
                T1 = eye(4);
            else
                T1 = Ttree{bid1};
            end
            
            T2 = eye(4);
            if nargin == 4
                % 4-argument case: getTransform(ROBOT, Q, BODYNAME1, BODYNAME2)
                
                bid2 = validateInputBodyName(obj, bodyName2);
                if bid2 == 0
                    T2 = eye(4);
                else
                    T2 = Ttree{bid2};
                end
            end
            
            R = T2(1:3,1:3)';
            p = -R*T2(1:3,4);
            T = [R,p;[0 0 0 1]]*T1; % the first term is inv(T2)
        end
        
        function Jac = geometricJacobian(obj, Q, endeffectorname)
            %geometricJacobian Compute the geometric Jacobian
            
            qvec = obj.validateConfiguration(Q);
            
            Ttree = forwardKinematics(obj, qvec);
            
            Jac = zeros(6, obj.VelocityNumber);
            
            chainmask = zeros(1,obj.MaxNumBodies);
            if strcmp(endeffectorname, obj.Base.Name)
                T2inv = eye(4);
                T2 = eye(4);
            else
                endeffectorIndex = validateInputBodyName(obj, endeffectorname);
                body = obj.Bodies{endeffectorIndex};
                T2 = Ttree{endeffectorIndex};
                R = T2(1:3,1:3)';
                p = -R*T2(1:3,4);
                T2inv = [R,p;[0 0 0 1]];
                
                % mark the support set of end-effector
                chainmask(endeffectorIndex) = 1;
                while body.ParentIndex>0
                    body = obj.Bodies{body.ParentIndex};
                    chainmask(body.Index) = 1;
                end
            end
            
            for i = 1:obj.NumBodies
                body = obj.Bodies{i};
                
                % only non-fixed bodies in the support set of end-effector
                if ~strcmp(body.JointInternal.Type, 'fixed') && chainmask(i)
                    
                    T1 = Ttree{body.Index};
                    T = T2inv*T1;
                    
                    Tdh = body.Joint.ChildToJointTransform;
                    R = Tdh(1:3,1:3)';
                    p = -R*Tdh(1:3,4);
                    Tdhinv = [R,p;[0 0 0 1]];
                    T = T*Tdhinv;
                    idx = obj.PositionDoFMap(i,:);
                    coder.varsize('rhs', [6, 6], [false, true]);
                    JacSlice = robotics.manip.internal.tformToSpatialXform(T)...
                        *body.Joint.MotionSubspace;
                    Jac(:,idx(1):idx(2)) = JacSlice;
                end
                
            end
            X = [T2(1:3,1:3), zeros(3);zeros(3), T2(1:3,1:3)];
            Jac = X*Jac;
        end
        
        
        
        function showdetails(obj)
            %showdetails Display details of the robot
            
            fprintf('--------------------\n');
            fprintf('Robot: (%d bodies)\n\n', int32(obj.NumBodies));
            
            widMaxBodyName = 10;
            widMaxJointName = 10;
            
            for i = 1:obj.NumBodies
                widMaxBodyName = ...
                    max(widMaxBodyName, length(obj.Bodies{i}.Name)+5);
                widMaxJointName = ...
                    max(widMaxJointName, length(obj.Bodies{i}.Joint.Name)+5);
            end
            
            fprintf('%4s   %*s   %*s   %*s   %*s(Idx)   Children Name(s)\n', ...
                'Idx', ...
                widMaxBodyName, 'Body Name',...
                widMaxJointName, 'Joint Name',...
                widMaxJointName, 'Joint Type',...
                widMaxBodyName+2, 'Parent Name' );
            fprintf('%4s   %*s   %*s   %*s   %*s-----   ----------------\n', ...
                '---', ...
                widMaxBodyName, '---------',...
                widMaxJointName, '----------',...
                widMaxJointName, '----------',...
                widMaxBodyName+2, '-----------');
            
            for i = 1:obj.NumBodies
                
                jointname = obj.Bodies{i}.JointInternal.Name;
                jointtype = obj.Bodies{i}.JointInternal.Type;
                bodyname = obj.Bodies{i}.Name;
                
                fprintf('%4d', int32(obj.Bodies{i}.Index));
                fprintf('   %*s', widMaxBodyName, bodyname);
                fprintf('   %*s', widMaxJointName, jointname);
                fprintf('   %*s', widMaxJointName, jointtype);
                
                
                pid = obj.Bodies{i}.ParentIndex;
                if pid > 0
                    parent = obj.Bodies{pid};
                else
                    parent = obj.Base;
                end
                pid = obj.Bodies{i}.ParentIndex;
                
                % estimate the number of digits for a body index
                p = pid;
                widID = 0;
                while p > 0.2
                    p = floor(p/10);
                    widID = widID+1;
                end
                widID = max(1, widID);
                fprintf('%*s(%*d)   ', widMaxBodyName+8-widID, parent.Name, widID, int32(pid));
                
                cids = find(int32(obj.Bodies{i}.ChildrenIndices));
                for j = 1:length(cids)
                    c = obj.Bodies{cids(j)};
                    fprintf('%s(%d)  ', c.Name, int32(cids(j)) );
                end
                
                fprintf('\n');
            end
            
            fprintf('--------------------\n');
        end
        
        function [ax, bodyDisplayObjArray] = show(obj, varargin)
            %show Calls either the fast or simple show methods depending on
            %the selected options
            parser = obj.parseShowInputs(varargin{:});
            
            fast=parser.Results.FastUpdate;
            preserve=logical(parser.Results.PreservePlot);
            config=parser.Results.Config;
            position=parser.Results.Position;
            parent=parser.Results.Parent;
            collisions=parser.Results.Collisions;
            visuals=parser.Results.Visuals;
            frames=parser.Results.Frames;
            
            if fast && preserve
                robotics.manip.internal.error('rigidbodytree:PreservePlotFastUpdate');
            elseif ~fast
                [ax, bodyDisplayObjArray] = obj.simpleShow(config, parent, collisions, position, preserve, visuals, frames);
            else
                ax = obj.fastShow(config, parent, collisions, position, visuals, frames);
            end
        end
        
        function ax = fastShow(obj, config, parent, collisions, position, visuals, frames)
            %fastShow HGTransform implementation of rigidBodyTree show
            if isempty(obj.FastVisualizationHelper)
                obj.FastVisualizationHelper=robotics.manip.internal.FastVisualizationHelper();
            end
            obj.FastVisualizationHelper.fastUpdate(obj, config, parent, collisions, position, visuals, frames);
            
            ax=obj.FastVisualizationHelper.AxesHandle;
        end
        
        function parser = parseShowInputs(obj, varargin)
            %parseShowInputs Parse inputs to show method
            parser = inputParser;
            parser.StructExpand = false;
            parser.addOptional('Config', obj.homeConfiguration);
            parser.addParameter('Parent', [], ...
                @(x)robotics.internal.validation.validateAxesHandle(x));
            parser.addParameter('PreservePlot', true, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
            parser.addParameter('FastUpdate', false, ...
                @(x)validateattributes(x,{'logical', 'numeric'}, {'nonempty','scalar'}));
            parser.addParameter('Visuals', 'on', ...
                @(x)any(validatestring(x, {'on', 'off'})));
            parser.addParameter('Collisions', 'off', ...
                @(x)any(validatestring(x, {'on', 'off'})));
            parser.addParameter('Frames', 'on', ...
                @(x)any(validatestring(x, {'on', 'off'})));
            parser.addParameter('Position', [0,0,0,0], ...
                @(x)(validateattributes(x, {'numeric'}, ...
                {'nonempty', 'real', 'nonnan', 'finite', 'vector', 'numel', 4})));
            
            parser.parse(varargin{:});
        end
        
        function [ax, bodyDisplayObjArray, fmanager] = simpleShow(obj, config, parent, collisions, position, preserve, visuals, frames, figManagerOpts)
            %SIMPLESHOW Plot robot
            
            if nargin < 9
                figManagerOpts = [];
            end
            
            configValidated = validateConfiguration(obj, config);
            
            displayVisuals = strcmpi(visuals,'on');
            displayCollisions = strcmpi(collisions,'on');
            displayFrames = strcmpi(frames,'on');

            if numel(position) == 4
                % robot base position is represented in the following form
                % [x,y,z,yaw,pitch,roll]. Currently only yaw input is
                % accepted by show method. Assumed the pitch and roll are zero.
                basePosition = [reshape(double(position),1,4),0,0];
            else
                % this enable option to provide all 6-DOF values,
                % [x,y,z,yaw,pitch,roll] which is used only for internal
                % purpose e.g. this is required in robotScenario
                % visualization.
                basePosition = double(position);
            end

            if isempty(parent)
                ax = newplot;
            else
                ax = newplot(parent);
            end
            
            if strcmp(ax.NextPlot, 'replace') || (isa(ax, 'matlab.ui.control.UIAxes') && ~ishold(ax))
                % When hold is off, for axes or uiaxes, respectively
                robotics.manip.internal.RigidBodyTreeVisualizationHelper.resetScene(obj, ax);
                % Adjust axis limits according to robot position
                ax.XLim = ax.XLim + basePosition(1);
                ax.YLim = ax.YLim + basePosition(2);
                ax.ZLim = ax.ZLim + basePosition(3);
            else % when hold is on
                if preserve == false
                    % If preserve flag is false,
                    % remove previous drawn objects, if they could be found
                    delete(findall(ax,'type','hgtransform','Tag', obj.ShowTag));
                    delete(findall(ax,'type','patch','Tag', obj.ShowTag));
                    delete(findall(ax,'type','line','Tag', obj.ShowTag));
                end
            end
            
            % converting the six-element vector of translations and
            % orientations ([x,y,z,yaw,pitch,roll]), respectively, to a
            % homogeneous transformation matrix
            tform = trvec2tform(basePosition(1:3))*eul2tform(basePosition(4:6));
            
            tTree = obj.forwardKinematics(configValidated);
            % Rotate and translate all the body frames according to robot position
            for k = 1:length(tTree)
                tTree{k} = tform*tTree{k};
            end
            tTree{end+1} = tform;
            
            [bodyDisplayObjArray, fmanager] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.drawRobotMemoryLess(obj, ax, tTree, displayFrames, displayVisuals, figManagerOpts);
            
            %If display collisions
            if(displayCollisions)
                robotics.manip.internal.RigidBodyTreeVisualizationHelper.drawCollisionGeometries(...,
                    obj, ax, tTree, displayCollisions);
            end
        end
        
    end
    
    methods (Access = private)
        function defaultInitializeBodiesCellArray(obj)
            obj.Bodies = cell(1, obj.MaxNumBodies);
            for i = coder.unroll(1: obj.MaxNumBodies)  % unroll due to codegen limitation
                obj.Bodies{i} = robotics.manip.internal.RigidBody(['dummybody' num2str(i)] );
                obj.Bodies{i}.ChildrenIndices = zeros(1,obj.MaxNumBodies);
            end
        end

        function clearAllBodies(obj)
            %clearAllBodies clear all bodies in the robot and reset robot numbers
            if coder.target('MATLAB')
                obj.Bodies = cell(1, obj.MaxNumBodies);
            else
                defaultInitializeBodiesCellArray(obj);
            end
            
            obj.NumBodies = 0;
            obj.NumNonFixedBodies = 0;
            obj.PositionNumber = 0;
            obj.VelocityNumber = 0;
            
            if(~coder.target('MATLAB'))
                obj.Base.ChildrenIndices = zeros(1, obj.MaxNumBodies);
            else
                % This change is required for backwards compatibility with
                % R2016b robot MAT files in exampleRobots2016b. This should be
                % removed if the test is refactored. Remove this branch once
                % g2594187 is resolved.
                obj.Base.ChildrenIndices = zeros(1,0);
            end
            
            obj.VisualizationInfo = robotics.manip.internal.VisualizationInfo();
            obj.PositionDoFMap = [zeros(obj.MaxNumBodies,1), (-1)*ones(obj.MaxNumBodies,1)];
            obj.VelocityDoFMap = [zeros(obj.MaxNumBodies,1), (-1)*ones(obj.MaxNumBodies,1)];
            
            obj.IsMaxReachUpToDate = false;
            
        end
        
    end
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
        function bid = findBodyIndexByName(obj, bodyname)
            %findBodyIndexByName Returns the index of the body with name
            %   'bodyname'. If the given body name is the base name, return 1.
            %   In all other cases, return -1.
            
            bid = -1;
            
            bodyname = convertStringsToChars(bodyname);
            validateattributes(bodyname, {'char', 'string'}, {'nonempty', 'row'}, ...
                'findBodyIndexByName', 'bodyname');
            
            if strcmp(obj.Base.Name, bodyname)
                bid = 0;
                return
            end
            
            for i = 1:obj.NumBodies
                if strcmp(obj.Bodies{i}.Name, bodyname)
                    bid = i;
                    break;
                end
            end
        end
        
        function bid = findBodyIndexByJointName(obj, jointname)
            %findBodyIndexByJointName Returns the index of the body whose
            %   joint has the name 'jointname'. Only search within
            %   Bodies.
            
            bid = -1;
            
            validateattributes(jointname, {'char'}, {'nonempty','row'}, ...
                'findBodyIndexByJointName', 'jointname');
            
            for i = 1:obj.NumBodies
                if strcmp(obj.Bodies{i}.Joint.Name, jointname)
                    bid = i;
                    break;
                end
            end
            
        end
        
        function Ttree = forwardKinematics(obj, qvec)
            %forwardKinematics
            n = obj.NumBodies;
            Ttree = repmat({eye(4)}, 1, n);
            k = 1;
            
            for i = 1:n
                body = obj.Bodies{i};
                pnum = body.JointInternal.PositionNumber;
                qi = qvec(k:k+pnum-1);
                Ttree{i} = body.JointInternal.transformBodyToParent(qi);
                
                k = k + pnum;
                if body.ParentIndex>0
                    Ttree{i} = Ttree{body.ParentIndex}*Ttree{i};
                end
            end
        end
        
        function fext = externalForce(obj, bodyName, wrench, q)
            %externalForce Helper function to form external force matrix
            %   This is a convenience function that helps to form
            %   the external force matrix (as input for other dynamics
            %   algorithms). This is a relatively slow method, and can only
            %   set one column of fext matrix at a time. Advanced user
            %   should consider directly form fext from scratch.
            %
            %   The fourth input argument is the current joint positions of
            %   the robot. If it is given, we assume the wrench is given in
            %   the body's local frame. We need q to compute the spatial
            %   force transform from body to base.
            
            if strcmp(obj.DataFormatInternal,'struct')
                robotics.manip.internal.error('rigidbodytree:DynamicsFunctionsUseVectorsOnly');
            end
            
            bid = validateInputBodyName(obj, bodyName);
            validateattributes(wrench, {'double', 'single'}, ...
                {'nonnan', 'finite', 'real', 'vector', 'numel', 6},...
                'externalForce', 'wrench');
            wr = double(wrench);
            
            fext_ = zeros(6, obj.NumBodies);
            if bid > 0
                X = eye(6);
                if nargin > 3
                    validateattributes(q, {'double', 'single'}, ...
                        {'nonnan', 'finite', 'real', 'vector', 'numel', obj.PositionNumber},...
                        'externalForce', 'joint position vector (q)');
                    qcol = double(q(:));
                    Ttree = forwardKinematics(obj, qcol);
                    Tinv = robotics.manip.internal.tforminv(Ttree{bid});
                    X = robotics.manip.internal.tformToSpatialXform(Tinv);
                end
                fext_(:, bid) = X'*wr(:);
                if strcmp(obj.DataFormatInternal,'column')
                    fext = fext_;
                else
                    fext = fext_';
                end
            else
                fext = 0;
                robotics.manip.internal.error('rigidbodytree:CannotSetExternalForceToBase');
            end
        end
        
        function outData = resultPostProcess(obj, inData)
            %resultPostProcess Post-process the results returned from internal
            %   dynamics functions based on the designated data format.
            %   NOTE that internal implementations always return column
            %   vector.
            
            if any(isnan(inData)) || any(isinf(inData))
                robotics.manip.internal.warning('rigidbodytree:InvalidDynamicsResult');
            end
            
            if strcmp(obj.DataFormatInternal, 'column')
                outData = inData(:);
            else
                outData = inData(:)';
            end
        end
        
        function [out1, out2, out3, out4] = validateDynamicsFunctionInputs(obj, out3IsJointAcc, varargin)
            %validateDynamicsFunctionInputs Check the inputs for rigid body
            %   tree dynamics functions.
            
            if strcmp(obj.DataFormatInternal, 'struct')
                robotics.manip.internal.error('rigidbodytree:DynamicsFunctionsUseVectorsOnly');
            end
            
            out1 = obj.homeConfiguration ;          % q
            out2 = zeros(obj.VelocityNumber, 1);    % qdot
            out3 = zeros(obj.VelocityNumber, 1);    % can be either qddot or tau
            out4 = zeros(6, obj.NumBodies);         % fext
            
            if strcmp(obj.DataFormatInternal, 'column')
                sz1 = [obj.PositionNumber, 1];
                sz2 = [obj.VelocityNumber, 1];
                sz3 = [obj.VelocityNumber, 1];
                sz4 = [6, obj.NumBodies];
            else %row
                sz1 = [1, obj.PositionNumber];
                sz2 = [1, obj.VelocityNumber];
                sz3 = [1, obj.VelocityNumber];
                sz4 = [obj.NumBodies, 6];
            end
            
            % joint positions
            if ~isempty(varargin)
                if ~isempty(varargin{1})
                    validateattributes(varargin{1}, {'double', 'single'}, ...
                        {'nonnan', 'finite', 'real', 'size', sz1},...
                        'validateDynamicsFunctionInputs', 'joint position vector (q)');
                    q = double(varargin{1});
                    out1 = q(:);
                end
            end
            
            % joint velocities
            if length(varargin)>1
                if ~isempty(varargin{2})
                    validateattributes(varargin{2}, {'double', 'single'}, ...
                        {'nonnan', 'finite', 'real', 'size', sz2},...
                        'validateDynamicsFunctionInputs', 'joint velocity vector (qdot)');
                    qdot = double(varargin{2});
                    out2 = qdot(:);
                end
            end
            
            % joint accelerations/joint torques
            if length(varargin)>2
                if ~isempty(varargin{3})
                    if out3IsJointAcc
                        s = 'joint acceleration vector (qddot)';
                    else
                        s = 'joint torque vector (tau)';
                    end
                    validateattributes(varargin{3}, {'double', 'single'}, ...
                        {'nonnan', 'finite', 'real', 'size', sz3},...
                        'validateDynamicsFunctionInputs', s);
                    
                    out3tmp = double(varargin{3});
                    out3 = out3tmp(:);
                end
            end
            
            % external forces
            if length(varargin)>3
                if ~isempty(varargin{4})
                    validateattributes(varargin{4}, {'double', 'single'}, ...
                        {'nonnan', 'finite', 'real', 'size', sz4},...
                        'validateDynamicsFunctionInputs', 'external forces (fext)');
                    
                    fext = double(varargin{4});
                    if strcmp(obj.DataFormatInternal, 'column')
                        out4 = fext;
                    else
                        out4 = fext';
                    end
                    
                end
            end
        end
        
        
        function qvec = validateConfiguration(obj, Q)
            %validateConfiguration Validate input joint configuration but
            %   does not check for joint limits
            %
            %   validateConfiguration(ROBOT, Q)
            
            if strcmp(obj.DataFormatInternal, 'struct')
                validateattributes(Q, {'struct'}, {'vector', 'numel', obj.NumNonFixedBodies},'validateConfiguration');
                if ~(isfield(Q, 'JointName') && isfield(Q, 'JointPosition'))
                    robotics.manip.internal.error('rigidbodytree:ConfigStructArrayMissingFieldName');
                end
                
                qvec = zeros(obj.PositionNumber, 1);
                
                for i = 1:obj.NumBodies
                    body = obj.Bodies{i};
                    if ~strcmp(body.JointInternal.Type, 'fixed')
                        jnt = body.Joint;
                        
                        idx = -1;
                        for j = 1:length(Q)
                            if strcmp(Q(j).JointName, jnt.Name)
                                idx = j;
                                break;
                            end
                        end
                        
                        if idx == -1
                            robotics.manip.internal.error('rigidbodytree:ConfigStructArrayInvalidJointNames');
                        end
                        
                        qi = Q(idx).JointPosition;
                        validateattributes(qi, {'double'}, {'nonempty', 'nonnan', 'finite', 'real', ...
                            'vector', 'numel', jnt.PositionNumber}, 'validateConfiguration');
                        
                        idx = obj.PositionDoFMap(i,:);
                        qvec(idx(1):idx(2)) = qi(:);
                    end
                end
            else
                if strcmp(obj.DataFormatInternal, 'column')
                    sz = [obj.PositionNumber, 1];
                else
                    sz = [1, obj.PositionNumber];
                end
                validateattributes(Q, {'double'}, ...
                    {'nonnan', 'finite', 'real', 'size', sz},...
                    'validateDynamicsFunctionInputs', 'joint position vector (q)');
                qvec = Q(:);
            end
            
        end
        
        
        function qvec = validateConfigurationWithLimits(obj, Q, autoAdjust)
            %validateConfigurationWithLimits Validate input joint configuration
            %   structure array and checking joint limit violation
            %
            %   validateConfigurationWithLimits(ROBOT, Q, AUTOADJUST) if
            %   AUTOADJUST flag is false, the validation will error out when
            %   the joint limits are violated. If AUTOADJUST flag is true,
            %   the validation will automatically adjust the input joint
            %   configuration so that those values that are outside the
            %   limits are clamped to the nearest bounds, a warning will
            %   also be raised informing user of the auto adjustment
            
            qvec = validateConfiguration(obj, Q);
            
            limits = obj.JointPositionLimits;
            ubOK = qvec <= limits(:,2)+2*eps;
            lbOK = qvec >= limits(:,1)-2*eps;
            if ~(all(ubOK) && all(lbOK))
                % two solutions when joint limits are violated
                if ~autoAdjust
                    % error out
                    robotics.manip.internal.error('rigidbodytree:ConfigJointLimitsViolation');
                else % auto adjust configuration vector and warn
                    indicesUpperBoundViolation = find(~(ubOK));
                    qvec(indicesUpperBoundViolation) = limits(indicesUpperBoundViolation,2);
                    
                    indicesLowerBoundViolation = find(~(lbOK));
                    qvec(indicesLowerBoundViolation) = limits(indicesLowerBoundViolation,1);
                    robotics.manip.internal.warning('rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');
                end
            end
            
        end
        
        
        
        function bid = validateInputBodyName(obj, bodyname)
            %validateInputBodyName
            
            bid = obj.findBodyIndexByName(bodyname);
            
            % error if bodyname cannot be found within robot (including base)
            if bid == -1
                robotics.manip.internal.error('rigidbodytree:BodyNotFound', bodyname);
            end
            
        end
        
        function [bodyIndices, newrobot] = subtreeInternal(obj, bid)
            %subtreeInternal Implements major function of subtree. note bid
            %   cannot be 0 (i.e. base).
            
            % populate affected body list (subtree)
            bodyIndices = bid;
            body = obj.Bodies{bid};
            k = 1;
            bodyIndices = [bodyIndices, find(int32(body.ChildrenIndices))];
            while k < length(bodyIndices)
                body = obj.Bodies{bodyIndices(k+1)};
                if any(body.ChildrenIndices)
                    bodyIndices = [bodyIndices, find(int32(body.ChildrenIndices))]; %#ok<AGROW>
                end
                k = k + 1;
            end
            
            % create a new robot if a second output is requested
            if nargout == 2
                % reorder affected body list based on existing indices
                bids = sort(bodyIndices); % ascending
                body = obj.Bodies{bids(1)};
                newrobot = robotics.manip.internal.RigidBodyTree(obj.MaxNumBodies, obj.DataFormat);
                
                pid = body.ParentIndex;
                if pid > 0
                    parent = obj.Bodies{pid};
                else
                    parent = obj.Base;
                end
                newrobot.BaseName = parent.Name;
                
                for i = coder.unroll(1:obj.MaxNumBodies)
                    if i <= length(bids)
                        pid = obj.Bodies{bids(i)}.ParentIndex;
                        if pid > 0
                            parent = obj.Bodies{pid};
                        else
                            parent = newrobot.Base;
                        end
                        newrobot.addBody(obj.Bodies{bids(i)}, parent.Name);
                    end
                end
                
            end
        end
        
        
        function parent = parentOf(obj, bodyname)
            %parentOf Get the parent of a specified body
            bid = obj.findBodyIndexByName(bodyname);
            if bid < 0
                robotics.manip.internal.error('rigidbodytree:BodyNotInTree', bodyname);
            end
            
            if bid == 0
                robotics.manip.internal.error('rigidbody:NoSuchPropertyForBase', 'Parent');
            end
            
            body = obj.Bodies{bid};
            pid = body.ParentIndex;
            if pid == 0
                parent = obj.Base;
            else
                parent = obj.Bodies{pid};
            end
            
            
        end
        
        
        function children = childrenOf(obj, bodyname)
            %childrenOf Get the children of a specified body
            bid = obj.findBodyIndexByName(bodyname);
            if bid < 0
                children = cell(1,0);
                return
            end
            
            if bid == 0
                body = obj.Base;
            else
                body = obj.Bodies{bid};
            end
            
            cids = find(int32(body.ChildrenIndices));
            l = length(cids);
            children = cell(1,l);
            for k = 1:l
                children{k} = obj.Bodies{cids(k)};
            end
            
        end
        
        
        function estimateWorkspace(obj)
            %estimatedWorkspace Estimate the range of the workspace of the
            %   manipulator
            
            % for repeatable result, use rng to control global random
            % stream
            
            numTrials = 20; % 20 trials (1 home config, 19 random config)
            TT = cell(numTrials, obj.NumBodies);
            
            if obj.NumBodies > 0
                Qh = obj.homeConfiguration; % definitely test home configuration
                qh = validateConfiguration(obj, Qh);
                
                TT(1,:) = obj.forwardKinematics(qh);
                for i = 2:numTrials
                    Qr = obj.randomConfiguration();
                    qr = validateConfiguration(obj, Qr);
                    Ttree = obj.forwardKinematics(qr);
                    TT(i,:) = Ttree;
                end
                
                % find maximum x, y, z (absolute values) from the
                % computed poses of each body for all 21 configurations
                xmax = max(max(abs(cellfun(@(x) x(1,4), TT))));
                ymax = max(max(abs(cellfun(@(x) x(2,4), TT))));
                zmax = max(max(abs(cellfun(@(x) x(3,4), TT))));
                
                % expand the max x, y, z a little bit (x1.1), then further
                % expand them to multiples of 0.5
                xmax = ceil(1.1*xmax*2)/2;
                ymax = ceil(1.1*ymax*2)/2;
                zmax = ceil(1.1*zmax*2)/2;
                
                obj.EstimatedMaxReach = max([xmax,ymax, zmax]);
                
                % in case there is no body in the robot
                % or all body frame origins overlap with base origin under
                % home configuration
                if obj.EstimatedMaxReach < 2*eps
                    obj.EstimatedMaxReach = 1.0;
                end
            end
            
        end
        
        
        
        
        function [T, Jac] = efficientFKAndJacobianForIK(obj, qv, body1Name, body2Name)
            %efficientFKAndJacobianForIK Efficient kinematic computation
            %   for IK algorithm. Internal use only, it will directly take
            %   in a valid configuration vector.
            computeJac = nargout > 1;
            if nargin < 4
                body2Name = obj.Base.Name;
            end
            
            bid1 = obj.validateInputBodyName(body1Name);
            bid2 = obj.validateInputBodyName(body2Name);
            
            if bid1 >= 0 && bid2 >= 0
                if bid1 == 0
                    body1 = obj.Base;
                else
                    body1 = obj.Bodies{bid1};
                end
                if bid2 == 0
                    body2 = obj.Base;
                else
                    body2 = obj.Bodies{bid2};
                end
                kinematicPathIndices = kinematicPathInternal(obj,body1,body2);
                
                T1 = eye(4); % tform from body1 frame
                Jac = zeros(6, obj.PositionNumber);
                for i = 1:numel(kinematicPathIndices)-1
                    if kinematicPathIndices(i) ~= 0
                        currentBody = obj.Bodies{kinematicPathIndices(i)};
                    else
                        currentBody = obj.Base;
                    end
                    if kinematicPathIndices(i+1) ~= 0
                        nextBody = obj.Bodies{kinematicPathIndices(i+1)};
                    else
                        nextBody = obj.Base;
                    end
                    % Determine whether we are traversing this joint in the
                    % inbound direction.
                    nextBodyIsParent = (nextBody.Index == currentBody.ParentIndex);
                    if nextBodyIsParent
                        % In this case we are moving in the usual direction
                        % (towards the root) and no adjustment needs to be
                        % made to the Jacobian
                        child = currentBody;
                        jointSign = 1;
                    else
                        % In this case we are moving away from the root and
                        % need to negate the slice of the Jacobian
                        % associated with this joint.
                        child = nextBody;
                        jointSign = -1;
                    end
                    joint = child.JointInternal;
                    if strcmp(joint.Type, 'fixed')
                        Tc2p = joint.transformBodyToParent(0); % child to parent tform
                    else
                        qidx = obj.PositionDoFMap(child.Index,:);
                        Tc2p = joint.transformBodyToParent(qv( qidx(1):qidx(2) )); % child to parent tform
                        if computeJac
                            vidx = obj.VelocityDoFMap(child.Index,:);
                            % NOTE: T1 is the transformation from body1 to
                            % the current body. However, computing the
                            % Jacobian requires the transformation from the
                            % joint to body1 (Tj1). To get Tj1 we need the
                            % transformation from the current body to the
                            % joint (Tj). The method for computing Tj
                            % depends on whether we are traversing the tree
                            % in the inbound or outbound direction.
                            if nextBodyIsParent
                                % Inbound traversal
                                Tj = joint.ChildToJointTransform;
                            else
                                % Outbound traversal
                                Tj = robotics.manip.internal.tforminv( ...
                                    joint.JointToParentTransform);
                            end
                            T1j = Tj*T1; % tform from body1 to joint
                            Tj1 = robotics.manip.internal.tforminv(T1j);
                            
                            JacSlice = robotics.manip.internal.tformToSpatialXform(Tj1)...
                                *joint.MotionSubspace*jointSign;
                            Jac(:,vidx(1):vidx(2)) = JacSlice;
                        end
                    end
                    if nextBodyIsParent
                        T1 = Tc2p*T1;
                    else
                        T1 = robotics.manip.internal.tforminv(Tc2p)*T1;
                    end
                end
                X = [T1(1:3, 1:3), zeros(3); zeros(3), T1(1:3, 1:3)];
                Jac = X*Jac; % Jacobian w.r.t. body2
                T = T1;
            else
                T = [];
                Jac = [];
            end
            
        end
        
        function indices = kinematicPath(obj, body1Name, body2Name)
            bid1 = obj.validateInputBodyName(body1Name);
            bid2 = obj.validateInputBodyName(body2Name);
            
            if bid1 >= 0 && bid2 >= 0
                if bid1 == 0
                    body1 = obj.Base;
                else
                    body1 = obj.Bodies{bid1};
                end
                if bid2 == 0
                    body2 = obj.Base;
                else
                    body2 = obj.Bodies{bid2};
                end
                indices = kinematicPathInternal(obj, body1, body2);
            else
                indices = [];
            end
        end
        
        function indices = kinematicPathInternal(obj, body1, body2)
            %kinematicPath Compute the shortest kinematic path from body1
            %to body 2.
            %
            %indices = kinematicPath(obj,body1,body2) returns
            %INDICES, a vector whose first element is the index of BODY1
            %and whose last element is the index of BODY2. The intermediate
            %elements are the indices of the kinematic chain from BODY1 to
            %BODY2.
            % ASSUMES BODY1 AND BODY2 ARE IN OBJ
            ancestorIndices1 = ancestorIndices(obj,body1);
            ancestorIndices2 = ancestorIndices(obj,body2);
            minPathLength = min(numel(ancestorIndices1),numel(ancestorIndices2));
            numCommonAncestors = minPathLength;
            for i = 1:minPathLength-1
                if ancestorIndices1(end-i) ~= ancestorIndices2(end-i)
                    numCommonAncestors = i;
                    break;
                end
            end
            commonAncestorIndex = ancestorIndices1(end - numCommonAncestors + 1);
            indices = [ancestorIndices1(1:end-numCommonAncestors), ...
                commonAncestorIndex, ...
                ancestorIndices2(end-numCommonAncestors:-1:1)];
        end
        
        function indices = ancestorIndices(obj, body)
            %ancestorBodies Return cell array of ancestors
            %BODIES = ancestorBodies(OBJ, BODY) returns a vector of body
            %indices whose first element is the index of BODY
            %and whose last element is 0.
            % ASSUMES BODY IS IN OBJ
            indices = zeros(1,obj.NumBodies+1);
            i = 2;
            indices(1) = body.Index;
            while body.ParentIndex > 0
                body = obj.Bodies{body.ParentIndex};
                indices(i) = body.Index;
                i = i + 1;
            end
            if body.Index > 0
                indices(i) = body.ParentIndex;
                i = i + 1;
            end
            indices = indices(1:i-1);
        end
        
        function indices = kinematicPathToBase(obj, endEffectorName)
            %kinematicPathToBase Compute the shortest kinematic path from
            %   end-effector to base
            indices = zeros(obj.NumBodies, 1);
            bid = obj.validateInputBodyName(endEffectorName);
            
            if bid == 0
                indices = 0;
            else
                body = obj.Bodies{bid};
                
                i = 1;
                while body.ParentIndex ~= 0
                    indices(i) = body.Index;
                    body = obj.Bodies{body.ParentIndex};
                    i = i + 1;
                end
                indices = [indices(1:i-1); body.Index;  0];
                
            end
        end
        
        
    end
    
    methods
        function saveAsMATLABFunction(obj, filename)
            %saveAsMATLABFunction Create a function that will re-create the
            %   RigidBodyTree object
            [pathstr, file, ext] = fileparts(filename);
            if isempty(ext)
                ext = '.m';
            end
            filename = fullfile(pathstr, [file, ext]);
            fid = fopen(filename, 'w');
            fprintf(fid, 'function robot = %s(dataFormat)\n', file);
            fprintf(fid, '%%%s Create rigidBodyTree for the robot model\n', file);
            fprintf(fid, ['%%   ROBOT = %s(DATAFORMAT) constructs a rigidBodyTree, ROBOT, and sets the\n', ...
                '%%   data format to DATAFORMAT. The possible values of DATAFORMAT are \n', ...
                '%%   ''struct'', ''column'' and ''row''. The default value is ''%s'', which \n', ...
                '%%   matches the data format of the rigidbodytree object used to generate \n', ...
                '%%   this function.\n\n'], file, obj.DataFormat);
            fprintf(fid, '%%   Auto-generated by MATLAB on %s\n\n', datestr(now));
            fprintf(fid, '%%#codegen\n\n');
            fprintf(fid, 'narginchk(0,1);\n');
            fprintf(fid, 'if ~nargin==1\n');
            fprintf(fid, 'dataFormat=''%s'';\n', obj.DataFormat);
            fprintf(fid, 'end\n');
            fprintf(fid, 'robot = rigidBodyTree(''MaxNumBodies'', %d, ''DataFormat'', dataFormat);\n', obj.MaxNumBodies);
            fprintf(fid, 'robot.Gravity = [% 22.16g, % 22.16g, % 22.16g];\n', obj.Gravity);
            for i = 0:obj.NumBodies
                if i == 0
                    body = obj.Base;
                else
                    body = obj.Bodies{i};
                    joint = body.Joint;
                    fprintf(fid, '%% Add body, ''%s'', and joint, ''%s''\n', body.Name, joint.Name);
                    fprintf(fid, 'bodyName = ''%s'';\n', body.Name);
                    fprintf(fid, 'bodyMass = % 22.16g;\n', body.Mass);
                    fprintf(fid, 'bodyCoM = [% 22.16g, % 22.16g, % 22.16g];\n', body.CenterOfMass);
                    fprintf(fid, 'bodyInertia = [% 22.16g, % 22.16g, % 22.16g, % 22.16g, % 22.16g, % 22.16g];\n', body.Inertia);
                end
                
                if i ~= 0
                    if body.ParentIndex == 0
                        fprintf(fid, 'parentName = ''%s'';\n', obj.Base.Name);
                    else
                        fprintf(fid, 'parentName = ''%s'';\n', obj.Bodies{body.ParentIndex}.Name);
                    end
                end
                
                if i > 0
                    fprintf(fid, 'jointName = ''%s'';\n', joint.Name);
                    fprintf(fid, 'jointType = ''%s'';\n', joint.Type);
                    if any(any((joint.ChildToJointTransform ~= eye(4))))
                        % extract DH Params
                        T = joint.ChildToJointTransform;
                        alpha = real(atan2(T(3,2), T(3,3)));
                        if abs(T(1,1))<1e-8
                            a = T(2,4)/T(2,1);
                        else
                            a = T(1,4)/T(1,1);
                        end
                        
                        switch(joint.Type)
                            case 'revolute'
                                theta = 0;
                                d = T(3,4);
                            case 'prismatic'
                                theta = real(atan2(T(1,2), T(1,1)));
                                d = 0;
                            case 'fixed'
                                d = T(3,4);
                                theta = real(atan2(T(1,2), T(1,1)));
                        end
                        
                        fprintf(fid, 'dhparams = [% 22.16g, % 22.16g, % 22.16g, % 22.16g];\n', [a alpha d theta]);
                        fprintf(fid, ['\n', ...
                            'joint = rigidBodyJoint(jointName, jointType);\n', ...
                            'joint.setFixedTransform(dhparams, ''dh'');\n']);
                    else
                        fprintf(fid, ['T_Joint_to_Parent = [% 22.16g, % 22.16g, % 22.16g, % 22.16g; ...\n', ...
                            '                     % 22.16g, % 22.16g, % 22.16g, % 22.16g; ...\n', ...
                            '                     % 22.16g, % 22.16g, % 22.16g, % 22.16g; ...\n', ...
                            '                     % 22.16g, % 22.16g, % 22.16g, % 22.16g];\n'], joint.JointToParentTransform');
                        fprintf(fid, ['\n', ...
                            'joint = rigidBodyJoint(jointName, jointType);\n', ...
                            'joint.setFixedTransform(T_Joint_to_Parent);\n']);
                    end
                    switch joint.Type
                        case {'revolute', 'prismatic'}
                            fprintf(fid, 'jointAxis = [% 22.16g, % 22.16g, % 22.16g];\n', joint.JointAxis);
                            fprintf(fid, 'jointPositionLimits = [% 22.16g, % 22.16g];\n', joint.PositionLimits);
                            fprintf(fid, 'jointHomePosition = % 22.16g;\n', joint.HomePosition);
                            fprintf(fid, 'joint.PositionLimits = jointPositionLimits;\n');
                            fprintf(fid, 'joint.JointAxis = jointAxis;\n');
                            fprintf(fid, 'joint.HomePosition = jointHomePosition;\n');
                        otherwise
                    end
                    
                    % To support code generation for adding collision
                    % geometries, we need to define the "MaxNumCollisions"
                    fprintf(fid, ...
                        'body = rigidBody(bodyName, "MaxNumCollisions", %d);\n',...
                        body.CollisionsInternal.Size);
                    fprintf(fid, ['\n', ...
                        'body.Joint = joint;\n'...
                        'body.Mass = bodyMass;\n'...
                        'body.CenterOfMass = bodyCoM;\n'...
                        'body.Inertia = bodyInertia;\n']);
                    
                    
                    
                end
                
                % A zero index corresponds to the Base, while a non-zero index
                % corresponds to the Bodies property
                if i==0
                    % If there is collision data
                    if(body.CollisionsInternal.Size)
                        fprintf(fid, 'robot.Base.addCollision(%s)\n', ...
                            serialize(body.CollisionsInternal));
                    end
                else
                    % If there is collision data
                    if(body.CollisionsInternal.Size)
                        fprintf(fid, 'body.addCollision(%s)\n', ...
                            serialize(body.CollisionsInternal));
                    end
                end
                if i == 0
                    fprintf(fid, 'robot.BaseName = ''%s'';\n', obj.Base.Name);
                else
                    fprintf(fid, 'robot.addBody(body, parentName);\n');
                end
                fprintf(fid, '\n\n');
                
            end
            fclose(fid);
        end
        
        function showGraph(obj)
            %showGraph Display the tree structure of the robot
            s = {};
            t = {};
            hl = zeros(1, obj.NumBodies);
            
            for i = 1: obj.NumBodies
                p = obj.Bodies{i}.ParentIndex;
                if p == -1
                    s{end+1} = 'reference';
                elseif p == 0
                    s{end+1} = sprintf('%d-%s', p, obj.Base.Name);
                else
                    s{end+1} = sprintf('%d-%s', p, obj.Bodies{p}.Name);
                end
                
                t{end+1} = sprintf('%d-%s', i, obj.Bodies{i}.Name);
                if strcmpi( obj.Bodies{i}.Joint.Type, 'revolute')
                    hl(i) = 1;
                elseif strcmpi( obj.Bodies{i}.Joint.Type, 'prismatic')
                    hl(i) = 2;
                end
            end
            hp = plot(graph(s,t));
            hp.NodeColor = 'b';
            hp.Marker = 's';
            for i = 1:obj.NumBodies
                if hl(i) == 1
                    highlight(hp, [obj.Bodies{i}.ParentIndex+1, i+1], 'EdgeColor', 'r','LineWidth',2); % need to plus 1 here
                elseif hl(i) == 2
                    highlight(hp, [obj.Bodies{i}.ParentIndex+1, i+1], 'EdgeColor', 'g','LineWidth',2); % need to plus 1 here
                end
            end
            
            set(gcf,'color','w');
            set(gca,'visible','off');
        end
    end
    
    
    % Property access methods
    methods
        function bnames = get.BodyNames(obj)
            %get.BodyNames
            bnames = repmat({''},1, obj.NumBodies );
            
            for i = 1:obj.NumBodies
                bnames{i} = obj.Bodies{i}.Name;
            end
            
        end
        
        function basename = get.BaseName(obj)
            %get.BaseName
            basename = obj.Base.Name;
        end
        
        function set.BaseName(obj, baseName)
            %set.BaseName
            validateattributes(baseName, {'char'},...
                {'row','nonempty'}, 'rigidBodyTree', 'baseName');
            
            bid = obj.findBodyIndexByName(baseName);
            if bid == 0
                return
            end
            
            if bid < 0
                obj.Base.NameInternal = baseName;
            else
                robotics.manip.internal.error('rigidbodytree:BaseNameCollision', baseName);
            end
        end
        
        function limits = get.JointPositionLimits(obj)
            %get.JointPositionLimits
            limits = zeros(obj.PositionNumber, 2);
            k = 1;
            for i = 1:obj.NumBodies
                body = obj.Bodies{i};
                if ~strcmp(body.JointInternal.Type, 'fixed')
                    pnum = body.JointInternal.PositionNumber;
                    limits(k:k+pnum-1, :) = body.JointInternal.PositionLimits;
                    k = k + pnum;
                end
            end
        end
        
        function dataformat = get.DataFormat(obj)
            dataformat = dataFormatFilterOutbound(obj.DataFormatInternal);
        end
        
        function set.DataFormat(obj, dataformat)
            fmt_ = validatestring(dataformat, {'struct', 'column', 'row'}, 'rigidBodyTree', 'DataFormat');
            fmt = dataFormatFilterInbound(fmt_);
            obj.DataFormatInternal = fmt;
            
        end
        
        function value = get.ShowTag(obj)
            value = obj.VisualizationInfo.ShowTag;
        end
        
        function value = get.EstimatedMaxReach(obj)
            value = obj.VisualizationInfo.EstimatedMaxReach;
        end
        
        function set.EstimatedMaxReach(obj, value)
            obj.VisualizationInfo.EstimatedMaxReach = value;
        end
        
        function value = get.IsMaxReachUpToDate(obj)
            value = obj.VisualizationInfo.IsMaxReachUpToDate;
        end
        
        function set.IsMaxReachUpToDate(obj, value)
            obj.VisualizationInfo.IsMaxReachUpToDate = value;
        end
        
        function set.Gravity(obj, g)
            %set.Gravity
            validateattributes(g, {'double'},{'nonempty','vector','nonnan', 'finite', 'real', 'numel', 3},...
                'rigidBodyTree','Gravity');
            obj.Gravity = g(:)';
        end
    end
    
    
    methods(Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters
            props = {'MaxNumBodies', 'MaxNameLength', 'MaxJointPositionNumber', 'DataFormatInternal'};
        end
    end
    
end




function fmt = dataFormatFilterOutbound(fmt_)
%dataFormatFilterOutbound Needed for code generation
if strcmp(fmt_,'row___')
    fmt = 'row';
else
    fmt = fmt_;
end
end

function fmt = dataFormatFilterInbound(fmt_)
%dataFormatFilterInbound Needed for code generation
if strcmp(fmt_,'row')
    fmt = 'row___';
else
    fmt = fmt_;
end
end

% LocalWords:  SIMPLESHOW ph
