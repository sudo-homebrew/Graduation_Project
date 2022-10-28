classdef obstacleListCollisionGeometry < nav.algs.internal.ObstacleList & handle & matlab.mixin.CustomDisplay
%OBSTACLELISTCOLLISIONGEOMETRY Performs collision checking on object in an environment
    %   The obstacleListCollisionGeometry is a nav.ObstacleList, used to
    %   represent a sparse environment using a discrete set of 
    %   robotics.core.internal.CollisionGeometryBase objects. The class
    %   also stores an EgoObject, which can be transformed in the SE3 space
    %   to determine whether the EgoObject is in collision with the
    %   environment. When encapsulated inside of a validatorObstacleList,
    %   this object can be used by local and global path-planners shipped in
    %   the Navigation Toolbox, such as plannerRRT, plannerRRTStar, and 
    %   trajectoryOptimalFrenet.
    %   
    %   OBSTACLELIST = obstacleListCollisionGeometry creates an
    %   ObstacleList that performs collision checks between
    %   CollisionGeometry-based objects. By default, OBSTACLELIST is a
    %   collisionBox(2,2,2) located at the world-origin, and the
    %   environment contains no obstacles.
    %
    %   OBSTACLELIST = obstacleListCollisionGeometry(EGOOBJECT) takes a
    %   CollisionGeometryBase object, EGOOBJECT, as input. EGOOBJECT can be
    %   a collisionBox, collisionCylinder, collisionSphere, or collisionMesh
    %   object.
    %
    %   OBSTACLELIST = obstacleListCollisionGeometry(EGOOBJECT, COLLISIONOBJECTS) takes a
    %   CollisionGeometryBase (CGB) object, EGOOBJECT, and a set of 
    %   CGB objects, COLLISIONOBJECTS, as input.
    %   COLLISIONOBJECTS can be provided as a single CGB, a struct array of
    %   CGB objects of the same type, or a cell array of different CGB
    %   objects of differing types
    %
    %   obstacleListCollisionGeometry properties:
    %      EgoObject                        - The collision object representing the ego body
    %      NumObstacles                     - The number of obstacles contained in the ObstacleList
    %      CollisionObjects                 - The current list of CGB obstacles representing the environment
    %
    %   validatorObstacleList methods:
    %      addObstacle                      - Append one or more obstacles to the ObstaclesList
    %      checkCollision                   - Check whether EgoObject collides with any obstacles at specified poses
    %      copy                             - Returns a deep copy of the ObstacleList
    %      removeObstacle                   - Remove one or more obstacles from the ObstacleList
    %      show                             - Display the ego object and obstacles in a figure
    %      updateObstacleGeometry           - Update the geometric properties of one or more obstacles
    %      updateObstaclePose               - Update the pose of one or more obstacles
    %
    %   See also nav.ObstacleList.

%   Copyright 2019 The MathWorks, Inc.
    
    properties (Transient, Access = ?nav.algs.internal.InternalAccess)
        %ObstacleHGGroupObj Handle to a graphics object representing the obstacles visualized by the show method
        %
        %   ObstacleHGGroupObj is an hggroup object that contains a set of
        %   hgtransform objects. Each hgtransform contains the 3D patch of an
        %   object in CollisionObjects. When an obstacle's geometry is
        %   changed, the patch object is updated, and when an obstacle's
        %   pose is changed, the hgtransform 'Matrix' property is updated,
        %   reducing visualization overhead.
        ObstacleHGGroupObj
        
        %EgoHGTransformObj Handle to a graphics object representing the EgoObject visualized by the show method
        %
        %   EgoHGTransformObj is an hgtransform containing the patch of the
        %   EgoObject. When the obstacleList is shown, the patch and
        %   hgtransform Matrix are updated to reflect the current Pose
        %   of the EgoObject
        EgoHGTransformObj
        
        %EgoCollisionHGGroupObj Handle to a graphics object representing the EgoObject during checkCollision
        %
        %   EgoCollisionHGGroupObj is an hggroup object that manages a set
        %   of hgtransform objects representing the EgoObject across a set
        %   of poses. When checkCollision(OBJ, STATE, DISPLAYSTATE) is called 
        %   with the DISPLAYSTATE flag set to 'true', the EgoObject is
        %   shown at each state, appearing red if it is in collision with any
        %   CollisionObjects, and green otherwise.
        EgoCollisionHGGroupObj
        
        %obstaclesChanged A flag that is updated when the geometry of an obstacle has changed
        %
        %   When obstaclesChanged is false, visualization methods perform a
        %   lightweight update in which they only modify the transformation
        %   of the patch object. If obstaclesChanged is true, then the
        %   patch objects are updated to reflect the changes.
        obstaclesChanged = 1
    end
        
    properties (Access = ?nav.algs.internal.InternalAccess)
        %ObstacleRadii Cached radii of obstacles used for collision precheck
        %
        %   For each obstacle in CollisionObjects, ObstacleList stores the
        %   minimum radius for a sphere that bounds all vertices in the
        %   mesh. This enables a computationally simple check between
        %   bounding sphere centers as a way to reduce the number of calls
        %   to the more expensive check between ego and obstacle meshes.
        ObstacleRadii
        
        %ObstaclePositions Matrix containing the center positions of all obstacles
        ObstaclePositions
        
        %EgoObjectType Numeric flag used for accelerating type checks during planning
        %
        %   0 - collisionMesh or user-defined class based on CollisionGeometryBase
        %   1 - collisionBox (Default)
        %   2 - collisionCylinder
        %   3 - collisionSphere
        EgoObjectType = 1
    end
    
    properties (SetAccess = ?nav.algs.internal.InternalAccess, GetAccess = public)
        %CollisionObjects Cell or struct array containing obstacle config information
        %
        %   Contains information that describes the type, size, initial
        %   location, and other meta information used by the obstacle list
        %   when updating current location or checking for collisions.
        CollisionObjects
    end
    
    properties (Access = private, Constant)
        %ValidClasses List of valid class types for use in this obstaclelist
        ValidClasses = {'collisionBox','collisionCylinder','collisionSphere'};
    end
    
    methods
        function obj = obstacleListCollisionGeometry(varargin)
        %obstacleListCollisionGeometry Creates a CollisionGeometry-based obstacleList
            % Parse inputs and return defaults if all arguments are not
            % provided
            [egoObject, obstacles, positions, radii] = nav.algs.internal.obstacleListCollisionGeometry.parseConstructor(varargin{:});

            % Construct parent
            obj = obj@nav.algs.internal.ObstacleList(egoObject, obstacles);
            
            % Update number of obstacles
            obj.NumObstacles = numel(obj.CollisionObjectsInternal);
            
            % Update obstacle precheck properties
            obj.ObstaclePositions = positions;
            obj.ObstacleRadii = radii;
        end
    end
    
    methods
        function out = get.CollisionObjects(obj)
            objectTypes = cellfun(@(x)[class(x) newline],obj.CollisionObjectsInternal,'UniformOutput',false);
            out = [objectTypes{:}];
        end
    end
    
    methods
        function [isColliding, axHandle] = checkCollision(obj, states, displayState)
        %checkCollision Check whether EgoObject collides with any obstacles at specified poses
        %   
        %   ISCOLLIDING = checkCollision(OBJ, STATES) Transforms the EgoObstacle 
        %   using the N-by-M state-array, STATES, and checks whether it
        %   is in collision with any of the CollisionObjects. checkCollision
        %   returns an N-by-1 vector of booleans, ISCOLLIDING, where false 
        %   means the EgoObject is not in collision with any CollisionObjects,
        %   and true means at least one intersection has been found. STATES
        %   can be provided as a matrix of either SE2 or SE3 vectors in the
        %   following forms:
        %
        %     N-by-[x  y Oz]
        %     N-by-[x  y  z Ox Oy Oz]
        %     N-by-[x  y  z Qw Qx Qy Qz]
        %   
        %   ISCOLLIDING = checkCollision(OBJ, STATES, DISPLAYSTATE) Checks
        %   whether the EgoObject is in collision with objects in
        %   CollisionObjects and plots the EgoObject at each state. If 
        %   DISPLAYSTATE is set to TRUE, checkCollision plots the EgoObject
        %   at each state. States that are in collision are shown in red, 
        %   and free states are displayed in green.
            
            showCollisionCheckState = 0;
            if nargin == 3
                % Ensure the input is a boolean or scalar
                validateattributes(displayState,{'logical','numeric'},{'nonempty','scalar','binary'},'checkCollision','displayState',2);
            
                if displayState == 1
                    if ishold
                        holdState = 'on';
                    else
                        holdState = 'off';
                    end
                    
                    ax = obj.show('FastUpdate',1);
                    temp = onCleanup(@()hold(ax,holdState));
                    showCollisionCheckState = 1;
                    
                    % Remove previously plotted EgoObjects
                    hgGroups = findobj(ax,'Type','hggroup','Tag','checkCollisionObject');
                    if isempty(obj.EgoCollisionHGGroupObj) || ~isvalid(obj.EgoCollisionHGGroupObj)
                        % Ego trajectory list has:
                            % 1) Not been plotted
                            % 2) Was deleted
                            
                        % Create new hggroup to store the hgtransforms
                        obj.EgoCollisionHGGroupObj = hggroup('Parent',ax,'Tag','checkCollisionObject');
                    else
                        % Remove any collision views plotted by other obstacleLists
                        delete(hgGroups(hgGroups ~= obj.EgoCollisionHGGroupObj));
                        
                        % Move this hgTransform group to the current axes
                        set(obj.EgoCollisionHGGroupObj, 'Parent', ax);
                    end
                    
                    numStates = size(states,1);
                    numChildren = numel(obj.EgoCollisionHGGroupObj.Children);
                    
                    if numChildren > numStates
                        % Remove excess hgtransform/patches representing
                        % the EgoObject if the number of previously shows
                        % objects exceeds the number of states provided
                        delete(obj.EgoCollisionHGGroupObj.Children(1:numChildren-numStates));
                    else
                        % Add additional hgtransform/patches to represent the
                        % EgoObject if the number of provided states
                        % exceeds the number of previously displayed states
                        for i = 1:numStates-numChildren
                            hgT = copy(obj.EgoHGTransformObj);
                            set(hgT,'Parent',obj.EgoCollisionHGGroupObj);
                        end
                    end
                    drawnow limitrate;
                    if nargout == 2
                        axHandle = ax;
                    end
                elseif nargout == 2
                    axHandle = [];
                end
            else
                if nargout == 2
                    axHandle = [];
                end
            end
            
            if isempty(states) || obj.NumObstacles == 0
                isColliding = [];
            else
                % Validate state input
                validateattributes(states,{'numeric'},{'nonnan'},'checkCollision','states',1);
                if ~any(size(states,2) == [3 6 7])
                    coder.internal.error('nav:navalgs:obstacleListCG:InvalidStateDimensions');
                end
                
                % Calculate radius used for precheck
                switch obj.EgoObjectType
                    case 1
                        %collisionBox
                        rEgo = sqrt(obj.EgoObject.XInternal^2+obj.EgoObject.YInternal^2+obj.EgoObject.YInternal^2);
                    case 2
                        %collisionCylinder
                        rEgo = sqrt(obj.EgoObject.RadiusInternal^2+obj.EgoObject.LengthInternal^2);
                    case 3
                        %collisionSphere
                        rEgo = obj.EgoObject.RadiusInternal;
                    otherwise
                        %collisionMesh or user-defined geometry
                        rEgo = max(sqrt(sum(obj.EgoObject.VisualMeshVertices.^2,2)));
                end
                
                % Preallocate local variables
                isColliding = false(size(states,1),1);
                geometry = obj.EgoObject.GeometryInternal;
                numStates = size(states,1);
                stateSize = size(states,2);
                
                if stateSize == 3
                    % State = [x y theta]
                    P = [states(:,1:2) zeros(numStates,1)];
                    zO = states(:,3);
                    Q = [cos(zO/2) zeros(numStates,2) sin(zO/2)];
                elseif stateSize == 6
                    % State = [x y z thetaX thetaY thetaZ]
                    P = states(:,1:3);
                    Q = eul2quat(states(:,6:-1:4),'ZYX');
                else
                    % State = [x y z qW qX qY qZ]
                    P = states(:,1:3);
                    Q = states(:,4:7)./vecnorm(states(:,4:7),2,2);
                end
                
                if showCollisionCheckState
                    R = quat2rotm(Q);
                end
                
                % NOTE on the bounding-sphere precheck: To avoid more costly 
                % operations we precalculate the L2 distance between states
                % of the EgoObject. Each time the EgoObject moves, we assume
                % a worst case scenario where it moves directly towards the
                % obstacle being checked against, which simplifies to subtracting
                % the step distance from the current center-center distance. 
                % When this falls below the sum of the radii, a full check
                % and center-distance update is performed for that
                % ego/obstacle pair. For non-cluttered environments where
                % states represent a discretized motion, this should
                % eliminate a large number of full distance/collision checks.
                % The worst-case scenario (prechecks always fail, either due
                % to large motions between states, poor fit of bounding sphere
                % to geometry, or all obstacles overlapping with EgoObject)
                % would add a small, constant scaling-factor to a pre-checkless
                % performance.
                
                % Calculate BoundingSphere thresholds
                rBounding = obj.ObstacleRadii+rEgo;
                
                % Find cartesian distance moved by the Ego between each state
                P(end+1,:) = [0 0 0];
                vecStep = diff(P,1,1);
                distMoved = sqrt(sum(vecStep.*vecStep,2));
                
                % Calculate initial distance between Ego/Obstacles
                d = sqrt(sum((obj.ObstaclePositions - P(1,:)).^2,2));
                
                poseInit = obj.EgoObject.PoseInternal;
                
                for i = 1:numStates
                    for j = 1:obj.NumObstacles
                        if d(j) < rBounding(j)
                            % Check for collision with each obstacle that
                            % fails the precheck
                            isColliding(i) = isColliding(i) || robotics.core.internal.intersect(geometry, P(i,:), Q(i,:),...
                                obj.CollisionObjectsInternal{j}.GeometryInternal, obj.CollisionObjectsInternal{j}.Position, obj.CollisionObjectsInternal{j}.Quaternion,...
                                0);

                            % Calculate new bounding-sphere distance for object
                            % Pre-add the upcoming distMoved since we know it
                            d(j) = sqrt(sum((obj.ObstaclePositions(j,:) - P(i+1,:)).^2,2));
                        else
                            % If the precheck passes, update precheck distance
                            % with worst-case scenario (Ego moves directly
                            % towards obstacle)
                            d(j) = d(j)-distMoved(i);
                        end
                    end
                    
                    if showCollisionCheckState
                        obj.EgoCollisionHGGroupObj.Children(i).Children.FaceColor = [isColliding(i) ~isColliding(i) 0];
                        set(obj.EgoCollisionHGGroupObj.Children(i),'Matrix',[R(:,:,i) P(i,:)'; 0 0 0 1]);
                    end
                end
                
                obj.EgoObject.PoseInternal = poseInit;
            end
        end
        
        function addObstacle(obj, collisionObjects)
        %addObstacle Append one or more obstacles to the ObstaclesList
        %
        %   addObstacle(OBJ, COLLISIONOBJECTS) accepts one or more obstacles, 
        %   COLLISIONOBJECTS, in the form of a scalar, struct array, or cell 
        %   array of CollisionGeometryBase objects and appends them to the
        %   end of the existing set of obstacles in OBJ.
        
            [obstacles, positions, radii] = nav.algs.internal.obstacleListCollisionGeometry.convertObstacleInput(collisionObjects,'addObstacle','collisionObjects',1);
            obj.CollisionObjectsInternal = [obj.CollisionObjectsInternal; obstacles];
            obj.ObstaclePositions = [obj.ObstaclePositions; positions];
            obj.ObstacleRadii = [obj.ObstacleRadii; radii];
            obj.NumObstacles = numel(obj.CollisionObjectsInternal);
            obj.obstaclesChanged = 1;
        end
        
        function removeObstacle(obj, obstacleIndices)
        %removeObstacle Remove one or more obstacles from the ObstacleList
        %
        %   removeObstacle(OBJ, OBSTACLEINDICES) removes one or more obstacles 
        %   corresponding to the list of supplied linear indices, OBSTACLEINDICES, 
        %   from the ObstacleList.
        
            validateattributes(obstacleIndices,{'numeric'},{'nonempty','positive','integer','<=', numel(obj.CollisionObjectsInternal)},'removeObstacle','indices',1);
            obj.CollisionObjectsInternal(obstacleIndices(:)) = [];
            obj.ObstacleRadii(obstacleIndices) = [];
            obj.ObstaclePositions(obstacleIndices,:) = [];
            obj.obstaclesChanged = 1;
            obj.NumObstacles = numel(obj.CollisionObjectsInternal);
        end
        
        function updateObstaclePose(obj, states, obstacleIndices)
        %updateObstaclePose Update the pose of one or more obstacles
        %
        %   updateObstaclePose(OBJ, STATES) update the Pose property of
        %   each obstacle in OBJ using the provided N-by-M state matrix,
        %   STATES. Each row in STATES will be converted to a 4x4
        %   homogenous matrix for its corresponding collision object, and
        %   can take the following forms:
        %
        %     N-by-[x  y Oz]
        %     N-by-[x  y  z Ox Oy Oz]
        %     N-by-[x  y  z Qw Qx Qy Qz]
        %
        %   updateObstaclePose(OBJ, STATES, OBSTACLEINDICES) update the 
        %   Pose property for a set of obstacles in OBJ using an N-by-M matrix,
        %   STATES, and an N-element vector of linear indices, OBSTACLEINDICES.
        
            narginchk(2,3)
            validateattributes(states, {'numeric'}, {'2d', 'nonnan'}, 'updateObstaclePose', 'states', 1);
            
            if nargin == 2
                if isempty(states)
                    return;
                elseif size(states,1) ~= obj.NumObstacles
                    coder.internal.error('nav:navalgs:obstacleListCG:NumStatesMatchNumObjects')
                else
                    % Update state of each obstacle
                    H = eye(4);
                    switch size(states,2)
                        case 3
                            % State = [x y theta]
                            obj.ObstaclePositions = [states(:,1:2) zeros(obj.NumObstacles,1)];
                            for i = 1:obj.NumObstacles
                                zO = states(i,3);
                                R = [cos(zO) -sin(zO) 0;
                                     sin(zO)  cos(zO) 0;
                                          0       0  1];
                                H(1:3,:) = [R [states(i,1:2) 0]'];
                                obj.CollisionObjectsInternal{i}.Pose = H;
                            end
                        case 6
                            % State = [x y z thetaX thetaY thetaZ]
                            R = eul2rotm(states(:,6:-1:4),'ZYX');
                            obj.ObstaclePositions = states(:,1:3);
                            for i = 1:obj.NumObstacles
                                H(1:3,:) = [R(:,:,i) states(i,1:3)'];
                                obj.CollisionObjectsInternal{i}.Pose = H;
                            end
                        case 7
                            % State = [x y z qW qX qY qZ]
                            R = quat2rotm(states(:,4:7));
                            obj.ObstaclePositions = states(:,1:3);
                            for i = 1:obj.NumObstacles
                                H(1:3,:) = [R(:,:,i) states(i,1:3)'];
                                obj.CollisionObjectsInternal{i}.Pose = H;
                            end
                        otherwise
                            coder.internal.error('nav:navalgs:obstacleListCG:InvalidStateDimensions');
                    end
                end
            else
                if size(states,1) > numel(obj.CollisionObjectsInternal) 
                    coder.internal.error('nav:navalgs:obstacleListCG:NumStatesMatchNumIndices')
                end
                
                % Ensure the number of indices match the number of provided
                % states and that all indices are valid
                validateattributes(obstacleIndices, {'numeric'}, {'nonempty','positive','integer','numel',size(states,1),'<=',obj.NumObstacles}, 'updateObstaclePose', 'obstacleIndex', 2);
                
                % Update state for each obstacle corresponding to obstacleIndices
                H = eye(4);
                switch size(states,2)
                    case 3
                        % State = [x y theta]
                        obj.ObstaclePositions(obstacleIndices(:),:) = [states(:,1:2) zeros(numel(obstacleIndices),1)];
                        for i = 1:size(states,1)
                            zO = states(i,3);
                            R = [cos(zO) -sin(zO) 0;
                                 sin(zO)  cos(zO) 0;
                                      0       0  1];
                            H(1:3,:) = [R [states(i,1:2) 0]'];
                            obj.CollisionObjectsInternal{obstacleIndices(i)}.Pose = H;
                        end
                    case 6
                        % State = [x y z thetaX thetaY thetaZ]
                        R = eul2rotm(states(:,6:-1:4),'ZYX');
                        obj.ObstaclePositions(obstacleIndices(:),:) = states(:,1:3);
                        for i = 1:size(states,1)
                            H(1:3,:) = [R(:,:,i) states(i,1:3)'];
                            obj.CollisionObjectsInternal{obstacleIndices(i)}.Pose = H;
                        end
                    case 7
                        % State = [x y z qW qX qY qZ]
                        R = quat2rotm(states(:,4:7));
                        obj.ObstaclePositions(obstacleIndices(:),:) = states(:,1:3);
                        for i = 1:size(states,1)
                            H(1:3,:) = [R(:,:,i) states(i,1:3)'];
                            obj.CollisionObjectsInternal{obstacleIndices(i)}.Pose = H;
                        end
                    otherwise
                        coder.internal.error('nav:navalgs:obstacleListCG:InvalidStateDimensions');
                end
            end
        end
        
        function updateObstacleGeometry(obj, geometryInfo, obstacleIndices)
        %updateObstacleGeometry Update the geometric properties of one or more obstacles
        %
        %   updateObstacleGeometry(OBJ, GEOMETRYINFO) update the properties
        %   of each obstacle in OBJ using a cell array of property values,
        %   GEOMETRYINFO. Each cell in GEOMETRYINFO contains one of the
        %   following, and each set of properties must match the type
        %   of CollisionGeometryBase object stored in the corresponding
        %   cell:
        %
        %     [X Y Z]               - collisionBox 
        %     [Radius Length]       - collisionCylinder
        %     [Radius]              - collisionSphere
        %     N-by-[X Y Z]          - collisionMesh
        %
        %   updateObstacleGeometry(OBJ, STATES, OBSTACLEINDICES) update the
        %   properties for the set of obstacles in OBJ corresponding to the
        %   N-element vector of linear indices, OBSTACLEINDICES.
        
            narginchk(2,3)
            if nargin == 2
                if isempty(geometryInfo)
                    % Do nothing and return
                    return
                end
                % Update geometry of all CollisionObjects
                validateattributes(geometryInfo,{'cell'}, {'numel',numel(obj.CollisionObjectsInternal)}, 'updateObstacleGeometry', 'geometryInfo', 1);
                obstacleIndices = 1:numel(obj.CollisionObjectsInternal);
            else
                % Ensure first argument is a cell array
                validateattributes(geometryInfo,{'cell'}, {'nonempty'}, 'updateObstacleGeometry', 'geometryInfo', 1);
                
                % Ensure all indices are valid
                validateattributes(obstacleIndices,{'numeric'},{'nonempty','positive','integer','<=',numel(obj.CollisionObjectsInternal)}, 'updateObstacleGeometry', 'obstacleIndex', 1);
                
                % Ensure same number of geometryInfo as obstacleIndices
                if size(geometryInfo,1) ~= numel(obstacleIndices)
                    coder.internal.error('nav:navalgs:obstacleListCG:NumGeometriesMatchNumIndices');
                end
            end
            
            for i = 1:numel(obstacleIndices)
                objIdx = obstacleIndices(i);
                objClass = class(obj.CollisionObjectsInternal{objIdx});
                if strcmpi(objClass, 'collisionSphere')
                    %Sphere
                    validateattributes(geometryInfo{i},{'numeric'},{'positive','nonnan','numel',1},'updateObstacleGeometry',['geometryInfo{' num2str(i) '}']);
                    obj.CollisionObjectsInternal{objIdx}.Radius = geometryInfo{i};
                    obj.ObstacleRadii(objIdx) = geometryInfo{i};
                elseif strcmpi(objClass, 'collisionCylinder')
                    %Cylinder
                    validateattributes(geometryInfo{i},{'numeric'},{'positive','nonnan','numel',2},'updateObstacleGeometry',['geometryInfo{' num2str(i) '}']);
                    obj.CollisionObjectsInternal{objIdx}.Radius = geometryInfo{i}(1);
                    obj.CollisionObjectsInternal{objIdx}.Length = geometryInfo{i}(2);
                    obj.ObstacleRadii(objIdx) = sqrt(sum(geometryInfo{i}.^2));
                elseif strcmpi(objClass, 'collisionBox')
                    %Box
                    validateattributes(geometryInfo{i},{'numeric'},{'positive','nonnan','numel',3},'updateObstacleGeometry',['geometryInfo{' num2str(i) '}']);
                    obj.CollisionObjectsInternal{objIdx}.X = geometryInfo{i}(1);
                    obj.CollisionObjectsInternal{objIdx}.Y = geometryInfo{i}(2);
                    obj.CollisionObjectsInternal{objIdx}.Z = geometryInfo{i}(3);
                    obj.ObstacleRadii(objIdx) = sqrt(sum(geometryInfo{i}.^2));
                else
                    % Mesh
                    validateattributes(geometryInfo{i},{'numeric'},{'positive','nonnan','size',[nan 3]},'updateObstacleGeometry',['geometryInfo{' num2str(i) '}']);
                    obj.CollisionObjectsInternal{objIdx}.Vertices = geometryInfo{i};
                    obj.ObstacleRadii(objIdx) = max(sqrt(sum(geometryInfo{i}.^2,2)));
                end
                % Update dirty flag
                obj.obstaclesChanged = 1;
            end
        end
        
        function copyObj = copy(obj)
        %copy Returns a deep copy of the ObstacleList
            numObstacles = numel(obj.CollisionObjectsInternal);
            obstacleList = cell(numObstacles,1);
            for i = 1:numObstacles
                obstacleList{i} = copy(obj.CollisionObjectsInternal{i});
            end
            
            % Create new class using deep copy of all properties
            copyObj = nav.algs.internal.obstacleListCollisionGeometry(copy(obj.EgoObject), obstacleList);
        end
        
        function delete(obj)
        %delete Cleans up any graphics handles
            delete(obj.ObstacleHGGroupObj);
            delete(obj.EgoHGTransformObj);
            delete(obj.EgoCollisionHGGroupObj);
        end
        
        function axHandle = show(obj, varargin)
        %SHOW Display the ego object and obstacles in a figure
        %   SHOW(LIST) displays the obstacleList object, LIST, in the current
        %   axes with the axes labels representing the world coordinates.
        %
        %   AXHANDLE = SHOW(LIST, ___) returns the handle to the axes on
        %   which the LIST is shown.
        %
        %   SHOW(LIST,___,Name,Value) provides additional options specified
        %   by one or more Name,Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       'Parent'        - Axes to plot the map, specified as an axes handle.
        %
        %                         Default: gca
        %
        %       'FastUpdate'    - Boolean value used to speed up show method
        %                         for existing list plots. If you have 
        %                         previously plotted this list on the axes,
        %                         specify 1 to perform a lightweight update
        %                         to the map in the figure. Providing this
        %                         input will also leave graphics handles
        %                         created previously by other obstacleList
        %                         objects.
        %
        %                         Default: 0 (regular update)
            
            % Create default inputs
            p = inputParser;
            addParameter(p, 'Parent', [], @robotics.internal.validation.validateAxesUIAxesHandle);
            addParameter(p, 'FastUpdate', 0, @(x)(isnumeric(x)||islogical(x))&&~isempty(x)&&isscalar(x));
            parse(p, varargin{:});
            res = p.Results;
            axHandle = res.Parent;
            fastUpdate = res.FastUpdate;
            
            if isempty(axHandle)
                if ~fastUpdate
                    axHandle = newplot;
                else
                    axHandle = gca;
                end
            end
            
            % Get children of current axes
            children = axHandle.Children;
            if isempty(findobj(axHandle,'Type','Light'))
                % Turn lighting on if not already active
                light(axHandle);
            end
            
            % Create or update the EgoObject patch
            updateEgoPatch(obj, axHandle, children, fastUpdate);            
            
            % Create or update the CollisionObjects patch
            updateObstaclePatch(obj, axHandle, children, fastUpdate);
            axHandle.View = [-45,45]; % Isometric view
            
            if fastUpdate
                set(axHandle, 'DataAspectRatio', [1 1 1]);
            else
                set(axHandle, 'DataAspectRatio', [1 1 1], 'View', [-45, 45], ...
                'XLimMode','manual','YLimMode','manual','ZLimMode','manual')
                axis(axHandle,'equal');
            end
        end
    end
    
    methods (Access = private)
        function updateEgoPatch(obj, axHandle, children, fastUpdate)
        %updateObstaclePatch Creates or updates the patch object for the CollisionObjects
        
            % Check if the EgoObject patch is in the axes
            missingEgo = isempty(obj.EgoHGTransformObj) || ~any(children == obj.EgoHGTransformObj);
            
            if ~fastUpdate
                % If FastUpdate is not provided, then delete EgoObject patches
                % previously plotted and abandoned, or plotted by other ObstacleLists
                egoChildren = findobj(axHandle,'Tag','ObstacleListEgoObject');
                if missingEgo
                    delete(egoChildren);
                else
                    delete(egoChildren(egoChildren ~= obj.EgoHGTransformObj));
                end
            end
            
            % Calculate new location of EgoObject
            R = quat2rotm(obj.EgoObject.Quaternion);

            if missingEgo
                % Create the patch if it is missing
                obj.EgoHGTransformObj = hgtransform('Parent',axHandle,'Tag','ObstacleListEgoObject','Matrix',[R obj.EgoObject.Position'; 0 0 0 1]);
                p = patch(axHandle, 'Vertices', obj.EgoObject.VisualMeshVertices, 'Faces', obj.EgoObject.VisualMeshFaces, 'FaceColor', [0, 0, 1], 'EdgeColor', 'none');
                set(p,'Parent',obj.EgoHGTransformObj);
            else
                % Otherwise update the data directly
                set(obj.EgoHGTransformObj,'Matrix',[R obj.EgoObject.Position'; 0 0 0 1]);
            end
        end
        
        function updateObstaclePatch(obj, axHandle, children, fastUpdate)
        %updateObstaclePatch Creates or updates the patch object for the CollisionObjects
        
            % Check if the CollisionObjects patch is in the axes
            missingObstacles = isempty(obj.ObstacleHGGroupObj) || ~any(children == obj.ObstacleHGGroupObj);
            
            if ~fastUpdate
                % If FastUpdate is not provided, then delete CollisionObject patches
                % previously plotted and abandoned, or plotted by other ObstacleLists 
                obstacleChildren = findobj(axHandle,'Tag','ObstacleListCollisionObject');
                if missingObstacles
                    delete(obstacleChildren);
                else
                    delete(obstacleChildren(obstacleChildren ~= obj.ObstacleHGGroupObj));
                end
            end
            
            if obj.obstaclesChanged || missingObstacles
                
                if missingObstacles
                    % Create the hggroup for all obstacles if it is missing
                    obj.ObstacleHGGroupObj = hggroup(axHandle);
                    obj.ObstacleHGGroupObj.Tag = 'ObstacleListCollisionObject';
                end

                % Remove old transforms/patches
                delete(obj.ObstacleHGGroupObj.Children);

                % Add new patches/transforms
                for i = obj.NumObstacles:-1:1
                    R = quat2rotm(obj.CollisionObjectsInternal{i}.Quaternion);
                    hgT = hgtransform('Parent',obj.ObstacleHGGroupObj,'Matrix',[R obj.CollisionObjectsInternal{i}.Position'; 0 0 0 1], 'Tag', num2str(i));
                    p = patch(axHandle, 'Vertices', obj.CollisionObjectsInternal{i}.VisualMeshVertices, 'Faces', obj.CollisionObjectsInternal{i}.VisualMeshFaces, 'FaceColor', [1, 0.5, 0], 'EdgeColor', 'none');
                    set(p,'Parent',hgT);
                end
            else
                for i = 1:obj.NumObstacles
                    R = quat2rotm(obj.CollisionObjectsInternal{i}.Quaternion);
                    obj.ObstacleHGGroupObj.Children(i).Matrix = [R obj.CollisionObjectsInternal{i}.Position'; 0 0 0 1];
                end
            end
            obj.obstaclesChanged = 0;
        end
    end
    
    methods (Access = private, Static)
        function [egoObject, obstacleList, positions, radii] = parseConstructor(varargin)
        %parseConstructor Parse inputs to constructor for construction of nav.ObstacleList
            narginchk(0,3)
            switch numel(varargin)
                case 0
                    inputs = varargin;
                    positions = [];
                    radii = [];
                case 1
                    validateattributes(varargin{1},{'robotics.core.internal.CollisionGeometryBase'},{'nonempty','scalar'},'nav.algs.internal.obstacleListCollisionGeometry','EgoObject',1);
                    inputs = {'EgoObject',varargin{1}};
                    positions = [];
                    radii = [];
                otherwise
                    validateattributes(varargin{1},{'robotics.core.internal.CollisionGeometryBase'},{'nonempty','scalar'},'nav.algs.internal.obstacleListCollisionGeometry','EgoObject',1);
                    [obstacles, positions, radii] = nav.algs.internal.obstacleListCollisionGeometry.convertObstacleInput(varargin{2},'nav.algs.internal.obstacleListCollisionGeometry','collisionObjects',2);
                    inputs = {'EgoObject',varargin{1},'ObstacleList',obstacles(:)};
            end
            
            % Create default values for constructor
            defaultNames = {'EgoObject', 'ObstacleList'};
            defaultValues = {collisionBox(2,2,2), {}};
            parser = robotics.core.internal.NameValueParser(defaultNames, defaultValues);
            
            % Parse incoming values and populate inputs to constructor
            parse(parser, inputs{:});
            egoObject = parser.parameterValue('EgoObject');
            obstacleList = parser.parameterValue('ObstacleList');
        end
        
        function [obstacles, positions, boundingRadii] = convertObstacleInput(collisionObjects, functionName, argName, argNum)
        %convertObstacleInput Converts obstacle inputs to cell-array format
        %
        %   Converts the incoming obstacles to a cell-array and pulls out
        %   translation and bounding radii used for pre-collisionCheck
        %   culling.
        
            % Validate that input consists only of CollisionGeometryBase objects
            numObjs = numel(collisionObjects);
            obstacles = cell(numObjs,1);
            boundingRadii = inf(numObjs,1);
            positions = zeros(numObjs,3);
            if iscell(collisionObjects)
                % Verify that each element of the cell-array is a collision
                % object
                cellfun(@(x)validateattributes(x,{'robotics.core.internal.CollisionGeometryBase'},{'nonempty','scalar'},functionName,argName,argNum),collisionObjects);
                for i = 1:numel(collisionObjects)
                    % Add copy of the obstacle to the cell-array
                    obstacles{i} = copy(collisionObjects{i});
                end
            else
                % Verify that the input is a scalar or vector of collisionObjects
                validateattributes(collisionObjects,{'robotics.core.internal.CollisionGeometryBase'},{'nonempty'},functionName,argName,argNum)

                % Convert input to cell-array
                if numObjs > 1
                    % Convert input
                    for i = 1:numObjs
                        obstacles{i} = copy(collisionObjects(i));
                    end
                else
                    obstacles = {copy(collisionObjects)};
                end
            end
            
            for i = 1:numObjs
                % Cache position and radius of bounding sphere for each
                % object.
                boundingRadii(i) = max(sqrt(sum(obstacles{i}.VisualMeshVertices.^2,2)));
                positions(i,:) = obstacles{i}.Position;
            end
        end
    end
    
    %Custom property display
    methods (Access = protected)
      function propgrp = getPropertyGroups(obj)
         if ~isscalar(obj)
            propgrp = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
         else
            propList = struct(...
               'EgoObject',obj.EgoObject,...
               'NumObstacles',obj.NumObstacles,...
               'CollisionObjects',obj.CollisionObjects);
            propgrp = matlab.mixin.util.PropertyGroup(propList);
         end
      end
      
      function egoObject = validateEgoObject(obj, newEgoObject)
        %validateEgoObject Ensures the new EgoObject is of the correct size and type
        %
        %   EGOOBJECT = validateEgoObject(OBJ, NEWEGOOBJECT) Method used by 
        %   the parent class to validate the EgoObject for the set.EgoObject 
        %   method and perform any formatting required by the internal
        %   implementation. This method takes in NEWEGOOBJECT, which must
        %   be a scalar collision object based on the CollisionGeometryBase
        %   class, and returns the same object as output, EGOOBJECT, if all
        %   of the validations are passed.
        
            validateattributes(newEgoObject,{'robotics.core.internal.CollisionGeometryBase'},{'nonempty','scalar'},'setEgoObject','newEgoObject',1);
            objType = find(strcmpi(class(newEgoObject),nav.algs.internal.obstacleListCollisionGeometry.ValidClasses),1);
            if isempty(objType)
                % Object is either of type 'collisionMesh', or is a custom
                % geometry defined by the user. Treat as collisionMesh for
                % precheck
                obj.EgoObjectType = 0;
            else
                % Object is one of the basic geometry types
                obj.EgoObjectType = objType;
            end
            egoObject = newEgoObject;
        end
   end
end
