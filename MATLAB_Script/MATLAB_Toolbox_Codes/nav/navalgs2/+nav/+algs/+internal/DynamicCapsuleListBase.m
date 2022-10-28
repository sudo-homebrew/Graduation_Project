classdef (Abstract, Hidden) DynamicCapsuleListBase < nav.algs.internal.DynamicObstacleList
% This class is for internal use only. It may be removed in the future.
    
%DynamicCapsuleListBase Base class for the 2D/3D dynamic capsule lists
%
%   See also dynamicCapsuleList, dynamicCapsuleList3D.

%   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    properties (Transient, Access = ?nav.algs.internal.InternalAccess)
        %ObstaclePatchObj Handle to the patch object representing the obstacles
        ObstaclePatchObj
        obstacleVisChanged = 1
        
        %EgoPatchObj Handle to the patch object that represents the ego-object
        EgoPatchObj
        egoVisChanged = 1
        
        %TopLevelGroup Group containing the ego/obstacle groups
        TopLevelGroup
        
        %ObjLength Matrix containing length of all obstacles
        ObjLength;
        
        %ObjRadius Matrix containing radii of all obstacles
        ObjRadius;
        
        %ObjXYZ Matrix containing the center positions of all obstacles
        ObjXYZ;
        
        %ObjV Matrix containing the orientation of all obstacles
        ObjV;
        
        %EgoLength Matrix containing length of all ego bodies
        EgoLength;
        
        %EgoRadius Matrix containing radii of all ego bodies
        EgoRadius;
        
        %EgoXYZ Matrix containing the center positions of all ego bodies
        EgoXYZ;
        
        %EgoV Matrix containing the orientation of all ego bodies
        EgoV;
    
        %NumStates Number of predicted states
        NumStates = 1
        
        %CacheForReuse Determines whether ego/obstacle information is cached when calling 'checkCollision' repeatedly
        CacheForReuse = true
        
        %LastQueryIndices Set of most recently queried indices
        LastQueryIndices = nan
    end
        
    properties (Access = protected)
        %DefaultGeometry A var-sizeable struct-array for storing geometric object properties
        DefaultGeometry
    end
    
    properties (Abstract, Access = protected)
        %CapHelper Class storing static helper methods
        CapHelper
    end
    
    properties (Abstract, Constant, Access = ?nav.algs.internal.DynamicCapsuleListBase)
        %Dimension Determines whether the list operates in SE2 or SE3
        Dimension
    end
    
    methods
        function obj = DynamicCapsuleListBase(capHelper)
        %dynamicCapsuleList Creates a Capsule-based obstacleList
            narginchk(1,1);
            
            % Create template data structures used for internal storage
            defaultCapsule = struct(capHelper);
            capDefault = repelem(defaultCapsule,0,1);
            defaultState = defaultCapsule.StatesInternal;
            
            % Construct empty list
            obj = obj@nav.algs.internal.DynamicObstacleList(defaultState, capDefault, capDefault);
            obj.CapHelper = capHelper;
            
            % Provide default geometry and primitive to the base class
            if coder.target('MATLAB')
                % Store struct fields for input validation
                obj.PrimFields = fields(obj.CapHelper);
                obj.StateFields = 'States';
                obj.GeomFields = fields(obj.CapHelper.Geometry);
            end
            obj.DefaultGeometry = defaultCapsule.Geometry;
            
            % Cache
            obj.cacheEgoInfo();
            obj.cacheObstacleInfo();
        end
        
        function [egoIDs, geomStruct, status] = egoGeometry(obj, varargin)
        %egoGeometry Geometric properties for set of ego bodies
        %    
        %   [EGOIDS, GEOMSTRUCT] = egoGeometry(OBJ) returns a structure
        %   array, GEOMSTRUCT, containing geometry of all ego bodies in the
        %   list. The Nth element of GEOMSTRUCT contains the geometric
        %   properties of the ego body matching EGOIDS(N).
        %
        %   [EGOIDS, GEOMSTRUCT] = egoGeometry(OBJ, EGOIDS) returns a 
        %   structure array for ego bodies matching the EGOIDS vector. If an
        %   identifier is not found in the list, the corresponding entry of
        %   GEOMSTRUCT contains default geometry.
        %
        %   [EGOIDS, GEOMSTRUCT, STATUS] = egoGeometry(OBJ, EGOIDS) returns
        %   an optional output, STATUS, an N-element vector with values of
        %   1 (body found), 0 (body not found), -1 (duplicate input).
        %
        %   See also dynamicCapsuleList.GEOMETRYSTRUCT, dynamicCapsuleList3D.GEOMETRYSTRUCT
        
            narginchk(1,2);
            
            % Validate inputs and retrieve indices of matching objects
            [status, egoIDs, internalIndices] = obj.validateIntrospectionInputs('egoGeometry','egoIDs',obj.EgoIDs,varargin{:});
            
            % Construct output variable
            geomStruct = obj.getGeometry(egoIDs);
            
            % Populate ID's and geometric properties for objects found in
            % the list
            for i = 1:numel(geomStruct)
                if internalIndices(i) ~= 0
                    idx = internalIndices(i);
                    geomStruct(i).Geometry = obj.EgoObjectsInternal(idx).Geometry;
                end
            end
        end
        
        function [egoIDs, poseStruct, status] = egoPose(obj, varargin)
        %egoPose Poses for set of ego bodies
        %    
        %   [EGOIDS, POSESTRUCT] = egoPose(OBJ) returns a structure array
        %   POSESTRUCT, containing states of all ego bodies in the list.
        %   The Nth element of POSESTRUCT contains the matrix of states stored
        %   by the ego body matching EGOIDS(N).
        %
        %   [EGOIDS, POSESTRUCT] = egoPose(OBJ, EGOIDS) returns a 
        %   structure array for ego bodies matching the EGOIDS vector. If an
        %   identifier is not found in the list, the corresponding entry of
        %   POSESTRUCT contain a 1-row matrix containing the default state.
        %
        %   [EGOIDS, POSESTRUCT, STATUS] = egoPose(OBJ, EGOIDS) returns
        %   an optional output, STATUS, an N-element vector with values of
        %   1 (body found), 0 (body not found), -1 (duplicate input).
        
            narginchk(1,2);
            
            % Validate inputs and retrieve indices of matching objects
            [status, egoIDs, internalIndices] = obj.validateIntrospectionInputs('egoPose','egoIDs',obj.EgoIDs,varargin{:});
            
            poseStruct = obj.getPoses(true, internalIndices, egoIDs);
        end
        
        function isEqual = isequal(this, other)
        %isequal Performs a lightweight check to ensure the two lists represent the same set of objects.
        %
        %   NOTE: Internal state information, such as cached capsule
        %   information or properties related to visualization info, is NOT
        %   included in the comparison.
        
            isEqual = ...
                isequal(this.EgoObjectsInternal,other.EgoObjectsInternal) && ...
                isequal(this.CollisionObjectsInternal,other.CollisionObjectsInternal) && ...
                isequal(this.EgoIDs,other.EgoIDs) && ...
                isequal(this.ObstacleIDs,other.ObstacleIDs) && ...
                this.NumEgos == other.NumEgos && ...
                this.NumObstacles == other.NumObstacles && ...
                this.MaxNumSteps == other.MaxNumSteps;
        end
        
        function [obstacleIDs, geomStruct, status] = obstacleGeometry(obj, varargin)
        %obstacleGeometry Geometric properties for set of obstacles
        %
        %   [OBSTACLEIDS, GEOMSTRUCT] = obstacleGeometry(OBJ) returns a structure
        %   array, GEOMSTRUCT, containing geometry of all obstacles in the
        %   list. The Nth element of GEOMSTRUCT contains the geometric
        %   properties of the obstacle matching OBSTACLEIDS(N).
        %
        %   [OBSTACLEIDS, GEOMSTRUCT] = obstacleGeometry(OBJ, OBSTACLEIDS) 
        %   returns a structure array for obstacles matching the OBSTACLEIDS
        %   vector. If an identifier is not found in the list, the
        %   corresponding entry of GEOMSTRUCT contains default geometry.
        %
        %   [OBSTACLEIDS, GEOMSTRUCT, STATUS] = obstacleGeometry(OBJ, OBSTACLEIDS)
        %   returns an optional output, STATUS, an N-element vector with values of
        %   1 (body found), 0 (body not found), -1 (duplicate input).
            
            narginchk(1,2);
            
            % Validate inputs and retrieve indices of matching objects
            [status, obstacleIDs, internalIndices] = obj.validateIntrospectionInputs('egoGeometry','obstacleIDs',obj.ObstacleIDs,varargin{:});
            
            % Construct output variable
            geomStruct = obj.getGeometry(obstacleIDs);
            
            % Populate ID's and geometric properties for objects found in
            % the list
            for i = 1:numel(geomStruct)
                if internalIndices(i) ~= 0
                    idx = internalIndices(i);
                    geomStruct(i).Geometry = obj.CollisionObjectsInternal(idx).Geometry;
                end
            end
        end
        
        function [obstacleIDs, curPoseStruct, status] = obstaclePose(obj, varargin)
        %obstaclePose Poses for set of obstacles
        %
        %   [OBSTACLEIDS, POSESTRUCT] = obstaclePose(OBJ) returns a structure array
        %   POSESTRUCT, containing states of all obstacles in the list.
        %   The Nth element of POSESTRUCT contains the matrix of states stored
        %   by the obstacle matching OBSTACLEIDS(N).
        %
        %   [OBSTACLEIDS, POSESTRUCT] = obstaclePose(OBJ, OBSTACLEIDS) 
        %   returns a structure array for obstacles matching the OBSTACLEIDS
        %   vector. If an identifier is not found in the list, the 
        %   corresponding entry of POSESTRUCT contain a 1-row matrix 
        %   containing the default state.
        %
        %   [OBSTACLEIDS, POSESTRUCT, STATUS] = obstaclePose(OBJ, OBSTACLEIDS) 
        %   returns an optional output, STATUS, an N-element vector with values of
        %   1 (body found), 0 (body not found), -1 (duplicate input).
        %
        %   See also dynamicCapsuleList.POSESTRUCT, dynamicCapsuleList3D.POSESTRUCT
            
            narginchk(1,2);
            
            % Validate inputs and retrieve indices of matching objects
            [status, obstacleIDs, internalIndices] = obj.validateIntrospectionInputs('obstaclePose','obstacleIDs',obj.ObstacleIDs,varargin{:});
            
            curPoseStruct = obj.getPoses(false, internalIndices, obstacleIDs);
        end
        
        function status = removeEgo(obj, varargin)
        %removeEgo Remove ego bodies from the list
        % 
        %   removeEgo(OBJ, EGOIDS) removes ego bodies with ID matching those
        %   found in the N-element vector EGOIDS.
        %
        %   STATUS = removeEgo(OBJ, EGOIDS) returns an optional output, STATUS,
        %   an N-element vector with values of 1 (body removed), 
        %   0 (body not found), -1 (duplicate input).
        
            narginchk(1,2);
            
            % Validate inputs and retrieve indices of matching objects
            [stat, indices] = obj.validateRemove('removeEgo','egoIDs',obj.EgoIDs,varargin{:});
            
            if ~isempty(stat)
                % Remove 
                obj.EgoObjectsInternal(indices(indices~=0)) = [];
                obj.EgoIDs(indices(indices~=0)) = [];
                obj.NumEgos = numel(obj.EgoObjectsInternal);

                % Mark visual/cached data as 'dirty'
                obj.EgosChanged = 1;
                obj.egoVisChanged = 1;
            end
            
            if nargout == 1
                status = stat;
            else
                status = obj;
            end
        end
        
        function status = removeObstacle(obj, varargin)
        %removeObstacle Remove obstacles from the list
        % 
        %   removeObstacle(OBJ, OBSTACLEIDS) removes obstacles with ID
        %   matching those found in the N-element vector OBSTACLEIDS.
        %
        %   STATUS = removeObstacle(OBJ, OBSTACLEIDS) returns an optional 
        %   output, STATUS, an N-element vector with values of 1 (body removed), 
        %   0 (body not found), -1 (duplicate input).
        
            narginchk(1,2);
            
            % Validate inputs and retrieve indices of matching objects
            [stat, indices] = obj.validateRemove('removeObstacle','obstacleIDs',obj.ObstacleIDs,varargin{:});
            if ~isempty(stat)
                % Remove objects with matching indices
                obj.CollisionObjectsInternal(indices(indices~=0)) = [];
                obj.ObstacleIDs(indices(indices~=0)) = [];
                obj.NumObstacles = numel(obj.CollisionObjectsInternal);

                % Mark visual/cached data as 'dirty'
                obj.ObstaclesChanged = 1;
                obj.obstacleVisChanged = 1;
            end
            
            if nargout == 1
                status = stat;
            else
                status = obj;
            end
        end
                
        function status = updateEgoGeometry(obj, varargin)
        %updateEgoGeometry Update geometric properties of ego bodies
        %
        %   updateEgoGeometry(OBJ, EGOIDS, GEOMETRYSTRUCT) updates geometry
        %   for a set of ego bodies. Each element of GEOMETRYSTRUCT contains
        %   a struct with Length, Radius, and FixedTransform fields. The 
        %   Nth element of GEOMETRYSTRUCT updates the ego body matching 
        %   EGOIDS(N). If a specified ID does not exist, a new ego body is
        %   added to the list with provided states and default geometric properties.
        %
        %   STATUS = updateEgoGeometry(OBJ, EGOIDS, GEOMETRYSTRUCT) returns
        %   an optional output, STATUS, an N-element vector with values of
        %   1 (body added), 0 (body updated), -1 (duplicate input).
        %
        %   To see the fields of GEOMETRYSTRUCT, follow the corresponding link:
        %   See also dynamicCapsuleList.GEOMETRYSTRUCT, dynamicCapsuleList3D.GEOMETRYSTRUCT
        
            narginchk(1,3)
        
            % Validate inputs and retrieve indices of matching objects
            [stat, egoIDs, geometryStruct, uniqueInternalIdx, uniqueExternalIdx] = ...
                obj.validateUpdate('updateEgoGeometry', 'egoIDs', 'geometryStruct', ...
                obj.GeomFields, obj.EgoIDs, varargin{:});
            
            if ~isempty(stat)
                % Allocate memory for new set of obstacles
                [obj.EgoObjectsInternal, obj.EgoIDs, obj.NumEgos] = obj.updateList(obj.NumEgos, obj.EgoObjectsInternal, obj.EgoIDs, uniqueInternalIdx, uniqueExternalIdx, egoIDs, geometryStruct, [1 0]);

                % Mark visual/cached data as 'dirty'
                obj.EgosChanged = 1;
                obj.egoVisChanged = 1;
            end
            
            if nargout == 1
                status = stat;
            else
                status = obj;
            end
        end
        
        function status = updateEgoPose(obj, varargin)
        %updateEgoPose Update ego-body poses
        %
        %   updateEgoPose(OBJ, EGOIDS, POSESTRUCT) updates states for a set
        %   of ego bodies. Each element of POSESTRUCT may contain a matrix
        %   of states, and the Nth element of POSESTRUCT updates the ego
        %   body matching EGOIDS(N). If a specified ID does not exist, a 
        %   new ego body is added to the list with provided states and
        %   default geometric properties.
        %
        %   STATUS = updateEgoPose(OBJ, EGOIDS, POSESTRUCT) returns an optional 
        %   output, STATUS, an N-element vector with values of 1 (body added), 
        %   0 (body updated), -1 (duplicate input).
        %
        %   To see the fields of POSESTRUCT, follow the corresponding link:
        %   See also dynamicCapsuleList.POSESTRUCT, dynamicCapsuleList3D.POSESTRUCT
        
            narginchk(1,3)
            
            % Validate inputs and retrieve indices of matching objects
            [stat, egoIDs, poseStruct, uniqueInternalIdx, uniqueExternalIdx] = ...
                obj.validateUpdate('updateEgoPose', 'egoIDs', 'poseStruct', ...
                obj.StateFields, obj.EgoIDs, varargin{:});
            
            if ~isempty(stat)
                % Allocate memory for new set of obstacles
                [obj.EgoObjectsInternal, obj.EgoIDs, obj.NumEgos] = ...
                    obj.updateList(obj.NumEgos, obj.EgoObjectsInternal, ...
                    obj.EgoIDs, uniqueInternalIdx, uniqueExternalIdx, egoIDs, poseStruct, [0 1]);

                % Mark visual/cached data as 'dirty'
                obj.EgosChanged = 1;
                obj.egoVisChanged = 1;

                % Reset the query times
                obj.LastQueryIndices = nan;
            end
            
            if nargout == 1
                status = stat;
            else
                status = obj;
            end
        end
        
        function status = updateObstacleGeometry(obj, varargin)
        %updateObstacleGeometry Update geometric properties of obstacles
        %
        %   updateObstacleGeometry(OBJ, OBSTACLEIDS, GEOMETRYSTRUCT) updates
        %   geometry for a set of obstacles. Each element of GEOMETRYSTRUCT
        %   contains a struct with Length, Radius, and FixedTransform fields.
        %   The  Nth element of GEOMETRYSTRUCT updates the obstacle matching 
        %   OBSTACLEIDS(N). If a specified ID does not exist, a new obstacle is
        %   added to the list with provided states and default geometric properties.
        %
        %   STATUS = updateObstacleGeometry(OBJ, OBSTACLEIDS, GEOMETRYSTRUCT)
        %   returns an optional output, STATUS, an N-element vector with 
        %   values of 1 (body added), 0 (body updated), -1 (duplicate input).
        %
        %   To see the fields of GEOMETRYSTRUCT, follow the corresponding link:
        %   See also dynamicCapsuleList.GEOMETRYSTRUCT, dynamicCapsuleList3D.GEOMETRYSTRUCT
        
            narginchk(1,3)
            
            % Validate inputs and retrieve indices of matching objects
            [stat, obstacleIDs, geometryStruct, uniqueInternalIdx, uniqueExternalIdx] = ...
                obj.validateUpdate('updateObstacleGeometry', 'obstacleIDs', 'geometryStruct', ...
                obj.GeomFields, obj.ObstacleIDs, varargin{:});
            
            if ~isempty(stat)
                % Allocate memory for new set of capsules
                [obj.CollisionObjectsInternal, obj.ObstacleIDs, obj.NumObstacles] = ...
                    obj.updateList(obj.NumObstacles, obj.CollisionObjectsInternal, ...
                    obj.ObstacleIDs, uniqueInternalIdx, uniqueExternalIdx, obstacleIDs, geometryStruct, [1 0]);

                % Mark visual/cached data as 'dirty'
                obj.ObstaclesChanged = 1;
                obj.obstacleVisChanged = 1;
            end
            
            if nargout == 1
                status = stat;
            else
                status = obj;
            end
        end
        
        function status = updateObstaclePose(obj, varargin)
        %updateObstaclePose Update obstacle poses
        %
        %   updateObstaclePose(OBJ, OBSTACLEIDS, POSESTRUCT) updates states
        %   for a set of obstacles. Each element of POSESTRUCT may contain
        %   a matrix of states, and the Nth element of POSESTRUCT updates
        %   the ego body matching OBSTACLEIDS(N). If a specified ID does not
        %   exist, a new obstacle is added to the list with provided states and
        %   default geometric properties.
        %
        %   STATUS = updateObstaclePose(OBJ, OBSTACLEIDS, POSESTRUCT) returns an optional 
        %   output, STATUS, an N-element vector with values of 1 (body added), 
        %   0 (body updated), -1 (duplicate input).
        %
        %   To see the fields of POSESTRUCT, follow the corresponding link:
        %   See also dynamicCapsuleList.POSESTRUCT, dynamicCapsuleList3D.POSESTRUCT
            
            narginchk(1,3)
        
            % Validate inputs and retrieve indices of matching objects
            [stat, obstacleIDs, poseStruct, uniqueInternalIdx, uniqueExternalIdx] = ...
                obj.validateUpdate('updateObstaclePose', 'obstacleIDs', 'poseStruct', ...
                obj.StateFields, obj.ObstacleIDs, varargin{:});
            
            if ~isempty(stat)
                % Allocate memory for new set of capsules
                [obj.CollisionObjectsInternal, obj.ObstacleIDs, obj.NumObstacles] = ...
                obj.updateList(obj.NumObstacles, obj.CollisionObjectsInternal, ...
                obj.ObstacleIDs, uniqueInternalIdx, uniqueExternalIdx, obstacleIDs, poseStruct, [0 1]);

                % Mark visual/cached data as 'dirty'
                obj.ObstaclesChanged = 1;
                obj.obstacleVisChanged = 1;
            end
            
            if nargout == 1
                status = stat;
            else
                status = obj;
            end
        end

        function axHandle = show(obj, varargin)
        %show Display ego bodies and obstacles in environment
        %   SHOW(OBJ) displays the initial state of all ego bodies and 
        %   obstacles in the environment.
        %
        %   AXHANDLE = SHOW(OBJ) returns the handle to the axes on which 
        %   the capsule list is shown.
        %
        %   __ = SHOW(OBJ,Name,Value) provides additional options specified
        %   by Name,Value pair arguments. Name must appear inside single 
        %   quotes (''). You can specify several name-value pair arguments
        %   in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       'Parent'                - Axes to plot the map, specified 
        %                                 as an axes handle.
        %                                   Default: gca
        %
        %       'FastUpdate'            - Boolean used to speed up show method
        %                                 for existing plots. If this object
        %                                 has been previously plotted on the axes,
        %                                 specify 1 to perform a lightweight update
        %                                 to the figure.
        %                                   Default: false
        %
        %       'TimeStep'              - Time step(s) to display in the
        %                                 range of [1 MaxNumSteps]. 
        %                                   Default: 1
        %
        %       'ShowCollisions'        - Boolean value indicating whether 
        %                                 to show collisions. If set to 1,
        %                                 collisions are calculated and
        %                                 shown in red.
        %                                   Default: false
        %
        %       'EgoIDs'                - Only display ego bodies whose IDs
        %                                 are found in the input vector.
        %                                   Default: obj.EgoIDs
        %
        %       'ObstacleIDs'           - Only display obstacles whose IDs
        %                                 are found in the input vector.
        %                                   Default: obj.ObstacleIDs
            
            % If called in code generation, throw incompatibility error
            coder.internal.errorIf(~coder.target('MATLAB'), 'nav:navalgs:dynamiccapsulelist:GraphicsSupportCodegen', 'show');
        
            % Parse inputs to show
            [axHandle, fastUpdate, timeSteps, showCollisions, egoIdx, obsIdx] = ...
                obj.parseShowInputs(varargin);
            
            % Prepare axes
            [axHandle, holdOn] = obj.getAxes(axHandle, fastUpdate);
            
            % Create or update the CollisionObjects patch
            obj.updateObstaclePatch(axHandle, holdOn, fastUpdate, timeSteps, obsIdx);
            
            % Create or update the EgoObjectsInternal patch
            obj.updateEgoPatch(axHandle, holdOn, fastUpdate, timeSteps, egoIdx, showCollisions);
            
            if fastUpdate && obj.Dimension == 2
                drawnow limitrate
            end
        end
    end
    
    %% Show Helpers
    methods (Access = ?nav.algs.internal.DynamicCapsuleListBase)
        function updateEgoPatch(obj, axHandle, holdOn, fastUpdate, timeSteps, egoIndices, showCollisions)
        %updateEgoPatch Creates or updates the patch object for the CollisionObjects
            
            % Prepare axes for update and retrieve current number of
            % graphic handles maintained by the list
            [obj.EgoPatchObj, numToDisplay, numStored] = obj.prepAxes(axHandle, obj.EgoPatchObj, holdOn, fastUpdate, egoIndices, 'Ego');
            
            if timeSteps(1) == 1 && numel(timeSteps) > size(obj.EgoXYZ,1)/obj.Dimension
                % Only display the minimum number of ego bodies necessary
                showAllStates = true;
                timeSteps = (1:size(obj.EgoXYZ,1)/obj.Dimension)';
            else
                showAllStates = false;
            end
            
            % Assign colors that fade to grey over time
            if showCollisions == true
            % Check for collisions between ego bodies and obstacles
                if isnan(obj.LastQueryIndices)
                    isColliding = obj.checkCollision();
                else
                    isColliding = obj.checkAtIndices(obj.LastQueryIndices);
                end
                colorColliding = {generatePatches(obj, axHandle, obj.EgoPatchObj, obj.EgoObjectsInternal, obj.EgoXYZ, obj.EgoV, timeSteps, egoIndices, 'Ego', numToDisplay, numStored, showAllStates, isColliding)};
            else
                generatePatches(obj, axHandle, obj.EgoPatchObj, obj.EgoObjectsInternal, obj.EgoXYZ, obj.EgoV, timeSteps, egoIndices, 'Ego', numToDisplay, numStored, showAllStates);
                colorColliding = {};
            end
            
            if numToDisplay > 0
                % Color remaining patches
                colorBases = axHandle.ColorOrder(1,:).*linspace(1,.9,obj.NumEgos)';
                obj.colorPatches(obj.EgoPatchObj, numToDisplay, egoIndices, timeSteps, colorBases, colorColliding{:});
            end
        end
        
        function updateObstaclePatch(obj, axHandle, holdOn, fastUpdate, timeSteps, obstacleIndices)
        %updateObstaclePatch Creates or updates the patch object for the CollisionObjects
            
            % Clean up previously plotted obstacles
            [obj.ObstaclePatchObj, numToDisplay, numStored] = prepAxes(obj, axHandle, obj.ObstaclePatchObj, holdOn, fastUpdate, obstacleIndices, 'Obstacle');
            
            if numel(timeSteps)==obj.MaxNumSteps
                showAllStates = true;
            else
                showAllStates = false;
            end
            
            % Create new patches
            obj.generatePatches(axHandle, obj.ObstaclePatchObj, obj.CollisionObjectsInternal, obj.ObjXYZ, obj.ObjV, timeSteps, obstacleIndices, 'Obstacle', numToDisplay, numStored, showAllStates);
            
            if numToDisplay > 0
                % Color remaining patches
                colorOrder = axHandle.ColorOrder(2:end,:);
                colorBases = colorOrder(mod(obj.ObstacleIDs+1,max(size(colorOrder,1)-1,1))+1,:);
                obj.colorPatches(obj.ObstaclePatchObj, numToDisplay, obstacleIndices, timeSteps, colorBases);
            end
        end
        
        function [patchObj, numToDisplay, numStored] = prepAxes(~, axHandle, patchObj, holdOn, fastUpdate, objIDs, tag)
            % Check if the EgoObjectsInternal patch is in the axes
            missingPatch = isempty(patchObj) || ~isvalid(patchObj);
            
            % Create the patch if it is missing, otherwise determine how
            % many objects need to be created or removed.
            numToDisplay = numel(objIDs);
            if missingPatch || (holdOn == true && fastUpdate == false)
                patchObj = hggroup('Parent',axHandle);
                patchObj.Annotation.LegendInformation.IconDisplayStyle = 'children';
                patchObj.Tag = ['ObstacleList' tag 'Group'];
                numStored = 0;
            elseif numel(patchObj.Children) ~= numToDisplay
                numStored = numel(patchObj.Children);
            else
                numStored = numToDisplay;
            end
        end
        
        function [axHandle, fastUpdate, timeSteps, showCollisions, egoIdx, obsIdx] = parseShowInputs(obj, inputs)
        %parseShowInputs Parse inputs and prep internal state
            % Create default inputs
            defaultNames  = {'Parent','FastUpdate','TimeStep','ShowCollisions','EgoIDs','ObstacleIDs'};
            defaultValues = {[], 0, 1, 0, obj.EgoIDs, obj.ObstacleIDs};

            % Parse inputs
            parser = robotics.core.internal.NameValueParser(defaultNames, defaultValues);
            parse(parser, inputs{:});
            axHandle   = parser.parameterValue('Parent');
            fastUpdate = parser.parameterValue('FastUpdate');
            timeSteps = parser.parameterValue('TimeStep');
            showCollisions = parser.parameterValue('ShowCollisions');
            displayEgoID = parser.parameterValue('EgoIDs');
            displayObstacleID = parser.parameterValue('ObstacleIDs');

            % Verify that the requested time steps are valid
            validateattributes(timeSteps,{'numeric'},{'vector','integer','>=',1,'<=',obj.MaxNumSteps},'show','timeSteps');
            
            % Find unique steps
            timeSteps = unique(timeSteps(:));

            % Retrieve IDs that should be displayed

            if ~isequal(displayEgoID,obj.EgoIDs)
                validateattributes(displayEgoID,{'numeric','vector'},{'integer'});
                egoIdx = obj.findIdxByID(obj.EgoIDs, displayEgoID);
                egoIdx(egoIdx == 0) = [];
                egoIdx = unique(egoIdx);
            else
                egoIdx = (1:obj.NumEgos)';
            end
            if ~isequal(displayObstacleID,obj.ObstacleIDs)
                validateattributes(displayObstacleID,{'numeric','vector'},{'integer'});
                obsIdx = obj.findIdxByID(obj.ObstacleIDs, displayObstacleID);
                obsIdx(obsIdx == 0) = [];
                obsIdx = unique(obsIdx);
            else
                obsIdx = (1:obj.NumObstacles)';
            end
            
            % Cache bodies if there have been any changes
            if obj.EgosChanged == true
                obj.cacheEgoInfo();
            end
            if obj.ObstaclesChanged == true
                obj.cacheObstacleInfo();
            end
        end
        
        function [axHandle, holdOn] = getAxes(obj, axHandle, fastUpdate)
        %getAxes Retrieve handle to active axes and check if we can preserve handles
            % Retrieve the current axes or create one. If hold
            % all/on is applied, this will not clear the axes
            % children
            if isempty(axHandle)
                holdOn = ishold;
            else
                holdOn = ishold(axHandle);
            end
            isSaved = false;
            hgSave = {axHandle};
            if fastUpdate
                if ~isempty(obj.TopLevelGroup) && isvalid(obj.TopLevelGroup)
                    hgSave = {obj.TopLevelGroup};
                    isSaved = true;
                end
            end
            axHandle = newplot(hgSave{:});
            if ~isSaved
                obj.TopLevelGroup = hggroup(axHandle);
                obj.EgoPatchObj = [];
                obj.ObstaclePatchObj = [];
            end
            
            if obj.Dimension == 3
                % Turn on light in case of 3D patch visualization
                if isempty(findobj(axHandle,'Type','Light'))
                    light(axHandle);
                end
            end
        end 
    end
    
    methods (Abstract, Access = ?nav.algs.internal.DynamicCapsuleListBase)
        %generatePatches Create or update patches based on current state of the list
        colorColliding = generatePatches(obj, axHandle, patchObj, capObjects, cachedXYZ, cachedV, timeSteps, visMode, displayedIDs, tag, numToDisplay, numStored, showAllStates, collisionFound)
        
        %colorPatches Sets the color of each object-set differently
        colorPatches(obj, multiCapGroup, numObj, timeSteps, colorBases, collisionList)
    end
    
    %% Internal Helpers
    methods (Access = protected)
        function [collisionFound, distance] = checkCollisionImpl(obj, varargin)
        %checkCollisionImpl Check for collisions between ego bodies and obstacles
        
            % Parse input struct, cache data and pre-allocate outputs
            [noOp, collisionFound, distance, fullResults, distanceOut] = obj.prepForCheck(varargin{:});
            
            if noOp == true
                return;
            else
                dim = obj.Dimension;
                
                % Retrieve position, orientation, length, and radii
                % for every obstacles over the interval being checked
                objXYZs    = reshape(obj.ObjXYZ(1:obj.NumStates*dim,:),dim,[]); % Position
                objUnitVs  = reshape(obj.ObjV(1:obj.NumStates*dim,:),dim,[]); % Orientation
                D2         = repelem(obj.ObjLength,1,obj.NumStates); % Length
                R2         = repelem(obj.ObjRadius,1,obj.NumStates); % Radii
                
                % Check for collisions between each ego body and all obstacles
                for i = 1:obj.NumEgos
                    % Retrieve position, orientation, length, and radius
                    % for the i'th ego body over the interval being checked
                    if obj.NumEgos ~= 1
                        egoXYZ = reshape(obj.EgoXYZ(1:obj.NumStates*dim,i),dim,[]); % Position
                        egoUnitV = reshape(obj.EgoV(1:obj.NumStates*dim,i),dim,[]); % Orientation
                        D1 = obj.EgoLength(i); % Length
                        R1 = obj.EgoRadius(i); % Radius
                    else
                        [xyz, v] = obj.CapHelper.applyStatesToGeometry(obj.EgoObjectsInternal(i), obj.NumStates);
                        egoXYZ = reshape(xyz,dim,[]); % Position
                        egoUnitV = reshape(v,dim,[]); % Orientation
                        D1 = obj.EgoObjectsInternal(i).Geometry.Length; % Length
                        R1 = obj.EgoObjectsInternal(i).Geometry.Radius; % Radius
                    end

                    % Check for collision between ith ego body and all obstacles
                    [collisionResult, dist] = obj.checkCollisionAllocate(distanceOut, egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false);

                    if fullResults
                        collisionFound(:,:,i) = collisionResult;
                        if distanceOut
                            distance(:,:,i) = dist;
                        end
                    else
                        collisionFound(:,i) = any(collisionResult,2);
                        if distanceOut
                            distance(~collisionFound(:,i),i) = min(dist(~collisionFound(:,i),:),[],2);
                        end
                    end
                end
            end
        end
        
        function collisionFound = checkAtIndices(obj, idx)
        %checkAtIndices Aligns the states stored in the ego objects with the idx vector
        %   The values in IDX must be positive indices between [1 obj.MaxNumSteps]
        %   The number of elements in IDX must match the number of
        %   states stored in all ego objects.
            
            % Verify that the number states stored by all ego objects match
            % the number of indices
            numStates = size(obj.EgoObjectsInternal(1).StatesInternal,1);
            coder.internal.errorIf(numStates ~= numel(idx), 'nav:navalgs:dynamiccapsulelist:MismatchedNumStateNumIdx');
            
            for i = 2:obj.NumEgos
                cond = numStates ~= size(obj.EgoObjectsInternal(i).StatesInternal,1);
                coder.internal.errorIf(cond, 'nav:navalgs:dynamiccapsulelist:NumStatePerEgo');
            end
            
            if obj.NumEgos == 1 
                obj.NumStates = size(obj.EgoObjectsInternal(1).StatesInternal,1);
            else
                if obj.EgosChanged == true
                    obj.cacheEgoInfo();
                end
            end
            
            if obj.ObstaclesChanged == true
                obj.cacheObstacleInfo();
            end
            
            % Allocate output
            collisionFound = nan(numStates,obj.NumEgos);
            
            maxStates = obj.MaxNumSteps;
            
            validIndices = idx > 0 & idx <= maxStates;
            invalidFound = 0;
            if any(~validIndices)
                invalidFound = 1;
                idxUsed = idx(validIndices);
            else
                idxUsed = idx;
            end
            
            obstacleLimit = repelem(1,1,numel(idxUsed));
            
            dim = obj.Dimension;
            IDX = (1:dim)';
            
            for i = 1:obj.NumEgos
                if obj.NumEgos ~= 1
                    egoXYZ   = reshape(obj.EgoXYZ(:,i),dim,[]);
                    egoUnitV = reshape(obj.EgoV(:,i),dim,[]);
                    D1 = obj.EgoLength(i); % Length of ego-vehicle
                    R1 = obstacleLimit*obj.EgoRadius(i); % Including this in the obstacle radii
                else
                    [xyz, v] = obj.CapHelper.applyStatesToGeometry(obj.EgoObjectsInternal(i));
                    egoXYZ = reshape(xyz,dim,[]);
                    egoUnitV = reshape(v,dim,[]);
                    D1 = obj.EgoObjectsInternal(i).Geometry.Length;
                    R1 = obstacleLimit*obj.EgoObjectsInternal(i).Geometry.Radius;
                end
            
                indices = (idxUsed(:)'-1)*dim + IDX;
                
                % Gather capsule information for obstacles
                objXYZs    = reshape(obj.ObjXYZ(indices,:),dim,[]);
                objUnitVs  = reshape(obj.ObjV(indices,:),dim,[]);
                D2         = repelem(obj.ObjLength,1,numel(idxUsed));
                R2         = kron(obj.ObjRadius,obstacleLimit);
                
                if invalidFound == 0
                    if coder.target('MATLAB')
                        collisionFound(:,i) = double(any(nav.algs.internal.mex.checkCollisionCapsule(egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false),2));
                    else
                        collisionFound(:,i) = double(any(nav.algs.internal.impl.checkCollisionCapsule(egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false),2));
                    end
                else
                    if coder.target('MATLAB')
                        collisionFound(validIndices,i) = double(any(nav.algs.internal.mex.checkCollisionCapsule(egoXYZ(:,validIndices), egoUnitV(:,validIndices), D1, R1, objXYZs, objUnitVs, D2, R2, false),2));
                    else
                        collisionFound(validIndices,i) = double(any(nav.algs.internal.impl.checkCollisionCapsule(egoXYZ(:,validIndices), egoUnitV(:,validIndices), D1, R1, objXYZs, objUnitVs, D2, R2, false),2));
                    end
                end
            end
            obj.LastQueryIndices = idx;
        end
        
        function [newObjectList, idList, numObj] = updateList(obj, numOrig, origObjects, origIDs, internalIdx, externalIdx, externalID, externalStruct, updateFlag)
            % Number of new objects
            numNew = numel(internalIdx)-nnz(internalIdx);
            numFilled = numel(origObjects);
            
            % Construct list for first time if list is empty, otherwise
            % resize list
            if numNew ~= 0
                if numOrig == 0
                    newObjectList = repmat(struct(obj.CapHelper),numNew,1);
                    idList = nan(numNew,1);
                else
                    newObjectList = [origObjects; repmat(struct(obj.CapHelper),numNew,1)];
                    idList = [origIDs; nan(numNew,1)];
                end
            else
                newObjectList = origObjects;
                idList = origIDs;
            end
            
            dim = obj.Dimension;
            
            for i = 1:numel(externalIdx)
                % Find matching object in current list
                uniqueExternal = externalIdx(i);
                if internalIdx(i) == 0
                    % Create new object with default settings
                    numFilled = numFilled+1;
                    objIdx = numFilled;
                    newObjectList(objIdx).ID = externalID(uniqueExternal);
                else
                    objIdx = internalIdx(i);
                end
                
                % Update new object properties
                if updateFlag(1)
                    % Update geometry
                    geom = externalStruct(uniqueExternal).Geometry;
                    geom.FixedTransform = obj.CapHelper.validateTransform(geom.FixedTransform);
                    newObjectList(objIdx).Geometry = geom;
                    newObjectList(objIdx).OrientationVector = normalize(newObjectList(objIdx).Geometry.FixedTransform(1:dim,1),'norm')';
                    newObjectList(objIdx).Position = newObjectList(objIdx).Geometry.FixedTransform(1:dim,end)';
                end
                if updateFlag(2)
                    % Update state
                    newObjectList(objIdx).StatesInternal = obj.CapHelper.convertToInternal(externalStruct(uniqueExternal).States);
                end
                idList(objIdx) = externalID(uniqueExternal);
            end
            
            % New number of objects
            numObj = numel(idList);
        end
        
        function cacheObstacleInfo(obj)
        %cacheObstacleInfo Stores data for fast access during collision-checking
            dim = obj.Dimension;
            numObj = numel(obj.CollisionObjectsInternal);
            obj.NumObstacles = numObj;
            obj.ObstacleIDs = zeros(numObj,1);
            
            obj.ObjLength = zeros(1, numObj);
            obj.ObjRadius = zeros(1, numObj);
            obj.ObjV      = zeros(obj.MaxNumSteps*dim, numObj);
            obj.ObjXYZ    = zeros(obj.MaxNumSteps*dim, numObj);
            
            for i = 1:numObj
                [obj.ObjXYZ(:,i), obj.ObjV(:,i)] = obj.CapHelper.applyStatesToGeometry(obj.CollisionObjectsInternal(i), obj.MaxNumSteps);
                obj.ObjLength(i) = obj.CollisionObjectsInternal(i).Geometry.Length;
                obj.ObjRadius(i) = obj.CollisionObjectsInternal(i).Geometry.Radius;
                obj.ObstacleIDs(i) = obj.CollisionObjectsInternal(i).ID;
            end
            obj.ObstaclesChanged = 0;
        end
        
        function cacheEgoInfo(obj)
        %cacheObstacleInfo Stores data for fast access during collision-checking
            dim = obj.Dimension;
            numObj = numel(obj.EgoObjectsInternal);
            obj.NumEgos = numObj;
            obj.EgoIDs = zeros(numObj,1);
            numStates = 0;
            
            % Find and store maximum number of states
            for i = 1:numObj
                numStates = max(numStates, size(obj.EgoObjectsInternal(i).StatesInternal,1));
            end
            obj.NumStates = numStates;
            
            obj.EgoLength = zeros(1, numObj);
            obj.EgoRadius = zeros(1, numObj);
            obj.EgoV      = zeros(numStates*dim, numObj);
            obj.EgoXYZ    = zeros(numStates*dim, numObj);
            
            for i = 1:numObj
                [obj.EgoXYZ(:,i), obj.EgoV(:,i)] = obj.CapHelper.applyStatesToGeometry(obj.EgoObjectsInternal(i), numStates);
                obj.EgoLength(i) = obj.EgoObjectsInternal(i).Geometry.Length;
                obj.EgoRadius(i) = obj.EgoObjectsInternal(i).Geometry.Radius;
                obj.EgoIDs(i) = obj.EgoObjectsInternal(i).ID;
            end
            obj.EgosChanged = 0;
        end
    end
    
    methods (Access = ?nav.algs.internal.InternalAccess)
        function status = addEgoImpl(obj, varargin)
        %addEgoImpl Add ego bodies to the list
            % Validate inputs and retrieve indices of matching objects
            [status, egoIDs, egoStruct, uniqueInternalIdx, uniqueExternalIdx] = obj.validateAdd('addEgo', 'egoStruct', 'egoStruct.ID', obj.EgoIDs, varargin{:});
            
            if ~isempty(status)
                % Allocate memory for new set of obstacles
                [obj.EgoObjectsInternal, obj.EgoIDs, obj.NumEgos] = obj.updateList(obj.NumEgos, obj.EgoObjectsInternal, obj.EgoIDs, uniqueInternalIdx, uniqueExternalIdx, egoIDs, egoStruct, [1 1]);

                % Mark visual/cached data as 'dirty'
                obj.EgosChanged = 1;
                obj.egoVisChanged = 1;
            end
        end

        function status = addObstacleImpl(obj, varargin)
        %addObstacleImpl Add obstacles to the list
            % Validate inputs and retrieve indices of matching objects
            [status, obstacleIDs, obstacleStruct, uniqueInternalIdx, uniqueExternalIdx] = obj.validateAdd('addObstacle', 'obstacleStruct', 'obstacleStruct.ID', obj.ObstacleIDs, varargin{:});
            
            if ~isempty(status)
                % Allocate memory for new set of obstacles
                [obj.CollisionObjectsInternal, obj.ObstacleIDs, obj.NumObstacles] = obj.updateList(obj.NumObstacles, obj.CollisionObjectsInternal, obj.ObstacleIDs, uniqueInternalIdx, uniqueExternalIdx, obstacleIDs, obstacleStruct, [1 1]);

                % Mark visual/cached data as 'dirty'
                obj.ObstaclesChanged = 1;
                obj.obstacleVisChanged = 1;
            end
        end
        
        function cObj = copyInternal(obj, cObj)
        %copyInternal Transfers internal data to other copied object
            cObj.MaxNumSteps = obj.MaxNumSteps;
            cObj.EgoObjectsInternal = obj.EgoObjectsInternal;
            cObj.CollisionObjectsInternal = obj.CollisionObjectsInternal;
        end
        
        function collisionFound = checkCollisionAt(obj, egoIDs, egoPoseStruct, indices)
        %checkCollisionAt Checks for collisions between index-associated ego states and environment
        %
        %   COLLISIONFOUND = checkCollision(OBJ, EGOIDS, EGOPOSESTRUCT, INDICES) Exhaustively 
        %   checks for collisions between a set of ego bodies defined by
        %   EGOIDS, at corresponding states, EGOPOSESTRUCT. The states in
        %   EGOPOSESTRUCT are compared against the state of the obstacles
        %   found at the corresponding INDICES. The number of EGOIDS and
        %   INDICES must match the number of states for each ego body in 
        %   EGOPOSESTRUCT.
        
            narginchk(4,4);
            
            % Update internal state
            obj.updateEgoPose(egoIDs, egoPoseStruct);

            % Convert to indices and check for collisions
            collisionFound = obj.checkAtIndices(indices);
        end
        
        function [noOp, results, distance, fullResults, distanceOut] = prepForCheck(obj, options)
        %prepForCheck Parses input to checkCollision and prepares internal state/outputs
            noOp = false;

            if nargin == 1
                % No options provided
                fullResults = false;
                distanceOut = false;
            else
                % Default assumption is that options is struct
                fullResults = options.FullResults;
                distanceOut = options.ReturnDistance;
            end

            % Cache obstacle info if change has occurred, this will
            % improve performance for repeated calls to checkCollision
            % with only changes to ego bodies
            if obj.ObstaclesChanged == true
                obj.cacheObstacleInfo();
            end

            % Find range of times to check
            if obj.NumEgos == 1
                obj.NumStates = size(obj.EgoObjectsInternal(1).StatesInternal,1);
            else
                if obj.EgosChanged == true
                    obj.cacheEgoInfo();
                end
            end
            obj.NumStates = min(obj.NumStates, obj.MaxNumSteps);

            % Return empty if there are no ego objects
            if obj.NumEgos == 0
                results = false(0);
                distance = [];
                noOp = true;
                return;
            else
                % Allocate collision output
                if fullResults && obj.NumObstacles ~= 0
                    results = false(obj.NumStates,obj.NumObstacles,obj.NumEgos);
                else
                    results = false(obj.NumStates,obj.NumEgos);
                end
            end

            % Allocate distance output
            if distanceOut
                % Distance will have the same dimensions/defaults as
                % collisionFound
                distance = nan(size(results));
            else
                distance = [];
            end

            if obj.NumObstacles == 0
                % If no obstacles exist, all states are valid
                results(:) = false;
                noOp = true;
            end
        end
        
        % Local allocation helper
        function [collisionResult, dist] = checkCollisionAllocate(~, distanceOut, egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false)
            if coder.target('MATLAB')
                if distanceOut
                    [collisionResult, dist] = nav.algs.internal.mex.checkCollisionCapsule(...
                        egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false);
                else
                    dist = [];
                    collisionResult = nav.algs.internal.mex.checkCollisionCapsule(...
                        egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false);
                end
            else
                if distanceOut
                    [collisionResult, dist] = nav.algs.internal.impl.checkCollisionCapsule(...
                        egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false);
                else
                    dist = [];
                    collisionResult = nav.algs.internal.impl.checkCollisionCapsule(...
                        egoXYZ, egoUnitV, D1, R1, objXYZs, objUnitVs, D2, R2, false);
                end
            end
        end
    end
end
