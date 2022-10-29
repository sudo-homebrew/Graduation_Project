classdef (Abstract, Hidden) DynamicObstacleList < nav.algs.internal.InternalAccess
%DynamicObstacleList A data structure used to represent a collision checking environment over some short time horizon
%
%   This class serves as a primitive-agnostic baseclass for obstacle-lists.
%   It contains helper methods that are common across dynamic obstacle lists,
%   and defines the interface that derived classes must implement. Concrete
%   examples of DynamicObstacleList are dynamicCapsuleList, dynamicCapsuleList3D.
%
%   DynamicObstacleList properties:
%      EgoIDs                           - Integers that identify the ego bodies
%      MaxNumSteps                      - Maximum number of steps
%      NumObstacles                     - Number of obstacles in list
%      NumEgos                          - Number of ego bodies in list
%      ObstacleIDs                      - Integers that identify the obstacles
%
%   DynamicObstacleList methods:
%      addEgo                           - Add ego bodies to the list
%      addObstacle                      - Add obstacles to the list
%      checkCollision                   - Check for collisions between ego bodies and obstacles
%      copy                             - Returns a deep copy of the obstacle list
%      egoGeometry                      - Geometric properties of ego bodies
%      egoPose                          - Poses of ego bodies
%      obstacleGeometry                 - Geometric properties of obstacles
%      obstaclePose                     - Poses of obstacles
%      removeEgo                        - Remove ego bodies from the list
%      removeObstacle                   - Remove obstacles from the list
%      show                             - Display ego bodies and obstacles in environment
%      updateEgoGeometry                - Update geometric properties of ego bodies
%      updateEgoPose                    - Update ego-body poses
%      updateObstacleGeometry           - Update geometric properties of obstacles
%      updateObstaclePose               - Update obstacle poses
%
%   See also occupancyMap, binaryOccupancyMap.

%   Copyright 2020 The MathWorks, Inc.
    
%#codegen
    
    properties
    %MaxNumSteps Maximum number of steps
        MaxNumSteps = 31;
    end
    
    properties (SetAccess = protected)
        %EgoIDs IDs for ego bodies in environment
        EgoIDs
        
        %ObstacleIDs IDs for obstacles in environment
        ObstacleIDs
        
        %NumObstacles Number of obstacles in list
        NumObstacles
        
        %NumEgos Number of ego bodies in list
        NumEgos
    end
       
    properties (Access = ?nav.algs.internal.InternalAccess)
        %CollisionObjectsInternal
        CollisionObjectsInternal
        
        %EgoObjectsInternal Obstacle object representing the ego-body
        %
        %   Contains the type, size, initial location, and other meta
        %   information that defines the body of the ego-object
        EgoObjectsInternal
    end
    
    properties (Abstract, Access = protected)
        %DefaultGeometry A var-sizeable struct-array for storing geometric properties of collision objects
        %
        %   The derived class is responsible for defining a
        %   codegen-compatible struct that can be dynamically resized. This
        %   will serve as a concrete 'type' for representing sets of
        %   collision bodies in a way that Coder can accept.
        DefaultGeometry
    end
    
    properties (Access = protected)
        %ObstaclesChanged Dictates whether the Obstacle set is dirty
        ObstaclesChanged = 1
        
        %EgosChanged Dictates whether the Ego set is dirty
        EgosChanged = 1
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.InternalAccessNoHandle})
        %DefaultID A var-sizeable char-array for storing object names
        DefaultID = nan;
        
        %DefaultState A var-sizeable struct-array for storing dynamic states of collision objects
        DefaultState
        
        %PrimFields Fields found in top-level struct
        PrimFields
        
        %StateFields Fields found in top-level struct
        StateFields
        
        %GeomFields Fields found in top-level struct
        GeomFields
    end
    
    methods
        function obj = DynamicObstacleList(defaultState, egoObjects, obstacles)
        %DynamicObstacleList Construct dynamicCapsuleList
            egos = egoObjects;
            obs  = obstacles;
            coder.varsize('obs',[inf 1],[1 0]);
            coder.varsize('egos',[inf 1],[1 0]);
            
            % Store obstacleList
            obj.CollisionObjectsInternal = obs;
            
            % Store EgoObjectsInternal
            obj.EgoObjectsInternal = egos;
            
            % Update number of objects being managed
            obj.NumObstacles = numel(obj.CollisionObjectsInternal);
            obj.NumEgos = numel(obj.EgoObjectsInternal);
            
            state = defaultState;
            coder.varsize('state',[inf size(state,2)],[1 0]);
            obj.DefaultState = struct('States',defaultState);
        end
    end
    
    methods 
    % Getter/Setter methods
        function set.MaxNumSteps(obj, step)
        %set.MaxNumSteps
            validateattributes(step,{'numeric'},{'scalar','integer','positive'},'SetMaxStep','step');
            obj.MaxNumSteps = step;
            obj.setDirtyFlags();
        end
    end
    
    methods (Abstract)
        %addEgo Add ego bodies to the list
        %   Uses constructionInfo to create or update entries in
        %   EgoObjectsInternal. If an ego body with matching name already 
        %   exists in the list, it is updated with properties stored in the
        %   incoming constructionInfo object.
        addEgo(obj, constructionInfo)
        
        %addObstacle Add obstacles to the list
        %   Uses constructionInfo to create or update entries in
        %   CollisionObjectsInternal. If an obstacle with matching name already 
        %   exists in the list, it is updated with properties stored in the
        %   incoming constructionInfo object.
        addObstacle(obj, constructionInfo)
        
        %checkCollision Check for collisions between ego bodies and obstacles
        collisionDetected = checkCollision(obj, options)
        
        %copy Returns a deep copy of the obstacle list
        copy(obj)
        
        %egoGeometry Geometric properties of ego bodies
        %   Takes an array of objectIDs and returns a struct-array with fields
        %   corresponding to the geometric attributes of the underlying 
        %   collisionGeometry type.
        egoGeometry(obj, idInput)
        
        %egoPose Poses of ego bodies
        %   Takes an array of objectIDs and returns a struct-array with ID 
        %   and States fields.
        egoPose(obj, idInput)
        
        %obstacleGeometry Geometric properties of obstacles
        %   Takes an array of objectIDs and returns a struct-array with fields
        %   corresponding to the geometric attributes of the underlying 
        %   collisionGeometry type. 
        obstacleGeometry(obj, idInput)
        
        %egoPose Poses of obstacles
        %   Takes an array of objectIDs and returns a struct-array with 'ID'
        %   and 'States' fields.
        obstaclePose(obj, idInput)
        
        %removeEgo Remove ego bodies from the list
        %   Takes an N-element cell-array of char-arrays, and removes any
        %   ego bodies with matching identifiers from the list.
        removeEgo(obj, objectID)
        
        %removeObstacle Remove obstacles from the list
        %   Takes an N-element cell-array of char-arrays, and removes any
        %   obstacles with matching identifiers from the list.
        removeObstacle(obj, objectID)
        
        %updateEgoGeometry Update the geometric properties of ego bodies
        updateEgoGeometry(obj, geometryInfo)
        
        %updateEgoPose Update the pose of ego bodies
        updateEgoPose(obj, poseInfo)
        
        %updateObstacleGeometry Update the geometric properties of obstacles
        updateObstacleGeometry(obj, geometryInfo)
        
        %updateObstaclePose Update the pose of obstacles
        updateObstaclePose(obj, poseStruct)
    end
    
    methods (Access = protected)
        function setDirtyFlags(obj)
        %setDirtyFlags Sets the EgosChanged/ObstaclesChanged flags to dirty (g2187568)
            obj.ObstaclesChanged = 1;
            obj.EgosChanged = 1;
        end
        
        function [status, externalID, externalInfo, uniqueInternalIdx, uniqueExternalIdx] = validateAdd(obj, fcnName, inputName, idName, internalID, externalInfo)
        %validateAdd Verifies that inputs to add___ are of the correct format
            narginchk(5,6);
            if nargin == 5 || isempty(externalInfo)
                % No inputs provided, return empty status
                status = zeros(0,1);
                externalID = [];
                externalInfo = [];
                uniqueExternalIdx = [];
                uniqueInternalIdx = [];
            else
                obj.validateStruct(obj.PrimFields, externalInfo, fcnName, inputName)
                % Verify that input is a vector of integer-valued numeric IDs
                externalID = zeros(numel(externalInfo),1);
                for i = 1:numel(externalInfo)
                    externalID(i) = externalInfo(i).ID;
                end
                validateattributes(externalID,{'numeric'},{'integer'}, fcnName, idName);
                [status, uniqueExternalIdx, uniqueInternalIdx] = obj.addUpdateStatus(internalID, externalID);
            end
        end
        
        function [status, uniqueInternalIdx] = validateRemove(obj, fcnName, inputName, internalID, externalID)
        %validateInputs Validates the inputs and find matching indices for add, update, and remove methods
            if nargin == 4 || isempty(externalID)
                % No inputs provided, return empty status
                status = zeros(0,1);
                uniqueInternalIdx = [];
            else
                % Verify that input is a vector of integer-valued numeric IDs
                validateattributes(externalID,{'numeric'},{'integer'}, fcnName, inputName, 1);
                
                % Find unique identifiers and determine status of operation
                [status, uniqueInternalIdx] = obj.removeStatus(internalID, externalID);
            end
        end
        
        function [status, externalID, externalInfo, uniqueInternalIdx, uniqueExternalIdx] = validateUpdate(obj, fcnName, idName, structName, requiredFields, internalID, externalID, externalInfo)
        %validateUpdate Verifies that inputs to update___ are of the correct format
            if nargin == 6
                % No inputs provided, return empty status
                status = zeros(0,1);
                externalID = [];
                externalInfo = [];
                uniqueExternalIdx = [];
                uniqueInternalIdx = [];
            else
                % Verify that IDs are correct
                validateattributes(externalID,{'numeric'}, {'integer'}, fcnName, idName);
                if nargin == 7
                    % Not enough inputs
                    narginchk(8,8);
                end
                if numel(externalID)~=numel(externalInfo)
                    coder.internal.error('nav:navalgs:dynamiccapsulelist:InputSizeMismatch', numel(externalID), numel(externalInfo));
                end
                if ~isempty(externalID)
                    obj.validateStruct(requiredFields, externalInfo, fcnName, structName);
                end
                
                % Find unique identifiers and determine status of operation
                [status, uniqueExternalIdx, uniqueInternalIdx] = obj.addUpdateStatus(internalID, externalID);
            end
        end
        
        function [status, externalID, internalIndices] = validateIntrospectionInputs(obj, fcnName, inputName, internalID, externalID)
        %validateIntrospectionInputs Validates introspection inputs
            if nargin == 4
                % If no input is provided, then introspection outputs
                % information on all stored objects. All objects are found
                % and unique
                status = ones(numel(internalID),1);
                internalIndices = (1:numel(internalID))';
                externalID = internalID;
            else
                % Otherwise we validate the inputs and return a status
                % flag.
                validateattributes(externalID,{'numeric'},{'integer'}, fcnName, inputName);

                % Find unique identifiers and determine status of operation
                [status, internalIndices] = obj.removeStatus(internalID, externalID);
            end
        end
        
        function [status, uniqueExternalIdx, uniqueInternalIdx] = addUpdateStatus(obj, internalID, externalID)
        %addUpdateStatus determines whether the input:
        %    1   - added a new object
        %    0   - updated an existing object
        %   -1   - was an early duplicate, overwritten by later input
            
            % Get last index for each unique entry
            [~, uniqueExternalIdx] = unique(externalID,'last');
            
            % Search for input indices in the list
           	internalIdx = obj.findIdxByID(internalID, externalID);
            
            % Assume that all entries will be updates to existing objects
            mask = false(numel(externalID),1);
            mask(uniqueExternalIdx) = true;
            status = zeros(numel(externalID),1);
            
            % Mark duplicates
            status(~mask) = -1;
            
            % If an entry is unique and not in the list, mark it as added
            status(internalIdx==0 & mask) = 1;
            
            % Return unique internal indices
            uniqueInternalIdx = internalIdx(uniqueExternalIdx);
        end
        
        function [status, indices] = removeStatus(obj, internalID, externalID)
        %removeStatus determines whether the input ID:
        %    1   - successfully removed an object
        %    0   - was not found in the list
        %   -1   - was a duplicate of an earlier input and ignored
        
            % Assume all entries are duplicates
            status = -ones(numel(externalID),1);
            
            % Find index of objects with matching ID
            indices = obj.findIdxByID(internalID, externalID);
            
            % Find unique indices
            [~, uniqueIDIdx] = unique(indices,'stable');
            
            % The first instance of a unique ID is the entry that causes removal
            status(uniqueIDIdx) = 1;
            
            % Mark entries that were not found
            status(indices==0) = 0;
        end
        
        function indices = findIdxByID(~, internalID, externalID)
        %findIdxByID Return the indices corresponding to named bodies
            
            indices = zeros(numel(externalID),1);
            
            % Retrieve IDs for ego objects
            for n = 1:numel(externalID)
                idx = find(externalID(n) == internalID,1);
                if ~isempty(idx)
                    indices(n) = idx;
                end
            end
        end
        
        function curPoseStruct = getPoses(obj, egoFlag, internalIndices, externalIDs)
        %egoPoses Returns stored poses for set of ego bodies or obstacles
            
            % Create struct-array for state information of specified
            % objects
            curPoseStruct = repmat(obj.DefaultState,numel(externalIDs),1);
            
            if egoFlag == true
                objSet = obj.EgoObjectsInternal;
            else
                objSet = obj.CollisionObjectsInternal;
            end
            
            for i = 1:numel(externalIDs)
                if internalIndices(i) ~= 0
                    curPoseStruct(i).States = objSet(internalIndices(i)).StatesInternal;
                end
            end
        end
        
        function geomStruct = getGeometry(obj, externalIDs)
        %getGeometry Returns default geometry structure for a set of IDs
            
            % Create struct used to represent each object's geometry
            geomStruct = repmat(struct('Geometry', obj.DefaultGeometry),numel(externalIDs),1);
        end
    end
    
    methods (Static, Hidden)
        function validateStruct(refFields, inputStruct, fcnName, inputName)
        %validateStruct Performs lightweight validation of inputs
            % Confirm that input is of the correct type
            validateattributes(inputStruct,{'struct'},{'nonempty'},fcnName,inputName);
            
            % Only validate struct fields in MATLAB, as Coder will only
            % generate code if the types are correct
            if coder.target('MATLAB')
                fieldNotFound = ~isfield(inputStruct,refFields);
                if any(fieldNotFound)
                    if isfield(inputStruct,'Geometry')
                        fieldNotFound = ~isfield(inputStruct(1).Geometry,refFields);
                    end
                    if any(fieldNotFound)
                        missingFields = string(refFields(fieldNotFound));
                        coder.internal.error('nav:navalgs:dynamiccapsulelist:MissingFields',inputName,strjoin(missingFields,','));
                    end
                end
            end
        end
    end
end
