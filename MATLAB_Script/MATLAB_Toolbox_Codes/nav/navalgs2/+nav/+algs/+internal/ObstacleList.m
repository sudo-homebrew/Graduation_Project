classdef (Abstract) ObstacleList < handle & nav.algs.internal.InternalAccess & robotics.core.internal.InternalAccess
%OBSTACLELIST Contains a list of obstacles that can be used for collision-checking
%
%   OBSTACLELIST serves as a base class for classes that contain
%   collision-checking obstacles of different types. Obstacles can be
%   added, removed, and updated before determining whether a collision
%   occurs between the EgoObject and the list of stored obstacles over a
%   set of input poses

%   Copyright 2019 The MathWorks, Inc.

    properties
        %EgoObject Obstacle object representing the ego-body
        %
        %   Contains the type, size, initial location, and other meta
        %   information that defines the body of the ego-object
        EgoObject
    end
    
    properties (SetAccess = protected)
        %NumObstacles Number of obstacles in list
        NumObstacles
    end
       
    properties (Access = ?nav.algs.internal.InternalAccess)
        %CollisionObjectsInternal
        CollisionObjectsInternal
    end
    
    methods
        function obj = ObstacleList(egoObject, listOfObstacles)

            % Store obstacleList
            obj.CollisionObjects = listOfObstacles;
            obj.CollisionObjectsInternal = listOfObstacles;
            
            % Store egoObject
            obj.EgoObject = egoObject;
            
            % Update number of obstacles
            obj.NumObstacles = numel(obj.CollisionObjectsInternal);
        end
        
        function set.EgoObject(obj, newEgoObject)
        %set.EgoObject Sets the EgoObject property and performs input validation and formatting.
            obj.EgoObject = obj.validateEgoObject(newEgoObject);
        end
    end
    
    methods (Abstract)
        %checkCollision Checks whether the EgoObject collides with any obstacles at specified poses
        %   Transforms EgoObject using the N-by-M state-array, poses,
        %   and checks whether it is in collision with any of the obstacles
        %   in CollisionObjects. For each state, checkCollision returns true 
        %   if there is an intersection between ego and obstacleList, and
        %   false otherwise.
        collisionDetected = checkCollision(obj, poses)
        
        %addObstacle Appends a set of obstacles to the current CollisionObjects list
        %   Uses obstacleConstructionInfo to create new entries in
        %   CollisionObjects and adds them to the end of the list.
        addObstacle(obj, obstacleContructionInfo)
        
        %removeObstacle Removes a set of obstacles to the current CollisionObjects list
        %   Takes in an N-element list of indices, dictating the obstacles in
        %   CollisionObjects that are to be removed
        removeObstacle(obj, indices)
        
        %updateObstaclePose Updates the pose for one or more obstacles in CollisionObjects
        %   If one input is provided, state must be an N-by-M state-vector, 
        %   where N is equal to the number of obstacles in the ObstacleList.
        %
        %   If provided, the second input corresponds to an N-element vector 
        %   of indices that determines the obstacles updated by the corresponding
        %   state-vector.
        updateObstaclePose(obj, state, obstacleIndex)
        
        %updateObstacleGeometry Updates the geometric properties of one or more obstacles in CollisionObjects
        %   If one input is provided, geometryInfo must be an N-element cell-array,
        %   where N is equal to the number of obstacles in the
        %   ObstacleList, and the contents of each cell are the geometric
        %   properties needed to update the corresponding CollisionObject(s)
        %
        %   If provided, the second input corresponds to an N-element vector 
        %   of indices that determines the obstacles updated by the corresponding
        %   geometryInfo cell-array.
        updateObstacleGeometry(obj, geometryInfo, obstacleIndex)
        
        %copy Makes a deep copy of the object
        copy(obj)
    end
    
    methods (Abstract, Access = protected)
        %validateEgoObject Validates a new EgoObject given to the EgoObject setter
        %
        %   EGOOBJECT = validateEgoObject(OBJ, NEWEGOOBJECT) A method used
        %   to validate and format any object provided to the
        %   obstacleList's EgoObject property. If no validation or formatting
        %   is required, users can choose to have this method simply set
        %   EGOOBJECT equal to NEWEGOOBJECT.
        egoObject = validateEgoObject(obj, newEgoObject)
    end
end
