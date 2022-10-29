classdef dynamicCapsuleList < nav.algs.internal.DynamicCapsuleListBase
%dynamicCapsuleList Dynamic capsule-based obstacle list
%   The dynamicCapsuleList object manages two lists of collision primitive
%   objects, ego bodies and obstacles. Each object in the list is defined by
%   a unique identifier, geometry, and set of SE2 states. The list assumes
%   that states are separated by a fixed interval of time, where the N-th
%   state in the list corresponds to the time T = (N-1)*dT. One or more ego
%   bodies or obstacles are represented as structs-arrays, where each element
%   contains the fields:
%
%       ID       - Integer that identifies the object
%
%       States   - Nx3 array of SE2 states
%
%       Geometry - Structure to define the capsule geometry with the fields:
%
%                     Length          - Length of the capsule cylinder, in meters (Default: 2)
%                     Radius          - Radius of the capsule, in meters (Default: 1)
%                     FixedTransform  - A 3x3 transform relative to capsule's local frame (Default: eye(3))
%
%   Dynamically add, remove, and update the geometry and future poses of
%   ego bodies and obstacles in the environment. Validate stored trajectories
%   by calling the checkCollision function which checks collisions with obstacles
%   at each time step, and show the object to visualize the results.
%
%   Multiple objects can be updated at once by providing an N-element vector 
%   of IDs alongside an N-element struct-array of either STATES or
%   GEOMETRY, as defined above.
%
%   Note: If the number of poses for a specific obstacle is less than the
%         MaxNumSteps property, the final state is assumed to be constant.
%
%   dynamicCapsuleList properties:
%      EgoIDs                           - Integers that identify the ego bodies
%      MaxNumSteps                      - Maximum number of steps
%      NumObstacles                     - Number of obstacles in list
%      NumEgos                          - Number of ego bodies in list
%      ObstacleIDs                      - Integers that identify the obstacles
%
%   dynamicCapsuleList methods:
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
%   Example:
%
%       % Create a dynamicCapsuleList object.
%       oList = dynamicCapsuleList;
%
%       % Specify an ID for the first ego body.
%       egoID1 = 1;
%
%       % Generate a capsule geometry structure. Capsule geometries are 
%       % defined by a length, radius, and fixed-transform field.
%       geom = struct('Length', 4, 'Radius', 2, 'FixedTransform', eye(3));
%       
%       % Create a set of states based on an equation of motion for the ego
%       % body. Generate a series of time steps, and calculate the state
%       % based on the given equation.
%       timeStamps = linspace(0,1,oList.MaxNumSteps)';
%       states = timeStamps.*[100 100 pi/4];
%
%       % Generate a fully defined capsule structure with ID, states, and geometry.
%       egoCap1 = struct('ID', egoID1, 'States', states, 'Geometry', geom);
%
%       % Add the capsule to the list.
%       addEgo(oList, egoCap1);
%       
%       % Show the environment.
%       show(oList);
%       axis equal;
%       
%       % Show the environment across all time steps.
%       show(oList, 'TimeStep', 1:oList.MaxNumSteps);
%       axis equal;
%
%       % Generate a default geometry structure for an obstacle using a new ID.
%       newID = 10;
%       [newID, obstacleGeom] = obstacleGeometry(oList, newID);
%
%       % Customize the obstacle's geometry.
%       obstacleGeom.Geometry.Length = 10;
%       obstacleGeom.Geometry.Radius = 3;
%       t = [1; 0];
%       R = [eul2rotm([pi/4 0 0])];
%       obstacleGeom.Geometry.FixedTransform(1:2,:) = [R(1:2,1:2) t];
%
%       % Update the list to add this obstacle as a stationary object.
%       updateObstacleGeometry(oList, newID, obstacleGeom);
%
%       % Check for collisions to see if the ego body collides with the obstacle.
%       results = checkCollision(oList)
%
%       % Visualize the results by displaying the scene with 'ShowCollision'.
%       show(oList, 'TimeStep', 1:oList.MaxNumSteps, 'ShowCollision', true);
%       axis equal;
%
%       % Generate a state structure array for two obstacles: the previously
%       % added obstacle and another with a new ID.
%       newID2 = 21;
%       [IDs, obstaclePoseStruct] = obstaclePose(oList, [newID; newID2]);
%
%       % Use the ego body states as a start for obstacle 2.
%       obstaclePoseStruct(2).States = egoCap1.States;
%
%       % Update the states such that the obstacle and ego cross paths.
%       obstaclePoseStruct(2).States(:,[1 3]) = [100 pi]-obstaclePoseStruct(2).States(:,[1 3]);
%
%       % Move the original obstacle to a new location.
%       obstaclePoseStruct(1).States = [3 3 pi/3];
%
%       % Use the ID and struct-array to update the old obstacle and add a
%       % new obstacle to the list.
%       updateObstaclePose(oList, IDs, obstaclePoseStruct);
%
%       % Check for collisions to see where the objects intersect.
%       results = checkCollision(oList)
%
%       % Verify the results by displaying the updated environment.
%       ax = show(oList, 'TimeStep', 1:oList.MaxNumSteps, 'ShowCollision', true);
%       axis equal;
%       ax.XLimMode = "manual";
%       ax.YLimMode = "manual";
%       % Loop over the scene to see the collisions:
%       for i = 1:oList.MaxNumSteps
%           show(oList, 'Parent', ax, 'TimeStep', i, 'ShowCollision', true, 'FastUpdate', true);
%           drawnow;
%       end
%
%   See also dynamicCapsuleList3D.

%   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Hidden, Access = protected)
        %CapHelper Class storing static helper methods
        CapHelper
    end
    
    properties (Constant, Access = ?nav.algs.internal.DynamicCapsuleListBase)
        %Dimension
        Dimension = 2;
    end
    
    methods
        function status = addEgo(obj, varargin)
        %addEgo Add ego bodies to the list
        %   addEgo(OBJ, EGOSTRUCT) add one or more ego bodies to the list
        %   with specified ID, state, and geometry values. EGOSTRUCT is an
        %   N-element struct-array, where each element contains:
        %
        %       ID       - Integer that identifies the object
        %
        %       States   - Mx3 array of SE2 states
        %
        %       Geometry - Structure with the fields:
        %
        %                     Length          - Length of the capsule cylinder, in meters (Default: 2)
        %                     Radius          - Radius of the capsule, in meters (Default: 1)
        %                     FixedTransform  - A 3x3 transform relative to capsule's local frame (Default: eye(3))
        %
        %   STATUS = addEgo(OBJ, EGOSTRUCT) returns an optional output, STATUS,
        %   an N-element vector with values of 1 (body added), 0 (body updated), 
        %   -1 (duplicate input).
            
            narginchk(1,2)
            
            % Add or update ego bodies
            s = obj.addEgoImpl(varargin{:});
            if nargout == 1
                status = s;
            else
                status = obj;
            end
        end

        function status = addObstacle(obj, varargin)
        %addEgo Add obstacles to the list
        %   addObstacle(OBJ, OBSTACLESTRUCT) add one or more obstacles to the list
        %   with specified ID, state, and geometry values. OBSTACLESTRUCT is an
        %   N-element struct-array, where each element contains:
        %
        %       ID       - Integer that identifies the object
        %
        %       States   - Mx3 array of SE2 states
        %
        %       Geometry - Structure with the fields:
        %
        %                     Length          - Length of the capsule cylinder, in meters (Default: 2)
        %                     Radius          - Radius of the capsule, in meters (Default: 1)
        %                     FixedTransform  - A 3x3 transform relative to capsule's local frame (Default: eye(3))
        %
        %   STATUS = addObstacle(OBJ, OBSTACLESTRUCT) returns an optional 
        %   output, STATUS, an N-element vector with values of 1 (body added),
        %   0 (body updated), -1 (duplicate input).
        
            narginchk(1,2);
            
            % Add or update obstacles
            s = obj.addObstacleImpl(varargin{:});
            if nargout == 1
                status = s;
            else
                status = obj;
            end
        end
        
        function [collisionFound, distance] = checkCollision(obj, varargin)
        %checkCollision Check for collisions between ego bodies and obstacles
        %
        %   COLLISIONFOUND = checkCollision(OBJ) checks each ego body for 
        %   collisions with obstacles in the environment. The function 
        %   iterates through each ego body and each time step and returns
        %   an N-by-E matrix of logical values indicating collision. E is
        %   the number of ego bodies, and N is either obj.MaxNumSteps or the
        %   maximum number of states for any ego body, whichever is less.
        %   Ego bodies and obstacles are assumed to remain in their last
        %   specified state if their state is not defined for all N.
        %
        %   [RESULTS, DISTANCE] = checkCollision(OBJ, OPTIONS) check
        %   each ego body for collisions with obstacles in the environment.
        %   Specify additional options as a structure with the fields:
        %
        %       FullResults    - Boolean indicating whether to return the 
        %                        collision results for each obstacle separately.
        %                        If set to true, RESULTS is an NxOxE matrix
        %                        array, where O is the number of obstacles,
        %                        and E is the number of ego bodies. (Default: false)
        %
        %       ReturnDistance - Boolean indicating whether to return the
        %                        distance calculation. If set to true,
        %                        DISTANCE returns the distance between
        %                        each ego body and the closest obstacle (NxE matrix). 
        %
        %                        If OPTIONS.FullResults is true, the distance
        %                        to each obstacle is returned (NxOxE matrix array). (Default: false).
        %
        %   Example:
        %       % Plot helpers
        %       f = @(g,h,ax)legend(ax);
        %       g = @(ax)set(ax,'DataAspectRatio',[1 1 1]);
        %       h = @(ax,str)title(ax,str);
        %       fUpdateFigure = @(ax,str)f(g(ax),h(ax,str),ax);
        %
        %       % Create empty list.
        %       oList = dynamicCapsuleList
        %
        %       % Add two ego bodies to the list.
        %       [egoID, egoGeom] = egoGeometry(oList, [1;2])
        %       updateEgoGeometry(oList, egoID, egoGeom)
        % 
        %       % Add two obstacles with default geometry and fully defined
        %       % trajectories.
        %       [obstacleID, obsStates] = obstaclePose(oList, [1;2])
        %       obsStates(1).States = rand(31,3)+linspace(0,1,31)'*[ 10 0 pi]+[0 10 0];
        %       obsStates(2).States = rand(31,3)+linspace(0,1,31)'*[-10 5 pi]+[10 0 0];
        %       updateObstaclePose(oList, obstacleID, obsStates)
        % 
        %       % Update the poses for the two ego bodies so that one
        %       % travels for 10 time steps, and the other travels for 20.
        %       [egoID, egoPoseStruct] = egoPose(oList);
        %       egoPoseStruct(1).States = linspace(0,1,10)'*[10 0 0];
        %       egoPoseStruct(2).States = linspace(0,1,20)'*[10 10 0];
        %       updateEgoPose(oList, egoID, egoPoseStruct)
        %
        %       % Show the environment at the current time step.
        %       ax = subplot(1,3,1);
        %       show(oList,'Parent',ax,'TimeStep',1,'ShowCollision',1);
        %       fUpdateFigure(ax, 'Environment at TimeStep 1');
        %
        %       % Check for collisions
        %       results = checkCollision(oList)
        %
        %       % Verify the results by displaying the predicted environment.
        %       ax = subplot(1,3,2);
        %       show(oList,'Parent',ax,'TimeStep',1:oList.MaxNumSteps,'ShowCollision',1);
        %       fUpdateFigure(ax, sprintf('Environment at TimeSteps = 1:20\nEgo1: 10 step trajectory\nEgo2: 20 step trajectory'));
        %
        %       % Give the first ego body a 20 step trajectory
        %       egoPoseStruct(1).States = linspace(0,1,20)'*[10 0 0];
        %       updateEgoPose(oList, egoID, egoPoseStruct);
        %
        %       % Check for collisions, but this time return full results
        %       % and distance to each object.
        %       options = struct('FullResults', true, 'ReturnDistance', true);
        %       [results, distance] = checkCollision(oList, options);
        %
        %       % Verify the results by displaying the predicted environment.
        %       ax = subplot(1,3,3);
        %       show(oList,'Parent',ax,'TimeStep',1:oList.MaxNumSteps,'ShowCollision',1);
        %       fUpdateFigure(ax, sprintf('Environment at TimeSteps = 1:20\nEgo1: 20 step trajectory\nEgo2: 20 step trajectory'));
        %
        %   See also dynamicCapsuleList
        
            narginchk(1,2);
            
            % Check for collisions
            [collisionFound, distance] = checkCollisionImpl(obj, varargin{:});
        end
        
        function copyObj = copy(obj)
        %copy Returns a deep copy of the obstacle list
            copyObj = obj.copyInternal(dynamicCapsuleList);
            copyObj.cacheEgoInfo();
            copyObj.cacheObstacleInfo();
        end
        
        function obj = dynamicCapsuleList
        %dynamicCapsuleList Creates a Capsule-based obstacleList
            narginchk(0,0);
            
            % Create capsule helper class
            capHelper = nav.algs.internal.Capsule;
            
            % Construct empty list
            obj = obj@nav.algs.internal.DynamicCapsuleListBase(capHelper);
        end
    end
    
    %% Show helpers
    methods (Access = ?nav.algs.internal.DynamicCapsuleListBase)
        function colorColliding = generatePatches(obj, axHandle, patchObj, ...
                capObjects, cachedXY, cachedV, timeSteps, internalIdx, ...
                tag, numToDisplay, numStored, showAllStates, collisionFound)
        %generatePatches Create or update patches based on current state of the list
        %   axHandle       - Axes
        %   patchObj       - hggroup containing all ego or obstacle hg objects
        %   capObjects     - Internal set of ego/obstacle objects
        %   cachedXY       - Set of XY base points for set of ego/obstacles
        %   cachedV        - Set of orientation vectors
        %   timeSteps      - Set of times at which the list is displayed
        %   internalIdx    - Indices of objects to be displayed
        %   tag            - Char array, either 'ego' or 'obstacle'
        %   numToDisplay   - Number of patch objects to generate
        %   numStored      - Current number of patch objects on plot
        %   showAllStates  - Determines whether plot will show entire time horizon
        %   collisionFound - Dictates whether collision visualization is on for ego capsules
            if nargin == 13
                showCollisions = true;
                colorColliding = nan(numel(timeSteps),numToDisplay);
            else
                showCollisions = false;
            end
        
            for idx = 1:numToDisplay
                i = internalIdx(idx);
                id = capObjects(i).ID;
                
                if showCollisions
                    % Ego case where checkCollision has been requested
                    colorColliding(:,numToDisplay-idx+1)=collisionFound(timeSteps,i);
                end
                
                if numel(capObjects) > 0
                    L = capObjects(i).Geometry.Length;
                    R = capObjects(i).Geometry.Radius;
                    if idx > numStored
                        [axHandle, capGroupHandle] = obj.CapHelper.showCapsule(axHandle, cachedXY, cachedV, L, R, i, timeSteps, showAllStates);
                        capGroupHandle.Annotation.LegendInformation.IconDisplayStyle = 'on';
                        capGroupHandle.DisplayName = [tag ': ' num2str(id)];
                        set(capGroupHandle, 'Parent', patchObj, 'Tag', ['ObstacleList' tag 'Object']);
                    else
                        obj.CapHelper.showCapsule(axHandle, cachedXY, cachedV, L, R, i, timeSteps, showAllStates, patchObj.Children(end-i+1));
                    end
                end
            end
            % Remove extra patches
            delete(patchObj.Children(1:(numel(patchObj.Children)-numToDisplay)))
        end
        
        function colorPatches(obj, multiCapGroup, numToShow, objIndices, timeSteps, colorBases, collisionList)
        %colorPatches Sets the color of each object set differently
            
            % multiCapGroup structure in 2D list:
                    %   hggroup         Group used to contain all ego or obstacle handle-graphic objects
                    %   hggroup         Group used to contain all handle-graphic objects for a particular capsule object
                    %   hgtransform     Transform containing patch object
                    %   patchObject     Patch object representing one capsule at one or more states.
                    %                   Each face/referenced vertices corresponds to 1 state
            
            x = linspace(0,.9,obj.MaxNumSteps)';
            colorEnd  = [.5 .5 .5];
            
            if nargin >= 7
                % Collisions shown on ego patches
                for i = 1:numToShow
                    % Index of capsule in list
                    internalIdx = objIndices(i);
                    % Index of handle-object under hggroup
                    patchIdx = numToShow-i+1;
                    colorBase = colorBases(internalIdx,:);
                    color = colorBase + x*(colorEnd - colorBase);
                    for j = 1:size(collisionList,1)
                        if collisionList(j,patchIdx) == true
                            color(timeSteps(j),:) = [1 0 0];
                        end
                    end
                    multiCapGroup.Children(patchIdx).Children.Children.Children.CData = reshape(color(timeSteps,:),[],1,3);
                end
            else
                % Collisions not shown
                for i = 1:numToShow
                    % Index of capsule in list
                    internalIdx = objIndices(i);
                    % Index of handle-object under hggroup
                    patchIdx = numToShow-i+1;
                    colorBase = colorBases(internalIdx,:);
                    color = colorBase + x*(colorEnd - colorBase);
                    multiCapGroup.Children(patchIdx).Children.Children.Children.CData = reshape(color(timeSteps,:),[],1,3);
                end
            end
        end
    end
    
    properties (Hidden)
    % Help text helpers
        
        %POSESTRUCT An N-element struct-array containing fields:
        %
        %   States   - Mx3 array of SE2 states, where M is the number of states
        %
        %       NOTE: Each element may contain a different number of states.
        POSESTRUCT
        
        %GEOMETRYSTRUCT An N-element struct-array containing fields:
        %
        %     Length          - Length of the capsule cylinder, in meters (Default: 2)
        %     Radius          - Radius of the capsule, in meters (Default: 1)
        %     FixedTransform  - A 3x3 transform relative to capsule's local frame (Default: eye(3))
        GEOMETRYSTRUCT
    end
end
