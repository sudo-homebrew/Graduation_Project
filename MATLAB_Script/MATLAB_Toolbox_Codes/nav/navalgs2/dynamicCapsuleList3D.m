classdef dynamicCapsuleList3D < nav.algs.internal.DynamicCapsuleListBase
%dynamicCapsuleList3D Dynamic capsule-based obstacle list
%   The dynamicCapsuleList3D object manages two lists of collision primitive
%   objects, ego bodies and obstacles. Each object in the list is defined by
%   a unique identifier, geometry, and set of SE3 states. The list assumes
%   that states are separated by a fixed interval of time, where the N-th
%   state in the list corresponds to the time T = (N-1)*dT. One or more ego
%   bodies or obstacles are represented as structs-arrays, where each element
%   contains the fields:
%
%       ID       - Integer that identifies the object
%
%       States   - Nx7 array of SE3 states, [x y z qW qX qY qZ]
%
%       Geometry - Structure to define the capsule geometry with the fields:
%
%                     Length          - Length of the capsule cylinder, in meters (Default: 2)
%                     Radius          - Radius of the capsule, in meters (Default: 1)
%                     FixedTransform  - A 4x4 transform relative to capsule's local frame (Default: eye(4))
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
%   dynamicCapsuleList3D properties:
%      EgoIDs                           - Integers that identify the ego bodies
%      MaxNumSteps                      - Maximum number of steps
%      NumObstacles                     - Number of obstacles in list
%      NumEgos                          - Number of ego bodies in list
%      ObstacleIDs                      - Integers that identify the obstacles
%
%   dynamicCapsuleList3D methods:
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
%       % Create a dynamicCapsuleList3D object.
%       oList = dynamicCapsuleList3D;
%
%       % Specify an ID for the first ego body.
%       egoID1 = 1;
%
%       % Generate a capsule geometry structure. Capsule geometries are 
%       % defined by a length, radius, and fixed-transform field.
%       geom = struct('Length', 4, 'Radius', 2, 'FixedTransform', eye(4));
%       
%       % Create a set of states based on an equation of motion for the ego
%       % body. Generate a series of time steps, and calculate the state
%       % based on the given equation.
%       timeStamps = linspace(0,1,oList.MaxNumSteps)';
%       xyz = timeStamps.*[100 100 20];
%       orientation = eul2quat(timeStamps.*[pi/6 0 pi]);
%       states = [xyz orientation];
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
%       view(3);
%
%       % Generate a default geometry structure for an obstacle using a new ID.
%       newID = 10;
%       [newID, obstacleGeom] = obstacleGeometry(oList, newID);
%
%       % Customize the obstacle's geometry.
%       obstacleGeom.Geometry.Length = 10;
%       obstacleGeom.Geometry.Radius = 3;
%       t = [1; 0; 0];
%       R = [eul2rotm([pi/4 0 0])];
%       obstacleGeom.Geometry.FixedTransform(1:3,:) = [R t];
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
%       view(3);
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
%       obstaclePoseStruct(2).States(:,1:3) = [100 100 20]-obstaclePoseStruct(2).States(:,1:3);
%
%       % Move the original obstacle to a new location.
%       obstaclePoseStruct(1).States = [10*(rand(1,3)-.5) eul2quat(2*pi*(rand(1,3)-.5))];
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
%       view(3);
%       ax.XLimMode = "manual";
%       ax.YLimMode = "manual";
%       ax.ZLimMode = "manual";
%
%       % Loop over the scene to see the collisions:
%       for i = 1:oList.MaxNumSteps
%           show(oList, 'Parent', ax, 'TimeStep', i, 'ShowCollision', true, 'FastUpdate', true);
%           drawnow;
%       end
%
%   See also dynamicCapsuleList.

%   Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties (Access = protected)
        %CapHelper Class storing static helper methods
        CapHelper
    end
    
    properties (Constant, Access = ?nav.algs.internal.DynamicCapsuleListBase)
        %Dimension
        Dimension = 3;
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
        %       States   - Mx7 array of SE3 states, [x y z qW qX qY qZ]
        %
        %       Geometry - Structure with the fields:
        %
        %                     Length          - Length of the capsule cylinder, in meters (Default: 2)
        %                     Radius          - Radius of the capsule, in meters (Default: 1)
        %                     FixedTransform  - A 4x4 transform relative to capsule's local frame (Default: eye(4))
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
        %       States   - Mx7 array of SE3 states, [x y z qW qX qY qZ]
        %
        %       Geometry - Structure with the fields:
        %
        %                     Length          - Length of the capsule cylinder, in meters (Default: 2)
        %                     Radius          - Radius of the capsule, in meters (Default: 1)
        %                     FixedTransform  - A 4x4 transform relative to capsule's local frame (Default: eye(4))
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
        %       g = @(ax)set(ax,'DataAspectRatio',[1 1 1],'View',[-37.5, 30]);
        %       h = @(ax,str)title(ax,str);
        %       fUpdateFigure = @(ax,str)f(g(ax),h(ax,str),ax);
        %
        %       % Create empty list.
        %       oList = dynamicCapsuleList3D
        %
        %       % Add two ego bodies to the list.
        %       [egoID, egoGeom] = egoGeometry(oList, [1;2])
        %       updateEgoGeometry(oList, egoID, egoGeom)
        % 
        %       % Add two obstacles with default geometry and fully defined
        %       % trajectories.
        %       [obstacleID, obsStates] = obstaclePose(oList, [1;2])
        %       obsStates(1).States = [rand(31,3)+linspace(0,1,31)'*[ 10 0 5]+[0 10 0] eul2quat(2*pi*rand(31,3))];
        %       obsStates(2).States = [rand(31,3)+linspace(0,1,31)'*[-10 5 0]+[10 0 0] eul2quat(2*pi*rand(31,3))];
        %       updateObstaclePose(oList, obstacleID, obsStates)
        % 
        %       % Update the poses for the two ego bodies so that one
        %       % travels for 10 time steps, and the other travels for 20.
        %       [egoID, egoPoseStruct] = egoPose(oList);
        %       egoPoseStruct(1).States = [linspace(0,1,10)'*[10 0 0] normalize(linspace(0,1,10)'*[1 0 0 -1]+[0 0 0 1],2,'norm')];
        %       egoPoseStruct(2).States = [linspace(0,1,20)'*[10 10 0] normalize(linspace(0,1,20)'*[-1 0 0 1]+[1 0 0 0],2,'norm')];
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
        %       egoPoseStruct(1).States = [linspace(0,1,20)'*[10 0 0] normalize(linspace(0,1,20)'*[1 0 0 -1]+[0 0 0 1],2,'norm')];
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
            copyObj = obj.copyInternal(dynamicCapsuleList3D);
            copyObj.cacheEgoInfo();
            copyObj.cacheObstacleInfo();
        end
        
        function obj = dynamicCapsuleList3D
        %dynamicCapsuleList Creates a Capsule-based obstacleList
            narginchk(0,0);
            
            % Create 3D capsule helper class
            capHelper = nav.algs.internal.Capsule3D;
            
            % Construct empty list
            obj = obj@nav.algs.internal.DynamicCapsuleListBase(capHelper);
        end
    end
    
    %% Show helpers
    methods (Access = ?nav.algs.internal.DynamicCapsuleListBase)
        function colorColliding = generatePatches(obj, axHandle, patchObj, ...
                capObjects, ~, ~, timeSteps, internalIdx, tag, ...
                numToDisplay, numStored, showAllStates, collisionFound)
        %generatePatches Create or update patches based on current state of the list
        %   axHandle       - Axes
        %   patchObj       - hggroup containing all ego or obstacle hg objects
        %   capObjects     - Internal set of ego/obstacle objects
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
                
                if idx > numStored
                    [~, capGroupHandle] = obj.CapHelper.showCapsule(axHandle,capObjects(i),timeSteps,showAllStates);
                    capGroupHandle.Annotation.LegendInformation.IconDisplayStyle = 'on';
                    capGroupHandle.DisplayName = [tag ': ' num2str(id)];
                    set(capGroupHandle, 'Parent', patchObj, 'Tag', ['ObstacleList' tag 'Object']);
                else
                    capGroupHandle = patchObj.Children(end-i+1);
                    obj.CapHelper.showCapsule(axHandle,capObjects(i),timeSteps,showAllStates,capGroupHandle);
                end
            end
            % Remove extra patches
            delete(patchObj.Children(1:(numel(patchObj.Children)-numToDisplay)))
        end
        
        function colorPatches(obj, multiCapGroup, numToShow, objIndices, timeSteps, colorBases, collisionList)
        %colorPatches Sets the color of each object set differently
            x = linspace(0,.9,obj.MaxNumSteps)';
            colorEnd  = [.5 .5 .5];
            
            % multiCapGroup structure in 2D list:
                    %   hggroup         Group used to contain all ego or obstacle handle-graphic objects
                    %   hggroup         Group used to contain all handle-graphic objects for a particular capsule object
                    %   hgtransform     Transform containing global state transform
                    %   hgtransform     Transform containing local FixedTransform
                    %   patchObject     Patch object vertices representing the capsule geometry in default pose
            
            if nargin >= 7
                % Collisions shown on ego patches
                for i = 1:numToShow
                    % Index of capsule in list
                    internalIdx = objIndices(i);
                    % Index of handle-object under hggroup
                    transformIdx = numToShow-i+1;
                    colorBase = colorBases(internalIdx,:);
                    color = colorBase + x*(colorEnd - colorBase);
                    for j = numel(multiCapGroup.Children(transformIdx).Children):-1:1
                        if collisionList(j,transformIdx) == true
                            multiCapGroup.Children(transformIdx).Children(j).Children.Children.FaceColor = [1 0 0];
                        else
                            multiCapGroup.Children(transformIdx).Children(j).Children.Children.FaceColor = color(timeSteps(j),:);
                        end
                    end
                end
            else
                % Collisions not shown
                for i = 1:numToShow
                    % Index of capsule in list
                    internalIdx = objIndices(i);
                    % Index of handle-object under hggroup
                    transformIdx = numToShow-i+1;
                    colorBase = colorBases(internalIdx,:);
                    color = colorBase + x*(colorEnd - colorBase);
                    numStep = numel(multiCapGroup.Children(transformIdx).Children);
                    for j = numStep:-1:1
                        multiCapGroup.Children(transformIdx).Children(j).Children.Children.FaceColor = color(timeSteps(j),:);
                    end
                end
            end
        end
    end
    
    properties (Hidden)
    % Help text helpers
        
        %POSESTRUCT An N-element struct-array containing fields:
        %
        %   States   - Mx7 array of SE3 states, [x y z qW qX qY qZ], where M is the number of states
        %
        %   	NOTE: Each element may contain a different number of states.
        POSESTRUCT
        
        %GEOMETRYSTRUCT An N-element struct-array containing fields:
        %
        %     Length          - Length of the capsule cylinder, in meters (Default: 2)
        %     Radius          - Radius of the capsule, in meters (Default: 1)
        %     FixedTransform  - A 4x4 transform relative to capsule's local frame (Default: eye(4))
        GEOMETRYSTRUCT
    end
end
