classdef FastVisualizationHelper < robotics.manip.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%FASTVISUALIZATIONHELPER Fast visualization helper for rigidBodyTree
%   This class acts as an interface between the figure axes, the
%   HGTransform objects, and the RigidBodyTree class.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    properties (Access = ?robotics.manip.internal.InternalAccess)

        %HGTransforms - Cell array of HG Transforms
        HGTransforms

        %RBTLineData - (N+1)-by-2 Cell array of line data
        %   The RBTLineData cell array has N+1 rows (for the N rigid bodies
        %   and base in the associated RBT) and 2 columns. The first column
        %   is a cell array of line handles, and the second column is a
        %   cell array of child body indices. For the ith row, the cell
        %   array in the first column contains all the lines that start at
        %   the rigidBody with index i. The second column contains the
        %   indices of all the child bodies to which these lines connect.
        %   For the base, the index N+1 is used. This data is used when
        %   lines are updated during visualization updates.
        RBTLineData

        %BaselineConfig - Configuration of the rigidBodyTree when it is first plotted and hgTransforms are initialized
        BaselineConfig

        %AxesHandle - Handle to the figure axes
        AxesHandle

        %IsUpdated - Flag to check if properties have been initialized
        IsUpdated
        
        %MeshAndFrameOptions - Array to store mesh and frame options
        %   3-by-1 double array to hold options for collision
        %   meshes, visual meshes or frames in that order. Value '1' 
        %   corresponds to 'true' and '0' to 'false'. Used to check if the 
        %   options have changed
        MeshAndFrameOptions
    end

    methods
        function obj = FastVisualizationHelper()
            obj.IsUpdated = false;
            obj.MeshAndFrameOptions = [0, 1, 1];
        end

        function fastUpdate(obj, rbtObj, config, parent, collisions, position, visuals, frames)
        %fastUpdate Update position and orientation of HGTransforms, Lines
        %   Inputs config, position, parent, collisions, position, visuals and frames are the values
        %   specified in the call to show method.
            configValidated = validateConfiguration(rbtObj, config);
            displayCollisions = strcmpi(collisions,'on');
            displayVisuals = strcmpi(visuals,'on');
            displayFrames = strcmpi(frames,'on');

            if numel(position) == 4
                % robot base position is represented in the following form
                % [x,y,z,yaw,pitch,roll]. Currently only yaw input is
                % accepted by show method. Assumed the pitch and roll are zero.
                basePosition = [double(position(:)'),0,0];
            else
                % this enable option to provide all 6-DOF values,
                % [x,y,z,yaw,pitch,roll] which is used only for internal
                % purpose e.g. this is required in robotScenario
                % visualization.
                basePosition = double(position);
            end

            %Determine if reinitialization needed
            obj.checkIfReinitializationRequired(rbtObj, parent, [displayCollisions, displayVisuals, displayFrames]);
            obj.MeshAndFrameOptions = [displayCollisions, displayVisuals, displayFrames];
            
            if ~obj.IsUpdated
                obj.initializeAxesChildrenAndTransforms(rbtObj, config, parent, collisions, position, visuals, frames);
                obj.BaselineConfig = robotics.manip.internal.FastVisualizationHelper.transformBodyFrames(rbtObj, configValidated, basePosition);
                obj.IsUpdated = true;
            else
                tTree = obj.transformBodyFrames(rbtObj, configValidated, basePosition);
                robotics.manip.internal.RigidBodyTreeVisualizationHelper.fastVisualizationUpdate(obj.HGTransforms, obj.RBTLineData, tTree, obj.BaselineConfig);

                %If displaying collision meshes, delete all meshes and display only collision meshes.
                %HGTransforms are not being used with the collision meshes.
                if(displayCollisions)
                    delete(findall(obj.AxesHandle,'type','patch','Tag', rbtObj.ShowTag));
                    robotics.manip.internal.RigidBodyTreeVisualizationHelper.drawCollisionGeometries(...,
                        rbtObj, obj.AxesHandle, tTree, displayCollisions);
                end
                %TODO Investigate possible usage of HGTransforms with
                %collision meshes
            end
        end
    end

    methods (Access = private)

        function initializeAxesChildrenAndTransforms(obj, rbtObj, config, parent, collisions, position, visuals, frames)
        %initializeAxesChildrenAndTransforms Initialize Axes, plot objects HGTransform array, LineData
            [obj.AxesHandle, axesObjects] = simpleShow(rbtObj, config, parent, collisions, position, false, visuals, frames);
            obj.HGTransforms = robotics.manip.internal.RigidBodyTreeVisualizationHelper.addHGTransforms(axesObjects, obj.AxesHandle);
            obj.RBTLineData = axesObjects(:,3:4);

            grid(obj.AxesHandle, 'on');
            rotate3d(obj.AxesHandle, 'off');
        end

        function checkIfReinitializationRequired(obj, rbtObj, parent, newMeshAndFrameOptions)
        %checkIfReinitializationRequired Check if HGTransform reinitialization required
        %   Check if there is a need to reinitialize HGTransforms which may
        %   be if there is any change to the rigidBodyTree or any 
        %   visualization options or if either axis, parent or HGTransforms
        %   are not defined properly
            if obj.IsUpdated == false
                return;
            else
                %if the collisions, visuals or frames option has changed
                haveMeshFrameOptionsChanged = any(obj.MeshAndFrameOptions ~= newMeshAndFrameOptions);
                
                %if "Parent" is defined and Parent is not equal to
                %obj.AxesHandle
                isNewParent = ~isempty(parent) && ~isequal(parent, obj.AxesHandle);

                %If HGtransforms/ axes has been deleted
                isDeletedHGT = isempty(obj.HGTransforms) || ~isvalid(obj.HGTransforms{1});

                %if a parent is not defined and the gca doesn't match the
                %obj.AxesHandle
                isAxesChanged = isempty(parent) && ~isequal(gca, obj.AxesHandle);

                %If axis is undefined or invalid
                isAxesUndefined = isempty(obj.AxesHandle) || ~isvalid(obj.AxesHandle);
                
                %If bodies have been added or removed
                %Number of HGTransforms equals the number of bodies +1
                hasTreeSizeChanged = ~isequal(rbtObj.NumBodies+1, size(obj.HGTransforms, 1));
                
                %Needs re-initialization when IsUpdated is false
                obj.IsUpdated = ~(isAxesUndefined || isNewParent || isDeletedHGT || isAxesChanged || haveMeshFrameOptionsChanged || hasTreeSizeChanged);
            end
        end
    end

    methods(Static, Access = private)
        function robotTransformTree = transformBodyFrames (rbtObj, configValidated, basePosition)
        %transformBodyFrames Rotate and translate all the body frames according to robot position
            positionTform = trvec2tform(basePosition(1:3))*eul2tform(basePosition(4:6));

            robotTransformTree = rbtObj.forwardKinematics(configValidated);
            % Rotate and translate all the body frames according to robot position
            for k = 1:length(robotTransformTree)
                robotTransformTree{k} = positionTform*robotTransformTree{k};
            end
            robotTransformTree{end+1} = positionTform;
        end
    end

end

% LocalWords:  showargs HGtransforms
