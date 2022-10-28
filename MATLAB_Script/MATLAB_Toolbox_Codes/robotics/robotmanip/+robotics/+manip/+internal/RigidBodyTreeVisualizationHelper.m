classdef RigidBodyTreeVisualizationHelper < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %RIGIDBODYTREEVISUALIZATIONHELPER Helper functions for RigidBodyTree
    %   show method.
    
    %   Copyright 2017-2021 The MathWorks, Inc.
    
    properties (Constant)
        %NumTessellation
        NumTessellation = 16
        
        %CollisionMeshPatchColor
        %   Arbitrary choice of color for a collision mesh patch aside from the
        %   default black [0, 0, 0].
        CollisionMeshPatchColor = [0.2, 0.7, 0.2]
        
    end
    
    methods (Static, Access = ?robotics.manip.internal.InternalAccess)
        
        function resetScene(robot, ax)
            %resetScene
            
            % PRIMARY axes
            axis(ax, 'vis3d');
            hFigure = ax.Parent;
            pan(hFigure,'on');
            rotate3d(hFigure,'on');
            
            ax.Visible = 'off';
            daspect(ax, [1 1 1]);
            
            
            % estimate the size of workspace
            if ~robot.IsMaxReachUpToDate
                robot.estimateWorkspace;
                robot.IsMaxReachUpToDate = true;
            end
            a = robot.EstimatedMaxReach; % updated by estimateWorkspace method
            set(ax, 'xlim', [-a, a], 'ylim', [-a, a], 'zlim', [-a, a]);
            
            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');
            
            % set up view
            view(ax, [135 8]);
            set(ax, 'Projection', 'perspective');
            ax.CameraViewAngle = 8.0;
            ax.Tag = 'Primary';
            grid(ax, 'on');
            
            % set up lighting
            if isempty(findobj(ax,'Type','Light'))
                light('Position',[a, 0, a],'Style','local','parent',ax);
            end
            ax.Visible = 'on';
            
            ax.DeleteFcn = @robotics.manip.internal.RigidBodyTreeVisualizationHelper.clearCornerAxes;
            
        end
        
        function clearCornerAxes(src, ~)
            %clearCornerAxes
            axArray = findall(src.Parent, 'Type', 'Axes', 'Tag', 'CornerCoordinateFrame');
            for idx = 1:numel(axArray)
                delete(axArray(idx));
            end
        end
        
        function hgArray = addHGTransforms(bodyDisplayObjArray, parentAxes)
            %addHGTransforms Add HGTransforms to the figure objects
            %   The rigidBodyTree figure display contains patches
            %   representing the frames, visual patches, and lines. This
            %   method assigns hgTransform object associations to these objects
            %   so that they can be moved in a figure without needing to
            %   redefine them. The method accepts an (N+1)x3 cell array,
            %   bodyDisplayObjArray, corresponding to the N bodies of a
            %   rigidBodyTree and the base, and an axes handle, parentAxes,
            %   that specifies the parent of the hgtransform objects. The three
            %   columns contain cell arrays of visual patches, patch objects
            %   for each frame, and cell arrays of lines, respectively. The
            %   method assigns one hgTransform for each rigidBody link, and
            %   then assigns all the figure objects associated with that body
            %   as its children. For lines, which connect two bodies, the
            %   hgTransforms are not used since they are shared over all the
            %   objects and are only used to affect orientation, not scale (the
            %   issue of scale is unique to the lines in this application).
            
            hgArray = cell(size(bodyDisplayObjArray,1),1);
            
            for i = 1:size(bodyDisplayObjArray,1)
                % For each rigidBody link, there is one associated
                % hgtransform object. The transform is explicitly parented
                % to the axes handle given as input.
                hgArray{i} = hgtransform(parentAxes);
                
                % Cell arrays of patches for each visual
                visualPatchArray = bodyDisplayObjArray{i,1};
                for j = 1:length(visualPatchArray)
                    visualPatchArray{j}.Parent = hgArray{i};
                end
                
                % Patch objects for each frame
                framePatch = bodyDisplayObjArray{i,2};
                if ~isempty(framePatch)
                    framePatch.Parent = hgArray{i};
                end
                
                % HGTransforms are not attached to lines because lines are
                % defined by their end points. Since this approach uses one
                % HGTransform per body (that controls the motion of all the
                % associated frames, meshes, etc.), HGTransforms can't also
                % be set to control line scale, because that would
                % adversely affect the scale of other items that we want to
                % keep the same size. As a result, lines are updated by
                % changing the endpoints, rather than by associating them
                % with an hgTransform.
            end
        end
        
        function fastVisualizationUpdate(hgDataArray, rbtLineData, Ttree, TtreeBaseline)
            
            % Update the HGTransforms
            robotics.manip.internal.RigidBodyTreeVisualizationHelper.updateBodyHGTransforms(hgDataArray, Ttree, TtreeBaseline);

            hgArrayLength = size(hgDataArray,1);
            TtreeLength = size(Ttree,2);
            rbtLineDataLength = size(rbtLineData,1);

            % Update the lines connecting the frames, if applicable
            for i = 1:size(rbtLineData,1)
                bodyChildLines = rbtLineData{i,1};
                bodyChildIndices = rbtLineData{i,2};

                if hgArrayLength == TtreeLength
                    % In RBT visualization support, the base can move,
                    % thus Ttree and hgArray length are same.
                    parentBodyTransform = Ttree{i};
                else
                    % In the interactive use cases the rigid body tree is
                    % fixed, thus Ttree and hgArray length are different.
                    if i < rbtLineDataLength
                        parentBodyTransform = Ttree{i};
                    end
                end

                for j = 1:numel(bodyChildLines)
                    % Get the line
                    rigidBodyLine = bodyChildLines{j};

                    % Get the child transform
                    childBodyTransform = Ttree{bodyChildIndices{j}};

                    if hgArrayLength == TtreeLength
                        % In RBT visualization support, the base can move,
                        % thus Ttree and hgArray length are same.

                        % Get the line origin from the parent body origin,
                        % given by the parent body transform.
                        lineOrigin = parentBodyTransform(1:3,4);
                    else
                        % In the interactive use cases the rigid body
                        % tree is fixed, thus Ttree and hgArray length are
                        % different.
                        if i < rbtLineDataLength
                            lineOrigin = parentBodyTransform(1:3,4);
                        else
                            lineOrigin = [rigidBodyLine.XData; rigidBodyLine.YData; rigidBodyLine.ZData];
                        end
                    end

                    % Re-assign the line data points
                    rigidBodyLine.XData = [lineOrigin(1) childBodyTransform(1,4)];
                    rigidBodyLine.YData = [lineOrigin(2) childBodyTransform(2,4)];
                    rigidBodyLine.ZData = [lineOrigin(3) childBodyTransform(3,4)];
                end
            end
        end
        
        function updateBodyHGTransforms(hgArray, Ttree, TtreeBaseline)
            %updateBodyHGTransforms Update HGTransform matrices
            %   This method accepts and N+1 cell array of HGTransform
            %   objects (corresponding to the N bodies of an associated
            %   rigidBodyTree, and the base), a transform tree representing
            %   the poses of each of those bodies, and a base
            %   transform tree, indicating the poses of each of those
            %   bodies at the time when the HGTransforms were initialized.
            %   The method updates the HGTransform matrices, which are 4x4
            %   homogeneous transform matrices, to represent the current
            %   desired positions given by Ttree. Note that the poses in
            %   Ttree are given with respect to the base, which is defined
            %   with respect to the figure origin, whereas the HGTransform
            %   pose is defined with respect to the poses they were in when
            %   the HGTransform was initialized. These poses are defined by
            %   TtreeBaseline.
            
            hgArrayLength = size(hgArray,1);
            TtreeLength = size(Ttree,2);
            if hgArrayLength == TtreeLength
                % In RBT visualization support, the base can move, thus
                % Ttree and hgArray length are same.
                N = hgArrayLength;
            else
                % In the interactive use cases the rigid bodytree is fixed,
                % thus Ttree and hgArray length are different.
                N = hgArrayLength - 1;
            end
            % Iterate through the movable bodies and base
            for i = 1:N
                T = Ttree{i};
                % The HGTransforms are initialized in a position
                % other than the base, so the transform must be provided
                % with respect to that position. However, the rigidBodyTree
                % position is only known with respect to the origin, so it
                % is necessary to post multiply by the inverse of the
                % initial position of the rigidBodyTree (i.e. the position
                % it was in when the HGTransform was initialized).
                TBaseline = TtreeBaseline{i};
                hgMatrix = T*robotics.manip.internal.tforminv(TBaseline);

                % Update the HGTransformation matrix
                hgArray{i}.Matrix = hgMatrix;
            end
        end
        
        function clearMeshPatches(robot, ax)
            %clearMeshPatches Clears the mesh patches of the robot
            visualMeshPatches = ...
                findobj(ax.Children, ...
                "Type", "Patch", ...
                "-regexp", "DisplayName", '_mesh', ...
                'Tag', robot.ShowTag);
            visualMeshPatches(:).delete()
        end
        
        function drawCollisionGeometries(robot, ax, Ttree, displayCollisions)
            %drawCollisionGeometries Draws the collision mesh patches of the robot
            
            if displayCollisions
                vis = 'on';
            else
                vis = 'off';
            end
            
            for i = 1:robot.NumBodies+1
                if i > robot.NumBodies
                    collisionSet = robot.Base.CollisionsInternal;
                    T = Ttree{i};
                    bname = robot.BaseName;
                else
                    collisionSet = robot.Bodies{i}.CollisionsInternal;
                    T = Ttree{i};
                    bname = robot.Bodies{i}.Name;
                end
                
                visVertices = collisionSet.getVertices();
                visFaces = collisionSet.getFaces();
                visTForms = collisionSet.getTforms();
                
                for k = 1:collisionSet.Size
                    
                    V = visVertices{k};
                    F = visFaces{k};
                    TVis = visTForms{k};
                    
                    numVertices = size(V,1);
                    Vi = T*TVis*[V'; ones(1,numVertices) ];
                    Vi = Vi(1:3,:)';
                    
                    patch(ax, ...
                        'Faces', F,...
                        'Vertices', Vi,...
                        'LineStyle', 'none',...
                        'FaceColor', robotics.manip.internal.RigidBodyTreeVisualizationHelper.CollisionMeshPatchColor, ...
                        'Tag', [robot.ShowTag], ...
                        'DisplayName', [bname '_coll_mesh'], ...
                        'Visible', vis, ...
                        'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, i, T}, ...
                        'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
                end
            end
        end
        
        function [bodyDisplayObjArray, fmanager] = drawRobotMemoryLess(robot, ax, Ttree, displayFrames, displayVisuals, figManagerOpts)
            %drawRobotMemoryLess Display robot and output handles to the figure patches and lines
            %   This method draws the robot and provides a cell array
            %   output that contains the handles to the patches and lines
            %   used to draw the robot. The output cell array has dimension
            %   (N+1)x4. The dimensions correspond to the N rigidBodies and
            %   base of the associated rigidBodyTree (the base occupies the
            %   (N+1)th row). The first three columns of each cell array correspond to
            %   the handles to the associated objects for each body: visual
            %   patches, frame patches, and connecting lines, while the
            %   last column contains the child bodies corresponding to each
            %   of the connecting lines at that index (the parent body for
            %   each line is already given by the column index).
            
            % Configure the figure manager for the given axes
            if nargin < 6 || isempty(figManagerOpts)
                figManagerOpts = robotics.manip.internal.createFigureManagerOpts();
            end
            
            % Configure the figure manager for the given axes
            fmanager = robotics.manip.internal.FigureManager(ax, figManagerOpts);
            fmanager.createCornerCoordinateFrame(ax);
            
            % Initialize output
            bodyDisplayObjArray = cell(robot.NumBodies+1, 4);
            
            robotics.manip.internal.RigidBodyTreeVisualizationHelper.depositRobotShowTag(robot, ax.Parent);
            
            % body frame graphical parameters
            N = robotics.manip.internal.RigidBodyTreeVisualizationHelper.NumTessellation; % tessellation
            % magicLength, found to produce good visual for most common robots
            magicLength = 1.1;
            r = 0.005*magicLength; % axis arrow radius
            l = 15*r; % axis arrow length
            
            % frame visibility
            if displayFrames
                vis = 'on';
            else
                vis = 'off';
            end
            
            % draw base frame
            [F, V] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.bodyFrameMesh(r*0.6, l*1.3, N, false);
            V = Ttree{end}*[V'; ones(1,length(V)) ];
            V = V(1:3,:)';
            
            hasVisuals = ~isempty(robot.Base.VisualsInternal);
            % Base frame patches
            bodyDisplayObjArray{end,2} = patch(ax, ...
                'Faces', F,...
                'Vertices', V, ...
                'FaceColor', 'k', ...
                'LineStyle', 'none', ...
                'Tag', robot.ShowTag, ...
                'Visible', vis, ...
                'DisplayName', robot.BaseName, ...
                'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, robot.NumBodies+1, Ttree{end}}, ...
                'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, robot.BaseName, hasVisuals, displayVisuals), ...
                'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
            
            % draw regular body frames and connectors
            [F, V, C] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.bodyFrameMesh(r, l, N, false);
            [Ff, Vf, Cf] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.bodyFrameMesh(r, l, N, true);
            
            for i = 1:robot.NumBodies
                if strcmp(robot.Bodies{i}.JointInternal.Type, 'fixed')
                    Vi = Vf;
                    Fi = Ff;
                    Ci = Cf;
                else
                    Vi = V;
                    Fi = F;
                    Ci = C;
                end
                Vi = Ttree{i}*[Vi'; ones(1,length(Vi)) ];
                Vi = Vi(1:3,:)';
                hasVisuals = ~isempty(robot.Bodies{i}.VisualsInternal);
                % Body frame patches
                bodyDisplayObjArray{i,2} = patch(ax, 'Faces', Fi,...
                    'Vertices', Vi,...
                    'FaceVertexCData', Ci,...
                    'FaceColor', 'flat',...
                    'LineStyle', 'none',...
                    'Tag', robot.ShowTag, ...
                    'Visible', vis, ...
                    'DisplayName', robot.Bodies{i}.Name, ...
                    'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, i, Ttree{i}},...
                    'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, robot.Bodies{i}.Name, hasVisuals, displayVisuals), ...
                    'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
                
                pid = robot.Bodies{i}.ParentIndex;
                if pid == 0
                    % If the parent is the base, the transform of the
                    % parent is the origin
                    Tparent = Ttree{end};
                else
                    Tparent = Ttree{pid};
                end
                Tcurr = Ttree{i};
                p = [Tparent(1:3,4)'; Tcurr(1:3,4)'];
                
                % Create the line and associate it with the parent object
                lineObject = line(ax, p(:,1), p(:,2), p(:,3),  'Tag', robot.ShowTag, 'Visible', vis);
                
                % Store the line object in the hgObject array. The lines
                % have to index to their parent bodies, since that is the
                % transform that will be associated with the line motion.
                if pid == 0
                    % Base links, which have the origin as the parent, are
                    % stored last
                    lineIndex = robot.NumBodies + 1;
                else
                    % All other lines are indexed by the parent
                    lineIndex = pid;
                end
                
                % Since the line is associated with its parent, a frame
                % can be associated with multiple lines, which are stored
                % in a cell array. Index the line by its parent index. The
                % corresponding child body index is stored in the fourth
                % column of the bodyDisplayObjArray cell array.
                if isempty(bodyDisplayObjArray{lineIndex,3})
                    % Create the cell array if it does not yet exist
                    bodyDisplayObjArray{lineIndex,3} = {lineObject};
                    bodyDisplayObjArray{lineIndex,4} = {i};
                else
                    % Add the line to the cell array if it already exists
                    bodyDisplayObjArray{lineIndex,3} = [bodyDisplayObjArray{lineIndex,3}; {lineObject}];
                    bodyDisplayObjArray{lineIndex,4} = [bodyDisplayObjArray{lineIndex,4}; {i}];
                end
            end
            
            % body visuals visibility
            if displayVisuals
                vis = 'on';
            else
                vis = 'off';
            end
            
            for i = 1:robot.NumBodies+1
                if i > robot.NumBodies
                    visGeom = robot.Base.VisualsInternal;
                    T = Ttree{i};
                    bname = robot.BaseName;
                else
                    visGeom = robot.Bodies{i}.VisualsInternal;
                    T = Ttree{i};
                    bname = robot.Bodies{i}.Name;
                end
                
                meshPatchCell = cell(length(visGeom),1);
                for k = 1:length(visGeom)
                    V = visGeom{k}.Vertices;
                    F = visGeom{k}.Faces;
                    TVis = visGeom{k}.Tform;
                    color = visGeom{k}.Color;
                    
                    numVertices = size(V,1);
                    Vi = T*TVis*[V'; ones(1,numVertices) ];
                    Vi = Vi(1:3,:)';
                    
                    meshPatchCell{k} = patch(ax, ...
                        'Faces', F,...
                        'Vertices', Vi,...
                        'FaceColor', color(1:3),...
                        'LineStyle', 'none',...
                        'Tag', robot.ShowTag, ...
                        'DisplayName', [bname '_mesh'], ...
                        'Visible', vis, ...
                        'ButtonDownFcn', {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.infoDisplay, robot, i, T}, ...
                        'UIContextMenu', robotics.manip.internal.RigidBodyTreeVisualizationHelper.getBodyContextMenu(robot.ShowTag, ax, bname, 1, displayVisuals), ...
                        'DeleteFcn', @robotics.manip.internal.RigidBodyTreeVisualizationHelper.deleteUIContextMenuUponPatchDestruction);
                end
                % Base and body visual patches
                bodyDisplayObjArray{i,1} = meshPatchCell;
            end
            
        end
        
        function [ F, V, C ] = bodyFrameMesh( r, l, N, isFixed )
            %BODYFRAMEMESH Create body frame mesh data. The data will be used as input
            %   for patch command.
            %   r - radius of each axis
            %   l - length of each axis
            %   N - number of side surfaces of the prism (used to approximate axis
            %       cylinder)
            %   isFixed - whether fixed body or not
            %
            %   F - Faces (array of vertex indices)
            %   V - Vertex (3D point)
            %   C - Color
            
            theta = linspace(0, 2*pi,N+1)';
            
            m = length(theta);
            d = 0.9;
            
            % z-axis
            Vz = [r*cos(theta), r*sin(theta), theta*0];
            
            if isFixed
                Vz = [Vz;Vz;Vz];
                Vz(m+1:2*m,3) = d*l*ones(m, 1);
                Vz(2*m+1:3*m, 3) = l*ones(m,1);
            else
                Vz = [Vz;Vz];
                Vz(m+1:2*m,3) = l*ones(m, 1);
            end
            
            
            Fz = [];
            for i = 1:m-1
                f = [i, i+1, m+i;
                    m+i, i+1, m+i+1];
                Fz = [Fz; f ]; %#ok<AGROW>
            end
            lf1 = size(Fz,1);
            
            if isFixed
                for i = m+1:2*m-1
                    f = [i, i+1, m+i;
                        m+i, i+1, m+i+1];
                    Fz = [Fz; f ]; %#ok<AGROW>
                end
                
                for i = (2*m+2):(3*m-2)
                    f = [i,i+1,3*m];
                    Fz = [Fz; f];  %#ok<AGROW>
                end
            end
            
            
            % y-axis
            Vy = [r*cos(theta), theta*0, r*sin(theta)];
            if isFixed
                Vy = [Vy;Vy;Vy];
                Vy(m+1:2*m,2) = d*l*ones(m, 1);
                Vy(2*m+1:3*m, 2) = l*ones(m,1);
            else
                Vy = [Vy;Vy];
                Vy(m+1:2*m,2) = l*ones(m, 1);
            end
            
            % x-axis
            Vx = [theta*0, r*cos(theta), r*sin(theta)];
            if isFixed
                Vx = [Vx;Vx;Vx];
                Vx(m+1:2*m, 1) = d*l*ones(m, 1);
                Vx(2*m+1:3*m, 1) = l*ones(m, 1);
            else
                Vx = [Vx;Vx];
                Vx(m+1:2*m, 1) = l*ones(m, 1);
            end
            
            % assemble
            V = [Vz; Vy; Vx];
            if isFixed
                Fy = 3*m + Fz;
                Fx = 6*m + Fz;
            else
                Fy = 2*m + Fz;
                Fx = 4*m + Fz;
            end
            F = [Fz; Fy; Fx];
            
            lf = length(Fz);
            if isFixed
                lf2 = lf - lf1;
                
                % fixed frame color scheme
                C =  [repmat([1,0,1],lf1,1);
                    repmat([0,0,1],lf2,1);
                    repmat([1,0,1],lf1,1);
                    repmat([0,1,0],lf2,1);
                    repmat([1,0,1],lf1,1);
                    repmat([1,0,0],lf2,1)];
            else
                
                % non-fixed frame color scheme
                C =  [repmat([0,0,1],lf,1);
                    repmat([0,1,0],lf,1);
                    repmat([1,0,0],lf,1)];
            end
            
            F = robotics.core.internal.PrimitiveMeshGenerator.flipFace(F);
        end
        
        function [ F, V ] = torusSectionMesh( r, d )
            %torusSectionMesh Create mesh for a section of torus. Note the
            %   mesh generated does not include the cross section surface.
            %
            %   Imagine the torus section as a donut lying in the xy plane,
            %   with the center at the origin, and a quarter of it is
            %   bitten off.
            %
            %   The input r is the radius of the torus, d is the radius of
            %   the cross section.
            if nargin == 0
                r = 0.5;
                d = 0.02;
            end
            
            if nargin == 1
                d = 0.02;
            end
            
            % torus section tessellation
            m = 16;
            n = 32;
            s = 4;
            
            V1 = zeros(m, 3);
            for j = 1:m
                theta = j*2*pi/m;
                V1(j, :) = [r + d * cos(theta), 0, d*sin(theta)];
            end
            
            F1 = [];
            for i = 1:m
                f = [i, i+1, m+i;
                    m+i, i+1, m+i+1];
                if i==m
                    f = [m, 1, m+m;
                        m+m, 1, m+1];
                end
                F1 = [F1; f ]; %#ok<AGROW>
            end
            
            V = zeros(m*(n-2*s), 3);
            for i = s:n-s
                phi = i/n*(2*pi);
                R = expm(robotics.manip.internal.skew([0 0 1])*phi);
                V( (i-s)*m+1:(i-s+1)*m,:) = (R*V1')';
            end
            
            nf = length(F1);
            F = zeros((n-2*s)*nf, 3);
            for i = 1:n-2*s
                F((i-1)*nf+1:i*nf, :) = F1+(i-1)*m*ones(nf,3);
            end
            
            F = robotics.core.internal.PrimitiveMeshGenerator.flipFace(F);
            
        end
        
        
        function [ Fa, Va ] = arrowHeadMesh( r, l )
            %arrowHeadMesh Create mesh for an arrow head. The arrow head is
            %   a circular cone that sits at the origin, tapering towards
            %   the +z direction. The input  r is the radius of the cone
            %   base, and l is the height of the cone.
            N = 32;
            theta = linspace(0, 2*pi,N);
            theta = theta(1:end-1)';
            
            m = length(theta);
            
            Va = [r*cos(theta), r*sin(theta), theta*0];
            apex = [0 0 l];
            center = [0 0 0];
            Va = [Va; apex; center];
            
            Fa = zeros(2*m, 3);
            for i = 1:m-1
                Fa(i,:) = [ i, i+1, m+1];
                Fa(i+m, :) = [i, m+2, i+1];
            end
            Fa(m,:) = [m, 1, m+1];
            Fa(2*m,:) = [m, m+2, 1];
            
            Fa = robotics.core.internal.PrimitiveMeshGenerator.flipFace(Fa);
        end
        
        
        function [ F, V ] = jointAxisMesh( r, l, isRevolute )
            %jointAxisMesh Create visual meshes for revolute/prismatic
            %   joint axis. The input r is base radius for the joint axis
            %   arrow head, l is the length of the joint axes,
            %   isRevolute is a logical that indicates whether this joint
            %   is a revolute joint.
            
            % cylinder
            [ Fz, Vz ] = robotics.core.internal.PrimitiveMeshGenerator.cylinderMesh([r, l]);
            Vz(:,3) = Vz(:,3) + l/2;
            
            % arrow head
            [ Fa, Va ] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.arrowHeadMesh(3*r, 10*r);
            Va(:,3) = Va(:,3) + l;
            
            Ft = []; Vt = [];
            Fa2 = []; Va2 = [];
            if isRevolute % draw the circling arrow
                % torus section
                p = 5;
                q = 1.2;
                [ Ft, Vt ] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.torusSectionMesh( p*r, 0.5*r );
                Vt(:,3) = Vt(:,3) + q*l;
                
                % arrow head 2
                [ Fa2, Va2 ] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.arrowHeadMesh(r, 3*r);
                th = (28/32)*2*pi;
                T = [ 0 cos(th) -sin(th) p*r*cos(th);
                    0 sin(th)  cos(th) p*r*sin(th);
                    1 0        0       q*l;
                    0 0        0       1];
                Va2 = (T*([Va2 ones(length(Va2),1)])')';
                Va2 = Va2(:,1:3);
            end
            
            F = [Fz; Fa+ length(Vz); Ft+length(Vz)+length(Va); Fa2+length(Vz)+length(Va)+length(Vt)];
            V = [Vz; Va; Vt; Va2];
        end
        
        function infoDisplay(src, evt, robot, bid, T)
            %infoDisplay Callback to display body information
            
            % Need to obtain the axes from the source object that has been
            % clicked on. If the object has an assigned hgTransform, this
            % will be up two levels; otherwise, the axes should be the
            % object's immediate parent.
            if isa(src.Parent, 'matlab.graphics.primitive.Transform')
                ax = src.Parent.Parent;
                
                % If the parent is an hgTransform, the source transform
                % needs to be updated to reflect the current configuration,
                % rather than the configuration when this callback was
                % initialized
                T = src.Parent.Matrix*T;
            else
                ax = src.Parent;
            end
            
            if bid > robot.NumBodies
                bodyhandle = robot.Base;
            else
                bodyhandle = robot.Bodies{bid};
            end
            
            bm1 = findobj(ax, 'type', 'patch', 'DisplayName', bodyhandle.Name, 'Tag', robot.ShowTag);
            bm2 = findobj(ax, 'type', 'patch', '-regexp', 'DisplayName', [bodyhandle.Name '_(coll_)?mesh'], 'Tag', robot.ShowTag);
            bm = [bm1; bm2]; % must be assembled vertically
            
            robotics.manip.internal.RigidBodyTreeVisualizationHelper.drawJointAxisMemoryLess( ax, bodyhandle, T);
            
            for i = 1:length(bm)
                bm(i).LineStyle = '-';
                bm(i).EdgeColor = robotics.manip.internal.FigureManager.BackgroundColor;
            end
            
            if evt.Button == 1 % left click
                msgs{1} = sprintf('Body Name: %s', bodyhandle.Name);
                if bodyhandle.Index == 0
                    msgs{end+1} = sprintf('Robot Base');
                else
                    msgs{end+1} = sprintf('Body Index: %d', bodyhandle.Index);
                end
                if (bodyhandle.Index > 0)
                    msgs{end+1} = sprintf('Joint Type: %s', bodyhandle.Joint.Type);
                end
                
                fmanager = getappdata(ax.Parent, robotics.manip.internal.FigureManager.FigureManagerIdentifier);
                if ~isempty(fmanager)
                    fmanager.BannerText = msgs;
                    fmanager.updateBannerText();
                    fmanager.showBanner();
                end
            end
        end
        
        function jointAxisPatch = drawJointAxisMemoryLess(ax, body, T)
            %drawJointAxisMemoryLess Draw the joint axis
            %   The joint axis is drawn when the joint is a moving joint.
            %   This function draws the axis and direction of rotation, and
            %   returns a patch object. When the joint does not move, the
            %   patch is returned as empty.

            % Initialize output
            jointAxisPatch = [];

            if body.Index < 1 % base, ignore
                return
            end
            isRevolute = strcmp(body.Joint.Type, 'revolute');
            isPrismatic = strcmp(body.Joint.Type, 'prismatic');
            if isRevolute || isPrismatic
                ja = body.Joint.JointAxis;
                if isfinite(ja)
                    n = robotics.manip.internal.skew([0 0 1])*ja(:);
                    if norm(n) < 1e-12
                        if ja(3) > 0
                            R = eye(3);
                        else
                            R = eul2rotm([0 0 pi]);
                        end
                    else
                        n = n/norm(n); % the rotation axis vector needs to be normalized
                        theta = acos(ja(3)/norm(ja));
                        R = expm(theta*robotics.manip.internal.skew(n));
                    end
                    TJ = eye(4);
                    TJ(1:3,1:3) = R;
                    TJ = robotics.core.internal.SEHelpers.tforminvSE3(body.Joint.ChildToJointTransform)*TJ;
                end
                
                [ F, V ] = robotics.manip.internal.RigidBodyTreeVisualizationHelper.jointAxisMesh(0.005, 0.11, isRevolute);
                Vs = single( T*TJ*[V'; ones(1, size(V,1))] );
                Vs = Vs(1:3, :)';
                Fs = int32(F);
                jointAxisPatch = patch('Faces', Fs, 'Vertices', Vs, ...
                    'FaceColor', [1 1 0],... % yellow
                    'Parent', ax, ...
                    'LineStyle','none',...
                    'Tag', 'HighlightedJointAxis', ...
                    'HitTest', 'off'); % make sure the joint axis visual does not respond to mouse click
            end
        end
        
        
        
        function cm = getBodyContextMenu(robotShowTag, ax, bodyName, hasVisuals, isVisible)
            %getBodyContextMenu Helper to set up body-attached context
            %   menu
            cm = uicontextmenu(ax.Parent);
            props.Parent = cm;
            props.Callback = {@robotics.manip.internal.RigidBodyTreeVisualizationHelper.setSingleBodyMeshVisibility, robotShowTag, bodyName};
            props.Separator = 'on';
            props.Tag = bodyName;
            if hasVisuals
                if isVisible
                    props.Label = message('robotics:robotmanip:figuremanager:MeshOff').getString;
                else
                    props.Label = message('robotics:robotmanip:figuremanager:MeshOn').getString;
                end
            else
                props.Label = message('robotics:robotmanip:figuremanager:NoMesh').getString;
            end
            uimenu(props);
            
        end
        
        function setSingleBodyMeshVisibility(src, evt, robotShowTag, bodyName) %#ok<INUSL>
            %setSingleBodyMeshVisibility
            ax = src.Parent.Parent.CurrentAxes;
            meshObjs = findall(ax, 'Type', 'patch', 'DisplayName', [bodyName '_mesh'], 'Tag', robotShowTag);
            umObjs = findall(ax.Parent, 'Type', 'uimenu', 'Tag', bodyName);
            
            if ~isempty(meshObjs)
                if strcmp(meshObjs(1).Visible, 'on')
                    [meshObjs.Visible] = deal('off');
                    [umObjs.Label] = deal(message('robotics:robotmanip:figuremanager:MeshOn').getString);
                else
                    [meshObjs.Visible] = deal('on');
                    [umObjs.Label] = deal(message('robotics:robotmanip:figuremanager:MeshOff').getString);
                end
            end
        end
        
        function depositRobotShowTag(robot, parent)
            %depositRobotShowTag
            name = robotics.manip.internal.FigureManager.RobotShowTagsIdentifier;
            retObj = getappdata(parent, name);
            
            if isempty(retObj)
                
                setappdata(parent, name, {robot.ShowTag});
            else
                tmp = [retObj, robot.ShowTag];
                setappdata(parent, name, unique(tmp));
            end
        end
        
        
        function deleteUIContextMenuUponPatchDestruction(src, evt) %#ok<INUSD>
            %deleteUIContextMenuUponPatchDestruction
            %   src is the deleted object, evt is empty in this case
            
            if ~isempty(src.UIContextMenu) && ishandle(src.UIContextMenu)
                delete(src.UIContextMenu);
            end
        end
    end
    
    
    
end
