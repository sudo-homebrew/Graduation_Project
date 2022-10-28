classdef RotationBoundsVisual
%This class is for internal use only. It may be removed in a future release.

%RotationBoundsVisual A visual for rotation bounds
%   The rotation bounds visual is a set of patches in X, Y, and Z visualized as
%   planar sectors from thetaMin to thetaMax bounds about each of those axes.

%   Copyright 2020 The MathWorks, Inc.
%#codegen

    properties
        %PatchRadius
        PatchRadius = ...
            robotics.manip.internal.workspaceGoalRegion.WGRVisual.DefaultPatchRadius

        %PatchColor
        PatchColor = {'r', 'g', 'b'}

        %RotZ
        RotZ = [0, 0]

        %RotY
        RotY = [0, 0]

        %RotZ
        RotX = [0, 0]

        %Alpha
        Alpha = 0.2

        %Pose
        Pose = eye(4)
    end

    properties(Constant, Access=private)
        %NumAngularInterpolations
        NumAngularInterpolations = 100

        %Tag
        Tag = 'DO_NOT_EDIT_RotationBounds'

        %FirstRowDataTipTemplateLabel
        FirstRowDataTipTemplateLabel = ' Orientation Bounds'

        %SecondRowDataTipTemplateLabel
        SecondRowDataTipTemplateLabel = 'min: '

        %ThirdRowDataTipTemplateLabel
        ThirdRowDataTipTemplateLabel = 'max: '

        %EmptyDataTipTemplateValue
        EmptyDataTipTemplateValue = ''
    end

    methods(Access=private, Static)
        function exposeDataTipTemplateOnPatch(p)
        %exposeDataTipTemplateOnPatch Exposes the DataTipTemplate property on patch
        %   By default, the DataTipTemplate doesn't show up for a patch unless
        %   it is pinned. Hence, we need to add a datatip first in order for the
        %   property to show up.
            dt = datatip(p,'Visible','off');
            delete(dt);
        end
    end

    methods(Access=private)
        function showDataTipOnPatch(obj, p, axisLetter, thetaMin, thetaMax)
        %showDataTipOnPatch Creates a datatip on the patch
        %   On clicking on the rotation bound patch, the nearest point on the
        %   periphery snaps to show the "<AXISLETTER> Orientation Bounds" label,
        %   where axis is the corresponding axis of the reference frame.
        %   A datatip on the rotation bound patch should appear as follows:
        %   ---
        %   <AXISLETTER> Orientation Bounds
        %   min: minBound
        %   max: maxBound
        %   ---
            obj.exposeDataTipTemplateOnPatch(p);
            p.DataTipTemplate.DataTipRows(1).Label = ...
                [axisLetter, obj.FirstRowDataTipTemplateLabel];
            p.DataTipTemplate.DataTipRows(1).Value = obj.EmptyDataTipTemplateValue;
            p.DataTipTemplate.DataTipRows(2).Label = obj.SecondRowDataTipTemplateLabel;
            p.DataTipTemplate.DataTipRows(3).Label = obj.ThirdRowDataTipTemplateLabel;

            % A datatip for a patch means that each vertex which makes the patch
            % has a datatip. That means that hovering over any of the vertices
            % of the patch should show a datatip.
            numVertices = size(p.Vertices, 1);
            p.DataTipTemplate.DataTipRows(2).Value = repmat(thetaMin, numVertices, 1);
            p.DataTipTemplate.DataTipRows(3).Value = repmat(thetaMax, numVertices, 1);
        end

        function p = showPatch(obj, x, y, z, color, parent, tag)
        %showPatch Plots a patch corresponding to the sector
            p = patch(x, y, z, color, ...
                      'Parent', parent, ...
                      'FaceAlpha', obj.Alpha,...
                      'EdgeAlpha', obj.Alpha,...
                      'EdgeColor', color, ...
                      'Tag', tag);
        end

        function [c1, c2] = circularArc(obj, thetaMin, thetaMax)
        %circularArc are the points that form an arc of patch-radius from thetaMin to thetaMax
        %   The points form a sector which is visualized via a patch object.
        %   The patch's polygon is formed by joining these points with straight
        %   lines.
            c1 = obj.PatchRadius * cos(linspace(thetaMin, thetaMax, obj.NumAngularInterpolations));
            c2 = obj.PatchRadius * sin(linspace(thetaMin, thetaMax, obj.NumAngularInterpolations));
        end
    end

    methods
        function t = show(obj, parent)
        %show Visualizes the rotation patches and attaches them to an hgtransform
            t = hgtransform('Parent', parent);
            t.Tag = obj.Tag;
            zeroCoordinate = zeros(1, obj.NumAngularInterpolations);

            % Show Z
            if(abs(diff(obj.RotZ)) > 2*pi)
                [x, y] = obj.circularArc(obj.RotZ(1), obj.RotZ(1) + 2*pi);
            else
                [x, y] = obj.circularArc(obj.RotZ(1), obj.RotZ(2));
            end
            obj.showDataTipOnPatch(...
                showPatch(...
                    obj, [0, x], [0, y], [0, zeroCoordinate], obj.PatchColor{3}, t, 'Z Patch'),...
                'Z', obj.RotZ(1), obj.RotZ(2));

            % Show Y
            if(abs(diff(obj.RotY)) > 2*pi)
                [z, x] = obj.circularArc(obj.RotY(1), obj.RotY(1) + 2*pi);
            else
                [z, x] = obj.circularArc(obj.RotY(1), obj.RotY(2));
            end
            obj.showDataTipOnPatch(...
                showPatch(...
                    obj, [0, x], [0, zeroCoordinate], [0, z], obj.PatchColor{2}, t, 'Y Patch'),...
                'Y', obj.RotY(1), obj.RotY(2));

            % Show X
            if(abs(diff(obj.RotX)) > 2*pi)
                [y, z] = obj.circularArc(obj.RotX(1), obj.RotX(1) + 2*pi);
            else
                [y, z] = obj.circularArc(obj.RotX(1), obj.RotX(2));
            end
            obj.showDataTipOnPatch(...
                showPatch(...
                    obj, [0, zeroCoordinate], [0, y], [0, z], obj.PatchColor{1}, t, 'X Patch'),...
                'X', obj.RotX(1), obj.RotX(2));

            % Set the pose of the patch
            set(t, 'Matrix', obj.Pose);
        end
    end

end
