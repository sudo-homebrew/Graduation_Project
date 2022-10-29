classdef WGRVisual
%This class is for internal use only. It may be removed in the future.
%
%WGRVisual Visual of a workspaceGoalRegion.
%   The visualization comprises of visualizing the following elements in a
%   single view of the workspaceGoalRegion:
%       1) Reference frame
%       2) End-Effector offset frame
%       3) Rotation bounds
%       4) Translation bounds
%   Each visual element is overlayed on a single axes to visualize the workspace
%   goal region.
%   This visual object is responsible for constructing and updating its
%   composite visual elements.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties
        %Bounds
        Bounds

        %ReferencePose
        ReferencePose

        %EndEffectorOffsetPose
        EndEffectorOffsetPose
    end

    properties(Access=private)
        %EndEffectorOffsetFrameVisual The end-effector offset frame visual
        %   This frame is a FrameVisual made up of orthogonal line objects, and
        %   is slightly shorter than the reference frame's visual.
        EndEffectorOffsetFrameVisual

        %ReferenceFrameVisual The reference frame visual
        %   Similar to the end-effector offset frame, this is a FrameVisual.
        ReferenceFrameVisual

        %TranslationBoundsVisual The translation bounds visual
        %   This visual is similar to a bounding box. The bounds of this visual
        %   correspond to the translation bounds of the workspaceGoalRegion.
        TranslationBoundsVisual

        %RotationBoundsVisual The rotation bounds visual
        %   This visual is a set of three circular sectors/patches along each of
        %   the axes. It represents the rotation bounds of the
        %   workspaceGoalRegion.
        RotationBoundsVisual
    end

    properties(Constant)
        %ScalingFactor The scaling factor of the visual with respect to the translation bounds
        ScalingFactor = 0.1

        %DefaultPatchRadius Default patch radius of the rotation bounds visual
        DefaultPatchRadius = 0.075

        %DefaultFrameSize Default size of the frame visual
        DefaultFrameSize = [0.075, 0.075, 0.075]

        %EndEffectorOffsetFrameSize Size of the end-effector offset frame
        EndEffectorOffsetFrameSize = [0.055, 0.055, 0.055]
    end

    methods
        function obj = WGRVisual()
        %WGRVisual Constructor
            obj.Bounds = zeros(6, 2);
            obj.ReferencePose = eye(4);
            obj.EndEffectorOffsetPose = eye(4);

            % Create the two frame visuals - reference frame and end-effector offset
            % frame
            obj.EndEffectorOffsetFrameVisual = ...
                robotics.manip.internal.workspaceGoalRegion.FrameVisual();
            obj.ReferenceFrameVisual = ...
                robotics.manip.internal.workspaceGoalRegion.FrameVisual();

            % Create a translation bounds and a rotation bounds visual
            obj.TranslationBoundsVisual = ...
                robotics.manip.internal.workspaceGoalRegion.TranslationBoundsVisual();
            obj.RotationBoundsVisual = ...
                robotics.manip.internal.workspaceGoalRegion.RotationBoundsVisual();

            % Configure the frame visuals with the right names and other visual
            % properties
            obj.ReferenceFrameVisual.Name = "ReferenceFrame";
            obj.EndEffectorOffsetFrameVisual.Name = "EndEffectorOffsetFrame";

            % Lengths and alpha value of the end-effector offset frame axes
            obj.EndEffectorOffsetFrameVisual.FrameSize = obj.EndEffectorOffsetFrameSize;
            obj.EndEffectorOffsetFrameVisual.AxisSpec = ':';
        end

        function show(obj, parent)
        %show show the visual on a parent axes
            % Before visualizing update the visuals with the correct bounds and
            % poses
            obj = updateVisuals(obj);

            show(obj.TranslationBoundsVisual, parent);
            show(obj.RotationBoundsVisual, parent);
            show(obj.EndEffectorOffsetFrameVisual, parent);
            show(obj.ReferenceFrameVisual, parent);
        end
    end

    methods(Access=private)
        function obj = updateVisualSizes(obj)
        %updateVisualSizes Update the size of the visuals
        %   The visuals have different measures for size. Here, scale the sizes
        %   based on the maximum translation bounds.
            maxTranslationBoundDiff = max(diff(obj.Bounds(1:3, :), 1, 2));
            obj.RotationBoundsVisual.PatchRadius = ...
                obj.DefaultPatchRadius + obj.ScalingFactor * maxTranslationBoundDiff;
            obj.EndEffectorOffsetFrameVisual.FrameSize = ...
                obj.EndEffectorOffsetFrameSize + obj.ScalingFactor * maxTranslationBoundDiff;
            obj.ReferenceFrameVisual.FrameSize = ...
                obj.DefaultFrameSize + obj.ScalingFactor * maxTranslationBoundDiff;
        end

        function obj = updateVisualPoses(obj)
        %updateVisualPoses Update visuals based on current poses
        %   The frames must reflect the state of the current reference pose and
        %   end-effector offset pose
            obj.TranslationBoundsVisual.Pose = obj.ReferencePose;
            obj.RotationBoundsVisual.Pose = obj.ReferencePose;
            obj.ReferenceFrameVisual.Pose = obj.ReferencePose;

            % Assign end-effector offset pose to the end-effector offset frame
            % visual. Note that the Pose of the visual is in the world frame
            obj.EndEffectorOffsetFrameVisual.Pose = ...
                obj.ReferencePose * obj.EndEffectorOffsetPose;
        end

        function obj = updateVisualBounds(obj)
        %updateVisualBounds Update visuals based on current bounds
        %   Assign the current bounds to the translation and rotation bounds
        %   visuals
            obj.RotationBoundsVisual.RotZ = obj.Bounds(4, :);
            obj.RotationBoundsVisual.RotY = obj.Bounds(5, :);
            obj.RotationBoundsVisual.RotX = obj.Bounds(6, :);
            obj.TranslationBoundsVisual.TrX = obj.Bounds(1, :);
            obj.TranslationBoundsVisual.TrY = obj.Bounds(2, :);
            obj.TranslationBoundsVisual.TrZ = obj.Bounds(3, :);
        end

        function obj = updateVisuals(obj)
        %updateVisuals Update visuals based on current state of workspace goal region
            obj = updateVisualPoses(obj);
            obj = updateVisualSizes(obj);
            obj = updateVisualBounds(obj);
        end
    end
end
