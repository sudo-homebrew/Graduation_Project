classdef (ConstructOnLoad) PoseConstraintDisplayEvent < robotics.ikdesigner.internal.event.ConstraintDisplayEvent
    %PoseConstraintDisplayEvent Constraint display update

    %   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        AxesOverlayConstructorHandle = @(evt, figStateHandler)robotics.ikdesigner.internal.interactive.PoseConstraintOverlay(evt, figStateHandler)
    end

    properties

        %TargetTransform The target pose, a 4x4 homogeneous matrix defined with respect to the reference body origin
        TargetTransform

        %EEBodyKey The scene objects map key corresponding to the end effector body
        EEBodyKey

        %RefBodyKey The scene objects map key corresponding to the reference body
        RefBodyKey

        %UpdateType An enumeration indicating the update type
        UpdateType
    end

    properties (SetAccess = {?robotics.ikdesigner.internal.model.Model, ?matlab.unittest.TestCase})
        % The following properties are obtained from the model

        %TFTree The cell array of transform trees indicating the current configuration state of the robot
        TFTree
    end
end
