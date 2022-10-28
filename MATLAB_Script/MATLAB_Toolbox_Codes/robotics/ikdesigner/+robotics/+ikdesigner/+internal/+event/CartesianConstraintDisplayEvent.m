classdef (ConstructOnLoad) CartesianConstraintDisplayEvent < robotics.ikdesigner.internal.event.ConstraintDisplayEvent
    %CartesianConstraintDisplayEvent Constraint display update

    %   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        AxesOverlayConstructorHandle = @(evt, figStateHandler)robotics.ikdesigner.internal.interactive.CartesianConstraintOverlay(evt, figStateHandler)
    end

    properties

        %TargetTransform The target pose, a 4x4 homogeneous matrix defined with respect to the reference body origin
        TargetTransform

        %Bounds Bounds, presented as a 3x2 matrix that define bounds on a target range
        Bounds

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
