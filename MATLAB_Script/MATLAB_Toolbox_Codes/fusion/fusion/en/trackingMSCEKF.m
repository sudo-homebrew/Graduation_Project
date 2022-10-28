classdef trackingMSCEKF< trackingEKF
%trackingMSCEKF An extended Kalman Filter for object tracking in MSC Frame.
%   The MSC-EKF filter is designed to use an extended kalman filter for
%   object tracking in modified spherical coordinates (MSC) using
%   angle-only measurements from a single observer. You can use the filter
%   to predict an object's future location in MSC frame, or to help
%   associate multiple object detections with their tracks. You can specify
%   the observer maneuver or acceleration required by state-transition
%   functions (@constvelmsc and @constvelmscjac) by using the property
%   ObserverInput.
%
%   MSCEKF = trackingMSCEKF returns an extended kalman filter to use the
%   MSC state-transition and measurement functions with the trackers. The
%   default state is defined as [0;0;0;0;1;0], which implies a static
%   target at 1 meters from the observer at zero azimuth and elevation.
%
%   The following properties are fixed for the MSCEKF:
%   StateTransitionFcn          - @constvelmsc
%   StateTransitionJacobianFcn  - @constvelmscjac
%   MeasurementFcn              - @cvmeasmsc
%   MeasurementJacobianFcn      - @cvmeasmscjac
%   HasAdditiveProcessNoise     - false
%   HasAdditiveMeasurementNoise - true
%
%   MSCEKF = trackingMSCEKF(Name, Value) allows you to specify the
%   properties of the extended kalman filter, which are not specified
%   above.
%
%   trackingMSCEKF properties:
%   State                       - State, (x)
%   StateCovariance             - State estimation error covariance, (P)
%   StateTransitionFcn          - Propagates the state to next time step, (f)
%   StateTransitionJacobianFcn  - State transition jacobian matrix, (df/dx)
%   ProcessNoise                - Process noise covariance, (Q)
%   ObserverInput               - acceleration or maneuver of observer
%   HasAdditiveProcessNoise     - True if process noise is additive
%   MeasurementFcn              - Calculates the measurement, (h)
%   MeasurementJacobianFcn      - Measurement jacobian matrix, (dh/dx)
%   MeasurementNoise            - Measurement noise covariance, (R)
%   HasAdditiveMeasurementNoise - True if measurement noise is additive
%   HasMeasurementWrapping      - True if the measurement wraps (read only)
%
%   trackingMSCEKF methods:
%   predict     - Predict the state and state estimation error covariance
%   correct     - Correct the state and state estimation error covariance
%   correctjpda - Correct using joint probabilistic detection assignment
%   distance    - Calculate the distance between measurements and the filter
%   residual    - Calculate the measurement residual and residual noise
%   likelihood  - Calculate the likelihood of a measurement
%   clone       - Create a copy of the object with the same property values
%   initialize  - Initialize filter properties
%
%   % Example 1: Create a MSC-EKF for a 2-D motion model
%   % --------------------------------------------------
%   % Set the estimates for MSC frame.
%   az = 0.1; azRate = 0; r = 1000; rDot = 10;
%   MSCEKF = trackingMSCEKF('State',[az;azRate;1/r;rDot/r]);
%
%   % Example 2: Create and run a MSC-EKF for a 3-D motion model
%   % --------------------------------------------------
%   az = 0.1; azRate = 0; r = 1000; rDot = 10;
%   el = 0.3; elRate = 0;
%   omega = azRate*cos(el);
%   MSCEKF2 = trackingMSCEKF('State',[az;omega;el;elRate;1/r;rDot/r]);
%   % predict the filter using constant observer acceleration
%   MSCEKF2.ObserverInput = [1;2;3];
%   predict(MSCEKF2); % Default time 1 second.
%   predict(MSCEKF2,0.1); % Predict using dt = 0.1 second.
%   % Correct using angle-only measurement
%   meas = [5;18]; %degrees
%   correct(MSCEKF2,meas);
%
%   See also: trackingEKF, initcvmscekf, constvelmsc, constvelmscjac,
%   cvmeasmsc, cvmeasmscjac

 
%   Copyright 2018 The MathWorks, Inc.

    methods
        function out=trackingMSCEKF
        end

        function out=clone(~) %#ok<STOUT>
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=predict(~) %#ok<STOUT>
        end

        function out=validateStateDim(~) %#ok<STOUT>
        end

    end
    properties
        %ObserverInput defines the acceleration or maneuver of the
        %observer in the global scenario frame. 
        ObserverInput;

        pObserverInput;

    end
end
