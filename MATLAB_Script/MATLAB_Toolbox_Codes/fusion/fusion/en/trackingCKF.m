classdef trackingCKF< matlabshared.tracking.internal.CubatureKalmanFilter & matlabshared.tracking.internal.fusion.CustomDisplay & matlabshared.tracking.internal.AbstractTrackingFilter & matlabshared.tracking.internal.AbstractJPDAFilter & matlabshared.smoothers.internal.CKFSmoother
%trackingCKF   An cubature Kalman filter for object tracking
%   The cubature Kalman filter is designed for tracking objects that
%   follow a nonlinear motion model or that are measured by a nonlinear
%   measurement model. You can use the filter to predict an object's future
%   location, to reduce noise in the detected location, or to help
%   associate multiple object detections with their tracks.
%
%   The cubature Kalman filter estimates the uncertainty about the
%   state, and its propagation through the nonlinear state and
%   measurement equations, using a fixed number of cubature points.
%
%   The cubature points are chosen based on the spherical-radial
%   cubature transformation to guarantee an exact approximation of a
%   Gaussian distribution up to the third moment. As a result, the
%   corresponding cubature Kalman filter is the same as an unscented
%   Kalman filter with parameters Alpha = 1 and Beta = Kappa = 0.
%
%   CKF = trackingCKF returns a cubature Kalman filter object with state
%   transition function (@constvel), measurement function (@cvmeas) and
%   state ([0;0;0;0]), and assumes an additive noise model.
%
%   CKF = trackingCKF(StateTransitionFcn, MeasurementFcn, State) lets you
%   specify the state transition function, f, and the measurement function,
%   h. Both must be specified as function_handles. In addition, the object
%   lets you specify an initial value for the state.
%
%   CKF = trackingCKF(..., Name, Value) specifies additional
%   name-value pair arguments to define the properties described below.
%
%   trackingCKF properties:
%   State                       - State, (x)
%   StateCovariance             - State estimation error covariance, (P)
%   StateTransitionFcn          - Propagates the state to next time step, (f)
%   ProcessNoise                - Process noise covariance, (Q)
%   HasAdditiveProcessNoise     - True if process noise is additive
%   MeasurementFcn              - Calculates the measurement, (h)
%   HasMeasurementWrapping      - True if the measurement wraps (read only)
%   MeasurementNoise            - Measurement noise covariance, (R)
%   HasAdditiveMeasurementNoise - True if measurement noise is additive
%   EnableSmoothing             - Enable backward smoothing for the filter
%   MaxNumSmoothingSteps        - Maximum number of backward smoothing steps
%                                 allowed for smoothing
%
%   trackingCKF methods:
%   predict     - Predict the state and state estimation error covariance
%   correct     - Correct the state and state estimation error covariance
%   correctjpda - Correct using joint probabilistic detection assignment
%   smooth      - Smooth the filter estimate using backward recursion
%   distance    - Calculate the distance between measurements and the filter
%   residual    - Calculate the measurement residual and residual noise
%   likelihood  - Calculate the likelihood of a measurement
%   clone       - Create a copy of the object with the same property values
%
%   Notes:
%   ------
%   * If the measurement exists, e.g., the object has been detected, you
%     can call the predict method and the correct method together. If the
%     measurement is missing, you can call the predict method but not the
%     correct method.
%   * You can use the distance method to compute distances that describe
%     how a set of measurements matches the Kalman filter. You can thus
%     find a measurement that best fits the filter. This strategy can be
%     used for matching object detections against object tracks in a
%     multi-object tracking problem.
%
%   % Example 1: Create a trackingCKF for a 2D constant velocity model
%   % ----------------------------------------------------------------
%   %   Use the pre-packaged constant velocity motion model, constvel, and
%   %   the associated measurement model, cvmeas. These models assume that
%   %   the state is [x;vx;y;vy] and a position measurement is [x;y;z].
%   state = [0;0;0;0];
%   CKF = trackingCKF(@constvel, @cvmeas, state);
%
%   % Example 2: Create a trackingCKF with an additional name-value pair
%   % ------------------------------------------------------------------
%   %   Use an additional name-value pair to define the Cubature
%   %   Transformation parameter alpha.
%   state = [0;0;0;0];
%   CKF = trackingCKF(@constvel, @cvmeas, state, 'ProcessNoise', diag([1;2;3;4]));
%
%   % Example 3: Running the filter
%   % -----------------------------
%   %   After constructing a filter, use the following steps to call
%   %   predict and correct. You may call predict and correct in any order
%   %   and as many times you would like to call each.
%   state = [0;0;0;0];
%   CKF = trackingCKF(@constvel, @cvmeas, state);
%   meas = [1;1;0];
%   [x_pred, P_pred] = predict(CKF, 0.5)    % Predict over 0.5 seconds
%   [x_corr, P_corr] = correct(CKF, meas)   % Correct using the measurement
%   [x_pred, P_pred] = predict(CKF)         % Predict over 1 second
%   [x_pred, P_pred] = predict(CKF, 2)      % Predict over 2 seconds
%
%   % Example 4: Using the distance method
%   % ------------------------------------
%   %   You can measure the distance between the filter and a set of
%   %   measurements, z_matrix, by calling the distance method.
%   state = [0;0;0;0];
%   CKF = trackingCKF(@constvel, @cvmeas, state, 'ProcessNoise', diag([1;2;3;4]));
%   [x_pred, P_pred] = predict(CKF);
%   z_matrix = [0, 0, 0; 1, 1, 0; 2, 2, 0];
%   d = distance(CKF, z_matrix)
%
%  See also: trackingKF, trackingEKF, trackingUKF, constvel, cvmeas

 
%   Copyright 2017-2019 The MathWorks, Inc.

    methods
        function out=trackingCKF
            %trackingCKF  The constructor to create a cubature Kalman
            %filter object.
        end

        function out=clone(~) %#ok<STOUT>
        end

        function out=correct(~) %#ok<STOUT>
        end

        function out=correctjpda(~) %#ok<STOUT>
            %CORRECTJPDA Correct using joint probabilistic detection assignment
            %   Corrects the state and state error covariance with
            %   a set of measurements and their probabilistic data association 
            %   coefficients.
            %
            %   [x_corr, P_corr] = CORRECTJPDA(filter, z, jpda) returns the
            %   correction of state, x_corr, and state estimation error
            %   covariance, P_corr, based on the current set of measurements
            %   z and their joint probabilistic data association
            %   coefficients jpda.
            %
            %   Inputs:
            %           - filter    filter of the class trackingCKF
            %
            %           - z         measurements matrix of size m-M  where m
            %                       is the dimension of a measurement and M
            %                       is the number of measurements.
            %
            %           - jpda     M+1 vector of joint probabilities. 
            %                      For i=1:M, jpda(i) is the joint 
            %                      probability of measurement i to be 
            %                      associated with the filter. jpda(M+1) is
            %                      the probability that no measurement is
            %                      associated to the filter. correctjpda
            %                      expects sum(jpda) to be equal to 1.
            %
            %   [x_corr, P_corr] = CORRECTJPDA(filter, z, jpda, varargin) 
            %   additionally, allows the definition of parameters used by
            %   the MeasurementFcn in addition to obj.State. For example,
            %   the sensor's location.
        end

        function out=distance(~) %#ok<STOUT>
            % distance Computes distances between measurements and the
            % cubature Kalman filter object.
            %   d = distance(CKF, z_matrix) computes a distance between one
            %   or more measurements supplied by the z_matrix and the
            %   measurement predicted by the cubature Kalman filter object.
            %   This computation takes into account the covariance of the
            %   predicted state and the measurement noise. Each row of the
            %   input z_matrix must contain a measurement vector of length
            %   N.
            %
            %   d = distance(CKF, z_matrix, measurementParams) allows to
            %   define additional parameters that will be used by the
            %   CKF.MeasurementFcn. It should be specified as a cell array,
            %   e.g., {1, [2;3]}. If unspecified, it will be assumed to be
            %   an empty cell array.
            %
            %   The distance method returns a row vector where each element
            %   is a distance associated with the corresponding measurement
            %   input.
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=likelihood(~) %#ok<STOUT>
            %LIKELIHOOD Calculate the likelihood of a measurement
            %  l = likelihood(CKF, Z) calculates the likelihood of a
            %  measurement, Z, given the object, CKF.
            %
            %  l = likelihood(CKF, Z, measurementParams) allows to
            %   define additional parameters that will be used by the
            %   CKF.MeasurementFcn. It should be specified as a cell array,
            %   e.g., {1, [2;3]}. If unspecified, it will be assumed to be
            %   an empty cell array.
        end

        function out=loadobj(~) %#ok<STOUT>
        end

        function out=predict(~) %#ok<STOUT>
        end

        function out=residual(~) %#ok<STOUT>
            % RESIDUAL Computes the residual of measurement z and the
            % filter.
            %   [res, S] = RESIDUAL(CKF, z) computes a residual, res, and
            %   the residual matrix, S, where:
            %       res = z-h(CKF.State), h is the measurement function
            %       S = Rp+R, where Rp is the state covariance matrix
            %       projected onto the measurement space using the cubature
            %       transformation.
            %
            %   [...] = RESIDUAL(CKF, z, measurementParams) allows
            %   passing additional parameters that will be used by the
            %   CKF.MeasurementFcn. It should be specified as a cell array,
            %   e.g., {1, [2;3]}. If unspecified, it will be assumed to be
            %   an empty cell array.
        end

        function out=saveobj(~) %#ok<STOUT>
        end

    end
end
