classdef trackingGSF< matlabshared.tracking.internal.fusion.multipleModelSystem
%trackingGSF A Gaussian-sum filter for object tracking
%   The Gaussian-sum filter defines the state probability density function
%   by a set of finite sum of Gaussian components. This filter can be used
%   for tracking objects which require a multi-model description due to
%   incomplete observability of state through measurements. For example,
%   this filter can be used as a range-parametrized extended kalman filter,
%   when the detection contains angle-only measurements.
%
%   GSF = trackingGSF returns a Gaussian-sum filter with two constant
%   velocity Extended Kalman Filters (trackingEKF), each with equal
%   initial probability or weight.
%
%   GSF = trackingGSF(trackingFilters) allows specifying the Gaussian
%   components of the filter. trackingFilters must be a cell array of
%   tracking filters, for example, trackingUKF, and must define the state
%   of the object with same size. The initial probability or weights of the
%   filters are assumed equal.
%
%   GSF = trackingGSF(trackingFilters,modelProbabilities) allows specifying
%   the initial probability or weight of the Gaussian components.
%
%   GSF = trackingGSF(..,'MeasurementNoise',measNoise) allows specifying the
%   measurement noise of the filter. The MeasurementNoise for each
%   Gaussian component is set as measNoise. The size of measNoise must
%   be same as size of MeasurementNoise of each TrackingFilter.
%
%   trackingGSF properties:
%   State                   - (read-only) weighted estimate of state
%   StateCovariance         - (read-only) weighted estimate of state-covariance
%   TrackingFilters         - (read-only) list of filters used in the Gaussian-sum
%   HasMeasurementWrapping  - (read only) Flags to show if TrackingFilters use wrapping
%   ModelProbabilities      - Weight of each filter used in the Gaussian-sum
%   MeasurementNoise        - Measurement noise covariance of the filter
%
%   trackingGSF methods:
%   predict    - Predict the state and state estimation error covariance
%   correct    - Correct the state and state estimation error covariance
%   distance   - Calculate the distance between measurements and the filter
%   likelihood - Calculate the likelihood of a measurement
%   clone      - Create a copy of the object with the same property values
%
%   Notes:
%   ------
%   * If the measurement exists, e.g., the object has been detected, you
%     can call the predict method and the correct method together. If the
%     measurement is missing, you can call the predict method but not the
%     correct method.
%   * You can use the distance method to compute distances that describe
%     how a set of measurements matches the filter. You can thus find a
%     measurement that best fits the filter. This strategy can be
%     used for matching object detections against object tracks in a
%     multi-object tracking problem.
%
%   % Example:
%   % --------
%   % Creating a trackingGSF
%   filters = cell(3,1);
%   % Create three EKFs each with state distributed around [0;0;0;0;0;0]
%   % and running on position measurements.
%   filter{1} = trackingEKF(@constvel,@cvmeas,rand(6,1),'MeasurementNoise',eye(3));
%   filter{2} = trackingEKF(@constvel,@cvmeas,rand(6,1),'MeasurementNoise',eye(3));
%   filter{3} = trackingEKF(@constvel,@cvmeas,rand(6,1),'MeasurementNoise',eye(3));
%   gsf = trackingGSF(filter);
%
%   % Running the filter
%   % predict the state after 0.1 seconds
%   [x_pred, P_pred] = predict(gsf,0.1);
%   % correct the state with measurement [0.5;0.2;0.3];
%   [x_corr, P_corr] = correct(gsf,[0.5;0.2;0.3]);
%   % compute the distance between the filter and a measurement
%   d = distance(gsf,[0;0;0]);
%
%   See also: trackingEKF, trackingUKF, trackingCKF, trackingPF, 
%   trackingMSCEKF, initrpekf, initapekf.

 
%   Copyright 2018 The MathWorks, Inc.

    methods
        function out=trackingGSF
        end

        function out=getPropertyGroups(~) %#ok<STOUT>
        end

        function out=parseInputs(~) %#ok<STOUT>
        end

        function out=validateFilterInput(~) %#ok<STOUT>
        end

    end
end
