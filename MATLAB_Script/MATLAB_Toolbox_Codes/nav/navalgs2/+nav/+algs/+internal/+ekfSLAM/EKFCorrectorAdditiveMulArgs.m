classdef EKFCorrectorAdditiveMulArgs < matlabshared.tracking.internal.EKFCorrector
% This class is for internal use only. It may be removed in the future.

% EKFCorrectorAdditiveMulArgs EKF Corrector class for Additive noise with multiple arguments
% EKF Corrector class for Additive noise where multiple arguments (pose and
% landmarks) are passed to MeasurementFcn, instead of single argument (state).

%   Copyright 2021 The MathWorks, Inc.

%#codegen
    properties
        HasAdditiveNoise = true();
    end

    methods
        function [state, stateCovar] = correct(obj, ...
                                               meas, measCovar, ...
                                               state, stateCovar, ...
                                               measFcn, ...
                                               landmarkIndex)
            %CORRECT Performs the correction of state and state error covariance
            % Inputs:
            %   obj           - an EKFCorrectorAdditiveMulArgs object
            %   meas          - measurement of landmarks
            %   measCovar     - covariance of measurements
            %   state         - state estimate
            %   stateCovar    - state error covariance matrix
            %   measFcn       - function_handle to the measurement fcn
            %   landmarkIndex - indices of landmarks in state
            %
            % Outputs:
            %   state         - corrected state estimate
            %   stateCovar    - corrected state error covariance

            % Extract the associated landmarks from the State vector
            landmarks      = reshape(state(4:end),2,[])';
            landmarksAssoc = landmarks(landmarkIndex,:);

            % Get square root of state error covariance matrix
            stateCovarSqrt = ...
                        matlabshared.tracking.internal.cholPSD(stateCovar);

            % Get predicted measurement
            measPred = reshape(measFcn(state(1:3), ...
                               landmarksAssoc)',1,[])';

            % Get the Jacobian and square root of measurement covariance
            % matrix
            [H, measCovarSqrt] = obj.measurementMatrices(measCovar,  ...
                                                         measFcn, state, ...
                                                         landmarksAssoc, ...
                                                         landmarkIndex);

            % get the innovation
            innovation = meas-measPred;

            % Get covariance Pxy
            Pxy = (stateCovarSqrt*stateCovarSqrt.')* H.';

            % Compute Sy
            Sy = matlabshared.tracking.internal.qrFactor(H, ...
                                                         stateCovarSqrt, ...
                                                         measCovarSqrt);

            % Perform the Correction
            [state, stateCovarSqrt] = ...
                obj.correctStateAndSqrtCovariance(state, stateCovarSqrt, ...
                                                  innovation, Pxy, Sy, ...
                                                  H, measCovarSqrt);

            % Get the State covariance from square root of state covariance
            stateCovar = stateCovarSqrt*stateCovarSqrt';
        end
    end
    methods(Static) % Abstract methods of matlabshared.tracking.internal.EKFCorrector
        function [H, measCovarSqrt] = measurementMatrices(measCovar, ...
                                            measFcn, state, ...
                                            landmarksAssoc, landmarkIndex)
            %measurementMatrices Calculate the measurement matrices H and measCovarSqrt
            % Inputs:
            %   measCovar      - measurement noise covariance matrix
            %   measFcn        - function_handle to the measurement fcn
            %   state          - state estimate
            %   landmarksAssoc - associated landmarks from the State vector
            %   landmarkIndex  - indices of associated landmarks in the
            %                    State vector
            %
            % Outputs:
            %   H              - measurement jacobian
            %   measCovarSqrt  - square root of measurement noise 
            %                    covariance matrix

            % Get measurement Jacobian computed at associated landmark
            if nargout(measFcn) == 3
                % User provided analytical jacobian fcn
                [~, Hpose, Hlandmarks] = measFcn(state(1:3), landmarksAssoc);

                % Put together Hpose and Hlandmarks in the expected form of
                % measurement jacobian
                numLM = size(landmarksAssoc,1);
                H = zeros(2*numLM, size(state,1));
                for i = 1:numLM
                    index = 2*i + [-1,0];
                    lmPos = landmarkIndex(i)*2 + [2 3];
                    H(index,1:3) = Hpose(index,:);
                    H(index,lmPos) = Hlandmarks(index,:);
                end
            elseif nargout(measFcn) == 1
                % Use numerical perturbation to get Jacobian
                % Compute and put together measurement jacobian in the 
                % expected form
                numLM = size(landmarksAssoc,1);
                H = zeros(2*numLM, size(state,1));
                for i = 1:numLM
                    index = 2*i + [-1,0];
                    lmPos = landmarkIndex(i)*2 + [2 3];
                    H(index,1:3) = matlabshared.tracking.internal.numericJacobian( ...
                        measFcn, ...
                        {state(1:3), landmarksAssoc(i,:)}, 1);
                    H(index,lmPos) = matlabshared.tracking.internal.numericJacobian( ...
                        measFcn, ...
                        {state(1:3), landmarksAssoc(i,:)}, 2);
                end
            else
                % Error out if measurementFcn has any other number of
                % outputs
                coder.internal.error( ...
                    'nav:navalgs:ekfslam:invalidNumOutputsFromFcn', ...
                    'MeasurementFcn');
            end
            
            % Get square root of measurement noise covariance matrix
            measCovarSqrt = ...
                matlabshared.tracking.internal.cholPSD(measCovar);
        end

        function n = getNumberOfMandatoryInputs()
        % The measurement function, measFcn, must have the syntax
        %      measFcn(robotPose, landmarks)
        % robotPose and landmarks are mandatory
            n = 2;
        end
        % validate measurement output from measurementFcn
        function validateMeasurementFcn()
        end
        % validate Jacobians output from measurementFcn
        function validateMeasurementJacobianFcn
        end
    end
end
