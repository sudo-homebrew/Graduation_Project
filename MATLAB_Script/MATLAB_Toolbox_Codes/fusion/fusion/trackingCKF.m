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

%   References:

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>
%#ok<*MCSUP>

classdef trackingCKF < matlabshared.tracking.internal.CubatureKalmanFilter ...
        & matlabshared.tracking.internal.fusion.CustomDisplay ...
        & matlabshared.tracking.internal.AbstractTrackingFilter ...
        & matlabshared.tracking.internal.AbstractJPDAFilter ...
        & matlabshared.smoothers.internal.CKFSmoother
    methods
        function CKF = trackingCKF(varargin)
            %trackingCKF  The constructor to create a cubature Kalman
            %filter object.
            [smoothArgs,idx1,idx2] = trackingCKF.parseSmoothingNVPairs(varargin{:});
            filterArgs = {varargin{1:idx1-1},varargin{idx1 + 2:idx2 - 1},varargin{idx2 + 2:end}};
            CKF@matlabshared.tracking.internal.CubatureKalmanFilter(filterArgs{:});
            CKF@matlabshared.smoothers.internal.CKFSmoother(smoothArgs{:});
            
            % If user didn't specify StateTransitionFcn and MeasurementFcn,
            % use a 2-D constant velocity model as a default.
            if nargin == 0
                CKF.StateTransitionFcn = @constvel;
                CKF.MeasurementFcn = @cvmeas;
                CKF.State = zeros(4,1);
            end
        end
        
        function varargout = predict(filter,varargin)
           %
           
           % The above line is intentionally left empty to divert the help
           % text to superclass.
           
           % Predict must be called with a single object
           coder.internal.assert(numel(filter) == 1, ...
                'shared_tracking:CubatureKalmanFilter:NonScalarFilter', ...
                'cubature Kalman filter', 'predict');
            
           % Call setup on smoother, the smoother knows when to return
           % safely without doing setup multiple times. 
           setupInitialDistributions(filter);
           
           % Predict using the superclass 
           [varargout{1:nargout}] = predict@matlabshared.tracking.internal.CubatureKalmanFilter(filter,varargin{:});
           
           % Update the data for the smoother
           filter.LastCrossCov = filter.pCrossCov;
           updatePredictionData(filter);
       end
       
       function varargout = correct(filter,varargin)
           %
           
           % The above line is intentionally left empty to divert the help
           % text to superclass.
           
           % Correct using the superclass
           [varargout{1:nargout}] = correct@matlabshared.tracking.internal.CubatureKalmanFilter(filter,varargin{:});
           
           % Update the data for the smoother
           updateCorrectionData(filter);
       end
        
        %----------------------------------------------------------------------
        % Correctjpda method
        %----------------------------------------------------------------------
        function [x_corr, P_corr] = correctjpda(filter, z, jpda, varargin)
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
            
           % number of inputs should be at least 3
           narginchk(3,inf)
           % correctjpda must be called with a single object
           coder.internal.errorIf(numel(filter) > 1, ...
               'shared_tracking:CubatureKalmanFilter:NonScalarFilter', ...
               'trackingCKF', 'correctjpda');
           
           % validate some basic attributes of the measurement matrix
           validateattributes(z,{'numeric'}, ...
               {'real', 'finite', 'nonsparse','2d'},...
               'CubatureKalmanFilter', 'z');
           
           % Further validate the dimension of a measurement
           if coder.internal.is_defined(filter.pN)
               matlabshared.tracking.internal.validateInputSizeAndType...
                   ('z(:,1)', 'trackingCKF', z(:,1), filter.pN);
           else
               matlabshared.tracking.internal.validateInputSizeAndType...
                   ('z(:,1)', 'trackingCKF', z(:,1));
           end

           % Measurements z is a m by M matrix, validate a single
           % column of it using the inherited validation function:
           validateMeasurementAndRelatedProperties(filter, z(:,1),'correctjpda', varargin{:});
           
                      
           % jpda should have length of numMeasurements + 1
           numMeasurements = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(size(z,2));
           ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
           expectedNumel = numMeasurements + ONE;
           matlabshared.tracking.internal.validateInputSizeAndType...
               ('jpda','correctjpda',jpda,expectedNumel);
           
           % jpda coefficients must sum to 1
           coder.internal.errorIf(abs(sum(jpda)-1)>sqrt(eps(filter.pDataType)),...
               'shared_tracking:UnscentedKalmanFilter:InvalidJPDAInput'...
               ,'jpda')
            
            % Perform the CKF correction
            [filter.pState, filter.pStateCovariance] = ...
                filter.pCorrector.correctjpda(...
                z, ...
                jpda,...
                filter.pMeasurementNoise, ...
                filter.pState, ...
                filter.pStateCovariance, ...
                filter.constAlpha, ...
                filter.constBeta, ...
                filter.constKappa, ...
                filter.MeasurementFcn, ...
                varargin{:});
            
            filter.pHasPrediction = false;
            
            % obj.State outputs the estimated state in the orientation of
            % initial state provided by the user
            if nargout
                x_corr = filter.State;
                if nargout > 1
                    P_corr = filter.StateCovariance;
                end
            end
            
            % Update the data for the smoother
            updateCorrectionData(filter);
        end
        
        
        %-------------------------------------------------------------------
        % Distance method: cannot be inherited from superclass, because the
        % superclass doesn't have this method
        %-------------------------------------------------------------------
        function d = distance(CKF, z_matrix, measurementParams)
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
            
            % The procedure for computing the distance is described in Page
            % 93 of "Multiple-Target Tracking with Radar Applications" by
            % Samuel Blackman.

            cond = (numel(CKF) > 1);
            coder.internal.errorIf(cond, ...
                'shared_tracking:CubatureKalmanFilter:NonScalarFilter', ...
                'cubature Kalman filter', 'distance');

            if nargin == 2
                measurementParams = cell(0,0);
            end

            % Validate z_matrix. Skip the size check if it's not known yet
            % (first call to distance)
            if coder.internal.is_defined(CKF.pN)
                matlabshared.tracking.internal.validateMeasurementMatrix(...
                    z_matrix, 'CubatureKalmanFilter', CKF.pN);
            else
                matlabshared.tracking.internal.validateMeasurementMatrix(...
                    z_matrix, 'CubatureKalmanFilter');
            end

            % Validate x, P, R
            if isvector(z_matrix)
                zcol = z_matrix(:);
            else
                zcol = z_matrix(1,:).';
            end
            validateMeasurementAndRelatedProperties(CKF, zcol, 'distance', measurementParams{:});

            % Use redisual
            [nr,nc] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(z_matrix);
            ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne;
            if nc == CKF.pN
                d = zeros(1,nr,'like',z_matrix);
                for i = ONE:nr
                    [res, S] = residual(CKF, z_matrix(i,:), measurementParams);
                    d2 = res' / S * res;
                    determinant = det(S);
                    d(i) = d2 + log(determinant);
                end
            else
                d = zeros(1,nc,'like',z_matrix);
                for i = ONE:nc
                    [res, S] = residual(CKF, z_matrix(:,i), measurementParams);
                    d2 = res' / S * res;
                    determinant = det(S);
                    d(i) = d2 + log(determinant);
                end
            end
        end
        
        function [res, S] = residual(CKF, z, measurementParams)
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

            if nargin == 2
                measurementParams = cell(0,0);
            end
            
            [res, S] = residual@matlabshared.tracking.internal.CubatureKalmanFilter(CKF, z, measurementParams{:}); 

        end
        
        %-------------------------------------------------------------------
        % likelihood method: calculate the likelihood of a measurement
        %-------------------------------------------------------------------
        function l = likelihood(CKF, z, varargin)
            %LIKELIHOOD Calculate the likelihood of a measurement
            %  l = likelihood(CKF, Z) calculates the likelihood of a
            %  measurement, Z, given the object, CKF.
            %
            %  l = likelihood(CKF, Z, measurementParams) allows to
            %   define additional parameters that will be used by the
            %   CKF.MeasurementFcn. It should be specified as a cell array,
            %   e.g., {1, [2;3]}. If unspecified, it will be assumed to be
            %   an empty cell array.
            
            % Validation is done in the residual method
            [zres, S] = residual(CKF,z,varargin{:});
            l = matlabshared.tracking.internal.KalmanLikelihood(zres,S);
        end
        
        function obj2 = clone(obj)
          obj2 = clone@matlabshared.tracking.internal.CubatureKalmanFilter(obj); 
          copySmootherProperties(obj, obj2);
       end
    end
    
    methods (Access = ...
            {?matlabshared.tracking.internal.AbstractTrackingFilter, ...
            ?matlabshared.tracking.internal.AbstractContainsFilters, ...
            ?matlab.unittest.TestCase})
        %-------------------------------------------------------------------
        % nullify method: cannot be inherited from superclass, because the
        % superclass doesn't have this method
        %-------------------------------------------------------------------
        function nullify(CKF)
            % nullify: sets the State and StateCovariance to zeros
            CKF.State           = zeros(numel(CKF.State), 1, 'like', CKF.State);
            CKF.StateCovariance = eye(numel(CKF.State), numel(CKF.State), 'like', CKF.State);
        end
        
        %-------------------------------------------------------------------
        % modelName method: cannot be inherited from the superclass, because
        % the superclass does not have this method
        %-------------------------------------------------------------------
        function name = modelName(CKF)
            % modelName Return the name of the motion model
            %    name = modelName(filter) returns the motion model name,
            %    name.
            coder.internal.assert(coder.internal.is_defined(CKF.StateTransitionFcn),...
                'shared_tracking:CubatureKalmanFilter:undefinedStateTransitionFcn');
            name =  func2str(CKF.StateTransitionFcn);
        end
        
        %-------------------------------------------------------------------
        % sync method: Sync the filter with the filter provided as second
        % input.
        %-------------------------------------------------------------------
        function sync(CKF, CKF2)
            % sync: Synchronizes the CKF with CKF2 to make sure that:
            % 1. State is same
            % 2. StateCovariance is same.
            % 3. Measurement and Process Noise are same.
            
            % Validation is performed via set method of each property.
            CKF.pState = CKF2.State;
            CKF.pStateCovariance = CKF2.StateCovariance;
            CKF.pMeasurementNoise = CKF2.MeasurementNoise;
            CKF.pProcessNoise = CKF2.ProcessNoise;
        end
        
        %-------------------------------------------------------------------
        % models method: Returns the state transition and measurement models
        %-------------------------------------------------------------------
        function [stm, mm] = models(filter,~)
            % MODELS Return the state transition and measurement models
            %   [stm, mm] = MODELS(filter) returns stm, the state transition
            %   model, and mm, the measurement model. Both are returned as
            %   function handles.
            if filter.HasAdditiveProcessNoise
                stm = filter.StateTransitionFcn;
            else
                v = zeros(1,1,'like',filter.State);
                stm = @(state,dt) filter.StateTransitionFcn(state,v,dt);
            end
            
            if filter.HasAdditiveMeasurementNoise
                mm = filter.MeasurementFcn;
            else
                w = zeros(1,1,'like',filter.State);
                mm = @(state,varargin) filter.MeasurementFcn(state,w,varargin{:});
            end
        end
    end
    
    methods (Access = protected)
        % saveobj to make sure the filter is saved correctly. The
        % method is overridden to save smoother properties in addition
        % to filter properties.
        function s = saveobj(obj)
            sCKF = saveobj@matlabshared.tracking.internal.CubatureKalmanFilter(obj);
            s = saveSmootherProperties(obj, sCKF);
        end
    end
    
    methods (Static = true)
        %--------------------------------------------------------------------
        % loadobj to make sure that the filter is loaded correctly.
        % This method cannot be inherited from the superclass because the
        % correct object type has to be created. 
        function retCKF = loadobj(sobj)
            if isfield(sobj,'HasMeasurementWrapping')
                hasWrap = sobj.HasMeasurementWrapping;
            else
                hasWrap = false;
            end
            retCKF = trackingCKF ...
                ('DataType', sobj.pDataType, ...
                'HasAdditiveProcessNoise', sobj.HasAdditiveProcessNoise, ... % set pPredictor
                'HasAdditiveMeasurementNoise', sobj.HasAdditiveMeasurementNoise, ... % set pCorrector
                'HasMeasurementWrapping', hasWrap); % set measurement wrapping
            % Load the remaining properties
            loadobjHelper(retCKF,sobj);
            
            % Load smoother properties
            loadSmootherProperties(retCKF,sobj);
        end
    end
    %------------------------------------------------------------------
    % Overrides for Custom Display
    %------------------------------------------------------------------
    methods (Access='protected')
        function propGroups = getPropertyGroups(obj)
            
            propGroups = [ matlab.mixin.util.PropertyGroup( {'State', 'StateCovariance'}, ...
                ''); ...
                matlab.mixin.util.PropertyGroup( {'StateTransitionFcn', 'ProcessNoise', 'HasAdditiveProcessNoise'}, ...
                ''); ...
                matlab.mixin.util.PropertyGroup( {'MeasurementFcn', 'HasMeasurementWrapping', 'MeasurementNoise', 'HasAdditiveMeasurementNoise'}, ...
                '')];
            
            smoothGroups = getPropertyGroups@matlabshared.smoothers.internal.CKFSmoother(obj);
            propGroups = [propGroups;smoothGroups];
        end
    end
    
    %------------------------------------------------------------------
    % Overrides for Custom Display when calling get(CKF)
    %------------------------------------------------------------------
    methods (Hidden)
        function getdisp(obj)
            display(obj);
        end
    end
    
    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            propsCKF = matlabshared.tracking.internal.CubatureKalmanFilter.matlabCodegenNontunableProperties;
            propsSmoother = matlabshared.smoothers.internal.CKFSmoother.matlabCodegenNontunableProperties;
            props = {propsCKF{:} propsSmoother{:}};
        end
    end
end
