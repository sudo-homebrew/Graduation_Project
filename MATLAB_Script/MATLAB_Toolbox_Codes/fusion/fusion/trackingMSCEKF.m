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

%   References:
%   [1] Stallard, David V. "Angle-only tracking filter in modified
%       spherical coordinates." Journal of guidance, control, and dynamics
%       14.3 (1991): 694-696.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

classdef trackingMSCEKF < trackingEKF
    
    properties (Dependent)
        %ObserverInput defines the acceleration or maneuver of the
        %observer in the global scenario frame. 
        ObserverInput
    end
      
    properties (Access = protected)
        pObserverInput
    end
    
    methods
       function MSCEKF = trackingMSCEKF(varargin)
           obsIndex = fusion.internal.findProp('ObserverInput',varargin{1:end});
           ekfArgs = {varargin{1:obsIndex-1},varargin{obsIndex+2:end}};
           MSCEKF@trackingEKF...
               ('StateTransitionFcn',@constvelmsc,'MeasurementFcn',@cvmeasmsc,...
               'StateTransitionJacobianFcn',@constvelmscjac,...
               'MeasurementJacobianFcn',@cvmeasmscjac,ekfArgs{:},'HasAdditiveProcessNoise',false);
           
           % Always store the internal observer input as a 6-element column 
           % vector.
           MSCEKF.pObserverInput = Inf(6,1,MSCEKF.pDataType);
           if obsIndex <= numel(varargin) - 1
               MSCEKF.ObserverInput = varargin{obsIndex+1};
           end
           if nargin == 0
               MSCEKF.State = [0;0;0;0;1;0];
               MSCEKF.StateCovariance = eye(6);
               MSCEKF.ObserverInput = zeros(3,1);
           end
           % Set the size of non-additive process noise.
           if coder.internal.is_defined(MSCEKF.pM)
               validateStateDim(MSCEKF);
               MSCEKF.pW = MSCEKF.pM/2;
           end
       end
       
       % ------------------------------------------------------------------
       % predict: Pass observer input to the predict method of
       % trackingEKF.
       % -----------------------------------------------------------------
       function [x_Pred,P_Pred] = predict(MSCEKF,dT)
           if nargin == 1
               dT = ones(1,'like',MSCEKF.State);
           end
           coder.internal.assert(~isempty(MSCEKF.ObserverInput),...
               'fusion:MSC:undefinedObserverInput');
           predict@trackingEKF(MSCEKF,dT,MSCEKF.ObserverInput);
           if nargout
               x_Pred = MSCEKF.State;
               if nargout > 1
                   P_Pred = MSCEKF.StateCovariance;
               end
           end
       end
       
       % ------------------------------------------------------------------
       % clone: Set Observer Input after cloning.
       % ------------------------------------------------------------------
       function newMSCEKF = clone(MSCEKF)
           newMSCEKF = clone@trackingEKF(MSCEKF);
           if ~isempty(MSCEKF.ObserverInput)
               newMSCEKF.ObserverInput = MSCEKF.ObserverInput;
           end
       end
    end
    
    methods (Access = ...
            {?matlabshared.tracking.internal.AbstractTrackingFilter, ...
            ?matlabshared.tracking.internal.AbstractContainsFilters, ...
            ?matlab.unittest.TestCase})
       % ------------------------------------------------------------------
       % sync: Synchronizes the filter with another filter of same type.
       % ------------------------------------------------------------------
       function sync(MSCEKF,MSCEKF2)
           % sync: Synchronizes the MSCEKF with MSCEKF2 to make sure that 
           % 1. State is same
           % 2. StateCovariance is same.
           % 3. ObserverInput is same.
           % 4. Measurement and Process Noise are same. 
           
           % Validate filter input
           % It is expected to override sync for filters derived from this
           % class.
           classToExpect = 'trackingMSCEKF';
           validateattributes(MSCEKF2,{classToExpect},{'scalar'},'trackingMSCEKF');
           
           MSCEKF.State = MSCEKF2.State;
           MSCEKF.StateCovariance = MSCEKF2.StateCovariance;
           MSCEKF.ProcessNoise = MSCEKF2.ProcessNoise;
           MSCEKF.MeasurementNoise = MSCEKF2.MeasurementNoise;
           if ~isempty(MSCEKF2.ObserverInput)
               MSCEKF.ObserverInput = MSCEKF2.ObserverInput;
           end
       end
       
       %-------------------------------------------------------------------
       % models method: Return the state transition and measurement model
       %-------------------------------------------------------------------
       function [stm, mm] = models(MSCEKF,~)
           % MODELS Return the state transition and measurement models
           %   [stm, mm] = MODELS(filter) returns stm, the state transition
           %   model, and mm, the measurement model. Both are returned as
           %   function handles.
           
           n = size(MSCEKF.ProcessNoise,1);
           v = zeros(1,1,'like',MSCEKF.State);
           stm = @(state,dt) MSCEKF.StateTransitionFcn(state,v,dt,MSCEKF.ObserverInput);
           mm  = MSCEKF.MeasurementFcn;
       end
    end
    
    methods    
        function set.ObserverInput(MSCEKF,val)
            validateattributes(val,{'numeric'},{'real','finite','vector','nonsparse'},'trackingMSCEKF');
            % Set state size using the input, if not defined yet.
            if ~coder.internal.is_defined(MSCEKF.pM)
                if any(numel(val) == [2 4])
                    stateDim = 4;
                elseif any(numel(val) == [3 6])
                    stateDim = 6;
                else
                    % Throw an error with possible choices for ObserverInput.
                    coder.internal.errorIf(true,'fusion:MSC:undefinedStateAndObserverInput');
                end
                setStateSizes(MSCEKF,stateDim,stateDim/2);
            else
                % Validate that stateDimension, if already set can only be
                % of dimension 4 or 6.
                validateStateDim(MSCEKF);
                stateDim = MSCEKF.pM;
                coder.internal.assert(any(numel(val) == [stateDim stateDim/2]),'fusion:MSC:invalidObserverInputProperty',stateDim,stateDim/2);
            end
            % Accommodate row, column orientation with maneuver for code
            % generation.
            MSCEKF.pObserverInput(1:numel(val)) = val(:);
            MSCEKF.pObserverInput(numel(val)+1:end) = Inf;
        end
        function val = get.ObserverInput(MSCEKF)
            val = MSCEKF.pObserverInput(isfinite(MSCEKF.pObserverInput));
        end
    end
    
    methods (Access='protected')
        function propGroups = getPropertyGroups(~)
                                    
            propGroups = [ matlab.mixin.util.PropertyGroup( {'State', 'StateCovariance'}, ...
                ''); ...
                matlab.mixin.util.PropertyGroup( {'StateTransitionFcn', 'StateTransitionJacobianFcn', 'ProcessNoise', 'HasAdditiveProcessNoise','ObserverInput'}, ...
                ''); ... 
                matlab.mixin.util.PropertyGroup( {'MeasurementFcn', 'MeasurementJacobianFcn', 'HasMeasurementWrapping', 'MeasurementNoise', 'HasAdditiveMeasurementNoise'}, ...
                '')];
        end
        
        function validateStateDim(MSCEKF)
            if coder.internal.is_defined(MSCEKF.pM)
                coder.internal.assert(any(MSCEKF.pM == [4 6]),'shared_tracking:motion:incorrectStateVecWithInfo','[4 6]');
            end
        end
    end
    
end


