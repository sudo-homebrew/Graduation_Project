classdef SecondOrderSysHelper
    % This class is for internal use only and may be removed in a future release
    
    % SECONDORDERSYSHELPER Helper class for relating second-order system ODE parameters with its unit step response
    
    % Copyright 2019 The MathWorks, Inc.
    
    methods (Static)
        
        function [zeta, wn] = getSysParamsFromStepParams(settlingTime, overshoot, varargin)
            %GETSECONDORDERPARAMSFROMSTEP Compute second-order ODE parameters from step response parameters
            %   Given the overshoot to a unit step and the 2% settling time, compute
            %   the natural frequency and damping ratio of the associated second order
            %   response. Overshoot is defined as the fractional overshoot of a unit
            %   step, i.e. 0.01 indicates 1% overshoot. Settling time is defined in
            %   seconds, and natural frequency is given in rad/s. The damping ratio is
            %   a unitless positive quantity.
            
            % The third optional input is used to specify the damping ratio
            % value to use when there is zero overshoot. This is used in
            % Simulink masks to ensure parity between the two methods of
            % updating the second-order parameters. The default simplifying
            % assumption is that a system with zero overshoot is critically
            % damped.
            if nargin > 2
                zeroOvershootZeta = varargin{1};
            else
                zeroOvershootZeta = 1;
            end
            
            % Validate inputs
            validateattributes(settlingTime, {'double'}, {'nonempty','scalar','finite','real','positive'}, 'getSysParamsFromStepParams', 'settlingTime');
            validateattributes(overshoot, {'double'}, {'nonempty','scalar','finite','real','nonnegative','<',1}, 'getSysParamsFromStepParams', 'overshoot');
            
            if overshoot > 0
                % Underdamped case: The damping ratio is computed from the
                % envelope, which has a maximum value of 1+exp(-zeta*wn*T)
                zeta = sqrt(log(overshoot)^2/(log(overshoot)^2 + pi^2));
                
            else
                % If overshoot is zero, assume critical damping (zeta = 1)
                % unless a different value is specified.
                zeta = zeroOvershootZeta;
                
            end
            
            wn = robotics.manip.internal.SecondOrderSysHelper.computeDampingRatioDependentParams(zeta, [], settlingTime);
        end
        
        function [overshoot, settlingTime] = getStepParamsFromSysParams(zeta, omega)
            %updateStepFromErrorParameters Update step response parameters from damping & natural frequency
            
            if zeta < 1
                % Underdamped system
                overshoot = exp(-(zeta/sqrt(1-zeta^2))*pi);
            else
                % Critically damped or overdamped system
                overshoot = 0;
            end
            
            % Compute settling time using 2% criterion
            settlingTime = robotics.manip.internal.SecondOrderSysHelper.computeDampingRatioDependentParams(zeta, omega, []);
        end
        
        
        function outputVal = computeDampingRatioDependentParams(zeta, omega, settlingTime)
            %computeDampingRatioDependentParams Compute omega or settling time
            %   This method accepts the damping ratio, ZETA, as well as one
            %   of either OMEGA or SETTLINGTIME. The other value is
            %   provided as an empty input. This method then computes the
            %   un-provided input and provides it as the output value.
            
            % This method computes the product omega*T for different cases
            % of the damping ratio, zeta
            if zeta < 1
                % Underdamped case: The 2% criterion is derived from the
                % envelope, which has a maximum value of
                % 1-exp(-zeta*wn*T)/sqrt(1-zeta^2). The settling time
                % occurs when exp(-zeta*wn*T)/sqrt(1-zeta^2) <= 0.02. A
                % precise 2% time would be omegaT =
                % ln(0.02*sqrt(1-zeta^2)/zeta. However, it is common
                % practice to use the approximation wn*T = 4/zeta for the
                % underdamped 2% criterion.
                omegaT = 4/zeta;
                
            elseif zeta == 1
                % Critically damped case: The 2% criterion is found by
                % solving from the time response: exp(-wn*T)(1+wn*T) < 0.02
                % for wn numerically. This returns wn*T > 5.83.
                omegaT = 5.9/zeta;
                
            else
                % Overdamped system that must be numerically resolved by
                % solving for the point at which a unit step crosses 0.98.
                E1 = -(zeta + sqrt(zeta^2-1));
                E2 = -(zeta - sqrt(zeta^2-1));
                C1 = 1/(2*sqrt(zeta^2-1)*(zeta + sqrt(zeta^2-1)));
                C2 = -1/(2*sqrt(zeta^2-1)*(zeta - sqrt(zeta^2-1)));
                f = @(x)(.02+C1*exp(E1*x)+C2*exp(E2*x));
                
                % The second exponential dominates this equation and can be
                % used to find a guess at the zero value and to detect
                % problem cases.
                omegaTGuess = log(-0.02/C2)/E2;
                if isinf(omegaTGuess)
                    coder.internal.error('robotics:robotmanip:motionmodels:DampingRatioTooLarge');
                end
                
                % Define an interval over which to solve. Use an empirical
                % upper bound that is at least two orders of magnitude
                % larger than the damping ratio
                omegaT = fzero(f, [0 omegaTGuess*4]);
            end
            
            if isempty(settlingTime)
                outputVal = omegaT/omega;          %Return settling time
            else
                outputVal = omegaT/settlingTime;   %Return natural frequency
            end
            
        end
        
        
    end
    
end