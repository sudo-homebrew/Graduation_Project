function validConfig = applyJointLimits(inputConfig, jointLimits, isJointRevolute)
%applyJointLimits Convert solutions with invalid joint limits to NaNs
%   Given an N-element configuration, an Nx2 set of lower and upper joint
%   limits, and an N-element vector indicating the joint type (revolute or
%   prismatic), this function checks whether the configuration is within
%   the joint limits. If not, the configuration is converted to NaNs.
 
%   Copyright 2020-2021 The MathWorks, Inc.
 
% Initialize output
validConfig = inputConfig;
 
for i = 1:numel(inputConfig)
    if jointLimits(i,1) > inputConfig(i) || jointLimits(i,2) < inputConfig(i)
        
        % Compute the offset from the lower joint limit and compare that to
        % the total range
        wrappedJointValueOffset = robotics.internal.wrapTo2Pi(inputConfig(i) - jointLimits(i,1));
        
        % If the wrapped value is 2*pi, make sure it is instead registered
        % as zero to ensure this doesn't fall outside the range
        if isEqualWithinTolerance(wrappedJointValueOffset, 2*pi)
            wrappedJointValueOffset = 0;
        end
        
        jointRange = jointLimits(i,2) - jointLimits(i,1);
        
        if isJointRevolute(i) && ((wrappedJointValueOffset < jointRange) || isEqualWithinTolerance(wrappedJointValueOffset, jointRange))
            
            % Make sure the final value is definitively inside the joint
            % limits if it was on the bound
            wrappedJointValueOffset = min(wrappedJointValueOffset, jointRange);
            
            % Update the configuration
            validConfig(i) = jointLimits(i,1) + wrappedJointValueOffset;
        else
            % If any element is NaN, the whole array will be thrown out so
            % there is no need to continue
            validConfig = nan(size(validConfig));
            return;
        end
    end
end
   
end
