function isCompatible = isMultiModelCompatible(filter)
% This is an internal function and may be modified in a future release.
% isCompatible = isMultiModelCompatible(filter)
% returns two outputs.
% isCompatible is a 2 element logical vector.
% isCompatible(1) = true means the filter is compatible to be used as a
% TrackingFilter in trackingGSF.
% isCompatible(2) = true means the filter is compatible to be used as a 
% TrackingFilter in trackingIMM.

% Copyright 2018 The Mathworks, Inc

%#codegen

% list of filters compatible in both filters
bothCompatible = {'trackingKF',...
                  'trackingABF',...
                  'trackingEKF',...
                  'trackingUKF',...
                  'trackingCKF',...
                  'trackingPF'};

isCompatibleInGSF = false;
for i = 1:numel(bothCompatible)
    if isa(filter,bothCompatible{i})
        isCompatibleInGSF = true;
        break;
    end
end

% trackingMSCEKF is inherited from trackingEKF and hence it will pass
% isa(filter,'trackingEKF') test. However, it is not compatible with
% trackingIMM, but is compatible with trackingGSF.
isCompatibleInIMM = isCompatibleInGSF && ~isa(filter,'trackingMSCEKF');
isCompatible = [isCompatibleInGSF isCompatibleInIMM];
