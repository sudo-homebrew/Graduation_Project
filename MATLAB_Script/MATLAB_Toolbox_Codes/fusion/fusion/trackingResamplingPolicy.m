classdef (Sealed) trackingResamplingPolicy < matlabshared.tracking.internal.ResamplingPolicy
%trackingResamplingPolicy Create resampling policy for particle filter
%   After a sensor measurement is incorporated in the particle filter
%   during the correction stage, a resampling of all particles might
%   occur.
%
%   The settings in this object determine if and when resampling should
%   occur.
%
%   policy = trackingResamplingPolicy creates a trackingResamplingPolicy 
%   object policy. Its properties can be modified to control when 
%   resampling should be triggered.
%
%   trackingResamplingPolicy properties:
%       TriggerMethod             - Method used for determining if resampling should be triggered 
%       SamplingInterval          - Fixed interval between resampling 
%       MinEffectiveParticleRatio - Minimum desired ratio of effective to total particles
%
%   Use the trackingResamplingPolicy object associated with the trackingPF
%   property ResamplingPolicy to modify these properties and control
%   resampling triggering.
%
%   See also: trackingPF. 

% Copyright 2018 The MathWorks, Inc.

%#codegen
end