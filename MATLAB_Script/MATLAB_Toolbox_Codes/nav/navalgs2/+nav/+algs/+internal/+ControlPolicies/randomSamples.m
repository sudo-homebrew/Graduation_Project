classdef randomSamples < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%randomSamples Propagate state with N control inputs and select best result
%
%   randomSamples randomly samples the kinematic model's 
%   control space a set number of times and performs a lightweight 
%   propagation. The input that propagates the system closest to the target
%   is returned.
%
%   See also linearPursuit, arcPursuit

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        SettableParams = {'NumSamples'};
        NumSamples = 1;
        LowerControlLim
        ControlRange
    end
    
    methods
        function [uBest, numStep] = sampleControl(obj, propagator, q, u, qTgt)
        %sampleControl Returns best control from randomly sampled finite set
            
            % Define integration times
            numStep   = propagator.MaxControlSteps;
            
            % Define sampling bounds
            uLowerLim = propagator.KinematicModelInternal.ControlLimits(:,1)';
            uRange    = propagator.KinematicModelInternal.LimitSpan;
            
            if obj.NumSamples == 1
                uBest = uLowerLim+rand(1,numel(uRange)).*uRange;
            else
                %Propagate the system and return the best result 
                integrator = propagator.odeFcn;
                times = (1:numStep)*propagator.ControlStepSize;
                
                uBest = u;
                minDist = inf;
                for i = 1:obj.NumSamples
                    % Draw sample
                    u = uLowerLim+rand(1,numel(uRange)).*uRange;

                    % Integrate
                    Q = integrator(q,u,times);

                    % Check distance against current best
                    [dist, idx] = min(propagator.distance(Q,qTgt));
                    if dist < minDist
                        uBest = u;
                        numStep = idx;
                    end
                end
            end
        end
        
        function u = sampleNext(~, ~, ~, u, ~)
        %sampleNext Holds the original control constant
        end
    end
end