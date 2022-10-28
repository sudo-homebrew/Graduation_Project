classdef euler < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%euler System integrator using Euler method

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        StepSize = 0.05;
        SettableParams = {'StepSize'};
    end
    
    methods
        function fcn = getIntegrator(obj, derivFcn, updateFcn)
            dt = obj.StepSize;
            fcn = @(q,u,T)obj.integrate(derivFcn,updateFcn,dt,q,u,T);
        end
    end
    
    methods (Static)
        function qNew = integrate(derivFcn, upFcn, stepSz, q, u, T)
        %integrate Propagate motion using Euler integration
            
            % Allocate output
            qNew = zeros(numel(T),numel(q));
            tIdx = 1;
            hTot = 0;
            h = stepSz;
            
            % Integrate to end value
            while true
                % Calc derivative
                dq = derivFcn(q,u);
                if hTot+h >= T(tIdx)
                    % Record state
                    qNew(tIdx,:) = upFcn(q,(T(tIdx)-hTot)*derivFcn(q,u));
                    if hTot+h >= T(end)
                        return;
                    end
                    tIdx = tIdx+1;
                end
                q = upFcn(q,stepSz.*dq);
                hTot = hTot+h;
            end
        end
    end
end
