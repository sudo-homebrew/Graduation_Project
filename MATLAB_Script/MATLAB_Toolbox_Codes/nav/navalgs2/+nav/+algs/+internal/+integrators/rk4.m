classdef rk4 < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%rk4 System integrator using RungeKutta4 method

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
        function qNew = integrate(derivFcn, upFcn, stepSz, q0, u, T)
        %integrate Propagate motion using RungeKutta4 integration
            
            % Allocate output
            qNew = zeros(numel(T),numel(q0));
            tIdx = 1;
            hTot = 0;
            q = q0;
            
            % Integrate to end value
            while true
                % Calc derivative
                if hTot+stepSz >= T(tIdx)
                    % Record state
                    h = (T(tIdx)-hTot);
                    k1 = h .* derivFcn(q,         u);
                    k2 = h .* derivFcn(q+1/2.*k1, u);
                    k3 = h .* derivFcn(q+1/2.*k2, u);
                    k4 = h .* derivFcn(q+k3     , u);
                    dq = 1/6.*k1 + 1/3.*k2 + 1/3.*k3 + 1/6.*k4;
                    qNew(tIdx,:) = upFcn(q,dq);
                    if hTot+stepSz >= T(end)
                        return;
                    end
                    tIdx = tIdx+1;
                end
                k1 = stepSz .* derivFcn(q,         u);
                k2 = stepSz .* derivFcn(q+1/2.*k1, u);
                k3 = stepSz .* derivFcn(q+1/2.*k2, u);
                k4 = stepSz .* derivFcn(q+k3     , u);
                dq = 1/6.*k1 + 1/3.*k2 + 1/3.*k3 + 1/6.*k4;
                q = upFcn(q,dq);
                hTot = hTot+stepSz;
            end
        end
    end
end
