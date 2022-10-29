% This is an internal class and may be removed in future release.
% iwFilter provides static methods for predicting and correcting the
% multiple IW distributions using the random matrix model

% Copyright 2018 The MathWorks, Inc.
classdef iwFilter
    %#codegen
    methods (Static)
        function [v,V] = predict(v,V,x,tau,dT,M)
           d = size(V,1);
           v = (2*d + 2) + exp(-dT./tau).*(v - 2*d - 2);
           n = size(V,3);
           for i = 1:n
               Rot = M(x(:,i),dT);
               V(:,:,i) = exp(-dT./tau)*Rot*V(:,:,i)*Rot';
           end
        end
        
        function [v,V] = correct(v,V,N,Z,m)
            V = V + N + Z;
            v = v + m;
        end
    end
end