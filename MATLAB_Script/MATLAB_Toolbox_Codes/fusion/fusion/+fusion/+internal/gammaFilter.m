% This is an internal class and may be removed in future release.
% gammaFilter provides static methods for predicting and correcting the
% multiple Gamma distribution using the random matrix model

% Copyright 2018 The MathWorks, Inc.

classdef gammaFilter
    %#codegen
    methods (Static)
        function [alpha,beta] = predict(alpha,beta,nk)
           alpha = alpha./nk;
           beta = beta./nk;
        end
        
        function [alpha,beta] = correct(alpha,beta,numMeas)
            alpha = alpha + numMeas;
            beta = beta + 1;
        end
    end
end