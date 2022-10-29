function dims = constturndims(x)
% This function is for internal use and may be removed or modified later.
%constturndims determines the dimension of constant turn motion model

% Copyright 2018 The MathWorks, Inc.

%#codegen

dims = (size(x,1)-1)/2;
end
