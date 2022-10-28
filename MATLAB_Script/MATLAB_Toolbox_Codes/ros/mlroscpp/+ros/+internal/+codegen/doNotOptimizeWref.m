function doNotOptimizeWref(input)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.
%#codegen
coder.inline('always');
coder.ceval('//',coder.wref(input));
end

