function disp(blk)
% Display method.

%   Copyright 1986-2020 The MathWorks, Inc.
ios = blk.IOSize_;
M = message('Robust:umodel:ucomplexm5',blk.Name,ios(1),ios(2));
fprintf('  %s\n\n',getString(M))
