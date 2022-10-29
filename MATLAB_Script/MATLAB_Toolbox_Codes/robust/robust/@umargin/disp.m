function disp(blk)
% Display method.

%   Copyright 2020 The MathWorks, Inc.
M = message('Robust:umodel:umargin6',blk.Name,sprintf('[%.3g,%.3g]',blk.DGM_),...
   sprintf('%.3g',blk.PhaseChange(2)));
fprintf('  %s\n\n',getString(M))