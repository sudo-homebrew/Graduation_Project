function display(blk)
% Display method.

%   Copyright 1986-2011 The MathWorks, Inc.

% Variable name
VarName = inputname(1);
if isempty(VarName),
   VarName = 'ans';
end
fprintf('\n%s =\n\n',VarName)

nyu = iosize(blk);
M = message('Robust:umodel:udyn4',blk.Name,nyu(1),nyu(2));
fprintf('  %s\n\n',getString(M))
