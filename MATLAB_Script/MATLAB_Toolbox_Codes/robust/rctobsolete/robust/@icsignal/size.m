% SIZE   Size of ICSGINAL object.

function [varargout] = size(m,arg2)
% Copyright 2003-2004 The MathWorks, Inc.

out = [size(m.System,1) 1];
if nargin==1
   arg2 = nan;
end
varargout = csize(out,arg2,nargin,nargout);