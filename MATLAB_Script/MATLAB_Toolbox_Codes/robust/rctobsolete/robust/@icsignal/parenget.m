%PARENGET Low level get command for ICSIGNAL objects

function out = parenget(icsig,L)
% Copyright 2003-2004 The MathWorks, Inc.

if length(L(1))==1
   tmp = L(1);
   idx = tmp{1};
   out = icsig;
   out.System = out.System(idx,:);
elseif length(L(1))>=2
   error('ICSIGNAL: Only single indexing is allowed');
end
