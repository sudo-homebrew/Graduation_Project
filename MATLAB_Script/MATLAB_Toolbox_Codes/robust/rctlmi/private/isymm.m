function [bool]=isymm(M,tol)
% Tests if M is symmetric

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.
if size(M,1)~=size(M,2),
  bool=0; return
elseif nargin==1,
  tol=eps^(2/3);
end
bool=(max(max(abs(M-M'))) <= tol*max(max(abs(M))));
