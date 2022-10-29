function V=ma2ve(M,type)
% V=ma2ve(M,type)
%
% Converts the matrix M into the vector V of its "free entries".
% The matrix data is stored row by row and structure
% is taken into account. For symmetric matrices, only data on
% or below the diagonal is stored in V.
%
% Input:
%    M           matrix
%    TYPE        specifies the structure of M
%                   1 -> symmetric
%                   2 -> full rectangular
%

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.
if type==1
  % WARNING: M symmetrized if necessary...
  M =(M+M')/2;
  n = size(M,1);
  V = zeros(n*(n+1)/2,1);
  i = 0;
  for ct=1:n
     V(i+1:i+ct,:) = M(1:ct,ct);
     i = i+ct;
  end
else
   tmpM = M';
   V = tmpM(:);
end
