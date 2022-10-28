% DECNBR   Get the number of decision variables in LMI system
%
%   NDECV = DECNBR(LMISYS) returns the number of decision variables NDECV 
%   in the system of LMIs, LMISYS. The decision varibales are the free
%   scalar variables to be optimized by the LMI solvers.
%
% See also  DECINFO, DEC2MAT, MAT2DEC, MATNBR, LMIVAR.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.


function ndecv=decnbr(LMIsys)

if nargin ~= 1,
  error('usage: ndecv = decnbr(lmisys)');
elseif size(LMIsys,1) < 10 | size(LMIsys,2)>1,
  error('LMISYS is not an LMI description');
end

ndecv=LMIsys(8);
