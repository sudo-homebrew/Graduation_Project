%LMINBR   Get the number of LMIs in an LMI system
%
% NLMIS = LMINBR(LMISYS) returns the number of LMIs in LMI system LMISYS
%
% See also  MATNBR, DECNBR, LMIINFO.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.

function nlmi=lminbr(LMIsys)

if nargin ~= 1,
  error('usage: nlmi = lminbr(LMISYS)');
elseif size(LMIsys,1)<10,
  error('LMISYS is an incomplete LMI system description');
elseif any(LMIsys(1:8)<0),
  error('LMISYS is not an LMI description');
end

nlmi=LMIsys(1);
