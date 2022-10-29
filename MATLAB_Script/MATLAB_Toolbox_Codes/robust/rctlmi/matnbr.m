%MATNBR   Get the number of matrix variables in an LMI system
%
% NMVARS = MATNBR(LMISYS) returns the number of matrix variables NMVARS in
% the LMI system LMISYS.
%
% [NMVARS,VARID] = MATNBR(LMISYS) returns the number of matrix variables 
% NMVARS and the vector of matrix variable identifiers VARID in the LMI 
% system LMISYS (see LMIVAR for details).
%
% See also  DECNBR, LMINBR, LMIINFO.

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2004 The MathWorks, Inc.

function [nmvars,varid]=matnbr(LMIsys)

if nargin ~= 1,
  error('usage: nmvars = matnbr(lmisys)');
elseif size(LMIsys,1)<10 | size(LMIsys,2)>1,
  error('LMISYS is an incomplete LMI system description');
elseif any(LMIsys(1:8)<0),
  error('LMISYS is not an LMI description');
end

nmvars=LMIsys(2);

if nargout > 1,
  [lmis,lmiv]=lmiunpck(LMIsys);
  varid=lmiv(1,:);
end
