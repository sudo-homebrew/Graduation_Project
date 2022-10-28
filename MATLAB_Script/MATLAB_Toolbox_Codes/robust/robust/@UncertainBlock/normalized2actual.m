function AV = normalized2actual(blk,NV)
% NORMALIZED2ACTUAL  Transforms normalized values to actual values.
%
%   AV = NORMALIZED2ACTUAL(BLK,NV) transforms the normalized values NV
%   of the uncertain block BLK into actual block values AV. This is
%   the inverse of the ACTUAL2NORMALIZED transform. The variable NV
%   can contain a single value or an array of normalized values. The
%   output AV has the same size as NV.
%
%   ACTUAL2NORMALIZED supports all uncertain block types (see
%   UNCERTAINBLOCK for a complete list).
%
%   See also ACTUAL2NORMALIZED, getLimits, UNCERTAINBLOCK.

%   Copyright 2010-2012 The MathWorks, Inc.
if nargin~=2
   error(message('Control:general:TwoInputsRequired',...
      'normalized2actual','normalized2actual'))
end
try
   AV = norm2act(blk,NV);
catch ME
   throw(ME)
end