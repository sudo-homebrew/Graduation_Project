function [NV,Ndist] = actual2normalized(blk,AV)
% ACTUAL2NORMALIZED  Transforms actual values to normalized values.
%
%   NV = ACTUAL2NORMALIZED(BLK,AV) transforms the values AV of the
%   uncertain block BLK into normalized values NV. The nominal value
%   of BLK maps to NV=0, values inside the uncertainty range map to
%   the unit ball ||NV||<=1, and values outside the uncertainty range
%   correspond to ||NV||>1. The variable AV can contain a single value
%   or an array of block values, and NV has the same size as AV.
%
%   [NV,NDIST] = ACTUAL2NORMALIZED(BLK,AV) also returns the normalized
%   distance NDIST between the block values AV and the nominal value.
%   This distance is the norm of NV, so NDIST<=1 for values AV inside the
%   uncertainty range and NDIST>1 for values AV outside the uncertainty
%   range. If AV is an array of block values, then NDIST is an array of
%   normalized distances.
%
%   ACTUAL2NORMALIZED supports all uncertain block types (see UNCERTAINBLOCK).
%   The normalized distance NDIST is useful to interpret the robustness
%   margins computed by the ROBSTAB and ROBGAIN commands.
%
%   See also NORMALIZED2ACTUAL, getLimits, UNCERTAINBLOCK, ROBSTAB, ROBGAIN.

%   Copyright 2010-2012 The MathWorks, Inc.
if nargin~=2
   error(message('Control:general:TwoInputsRequired',...
      'actual2normalized','actual2normalized'))
end
try
   if nargout<2
      NV = act2norm(blk,AV);
   else
      [NV,Ndist] = act2norm(blk,AV);
   end
catch ME
   throw(ME)
end

