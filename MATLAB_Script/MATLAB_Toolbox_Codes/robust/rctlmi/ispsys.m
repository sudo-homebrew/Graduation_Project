function bool = ispsys(sys)
% ISPSYS   True for parameter-dependent systems
%
%   BOOL = ISPSYS(SYS) returns 1 if SYS is a polytopic or parameter-dependent
%   system
%
%   See also  PSYS, PSINFO.

%  Author: P. Gahinet  6/94
%  Copyright 1995-2004 The MathWorks, Inc.
bool=(size(sys,1)>6 && size(sys,2)>1 && isnumeric(sys) && sys(1,1)==-Inf);
