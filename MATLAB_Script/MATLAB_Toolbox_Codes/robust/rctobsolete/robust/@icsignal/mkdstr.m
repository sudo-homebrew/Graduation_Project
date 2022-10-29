%MKDSTR Nice display of ICSIGNAL objects.
%
%OUT = MKDSTR(ICS) returns a nice display of the ICSIGNAL object ICS.
%
%See also: DISPLAY

function dstr = mkdstr(arg1)
% Copyright 2003-2004 The MathWorks, Inc.

%exstr = '';
%go = 1;
%cnt = 1;
%MaxCnt = length(arg1.Float);
%while cnt<=MaxCnt & go==1
%   if ~isempty(arg1.Float{cnt})
%      exstr = ' (contains floating dimensions)';
%      go = 0;
%   else
%      cnt = cnt + 1;
%   end
%end
%dstr = ['IC Signal: dimension = ' int2str(size(arg1.System,1)) exstr];
dstr = ['IC Signal: dimension = ' int2str(size(arg1.System,1))];