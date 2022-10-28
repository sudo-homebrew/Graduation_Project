function assigninGlobalScope(~, varName, varValue)
%This function is for internal use only. It may be removed in the future.

%ASSIGNINGLOBALSCOPE Assign value to variable
%   If the MODEL input is empty, the expression will always be assigned in the
%   base workspace. Otherwise, the expression will be passed along to the
%   assigninGlobalScope function.

%   Copyright 2017-2021 The MathWorks, Inc.

    sec = ros.slros.internal.bus.Util.getDictionaryDataSection();
    assignin(sec,varName,varValue);
end
