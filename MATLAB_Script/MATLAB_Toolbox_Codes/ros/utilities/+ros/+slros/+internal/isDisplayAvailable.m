function hasDisplay = isDisplayAvailable
%This function is for internal use only. It may be removed in the future.

%isDisplayAvailable Verify if MATLAB session has an active display
%   isDisplayAvailable returns FALSE if users start MATLAB in the "nodisplay" 
%   or "nodesktop" mode. Otherwise, a valid display is connected and this
%   function returns TRUE.

%   Copyright 2019 The MathWorks, Inc.

% Use an undocumented feature call. P-code this file to obfuscate for users.
hasDisplay = feature('HasDisplay');
end

