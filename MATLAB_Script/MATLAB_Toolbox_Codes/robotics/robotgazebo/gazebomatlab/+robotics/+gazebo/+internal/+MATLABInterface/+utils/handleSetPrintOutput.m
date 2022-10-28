function handleSetPrintOutput( status , errorMessage)
%This function is for internal use only. It may be removed in the future.
%
% This function prints 'set' operation status based on input.

%   Copyright 2020 The MathWorks, Inc.

% 'Succeed' if status is true
    if(status)
        statusMsg = "Succeed";
    else
        statusMsg = "Failed";
    end
    % print output
    robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
        statusMsg,'h','STATUS:');
    robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
        errorMessage,'h','MESSAGE:');

end
