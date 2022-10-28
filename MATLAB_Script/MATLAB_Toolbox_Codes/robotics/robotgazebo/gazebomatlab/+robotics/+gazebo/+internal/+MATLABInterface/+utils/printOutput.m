function printOutput( info , printType, initMessage)
%This function is for internal use only. It may be removed in the future.
%
% This function displays input information. It accepts input forms such as
% cell, char ans string. Further, it displays in vertical as well
% as horizontal pattern. It can be initialized with specific text.

%   Copyright 2020 The MathWorks, Inc.

% initial text
    message = initMessage;
    
    if(isempty(info))
        % process empty input
        message = [message,'  '];
    elseif(iscell(info) || isstring(info))
        % process cell
        if(strcmp(printType,'h'))
            message = [message,'  ', char(join(info,' '))];
        else
            message = [message, newline, char(join(info,newline))];
        end
    elseif(ischar(info))
        % process char
        if(strcmp(printType,'h'))
            message = [message,'  ', info];
        else
            message = [message, newline, info];
        end

    end

    % display information
    disp(message);
    disp(newline);

end
