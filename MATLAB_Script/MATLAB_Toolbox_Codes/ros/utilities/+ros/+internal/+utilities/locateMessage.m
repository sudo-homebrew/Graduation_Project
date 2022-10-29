function location = locateMessage(packageName,fileName,DirsToLook,args)
%locateMessage will find the location of message file and
%   returns the full path of the message file.

%   Copyright 2019-2020 The MathWorks, Inc.

% Get the message package and message file.
extension = ['.' args];
msgFile = [char(fileName) extension];

% Check for the message package in given dirsToLook cell.
% If found then check for the message file in that directory
% and return the location of message.
location = '';
for i = 1:numel(DirsToLook)
    dirsToLook = fullfile(convertStringsToChars(...
          DirsToLook{i}));
    packagedirs = fullfile(dirsToLook, packageName, args);
    if isfolder(packagedirs)
        dirs = fullfile(packagedirs,msgFile);
        if isfile(dirs)
            location = dirs;
            break;
        end
    end
end

% If location returned is empty, then return error message.
if isempty(location)
    if isfolder(packagedirs)
        error(message(...
            'ros:utilities:messageparser:MessageNotFound',...
            msgFile));
    else
        if isfolder(dirsToLook)
            error(message(...
                'ros:utilities:messageparser:InvalidMessagePackage',...
                packageName));
        else
            error(message(...
                'ros:utilities:messageparser:IncorrectPath'));
        end
    end
end

end