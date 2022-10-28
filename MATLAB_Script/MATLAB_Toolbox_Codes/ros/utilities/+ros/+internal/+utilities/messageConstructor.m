function [packageName,fileName,dirsToLook] = messageConstructor(msg,dirsToLook)
%messageConstructor takes msg and dirsToLook as inputs and returns its
%   package name, file name and dirsToLook.

%   Copyright 2019-2020 The MathWorks, Inc.

partsOfMessage = strsplit(msg,'/');
if isequal(numel(partsOfMessage),2)
    [packageName, fileName] = partsOfMessage{:};
    if ~iscell(dirsToLook)
        error(message(...
            'ros:utilities:messageparser:PathIsNotCell'));
    end
else
    error(message(...
        'ros:utilities:messageparser:InvalidMessageType',msg));
end
end
