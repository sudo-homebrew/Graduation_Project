function [pkgMsgFiles, pkgSrvFiles, pkgActionFiles] = checkValidityOfMessages(pkgDirs,folderPath,rosver, pkgCell)
%This function is for internal use only. It may be removed in the future.

%CHECKVALIDITYOFMESSAGES checks the validity of message name

%   Copyright 2020-2021 The MathWorks, Inc.

%Precheck, as validateMsg error is not helpful when the name is invalid
pkgMsgFiles = pkgCell;
pkgSrvFiles = pkgCell;
pkgActionFiles = pkgCell;
for iPkg = 1:numel(pkgDirs)
    msgFiles = dir(fullfile(folderPath, pkgDirs{iPkg}, 'msg', '*.msg'));
    srvFiles = dir(fullfile(folderPath, pkgDirs{iPkg}, 'srv', '*.srv'));
    actionFiles = dir(fullfile(folderPath, pkgDirs{iPkg}, 'action', '*.action'));
    if (numel(msgFiles) < 1) && (numel(srvFiles) < 1 ) && (numel(actionFiles) < 1)
        error(message('ros:utilities:util:NoMessageFileFound', fullfile(folderPath, pkgDirs{iPkg}, 'msg')));
    end
    if isequal(rosver,'ros2')
        for iMsg = 1:numel(msgFiles)
            [~, msgName] = fileparts(msgFiles(iMsg).name);
            isValid = ros.internal.isValidMessageName(msgName);
            if ~isValid
                error(message('ros:utilities:custommsg:InvalidMessageName', msgName));
            end
        end
    end
    pkgMsgFiles{iPkg} = msgFiles;
    pkgSrvFiles{iPkg} = srvFiles;
    pkgActionFiles{iPkg} = actionFiles;
end
end