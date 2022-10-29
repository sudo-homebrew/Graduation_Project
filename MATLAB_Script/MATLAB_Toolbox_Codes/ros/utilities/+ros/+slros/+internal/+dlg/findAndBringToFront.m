function [src, dlg] = findAndBringToFront(className, tag, hndlCompare)
%This class is for internal use only. It may be removed in the future.

% findAndBringToFront find all open dialogs with a given tag and iterate over them

%   Copyright 2018-2019 The MathWorks, Inc.

[src, dlg] = robotics.slcore.internal.dlg.findAndBringToFront(className, tag, hndlCompare);
