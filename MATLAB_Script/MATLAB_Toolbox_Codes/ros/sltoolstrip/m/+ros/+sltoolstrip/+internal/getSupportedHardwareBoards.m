function out = getSupportedHardwareBoards()
% getSupportedHardwareBoards - Return all boards supported by Robot
% Operating System (ROS) app.

% Copyright 2019 The MathWorks, Inc.

out = codertarget.targethardware.getSupportedHardwareBoardsForID(...
    codertarget.targethardware.BaseProductID.ROS);
end