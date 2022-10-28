%

% Copyright 2018 The MathWorks, Inc.

function [ value, msg ] = isRoboticsInstalledAndLicensed( ~ )
    value = dig.isProductInstalled('Robotics System Toolbox');
    msg = '';
end
