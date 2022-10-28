function ret = getExtmodeDeviceAddress(hCS)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.

%getExtmodeDeviceAddress Get the device address for 'Monitor and tune' mode based on device selection (remote device or localhost)

ctData = codertarget.data.getData(hCS);
if isfield(ctData,'ROS') && isfield(ctData.ROS,'RemoteBuild') ....
        && islogical(ctData.ROS.RemoteBuild) && ctData.ROS.RemoteBuild
    % Read the remote device address saved in the DeviceParameters storage
    ret = ros.codertarget.internal.DeviceParameters.getDeviceAddress;
else
    % Default localhost address for local deployment
    ret = '127.0.0.1';
end

end
