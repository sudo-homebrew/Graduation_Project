function coreExecutor = createCoreExecutor(deviceAddress,systemExecutor)
%CREATESYSTEMEXECUTOR 

% Copyright 2021 The MathWorks, Inc.
if isequal(deviceAddress,'localhost')
    % Use core executor for local device
    coreExecutor = ros.codertarget.internal.LocalCoreExecutor(systemExecutor);
else
    % Use core executor for remote Linux device
    coreExecutor = ros.codertarget.internal.RemoteLnxCoreExecutor(systemExecutor);
end
end