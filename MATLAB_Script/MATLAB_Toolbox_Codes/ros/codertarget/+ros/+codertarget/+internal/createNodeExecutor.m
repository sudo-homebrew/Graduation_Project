function nodeExecutor = createNodeExecutor(deviceAddress,systemExecutor,rosVersion)
%CREATESYSTEMEXECUTOR

% Copyright 2021 The MathWorks, Inc.
validateattributes(systemExecutor,...
    {'ros.codertarget.internal.SystemInterface'},{'nonempty'},'createNodeExecutor','systemExecutor');
rosVersion = validatestring(rosVersion,{'ros','ros2'});
if strcmpi(deviceAddress,'localhost')
    if strcmpi(computer('arch'),'win64')
        nodeExecutor = ros.codertarget.internal.WinNodeExecutor(systemExecutor,rosVersion);
    elseif strcmpi(computer('arch'),'glnxa64')
        nodeExecutor = ros.codertarget.internal.LnxNodeExecutor(systemExecutor,rosVersion);
    elseif strcmpi(computer('arch'),'maci64')
        nodeExecutor = ros.codertarget.internal.MacNodeExecutor(systemExecutor,rosVersion);
    end
else
    nodeExecutor = ros.codertarget.internal.LnxNodeExecutor(systemExecutor,rosVersion);
end
end