function systemExecutor = createSystemExecutor(deviceAddress,varargin)
%CREATESYSTEMEXECUTOR Create a SystemExecutor object that implements
%SystemInterface

% Copyright 2021 The MathWorks, Inc.
if strcmpi(deviceAddress,'localhost')
    systemExecutor = ros.codertarget.internal.LocalSystemExecutor();
else
    % Execute system interface over SSH
    systemExecutor = ros.codertarget.internal.RemoteLnxSystemExecutor(deviceAddress,varargin{:});
end
end