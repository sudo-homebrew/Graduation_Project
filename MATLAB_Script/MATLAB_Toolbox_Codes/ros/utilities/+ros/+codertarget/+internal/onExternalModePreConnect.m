function onExternalModePreConnect(hCS, timeout)
%This function is for internal use only. It may be removed in the future.

%onExternalModePreConnect Pre-connect function for one click external mode
%   This function is called after code generation and after the ROS node is
%   started, but before external mode tries to connect to the executable.

%   Copyright 2016-2020 The MathWorks, Inc.

% Default timeout is 10 seconds
    if nargin < 2
        timeout = 10;
    end

    % Create diagnostic stage for external mode
    modelName = ros.codertarget.internal.getModelName(hCS);
    diagStage = ros.slros.internal.diag.DiagnosticViewerStream(modelName, false);
    diagStage.open(message('ros:slros:extmode:ExternalModeStage').getString);

    % Print out some information about the external mode target
    extModeAddress = regexprep(codertarget.attributes.getExtModeData('IPAddress', hCS), '\''', '');
    diagStage.reportInfo(message('ros:slros:extmode:ExternalModeConnect', ...
                                 extModeAddress, codertarget.attributes.getExtModeData('Port', hCS)).getString)

    % External mode connection should use the latest ipaddress, hence refresh
    % the ipaddress. Needed when the user changes an erroneous ROS target address
    % during the deployment phase.
    mexParam = get_param(hCS, 'ExtModeMexArgs');
    mexParam = regexprep(mexParam, '''.*''', ['''', extModeAddress, '''']);
    set_param(bdroot, 'ExtModeMexArgs', mexParam);

    if ros.codertarget.internal.isRemoteBuild(hCS)
        % Note that the connection has been verified and the ROS node has been launched
        % in ros.codertarget.internal.DeploymentHooks.loadCommand
        target = rosdevice;
        
        % Ensure that the IP address for the external mode connection is the same
        % as the device address. If not, display a warning to the user.
        % This scenario can occur if somebody has a model that they previously used
        % with a different target in external mode and then switch to ROS.
        if string(extModeAddress) ~= string(target.DeviceAddress)
            diagStage.reportWarning(message('ros:slros:extmode:ExternalModeAddressMismatch', ...
                extModeAddress, target.DeviceAddress))
        end
        
        % Pause here until application starts
        isRunning = false;
        for i = 1:1:timeout
            isRunning = target.isNodeRunning(modelName);
            if ~isRunning
                % Pause for a second to see if node starts up
                pause(1);
            else
                % If node is up, break out of loop
                break;
            end
        end
        
        % If node did not start up, throw error and display log file
        if ~isRunning
            logFile = [target.CatkinWorkspace '/' modelName '.log'];
            try
                logContents = system(target, ['cat ' logFile]);
            catch
                % Log file might not exist
                logContents = '';
            end
            error(message('ros:slros:extmode:ExternalModeDidNotStart', logFile, logContents));
        end
    end

    % Disable external mode warnings about unsupported bus signals
    warning('off', 'Simulink:Engine:ExtModeCannotDownloadParamBecauseNoHostToTarget');

end
