classdef LocalCoreExecutor < ros.codertarget.internal.CoreInterface
    %LOCALCOREEXECUTOR Implements CoreInterface for localhost

    % Copyright 2021 The MathWorks, Inc.
    properties (GetAccess = private, SetAccess = immutable)
        SystemExecutor
    end

    properties (Access = ?ros.slros.internal.InternalAccess)
        CoreHandle
    end

    methods
        function obj = LocalCoreExecutor(systemExecutor)
            obj.SystemExecutor = systemExecutor;
        end

        function isRunning = isCoreRunning(obj)
            % Core was started outside MATLAB
            if isequal(computer("arch"),'win64')
                % win64 - use tasklist
                procName = 'rosmaster.exe';
                cmd = 'tasklist /FI "IMAGENAME eq rosmaster.exe" /FO list';
            elseif isequal(computer("arch"),'glnxa64')
                % Output pid and args left justified
                procName = 'rosmaster';
                cmd = 'ps axo pid:1,args:1 | grep -E "rosmaster(\\s|$)" | grep -v "grep"';                
            else
                procName = 'rosmaster';
                % rosmaster runs through python2. Specifying args allows us
                % to examine the whole command line for python2 & rosmaster
                cmd = 'ps -axO args | grep -E "rosmaster" | grep -E "python" | grep -v "grep"';
            end

            % If command fails, return not running
            try
                result = system(obj.SystemExecutor,cmd);
            catch
                result = '';
            end
            isRunning = contains(result,procName); 
        end

        function stopCore(obj)
            if ~isempty(obj.CoreHandle)
                delete(obj.CoreHandle);
                obj.CoreHandle = ros.Core.empty;
            else
                % Core was started outside rosdevice class using rosinit. Attempt to
                % shutdown core via rosshutdown
                rosshutdown;
                if isCoreRunning(obj)
                    % Kill core started outside MATLAB
                    if isequal(computer("arch"),'win64')
                        system(obj.SystemExecutor,...
                                'taskkill /f /fi "IMAGENAME eq rosmaster.exe');
                    else
                        % Linux & MAC
                        try
                            % Note there is no sudo here. We do not attempt
                            % to kill processes created by other users
                            cmd = 'killall roscore rosmaster rosout';
                            system(obj.SystemExecutor,cmd);
                        catch ex
                            % Parse exception
                            exMsg = string(ex.message);

                            if exMsg.contains('Operation not permitted')
                                % The user does not have the correct privileges to kill
                                % at least one of the roscore processes
                                error(message('ros:slros:rosdevice:StopROSCoreNoPrivileges', 'current user'));
                            elseif exMsg.contains('no process found')
                                % This is okay, since all roscore processes are already dead.
                                % Silently swallow this exception.
                            else
                                % Throw generic error if something else went wrong
                                rethrow(ex);
                            end
                        end
                    end
                end
            end
        end

        function runCore(obj,~,~)
            % runCore does not need catkinWorkspace and ROSFolder input
            % arguments as in remote core executor. Local core executor
            % uses ros.Core class to start the core
            obj.CoreHandle = ros.Core;
        end
    end
end