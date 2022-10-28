classdef NodeExecutorBase < handle
    %NODEEXECUTORBASE Class implementing methods common to all node
    %executors

    % Copyright 2021 The MathWorks, Inc.
    properties (Access = protected)
        IsRemote = false
        ROSVersion
        SystemExecutor
    end
    
    properties (Constant, Access = protected)
        Timeout = 3
    end

    methods 
        function obj = NodeExecutorBase(systemExecutor,rosVersion)
            obj.SystemExecutor = systemExecutor;
            obj.ROSVersion = rosVersion;
        end
    end

    methods (Access = protected)
        function exeFolder = getExecutableFolder(obj,workspaceFolder)
            if isequal(obj.ROSVersion,'ros')
                exeFolder = obj.SystemExecutor.fullfile(workspaceFolder,'devel');
            else
                % ROS 2 does not have devel folder. Executables are stored
                % under <ROSWorkspace>/install folder.
                exeFolder = obj.SystemExecutor.fullfile(workspaceFolder,'install');
            end
        end

        function nodeName = getNodeNameFromModel(~,modelName)
            %getNodeNameFromModel Convert Simulink model name to the corresponding ROS node name
                nodeName = modelName;
        end
    end

    methods (Static, Access = protected)
        function ret = isIPAddress(input)
            ret = ~isempty(regexp(input,'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}','match','once'));
        end

        function txt = readTextFile(fileName)
            fid = fopen(fileName,'r');
            if fid > 0
                txt = fread(fid,[1,inf],'*char');
                fclose(fid);
            else
                txt = '';
            end
        end
    end
end
