classdef PropertyInfo < robotics.core.internal.mixin.Unsaveable
%This class is for internal use only. It may be removed in the future.

% PROPERTYINFO Class handling MATLAB target parameters.

%   Copyright 2020-2021 The MathWorks, Inc.
    properties (Constant)
        DeployToOptions = {...
            getString(message('ros:mlroscpp:codegen:Localhost')),...
            getString(message('ros:mlroscpp:codegen:RemoteDevice'))}
        BuildActionOptions = {...
            getString(message('ros:mlroscpp:codegen:None')),...
            getString(message('ros:mlroscpp:codegen:Buildandload')),...
            getString(message('ros:mlroscpp:codegen:Buildandrun'))}
        TargetVersion = 2
        TargetParameterDefaults = struct(...
            'PackageMaintainerName','ROS User',...
            'PackageMaintainerEmail','rosuser@test.com',...
            'PackageLicense','BSD',...
            'PackageVersion','1.0.0',...
            'DeployTo',ros.codertarget.internal.PropertyInfo.DeployToOptions{1},...
            'ROSFolder',ros.codertarget.internal.DeviceParameters.DefaultROSInstall,...
            'CatkinWorkspace',ros.codertarget.internal.DeviceParameters.DefaultCatkinWorkspace,...
            'ROS2Folder',ros.codertarget.internal.DeviceParameters.DefaultROS2Install,...
            'ROS2Workspace',ros.codertarget.internal.DeviceParameters.DefaultROS2Workspace,...
            'RemoteDeviceAddress','192.168.128.130',...
            'RemoteDeviceUsername','user',...
            'RemoteDevicePassword','password',...
            'BuildAction',ros.codertarget.internal.PropertyInfo.BuildActionOptions{1})
    end

    properties (Access = ?matlab.unittest.TestCase)
        % These are properties that has R/W access
        TargetParameter = ros.codertarget.internal.PropertyInfo.TargetParameterDefaults
    end

    methods (Static)
        function obj = getInstance()
        % Singleton class to maintain target parameters for the
        % duration of the MATLAB session. Unless user calls 'clear
        % classes', the parameters set in this MATLAB session will be
        % retained.
            persistent instance
            if isempty(instance)
                instance = ros.codertarget.internal.PropertyInfo;
            end
            obj = instance;
        end
    end

    methods (Hidden)
        function val = getTargetParam(obj,paramName)
            validateTargetParam(obj,paramName);
            val = obj.TargetParameter.(paramName);
        end

        function setTargetParam(obj,paramName,val)
            validateTargetParam(obj,paramName);
            obj.TargetParameter.(paramName) = val;
        end

        function reset(obj)
            obj.TargetParameter = obj.TargetParameterDefaults;
        end
    end

    methods (Access = private)
        function obj = PropertyInfo
        % Private constructor to prevent explicit construction
        end

        function validateTargetParam(obj,paramName)
            fnames = fieldnames(obj.TargetParameterDefaults);
            if ~ismember(paramName,fnames)
                txt = sprintf('''%s'', ',fnames{:});
                error(message('ros:mlroscpp:codegen:InvalidTargetParameterName',...
                              txt,['''' paramName '''']))
            end
        end
    end

    methods (Static)
        % Methods in this block are used as get / set methods on the
        % coder.hardware object. These methods must be static.
        function out = getTargetVersion(~)
        % TargetVersion >= 2 executes registered hook points for
        % onHardwareSelectHook, preBuild, postCodegen and postBuild
            out = ros.codertarget.internal.PropertyInfo.TargetVersion;
        end

        function out = setgetPackageMaintainerName(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'PackageMaintainerName');
                setTargetParam(obj,'PackageMaintainerName',data);
            end
            out = getTargetParam(obj,'PackageMaintainerName');
        end

        function out = setgetPackageMaintainerEmail(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'PackageMaintainerEmail');
                setTargetParam(obj,'PackageMaintainerEmail',data);
            end
            out = getTargetParam(obj,'PackageMaintainerEmail');
        end

        function out = setgetPackageLicense(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'PackageLicense');
                setTargetParam(obj,'PackageLicense',data);
            end
            out = getTargetParam(obj,'PackageLicense');
        end

        function out = setgetPackageVersion(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'PackageVersion');
                setTargetParam(obj,'PackageVersion',data);
            end
            out = getTargetParam(obj,'PackageVersion');
        end

        function out = setgetDeployTo(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                data = validatestring(data,ros.codertarget.internal.PropertyInfo.DeployToOptions,'','DeployTo');
                setTargetParam(obj,'DeployTo',data);
            end
            out = getTargetParam(obj,'DeployTo');
        end

        function out = setgetROS2Folder(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'ROS2Folder');
                setTargetParam(obj,'ROS2Folder',data);
            end
            out = getTargetParam(obj,'ROS2Folder');
        end

        function out = setgetROSFolder(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'ROSFolder');
                setTargetParam(obj,'ROSFolder',data);
            end
            out = getTargetParam(obj,'ROSFolder');
        end

        function out = setgetROS2Workspace(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'ROS2Workspace');
                setTargetParam(obj,'ROS2Workspace',data);
            end
            out = getTargetParam(obj,'ROS2Workspace');
        end

        function out = setgetCatkinWorkspace(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'CatkinWorkspace');
                setTargetParam(obj,'CatkinWorkspace',data);
            end
            out = getTargetParam(obj,'CatkinWorkspace');
        end

        function out = setgetRemoteDeviceAddress(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'RemoteDeviceAddress');
                setTargetParam(obj,'RemoteDeviceAddress',data);
            end
            out = getTargetParam(obj,'RemoteDeviceAddress');
        end

        function out = setgetRemoteDeviceUsername(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'RemoteDeviceUsername');
                setTargetParam(obj,'RemoteDeviceUsername',data);
            end
            out = getTargetParam(obj,'RemoteDeviceUsername');
        end

        function out = setgetRemoteDevicePassword(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                i_validateCharArray(data,'RemoteDevicePassword');
                setTargetParam(obj,'RemoteDevicePassword',data);
            end
            out = getTargetParam(obj,'RemoteDevicePassword');
        end

        function out = setgetBuildAction(~,data)
            obj = ros.codertarget.internal.PropertyInfo.getInstance;

            % Set operation
            if nargin > 1
                data = validatestring(data,ros.codertarget.internal.PropertyInfo.BuildActionOptions,'','BuildAction');
                setTargetParam(obj,'BuildAction',data);
            end
            out = getTargetParam(obj,'BuildAction');
        end
    end
end

function i_validateCharArray(data,varName)
    validateattributes(data,{'char'},{'row'},'',varName);
end
