classdef Constants
%This class is for internal use only. It may be removed in the future.

%Constants Some constants that are used uniformly in Simulink ROS

%   Copyright 2016-2020 The MathWorks, Inc.

    properties (Constant)
        %PreferenceGroup - The group name for MATLAB preference storage
        PreferenceGroup = 'ROS_Toolbox'

        %TargetHardware - The name of the target selection in Config Params > Hardware Implementation
        TargetHardware = 'Robot Operating System (ROS)'
    end

    % All widget tags for widgets in the ROS configuration set
    % See toolbox\robotics\robotsimulink\robotslros\registry\parameters\ros_parameters.xml
    % for the source of the strings
    properties (Constant)
        %CoderTargetParameter - Config set parameter that stores coder targets settings
        CoderTargetParameter = 'HardwareBoard'

        %TagCSMaintainerName - Tag for maintainer name edit field
        TagCSMaintainerName = ros.slros.internal.Constants.fullTagName('ROSMaintainerName')

        %TagCSMaintainerEmail - Tag for maintainer e-mail edit field
        TagCSMaintainerEmail = ros.slros.internal.Constants.fullTagName('ROSMaintainerEmail')

        %TagCSLicense - Tag for package license edit field
        TagCSLicense = ros.slros.internal.Constants.fullTagName('ROSLicense')

        %TagCSVersion - Tag for package version edit field
        TagCSVersion = ros.slros.internal.Constants.fullTagName('ROSVersion')

        %TagCSDeviceAddress - Tag for ROS device address edit field
        TagCSDeviceAddress = ros.slros.internal.Constants.fullTagName('ROSDeviceAddress')

        %TagCSUsername - Tag for ROS device username edit field
        TagCSUsername = ros.slros.internal.Constants.fullTagName('ROSUsername')

        %TagCSEditButton1 - Tag for "Edit" button in "Device parameters" group
        TagCSEditButton1 = ros.slros.internal.Constants.fullTagName('ROSEditDeviceInfo1')

        %TagCSEditButton2 - Tag for "Edit" button in "Build options" group
        TagCSEditButton2 = ros.slros.internal.Constants.fullTagName('ROSEditDeviceInfo2')

        %TagCSBuildAction - Tag for build action dropdown
        TagCSBuildAction = ros.slros.internal.Constants.fullTagName('ROSBuildAction')

        %TagCSInstallFolder - Tag for ROS installation folder edit field
        TagCSInstallFolder = ros.slros.internal.Constants.fullTagName('ROSInstall')

        %TagCSCatkinWorkspace - Tag for Catkin workspace folder edit field
        TagCSCatkinWorkspace = ros.slros.internal.Constants.fullTagName('ROSCatkinWS')

        % ROS 2 Tags 
        
        %TagCSROS2MaintainerName - Tag for maintainer name edit field
        TagCSROS2MaintainerName  = ros.slros.internal.Constants.fullTagName('ROS2MaintainerName')

        %TagCSROS2MaintainerEmail - Tag for maintainer e-mail edit field
        TagCSROS2MaintainerEmail = ros.slros.internal.Constants.fullTagName('ROS2MaintainerEmail')

        %TagCSROS2License - Tag for package license edit field
        TagCSROS2License = ros.slros.internal.Constants.fullTagName('ROS2License')

        %TagCSROS2Version - Tag for package version edit field
        TagCSROS2Version = ros.slros.internal.Constants.fullTagName('ROS2Version')

        %TagCSROS2DeviceAddress - Tag for ROS device address edit field
        TagCSROS2DeviceAddress = ros.slros.internal.Constants.fullTagName('ROS2DeviceAddress')

        %TagCSROS2Username - Tag for ROS device username edit field
        TagCSROS2Username = ros.slros.internal.Constants.fullTagName('ROS2Username')

        %TagCSROS2EditButton1 - Tag for "Edit" button in "Device parameters" group
        TagCSROS2EditButton1 = ros.slros.internal.Constants.fullTagName('ROS2EditDeviceInfo')

        %TagCSROS2EditButton2 - Tag for "Edit" button in "Build options" group
        TagCSROS2EditButton2 = ros.slros.internal.Constants.fullTagName('ROSEditDeviceInfo2')
        
        %TagCSROS2InstallFolder - Tag for ROS 2 installation folder edit field
        TagCSROS2InstallFolder = ros.slros.internal.Constants.fullTagName('ROS2Install')

        %TagCSROS2Workspace - Tag for ROS 2 workspace folder edit field
        TagCSROS2Workspace = ros.slros.internal.Constants.fullTagName('ROS2Workspace')
    end

    properties (Constant, Access = private)
        %CSWidgetTagPrefix Tag prefix for all widgets
        CSWidgetTagPrefix = 'Tag_ConfigSet_CoderTarget'
    end

    methods (Static)
        function fullTag = fullTagName(relativeTag)
        %fullTagName Assemble full tag name from prefix and relative tag

            fullTag = [ros.slros.internal.Constants.CSWidgetTagPrefix ...
                       '_' relativeTag];
        end
    end
end
