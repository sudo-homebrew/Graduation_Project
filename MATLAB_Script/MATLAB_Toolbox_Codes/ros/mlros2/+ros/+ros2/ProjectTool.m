classdef ProjectTool < ros.codertarget.internal.ProjectTool & coder.make.ProjectTool
% This class is for internal use only. It may be removed in the future.

% Project tool class for colcon. Uses ColconBuilder to create, build
% and run

% Copyright 2019-2021 The MathWorks, Inc.

    properties (Constant)
        ProjectName = 'Colcon Project';
        ROSVersion = 'ros2';
        DefaultDependencies = {'rclcpp'};
        % LinkReferenceLibraries Explicitly add dependent libraries from
        % the referenced model to target_link_libraries list
        %
        % Colcon toolchain does not need to link to these reference
        % libraries
        LinkReferenceLibraries = false;
    end

    methods
        function h = ProjectTool(~)
            h@coder.make.ProjectTool(ros.ros2.ProjectTool.ProjectName);
        end

        function [ret, context] = createProject(h, buildInfo, context, varargin)
            if isequal(buildInfo.getBuildName,'rtwshared')
                % Skip sharedutils
                ret = buildInfo.getBuildName;
                return;
            end
            [ret, context] = h.createProject@ros.codertarget.internal.ProjectTool(buildInfo, context, varargin{:});
        end
    end

    methods (Hidden)
        function ret = getProjectData(obj)
            infoFile = obj.getROS2ModelInfoFile;
            if isfile(infoFile)
                ros2ModelInfo = load(infoFile);
                ret = ros2ModelInfo.ros2ModelInfo;
            else
                ret = '';
            end
        end

        function ret = getProjectBuilder(~, anchorDir, pkgName, varargin)
            ret = ros.ros2.internal.ColconBuilder(anchorDir, pkgName, varargin{:});
        end

        function [res, installDir] = runBuildCommand(~, context)
            appendBuildArgs = sprintf(' --base-paths "%s" ', fullfile(context.anchorDir, 'src'));
            [res, installDir] = context.projectBuilder.buildPackage(context.pkgsToBuild,' --merge-install', appendBuildArgs);
        end

        function actionDep = getActionDependencies(~, ~)
            actionDep = {};
        end

        function tfDep = getTransformationDependencies(~, ~)
            tfDep = {};
        end
    end

    methods (Static, Hidden)
        function ret = getValidColconPackageName(val)
        %GETVALIDCOLCONPACKAGENAME Get a valid colcon package name for
        %a given character vector
            ret = ros.codertarget.internal.ProjectTool.getValidPackageName(val);
        end

        function ret = getROS2ModelInfoFile()
            ret = 'ros2ModelInfo.mat';
        end
    end

end
