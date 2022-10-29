classdef ProjectTool < ros.codertarget.internal.ProjectTool & coder.make.ProjectTool
% This class is for internal use only. It may be removed in the future.

% Project tool class for catkin. Uses CatkinBuilder to create, build
% and run ROS Node application from a Simulink model

% Copyright 2019-2021 The MathWorks, Inc.

    properties (Constant)
        ProjectName = 'Catkin Project';
        ROSVersion = 'ros';
        DefaultDependencies = {'std_msgs','roscpp'};
        % LinkReferenceLibraries Explicitly add dependent libraries from
        % the referenced model to target_link_libraries list
        %
        % Catkin toolchain must to link to these reference libraries
        % explicitly
        LinkReferenceLibraries = true;
    end

    methods
        function h = ProjectTool(~)
            h@coder.make.ProjectTool(ros.internal.ProjectTool.ProjectName);
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
            infoFile = obj.getROSModelInfoFile;
            if isfile(infoFile)
                info = load(infoFile);
                ret = info.rosProjectInfo;
            else
                ret = '';
            end
        end

        function ret = getProjectBuilder(~, anchorDir, pkgName, varargin)
            ret = ros.internal.CatkinBuilder(anchorDir, pkgName, varargin{:});
        end

        function [res, installDir] = runBuildCommand(~, context)
            if isfield(context,'ProjectData') && ...
                    isfield(context.ProjectData, 'BuildArguments') && ...
                    ~isempty(context.ProjectData.BuildArguments)
                buildArgs = context.ProjectData.BuildArguments;
            else
                buildArgs = '';
            end
            [res, installDir] = context.projectBuilder.buildPackage(...
                context.pkgsToBuild, buildArgs);
        end

        function actionDep = getActionDependencies(~, includeList)
            actionDep = {};
            if ismember('mlroscpp_actclient.h',includeList)
                actionDep = {'actionlib'};
            end
        end

        function tfDep = getTransformationDependencies(~, includeList)
            tfDep = {};
            if ismember('mlroscpp_transform.h',includeList)
                tfDep = {'tf2_ros'};
            end
        end
    end

    methods (Static, Hidden)
        function ret = getValidColconPackageName(val)
        %GETVALIDCOLCONPACKAGENAME Get a valid colcon package name for
        %a given character vector
            arguments
                val (1,1) string
            end
            ret = ros.codertarget.internal.ProjectTool.getValidPackageName(val);
        end

        function ret = getROSModelInfoFile()
            ret = 'rosModelInfo.mat';
        end

    end

end
