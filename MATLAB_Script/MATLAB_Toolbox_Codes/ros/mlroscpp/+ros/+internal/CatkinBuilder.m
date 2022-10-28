classdef CatkinBuilder < ros.internal.ROSProjectBuilder
% This class is for internal use only. It may be removed in the future.

% CatkinBuilder is a wrapper around Catkin. Used by ProjectTool to
% build ROS modules

% Copyright 2019-2021 The MathWorks, Inc.

    properties (Constant, Hidden)
        CMAKEMINVERSION = '3.15.5'
        FUNCTIONNAME = 'CatkinBuilder';
        CMAKETEMPLATE = fullfile(fileparts(mfilename('fullpath')),'CMakeList.txt.tmpl')
        PKGXMLTEMPLATE = fullfile(fileparts(mfilename('fullpath')),'package.xml.tmpl')
        VISIBILITYHTEMPLATE = fullfile(fileparts(mfilename('fullpath')),'visibility_control.h.tmpl') 
        CUSTOMMSG_FOLDER_SUFFIX = '';
        CUSTOMSRV_FOLDER_SUFFIX = '';
    end

    properties (SetAccess=private,GetAccess=public)
        CatkinPrefixPath
    end

    methods
        function h = CatkinBuilder(varargin)
            h@ros.internal.ROSProjectBuilder(varargin{:});
            h.CatkinPrefixPath = ['"' ros.internal.getCatkinPrefixPath '"'];
        end
    end
    
    methods (Access=protected)
        function [status, result] = runBuildSystemCommand(h, varargin)
            ros1InstallDir = ros.internal.getCatkinPrefixPath;
            [boostRoot,~] = ros.internal.getBoostRootPath;
            cmdline = sprintf('%s ',varargin{:});
            % set boost and ROS catkin install folder environment variables
            if ispc
                cmdline = ['set BOOST_INSTALL_DIR=',strrep(boostRoot,'\','/'), ...
                    '&& set ROS1_INSTALL_DIR=',strrep(ros1InstallDir,'\','/'), ...
                    '&& set ROS_LANG_DISABLE=geneus:genlisp:gennodejs', ...
                    '&& ',cmdline];
            else
                setenv('BOOST_INSTALL_DIR',boostRoot);
                setenv('ROS1_INSTALL_DIR', ros1InstallDir);
                setenv('ROS_LANG_DISABLE','geneus:genlisp:gennodejs');
            end
            
            %create a log folder, which contains build/catkin_log folder with time
            %stamps of each build.
            buildTimestamp = ['build_',char(datetime('now'), 'yyyy-MM-dd_HH-mm-ss')];
            logDir = fullfile(h.RootDir,'log', buildTimestamp, 'catkin_log');
            h.chkAndCreateDir(logDir);
            logfilePath = fullfile(logDir, 'stdout_stderr.log');
            [status, result] = ros.internal.runroscmd(cmdline, ['"' strtrim(h.MexInfo.Details.CommandLineShell) '"'], logfilePath);
        end
    end
    
    methods (Static, Hidden)
        function ret = getBuildCommand(rootFolder)
            cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
                                    {['catkin_make install -C "' rootFolder '" ' ], ...use -C in windows
                                'catkin_make install ',... no other option for mac and linux
                                'catkin_make install '});
            ret = cmdmap(computer('arch'));            
        end
        
        function [LocalPythonPath, ActivatePath, PyEnvDir] = setupPythonAndCmakeTools(forceRecreateVENV)
            [LocalPythonPath, ActivatePath, PyEnvDir] =  ros.internal.createOrGetLocalPython(forceRecreateVENV);
        end
        
        function cmkminver = getCMAKEMinimumVersion()
            cmkminver = ros.internal.CatkinBuilder.CMAKEMINVERSION;
        end
        
        function dotPrintObj = createDotPrinter()
            dotPrintObj = ros.internal.DotPrinter('ros:mlroscpp:util:RunningCatkinCmd', strrep(pwd,'\','/'));
        end
        
        function retCmd = formatCommand(cmd)
            retCmd = [cmd ' --pkg '];
        end
        
        function [aPath, aVersion] = getCMakePath()
            [aPath, aVersion] = ros.internal.utilities.getCMakeBinaryPath(ros.internal.CatkinBuilder.CMAKEMINVERSION);
        end
        
        function textProgressObj = createTextProgressBar()
            textProgressObj = ros.internal.TextProgressBar('ros:mlroscpp:util:RunningCatkinCmd', strrep(pwd,'\','/'));
        end
    end
end

% LocalWords:  geneus genlisp gennodejs genpy
