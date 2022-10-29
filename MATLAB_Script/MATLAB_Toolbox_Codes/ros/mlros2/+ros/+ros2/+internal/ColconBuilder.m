classdef ColconBuilder < ros.internal.ROSProjectBuilder
    % This class is for internal use only. It may be removed in the future.
    
    % ColconBuilder is a wrapper around colcon. Used by ProjectTool to
    % build ROS modules
    
    % Copyright 2019-2021 The MathWorks, Inc.
    
    properties (Constant, Hidden)
        CMAKEMINVERSION = '3.15.5'
        FUNCTIONNAME = 'ColconBuilder';
        CMAKETEMPLATE = fullfile(fileparts(mfilename('fullpath')),'CMakeList.txt.tmpl')
        PKGXMLTEMPLATE = fullfile(fileparts(mfilename('fullpath')),'package.xml.tmpl')
        VISIBILITYHTEMPLATE = fullfile(fileparts(mfilename('fullpath')),'visibility_control.h.tmpl')        
        CUSTOMMSG_FOLDER_SUFFIX = 'msg/';
        CUSTOMSRV_FOLDER_SUFFIX = 'srv/';
    end
    
    properties (SetAccess=private,GetAccess=public)
        AmentPrefixPath
    end
    
    methods
        function h = ColconBuilder(varargin)
            h@ros.internal.ROSProjectBuilder(varargin{:});
            h.AmentPrefixPath = ['"' ros.ros2.internal.getAmentPrefixPath '"'];
        end
    end
    
    methods (Access=protected)
        function [status, result] = runBuildSystemCommand(h, varargin)
            cmdline = sprintf('%s ',varargin{:});
            cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
                                    {['"' fullfile(fileparts(mfilename('fullpath')),'runcolconcmd') '"  "' strtrim(h.MexInfo.Details.CommandLineShell) '"'], ...use .bat file
                                ['"' fullfile(fileparts(mfilename('fullpath')),'runcolconcmd.sh') '"'],... use .sh
                                ['"' fullfile(fileparts(mfilename('fullpath')),'runcolconcmd.sh') '"']});
            cmd = cmdmap(computer('arch'));            
            [status, result] = system([cmd, ' ', h.AmentPrefixPath, ' ', h.PyEnvDir, ' ', cmdline]);
        end
    end
    
    methods (Static, Hidden)
        
        function ret = getBuildCommand(~)
            ret = 'build';
        end
        
        function [LocalPythonPath, ActivatePath, PyEnvDir] = setupPythonAndCmakeTools(forceRecreateVENV)
            [LocalPythonPath, ActivatePath, PyEnvDir] = ros.ros2.internal.createOrGetLocalPython(forceRecreateVENV);
        end
        
        function cmkminver = getCMAKEMinimumVersion()
            cmkminver = ros.ros2.internal.ColconBuilder.CMAKEMINVERSION;
        end
        
        function dotPrintObj = createDotPrinter()
            dotPrintObj = ros.internal.DotPrinter('ros:mlros2:util:RunningColconCmd', strrep(pwd,'\','/'));
        end
        
        function retCmd = formatCommand(cmd)
            retCmd = [cmd ' --packages-select '];
        end
        
        function [aPath, aVersion] = getCMakePath()
            [aPath, aVersion] = ros.internal.utilities.getCMakeBinaryPath(ros.ros2.internal.ColconBuilder.CMAKEMINVERSION);
        end
        
        function textProgressObj = createTextProgressBar()
            textProgressObj = ros.internal.TextProgressBar('ros:mlros2:util:RunningColconCmd', strrep(pwd,'\','/'));
        end
    end
end
