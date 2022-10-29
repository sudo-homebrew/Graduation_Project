function output = packageGazeboPlugin(packagePath,customMessagePath)
%packageGazeboPlugin create Gazebo plugin package for Simulink
%   packageGazeboPlugin creates a folder called GazeboPlugin in the current
%   directory. The folder is also compressed into a zip
%   file. This plugin package is used by Gazebo to communicate with
%   Simulink(R) for synchronized-stepping and sending and receiving
%   messages.
%
%   packageGazeboPlugin(PACKAGEPATH) creates the zipped plugin to the user
%   specified location in PACKAGEPATH. PACKAGEPATH must be a valid file
%   name or file path with the package folder name, specified as a string
%   or character vector. This folder is created and zipped with the Gazebo
%   plugin packaged inside.
%
%   For custom message support,
%   packageGazeboPlugin(PACKAGEPATH,CUSTOMMESSAGEPATH) creates the zipped
%   plugin to the user specified location in PACKAGEPATH which contains
%   custom message dependencies. PACKAGEPATH must be a valid file
%   name or file path with the package folder name, specified as a string
%   or character vector. CUSTOMMESSAGEPATH must be a valid folder path,
%   specified as a string or character vector, which contains custom
%   message dependencies.
%
%   OUTPUT = packageGazeboPlugin(__) returns the folder containing the
%   plugin in uncompressed form.
%
%   Example:
%
%       % package the plugin into GazeboPlugin folder under the current
%       % working directory and compress it
%       packageGazeboPlugin
%
%       % package the plugin into MyPlugin folder under the current working
%       % directory and compress it
%       outputDir = packageGazeboPlugin('MyPlugin')
%
%       % if the custom message dependencies are present in 'C:\GazeboCustomMsgTest'
%       % package the custom message plugin into MyPlugin folder under the
%       % current working directory and compress it
%       % outputDir = packageGazeboPlugin('MyPlugin','C:\GazeboCustomMsgTest')
%
%       % if the custom message dependencies are present in 'C:\GazeboCustomMsgTest'
%       % package the custom message plugin into 'C:\GazeboCustomMsgTest\MyPlugin'
%       % folder and compress it
%       % outputDir = packageGazeboPlugin('C:\GazeboCustomMsgTest\MyPlugin','C:\GazeboCustomMsgTest')

%   Copyright 2019-2020 The MathWorks, Inc.

    if nargin == 0
        % if not path is supplied, create a folder called GazeboPlugin in
        % current directory
        desDir = fullfile(pwd, 'GazeboPlugin');
        packagePath = fullfile(pwd, 'GazeboPlugin.zip');
    else
        % validate the user input path
        validateattributes(packagePath, {'string', 'char'}, {'scalartext'}, 'packageGazeboPlugin', 'packagePath');
        [desDir,name,ext] = fileparts(packagePath);

        if strcmp(ext, "")
            packagePath = strcat(packagePath, '.zip');
        elseif ~strcmp(ext, '.zip')
            % error out if the user input path point to non-zip file
            error(message('robotics:robotgazebo:packageplugin:InvalidFileExtension', packagePath));
        end

        if strcmp(name, "")
            error(message('robotics:robotgazebo:packageplugin:EmptyFileName', packagePath));
        end

        if nargin > 1
            % execute for custom message folder path
            validateattributes(char(customMessagePath), {'string', 'char'}, {'scalartext', 'nonempty'}, 'packageGazeboPlugin', 'packagePath');

            % error out if the user provided non-exist custom message folder path
            if(~exist(customMessagePath,'dir'))
                error(message('robotics:robotgazebo:packageplugin:InValidInputFolderPath', customMessagePath));
            end

            % error out if the user provided empty custom message folder path
            if( ~exist(fullfile(customMessagePath,'customMsgHandlers'),'dir') ||...
                ~exist(fullfile(customMessagePath,'pluginUtilities'),'dir') ||...
                ~exist(fullfile(customMessagePath,'ProtoFiles'),'dir'))
                error(message('robotics:robotgazebo:packageplugin:EmptyFolderPath', customMessagePath));
            end
        end

        % use a folder with the same name user supplied to contain the plugin
        % files
        desDir = fullfile(desDir, name);
    end

    % error out if the output folder already exist
    if exist(desDir, 'dir')
        error(message('robotics:robotgazebo:packageplugin:OutputTargetAlreadyExist', desDir));
    end

    % error out if the output file already exist
    if exist(packagePath, 'file')
        error(message('robotics:robotgazebo:packageplugin:OutputTargetAlreadyExist', packagePath));
    end

    % error out if the output folder cannot be created
    try
        mkdir (desDir)
    catch
        error(message('robotics:robotgazebo:packageplugin:CannotCreateFolder', desDir));
    end

    % path of C++ files from plugin source root
    srcPluginPath = fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab','pluginsrc');

    % copy the plugin source code first
    copyfile (srcPluginPath, desDir);

    % path of C++ files from gazebotransport root
    srcTransportPath = fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebotransport');

    src = {};
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'include', 'gazebotransport', 'transport');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'include', 'gazebotransport', 'gazeboserver');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'src', 'transport', '*.cpp');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'src', 'gazeboserver', '*.cpp');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'src', 'gazeboserver', 'gazebomsghandler', '*.cpp');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'export', 'include', 'gazebotransport', '*.hpp');

    % For custom message
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'src', 'gazebocustom', '*.cpp');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'src', 'gazebocustom', 'gazebocustommsghandler' , '*.cpp');
    src{end+1} = fullfile( srcTransportPath, 'gazebotransport', 'include', 'gazebotransport', 'gazebocustom');
    if nargin > 1
        % If gazebogenmsg called provided custom message dependencies path
        src{end+1} = fullfile( customMessagePath, 'customMsgHandlers', 'src');
        src{end+1} = fullfile( customMessagePath, 'customMsgHandlers', 'include');
        src{end+1} = fullfile( customMessagePath, 'pluginUtilities', 'src');
        src{end+1} = fullfile( customMessagePath, 'pluginUtilities', 'world');
        src{end+1} = fullfile( customMessagePath, 'ProtoFiles', '*.proto');
    end

    dst = {};
    dst{end+1} = fullfile( desDir , 'include', 'gazebotransport', 'transport');
    dst{end+1} = fullfile( desDir , 'include', 'gazebotransport', 'gazeboserver');
    dst{end+1} = fullfile( desDir , 'src', 'transport');
    dst{end+1} = fullfile( desDir , 'src', 'gazeboserver');
    dst{end+1} = fullfile( desDir , 'src', 'gazeboserver', 'gazebomsghandler');
    dst{end+1} = fullfile( desDir , 'export', 'include', 'gazebotransport');

    % For custom message
    dst{end+1} = fullfile( desDir , 'src', 'gazebocustom');
    dst{end+1} = fullfile( desDir , 'src', 'gazebocustom', 'gazebocustommsghandler');
    dst{end+1} = fullfile( desDir , 'include', 'gazebotransport', 'gazebocustom');
    if nargin > 1
        % If gazebogenmsg called provided custom message dependencies path
        dst{end+1} = fullfile( desDir, 'src', 'gazebocustom', 'gazebocustommsghandler');
        dst{end+1} = fullfile( desDir, 'include', 'gazebotransport', 'gazebocustom','gazebocustommsghandler');
        dst{end+1} = fullfile( desDir, 'src', 'gazeboplugin');
        dst{end+1} = fullfile( desDir, 'world');
        dst{end+1} = fullfile( desDir, 'msgsproto');
    end

    % copy the transport source code over
    for idx = 1:numel(src)
        mkdirIfNeeded(dst{idx});
        copyfile(src{idx}, dst{idx},'f');
    end

    % GazeboPlugin.cpp source file is created based on template. This file
    % is created for custom message while calling gazebogenmsg but for
    % non-custom message support, it is created while packaging source code
    % below.
    dstFolderPath = fullfile( desDir, 'src', 'gazeboplugin');
    if nargin <= 1
        robotics.gazebo.internal.customMessageSupport.createGazeboPlugin(...
            [],fullfile( dstFolderPath, 'GazeboPlugin.cpp'));
    end

    % avoid CoSimMsgs proto conflicts if same name available in custom
    % message proto file. Added package name and syntax in '.proto' file
    % this was previously done on server side
    CoSimMsgsPref = char(robotics.gazebo.internal.topic.Category.CoSimMsgsPrefix);

    cosimProtoSrc = fullfile( srcTransportPath, 'gazebotransport', 'msgsproto','CoSimMsgs.proto');
    testProtoSrc = fullfile( srcTransportPath, 'gazebotransport', 'msgsproto','TestMsgs.proto');
    CoSimMsgsContent = fileread(cosimProtoSrc);
    TestMsgsContent = fileread(testProtoSrc);

    % add package name and proto syntax
    msgPrefixContent = ['syntax="proto2";',newline,'package ',CoSimMsgsPref,';',newline];
    CoSimMsgsContentNew = [ msgPrefixContent, newline, CoSimMsgsContent];
    TestMsgsContentNew = [ msgPrefixContent, newline, TestMsgsContent];

    % create new CoSimMsgs.proto with extension
    createFile(CoSimMsgsContentNew,fullfile(desDir,'msgsproto'),...
               [char(robotics.gazebo.internal.topic.Category.CoSimMsgsPrefix),'.CoSimMsgs.proto']);

    % create new TestMsgs.proto with extension
    createFile(TestMsgsContentNew,fullfile(desDir,'msgsproto'),...
               [char(robotics.gazebo.internal.topic.Category.CoSimMsgsPrefix),'.TestMsgs.proto']);

    zip(packagePath,desDir);

    if nargout == 1
        pathInfo = what(desDir);
        output = pathInfo.path;
    end

end

function mkdirIfNeeded(p)
    if ~exist(p, 'dir')
        mkdir(p);
    end
end

function createFile(messageString, dstDir, fileName)
    mkdirIfNeeded(dstDir);
    fileID = fopen(fullfile(dstDir,fileName), 'wt');
    cleanup = onCleanup(@()fclose(fileID));
    fwrite(fileID, messageString);
end
