function createProtoSharedLib(folderPathList,libraryName)
%This function is for internal use only. It may be removed in the future.
%
%createProtoSharedLib generates Shared Library for protobuf generated
%sources files ( *.pb.cc and *.pb.h )
% input :
%               @param folderPathList                    Folder path of proto file present
%               @param libraryName                       Shared library name
%

%   Copyright 2019-2020 The MathWorks, Inc.

% output folder path of shared library
    outDirPath = folderPathList.installFolderPath;

    % create loadlibrary header
    libraryNameCapital = [upper(libraryName),'_H'];
    headerString = ['//Copyright 2019 The MathWorks, Inc.',newline,...
                    '#ifndef LIB',libraryNameCapital,newline,...
                    '#define LIB',libraryNameCapital,newline,newline,newline...
                    '#endif',newline];

    newHeaderFile = fopen(fullfile(outDirPath,['lib',char(libraryName),'.h']), 'wt');
    fprintf(newHeaderFile, '%s\n\n', headerString);
    fclose(newHeaderFile);

    % dependent protobuf cpp and header file path
    protoHeaderInclude = [' -I"',folderPathList.protoFilesFolderPath,'" '];

    % attach all proto .pb.cc file path ( these generated on protoc
    % exe call )
    if(ispc)
        protoFileString = [' "',fullfile(folderPathList.protoFilesFolderPath,'*.pb.cc'),'" '];
    else
        protoCCList = dir(fullfile(folderPathList.protoFilesFolderPath,'*.pb.cc'));

        % create string of *.pb.cc files present in folder
        protoFileString = [];
        for idx = 1:length(protoCCList)
            protoFileString = [protoFileString,' "',fullfile(folderPathList.protoFilesFolderPath,protoCCList(idx).name),'" ']; %#ok<AGROW>
        end
    end

    % path of protobuf shared libs for Windows ( .lib )
    protoLibPath = fullfile(matlabroot, 'toolbox', 'shared', 'robotics', 'externalDependency','libprotobuf','lib');
    % path of protobuf headers for Windows ( .h/.hpp )
    protoHeaderPath = fullfile(matlabroot, 'toolbox', 'shared', 'robotics', 'externalDependency','libprotobuf','include');
    % Protobuf shared library present at matlab bin folder
    protoBinPath = fullfile(matlabroot, 'bin', computer('arch'));

    if ismac
        % mac support
        % shared library path and name
        msgProtoLibPath = ['  "',fullfile(outDirPath,['lib',char(libraryName),'.dylib']),'"   '];
        % command string to build shared library
        protoLibCreate = ['clang++ -std=c++11 -stdlib=libc++  '....
                          '  ',protoFileString, protoHeaderInclude,' -I"',protoHeaderPath,'" -L"',protoBinPath,'" ',...
                          '  -lprotobuf3 '...
                          '  -dynamiclib -o ',msgProtoLibPath,' '];

    elseif isunix
        % Linux support
        % shared library path and name
        msgProtoLibPath = ['  "',fullfile(outDirPath,['lib',char(libraryName),'.so']),'"   '];
        % command string to build shared library
        protoLibCreate = ['gcc -std=c++11 -Wall -O3 -march=native -shared -o ',msgProtoLibPath,...
                          ' -fPIC ',protoFileString, protoHeaderInclude, ' -I"',protoHeaderPath,'" -L"',protoBinPath,'" ',...
                          ' -lprotobuf3 '];

    elseif ispc
        % windows support

        % retrieve compiler environment
        mexComplierConfig = mex.getCompilerConfigurations;
        mexDetails = mexComplierConfig.Details;
        envList = mexDetails.SetEnv;

        pathDetails = char(extractBetween( envList,'set PATH=',newline));
        includeDetails = char(extractBetween( envList,'set INCLUDE=',newline));
        libDetails = char(extractBetween( envList,'set LIB=',newline));
        libPathDetails = char(extractBetween( envList,'set LIBPATH=',newline));

        % set compiler environment
        setenv('PATH',pathDetails);
        setenv('INCLUDE',[includeDetails,';',protoHeaderPath,';',folderPathList.protoFilesFolderPath]);
        setenv('LIB',libDetails);
        setenv('LIBPATH',libPathDetails);

        % shared library name
        libName = ['/lib',char(libraryName),'.lib'];
        dllName = ['/lib',char(libraryName),'.dll'];

        % command string to build shared library
        protoLibCreate = ['cl /LD /EHsc /MD /DLL  /DPROTOBUF_USE_DLLS /DCREATE_DLL '...
                          ' /Fo"',outDirPath,'"\ ',protoFileString,...
                          ' /I \INCLUDE /I"',folderPathList.protoFilesFolderPath,'" ',...
                          '  /link /LIBPATH:"',protoLibPath,'" ',...
                          '  libprotobuf3.lib  ',...
                          ' /OUT:"',outDirPath,dllName,'" ',...'
                          ' /IMPLIB:"',outDirPath,libName,'" '];

    end

    % generate shared library
    system(protoLibCreate);

    % safe-check to exist gazebogenmsg if shared lib is not created in the
    % unknown case
    if ismac
        if( ~exist(fullfile(outDirPath,['lib',char(libraryName),'.dylib']),'file'))
            error(message('robotics:robotgazebo:gazebogenmsg:SharedLibNotCreated'));
        end
    elseif isunix
        if( ~exist(fullfile(outDirPath,['lib',char(libraryName),'.so']),'file'))
            error(message('robotics:robotgazebo:gazebogenmsg:SharedLibNotCreated'));
        end
    elseif ispc
        if( ~exist(fullfile(outDirPath,['lib',char(libraryName),'.dll']),'file') ...
            || ~exist(fullfile(outDirPath,['lib',char(libraryName),'.lib']),'file'))
            error(message('robotics:robotgazebo:gazebogenmsg:SharedLibNotCreated'));
        end
    end

end
