classdef PackageInfo
% This class is for internal use only. It may be removed in the future.

% PackageInfo for a given package

% Copyright 2019-2021 The MathWorks, Inc.
    properties
        BuildType        %CMakeBuild Type must be a valid CMake Build Type
        PackageName      %Name must be a valid C variable name
        Description      %Added as is to the package.xml
        Version          %Version of the package
        License          %Optional license
        Dependencies     %Other dependencies
        MsgDependencies  %Other dependencies for messages
        BuildDependencies %Build dependencies required
        RunDependencies  %Run dependencies required
        PkgConfigModules % Modules found using pkg-config
        MaintainerEmail  %Email for maintainer
        MaintainerName   %Name of the maintainer
        CppNodeName      %Node name
        CppLibraryName   %Library name
        SourceFiles      %Source files to be added
        OtherFiles       %Other files to be added from Buildinfo
        IncludeFiles     %Include files to be added
        LibSourceFiles   %Lib Source files to be added
        LibIncludeFiles  %Lib Include files to be added
        MessageFiles     %Message package source files to be added
        ServiceFiles     %Service package source files to be added
        ActionFiles      %Action package source files to be added
        MATLABFiles      %Generated MATLAB files to be copied
        MATLABDestPath   %Path postfix where the MATLAB files are to be installed
        MsgClassFiles    %Generated MATLAB message class files to be copied
        MsgClassDestPath %Path postfix where the MATLAB message class files are to be installed
        LibFormat        %Lib format accepted by CMake (STATIC, SHARED, MODULE, empty)
        IncludeDirectories % Include directories
        LibraryDirectories % Library directories
        Libraries        % Extra libraries to add
        CppFlags         %Flags including defines common to both lib and exe
        LinkerFlags      %Linker flags common to both lib and exe
        CUDAFlags        %CUDA flags common to both lib and exe

        ImportedLibraries % Declare libraries as IMPORTED, so that
                          % we have the full absolute path to the libraries
                          % during the CMake execution. This ensures that
                          % even library names that do not begin with lib*
                          % are picked up correctly. See
                          % http://stackoverflow.com/questions/33165270/force-cmake-to-use-the-full-library-path
        
    end

    %% Setters for the class
    methods
        function h = set.BuildType(h,val)
            h.BuildType = validatestring(val, ...
                {'Debug','Release','RelWithDebInfo','MinSizeRel'}, ...
                '','''Build Configuration > BuildType''');
        end
        function h = set.PackageName(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            if ~isvarname(value)
                error(message('ros:utilities:util:NotValidPackageName',value));
            end
            h.PackageName = convertStringsToChars(value);
        end
        function h = set.Description(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.Description = convertStringsToChars(value);
        end
        function h = set.Version(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.Version = convertStringsToChars(value);
        end
        function h = set.License(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.License = convertStringsToChars(value);
        end
        function h = set.Dependencies(h, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            h.Dependencies = convertStringsToChars(value);
        end
        function h = set.MsgDependencies(h, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            h.MsgDependencies = convertStringsToChars(value);
        end
        function h = set.MaintainerEmail(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.MaintainerEmail = convertStringsToChars(value);
        end
        function h = set.MaintainerName(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.MaintainerName = convertStringsToChars(value);
        end
        function h = set.CppNodeName(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.CppNodeName = convertStringsToChars(value);
        end
        function h = set.CppLibraryName(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.CppLibraryName = convertStringsToChars(value);
        end
        function value = checkForFiles(~, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            for i = 1:numel(value)
                if ~isfile(value{i})
                    error(message('ros:utilities:util:FileDoesNotExist',value{i}));
                end
            end
            value = convertStringsToChars(value);
        end
        function h = set.SourceFiles(h, value)
            h.SourceFiles = checkForFiles(h, value);
        end
        function h = set.OtherFiles(h, value)
            h.OtherFiles = checkForFiles(h, value);
        end
        function h = set.IncludeFiles(h, value)
            h.IncludeFiles = checkForFiles(h, value);
        end
        function h = set.LibSourceFiles(h, value)
            h.LibSourceFiles = checkForFiles(h, value);
        end
        function h = set.LibIncludeFiles(h, value)
            h.LibIncludeFiles = checkForFiles(h, value);
        end
        function h = set.MessageFiles(h, value)
            h.MessageFiles = checkForFiles(h, value);
        end
        function h = set.ActionFiles(h, value)
            h.ActionFiles = checkForFiles(h, value);
        end
        function h = set.ServiceFiles(h, value)
            h.ServiceFiles = checkForFiles(h, value);
        end
        function h = set.MATLABFiles(h, value)
            h.MATLABFiles = checkForFiles(h, value);
        end
        function h = set.MATLABDestPath(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.MATLABDestPath = convertStringsToChars(value);
        end
        function h = set.MsgClassFiles(h, value)
            h.MsgClassFiles = checkForFiles(h, value);
        end
        function h = set.MsgClassDestPath(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.MsgClassDestPath = convertStringsToChars(value);
        end
        function h = set.LibFormat(h,value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.LibFormat = convertStringsToChars(value);
        end
        function h = set.IncludeDirectories(h, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            h.IncludeDirectories = value;
        end
        function h = set.LibraryDirectories(h, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            h.LibraryDirectories = value;
        end
        function h = set.Libraries(h, value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            h.Libraries = value;
        end
        function h = set.CppFlags(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.CppFlags = convertStringsToChars(value);
        end
        function h = set.LinkerFlags(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.LinkerFlags = convertStringsToChars(value);
        end
        function h = set.CUDAFlags(h, value)
            validateattributes(value,{'char','string'},{'nonempty'});
            h.CUDAFlags = convertStringsToChars(value);
        end
        function h = set.PkgConfigModules(h, value)
            value = convertStringsToChars(value);
            validateattributes(value,{'cell'},{'nonempty'});
            cellfun(@(x)validateattributes(x,{'char','string'},{'nonempty'}),value);
            h.PkgConfigModules = value;
        end

        function h = set.ImportedLibraries(h,value)
            value = value(~cellfun(@isempty,value)); %remove empty values
            validateattributes(value,{'cell'},{'nonempty'}); %check if value is a non empty cell array
            h.ImportedLibraries = convertStringsToChars(value);
        end
        
    end

    %% Private methods
    methods (Hidden, Static, Access = {?ros.internal.ROSProjectBuilder})
        function addParamsToParser(parser)
        %adds to the given inputParser the required params
        %this can be called by the containing class and reuse the
        %parameters
            parser.addRequired('packageName',@(x)validateattributes(x,{'char','string','ros.internal.PackageInfo'},{'nonempty'}));
            parser.addParameter('buildtype','Release',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('description','TODO',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('version','0.0.0',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('license','TODO',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('dependencies','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('msgdependencies','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('buildDependencies','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('runDependencies','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('maintainerEmail','TODO@EMAIL.COM',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('maintainerName','TODO',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('cppNodeName','',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('cppLibraryName','',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('sourceFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('otherFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('includeFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('libSourceFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('libIncludeFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('messageFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('actionFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('serviceFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('matlabFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('matlabDestPath','m/',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('msgClassFiles','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('msgClassDestPath','m/',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('libFormat','',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('includeDirs','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('libDirs','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('libs','',@(x)(cellfun(@(y)validateattributes(y,{'char','string'},{'nonempty'}),x)));
            parser.addParameter('cppFlags','',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('linkerFlags','',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('CUDAFlags','',@(x)validateattributes(x,{'char','string'},{'nonempty'}));
            parser.addParameter('pkgConfigModules','',@(x)validateattributes(x,{'cell'},{'nonempty'}));
            parser.addParameter('importedLibs','',@(x)validateattributes(x,{'cell'},{'nonempty'}));
        end
    end

    %% Public methods
    methods
        function h = PackageInfo(varargin)
        %PackageInfo(packageName) Creates info with given packageName
        %PackageInfo(packageName, 'description', DESCRIPTION
        %...

        %If the parser is not passed in, then this creates a parsers
        %and parses the inputs. Otherwise, it is assumed we have the
        %parser with parsed inputs available as the first argument.
            narginchk(1, nargin);
            parser = varargin{1};
            if ~isa(parser,'inputParser')
                parser = inputParser;
                parser.FunctionName = 'PackageInfo';
                ros.internal.PackageInfo.addParamsToParser(parser);
                parser.parse(varargin{:});
            end

            if isa(parser.Results.packageName,'ros.internal.PackageInfo')
                h = parser.Results.packageName; %copy the packageInfo
                return;
            end

            h.PackageName = parser.Results.packageName;
            if ~isempty(parser.Results.buildtype)
                h.BuildType = parser.Results.buildtype;
            end
            if ~isempty(parser.Results.description)
                h.Description = parser.Results.description;
            end
            if ~isempty(parser.Results.version)
                h.Version = parser.Results.version;
            end
            if ~isempty(parser.Results.license)
                h.License = parser.Results.license;
            end
            if ~isempty(parser.Results.dependencies)
                h.Dependencies = parser.Results.dependencies;
            end
            if ~isempty(parser.Results.msgdependencies)
                h.MsgDependencies = parser.Results.msgdependencies;
            end
            if ~isempty(parser.Results.buildDependencies)
                h.BuildDependencies = parser.Results.buildDependencies;
            end
            if ~isempty(parser.Results.runDependencies)
                h.RunDependencies = parser.Results.runDependencies;
            end
            if ~isempty(parser.Results.maintainerEmail)
                h.MaintainerEmail = parser.Results.maintainerEmail;
            end
            if ~isempty(parser.Results.maintainerName)
                h.MaintainerName = parser.Results.maintainerName;
            end
            if ~isempty(parser.Results.cppNodeName)
                h.CppNodeName = parser.Results.cppNodeName;
            end
            if ~isempty(parser.Results.cppLibraryName)
                h.CppLibraryName = parser.Results.cppLibraryName;
            end
            if ~isempty(parser.Results.sourceFiles)
                h.SourceFiles = parser.Results.sourceFiles;
            end
            if ~isempty(parser.Results.otherFiles)
                h.OtherFiles = parser.Results.otherFiles;
            end
            if ~isempty(parser.Results.includeFiles)
                h.IncludeFiles = parser.Results.includeFiles;
            end
            if ~isempty(parser.Results.libSourceFiles)
                h.LibSourceFiles = parser.Results.sourceFiles;
            end
            if ~isempty(parser.Results.libIncludeFiles)
                h.LibIncludeFiles = parser.Results.includeFiles;
            end
            if ~isempty(parser.Results.messageFiles)
                h.MessageFiles = parser.Results.messageFiles;
            end
            if ~isempty(parser.Results.serviceFiles)
                h.ServiceFiles = parser.Results.serviceFiles;
            end
            if ~isempty(parser.Results.matlabFiles)
                h.MATLABFiles = parser.Results.matlabFiles;
            end
            if ~isempty(parser.Results.matlabDestPath)
                h.MATLABDestPath = parser.Results.matlabDestPath;
            end
            if ~isempty(parser.Results.msgClassFiles)
                h.MsgClassFiles = parser.Results.msgClassFiles;
            end
            if ~isempty(parser.Results.msgClassDestPath)
                h.MsgClassDestPath = parser.Results.msgClassDestPath;
            end
            if ~isempty(parser.Results.libFormat)
                h.LibFormat = parser.Results.libFormat;
            end
            if ~isempty(parser.Results.includeDirs)
                h.IncludeDirectories = parser.Results.includeDirs;
            end
            if ~isempty(parser.Results.libDirs)
                h.LibraryDirectories = parser.Results.libDirs;
            end
            if ~isempty(parser.Results.libs)
                h.Libraries = parser.Results.libs;
            end
            if ~isempty(parser.Results.cppFlags)
                h.CppFlags = parser.Results.cppFlags;
            end
            if ~isempty(parser.Results.linkerFlags)
                h.LinkerFlags = parser.Results.linkerFlags;
            end
            if ~isempty(parser.Results.CUDAFlags)
                h.CUDAFlags = parser.Results.CUDAFlags;
            end
            if ~isempty(parser.Results.pkgConfigModules)
                h.PkgConfigModules = parser.Results.pkgConfigModules;
            end

            if ~isempty(parser.Results.importedLibs)
                h.ImportedLibraries = parser.Results.importedLibs;
            end
        end

        function cmd = createPkgCreateCommand(h)
        %Builds the command line based on the values available
        %the resulting command can be passed to ros2 cmd
        %
        %Note: we have since added more capabilities and is here for
        %reference
        %
        %usage:
        % cmd = h.createPkgCreateCommand
        % ros2(cmd)

            cmd = ['pkg create ' h.PackageName];
            if ~isempty(h.Description)
                cmd = [cmd ' --description "' h.Description '"'];
            end
            if ~isempty(h.License)
                cmd = [cmd ' --license "' h.License '"'];
            end
            if ~isempty(h.MaintainerEmail)
                cmd = [cmd ' --maintainer-email "' h.MaintainerEmail '"'];
            end
            if ~isempty(h.MaintainerName)
                cmd = [cmd ' --maintainer-name "' h.MaintainerName '"'];
            end
            if ~isempty(h.CppNodeName)
                cmd = [cmd ' --cpp-node-name "' h.CppNodeName '"'];
            end
            if ~isempty(h.CppLibraryName)
                cmd = [cmd ' --cpp-library-name "' h.CppLibraryName '"'];
            end
            if ~isempty(h.Dependencies)
                depStr = sprintf('%s ',h.Dependencies{:});
                cmd = [cmd ' --dependencies ' depStr];
            end
        end
    end
end
