classdef LinuxSystemInterface < ros.codertarget.internal.SystemInterface
%This class is for internal use only. It may be removed in the future.

%LinuxSystemInterface Define an interface for system operations supported on Linux hardware
%
%   LinuxSystemInterface is a abstract class which
%   defines the interface methods for communicating with hardware running
%   a Linux distribution.
%
%   The following methods are implemented by this class:
%
%   output = system(OBJ,COMMAND,SUDO) Called when a system command gets
%   executed on the hardware. SUDO is an optional argument.
%
%   putFile(obj,localSrc,remoteDest) Copy localSrc on the host computer to
%   remoteDest on the hardware.
%
%   getFile(obj,remoteSrc,localDest) Copy the remoteSrc on the hardware to
%   to the localDest on the host computer.
%
%   a = dir(obj,fileSpec) Directory listing for fileSpec. The output a is a
%   structure containing name, folder, isdir and bytes fields.

%   Copyright 2015-2021 The MathWorks, Inc.

    properties (Abstract, Access = protected)
        Ssh
    end

    methods (Access = public)
        %% System
        function output = system(obj,command,sudo)
        % output = system(hw,command,<sudo>) executes the command on
        % the hardware and returns the resulting output. If the
        % optional input, 'sudo', is specified as the third argument,
        % the command is executed as super user.
            i_validateCharArg(command,'command');
            if nargin > 2
                sudo = validatestring(sudo,{'sudo'},'','sudo');
                sudo = [sudo, ' '];
            else
                sudo = '';
            end

            % Execute command on the remote host via SSH
            try
                output = execute(obj.Ssh,[sudo command]);
            catch ME
                throwAsCaller(ME);
            end
        end

        %% putFile
        function putFile(obj,localFile,remoteFile,throwEx)
        % PUTFILE(hw,localFile,remoteFile) copies the localFile on the
        % host computer to the remoteFile on the hardware. Path names
        % and wildcards may be used to specify the localFile. The
        % remoteFile argument must either be a directory or a name
        % representing a single file, depending on the localFile
        % specification.
        %
        % The remoteFile argument is optional. If not specified, the
        % localFile is copied to the user's home directory.
        %
        % See also dir, getFile and putFile.

            i_validateCharArg(localFile,'localFile');

            if nargin > 2
                i_validateCharArg(remoteFile,'remoteFile');

                % Run an echo on the target device to expand ~ and other
                % variables, such as $HOME
                remoteFile = strtrim(execute(obj.Ssh,['echo ' remoteFile],false));
            else
                % Default to user's home folder
                remoteFile = strtrim(execute(obj.Ssh,'echo ~',false));
            end
            remoteFile = i_handleSpaces(remoteFile);

            if (nargin < 4)
                throwEx = true;
            end

            % Retrieve local file information
            locFileInfo = dir(localFile);
            if numel(locFileInfo) == 0 && throwEx
                error('shared_linuxservices:utils:PutFileError',...
                      'Cannot identify %s. No such file or directory.',localFile);
            end

            % Determine remoteFile type
            remoteFile = strtrim(remoteFile);
            remFileIsDir = i_exist(obj.Ssh,remoteFile,'dir');
            remFileIsFile = i_exist(obj.Ssh,remoteFile,'file');
            if numel(locFileInfo) > 1 && ~remFileIsDir && remFileIsFile
                error('shared_linuxservices:utils:GetFileError',...
                      ['Cannot identify %s. No such directory. ',...
                       'When localFile argument specifies multiple files or a directory, ', ...
                       'the remoteFile argument must be an existing directory.'],remoteFile);
            elseif numel(locFileInfo) > 1 && ~remFileIsDir && ~remFileIsFile
                i_mkRemoteDir(obj.Ssh,remoteFile);
            end
            remFileIsDir = i_exist(obj.Ssh,remoteFile,'dir');

            % Copy each file/directory recursively
            for k = 1:numel(locFileInfo)
                thisFile = locFileInfo(k);
                if thisFile.isdir
                    if ~isequal(thisFile.name,'.') && ~isequal(thisFile.name,'..')
                        newFolder = i_fullLnxFile(remoteFile,thisFile.name);
                        i_mkRemoteDir(obj.Ssh,newFolder);
                        putFile(obj,...
                                fullfile(thisFile.folder,thisFile.name),...
                                newFolder,...
                                false);
                    end
                else
                    try
                        if remFileIsDir
                            remFileName = i_fullLnxFile(remoteFile,thisFile.name);
                        else
                            remFileName = remoteFile;
                        end
                        scpPutFile(obj.Ssh,...
                                   fullfile(thisFile.folder,thisFile.name),...
                                   remFileName);
                    catch ME
                        throwAsCaller(ME);
                    end
                end
            end
        end

        %% getFile
        function getFile(obj,remoteFile,localFile,throwEx)
        % GETFILE(hw,remoteFile,localFile) copies remoteFile on the
        % hardware to the localFile in the host computer. Path names
        % and wildcards may be used to specify the remoteFile. The
        % localFile argument must either be a directory or a name
        % representing a single file, depending on the remoteFile
        % specification.
        %
        % The localFile argument is optional. If not specified, the
        % remoteFile is copied to the current directory.
        %
        % See also dir, putFile and system.
            i_validateCharArg(remoteFile,'remoteFile');
            remoteFile = i_handleSpaces(remoteFile);
            if (nargin < 3)
                localFile = pwd;
            else
                i_validateCharArg(localFile,'localFile');
            end

            if (nargin < 4)
                throwEx = true;
            end

            % Copy remote file(s) to local file system
            remFileInfo = dir(obj,remoteFile);
            if numel(remFileInfo) == 0 && throwEx
                error('shared_linuxservices:utils:GetFileError',...
                      'Cannot identify %s. No such file or directory.',remoteFile);
            end

            % Determine localFile type
            localFile = strtrim(localFile);
            if numel(remFileInfo) > 1 && ~exist(localFile,'dir') ...
                    && exist(localFile,'file')
                error('shared_linuxservices:utils:GetFileError',...
                      ['Cannot identify %s. No such directory. ',...
                       'When remoteFile argument specifies multiple files or a directory, ', ...
                       'the localFile argument must be an existing directory.'],localFile);
            elseif numel(remFileInfo) > 1 && ~exist(localFile,'dir') ...
                    && ~exist(localFile,'file')
                i_mkdir(localFile);
            end

            % Copy files recursively
            for k = 1:numel(remFileInfo)
                thisFile = remFileInfo(k);
                if thisFile.isdir
                    % recursively copy the directory
                    if ~isequal(thisFile.name,'.') && ~isequal(thisFile.name,'..')
                        newDir = fullfile(localFile,thisFile.name);
                        i_mkdir(newDir);
                        getFile(obj,...
                                i_fullLnxFile(thisFile.folder,thisFile.name),...
                                newDir,...
                                false); % Don't throw an error if file does not exist
                    end
                else
                    try
                        if isfolder(localFile)
                            localFileName = fullfile(localFile,thisFile.name);
                        else
                            localFileName = localFile;
                        end
                        scpGetFile(obj.Ssh,...
                                   i_fullLnxFile(thisFile.folder,thisFile.name),...
                                   localFileName);
                    catch ME
                        throwAsCaller(ME);
                    end
                end
            end
        end

        %% dir
        function d = dir(obj,fileSpec)
        %dir List directory.
        %     dir directory_name lists the files in a directory. Pathnames and
        %     asterisk wildcards may be used. A single asterisk in the path touching
        %     only file separators will represent exactly one directory name. A
        %     single asterisk at the end of an input will represent any filename. An
        %     asterisk followed or preceded by characters will resolve to zero or
        %     more characters. A double asterisk can only be used in the path and
        %     will represent zero or more directory names. It cannot touch a character
        %     other than a file separator. For example, dir *.m lists all files with
        %     a .m extension in the current directory. dir */*.m lists all files with
        %     a .m extension exactly one directory under the current directory.
        %     dir **/*.m lists all files with a .m extension zero or more directories
        %     under the current directory.
        %
        %     D = dir(hw,'directory_name') returns the results in an M-by-1
        %     structure with the fields:
        %         name        -- Filename
        %         folder      -- Absolute path
        %         bytes       -- Number of bytes allocated to the file
        %         isdir       -- 1 if name is a directory and 0 if not
        %
        %     See also getFile, putFile and system.
            if nargin < 2
                fileSpec = '~';
            end
            i_validateCharArg(fileSpec,'fileSpec');
            fileSpec = i_fullLnxFile(strtrim(fileSpec));
            fileSpec = i_handleSpaces(fileSpec);
            if fileSpec(1) ~= '/' && fileSpec(1) ~= '~'
                rootDir = strtrim(system(obj,'echo ~'));
            else
                rootDir = '';
            end
            rootDir = i_handleSpaces(rootDir);
            d = [];

            % Stat the file spec. Dereference symbolic links
            out = execute(obj.Ssh,['stat -L --printf=''%n:&:%F:&:%s:&:%y\n'' ' fileSpec],false);
            if isempty(out)
                return;
            end

            % We treat directories differently
            tmp = regexp(strtrim(out),'\n','split');
            if numel(tmp) == 1
                tmp1 = regexp(tmp{1},':&:','split');
                if strfind(tmp1{2},'directory')
                    d = i_dirDirectory(obj.Ssh,tmp1{1});
                    return;
                end
            end
            for k = 1:numel(tmp)
                tmp1 = regexp(tmp{k},':&:','split');
                [p,n,ext] = fileparts(tmp1{1});
                d(end+1,1).name = [n ext]; %#ok<AGROW>
                d(end).folder = fileparts(i_fullLnxFile(rootDir,p,[n ext]));
                if strfind(tmp1{2},'directory')
                    d(end).isdir = true;
                    d(end).bytes = 0;
                else
                    d(end).isdir = false;
                    d(end).bytes = str2double(tmp1{3});
                end
            end
        end

        %% deleteFile
        function deleteFile(obj,fileName)
        % DELETEFILE(hw,fileName) deletes a file on the hardware.
            i_validateCharArg(fileName,'fileName');
            fileName = i_handleSpaces(fileName);

            try
                execute(obj.Ssh,['rm -fr ' fileName]);
            catch ME
                throwAsCaller(ME);
            end
        end

        %% openShell
        function openShell(obj)
        % OPENSHELL(hw) Open an SSH shell to target
            openShell(obj.Ssh);
        end
    end
end

% Internal functions
function i_validateCharArg(file,argName)
    validateattributes(file,{'char'},{'nonempty','row'},'',argName);
end

function file = i_fullLnxFile(varargin)
% Convert paths to Linux convention.

% Replace backslash with forward slash (if the user provides windows path
% separators), but ignore escaped spaces, since
% that backslash is used as an escape character, not as a path separator.
% Escaped spaces might occur if the path contains file or folder names with
% spaces.
    file = regexprep(varargin{1}, '\\[^ ]', '/');

    for i = 2:nargin
        file = [file, '/', varargin{i}]; %#ok<AGROW>
    end
    file = strrep(file, '//', '/');
    file = regexprep(file, '/$', '');  %remove trailing slash
end

function d = i_dirDirectory(ssh,directory)
    d = [];
    directory = i_handleSpaces(strtrim(directory));
    out = execute(ssh,['stat -L --printf=''%n:&:%F:&:%s\n'' ' ...
                       directory '/.* ' directory '/*'],false);
    if isempty(out)
        return;
    end
    tmp = regexp(strtrim(out),'\n','split');
    for k = 1:numel(tmp)
        tmp1 = regexp(tmp{k},':&:','split');
        if numel(tmp1) < 3
            break;
        end
        [p,n,ext] = fileparts(tmp1{1});
        d(end+1).name = [n ext]; %#ok<AGROW>
        d(end).folder = p;
        if strfind(tmp1{2},'directory')
            d(end).isdir = true;
            d(end).bytes = 0;
        else
            d(end).isdir = false;
            d(end).bytes = str2double(tmp1{3});
        end
    end
end

function i_mkdir(p)
    [s,msg,msgid] = mkdir(p);
    if s ~= 0 && ~isequal(msgid,'MATLAB:MKDIR:DirectoryExists')
        error(msgid,msg);
    end
end

function ret = i_exist(ssh,fileSpec,fileType)
% Replacement for exist for file on the remote file system
    ret = 0;
    fileSpec = i_handleSpaces(fileSpec);
    switch fileType
      case 'dir'
        out = execute(ssh,['[ -d ' fileSpec ' ] && echo 1 || echo 0'],false);
        if isequal(strtrim(out),'1')
            ret = 7;
        end
      case 'file'
        out = execute(ssh,['[ -f ' fileSpec ' ] && echo 1 || echo 0'],false);
        if isequal(strtrim(out),'1')
            ret = 2;
        end
    end
end

function i_mkRemoteDir(ssh,remoteFile)
    try
        remoteFile = i_handleSpaces(remoteFile);
        execute(ssh,['mkdir -p ' remoteFile]);
    catch ME
        throwAsCaller(ME);
    end
end

function escapedPath = i_handleSpaces(filePath)
%i_handleSpaces Handle spaces in file or folder path
%   In the system command that we send over SSH, spaces in the
%   path to a file or a folder need to be escaped with a backslash.

% Escape spaces. For example, this will change "file_w s" into "file_w\ s".
    escapedPath = strrep(filePath, ' ', '\ ');

    % If the spaces were already escaped, we should undo the
    % double-escaping. Since this function might be called multiple times with
    % the same names, we want to prevent double escapes. If the user calls
    % i_handleSpaces with "file_w\ s", the return will be an unchanged
    % "file_w\ s".
    escapedPath = strrep(escapedPath, '\\ ', '\ ');
end
