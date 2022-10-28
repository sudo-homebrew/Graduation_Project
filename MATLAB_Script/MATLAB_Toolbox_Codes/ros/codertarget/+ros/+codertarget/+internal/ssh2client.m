classdef ssh2client < handle
%This class is for internal use only. It may be removed in the future.

%SSH2CLIENT Execute commands on a remote host via SSH protocol.
%
% obj = ssh2client(hostname, username, password, <port>)
%
% Input parameters within <> are optional.

% Copyright 2015-2018 The MathWorks, Inc.

    properties (Hidden, Access = public)
        BatchMode = true;
        SevenZipRoot  % Needed for copying directories with tar
    end

    properties (Dependent, Access = public)
        Hostname
        Port
    end

    properties (Hidden, Access = private)
        IpNode
        Credentials
    end

    properties (Access = private)
        Key
    end

    properties (Dependent, Access = private)
        CommandLine
    end

    properties (Constant, Access = private)
        %LibraryPathWin - Environment variable for library search path on Windows
        %   This applies to both win32 and win64
        LibraryPathWin = 'PATH'

        %LibraryPathLinux - Environment variable for library search path on Linux
        LibraryPathLinux = 'LD_LIBRARY_PATH'

        %LibraryPathMac - Environment variable for library search path on Mac
        LibraryPathMac = 'DYLD_LIBRARY_PATH'
    end


    methods
        % Constructor
        function obj = ssh2client(hostname, username, password, port)
            narginchk(1,4);
            if nargin < 4
                port = 22;
            end

            % Validate / store device address
            try
                obj.IpNode = ros.codertarget.internal.ipnode(hostname,port);
                obj.Credentials = ros.codertarget.internal.credentials(username,password);
            catch ME
                throwAsCaller(ME);
            end

            % Connect to client
            obj.setupLIBSSHRuntime;
            obj.Key = ssh2client_mex('connect',...
                                     obj.IpNode.Hostname,...
                                     obj.Credentials.Username,...
                                     obj.Credentials.Password,...
                                     obj.IpNode.Port);
            if obj.Key < 0
                error('shared_linuxservices:utils:SSH2ConnectionError',...
                      'Error connecting to SSH server at %s', obj.IpNode.Hostname);
            end
        end
    end

    methods
        % Set and get methods
        function value = get.Hostname(obj)
            value = obj.IpNode.Hostname;
        end

        function value = get.Port(obj)
            value = obj.IpNode.Port;
        end

        % Get method for CommandLine
        function CommandLine = get.CommandLine(obj)
            if ispc
                % Use -batch to disable any interactive prompts. This way,
                % we will never hang.
                plinkExe = fullfile(toolboxdir('idelink'),'foundation',...
                                    'hostapps','plink.exe');
                CommandLine = ['"', plinkExe '" -ssh -pw "', ...
                               obj.Credentials.Password, '" -P ', num2str(obj.IpNode.Port)];
                if obj.BatchMode
                    CommandLine = [CommandLine, ' -batch'];
                end
            else
                CommandLine = ['sshpass -p "', ...
                               obj.Credentials.Password, '" ssh -p ', num2str(obj.IpNode.Port)];
            end
        end
    end % Set and get methods

    % User visible methods
    methods(Access = public)
        function [stdout,stderr,status] = execute(obj,cmd,throwEx)
        %EXECUTE Execute command on the remote host
        %   output = EXECUTE(obj, cmd, cmdprefix) executes cmd
        %   on the remote host. The resulting standard output is returned.
        %
        %   For consistent behavior on every target device, we enforce
        %   that the command is always executed with the bash shell.
        %   Common other shells are SH and ZSH.

            if nargin < 3
                throwEx = true;
            else
                validateattributes(throwEx,{'logical','double'},...
                                   {'binary'},'','throwEx');
            end

            % Make sure that command is executed with bash shell
            % It is important that you use single quotes instead of double
            % quotes around the command to prevent bash from expanding
            % $! and $0 macros.
            % See http://unix.stackexchange.com/questions/135070/why-doesnt-source-work-when-i-call-bash-c

            % If the command contains single quotes, e.g. in sed calls,
            % that will conflict with the enclosing single quotes of bash
            % -c. Use a trick to replace all ' with '\'' (see
            % https://muffinresearch.co.uk/bash-single-quotes-inside-of-single-quoted-strings/)
            cmd = strrep(cmd, '''', '''\''''');
            cmd = ['env LC_ALL=C bash -c ''' cmd ''''];

            [status,stdout,stderr] = ssh2client_mex('exec',obj.Key,cmd);
            if throwEx && status ~= 0
                error('utils:sshclient:system', ...
                      ['Error executing command "%s". Details:\n\n',...
                       'STDERR: %s\nSTDOUT: %s'],cmd,stderr,stdout);
            end
        end

        function openShell(obj)
        % Open an SSH shell to target
            if ispc
                ros.codertarget.internal.putty.openShell('ssh', ...
                                                         [obj.Hostname ...
                                    ' -l ' obj.Credentials.Username ...
                                    ' -pw "' obj.Credentials.Password '"'])
            else
                ros.codertarget.internal.nixssh.openShell(obj.Hostname, ...
                                                          obj.Credentials.Username);
            end
        end

        function fastGetFile(obj, source, dest)
        % copyUsingTar(obj, source, dest)
        %
            if ~exist(obj.SevenZipRoot,'dir')
                error('sshclient:utils:system', ...
                      'Set SevenZipRoot property to the directory where 7-zip is installed.');
            end

            % Work with both 7z.exe and 7za.exe
            Exe7zip = fullfile(obj.SevenZipRoot,'7za.exe');
            if ~exist(Exe7zip,'file')
                Exe7zip = fullfile(obj.SevenZipRoot,'7z.exe');
            end
            if ~exist(Exe7zip,'file')
                error('sshclient:utils:system', ...
                      'Set SevenZipRoot property to the directory where 7-zip is installed.');
            end

            %[status, output] = system([obj.CommandLine, ' -C ', ...
            %  obj.Host, ' ' obj.decoratecmd(cmd) ' | tar xf - -C ', dest]);
            % -: output to stdout
            % dereference hard and soft links. Compress when transferring.
            cmd = ['tar c --dereference --hard-dereference -O ', source, ' 2> /dev/null'];
            [status, output] = system([obj.CommandLine, ' -C ', ...
                                obj.Host, ' ' obj.decoratecmd(cmd) ...
                                ' | "' Exe7zip '" x -y -si -ttar -o"' dest '"']);
            if (status ~= 0) || ~isempty(regexp(output, '^Access denied', 'once'))
                error('utils:sshclient:system', ...
                      'Error executing command: %s', output);
            end
        end

        function status = scpPutFile(obj,source,dest,permissions)
        % The 'permissions' argument is optional. If not specified permissions
        % default to -rw-r--r-- (644). For ease specify permissions argument say,
        % -r--r--r-- (444) as 'bin2dec(100100100)'.
            narginchk(3,4);

            % Escaped spaces in the target path have to be removed for
            % ssh2client_mex
            dest = strrep(dest, '\ ', ' ');

            if nargin < 4
                status = ssh2client_mex('putfile',obj.Key,source,dest);
            else
                status = ssh2client_mex('putfile',obj.Key,source,dest,permissions);
            end
            switch status
              case -1
                error('shared_linuxservices:utils:SSH2ConnectionError',...
                      'Error connecting to SSH server at %s', obj.IpNode.Hostname);
              case -2
                error('utils:sshclient:system', ...
                      'Cannot open file %s for reading.',source);
              case -3
                error('shared_linuxservices:utils:SCPSessionError', ...
                      'Cannot open file %s for writing.',dest);
              case -4
                error('shared_linuxservices:utils:SCPWriteFileError', ...
                      'Error writing to file %s.',dest);
            end
        end

        function status = scpGetFile(obj,source,destination)
            status = ssh2client_mex('getfile',obj.Key,source,destination);
            switch status
              case -1
                error('shared_linuxservices:utils:SSH2ConnectionError',...
                      'Error connecting to SSH server at %s.',obj.IpNode.Hostname);
              case -2
                error('utils:sshclient:system', ...
                      'Cannot open file %s for writing.',destination);
              case -3
                error('shared_linuxservices:utils:SCPSessionError', ...
                      'Cannot open %s for reading.',source);
              case -4
                error('shared_linuxservices:utils:SCPReadFileError', ...
                      'Error reading file %s.',source);
            end
        end
    end % methods(Access = public)

    % Internal methods for use in unit testing
    methods (Access = ?matlab.unittest.TestCase)
        function [stdout, stderr, status] = executeRaw(obj, cmd)
        %executeRaw Execute command over SSH without any pre- or postprocessing

            [status, stdout, stderr] = ssh2client_mex('exec', obj.Key, cmd);
        end

        function executeRawAsynchronous(obj, cmd)
        %executeRawAsynchronous Execute command over SSH and return right away
        %   The process will continue to run until completion, but the
        %   function call returns right away.
        %   See http://stackoverflow.com/questions/29142/getting-ssh-to-execute-a-command-in-the-background-on-target-machine.

        % Use nohup so process is disowned and continues to run
            asyncCmd = ['nohup ' cmd];

            % Redirect stdout and stderr to /dev/null
            asyncCmd = [asyncCmd ' > /dev/null 2>&1'];

            % Put in background
            asyncCmd = [asyncCmd ' &'];

            obj.executeRaw(asyncCmd);
        end
    end

    %% Static methods used for MEX management
    methods (Static, Hidden)
        function reset()
            ssh2client_mex('reset');
        end

        function unlock()
            ssh2client_mex('unlock');
        end
    end

    methods (Static, Hidden)
        function libPath = getLibraryPathVariable
        %getLibraryPathVariable Get environment variable that stores the library search path
        %   Note that this variable is different for each platform.

            arch = computer('arch');
            switch (arch)
              case 'win64'
                libPath  = ros.codertarget.internal.ssh2client.LibraryPathWin;
              case 'glnxa64'
                libPath  = ros.codertarget.internal.ssh2client.LibraryPathLinux;
              case 'maci64'
                libPath  = ros.codertarget.internal.ssh2client.LibraryPathMac;
            end
        end
    end

    methods (Static, Access = ?matlab.unittest.TestCase)
        function setupLIBSSHRuntime
        %setupLIBSSHRuntime Add LIBSSH2 run-time libraries to system library search path
        %   Note that we ship the 3p/libssh2 binaries in the robotslros/bin folder.
        %   Since that folder is not on the library search path by default, add it.

            arch = computer('arch');

            % The platform-specific binary folder are under
            % robotslros/bin/<arch>. robotslros is 4 folder levels above
            % this file, so using 4 fileparts path reductions.
            topdir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            binArch = fullfile(topdir,'bin',arch);

            libpathvar = ros.codertarget.internal.ssh2client.getLibraryPathVariable;
            add2env(libpathvar,binArch);
        end
    end

    methods (Static, Access = private)
        function cmd = decoratecmd(varargin)
        % cmd = decoratecmd(varargin)
        % Creates a command string by concatenating the input strings.
        % Places double quotes around the final command string.
        %
        % Example:
        % cmd = obj.decoratecmd('ls', '-al')
        %
        % ans = "ls -al"
            cmd = '"';
            for i = 1:nargin
                cmd = [cmd, varargin{i}]; %#ok<AGROW>
                if (i ~= nargin)
                    cmd = [cmd, ' ']; %#ok<AGROW>
                end
            end
            cmd = [cmd, '"'];
        end
    end % methods(Static, Access = public)
end % classdef

%% Local functions

%--------------------------------------------------------------------------
function add2env(libpathvar, addVal)
    curVal = getenv(libpathvar);
    if ~contains(curVal,addVal)
        setenv(libpathvar, [addVal pathsep curVal]);
    end
end
%[EOF]
