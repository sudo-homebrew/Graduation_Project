classdef sshclient < handle
%This class is for internal use only. It may be removed in the future.

%SSHCLIENT Execute commands on a remote Unix host via SSH protocol.
%
% obj = sshclient(hostname, username, password, <port>)
%
% Input parameters within <> are optional.


% Copyright 2013-2018 The MathWorks, Inc.

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
        Host
    end

    properties (Access = private)
        PuttyRootDir
    end

    properties (Dependent, Access = private)
        CommandLine
    end


    methods
        % Constructor
        function obj = sshclient(hostname, username, password, port)
            narginchk(1, 4);
            if nargin < 4
                port = 22;
            end
            obj.IpNode = ros.codertarget.internal.ipnode(hostname, port);
            obj.Credentials = ros.codertarget.internal.credentials(username, password);
            obj.Host = [obj.Credentials.Username '@' obj.IpNode.Hostname];
            obj.PuttyRootDir = fullfile(toolboxdir('idelink'), ...
                                        'foundation', 'hostapps');
        end

        % Get method for CommandLine
        function CommandLine = get.CommandLine(obj)
            if ispc
                % Use -batch to disable any interactive prompts. This way,
                % we will never hang.
                CommandLine = ['"', fullfile(obj.PuttyRootDir, 'plink'), '" -ssh -pw "', ...
                               obj.Credentials.Password, '" -P ', num2str(obj.IpNode.Port)];
                if obj.BatchMode
                    CommandLine = [CommandLine, ' -batch'];
                end
            else
                CommandLine = ['sshpass -p "', ...
                               obj.Credentials.Password, '" ssh -p ', num2str(obj.IpNode.Port)];
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
    end % Set and get methods

    % User visible methods
    methods(Access = public)
        function msg = connect(obj, cmd)
        % Execute the command 'echo Connection successful' to test
        % ssh or rsh connection.
        %
        % If the RSA key of the remote host is not cached on the host
        % system, a yes or no prompt will be issued to the user to
        % continue with the connection
            if (nargin < 2)
                cmd = 'echo Connection successful';
            end
            cmdprefix = 'echo y|';
            batchMode = obj.BatchMode;
            obj.BatchMode = false;
            try
                msg = obj.executeCommand(cmd, cmdprefix, '');
                obj.BatchMode = batchMode;
            catch EX
                obj.BatchMode = batchMode;
                throwAsCaller(EX);
            end
        end

        function output = executeCommand(obj, cmd, cmdPrefix, sshOptions)
        % output = system(obj, cmd, cmdprefix) executes cmd
        % on the remote host. The resulting standard output is returned.
            if (nargin < 3)
                cmdPrefix = '';
            end
            if (nargin < 4)
                sshOptions = '';
            end
            [status, output] = system([cmdPrefix, ' ', obj.CommandLine, ' ', ...
                                sshOptions, ' ' obj.Host, ' ' obj.decoratecmd(cmd)]);
            if (status ~= 0) || ~isempty(regexp(output, '^Access denied', 'once'))
                error('utils:sshclient:system', ...
                      'Error executing command: %s', output);
            end
        end

        function openShell(obj)
        % Open an SSH shell to target
            system(['"' fullfile(obj.PuttyRootDir, 'putty.exe') ...
                    '" -ssh ', obj.Hostname, ' &']);
        end

        function copyUsingTar(obj, source, dest)
        % copyUsingTar(obj, source, dest)
        %

        % -: output to stdout
        % dereference hard and soft links. Compress when transferring.
            if ~exist(obj.SevenZipRoot, 'dir')
                error('utils:sshclient:system', ...
                      'Set SevenZipRoot property to the directory where 7-zip is installed.');
            end
            cmd = ['tar c --dereference --hard-dereference -O ', source, ' 2> /dev/null'];
            %% Linux
            %[status, output] = system([obj.CommandLine, ' -C ', ...
            %  obj.Host, ' ' obj.decoratecmd(cmd) ' | tar xf - -C ', dest]);
            [status, output] = system([obj.CommandLine, ' -C ', ...
                                obj.Host, ' ' obj.decoratecmd(cmd) ...
                                ' | "' fullfile(obj.SevenZipRoot, '7z.exe') '" x -y -si -ttar -o"' dest '"']);
            if (status ~= 0) || ~isempty(regexp(output, '^Access denied', 'once'))
                error('utils:sshclient:system', ...
                      'Error executing command: %s', output);
            end
        end
    end % methods(Access = public)

    methods(Static, Access = private)
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

%[EOF]
