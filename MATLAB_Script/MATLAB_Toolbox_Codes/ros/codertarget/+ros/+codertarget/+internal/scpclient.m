classdef scpclient < handle
%This class is for internal use only. It may be removed in the future.

%SCPCLIENT Transfer files using SCP.
%
% obj = scpclient(hostname, username, password, <port>, <flags>)
%
% Input parameters within <> are optional.


% Copyright 2013-2018 The MathWorks, Inc.

    properties (Dependent, Access = public)
        Hostname
        Port
    end

    properties (Access = public)
        Flags
    end

    properties (Hidden, Access = private)
        IpNode
        Credentials
        Host
        PuttyRootDir
    end

    properties (Access = private)
        PSCP
        CopyCmd1
        CopyCmd2
    end


    methods
        function obj = scpclient(hostname, username, password, port, flags)
        % Constructor
            narginchk(1, 5);
            if (nargin < 4)
                port = 22;
            end
            if (nargin < 5)
                flags = '';
            end
            obj.IpNode = ros.codertarget.internal.ipnode(hostname, port);
            obj.Credentials = ros.codertarget.internal.credentials(username, password);
            obj.Flags = flags;
            obj.Host = [obj.Credentials.Username '@' obj.IpNode.Hostname];
            obj.PuttyRootDir = fullfile(toolboxdir('idelink'), ...
                                        'foundation', 'hostapps');
            if ispc
                obj.PSCP = fullfile(obj.PuttyRootDir, 'pscp.exe');
                obj.CopyCmd1 = ['"', obj.PSCP, '" -pw "', ...
                                obj.Credentials.Password, '" -P ', ...
                                num2str(obj.IpNode.Port), ' -q ', obj.Flags];
            else
                obj.PSCP = 'scp';
                obj.CopyCmd1 = ['sshpass -p "', ...
                                obj.Credentials.Password, '" ', obj.PSCP ' -P ', ...
                                num2str(obj.IpNode.Port), ' ', obj.Flags];
            end
            obj.CopyCmd2 = [obj.Host ':'];
        end
    end

    methods
        function value = get.Hostname(obj)
            value = obj.IpNode.Hostname;
        end

        function value = get.Port(obj)
            value = obj.IpNode.Port;
        end

        % Set method for Protocol
        function set.Flags(obj, flags)
            validateattributes(flags, {'char'}, {}, 'SCPCLIENT', 'flags');
            obj.Flags = flags;
        end
    end

    % User visible methods
    methods(Access = public)
        function putFile(obj, source, destination)
        % [status, result] = copyfile(obj, source, destination) copies
        % the file or directory source on the host system to the file
        % or directory destination on the remote host. The resulting
        % status and standard output are returned.
            copycmd = [...
                obj.CopyCmd1, obj.decoratecmd(source), ...
                ' ', ...
                obj.CopyCmd2, obj.decoratecmd(destination)];
            [status, result] = system(copycmd);
            if status ~= 0
                error('utils:scpclient:putFile', ...
                      'Error executing command: %s', result);
            end
        end

        function getFile(obj, source, destination)
        % [status, result] = getFile(obj, source, destination) copies
        % the file or directory source on the remote host to the file
        % or directory destination on the host. The resulting status
        % and standard output are returned.
            copycmd = [obj.CopyCmd1, obj.CopyCmd2, ...
                       obj.decoratecmd(source), ...
                       ' ', ...
                       obj.decoratecmd(destination)];
            [status, result] = system(copycmd);
            if status ~= 0
                error('utils:scpclient:putFile', ...
                      'Error executing command: %s', result);
            end
        end
    end

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
    end
end

%[EOF]
