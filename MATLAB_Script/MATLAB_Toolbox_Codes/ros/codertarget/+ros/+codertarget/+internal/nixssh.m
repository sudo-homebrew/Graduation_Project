classdef nixssh < handle
%This class is for internal use only. It may be removed in the future.

%NIXSSH Command line interface to SSH client on *nix systems.
%
% openShell(type,varargin) Open an ssh session. Optional
% arguments are passed to ssh command line as is.
%
% Examples:
%
% 1. Open a session to host at 192.168.2.1 with username "pi" and
% password "raspberry":
%
% nixssh.openShell('ssh','172.28.195.236 -l pi -pw "raspberry"')
%

% Copyright 2015-2018 The MathWorks, Inc.

    methods (Static)
        function openShell(hostname,username,options)
        % Open an SSH shell to target
            if ispc
                error('nixssh:UnsupportedPlatform',...
                      'nixssh only works on Linux and MAC platform. Use putty for Windows.');
            end

            if nargin < 2
                userargs = '';
            else
                userargs = [' -l ' username];
            end

            if nargin < 3
                options = '';
            end

            % Call ssh command line utility
            try
                system(['ssh ' hostname ' ' userargs ' ' options]);
            catch
                % Once the user exits the shell return gracefully.
            end
        end
    end
end % classdef

%[EOF]
