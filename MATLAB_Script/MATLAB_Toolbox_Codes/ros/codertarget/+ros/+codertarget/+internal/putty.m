classdef putty < handle
%This class is for internal use only. It may be removed in the future.

%PUTTY Command line interface to PuTTY SSH, telnet, serial and rlogin
% client.
%
% openShell(type,varargin) Open a PuTTY session with specified type.
% The session type can be 'ssh', 'telnet', 'rlogin' and 'raw'. Optional
% arguments are passed to PuTTY command line as is.
%
% Examples:
%
% 1. Open a session to host at 192.168.2.1 with username "pi" and
% password "raspberry":
%
% putty.openShell('ssh','172.28.195.236 -l pi -pw "raspberry"')
%
% 2. Open a serial console to hardware connected to COM4:
%
% putty.openShell('serial','COM4')
%
%

% Copyright 2015-2018 The MathWorks, Inc.

    methods (Static)
        function openShell(type,options)
        % Open an SSH shell to target
            if ~ispc
                error('nixssh:UnsupportedPlatform',...
                      'Putty only works on Windows platform. Use nixssh for other platform.');
            end
            type = validatestring(type,{'ssh','serial','raw','telnet','rlogin'});
            puttyExe = fullfile(toolboxdir('idelink'),'foundation',...
                                'hostapps','putty.exe');
            switch type
              case 'ssh'
                system(['"' puttyExe '" -ssh ' options ' &']);
              case 'serial'
                system(['"' puttyExe '" -serial ' options ' &']);
              case 'telnet'
                system(['"' puttyExe '" -telnet ' options ' &']);
              case 'rlogin'
                system(['"' puttyExe '" -rlogin ' options ' &']);
              case 'raw'
                system(['"' puttyExe '" -raw ' options ' &']);
            end
        end
    end
end % classdef

%[EOF]
