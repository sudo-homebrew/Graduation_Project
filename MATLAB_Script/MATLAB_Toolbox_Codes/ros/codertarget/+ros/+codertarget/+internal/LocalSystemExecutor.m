classdef LocalSystemExecutor < ros.codertarget.internal.SystemInterface
    %SYSTEMEXECUTOR Implements SystemInterface for localhost
    
    %   Copyright 2021 The MathWorks, Inc.
    methods 
        function output = system(~,command,~)
            [status,output] = system(command); 
            if status ~= 0
                error(message('ros:utilities:util:SystemError',output));
            end
        end
        
        function putFile(~,localFile,remoteFile)
            if nargin < 3
                % Copy localFile to user directory to provide same
                % behavior as LinuxSystemInterface
                remoteFile = i_userdir;
            end
            copyfile(localFile,remoteFile);
        end
        
        function getFile(~,remoteFile,localFile)
            if nargin < 3
                % Copy remoteFile to user directory to provide same
                % behavior as LinuxSystemInterface
                localFile = i_userdir;
            end
            copyfile(remoteFile,localFile);
        end
        
        function deleteFile(~,fileName)
            delete(fileName);
        end
        
        function d = dir(~,fileSpec)
            d = dir(fileSpec);
        end
        
        function openShell(~)
            switch computer('arch')
                case 'win64'
                    system('&');
                case 'glnxa64'
                    % This may fail if gnome-terminal is not installed
                    system('gnome-terminal');
                case 'maci64'
                    system('open -a Terminal ~/');
            end
        end

        function cmd = commandWithSudo(~,cmd)
        end
    end
    
    methods (Static)
        function p = fullfile(varargin)
            p = fullfile(varargin{:});
        end
    end
end

%% Internal functions
function home = i_userdir
if ispc
    home = getenv('USERPROFILE');
else
    home = getenv('HOME');
end
end 