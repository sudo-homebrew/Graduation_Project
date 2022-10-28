classdef TextProgressBar < handle
%This function is for printing the percentage progress for running process

%   Copyright 2020 The MathWorks, Inc.   
    properties
        InfoStr
        LastMsgLen = 0;
    end
    
    methods 
        function obj = TextProgressBar(infoStrId, varargin)
            % To Initialize the TextProgressBar with a message from the
            % message catalog
            if nargin > 0
                msg = message(infoStrId, varargin{:});
                obj.InfoStr = msg.getString();
            else
                obj.InfoStr = [];
            end
            if ~isempty(obj.InfoStr)
                fprintf(obj.InfoStr);
            end
            obj.LastMsgLen = length(obj.InfoStr);
        end
		
        function obj = printProgress(obj, currentValue, totalValue)
            % To update and print the percentage progress along with the message catalog.
            percentProgress = floor(currentValue ./ totalValue * 100);
            fprintf('%s', char(8*ones(1, obj.LastMsgLen))); %delete previous text.
            infoStr = sprintf('[%d/%d] %s %d%%',currentValue, totalValue, obj.InfoStr, percentProgress);
            fprintf('%s', infoStr);
            if currentValue == totalValue
                fprintf('%s', char(8*ones(1, 4))); %replace 100% by Done
                doneMsg = message('ros:utilities:util:Done');
                fprintf(doneMsg.getString());
                fprintf('\n');
                %reset to avoid deleting previous lines
                obj.LastMsgLen = 0;
                return
            end
            obj.LastMsgLen = length(infoStr);
        end

        function printMessage(~, msg)
            %To print multiple messages in a new line after a process is finished
            fprintf("\n%s",msg);
        end
    end
end