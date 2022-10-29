classdef DotPrinter
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2021 The MathWorks, Inc.

%Creates a timer that prints '.' in the background
    properties (Hidden)
        InfoStr
        Timer
        CleanUp
    end

    methods (Hidden)
        function cleanUpCallback(h)
            if isvalid(h.Timer)
                stop(h.Timer);
                delete(h.Timer);
            end
            if ~isempty(h.InfoStr)
                doneMsg = message('ros:utilities:util:Done');
                fprintf(doneMsg.getString());
            end
            fprintf('\n');
        end
    end
    methods
        function h = DotPrinter(infoStrId, varargin)
            if nargin > 0
                msg = message(infoStrId, varargin{:});
                h.InfoStr = msg.getString();
            else
                h.InfoStr = [];
            end
            if ~isempty(h.InfoStr)
                fprintf(h.InfoStr);
            end
            %check if timer is available
            h.Timer = timer('ExecutionMode','fixedDelay','TimerFcn',@(x,y)fprintf('.'));
            h.CleanUp = onCleanup(@h.cleanUpCallback);
            start(h.Timer);
        end
    end
end
