classdef CommandLineStream < ros.slros.internal.diag.IDiagnosticStream
%This class is for internal use only. It may be removed in the future.

%CommandLineStream Stream diagnostic messages to MATLAB command-line
%   This class is responsible for streaming diagnostic messages to
%   MATLAB command-line.
%   Methods are implemented from the DiagnosticStream base class.
%
%   See also DiagnosticViewerStream.

%   Copyright 2016-2020 The MathWorks, Inc.

    properties (Access = private)
        %HasMSLException - Is MSLException class available?
        %   If the user does not have Simulink installed, MSLException will
        %   not be available
        HasMSLException = false
    end

    methods
        function obj = CommandLineStream
        %CommandLineStream Standard constructor

        % An MSLException gives the most functionality, but in
        % absence of that, use a standard MException object for basic
        % functionality.
            if exist('MSLException', 'file')
                obj.HasMSLException = true;
            else
                obj.HasMSLException = false;
            end

        end
    end

    %% Implement DiagnosticStream interface
    methods
        function open(obj, diagnosticName)
        %open Open the diagnostic stream. Call this before the report* functions.

            validateattributes(diagnosticName, {'char'}, {'nonempty','row'}, 'open', 'stageName');

            % Reset error count
            obj.ErrorCount = 0;

            % Display the diagnostic name
            hyphenLine = repmat('-', 1, length(diagnosticName));
            disp(hyphenLine);
            disp(diagnosticName);
            disp(hyphenLine);
            disp(' ');
        end

        function close(~)
        %close Close the diagnostic stream. Call this after you are done with your report
        end

        function reportInfo(obj, info)
        %reportInfo Report informational message about diagnostic progress

        % At build time do not report information
            if isequal(obj.RunMode, 'build')
                return;
            end

            obj.validateInfoInput(info);

            % Use standard display for informational messages
            disp(info);
        end

        function reportWarning(obj, msg)
        %reportWarning Report warning message

        % At build time do not report normal warnings
            if isequal(obj.RunMode, 'build')
                return;
            end

            obj.validateMessageInput(msg);

            % Throw a warning.
            diag = obj.createException(msg);
            robotics.internal.warningNoBacktrace(diag.identifier, strrep(diag.getReport, '\', '\\'));
        end

        function reportHighPriorityWarning(obj, msg)
        %reportHighPriorityWarning Report high-priority warning

            obj.validateMessageInput(msg);

            if isequal(obj.RunMode, 'build')
                % At build time, convert high-priority warnings into errors
                obj.reportError(msg);
                return;
            end

            % Throw a warning. Note that there is no concept of
            % high-priority warnings in MATLAB.
            diag = obj.createException(msg);

            % At test time, just display standard warning
            robotics.internal.warningNoBacktrace(diag.identifier, strrep(diag.getReport, '\', '\\'));
        end

        function reportError(obj, msg)
        %reportError Report error message

            obj.validateMessageInput(msg);

            % Increment error count
            obj.ErrorCount = obj.ErrorCount + 1;

            % Throw a standard error. Use throwAsCaller to remove the
            % stack.
            diag = obj.createException(msg);
            throwAsCaller(diag);
        end
    end

    methods (Access = private)
        function ex = createException(obj, msg)
        %createException Create exception from message object

            if obj.HasMSLException
                ex = MSLException([], msg);
            else
                ex = MException(msg);
            end
        end
    end

end
