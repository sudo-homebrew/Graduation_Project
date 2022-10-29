classdef DiagnosticViewerStream < ros.slros.internal.diag.IDiagnosticStream
%This class is for internal use only. It may be removed in the future.

%DiagnosticViewerStream Stream diagnostic messages to Simulink's Diagnostic Viewer
%   This class is responsible for streaming diagnostic messages to
%   Simulink's Diagnostic Viewer (aka Message Viewer). No messages are
%   displayed on the MATLAB command-line.
%   Methods are implemented from the DiagnosticStream base class.
%
%   See also CommandLineStream.

%   Copyright 2016-2020 The MathWorks, Inc.

    properties (Access = private)
        %Stage - The diagnostic viewer stage object
        %   All messages, warnings, and errors are displayed in the
        %   associated stage section until this object is deleted.
        Stage = []

        %ModelName - The name of the model associated with the stream
        ModelName

        %BringViewerInFocus - Bring diagnostic viewer in focus for each diagnostic run?
        BringViewerInFocus = false
    end

    methods
        function obj = DiagnosticViewerStream(modelName, bringInFocus)
        %DiagnosticViewerStream Create a Simulink Diagnostic Viewer stream
        %   Associate a Simulink model with name MODELNAME with this
        %   stream.
        %   If BRINGINFOCUS is TRUE, then the diagnostic viewer will be
        %   brought into the foreground (or opened if it is currently
        %   closed).

            validateattributes(modelName, {'char'}, {'nonempty','row'}, 'DiagnosticViewerStream', 'modelName');
            validateattributes(bringInFocus, {'logical'}, {'scalar'}, 'open', 'bringInFocus');

            obj.ModelName = modelName;
            obj.BringViewerInFocus = bringInFocus;
        end

        function delete(obj)
        %delete Delete stage on object destruction

            obj.deleteStage;
        end
    end

    %% Implement DiagnosticStream interface
    methods
        function open(obj, diagnosticName)
        %open Open the diagnostic stream. Call this before the report* functions.
        %   Create a stage with DIAGNOSTICNAME.
        %   This will open a new tab with MODELNAME in the diagnostic
        %   viewer (or use an existing one if a tab with the same name
        %   exists). DIAGNOSTICNAME determines the name of the viewer
        %   section.

            validateattributes(diagnosticName, {'char'}, {'nonempty','row'}, 'open', 'stageName');

            % Reset error count
            obj.ErrorCount = 0;

            % Create a new stage
            obj.Stage = sldiagviewer.createStage(diagnosticName, 'ModelName', obj.ModelName);

            if obj.BringViewerInFocus && ros.slros.internal.isDisplayAvailable
                try
                    % Bring diagnostic viewer into focus if display is
                    % connected.
                    % The call to slmsgviewer will fail if MATLAB runs in the
                    % nodisplay mode.
                    slmsgviewer.Instance().show();
                catch
                end
            end

        end

        function close(obj)
        %close Close the diagnostic stream. Call this after you are done with your report

            obj.deleteStage;
        end

        function reportInfo(obj, info)
        %reportInfo Report informational message about diagnostic progress

        % At build time do not report information
            if isequal(obj.RunMode, 'build')
                return;
            end

            obj.validateInfoInput(info);
            sldiagviewer.reportInfo(info);
        end

        function reportWarning(obj, msg)
        %reportWarning Report warning message

        % At build time do not report normal warnings
            if isequal(obj.RunMode, 'build')
                return;
            end

            obj.validateMessageInput(msg);

            diag = MSLException([], msg);
            sldiagviewer.reportWarning(diag);
        end

        function reportHighPriorityWarning(obj, msg)
        %reportHighPriorityWarning Report high-priority warning

            obj.validateMessageInput(msg);

            if isequal(obj.RunMode, 'build')
                % At build time, convert high-priority warnings into errors
                obj.reportError(msg);
                return;
            end

            diag = MSLException([], msg);
            Simulink.output.highPriorityWarning(diag);
        end

        function reportError(obj, msg)
        %reportError Report error message
            obj.validateMessageInput(msg);
            % Increment error count
            obj.ErrorCount = obj.ErrorCount + 1;
            diag = MSLException([], msg);
            sldiagviewer.reportError(diag);
        end
    end

    methods (Access = private)
        function deleteStage(obj)
            if isempty(obj.Stage)
                return;
            end

            obj.Stage.delete;
            obj.Stage = [];
        end
    end

end
