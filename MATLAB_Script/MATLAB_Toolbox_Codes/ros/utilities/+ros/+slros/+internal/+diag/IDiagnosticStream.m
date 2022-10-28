classdef IDiagnosticStream < handle
%This class is for internal use only. It may be removed in the future.

%DiagnosticStream Base class for diagnostic message streams
%   The diagnostic information might be displayed in different
%   contexts, e.g. MATLAB or Simulink. This base class abstracts the
%   interface and the subclasses can decide how to display the
%   information.

%   Copyright 2016-2020 The MathWorks, Inc.

    properties (Constant)
        %ValidRunModes - Valid settings for the RunMode property
        ValidRunModes = {'test', 'build'}
    end

    properties
        %RunMode - The run mode for the diagnostic stream
        %   Possible values:
        %   'test'  - For testing purposes, print out all information,
        %             warnings, and errors.
        %   'build' - At build time, do not print information and convert
        %             all high-priority warnings to errors.
        %
        %   Default: 'test'
        RunMode = 'test'
    end

    properties (SetAccess = protected)
        %ErrorCount - Number of errors that have been encountered since the stream has been opened
        ErrorCount
    end

    %% Abstract interface methods
    %  The "msg" input should always be a message object. Subclasses
    %  can decide internally if they want to convert this to an MException
    %  or MSLException object.To get proper behavior of the "suggested
    %  actions", use MSLException.
    methods (Abstract)
        %open Open the diagnostic stream. Call this before the report* functions.
        open(obj, diagnosticName, varargin)

        %close Close the diagnostic stream. Call this after you are done with your report
        close(obj)

        %reportInfo Report informational message about diagnostic progress
        %   The user should not have to react to this type of message.
        %   The INFO input is a character vector.
        reportInfo(obj, info)

        %reportWarning Report warning message
        %   A warning is a problem that the user might have to address. At
        %   testing time, this might not be an issue, but at build time,
        %   there is a possibility that this result in an error.
        reportWarning(obj, msg)

        %reportHighPriorityWarning Report high-priority warning
        %   A high-priority warning is a problem that the user has to
        %   address as soon as possible to prevent later errors. Use
        %   high-priority warnings if there is a near-certain likelihood that this
        %   condition will result in an error condition at build time.
        reportHighPriorityWarning(obj, msg)

        %reportError Report error message
        %   An error stops the current workflow and the user cannot
        %   proceed. Depending on the workflow in which the diagnostics are
        %   executed, messages might be warnings in one workflow, but
        %   errors in another.
        reportError(obj, msg)
    end

    %% Concrete methods
    methods
        function set.RunMode(obj, runMode)
        %set.RunMode Custom setter for RunMode property

            validateattributes(runMode, {'char'}, {'nonempty','row'}, '', 'RunMode');
            validRunMode = validatestring(runMode, obj.ValidRunModes, '', 'RunMode');
            obj.RunMode = validRunMode;
        end
    end

    methods (Static, Access = protected)
        function validateInfoInput(info)
        %validateInfoInput Validate the input to the reportInfo function

            validateattributes(info, {'char'}, {'nonempty', 'row'});
        end

        function validateMessageInput(msg)
        %validateMessageInput Validate the message input to the report* functions
        %   We expect this to be a proper message object.

            validateattributes(msg, {'message'}, {'nonempty', 'scalar'});
        end
    end

end
