classdef ROSException < MException
    %This class is for internal use only. It may be removed in the future.
    
    %ROSException Custom exception object for Java exceptions
    %   To avoid exposing (and printing) the whole Java error stack
    %   when exceptions are thrown, this class encapsulates the exception
    %   causes and only make them available through custom interface
    %   functions.
    %   This ROS exception class can be used similarly to the normal
    %   MException object, but call addCustomCause() instead of addCause() and
    %   getCustomReport() instead of getReport().
    %
    %
    %   ROSException methods:
    %      addCustomCause        - Add a custom cause to this exception
    %      getCustomReport       - Print a report of all custom causes
    %      saveCustomStackTrace  - Save the stack trace before deletion
    %
    %
    %   Example:
    %
    %      % Create an exception
    %      ex = MException('a:b:c', 'Error 1');
    %
    %      % Create the ROS exception object
    %      newEx = ros.internal.ROSException('a:b:c', 'Error 2');
    %      newEx = newEx.addCustomCause(ex));
    %
    %      % Print out an exception report
    %      newEx.getCustomReport
    
    %   Copyright 2020 The MathWorks, Inc.
    
    properties (Access = ?matlab.unittest.TestCase)
        %CustomException - Custom exception object
        %   This exception object stores all custom causes as normal causes
        CustomException;
    end
    
    methods
        function obj = ROSException(varargin)
            %ROSException Constructor
            
            % To properly handle backslashes, escape them in the message 
            % The message input is fed directly into sprintf,
            % backslashes in the message can lead to undesired behavior.
            
            switch(nargin)
                case 1
                    % Called with (message) object input.
                    validateattributes(varargin{1}, {'message'}, {'nonempty'}, ...
                        'ROSException', 'message');
                case 2
                    % Called with (id, msg) input
                    validateattributes(varargin{1}, {'char'}, {'nonempty'}, ...
                        'ROSException', 'identifier');
                    validateattributes(varargin{2}, {'char'}, {'nonempty'}, ...
                        'ROSException', 'msg');
                    
                    % Escape backslashes to avoid warnings
                    % on MException creation
                    varargin{2} = strrep(varargin{2}, '\', '\\');
                    
                    % If backslashes were doubly escaped, fix that
                    varargin{2} = strrep(varargin{2}, '\\\\', '\\');
                otherwise
                    % Called with (id, msg, v1, ...) input
                    validateattributes(varargin{1}, {'char'}, {'nonempty'}, ...
                        'ROSException', 'identifier');
                    validateattributes(varargin{2}, {'char'}, {'nonempty'}, ...
                        'ROSException', 'msg');                    
                    
                    % Don't escape backslashes, because the additional
                    % inputs indicate that message should be interpreted
                    % as proper input to sprintf
            end
            
            obj = obj@MException(varargin{:});
            obj.CustomException = obj;
        end
        
        function newObj = addCustomCause(obj, cause)
            %addCustomCause Add a custom cause to this exception
            %   This function will not alter the cause list for this
            %   exception object, but will store all causes in the object 
            %   stored in the CustomException property.
            %
            %   Example:
            %
            %     try
            %     catch ex
            %        newEx = ros.internal.ROSException( ...
            %             message('ros:mlros:SomeError'));
            %        throw(newEx.addCustomCause(ex));
            %     end
            
            if ~isa(cause, 'MException')
                return;
            end
            
            if ~isa(cause, 'ros.internal.ROSException')
                obj.CustomException = obj.CustomException.addCause(cause);
            else
                obj.CustomException = obj.CustomException.addCause(cause.CustomException);
            end
            newObj = obj;
        end
        
        function newObj = saveCustomStackTrace(obj)
            %saveCustomStackTrace Save the stack trace before deletion
            %   To avoid the stack deletion when using the throwAsCaller
            %   syntax, use this function to save the stack trace
            %   internally.
            %
            %   Example:
            % 
            %     try
            %     catch ex
            %        ex = saveCustomStackTrace(ex);
            %        throwAsCaller(ex);
            %     end
            %
            %     ...
            %
            %     ex = MException.last;
            %     if isa(ex, 'ros.internal.ROSException')
            %        % Print out full stack trace and list of causes
            %        ex.getCustomReport;
            %     else
            %        ex.getReport;
            %     end 
                        
            causes = obj.CustomException.cause;
            obj.CustomException = obj;
            
            if ~isempty(causes)
                for c = causes
                    obj.CustomException = obj.CustomException.addCause(c{1});
                end
            end
            
            newObj = obj;
        end
        
        function rep = getReport(obj, type, key, val)
            %getReport - Overloaded function from MException

            % Handle special case to avoid infinite recursion for
            % CustomException objects that are ROSExceptions themselves
            if exist('type', 'var')
                if strcmp(type, 'custom')
                    rep = getReport@MException(obj);
                    return;
                end
            end
            
            if strcmp(obj.logLevel, 'none')
                % Call the standard getReport function in normal mode

                switch nargin
                    case 1
                        rep = getReport@MException(obj);
                    case 2
                        rep = getReport@MException(obj, type);
                    otherwise
                        rep = getReport@MException(obj, type, key, val);
                end
            else
                % Retrieve the custom report in verbose mode
                rep = obj.getCustomReport;
            end
        end
        
        function rep = getCustomReport(obj)
            %getCustomReport - Print a report of all custom causes
            %   This is simply getting a standard report from the CustomEx
            %   exception.
            %
            %   Example:
            %     ex = MException.last;
            %     if isa(ex, 'ros.internal.ROSException')
            %        ex.getCustomReport;
            %     else
            %        ex.getReport;
            %     end
            
            if isa(obj.CustomException, 'ros.internal.ROSException')
                rep = obj.CustomException.getReport('custom');
            else
                rep = obj.CustomException.getReport;
            end
        end               
    end
    
    methods (Static)
        function rosex = fromException(ex)
            %fromException Construct ROSException from other exception
            %   The input EX can either be a generic MException or a
            %   ROSException already. All other inputs are invalid. The
            %   newly created ROSException is returned in ROSEX.
            
            if isa(ex, 'ros.internal.ROSException')
                rosex = saveCustomStackTrace(ex);
                return;
            elseif isa(ex, 'MException')
                rosex = ros.internal.ROSException(ex.identifier, ex.message);
                rosex.CustomException = ex;
                return;
            else
                error(message('ros:mlros:common:WrongInput'));
            end
        end
    end
    
    methods (Static, Access = private)       
        function newLevel = logLevel(level)            
            %logLevel Set the log level (verbosity) for ROS exceptions
            %   The default log level is 'none' and all errors encountered
            %   through the informal interface have no stack trace or
            %   causes. Setting the log level to 'all' displays all error
            %   information for debugging purposes, e.g. in testing.
            %
            %   NEWLEVEL = ros.internal.ROSException.logLevel('LEVEL') sets
            %   the ROSException log level to LEVEL, where LEVEL can be
            %   either 'none' or 'all'. If successful, the new log level is
            %   returned in NEWLEVEL.
            
            persistent currentLogLevel
            mlock
            
            if isempty(currentLogLevel)
                currentLogLevel = 'none';
            end
                        
            % Return current log level if no input
            if nargin == 0                
                newLevel = currentLogLevel;
                return;
            end
            
            % Valid log level strings
            % 'none' is the default log level that is appropriate for most
            % users; 'all' is the verbose log level for testing
            validLevels = {'all', 'none'};
            
            
            if ischar(level)        
                if any(strcmpi(level,validLevels))
                    currentLogLevel = level;
                    newLevel = currentLogLevel;
                end
            end
            
        end
    end
    
    methods(Static)
        function enableVerboseLogging(enable)
            %enableVerboseLogging Enable verbose output for ROS exceptions
            %   This is the public interface for enabling verbose exception
            %   output for the informal interface.
            %
            %   ros.internal.ROSException.enableVerboseLogging(ENABLE)
            %   enables (ENABLE == true) or disables (ENABLE == false) the
            %   verbose error output.
            
            validateattributes(enable, {'logical'}, {'nonempty'}, ...
                'ROSException', 'enable');
            
            if enable
                ros.internal.ROSException.logLevel('all');
            else
                ros.internal.ROSException.logLevel('none');
            end
        end
    end
end
