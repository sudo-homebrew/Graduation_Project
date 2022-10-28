classdef CallbackHandler < handle
%This class is for internal use only. It may be removed in the future.

%CallbackHandler Helper to execute ROS callbacks
%   The user-specified callback function and argument handling is easier
%   done in MATLAB, so the internal node needs a single method to call.
%   This class is used to avoid a circular reference between the MATLAB
%   ROS primary communication object and the internal node.
%
%   This class is called by the internal node, and will call the provided
%   method in the primary communication object. The primary communication
%   object must then manage the user-supplied callback, optional
%   additional arguments, and converting the ROS message to an object, if
%   applicable.
%
%   This class does very little checking or validation. It passes all
%   arguments it receives from the internal node to the primary object, and
%   ignores any return arguments. There is no error-handling; the primary
%   object must handle all errors appropriately.
%
%   HANDLER = ros.internal.CallbackHandler(PRIMARYOBJECT) Creates a
%   callback handler that does not make any calls when it receives a
%   request to process a message. PRIMARYOBJECT must be a WeakHandle.
%
%   HANDLER = ros.internal.CallbackHandler(PRIMARYOBJECT, CALLBACK) Creates
%   a callback handler that calls the CALLBACK method on the object
%   contained within the PRIMARYOBJECT weak reference when it receives a
%   request to process a message. CALLBACK must be a function handle.

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant, Access = {?ros.internal.mixin.ROSInternalAccess, ?ros.internal.mixin.InternalAccess})
        %CallbackName - Name of the method for the internal node to call
        %   Internal node can only handle string not function handle
        CallbackName = 'processMessage'
    end

    properties (Transient, Access = {?ros.internal.mixin.ROSInternalAccess, ?ros.internal.mixin.InternalAccess})
        %CallbackFcn - Function to call when message received
        %   Function handle to method on primary communication object
        CallbackFcn = function_handle.empty

        %PrimaryObject - Weak reference to primary ROS communication object
        PrimaryObject
    end

    methods
        function obj = CallbackHandler(primaryObject, callback)
            obj.PrimaryObject = primaryObject;
            if nargin > 1
                obj.CallbackFcn = callback;
            end
        end

        function set.PrimaryObject(obj, primaryObject)
            validateattributes(primaryObject, ...
                               {'matlab.internal.WeakHandle'}, ...
                               {'scalar'}, ...
                               'CallbackHandler', ...
                               'PrimaryObject')
            obj.PrimaryObject = primaryObject;
        end

        function processMessage(obj, varargin)
        %processMessage Pass message to primary communication object

        % Call the callback function if assigned
            if ~isempty(obj.CallbackFcn)
                feval(obj.CallbackFcn, get(obj.PrimaryObject), varargin{:});
            end
        end
    end
end
