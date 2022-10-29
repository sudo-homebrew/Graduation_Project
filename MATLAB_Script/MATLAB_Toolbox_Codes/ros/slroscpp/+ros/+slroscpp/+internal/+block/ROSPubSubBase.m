classdef (Abstract) ROSPubSubBase < ros.slros.internal.block.ROSPubSubBase
%#codegen

%   Copyright 2019-2020 The MathWorks, Inc.
    properties (Constant, Hidden)
        ROSVersion = 'ROS';
    end

    % The following should ideally not show up in the MATLAB System block
    % dialog. However, setting them as 'Hidden' will prevent them from
    % being accessible via set_param & get_param.
    %
    %   ModelName is needed for managing the node instance
    %   BlockId is needed to generate a unique identifier in codegen
    properties(Nontunable)        
        %MessageQueueLen Length of message queue
        %   Only used in generated code
        MessageQueueLen = 1;
    end

    properties(Constant,Access=protected)
        % Name of header file with declarations for variables and types
        % referred to in code emitted by setupImpl and stepImpl.
        HeaderFile = ros.slros.internal.cgen.Constants.InitCode.HeaderFile;
    end

    % public setter/getter methods
    methods
        function obj = ROSPubSubBase (varargin)
            coder.allowpcode('plain');
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end
end
