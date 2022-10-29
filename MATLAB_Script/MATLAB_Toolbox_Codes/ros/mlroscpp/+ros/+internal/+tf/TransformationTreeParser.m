classdef TransformationTreeParser < ...
        ros.internal.DataFormatBase & handle
    %This class is for internal use only. It may be removed in the future.

    %TransformationTreeParser Handles all parsing needs of the TransformationTree
    %   To facilitate testing, the parsing functionality is encapsulated in
    %   this class.

    %   Copyright 2016-2020 The MathWorks, Inc.

    methods
        function obj = TransformationTreeParser(varargin)
        %TransformationTreeParser Construct the parser
        %   Determine if using struct or object messages

            % Parse and validate input arguments
            inParser = inputParser;
            obj.addDataFormatToParser(inParser, 'TransformationTree')
            inParser.parse(varargin{:})
            setDataFormat(obj, inParser.Results.DataFormat)
        end

        function [validTargetFrame, validSourceFrame, sourceTime, timeout] = ...
                parseGetTransformInput(obj, defaults, targetFrame, sourceFrame, varargin)
            %parseGetTransformInput Parse arguments for "getTransform" method
            %   The following syntaxes are valid:
            %   - getTransform('TARGETFRAME', 'SOURCEFRAME')
            %   - getTransform('TARGETFRAME', 'SOURCEFRAME', SOURCETIME)
            %   - getTransform('TARGETFRAME', 'SOURCEFRAME', 'Timeout', TIMEOUT)
            %   - getTransform('TARGETFRAME', 'SOURCEFRAME', SOURCETIME, 'Timeout', TIMEOUT)
            import ros.internal.Parsing.validateROSTime;
            import ros.internal.Parsing.validateTimeout;

            % Convert all strings to character vectors
            if ~isempty(varargin)
                [varargin{:}] = convertStringsToChars(varargin{:});
            end

            if isfield(defaults, 'Timeout')
                % Parse "Timeout" name-value pair
                assert(length(varargin) <= 3);
            else
                % Only parse the (optional) source time input
                assert(length(varargin) <= 1);
                defaults.Timeout = 0;
            end

            % Validate frame names
            [validTargetFrame, validSourceFrame] = obj.validateTargetAndSourceFrame(targetFrame, sourceFrame, 'getTransform');

            switch length(varargin)
              case 0
                % Syntax: getTransform('TARGETFRAME', 'SOURCEFRAME')
                % Return defaults.
                sourceTime = rostime(defaults.SourceTime);
                timeout = defaults.Timeout;

              case 1
                % Syntax: getTransform('TARGETFRAME', 'SOURCEFRAME', SOURCETIME)
                sourceTime = validateROSTime(varargin{1}, 'getTransform', 'sourceTime');
                timeout = defaults.Timeout;

              case 2
                % Syntax: getTransform('TARGETFRAME', 'SOURCEFRAME', 'Timeout', TIMEOUT)
                validatestring(varargin{1}, {'Timeout'}, 'getTransform', 'timeout');
                sourceTime = rostime(defaults.SourceTime);
                timeout = validateTimeout(varargin{2}, 'getTransform', 'timeout');

              case 3
                % Syntax: getTransform('TARGETFRAME', 'SOURCEFRAME', SOURCETIME, 'Timeout', TIMEOUT)
                sourceTime = validateROSTime(varargin{1}, 'getTransform', 'sourceTime');
                validatestring(varargin{2}, {'Timeout'}, 'getTransform', 'timeout');
                timeout = validateTimeout(varargin{3}, 'getTransform', 'timeout');
            end
        end

        function [validTargetFrame, validSourceFrame] = ...
                parseGetTransformBackwardsCompatibleInput(obj, targetFrame, sourceFrame)
            %parseGetTransformBackwardsCompatibleInput Parse arguments for "getTransform" method
            %   This parsing is backwards compatible to the behavior in 15a-16a.
            %   The following syntaxes are valid:
            %   - getTransform('TARGETFRAME', 'SOURCEFRAME')

            sourceFrame = convertStringsToChars(sourceFrame);
            targetFrame = convertStringsToChars(targetFrame);

            % Only do lightweight validation
            validateattributes(sourceFrame, {'char','string'}, {}, 'getTransform', 'sourceFrame');
            validateattributes(targetFrame, {'char','string'}, {}, 'getTransform', 'targetFrame');

            validTargetFrame = obj.stripLeadingSlash(targetFrame);
            validSourceFrame = obj.stripLeadingSlash(sourceFrame);
        end

        function [validTargetFrame, validSourceFrame, sourceTime] = ...
                parseCanTransformInput(obj, defaultSourceTime, targetFrame, sourceFrame, varargin)
            %parseCanTransformInput Parse arguments for "canTransform" method
            %   The following syntaxes are valid:
            %   - canTransform('TARGETFRAME', 'SOURCEFRAME')
            %   - canTransform('TARGETFRAME', 'SOURCEFRAME', SOURCETIME)
            import ros.internal.Parsing.validateROSTime;

            assert(length(varargin) <= 1);

            % Validate frame names
            [validTargetFrame, validSourceFrame] = obj.validateTargetAndSourceFrame(targetFrame, sourceFrame, 'canTransform');

            switch length(varargin)
              case 0
                % Syntax: canTransform('TARGETFRAME', 'SOURCEFRAME')
                % Return default
                sourceTime = rostime(defaultSourceTime);

              case 1
                % Syntax: canTransform('TARGETFRAME', 'SOURCEFRAME', SOURCETIME)
                sourceTime = validateROSTime(varargin{1}, 'canTransform', 'sourceTime');
            end

        end

        function [validTargetFrame, validSourceFrame, msg, sourceTime] = ...
                parseTransformInput(obj, defaultSourceTime, targetFrame, msg, varargin)
            %parseTransformInput Parse arguments for "transform" method
            %   The following syntaxes are valid:
            %   - transform(tftree, targetFrame, msg)
            %   - transform(tftree, targetFrame, msg, 'msgtime')
            %   - transform(tftree, targetFrame, msg, sourceTime)

            import ros.internal.Parsing.validateROSTime;

            assert(length(varargin) <= 1);

            % Validate message type. All message types have a header.
            obj.validateInputMessage(msg, {'geometry_msgs/QuaternionStamped', ...
                                           'geometry_msgs/Vector3Stamped', ...
                                           'geometry_msgs/PointStamped', ...
                                           'geometry_msgs/PoseStamped', ...
                                           'sensor_msgs/PointCloud2'}, ...
                                     'TransformationTree', 'transform')

            % Validate frame names
            sourceFrame = msg.Header.FrameId;
            validTargetFrame = obj.validateFrame(targetFrame, 'transform', 'targetFrame');
            validSourceFrame = obj.validateFrame(sourceFrame, 'transform', 'msg.Header.FrameId');

            % Convert all strings to character vectors
            if ~isempty(varargin)
                [varargin{:}] = convertStringsToChars(varargin{:});
            end

            switch length(varargin)
              case 0
                % Syntax: transform(tftree, targetFrame, msg)
                % Return default
                sourceTime = rostime(defaultSourceTime);

              case 1
                % Syntax: transform(tftree, targetFrame, msg, 'msgtime')
                %         transform(tftree, targetFrame, msg, sourceTime)

                sourceTimeSpec = varargin{1};
                validateattributes(sourceTimeSpec, ...
                                   {'char', 'string', 'numeric', 'ros.msg.Time', 'struct'}, ...
                                   {}, 'transform', 'sourceTime');

                if ischar(sourceTimeSpec)
                    % Syntax: transform(tftree, targetFrame, msg, 'msgtime')
                    validatestring(sourceTimeSpec, {'msgtime'}, 'transform', 'sourceTime');
                    sourceTime = msg.Header.Stamp;
                else
                    % Syntax: transform(tftree, targetFrame, msg, sourceTime)
                    sourceTime = varargin{1};
                end
                sourceTime = validateROSTime(sourceTime, 'transform', 'sourceTime');
            end
        end

        function validBufferTime = parseBufferTime(~, bufferTime)
        %parseBufferTime Parse the input to the BufferTime setter

            validateattributes(bufferTime, {'numeric'}, {'nonempty', 'scalar', 'real', 'positive', ...
                                'nonnan', 'finite', '<', ros.internal.Parsing.MaxTimeNumeric}, ...
                               'TransformationTree', 'BufferTime');

            validBufferTime = double(bufferTime);
        end

        function [validTargetFrame, validSourceFrame, timeout] = ...
                parseWaitArguments(obj, funcName, defaults, targetFrame, sourceFrame, varargin)
            %parseWaitArguments Parse the arguments for waitForTransform

            % Validate frame names
            [validTargetFrame, validSourceFrame] = obj.validateTargetAndSourceFrame(targetFrame, sourceFrame, funcName);

            % Return defaults if there is nothing else to parse
            if isempty(varargin)
                timeout = defaults.timeout;
                return;
            end

            if numel(varargin) > 1
                error(message('ros:validation:TooManyInputs', funcName));
            end

            % Parse timeout property
            timeout = varargin{1};
            validateattributes(timeout, {'numeric'}, ...
                               {'scalar','nonempty','positive','nonnan'}, funcName, 'timeout');
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function [validTargetFrame, validSourceFrame] = ...
                validateTargetAndSourceFrame(obj, targetFrame, sourceFrame, funcName)
            validTargetFrame = obj.validateFrame(targetFrame, funcName, 'targetFrame');
            validSourceFrame = obj.validateFrame(sourceFrame, funcName, 'sourceFrame');
        end

        function validFrameName = validateFrame(obj, frameName, funcName, varName)
        %validateFrame Validate a frame name and return a name that is always valid
        %   This function will remove a leading slash (/) if it exists
        %   (consistent with ROS C++ and Python). If the frame name
        %   starts with two slashes, an error is displayed.

            frameNameChar = convertStringsToChars(frameName);
            validateattributes(frameNameChar, {'char','string'}, {'nonempty','scalartext'}, funcName, varName)

            % Remove the first slash, if it exists
            [validFrameName, isStripped] = obj.stripLeadingSlash(frameNameChar);
            if isStripped
                % Make sure that the frame is still non-empty after stripping
                % the slash.
                validateattributes(validFrameName, {'char','string'}, {'nonempty'}, funcName, varName);
            end

            % Error out if there is another slash
            if strncmp(validFrameName, '/', 1)
                error(message('ros:mlros:tf:FrameNameLeadingSlashes', frameNameChar));
            end

        end

        function [frameNameNoSlash, isStripped] = stripLeadingSlash(~, frameName)
        %stripLeadingSlash Remove the first slash, if it exists
        %   strncmp will return "false" if frameName is not a character
        %   vector.
            if strncmp(frameName, '/', 1)
                frameNameNoSlash = frameName(2:end);
                isStripped = true;
            else
                frameNameNoSlash = frameName;
                isStripped = false;
            end
        end
    end

end
