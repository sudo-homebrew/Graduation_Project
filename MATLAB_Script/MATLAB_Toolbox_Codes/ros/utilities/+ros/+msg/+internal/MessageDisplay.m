classdef MessageDisplay
%This class is for internal use only. It may be removed in the future.

%MessageDisplay Utility functions for displaying message data
%   The functions are also used to display data stored in ROS
%   parameters.

%   Copyright 2020 The MathWorks, Inc.

%#ok<*AGROW>
    properties (Constant)
        %MaxVectorElementsToPrint - Maximum number of numeric vector elements that should be printed
        MaxVectorElementsToPrint = 1000

        %LevelGap - Number of spaces between structure elements
        LevelGap = '  '

        %SingleDoubleFormatting - sprintf formatting used for single and double values
        SingleDoubleFormatting = '%.16g'
    end

    methods (Static)
        function dispString = printData(msg)
        %printData Print the data of a ROS message recursively
        %   DISPSTRING = printData(MSG) converts the ROS message object
        %   MSG into a structure and then recursively generates a
        %   display string for its data contents. The complete string
        %   is returned in DISPSTRING.

            validateattributes(msg, {'ros.Message'}, {'nonempty', 'scalar'}, ...
                               'printData', 'msg');

            % Convert message into structure
            msgStruct = msg.toStruct;

            % Print structure
            structLevel = 1;
            dispString = ros.msg.internal.MessageDisplay.printStruct(msgStruct, structLevel);
        end

        function dispString = printStruct(data, structLevel, isParam)
        %printStruct Print the contents of a structure
        %   DISPSTRING = printStruct(DATA, STRUCTLEVEL, ISPARAM)
        %   recursively evaluates the structure DATA and generates a
        %   display string, DISPSTRING, for its data contents.
        %   STRUCTLEVEL indicates the current level of nesting in the
        %   structure. The top-most level is 1.
        %   ISPARAM is true if the structure is a parameter dictionary.

            if nargin < 3
                isParam = false;
            end

            % dispString is the string that will be returned
            dispString = '';

            % levelGap is the number of spaces between structure elements
            levelGap = ros.msg.internal.MessageDisplay.LevelGap;

            % Read field names of structure
            fields = fieldnames(data);
            fieldsJustified = char(fields);

            % Handle structure arrays correctly
            numStructs = numel(data);
            for n = 1:numStructs
                space = repmat(levelGap, [1, structLevel]);
                % Print out array element number
                if numStructs > 1
                    arrayNum = sprintf('- (%d)', n);
                    dispString = [dispString, sprintf('\n%s%s', space, arrayNum)];
                end
                % Loop over all fields in structure
                for i = 1:numel(fields)
                    fieldVar = data(n).(fields{i});
                    if isstruct(fieldVar)
                        % Call function recursively for other structure elements
                        dispStringAdd = ros.msg.internal.MessageDisplay.printStruct( ...
                            fieldVar, structLevel + 1);
                        if isParam
                            postFix = ':';
                        else
                            postFix = ' ';
                        end
                    else
                        % Call standard print function for other data types
                        dispStringAdd = ros.msg.internal.MessageDisplay.printVariable(fieldVar, isParam);
                        postFix = ':';
                    end
                    dispString = sprintf('%s\n%s%s %s  %s', dispString, space, ...
                                         fieldsJustified(i,:), postFix, dispStringAdd);
                end
            end
        end

        function dispString = printVariable(data, isParam)
        %printVariable Print the contents of a variable
        %   The type of the variable determines what the printout looks
        %   like.

            if nargin < 2
                isParam = false;
            end

            switch class(data)
              case 'char'
                dispString = ros.msg.internal.MessageDisplay.printChar(data);
              case {'uint8', 'int8', 'uint16', 'int16', 'uint32', 'int32', ...
                    'int64', 'logical'}
                dispString = ros.msg.internal.MessageDisplay.printNumeric(data, '%ld');
              case 'uint64'
                % Use special "lu" formatting to make sure that
                % all digits of the integer are printed out
                % This is consistent with int2str behavior.
                dispString = ros.msg.internal.MessageDisplay.printNumeric(data, '%lu');
              case {'single', 'double'}
                % A maximum limit of 16 floating-point digits is in
                % line with the behavior of num2str.
                dispString = ros.msg.internal.MessageDisplay.printNumeric(data, ...
                                                                  ros.msg.internal.MessageDisplay.SingleDoubleFormatting);
              case 'struct'
                dispString = ros.msg.internal.MessageDisplay.printStruct(data, 1, isParam);
              case 'cell'
                if isParam
                    dispString = ros.msg.internal.MessageDisplay.printParamCell(data, 0);
                else
                    dispString = ros.msg.internal.MessageDisplay.printCell(data);
                end
              otherwise
                error(message('ros:mlroscpp:message:DisplayDataTypeNotValid', ...
                              class(data)));
            end
        end

        function dispString = printChar(data)
        %printChar Prints a character string

            if isempty(data)
                dispString = '';
                return;
            end

            dispString = sprintf('%s', strtrim(data(1, :)));
            for i = 2:size(data,1)
                dispString = [dispString sprintf(', %s', strtrim(data(i, :)))];
            end
        end

        function dispString = printNumeric(data, format)
        %printNumeric Prints a numeric value

            dispString = '';

            if isempty(data)
                dispString = '[]';
                return;
            end

            % Print a scalar value
            if isscalar(data)
                dispString = [dispString sprintf(format, data)];
                return;
            end

            % Data is a vector or a matrix
            dispString = '[';

            if isvector(data)
                % Print a 1-D vector. It has at least 2 elements, since we
                % checked for a scalar above.
                % Since the command window text output is limited to 25,000
                % characters and I place a comma and space after each
                % number, I limit the printout to 1,000 data elements.
                maxPrintLength = ros.msg.internal.MessageDisplay.MaxVectorElementsToPrint;
                printLength = min(length(data), maxPrintLength);
                dispString = [dispString sprintf(format, data(1))];
                dispString = [dispString sprintf([', ' format], data(2:printLength))];

                if length(data) > maxPrintLength
                    % Add a "..." string to indicate that there are more data
                    % elements that are not printed
                    truncationMessage = message('ros:mlroscpp:message:ArrayOutputTruncated', num2str(maxPrintLength)).getString;
                    dispString = [dispString '... ' truncationMessage];
                end
            else
                % Print a 2-D matrix
                [rows, cols] = size(data);
                for i = 1:rows
                    if i > 1
                        dispString = [dispString ', '];
                    end
                    dispString = [dispString sprintf(format, data(i, 1))];
                    if cols > 1
                        dispString = [dispString sprintf([', ' format], data(i, 2:cols))];
                    end
                end
            end

            dispString = [dispString ']'];
        end

        function dispString = printCell(data)
        %printCell Prints a cell array

            if isempty(data)
                dispString = '{}';
                return;
            end

            dispString = '';
            for i = 1:numel(data)
                dispStringAdd = ros.msg.internal.MessageDisplay.printVariable(data{i});
                dispString = [dispString sprintf('{%d}  %s   ', i, dispStringAdd)];
            end
        end

        function dispString = printParamCell(value, numDash)
        %printParamCell Print a cell array (list) received as a parameter value
        %   This is used primarily for the printouts of the "rosparam"
        %   command-line utility.

            dispString = '';
            if isempty(value)
                return;
            end

            % Determine if cell array is nested
            isNested = any(cellfun(@(x) iscell(x), value));

            % Handle easy case first: cell array of atomic data types
            % Printout looks like a cell array, e.g. {0.5,-10,'string',...}
            if ~isNested
                dash = getDash(numDash - 1);
                dispString = [dash, '{'];
                for i = 1:length(value)
                    val = value{i};
                    if isnumeric(val) || islogical(val)
                        % Handle numeric and logical values
                        dispString = [dispString, num2str(value{i}, ...
                                                          ros.msg.internal.MessageDisplay.SingleDoubleFormatting), ', '];
                    elseif isstruct(val)
                        dispString = [dispString, ros.msg.internal.MessageDisplay.printStruct(val, 1), ', '];
                    else
                        % Handle string input
                        dispString = [dispString, value{i}, ', '];
                    end

                end
                dispString(end-1) = '}';
                dispString(end) = [];
                return;
            end

            % This is a cell array that contains nested cell arrays.
            % Printout shows hierarchy, e.g. the following for {0.5, -10,
            % 'string', {1, 2}}
            % - 0.5
            % - -10
            % - string
            % - - 1
            % - - 2
            for i = 1:length(value)
                if iscell(value{i})
                    % Recursively print all nested elements
                    dispString = [dispString ros.msg.internal.MessageDisplay.printParamCell(value{i}, numDash+1)];
                elseif isnumeric(value{i}) || islogical(value{i})
                    % Handle numeric and logical values
                    dash = getDash(numDash);
                    dispString = [dispString, dash, num2str(value{i}, ros.msg.internal.MessageDisplay.SingleDoubleFormatting)];
                else
                    % Handle string input
                    dash = getDash(numDash);
                    dispString = [dispString, dash, value{i}];
                end
                dispString = [dispString newline];
            end

            dispString = [dispString sprintf('\b')];

            function dash = getDash(numDash)
                dash = '';
                for k = 0:numDash
                    dash = [dash '- '];
                end
            end

        end
    end
end
