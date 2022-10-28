function [data, info] = default_type(dataType, count, varsize, constantValue, defaultValue)

%usage - Used to generate variables and arrays of default data types
% 1. mw_msg_data_type(dataType, 1) - creates a variable of type 'dataType'
% 2. mw_msg_data_type(dataType, 1, 5) - creates a constant of type
%                                       'dataType' with 'value' as data
% 3. mw_msg_data_type(dataType, arraySize) - creates an array of type
%                        'dataType' and size 'arraySize'
% 4. mw_msg_data_type(dataType, NaN) - creates a variable sized
%                        array of type 'dataType'

%   Copyright 2020 The MathWorks, Inc.
%#codegen

    info = struct();
    info.MLdataType = dataType;
    info.constant = 0;
    info.default = 0;

    if isequal(nargin,3)
        if isnan(count)
            data = zeros(1,1,dataType);
        else
            data = zeros(count,1,dataType);
        end
    else
        if isnan(constantValue)
            % constant
            data = cast(defaultValue,dataType).';
            info.default = 1;
        else
            % default
            data = cast(constantValue,dataType).';
            info.constant = 1;
        end
    end

    info.maxstrlen = NaN;
    if isnan(count)
        info.MaxLen = NaN;
        info.MinLen = 0;
    else
        info.MaxLen = count;
        if varsize
            info.MinLen = 0;
        else
            info.MinLen = count;
        end
    end
