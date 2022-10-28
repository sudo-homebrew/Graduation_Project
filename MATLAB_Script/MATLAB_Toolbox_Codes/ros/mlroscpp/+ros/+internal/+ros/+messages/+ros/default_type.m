function [data, info] = default_type(dataType, count, constantValue)
%This function is for internal use only. It may be removed in the future.
%ros/default_type gives an empty data for default ros data types.

% Copyright 2020 The MathWorks, Inc.

%usage - Used to generate variables and arrays of default data types
% 1. mw_msg_data_type(dataType, 1) - creates a variable of type 'dataType'
% 2. mw_msg_data_type(dataType, 1, 5) - creates a constant of type
%                                       'dataType' with 'value' as data
% 3. mw_msg_data_type(dataType, arraySize) - creates an array of type 
%                        'dataType' and size 'arraySize'
% 4. mw_msg_data_type(dataType, NaN) - creates a variable sized 
%                        array of type 'dataType'
%#codegen

info = struct();
info.MLdataType = dataType;
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;

if nargin <= 2
    if isnan(count)
        data = zeros(0,1,dataType);
        info.MaxLen = NaN;
        info.MinLen = 0;
    elseif count == 1
        data = zeros(1,1,dataType);
    else
        data = zeros(count,1,dataType);
        info.MaxLen = count;
        info.MinLen = count;
    end
else
    % constant
    data = cast(constantValue,dataType).';
    info.constant = 1;
end