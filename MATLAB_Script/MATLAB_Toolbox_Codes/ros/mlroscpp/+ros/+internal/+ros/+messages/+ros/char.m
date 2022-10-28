function [data, info] = char(dataType, count, constantValue)
%This function is for internal use only. It may be removed in the future.
%ros/char gives an empty data for char and string data types.

% Copyright 2020 The MathWorks, Inc.
%#codegen
info = struct();
info.MLdataType = dataType;
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;

if isequal(dataType, 'char')
    if nargin <= 2
        if isnan(count)
            data = char.empty(0,1);
            info.MaxLen = NaN;
            info.MinLen = 0;
        elseif count == 1
            data = char(ones(1,1) * '');
        else
            data = char(ones(1,count) * '');
            info.MaxLen = count;
            info.MinLen = count;
        end
    else
        % constant
        data = char(constantValue);
        info.constant = 1;
    end
elseif isequal(dataType, 'string')
    if nargin <= 2
        if isnan(count)
            data = cell.empty(0,1);
            info.MaxLen = NaN;
            info.MinLen = 0;
        elseif count == 0
            data = '';
        else
            data = cell(count,1);
            data = cellfun(@(x)'', data, 'UniformOutput', false);
            info.MaxLen = count;
            info.MinLen = count;
        end
    else
        % constant
        data = char(constantValue);
        info.constant = 1;
    end
end