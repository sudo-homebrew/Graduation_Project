function [data, info] = char(dataType, count, strlen, varsize, constantValue, defaultValue)
%This function is for internal use only. It may be removed in the future.
%ros2/char gives an empty data for char and string data types.

%   Copyright 2020-2021 The MathWorks, Inc.
%#codegen


    info = struct();
    info.MLdataType = dataType;
    info.constant = 0;
    info.default = 0;

    if isequal(dataType, 'char')
        if isequal(nargin,4)
            if isnan(count)
                data = ' ';
            else
                data = char(ones(1,count) * ' ');
            end
        else
            if any(isnan(defaultValue))
                % constant
                data = char(constantValue);
                info.constant = 1;
            else
                % default
                data = char(defaultValue);
                info.default = 1;
            end
        end
    elseif isequal(dataType, 'string')
        if isequal(nargin,4)
            if isnan(count)
                data = {''};
            elseif isequal(count,1)
                data = char(ones(1,1) * '');
            else
                data = cell(count,1);
                data = cellfun(@(x)'', data, 'UniformOutput', false);
            end
        else
            if any(isnan(defaultValue))
                % constant
                data = char(constantValue);
                info.constant = 1;
            else
                % default
                data = char(defaultValue);
                info.default = 1;
            end
        end
    end

    if isnan(strlen)
        info.maxstrlen = NaN;
    else
        info.maxstrlen = strlen;
    end

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
