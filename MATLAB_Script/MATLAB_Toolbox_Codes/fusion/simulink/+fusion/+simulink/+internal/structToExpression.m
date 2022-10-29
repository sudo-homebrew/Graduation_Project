%This function is for internal use only. It may be removed in the future.
%
% Create an expression which results into a structure array when evaluated.
%
% EXP = STRUCTTOEXPRESSION(VAR) Converts structure array VAR into an
% expression that  can be evaluated.

% internal function, no error checking is performed

% Copyright 2021 The MathWorks, Inc.

function stExp  = structToExpression(var)
stExp = '[';
for i = 1:numel(var)
    in = var(i);
    fields = fieldnames(in);
    if isempty(fields)
        stExp = 'struct';
        return
    end
    stExp = append(stExp, '...' , newline,'struct(');
    for j = 1: numel(fields)
        stExp = append(stExp,'...' , newline,'''',fields{j},'''',',');
        if islogical(in.(fields{j}))
            if in.(fields{j})
                s = 'true';
            else
                s = 'false';
            end
        elseif ischar(in.(fields{j})) || isstring(in.(fields{j}))
            s = append('''', char(in.(fields{j})), '''');
        elseif isnumeric(in.(fields{j})) || isenum(in.(fields{j}))
            s = mat2str(in.(fields{j}));
        elseif isstruct(in.(fields{j}))
            s = fusion.simulink.internal.structToExpression(in.(fields{j}));
        elseif isa(in.(fields{j}),'function_handle')
            s = append('''', func2str(in.(fields{j})), '''');
        end
        stExp = append(stExp, s, ',');
    end
    if strcmp(stExp(end),',')
        stExp= stExp(1:end-1);
    end
    stExp =append(stExp,'),');
end
if strcmp(stExp(end),',')
    stExp= stExp(1:end-1);
end
stExp = append(stExp, ']');
end