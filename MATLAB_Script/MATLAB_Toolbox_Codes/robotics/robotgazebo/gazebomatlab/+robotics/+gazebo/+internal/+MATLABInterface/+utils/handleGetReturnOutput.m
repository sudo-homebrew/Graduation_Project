function output = handleGetReturnOutput( result, validParamName, getType)
%This function is for internal use only. It may be removed in the future.
%
% This function returns cell with multiple output values based on result
% and parameter names.

%   Copyright 2020 The MathWorks, Inc.

    if(strcmp(getType,'getJoint'))
        % for get joint operation
        axisOptions = {'Axis0','Axis1'};
        count = 1;
        for outIdx = 1:numel(validParamName)
            % combine output for single parameter with multiple axis request
            % e.g. 'Axis0' 'Axis1' and 'Damping' input will return two values
            % of damping
            if(contains(validParamName{outIdx},{'Damping','Angle','Friction','XYZ'}) )
                axisIndex = contains(axisOptions,validParamName);
                reqAxisNames = axisOptions(axisIndex);
                for axIdx = 1:numel(reqAxisNames)
                    axisData = result.(reqAxisNames{axIdx});
                    output{count} = axisData.(validParamName{outIdx});
                    count = count + 1;
                end
            else
                % parameter other than 'Damping','Angle','Friction','XYZ'
                % also exclude 'Axis0','Axis1' in output
                if ( ~contains(validParamName{outIdx},{'Axis0','Axis1'}) )
                    output{count} = result.(validParamName{outIdx});
                    count = count + 1;
                end
            end
        end

    else
        % for getmodel and get link operation
        output = cell(1,numel(validParamName));
        for outIdx = 1:numel(validParamName)
            output{outIdx} = result.(validParamName{outIdx});
        end
    end

end
