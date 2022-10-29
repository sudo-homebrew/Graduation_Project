function Joint = getJointParamStruct(jointName,p)
%This function is for internal use only. It may be removed in the future.
%
% This function creates MATLAB struct based on jointname and parameter name
% and value.

%   Copyright 2020 The MathWorks, Inc.

% get second parameter list
    secondParameterNames = {'Angle','Damping','Friction','XYZ'};
    % get second parameter values
    secondParameterIn = {p.Results.Angle, p.Results.Damping, p.Results.Friction, p.Results.XYZ};
    % find non-empty parameter index
    nonEmptyIndex = find(~cellfun(@isempty,secondParameterIn), 1);
    % error-out if first parameter 'Axis' is missing for second parameters
    if( ~isempty(nonEmptyIndex) && isempty(p.Results.Axis) )
        error(message('robotics:robotgazebo:gzsupport:FirstParameterMissing',...
                      secondParameterNames{nonEmptyIndex}, 'Axis'));
    end

    Joint.name = jointName;
    % check input 'Position' value and add into struct
    if(~isempty(p.Results.Position))
        Joint.pose.position.x = double(p.Results.Position(1));
        Joint.pose.position.y = double(p.Results.Position(2));
        Joint.pose.position.z = double(p.Results.Position(3));
    end
    % check input 'Orientation' value and add into struct
    if(~isempty(p.Results.Orientation))
        Joint.pose.orientation.w = double(p.Results.Orientation(1));
        Joint.pose.orientation.x = double(p.Results.Orientation(2));
        Joint.pose.orientation.y = double(p.Results.Orientation(3));
        Joint.pose.orientation.z = double(p.Results.Orientation(4));
    end
    % check input 'FudgeFactor' value and add into struct
    if(~isempty(p.Results.FudgeFactor))
        Joint.fudge_factor = double(p.Results.FudgeFactor);
    end
    % check input 'CFM' value and add into struct
    if(~isempty(p.Results.CFM))
        Joint.cfm = double(p.Results.CFM);
    end
    % check input 'SuspensionCFM' value and add into struct
    if(~isempty(p.Results.SuspensionCFM))
        Joint.suspension_cfm = double(p.Results.SuspensionCFM);
    end
    % check input 'SuspensionCFM' value and add into struct
    if(~isempty(p.Results.SuspensionCFM))
        Joint.suspension_cfm = double(p.Results.SuspensionCFM);
    end
    % check input 'SuspensionERP' value and add into struct
    if(~isempty(p.Results.SuspensionERP))
        Joint.suspension_erp = double(p.Results.SuspensionERP);
    end

    % check input 'Axis' input if exist
    if(~isempty(p.Results.Axis))

        if( isempty(nonEmptyIndex) )
            error(message('robotics:robotgazebo:gzsupport:SecondParameterMissing'));
        end
        % check input 'Axis0' value and add into struct
        if(strcmp(p.Results.Axis,'0'))
            % check input 'XYZ' value and add into struct
            if(~isempty(p.Results.XYZ))
                Joint.axis1.xyz.x = double(p.Results.XYZ(1));
                Joint.axis1.xyz.y = double(p.Results.XYZ(2));
                Joint.axis1.xyz.z = double(p.Results.XYZ(3));
            end
            % check input 'Damping' value and add into struct
            if(~isempty(p.Results.Damping))
                Joint.axis1.damping = double(p.Results.Damping);
            end
            % check input 'Friction' value and add into struct
            if(~isempty(p.Results.Friction))
                Joint.axis1.friction = double(p.Results.Friction);
            end
            % check input 'Angle' value and add into struct
            if(~isempty(p.Results.Angle))
                Joint.axis1.angle = double(p.Results.Angle);
            end
        end
        % check input 'Axis1' value and add into struct
        if(strcmp(p.Results.Axis,'1'))
            % check input 'XYZ' value and add into struct
            if(~isempty(p.Results.XYZ))
                Joint.axis2.xyz.x = double(p.Results.XYZ(1));
                Joint.axis2.xyz.y = double(p.Results.XYZ(2));
                Joint.axis2.xyz.z = double(p.Results.XYZ(3));
            end
            % check input 'Damping' value and add into struct
            if(~isempty(p.Results.Damping))
                Joint.axis2.damping = double(p.Results.Damping);
            end
            % check input 'Friction' value and add into struct
            if(~isempty(p.Results.Friction))
                Joint.axis2.friction = double(p.Results.Friction);
            end
            % check input 'Angle' value and add into struct
            if(~isempty(p.Results.Angle))
                Joint.axis2.angle = double(p.Results.Angle);
            end
        end

    end

end
