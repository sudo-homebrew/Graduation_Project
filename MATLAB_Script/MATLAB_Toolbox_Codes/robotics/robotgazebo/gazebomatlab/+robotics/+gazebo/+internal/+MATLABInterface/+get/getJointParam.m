function [param, paramString] = getJointParam(modelName, jointName, varargin)
%This function is for internal use only. It may be removed in the future.
%
% This function connects with Gazebo and request model parameter value
% based on modelname and jointname. Further, creates MATLAB struct and
% string of output as per received model-joint parameters.

%   Copyright 2020 The MathWorks, Inc.

% output parameter struct
    param.modelname = modelName;
    param.jointname = jointName;
    % output parameter string map. Contains parameter name and value in string form
    paramString = containers.Map('KeyType','char','ValueType','char');

    gazeboClient = robotics.internal.GazeboClient;

    % connect to Gazebo
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient);
    modelParam = gazeboClient.getGazeboModelParam(modelName,false,jointName);
    gazeboClient.shutdown();

    % error-out if requested model does not exist in Gazebo
    if(strcmp(modelParam.message.name,"") ||...
       ~strcmp(modelParam.message.name,modelName))
        error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                      modelName,'gzmodel("list")'));
    end

    % search input link struct
    modelJointNames = [modelParam.message.joints.name];
    if(isempty(modelJointNames))
        instMsg = [ 'gzjoint("list","' modelName,'")'];
        error(message('robotics:robotgazebo:gzsupport:InvalidJointName',...
                      jointName, modelName, instMsg));
    end
    jointParamStruct = modelParam.message.joints(1);

    %inputs = varargin{1};

    % error-out if first parameter 'Axis0 or Axis1' is  missing for
    if( any(contains({'Damping','Angle','Friction','XYZ'},varargin{:})) &&...
        ~any(contains({'Axis0','Axis1'},varargin{:})))
        error(message('robotics:robotgazebo:gzsupport:FirstParameterMissing',...
                      'Damping, Angle, Friction, XYZ', 'Axis0 or Axis1')); % UPDATE for exact parameter name
    end
    % create output struct and printable string for 'Position' values
    if contains("Position",varargin{:})
        param.Position = [ jointParamStruct.pose.position.x,...
                           jointParamStruct.pose.position.y,...
                           jointParamStruct.pose.position.z];
        paramString('Position') = [...
            'x : ', num2str(jointParamStruct.pose.position.x),' , ',...
            'y : ', num2str(jointParamStruct.pose.position.y),' , ',...
            'z : ', num2str(jointParamStruct.pose.position.z)];
    end
    % create output struct and printable string for 'Orientation' values
    if contains("Orientation",varargin{:})
        param.Orientation = [ jointParamStruct.pose.orientation.w,...
                            jointParamStruct.pose.orientation.x,...
                            jointParamStruct.pose.orientation.y,...
                            jointParamStruct.pose.orientation.z];
        paramString('Orientation') = [...
            'w : ', num2str(jointParamStruct.pose.orientation.w),' , ',...
            'x : ', num2str(jointParamStruct.pose.orientation.x),' , ',...
            'y : ', num2str(jointParamStruct.pose.orientation.y),' , ',...
            'z : ', num2str(jointParamStruct.pose.orientation.z)];
    end
    % error-out if requested joint does not have any axis and
    % axis-dependent parameters are requested. Other than, 'Position' and
    % 'Orientation'
    if(jointParamStruct.dof == 0 && ...
       ~all(contains(varargin{:},{'Position','Orientation'})))
        error(message('robotics:robotgazebo:gzsupport:JointAxisNone',...
                      jointName, modelName));
    end
    % create output struct and printable string for 'FudgeFactor' value
    if contains("FudgeFactor",varargin{:})
        param.FudgeFactor = jointParamStruct.fudge_factor;
        paramString('FudgeFactor') = num2str(jointParamStruct.fudge_factor);
    end
    % create output struct and printable string for 'CFM' values
    if contains("CFM",varargin{:})
        param.CFM = jointParamStruct.cfm;
        paramString('CFM') = num2str(jointParamStruct.cfm);
    end
    % create output struct and printable string for 'SuspensionCFM' value
    if contains("SuspensionCFM",varargin{:})
        param.SuspensionCFM = jointParamStruct.suspension_cfm;
        paramString('SuspensionCFM') = num2str(jointParamStruct.suspension_cfm);
    end
    % create output struct and printable string for 'SuspensionERP' value
    if contains("SuspensionERP",varargin{:})
        param.SuspensionERP = jointParamStruct.suspension_erp;
        paramString('SuspensionERP') = num2str(jointParamStruct.suspension_erp);
    end
    % for two input arguments
    if( any(contains({'Axis0','Axis1'},varargin{:})) && ...
        any(contains({'Damping','Angle','Friction','XYZ'},varargin{:})))
        % For axis index '0'
        if contains('Axis0',varargin{:})
            % create output struct and printable string for 'XYZ' values
            if contains("XYZ",varargin{:})
                param.Axis0.XYZ = [jointParamStruct.axis1.xyz.x, ...
                                   jointParamStruct.axis1.xyz.y, ....
                                   jointParamStruct.axis1.xyz.z];
                paramString('Axis0 XYZ') = [...
                    'X : ', num2str(jointParamStruct.axis1.xyz.x),' , ',...
                    'Y : ', num2str(jointParamStruct.axis1.xyz.y),' , ',...
                    'Z : ', num2str(jointParamStruct.axis1.xyz.z)];
            end
            % create output struct and printable string for 'Damping' value
            if contains("Damping",varargin{:})
                param.Axis0.Damping = jointParamStruct.axis1.damping;
                paramString('Axis0 Damping') = num2str(jointParamStruct.axis1.damping);
            end
            % create output struct and printable string for 'Friction' value
            if contains("Friction",varargin{:})
                param.Axis0.Friction = jointParamStruct.axis1.friction;
                paramString('Axis0 Friction') = num2str(jointParamStruct.axis1.friction);
            end
            % create output struct and printable string for 'Angle' value
            if contains("Angle",varargin{:})
                param.Axis0.Angle = jointParamStruct.axis1.angle;
                paramString('Axis0 Angle') = num2str(jointParamStruct.axis1.angle);
            end
        end
        % For axis index '1'
        if contains('Axis1',varargin{:})
            % error-out if requested axis index is not associated with joint
            if(jointParamStruct.dof == 1)
                error(message('robotics:robotgazebo:gzsupport:InvalidAxisIndex',...
                              1, jointName, modelName));
            end
            % create output struct and printable string for 'XYZ' values
            if contains("XYZ",varargin{:})
                param.Axis1.XYZ = [jointParamStruct.axis2.xyz.x,...
                                   jointParamStruct.axis2.xyz.y, ...
                                   jointParamStruct.axis2.xyz.z];
                paramString('Axis1 XYZ') = [...
                    'X : ', num2str(jointParamStruct.axis2.xyz.x),' , ',...
                    'Y : ', num2str(jointParamStruct.axis2.xyz.y),' , ',...
                    'Z : ', num2str(jointParamStruct.axis2.xyz.z)];
            end
            % create output struct and printable string for 'Damping' value
            if contains("Damping",varargin{:})
                param.Axis1.Damping = jointParamStruct.axis2.damping;
                paramString('Axis1 Damping') = num2str(jointParamStruct.axis2.damping);
            end
            % create output struct and printable string for 'Friction' value
            if contains("Friction",varargin{:})
                param.Axis1.Friction = jointParamStruct.axis2.friction;
                paramString('Axis1 Friction') = num2str(jointParamStruct.axis2.friction);
            end
            % create output struct and printable string for 'Angle' value
            if contains("Angle",varargin{:})
                param.Axis1.Angle = jointParamStruct.axis2.angle;
                paramString('Axis1 Angle') =  num2str(jointParamStruct.axis2.angle);
            end
        end
    else
        % error-out if input argument contains first parameter as 'Axis0'
        % or 'Axis1' and second parameter is missing in input argument
        if( any(contains({'Axis0','Axis1'},varargin{:})) )
            error(message('robotics:robotgazebo:gzsupport:SecondParameterMissing'));
        end
    end

end
