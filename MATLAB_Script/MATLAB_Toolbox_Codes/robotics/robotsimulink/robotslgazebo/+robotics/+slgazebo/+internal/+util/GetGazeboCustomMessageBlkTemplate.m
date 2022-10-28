function [template] = GetGazeboCustomMessageBlkTemplate(MessageType)
%This function is for internal use only. It may be removed in the future.

%  GetGazeboCustomMessageBlkTemplate calls importGazeboCustomMessageBlkTemplate function
%  and reads simulink message template for custom message

%   Copyright 2020 The MathWorks, Inc.

    template = [];

    % retrieve simulink message template if importGazeboCustomMessageBlkTemplate is available
    if(robotics.slgazebo.internal.util.checkCustomMessageOnPath...
       ('importGazeboCustomMessageBlkTemplate.m') && ~isempty(MessageType))
        template = importGazeboCustomMessageBlkTemplate(MessageType);
    end

end
