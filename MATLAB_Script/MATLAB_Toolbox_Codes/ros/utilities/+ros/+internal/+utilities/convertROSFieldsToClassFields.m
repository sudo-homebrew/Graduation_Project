function classField = convertROSFieldsToClassFields(rosField, fullMsgName)
%This function is for internal use only. It may be removed in the future.

%convertROSFieldsToClassFields converts field names used by ROS message
%structs to field names used by ROS 1 message classes
%
%   CLASSFIELD = convertROSFieldsToClassFields(ROSFIELD, MSGNAME)
%   provides the appropriate field name for a ROS 1 message class based on
%   the field name used in ROS 1 or ROS 2 message structs, ROSFIELD.
%
%   MSGNAME is the name of the message that contains the field, and can
%   either include the package name (e.g. "geometry_msgs/Pose2D") or not
%   (e.g. "Pose2D"). The message name is required because the field may be
%   modified if it matches the message name.
%
%   This also will automatically convert some of the field names used by
%   ROS 2 into field names used by ROS 1 (e.g. "nanosec" -> "Nsec").

%   Copyright 2020 The MathWorks, Inc.

% Get only non-packaged message name

    [~,msgName,~] = fileparts(fullMsgName);
    % Convert field name, handling common ROS differences
    if strcmp(rosField, 'nanosec')
        classField = 'Nsec';
    else
        classField = ros.internal.utilities.convertLowercaseUnderscoreToMixedCase(rosField);
    end

    % Handle field matching message class name or reserved properties
    if ismember(classField, ...
                {msgName, 'MD5Checksum', 'MessageType', 'PropertyList', 'ROSPropertyList', 'PropertyMessageTypes'})
        classField = strcat(classField, '_');
    end

end

