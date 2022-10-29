function value = convertROS2msgToROS1msg(msg)
%This function is for internal use only. It may be removed in the future.

%CONVERTROS2MSGTOROS1MSG converts ROS2 msg to its equivalent MATLAB ROS1 msg.

%   Copyright 2019-2020 The MathWorks, Inc.

msg = strsplit(msg,'.');
value = cell(1,numel(msg));
for i = 1:numel(msg)
    if isequal(msg{i},'nanosec')
        value{i} = 'Nsec';
    else
        value{i} = ros.internal.utilities.convertLowercaseUnderscoreToMixedCase(msg{i});
    end
end

value = strjoin(value,'.');

end
