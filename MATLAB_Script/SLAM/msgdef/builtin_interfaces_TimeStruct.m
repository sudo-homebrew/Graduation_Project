function msg = builtin_interfaces_TimeStruct
% Message struct definition for builtin_interfaces/Time
coder.inline("never")
msg = struct(...
    'MessageType','builtin_interfaces/Time',...
    'sec',ros.internal.ros2.messages.ros2.default_type('int32',1,0),...
    'nanosec',ros.internal.ros2.messages.ros2.default_type('uint32',1,0));
coder.cstructname(msg,'builtin_interfaces_TimeStruct_T');
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
