function msg = std_msgs_HeaderStruct
% Message struct definition for std_msgs/Header
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/Header',...
    'stamp',builtin_interfaces_TimeStruct,...
    'frame_id',ros.internal.ros2.messages.ros2.char('string',1,NaN,1));
coder.cstructname(msg,'std_msgs_HeaderStruct_T');
coder.varsize('msg.frame_id',[1 1000000000],[0 1]);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
