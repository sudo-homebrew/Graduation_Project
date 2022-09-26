function msg = std_msgs_MultiArrayDimensionStruct
% Message struct definition for std_msgs/MultiArrayDimension
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/MultiArrayDimension',...
    'label',ros.internal.ros2.messages.ros2.char('string',1,NaN,1),...
    'size',ros.internal.ros2.messages.ros2.default_type('uint32',1,0),...
    'stride',ros.internal.ros2.messages.ros2.default_type('uint32',1,0));
coder.cstructname(msg,'std_msgs_MultiArrayDimensionStruct_T');
coder.varsize('msg.label',[1 1000000000],[0 1]);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
