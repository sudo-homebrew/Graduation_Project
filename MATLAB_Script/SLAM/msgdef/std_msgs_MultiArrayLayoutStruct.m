function msg = std_msgs_MultiArrayLayoutStruct
% Message struct definition for std_msgs/MultiArrayLayout
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/MultiArrayLayout',...
    'dim',std_msgs_MultiArrayDimensionStruct,...
    'data_offset',ros.internal.ros2.messages.ros2.default_type('uint32',1,0));
coder.cstructname(msg,'std_msgs_MultiArrayLayoutStruct_T');
coder.varsize('msg.dim',[1000000000 1],[1 0]);
msg.dim = msg.dim([],1);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
