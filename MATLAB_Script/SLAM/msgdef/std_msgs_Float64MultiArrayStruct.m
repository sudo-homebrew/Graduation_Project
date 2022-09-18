function msg = std_msgs_Float64MultiArrayStruct
% Message struct definition for std_msgs/Float64MultiArray
coder.inline("never")
msg = struct(...
    'MessageType','std_msgs/Float64MultiArray',...
    'layout',std_msgs_MultiArrayLayoutStruct,...
    'data',ros.internal.ros2.messages.ros2.default_type('double',NaN,1));
coder.cstructname(msg,'std_msgs_Float64MultiArrayStruct_T');
coder.varsize('msg.data',[1000000000 1],[1 0]);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
