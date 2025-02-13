function [data, info] = interactiveMarkerControl
%InteractiveMarkerControl gives an empty data for visualization_msgs/InteractiveMarkerControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'visualization_msgs/InteractiveMarkerControl';
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.orientation, info.orientation] = ros.internal.ros2.messages.geometry_msgs.quaternion;
info.orientation.MLdataType = 'struct';
[data.INHERIT, info.INHERIT] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.FIXED, info.FIXED] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.VIEW_FACING, info.VIEW_FACING] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.orientation_mode, info.orientation_mode] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.NONE, info.NONE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.MENU, info.MENU] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.BUTTON, info.BUTTON] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.MOVE_AXIS, info.MOVE_AXIS] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 3, [NaN]);
[data.MOVE_PLANE, info.MOVE_PLANE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 4, [NaN]);
[data.ROTATE_AXIS, info.ROTATE_AXIS] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 5, [NaN]);
[data.MOVE_ROTATE, info.MOVE_ROTATE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 6, [NaN]);
[data.MOVE_3D, info.MOVE_3D] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 7, [NaN]);
[data.ROTATE_3D, info.ROTATE_3D] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 8, [NaN]);
[data.MOVE_ROTATE_3D, info.MOVE_ROTATE_3D] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 9, [NaN]);
[data.interaction_mode, info.interaction_mode] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.always_visible, info.always_visible] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.markers, info.markers] = ros.internal.ros2.messages.visualization_msgs.marker;
info.markers.MLdataType = 'struct';
info.markers.MaxLen = NaN;
info.markers.MinLen = 0;
[data.independent_marker_orientation, info.independent_marker_orientation] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.description, info.description] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'visualization_msgs/InteractiveMarkerControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,85);
info.MatPath{1} = 'name';
info.MatPath{2} = 'orientation';
info.MatPath{3} = 'orientation.x';
info.MatPath{4} = 'orientation.y';
info.MatPath{5} = 'orientation.z';
info.MatPath{6} = 'orientation.w';
info.MatPath{7} = 'INHERIT';
info.MatPath{8} = 'FIXED';
info.MatPath{9} = 'VIEW_FACING';
info.MatPath{10} = 'orientation_mode';
info.MatPath{11} = 'NONE';
info.MatPath{12} = 'MENU';
info.MatPath{13} = 'BUTTON';
info.MatPath{14} = 'MOVE_AXIS';
info.MatPath{15} = 'MOVE_PLANE';
info.MatPath{16} = 'ROTATE_AXIS';
info.MatPath{17} = 'MOVE_ROTATE';
info.MatPath{18} = 'MOVE_3D';
info.MatPath{19} = 'ROTATE_3D';
info.MatPath{20} = 'MOVE_ROTATE_3D';
info.MatPath{21} = 'interaction_mode';
info.MatPath{22} = 'always_visible';
info.MatPath{23} = 'markers';
info.MatPath{24} = 'markers.ARROW';
info.MatPath{25} = 'markers.CUBE';
info.MatPath{26} = 'markers.SPHERE';
info.MatPath{27} = 'markers.CYLINDER';
info.MatPath{28} = 'markers.LINE_STRIP';
info.MatPath{29} = 'markers.LINE_LIST';
info.MatPath{30} = 'markers.CUBE_LIST';
info.MatPath{31} = 'markers.SPHERE_LIST';
info.MatPath{32} = 'markers.POINTS';
info.MatPath{33} = 'markers.TEXT_VIEW_FACING';
info.MatPath{34} = 'markers.MESH_RESOURCE';
info.MatPath{35} = 'markers.TRIANGLE_LIST';
info.MatPath{36} = 'markers.ADD';
info.MatPath{37} = 'markers.MODIFY';
info.MatPath{38} = 'markers.DELETE';
info.MatPath{39} = 'markers.DELETEALL';
info.MatPath{40} = 'markers.header';
info.MatPath{41} = 'markers.header.stamp';
info.MatPath{42} = 'markers.header.stamp.sec';
info.MatPath{43} = 'markers.header.stamp.nanosec';
info.MatPath{44} = 'markers.header.frame_id';
info.MatPath{45} = 'markers.ns';
info.MatPath{46} = 'markers.id';
info.MatPath{47} = 'markers.type';
info.MatPath{48} = 'markers.action';
info.MatPath{49} = 'markers.pose';
info.MatPath{50} = 'markers.pose.position';
info.MatPath{51} = 'markers.pose.position.x';
info.MatPath{52} = 'markers.pose.position.y';
info.MatPath{53} = 'markers.pose.position.z';
info.MatPath{54} = 'markers.pose.orientation';
info.MatPath{55} = 'markers.pose.orientation.x';
info.MatPath{56} = 'markers.pose.orientation.y';
info.MatPath{57} = 'markers.pose.orientation.z';
info.MatPath{58} = 'markers.pose.orientation.w';
info.MatPath{59} = 'markers.scale';
info.MatPath{60} = 'markers.scale.x';
info.MatPath{61} = 'markers.scale.y';
info.MatPath{62} = 'markers.scale.z';
info.MatPath{63} = 'markers.color';
info.MatPath{64} = 'markers.color.r';
info.MatPath{65} = 'markers.color.g';
info.MatPath{66} = 'markers.color.b';
info.MatPath{67} = 'markers.color.a';
info.MatPath{68} = 'markers.lifetime';
info.MatPath{69} = 'markers.lifetime.sec';
info.MatPath{70} = 'markers.lifetime.nanosec';
info.MatPath{71} = 'markers.frame_locked';
info.MatPath{72} = 'markers.points';
info.MatPath{73} = 'markers.points.x';
info.MatPath{74} = 'markers.points.y';
info.MatPath{75} = 'markers.points.z';
info.MatPath{76} = 'markers.colors';
info.MatPath{77} = 'markers.colors.r';
info.MatPath{78} = 'markers.colors.g';
info.MatPath{79} = 'markers.colors.b';
info.MatPath{80} = 'markers.colors.a';
info.MatPath{81} = 'markers.text';
info.MatPath{82} = 'markers.mesh_resource';
info.MatPath{83} = 'markers.mesh_use_embedded_materials';
info.MatPath{84} = 'independent_marker_orientation';
info.MatPath{85} = 'description';
