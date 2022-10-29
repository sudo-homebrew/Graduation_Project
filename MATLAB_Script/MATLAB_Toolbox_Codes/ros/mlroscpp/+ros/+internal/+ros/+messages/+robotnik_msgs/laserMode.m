function [data, info] = laserMode
%LaserMode gives an empty data for robotnik_msgs/LaserMode

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/LaserMode';
[data.STANDARD, info.STANDARD] = ros.internal.ros.messages.ros.char('string',0);
[data.STANDARD, info.STANDARD] = ros.internal.ros.messages.ros.char('string',1,'standard');
[data.DOCKINGSTATION, info.DOCKINGSTATION] = ros.internal.ros.messages.ros.char('string',0);
[data.DOCKINGSTATION, info.DOCKINGSTATION] = ros.internal.ros.messages.ros.char('string',1,'docking_station');
[data.CART, info.CART] = ros.internal.ros.messages.ros.char('string',0);
[data.CART, info.CART] = ros.internal.ros.messages.ros.char('string',1,'cart');
[data.CARTDOCKINGCART, info.CARTDOCKINGCART] = ros.internal.ros.messages.ros.char('string',0);
[data.CARTDOCKINGCART, info.CARTDOCKINGCART] = ros.internal.ros.messages.ros.char('string',1,'cart_docking_cart');
[data.DOCKINGCART, info.DOCKINGCART] = ros.internal.ros.messages.ros.char('string',0);
[data.DOCKINGCART, info.DOCKINGCART] = ros.internal.ros.messages.ros.char('string',1,'docking_cart');
[data.INVALID, info.INVALID] = ros.internal.ros.messages.ros.char('string',0);
[data.INVALID, info.INVALID] = ros.internal.ros.messages.ros.char('string',1,'invalid');
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'robotnik_msgs/LaserMode';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'STANDARD';
info.MatPath{2} = 'DOCKING_STATION';
info.MatPath{3} = 'CART';
info.MatPath{4} = 'CART_DOCKING_CART';
info.MatPath{5} = 'DOCKING_CART';
info.MatPath{6} = 'INVALID';
info.MatPath{7} = 'name';
