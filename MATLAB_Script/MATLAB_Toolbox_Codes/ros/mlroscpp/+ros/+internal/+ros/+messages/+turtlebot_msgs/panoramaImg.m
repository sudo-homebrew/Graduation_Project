function [data, info] = panoramaImg
%PanoramaImg gives an empty data for turtlebot_msgs/PanoramaImg

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtlebot_msgs/PanoramaImg';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.PanoId, info.PanoId] = ros.internal.ros.messages.ros.char('string',0);
[data.Latitude, info.Latitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Longitude, info.Longitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Heading, info.Heading] = ros.internal.ros.messages.ros.default_type('double',1);
[data.GeoTag, info.GeoTag] = ros.internal.ros.messages.ros.char('string',0);
[data.Image, info.Image] = ros.internal.ros.messages.sensor_msgs.image;
info.Image.MLdataType = 'struct';
info.MessageType = 'turtlebot_msgs/PanoramaImg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,24);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pano_id';
info.MatPath{8} = 'latitude';
info.MatPath{9} = 'longitude';
info.MatPath{10} = 'heading';
info.MatPath{11} = 'geo_tag';
info.MatPath{12} = 'image';
info.MatPath{13} = 'image.header';
info.MatPath{14} = 'image.header.seq';
info.MatPath{15} = 'image.header.stamp';
info.MatPath{16} = 'image.header.stamp.sec';
info.MatPath{17} = 'image.header.stamp.nsec';
info.MatPath{18} = 'image.header.frame_id';
info.MatPath{19} = 'image.height';
info.MatPath{20} = 'image.width';
info.MatPath{21} = 'image.encoding';
info.MatPath{22} = 'image.is_bigendian';
info.MatPath{23} = 'image.step';
info.MatPath{24} = 'image.data';