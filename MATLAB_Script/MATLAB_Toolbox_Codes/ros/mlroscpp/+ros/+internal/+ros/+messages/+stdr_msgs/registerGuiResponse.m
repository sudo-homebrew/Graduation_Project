function [data, info] = registerGuiResponse
%RegisterGui gives an empty data for stdr_msgs/RegisterGuiResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/RegisterGuiResponse';
[data.Robots, info.Robots] = ros.internal.ros.messages.stdr_msgs.robotIndexedMsg;
info.Robots.MLdataType = 'struct';
info.Robots.MaxLen = NaN;
info.Robots.MinLen = 0;
data.Robots = data.Robots([],1);
info.MessageType = 'stdr_msgs/RegisterGuiResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,93);
info.MatPath{1} = 'robots';
info.MatPath{2} = 'robots.name';
info.MatPath{3} = 'robots.robot';
info.MatPath{4} = 'robots.robot.initialPose';
info.MatPath{5} = 'robots.robot.initialPose.x';
info.MatPath{6} = 'robots.robot.initialPose.y';
info.MatPath{7} = 'robots.robot.initialPose.theta';
info.MatPath{8} = 'robots.robot.footprint';
info.MatPath{9} = 'robots.robot.footprint.points';
info.MatPath{10} = 'robots.robot.footprint.points.x';
info.MatPath{11} = 'robots.robot.footprint.points.y';
info.MatPath{12} = 'robots.robot.footprint.points.z';
info.MatPath{13} = 'robots.robot.footprint.radius';
info.MatPath{14} = 'robots.robot.laserSensors';
info.MatPath{15} = 'robots.robot.laserSensors.maxAngle';
info.MatPath{16} = 'robots.robot.laserSensors.minAngle';
info.MatPath{17} = 'robots.robot.laserSensors.maxRange';
info.MatPath{18} = 'robots.robot.laserSensors.minRange';
info.MatPath{19} = 'robots.robot.laserSensors.numRays';
info.MatPath{20} = 'robots.robot.laserSensors.noise';
info.MatPath{21} = 'robots.robot.laserSensors.noise.noise';
info.MatPath{22} = 'robots.robot.laserSensors.noise.noiseMean';
info.MatPath{23} = 'robots.robot.laserSensors.noise.noiseStd';
info.MatPath{24} = 'robots.robot.laserSensors.frequency';
info.MatPath{25} = 'robots.robot.laserSensors.frame_id';
info.MatPath{26} = 'robots.robot.laserSensors.pose';
info.MatPath{27} = 'robots.robot.laserSensors.pose.x';
info.MatPath{28} = 'robots.robot.laserSensors.pose.y';
info.MatPath{29} = 'robots.robot.laserSensors.pose.theta';
info.MatPath{30} = 'robots.robot.sonarSensors';
info.MatPath{31} = 'robots.robot.sonarSensors.maxRange';
info.MatPath{32} = 'robots.robot.sonarSensors.minRange';
info.MatPath{33} = 'robots.robot.sonarSensors.coneAngle';
info.MatPath{34} = 'robots.robot.sonarSensors.frequency';
info.MatPath{35} = 'robots.robot.sonarSensors.noise';
info.MatPath{36} = 'robots.robot.sonarSensors.noise.noise';
info.MatPath{37} = 'robots.robot.sonarSensors.noise.noiseMean';
info.MatPath{38} = 'robots.robot.sonarSensors.noise.noiseStd';
info.MatPath{39} = 'robots.robot.sonarSensors.frame_id';
info.MatPath{40} = 'robots.robot.sonarSensors.pose';
info.MatPath{41} = 'robots.robot.sonarSensors.pose.x';
info.MatPath{42} = 'robots.robot.sonarSensors.pose.y';
info.MatPath{43} = 'robots.robot.sonarSensors.pose.theta';
info.MatPath{44} = 'robots.robot.rfidSensors';
info.MatPath{45} = 'robots.robot.rfidSensors.maxRange';
info.MatPath{46} = 'robots.robot.rfidSensors.angleSpan';
info.MatPath{47} = 'robots.robot.rfidSensors.signalCutoff';
info.MatPath{48} = 'robots.robot.rfidSensors.frequency';
info.MatPath{49} = 'robots.robot.rfidSensors.frame_id';
info.MatPath{50} = 'robots.robot.rfidSensors.pose';
info.MatPath{51} = 'robots.robot.rfidSensors.pose.x';
info.MatPath{52} = 'robots.robot.rfidSensors.pose.y';
info.MatPath{53} = 'robots.robot.rfidSensors.pose.theta';
info.MatPath{54} = 'robots.robot.co2Sensors';
info.MatPath{55} = 'robots.robot.co2Sensors.maxRange';
info.MatPath{56} = 'robots.robot.co2Sensors.frequency';
info.MatPath{57} = 'robots.robot.co2Sensors.frame_id';
info.MatPath{58} = 'robots.robot.co2Sensors.pose';
info.MatPath{59} = 'robots.robot.co2Sensors.pose.x';
info.MatPath{60} = 'robots.robot.co2Sensors.pose.y';
info.MatPath{61} = 'robots.robot.co2Sensors.pose.theta';
info.MatPath{62} = 'robots.robot.soundSensors';
info.MatPath{63} = 'robots.robot.soundSensors.maxRange';
info.MatPath{64} = 'robots.robot.soundSensors.frequency';
info.MatPath{65} = 'robots.robot.soundSensors.angleSpan';
info.MatPath{66} = 'robots.robot.soundSensors.frame_id';
info.MatPath{67} = 'robots.robot.soundSensors.pose';
info.MatPath{68} = 'robots.robot.soundSensors.pose.x';
info.MatPath{69} = 'robots.robot.soundSensors.pose.y';
info.MatPath{70} = 'robots.robot.soundSensors.pose.theta';
info.MatPath{71} = 'robots.robot.thermalSensors';
info.MatPath{72} = 'robots.robot.thermalSensors.maxRange';
info.MatPath{73} = 'robots.robot.thermalSensors.frequency';
info.MatPath{74} = 'robots.robot.thermalSensors.angleSpan';
info.MatPath{75} = 'robots.robot.thermalSensors.frame_id';
info.MatPath{76} = 'robots.robot.thermalSensors.pose';
info.MatPath{77} = 'robots.robot.thermalSensors.pose.x';
info.MatPath{78} = 'robots.robot.thermalSensors.pose.y';
info.MatPath{79} = 'robots.robot.thermalSensors.pose.theta';
info.MatPath{80} = 'robots.robot.kinematicModel';
info.MatPath{81} = 'robots.robot.kinematicModel.type';
info.MatPath{82} = 'robots.robot.kinematicModel.a_ux_ux';
info.MatPath{83} = 'robots.robot.kinematicModel.a_ux_uy';
info.MatPath{84} = 'robots.robot.kinematicModel.a_ux_w';
info.MatPath{85} = 'robots.robot.kinematicModel.a_uy_ux';
info.MatPath{86} = 'robots.robot.kinematicModel.a_uy_uy';
info.MatPath{87} = 'robots.robot.kinematicModel.a_uy_w';
info.MatPath{88} = 'robots.robot.kinematicModel.a_w_ux';
info.MatPath{89} = 'robots.robot.kinematicModel.a_w_uy';
info.MatPath{90} = 'robots.robot.kinematicModel.a_w_w';
info.MatPath{91} = 'robots.robot.kinematicModel.a_g_ux';
info.MatPath{92} = 'robots.robot.kinematicModel.a_g_uy';
info.MatPath{93} = 'robots.robot.kinematicModel.a_g_w';