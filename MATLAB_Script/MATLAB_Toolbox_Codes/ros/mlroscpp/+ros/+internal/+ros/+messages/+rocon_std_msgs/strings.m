function [data, info] = strings
%Strings gives an empty data for rocon_std_msgs/Strings

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/Strings';
[data.ROCONVERSION, info.ROCONVERSION] = ros.internal.ros.messages.ros.char('string',0);
[data.ROCONVERSION, info.ROCONVERSION] = ros.internal.ros.messages.ros.char('string',1,'acdc');
[data.URIWILDCARD, info.URIWILDCARD] = ros.internal.ros.messages.ros.char('string',0);
[data.URIWILDCARD, info.URIWILDCARD] = ros.internal.ros.messages.ros.char('string',1,'*');
[data.HWPC, info.HWPC] = ros.internal.ros.messages.ros.char('string',0);
[data.HWPC, info.HWPC] = ros.internal.ros.messages.ros.char('string',1,'pc');
[data.HWTURTLEBOT2, info.HWTURTLEBOT2] = ros.internal.ros.messages.ros.char('string',0);
[data.HWTURTLEBOT2, info.HWTURTLEBOT2] = ros.internal.ros.messages.ros.char('string',1,'turtlebot2');
[data.HWPR2, info.HWPR2] = ros.internal.ros.messages.ros.char('string',0);
[data.HWPR2, info.HWPR2] = ros.internal.ros.messages.ros.char('string',1,'pr2');
[data.HWWAITERBOT, info.HWWAITERBOT] = ros.internal.ros.messages.ros.char('string',0);
[data.HWWAITERBOT, info.HWWAITERBOT] = ros.internal.ros.messages.ros.char('string',1,'waiterbot');
[data.HWROBOTOTHER, info.HWROBOTOTHER] = ros.internal.ros.messages.ros.char('string',0);
[data.HWROBOTOTHER, info.HWROBOTOTHER] = ros.internal.ros.messages.ros.char('string',1,'robot_other');
[data.HWGALAXY, info.HWGALAXY] = ros.internal.ros.messages.ros.char('string',0);
[data.HWGALAXY, info.HWGALAXY] = ros.internal.ros.messages.ros.char('string',1,'galaxy');
[data.HWMEGA, info.HWMEGA] = ros.internal.ros.messages.ros.char('string',0);
[data.HWMEGA, info.HWMEGA] = ros.internal.ros.messages.ros.char('string',1,'mega');
[data.HWNOTE3, info.HWNOTE3] = ros.internal.ros.messages.ros.char('string',0);
[data.HWNOTE3, info.HWNOTE3] = ros.internal.ros.messages.ros.char('string',1,'note3');
[data.HWPHONEOTHER, info.HWPHONEOTHER] = ros.internal.ros.messages.ros.char('string',0);
[data.HWPHONEOTHER, info.HWPHONEOTHER] = ros.internal.ros.messages.ros.char('string',1,'phone_other');
[data.HWXOOM, info.HWXOOM] = ros.internal.ros.messages.ros.char('string',0);
[data.HWXOOM, info.HWXOOM] = ros.internal.ros.messages.ros.char('string',1,'xoom');
[data.HWNOTE10, info.HWNOTE10] = ros.internal.ros.messages.ros.char('string',0);
[data.HWNOTE10, info.HWNOTE10] = ros.internal.ros.messages.ros.char('string',1,'note10');
[data.HWTABLETOTHER, info.HWTABLETOTHER] = ros.internal.ros.messages.ros.char('string',0);
[data.HWTABLETOTHER, info.HWTABLETOTHER] = ros.internal.ros.messages.ros.char('string',1,'tablet_other');
[data.APPLICATIONFRAMEWORKOTHER, info.APPLICATIONFRAMEWORKOTHER] = ros.internal.ros.messages.ros.char('string',0);
[data.APPLICATIONFRAMEWORKOTHER, info.APPLICATIONFRAMEWORKOTHER] = ros.internal.ros.messages.ros.char('string',1,'application_framework_other');
[data.APPLICATIONFRAMEWORKOPROS, info.APPLICATIONFRAMEWORKOPROS] = ros.internal.ros.messages.ros.char('string',0);
[data.APPLICATIONFRAMEWORKOPROS, info.APPLICATIONFRAMEWORKOPROS] = ros.internal.ros.messages.ros.char('string',1,'opros');
[data.APPLICATIONFRAMEWORKGROOVY, info.APPLICATIONFRAMEWORKGROOVY] = ros.internal.ros.messages.ros.char('string',0);
[data.APPLICATIONFRAMEWORKGROOVY, info.APPLICATIONFRAMEWORKGROOVY] = ros.internal.ros.messages.ros.char('string',1,'groovy');
[data.APPLICATIONFRAMEWORKHYDRO, info.APPLICATIONFRAMEWORKHYDRO] = ros.internal.ros.messages.ros.char('string',0);
[data.APPLICATIONFRAMEWORKHYDRO, info.APPLICATIONFRAMEWORKHYDRO] = ros.internal.ros.messages.ros.char('string',1,'hydro');
[data.APPLICATIONFRAMEWORKINDIGO, info.APPLICATIONFRAMEWORKINDIGO] = ros.internal.ros.messages.ros.char('string',0);
[data.APPLICATIONFRAMEWORKINDIGO, info.APPLICATIONFRAMEWORKINDIGO] = ros.internal.ros.messages.ros.char('string',1,'indigo');
[data.APPLICATIONFRAMEWORKROSOTHER, info.APPLICATIONFRAMEWORKROSOTHER] = ros.internal.ros.messages.ros.char('string',0);
[data.APPLICATIONFRAMEWORKROSOTHER, info.APPLICATIONFRAMEWORKROSOTHER] = ros.internal.ros.messages.ros.char('string',1,'ros_other');
[data.OSOSX, info.OSOSX] = ros.internal.ros.messages.ros.char('string',0);
[data.OSOSX, info.OSOSX] = ros.internal.ros.messages.ros.char('string',1,'osx');
[data.OSFREEBSD, info.OSFREEBSD] = ros.internal.ros.messages.ros.char('string',0);
[data.OSFREEBSD, info.OSFREEBSD] = ros.internal.ros.messages.ros.char('string',1,'freebsd');
[data.OSWINXP, info.OSWINXP] = ros.internal.ros.messages.ros.char('string',0);
[data.OSWINXP, info.OSWINXP] = ros.internal.ros.messages.ros.char('string',1,'winxp');
[data.OSWINDOWS7, info.OSWINDOWS7] = ros.internal.ros.messages.ros.char('string',0);
[data.OSWINDOWS7, info.OSWINDOWS7] = ros.internal.ros.messages.ros.char('string',1,'windows7');
[data.OSARCH, info.OSARCH] = ros.internal.ros.messages.ros.char('string',0);
[data.OSARCH, info.OSARCH] = ros.internal.ros.messages.ros.char('string',1,'arch');
[data.OSDEBIAN, info.OSDEBIAN] = ros.internal.ros.messages.ros.char('string',0);
[data.OSDEBIAN, info.OSDEBIAN] = ros.internal.ros.messages.ros.char('string',1,'debian');
[data.OSFEDORA, info.OSFEDORA] = ros.internal.ros.messages.ros.char('string',0);
[data.OSFEDORA, info.OSFEDORA] = ros.internal.ros.messages.ros.char('string',1,'fedora');
[data.OSGENTOO, info.OSGENTOO] = ros.internal.ros.messages.ros.char('string',0);
[data.OSGENTOO, info.OSGENTOO] = ros.internal.ros.messages.ros.char('string',1,'gentoo');
[data.OSPRECISE, info.OSPRECISE] = ros.internal.ros.messages.ros.char('string',0);
[data.OSPRECISE, info.OSPRECISE] = ros.internal.ros.messages.ros.char('string',1,'precise');
[data.OSQUANTAL, info.OSQUANTAL] = ros.internal.ros.messages.ros.char('string',0);
[data.OSQUANTAL, info.OSQUANTAL] = ros.internal.ros.messages.ros.char('string',1,'quantal');
[data.OSRARING, info.OSRARING] = ros.internal.ros.messages.ros.char('string',0);
[data.OSRARING, info.OSRARING] = ros.internal.ros.messages.ros.char('string',1,'raring');
[data.OSSAUCY, info.OSSAUCY] = ros.internal.ros.messages.ros.char('string',0);
[data.OSSAUCY, info.OSSAUCY] = ros.internal.ros.messages.ros.char('string',1,'saucy');
[data.OSHONEYCOMB, info.OSHONEYCOMB] = ros.internal.ros.messages.ros.char('string',0);
[data.OSHONEYCOMB, info.OSHONEYCOMB] = ros.internal.ros.messages.ros.char('string',1,'honeycomb');
[data.OSICECREAMSANDWICH, info.OSICECREAMSANDWICH] = ros.internal.ros.messages.ros.char('string',0);
[data.OSICECREAMSANDWICH, info.OSICECREAMSANDWICH] = ros.internal.ros.messages.ros.char('string',1,'ice_cream_sandwich');
[data.OSJELLYBEAN, info.OSJELLYBEAN] = ros.internal.ros.messages.ros.char('string',0);
[data.OSJELLYBEAN, info.OSJELLYBEAN] = ros.internal.ros.messages.ros.char('string',1,'jellybean');
[data.OSKITKAT, info.OSKITKAT] = ros.internal.ros.messages.ros.char('string',0);
[data.OSKITKAT, info.OSKITKAT] = ros.internal.ros.messages.ros.char('string',1,'kitkat');
[data.OSCHROME, info.OSCHROME] = ros.internal.ros.messages.ros.char('string',0);
[data.OSCHROME, info.OSCHROME] = ros.internal.ros.messages.ros.char('string',1,'chrome');
[data.OSFIREFOX, info.OSFIREFOX] = ros.internal.ros.messages.ros.char('string',0);
[data.OSFIREFOX, info.OSFIREFOX] = ros.internal.ros.messages.ros.char('string',1,'firefox');
[data.OSINTERNETEXPLORER, info.OSINTERNETEXPLORER] = ros.internal.ros.messages.ros.char('string',0);
[data.OSINTERNETEXPLORER, info.OSINTERNETEXPLORER] = ros.internal.ros.messages.ros.char('string',1,'internet_explorer');
[data.OSSAFARI, info.OSSAFARI] = ros.internal.ros.messages.ros.char('string',0);
[data.OSSAFARI, info.OSSAFARI] = ros.internal.ros.messages.ros.char('string',1,'safari');
[data.OSOPERA, info.OSOPERA] = ros.internal.ros.messages.ros.char('string',0);
[data.OSOPERA, info.OSOPERA] = ros.internal.ros.messages.ros.char('string',1,'opera');
[data.TAGSERVICE, info.TAGSERVICE] = ros.internal.ros.messages.ros.char('string',0);
[data.TAGSERVICE, info.TAGSERVICE] = ros.internal.ros.messages.ros.char('string',1,'concert_service');
[data.TAGRAPP, info.TAGRAPP] = ros.internal.ros.messages.ros.char('string',0);
[data.TAGRAPP, info.TAGRAPP] = ros.internal.ros.messages.ros.char('string',1,'rocon_app');
[data.TAGGAZEBOROBOTTYPE, info.TAGGAZEBOROBOTTYPE] = ros.internal.ros.messages.ros.char('string',0);
[data.TAGGAZEBOROBOTTYPE, info.TAGGAZEBOROBOTTYPE] = ros.internal.ros.messages.ros.char('string',1,'concert_gazebo');
[data.TAGSOFTWARE, info.TAGSOFTWARE] = ros.internal.ros.messages.ros.char('string',0);
[data.TAGSOFTWARE, info.TAGSOFTWARE] = ros.internal.ros.messages.ros.char('string',1,'software_farm');
info.MessageType = 'rocon_std_msgs/Strings';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,45);
info.MatPath{1} = 'ROCON_VERSION';
info.MatPath{2} = 'URI_WILDCARD';
info.MatPath{3} = 'HW_PC';
info.MatPath{4} = 'HW_TURTLEBOT2';
info.MatPath{5} = 'HW_PR2';
info.MatPath{6} = 'HW_WAITERBOT';
info.MatPath{7} = 'HW_ROBOT_OTHER';
info.MatPath{8} = 'HW_GALAXY';
info.MatPath{9} = 'HW_MEGA';
info.MatPath{10} = 'HW_NOTE3';
info.MatPath{11} = 'HW_PHONE_OTHER';
info.MatPath{12} = 'HW_XOOM';
info.MatPath{13} = 'HW_NOTE10';
info.MatPath{14} = 'HW_TABLET_OTHER';
info.MatPath{15} = 'APPLICATION_FRAMEWORK_OTHER';
info.MatPath{16} = 'APPLICATION_FRAMEWORK_OPROS';
info.MatPath{17} = 'APPLICATION_FRAMEWORK_GROOVY';
info.MatPath{18} = 'APPLICATION_FRAMEWORK_HYDRO';
info.MatPath{19} = 'APPLICATION_FRAMEWORK_INDIGO';
info.MatPath{20} = 'APPLICATION_FRAMEWORK_ROS_OTHER';
info.MatPath{21} = 'OS_OSX';
info.MatPath{22} = 'OS_FREEBSD';
info.MatPath{23} = 'OS_WINXP';
info.MatPath{24} = 'OS_WINDOWS7';
info.MatPath{25} = 'OS_ARCH';
info.MatPath{26} = 'OS_DEBIAN';
info.MatPath{27} = 'OS_FEDORA';
info.MatPath{28} = 'OS_GENTOO';
info.MatPath{29} = 'OS_PRECISE';
info.MatPath{30} = 'OS_QUANTAL';
info.MatPath{31} = 'OS_RARING';
info.MatPath{32} = 'OS_SAUCY';
info.MatPath{33} = 'OS_HONEYCOMB';
info.MatPath{34} = 'OS_ICE_CREAM_SANDWICH';
info.MatPath{35} = 'OS_JELLYBEAN';
info.MatPath{36} = 'OS_KITKAT';
info.MatPath{37} = 'OS_CHROME';
info.MatPath{38} = 'OS_FIREFOX';
info.MatPath{39} = 'OS_INTERNET_EXPLORER';
info.MatPath{40} = 'OS_SAFARI';
info.MatPath{41} = 'OS_OPERA';
info.MatPath{42} = 'TAG_SERVICE';
info.MatPath{43} = 'TAG_RAPP';
info.MatPath{44} = 'TAG_GAZEBO_ROBOT_TYPE';
info.MatPath{45} = 'TAG_SOFTWARE';