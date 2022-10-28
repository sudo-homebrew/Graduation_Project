function mybus = getBusObject(busElements, msgType)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2020 The MathWorks, Inc.

% GETBUSOBJECT Return a Simulink.Bus object and set the Elements property
% of the Bus object to the provided input argument, BUSELEMENTS.

businfo = ros.slros.internal.bus.BusItemInfo;
businfo.MsgType = msgType;
mybus = Simulink.Bus;
mybus.HeaderFile = '';
mybus.Description = businfo.toDescription();
mybus.DataScope = 'Auto';
mybus.Alignment = -1;
mybus.Elements = busElements;
end
