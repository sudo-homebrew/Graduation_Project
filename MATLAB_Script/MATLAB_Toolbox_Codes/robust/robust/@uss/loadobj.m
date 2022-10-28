function sys = loadobj(s)
%LOADOBJ  Load filter for USS objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'uss')
   sys = DynamicSystem.updateMetaData(s);
   sys.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   
   if isfield(s,'UmatData')
      % Pre-MCOS]
      StateName = s.StateName;
      nx = numel(StateName);
      sys = lft(ss(zeros(nx),eye(nx),eye(nx),zeros(nx),s.Ts),s.UmatData);
      sys.StateName = StateName;
      % Normalize uncertainty
      sys = unormalize_(sys);
      % Restore metadata
      sys = reload(sys,s);
   end
   
end