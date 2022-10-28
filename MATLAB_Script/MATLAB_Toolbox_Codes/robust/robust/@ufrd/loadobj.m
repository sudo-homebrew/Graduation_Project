function sys = loadobj(s)
%LOADOBJ  Load filter for UFRD objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'ufrd')
   sys = DynamicSystem.updateMetaData(s);
   sys.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   
   if isfield(s,'ResponseData')
      % Pre-MCOS
      R = s.ResponseData;
      if isa(R,'uss')
         % In OOP, UMATs could contain ULTIDYN blocks. Such UMATs are reloaded 
         % as USS models with static IC. Force it back to UMAT to facilitate
         % reconstruction
         R = uss2umat(R);
      end
      Units = strrep(s.Units,'rad/s','rad/TimeUnit');
      sys = frd(R,s.Frequency,'FrequencyUnit',Units);
      sys.Ts = s.Ts;  % to impress correct sampling time on ULTIDYN blocks
      % Normalize uncertainty
      sys = unormalize_(sys);
      % Restore metadata
      sys = reload(sys,s);
   end
end
