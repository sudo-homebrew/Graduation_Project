function [sys,opt] = robSetUp(sys,opt,w)
% Setup for ROBSTAB, ROBGAIN, WCGAIN, and DKSYNPERF.

%   Copyright 2004-2016 The MathWorks, Inc.

% Validate and adjust options
if nmodels(sys)>1 && strcmp(opt.VaryFrequency,'on')
   error(message('Robust:analysis:ROBSTAB3'))
elseif strcmp(opt.Display,'on')
   opt.MussvOptions(:,opt.MussvOptions=='s') = [];
else
   opt.MussvOptions = [opt.MussvOptions 's'];
end

% Convert SYS to USS or UFRD
sys = ulti(sys);

% Process frequency spec
if nargin>2
   w = DynamicSystem.checkFreqSpec(w,true);
   if iscell(w)
      opt.Focus = [w{:}];
   else
      % ROBSTAB(USS,W) equivalent to ROBSTAB(UFRD(USS,W)) when
      % nominal model is stable
      if isa(sys,'FRDModel')
         error(message('Robust:analysis:ROBSTAB1'))
      end
      sys0 = sys.NominalValue;
      if hasInternalDelay(sys0)
         sys0 = pade(sys0,0,0,opt.PadeN);
      end
      if ~isstable(sys0)
         error(message('Robust:analysis:ROBSTAB4'))
      end
      sys = ufrd(sys,w);
   end
end