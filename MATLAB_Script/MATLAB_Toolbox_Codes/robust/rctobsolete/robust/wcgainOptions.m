function obj = wcgainOptions(varargin)
%WCGAINOPTIONS  Creates option set for the WCGAIN command.
%
%   OPT = WCGAINOPTIONS returns the default options for WCGAIN.
%
%   OPT = WCGAINOPTIONS('Option1',Value1,'Option2',Value2,...) uses name/value
%   pairs to override the default values for 'Option1','Option2',...
%
%   Supported options include:
%
%   Sensitivity       Sensitivity of worst-case gain [{'on'} | 'off'].
%                     Each uncertain element contributes, in a coupled manner, 
%                     to the overall worst-case gain. When this option is  
%                     set to 'on', the sensitivity of the worst-case gain  
%                     is computed with respect to each individual uncertain
%                     element, giving an indication of which elements are  
%                     most problematic.
%
%   VaryUncertainty   Percentage variation of uncertainty for sensitivity
%                     calculations (sensitivity estimate uses a finite
%                     difference calculation). The default value is 25 (%).
%
%   LowerBoundOnly    Only compute lower bound [{'off'} | 'on'].
%                     When 'on', WCGAIN only computes a lower bound on the 
%                     worst-case gain.
%
%   MaxOverFrequency  Compute worst-case gain over frequency [{'on'} | 'off'].
%                     When 'on', WCGAIN computes the worst-case H-infinity 
%                     norm (peak gain over frequency). When 'off', WCGAIN 
%                     computes the worst-case gain at each frequency point. 
%
%   MaxOverArray      Compute worst-case gain over set of models [{'on'} | 'off'].
%                     This option is only relevant for arrays of uncertain 
%                     models and specifies whether to compute the worst-case 
%                     gain over all models ('on') or for each model separately
%                     ('off').
%
%   AbsTol, RelTol    Absolute and relative tolerances on computed bounds.
%                     The algorithm terminates when 
%                       UpperBound-LowerBound <= max(AbsTol,Reltol*UpperBound).
%                     The default values are 0.02 and 0.05.
%
%   AbsMax, RelMax    Absolute and relative thresholds for lower bound. The
%                     algorithm terminates when
%                       LowerBound >= AbsMax + RelMax * NominalGain.
%                     The default values are 5 and 20. Use these options to
%                     terminate when the lower bound is large enough compared
%                     to the nominal gain.
%
%   NSearch           Number of lower bound searches at each frequency 
%                     (default = 2).
%
%   Example:
%      sys = feedback(tf(1,[1 0])*(1+0.5*ultidyn('Delta',[1 1])),1);
%      opt = wcgainOptions('MaxOverFrequency','off','RelTol',0.1);
%      [WCG,WCU,Info] = wcgain(sys,opt);
%      bodemag(WCG.UpperBound)  % upper bound as function of frequency
%
%   See also WCGAIN.

%   Copyright 1984-2011 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.wcgain,varargin);
catch E
   throw(E)
end
