function obj = wcOptions(varargin)
%WCOPTIONS  Creates option set for worst-case analysis.
%
%   OPT = WCOPTIONS returns the default options for WCGAIN, WCDISKMARGIN,
%   and WCSIGMA.
%
%   OPT = WCOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 
%   'Option1','Option2',...
%
%   Supported options include:
%
%   ULevel           Uncertainty level (default=1). Use this option to
%                    scale the normalized uncertainty up or down by a 
%                    factor ULevel and see the effect on the worst-case 
%                    gain. The default level ULevel=1 corresponds to the 
%                    specified amount of uncertainty. 
%
%   Display          Show progress and print a report [{'off'} | 'on'].
%
%   VaryFrequency    Capture how worst-case gain varies with frequency
%                    [{'off'} | 'on']. For USS models, set this option to 
%                    'on' to compute the worst-case gain over a frequency   
%                    grid suitable for plotting. By default, WCGAIN only
%                    returns data at peak frequencies. This option is 
%                    ignored for UFRD models.
%
%   Sensitivity      Calculate worst-case gain sensitivity [{'off'} | 'on']. 
%                    Each uncertain element contributes, in a coupled manner, 
%                    to the overall worst case. Set this option to 'on' to 
%                    estimate the sensitivity of the worst-case gain with
%                    respect to each individual uncertain element. This 
%                    gives an indication of which uncertain elements are
%                    most problematic.
%
%   SensitivityPercent  The sensitivity of the worst-case gain is
%                    estimated using finite difference calculations: For
%                    a given uncertain element, increase the normalized
%                    amount of uncertainty by some percentage, recompute
%                    the worst-case gain, and take the ratio of percent
%                    variations. This option specifies the percentage
%                    increase in uncertainty level applied to each element.
%                    The default value is 25%.
%
%   MussvOptions     Option string for MUSSV. Default is '', resulting
%                    in the default behavior of MUSSV.
%
%   Example:
%      sys = feedback(tf(1,[1 0])*(1+0.5*ultidyn('Delta',[1 1])),1);
%      opt = wcOptions('Sensitivity','on','ULevel',1.5);
%      [WCG,WCU,Info] = wcgain(sys,opt);
%      Info.Sensitivity 
%
%   See also WCGAIN, WCSIGMAPLOT, WCDISKMARGIN.

%   Copyright 1984-2016 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.wcAnalysis,varargin);
catch E
   throw(E)
end
