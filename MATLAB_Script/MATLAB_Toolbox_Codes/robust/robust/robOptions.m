function obj = robOptions(varargin)
%ROBOPTIONS  Creates option set for the ROBSTAB and ROBGAIN commands.
%
%   OPT = ROBOPTIONS returns the default options for ROBSTAB and ROBGAIN.
%
%   OPT = ROBOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 
%   'Option1','Option2',...
%
%   Supported options include:
%
%   Display          Show progress and print a report [{'off'} | 'on'].
%
%   VaryFrequency    Capture how robustness margin varies with frequency
%                    [{'off'} | 'on']. For USS models, set this option to 
%                    'on' to compute the margin over a frequency grid
%                    suitable for plotting. By default, ROBSTAB and ROBGAIN 
%                    only return data at frequencies where robustness is 
%                    weakest. This option is ignored for UFRD models.
%
%   Sensitivity      Calculate sensitivity of robustness margin 
%                    [{'off'} | 'on']. Each uncertain element contributes, 
%                    in a coupled manner, to the overall stability margin.  
%                    Set this option to 'on' to estimate the sensitivity of 
%                    the robustness margin with respect to each individual 
%                    uncertain element. This gives an indication of which 
%                    uncertain elements are most problematic.
%
%   SensitivityPercent  The sensitivity of the robustness margin is
%                    estimated using finite difference calculations: For
%                    a given uncertain element, increase the normalized
%                    amount of uncertainty by some percentage, recompute
%                    the margin, and take the ratio of percent variations.
%                    This option specifies the percentage increase in
%                    uncertainty level applied to each element. The default 
%                    value is 25%.
%
%   MussvOptions     Option string for MUSSV. Default is '', resulting
%                    in the default behavior of MUSSV.
%
%   Example:
%      sys = feedback(tf(1,[1 0])*(1+0.5*ultidyn('Delta',[1 1])),1);
%      opt = robOptions('Display','on','Sensitivity','off');
%      [SM,WCU,Info] = robstab(sys,opt);
%      Info.Sensitivity      % NaN, since it was not computed
%
%   See also ROBSTAB, ROBGAIN.

%   Copyright 1984-2016 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.robAnalysis,varargin);
catch E
   throw(E)
end
