function obj = dksynOptions(varargin)
%DKSYNOPTIONS  Creates option set for the DKSYN command.
%
%   OPT = DKSYNOPTIONS returns the default options for the DKSYN command.
%
%   OPT = DKSYNOPTIONS('Option1',Value1,'Option2',Value2,...) uses name/value 
%   pairs to override the default values for 'Option1','Option2',...
%
%   Supported options include:
%
%   FrequencyVector    Frequency vector used for analysis (default = []).
%                      When empty, the frequency range and number of points  
%                      are chosen automatically.
% 
%   InitialController  Controller used to initiate first iteration
%                      (default = []).
%
%   AutoIter           Automated mu-synthesis mode [{'on'} | 'off']. 
%
%   DisplayWhileAutoIter      Display of iteration progress in AutoIter mode 
%                             [{'off'} | 'on']. 
%
%   StartingIterationNumber   Starting iteration number (default = 1).
%
%   NumberOfAutoIterations    Number of iterations to perform (default = 10).
%
%   MixedMU            Perform real/complex mu-synthesis when real parameters
%                      are present [{'off'} | 'on']. 
%
%   AutoScalingOrder   State order for fitting D-scaling and G-scaling data
%                      for real/complex mu-synthesis. Default is [5 2], 
%                      5th order D-scalings and 2nd order G-scalings.
%
%   AutoIterSmartTerminate    Automatic termination of iteration procedure 
%                             based on progress of design iteration 
%                             [{'on'} | 'off']. 
% 
%   AutoIterSmartTerminateTol Tolerance for AutoIterSmartTerminate mode
%                             (default = 0.005).
%
%   See also DKSYN.

%   Copyright 2003-2012 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.dksyn,varargin);
catch E
   throw(E)
end
