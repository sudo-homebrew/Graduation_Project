function obj = hinfstructOptions(varargin)
%HINFSTRUCTOPTIONS  Creates option set for the HINFSTRUCT command.
%
%   OPT = HINFSTRUCTOPTIONS returns the default options for the HINFSTRUCT 
%   command.
%
%   OPT = HINFSTRUCTOPTIONS('Option1',Value1,'Option2',Value2,...) uses 
%   name/value pairs to override the default values for 'Option1','Option2',...
%
%   Supported options include:
%
%   Display        Display level [{'final'} | 'iter' | 'off'].
%                  Set Display='final' to print a one-line summary at the   
%                  end of each optimization run (default). Set Display='iter'
%                  to show the optimization progress after each iteration. 
%                  Set Display='off' to run HINFSTRUCT in silent mode. 
%
%   MaxIter        Maximum number of iterations (default = 300).
%
%   RandomStart    Number of randomized starts (default = 0).
%                  Setting RandomStart=0 runs a single optimization starting 
%                  from the initial values of the tunable blocks. Setting
%                  RandomStart=N>0 runs N additional optimizations starting 
%                  from N randomly generated values of the block parameters. 
%                  Running a few random starts helps mitigate the risk of 
%                  premature termination due to local minima. 
%
%   UseParallel    Parallel processing flag (default = false).
%                  Setting UseParallel=true enables parallel processing by
%                  distributing the randomized starts among MATLAB workers
%                  and running the optimizations concurrently. This option
%                  requires the Parallel Computing Toolbox.
%
%   TargetGain     Target peak closed-loop gain (default = 0).
%                  The optimization stops when the peak gain (H-infinity
%                  norm) falls below the specified TargetGain value. Set 
%                  TargetGain=0 to minimize the peak closed-loop gain. Set 
%                  TargetGain=Inf to just stabilize the closed-loop system.
%
%   TolGain        Relative tolerance for termination (default = 1e-3).
%                  The optimization stops when the H-infinity norm decreases
%                  by less than this relative amount over 10 consecutive 
%                  iterations. Increasing TolGain speeds up termination, 
%                  decreasing TolGain yields tighter final values.
%
%   MinDecay       Minimum decay rate for closed-loop poles (default = 1e-7).
%                  Constrains all closed-loop poles to satisfy:
%                     Re(p) < -MinDecay.
%                  Increase this value to push the closed-loop poles farther 
%                  into the stable region.
%
%   MaxFrequency   Maximum natural frequency of closed-loop poles (default=Inf).
%                  Constrains the closed-loop poles to satisfy: 
%                     |p| < MaxFrequency. 
%                  Use this option to prevent fast dynamics and high-gain 
%                  control.
%
%   Example: Tune the design parameters to drive the peak closed-loop gain 
%   below 1, using three randomized starts and a minimum decay rate of 1e-3:
%      opt = hinfstructOptions(...
%                'TargetGain',1,'RandomStart',3,'MinDecay',1e-3);
%      CL = hinfstruct(CL0,opt)
%
%   See also HINFSTRUCT.

%   Author(s): P.Gahinet
%   Copyright 1984-2012 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.hinfstruct,varargin);
catch E
   throw(E)
end
