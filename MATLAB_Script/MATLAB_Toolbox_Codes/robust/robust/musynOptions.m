function obj = musynOptions(varargin)
%MUSYNOPTIONS  Creates option set for the MUSYN command.
%
%   OPT = MUSYNOPTIONS returns the default options for the MUSYN command.
%
%   OPT = MUSYNOPTIONS('Option1',Value1,'Option2',Value2,...) uses name/value
%   pairs to override the default values for 'Option1','Option2',...
%
%   Supported options include:
%
%   Display        Display level [{'short'} | 'full' | 'off']
%                  By default, MUSYN prints a brief summary after each
%                  iteration. Set Display='full' to print detailed results
%                  and pause after each iteration. Set Display='off' to 
%                  suppress all display.
%
%   MaxIter        Maximum number of D-K iterations (default = 10).
%
%   TargetPerf     Target robust H-infinity performance (default = 0).
%                  D-K iteration terminates when the robust H-infinity 
%                  performance drops below this target value. See MUSYNPERF
%                  for details on the robust H-infinity performance.
%
%   TolPerf        Stopping tolerance (default = 0.01). 
%                  D-K iteration terminates when the performance improved 
%                  by less than TolPerf over the last two iterations. Set 
%                  TolPerf=0 to always complete MaxIter iterations.
%
%   The following options apply to the D step (mu analysis and D,G fitting):
%
%   MixedMU        Perform real/complex mu-synthesis [{'off'} | 'on'].
%                  When ureal parameters are present, set MixedMU='on' to
%                  use G scalings and obtain sharper mu upper bounds.
%
%   FullDG         Use full D,G scalings for repeated uncertainty.
%                  By default, MUSYN uses full D,G scalings for repeated 
%                  ureal or ucomplex blocks. Set FullDG=false to restrict 
%                  D,G to be diagonal, or FullDG=[true false] to use full D 
%                  and diagonal G. Using full scalings is less conservative
%                  but often impractical for more than a few repetitions.
%
%   FitOrder       Maximum order for fitting D,G scaling data.
%                  The default is [5 2], meaning order 5 for D scalings
%                  and order 2 for G scalings. Each entry of D,G is fitted
%                  by a rational function whose order is automatically
%                  selected between 0 and the maximum order. For diagonal
%                  entries of G, this maximum order is in addition to the 
%                  zeros needed to capture sign changes.
%
%   FrequencyGrid  Frequency grid used for mu analysis (default = []).
%                  When this vector is empty, the frequency range and
%                  number of points are chosen automatically.
%
%   The following options apply to the K step when using HINFSYN:
%
%   AutoScale      Automatic plant scaling [{'on'} | 'off'].
%                  HINFSYN automatically scales the plant states, controls,
%                  and measurements to improve numerical accuracy. The
%                  controller K is always returned in the original coordinates.
%                  Set AutoScale='off' to skip this step when your plant is
%                  known to be well scaled.
%
%   Regularize     Automatic regularization [{'on'} | 'off'].
%                  HINFSYN automatically regularizes the plant to enforce
%                  the requirements on P12 and P21. Set Regularize='off' to
%                  skip this when your problem is far enough from singular.
%
%   LimitGain      Limit controller gains [{'on'} | 'off'].
%                  In continuous time, near-singular D12 or D21 can result
%                  in controllers with large coefficients and fast dynamics.
%                  Use this option to compute a controller with similar
%                  performance but lower gains and better conditioning.
%
%   The following options apply to the K step when using HINFSTRUCT:
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
%   MinDecay       Minimum decay rate for closed-loop poles (default = 1e-7).
%                  Constrains all closed-loop poles to satisfy:
%                       Re(p) < -MinDecay.
%                  Increase this value to push the closed-loop poles farther
%                  into the stable region.
%
%   MaxFrequency   Maximum natural frequency of closed-loop poles (default=Inf).
%                  Constrains the closed-loop poles to satisfy:
%                       |p| < MaxFrequency.
%                  Use this option to prevent fast dynamics and high-gain
%                  control.
%
%   See also MUSYN.

%   Copyright 2003-2019 The MathWorks, Inc.
try
   obj = initOptions(rctoptions.musyn,varargin);
catch E
   throw(E)
end
