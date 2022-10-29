classdef (CaseInsensitiveProperties = true) ncfmrInfo < ltipack.balredInfo
   % INFO structure for NCFMR.
   
   % Copyright 2021 The MathWorks, Inc.
   properties
      GL
   end   
   
   methods
      
      function this = ncfmrInfo(balredInfo)
         % Constructor
         if nargin>0
            % Initialize with BALRED data for left NCF 
            this.Split = balredInfo.Split;
            this.as = balredInfo.as;
            this.bs = balredInfo.bs;
            this.cs = balredInfo.cs;
            this.es = balredInfo.es;
            this.d = balredInfo.d;
            this.L = balredInfo.L;
            this.R = balredInfo.R;
            this.u = balredInfo.u;
            this.v = balredInfo.v;
            this.Rr = balredInfo.Rr;
            this.Ro = balredInfo.Ro;
            this.ZeroTol = balredInfo.ZeroTol;
            this.HSV = balredInfo.HSV;
            this.ErrorBound = balredInfo.ErrorBound;
         end
      end
      
      function disp(this)
         % Make it look like a struct
         S = struct('GL',this.GL,'HSV',this.HSV,...
            'ErrorBound',this.ErrorBound);
         disp(S)
      end
      
   end
end