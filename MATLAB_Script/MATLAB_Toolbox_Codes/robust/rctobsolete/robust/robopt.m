classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) robopt < rctoptions.robustperf
   %ROBOPT  Options for ROBUSTSTAB and ROBUSTPERF.
   %
   %   This function is obsolete, use robuststabOptions and robustperfOptions instead.
   %
   %   See also robuststabOptions, robustperfOptions.
   
%   Copyright 2003-2011 The MathWorks, Inc.
   properties (Dependent, Transient, SetAccess = private)
      Default
   end
   
   methods
      
      function opt = robopt(varargin)
         set(opt,varargin{:})
      end
      
      function V = get.Default(~)
         V = get(robustperfOptions);
      end

   end
   
   methods (Static, Hidden)
      
      function opt = loadobj(s)
         %LOADOBJ  Load filter for ROBOPT objects
         if isa(s,'robopt')
            opt = s;
         else
            % Remap old option object to new option object
            opt = robustperfOptions;
            for ct=1:numel(s.OptCell)
               sopt = s.OptCell{ct};
               opt.(sopt.Name) = sopt.Value;
            end
         end
      end
      
   end
      
end
