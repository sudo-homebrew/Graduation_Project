classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) dkitopt < rctoptions.dksyn
   %DKITOPT  Options for DKSYN.
   %
   %   This function is obsolete, use DKSYNOPTIONS instead.
   %
   %   See also DKSYNOPTIONS.
   
%   Copyright 2003-2011 The MathWorks, Inc.
   properties (Dependent, Transient, SetAccess = private)
      Default
   end
   
   methods
      
      function opt = dkitopt(varargin)
         set(opt,varargin{:})
      end
      
      function V = get.Default(~)
         V = get(dksynOptions);
      end

   end
   
   methods (Static, Hidden)
      
      function opt = loadobj(s)
         %LOADOBJ  Load filter for UREAL objects
         if isa(s,'dkitopt')
            opt = s;
         else
            % REmap old option object to new option object
            opt = dksynOptions;
            for ct=1:numel(s.OptCell)
               sopt = s.OptCell{ct};
               opt.(sopt.Name) = sopt.Value;
            end
         end
      end
      
   end
      
end
