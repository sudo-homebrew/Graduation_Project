classdef (CaseInsensitiveProperties = true, TruncatedProperties = true) wcgopt < rctoptions.wcgain
   %WCGOPT  Options for WCGAIN.
   %
   %   This function is obsolete, use wcgainOptions instead.
   %
   %   See also wcgainOptions.
   
%   Copyright 2003-2011 The MathWorks, Inc.
   properties (Dependent, Transient)
      FreqPtWise
      ArrayDimPtWise
      MBadThreshold
      ABadThreshold
      NTimes
   end
   
   properties 
      % No longer used
      MGoodThreshold = 1.04;
      AGoodThreshold = 5e-2;
   end
   
   properties (Dependent, Transient, SetAccess = private)
      Default
   end

   methods
      
      function opt = wcgopt(varargin)
         set(opt,varargin{:})
      end
      
      function V = get.Default(~)
         V = get(wcgainOptions);
      end
      function V = get.FreqPtWise(opt)
         V = strcmp(opt.MaxOverFrequency,'off');
      end
      function V = get.ArrayDimPtWise(~)
         % Note: Pointwise along a subset of dimensions is no longer supported
         V = [];
      end
      function V = get.MBadThreshold(opt)
         V = opt.RelMax;
      end
      function V = get.ABadThreshold(opt)
         V = opt.AbsMax;
      end
      function V = get.NTimes(opt)
         V = opt.NSearch;
      end
      function opt = set.FreqPtWise(opt,V)
         if V
            opt.MaxOverFrequency = 'off';
         else
            opt.MaxOverFrequency = 'on';
         end
      end
      function opt = set.ArrayDimPtWise(opt,V)
         if isempty(V)
            opt.MaxOverArray = 'on';
         else
            opt.MaxOverArray = 'off';
         end
      end
      function opt = set.MBadThreshold(opt,V)
         opt.RelMax = V;
      end
      function opt = set.ABadThreshold(opt,V)
         opt.AbsMax = V;
      end
      function opt = set.NTimes(opt,V)
         opt.NSearch = V;
      end

   end
   
   methods (Static, Hidden)
      
      function opt = loadobj(s)
         %LOADOBJ  Load filter for WCGOPT objects
         if isa(s,'wcgopt')
            opt = s;
         else
            % Remap old option object to new option object
            opt = wcgainOptions;
            for ct=1:numel(s.OptCell)
               sopt = s.OptCell{ct};
               OptionName = sopt.Name;
               switch OptionName
                  case 'FreqPtWise'
                     if sopt.Value
                        opt.MaxOverFrequency = 'off';
                     else
                        opt.MaxOverFrequency = 'on';
                     end
                  case 'ArrayDimPtWise'
                     if isempty(sopt.Value)
                        opt.MaxOverArray = 'on';
                     else
                        opt.MaxOverArray = 'off';
                     end
                  case 'MBadThreshold'
                     opt.RelMax = sopt.Value;
                  case 'ABadThreshold'
                     opt.AbsMax = sopt.Value;
                  case 'NTimes'
                     opt.NSearch = sopt.Value;
                  case {'MGoodThreshold','AGoodThreshold'}
                     % ignore
                  otherwise
                     opt.(OptionName) = sopt.Value;
               end
            end
         end
      end
      
   end
      
end
