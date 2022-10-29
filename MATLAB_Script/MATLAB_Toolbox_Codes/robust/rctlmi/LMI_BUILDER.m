classdef (Sealed) LMI_BUILDER < handle
   % LMI data management during construction (singleton class).
   
   %   Copyright 1984-2016 The MathWorks, Inc.
   properties
      Header
      LMIs
      Variables
      Terms
      Data
   end
   
   methods (Access = private)
      function obj = LMI_BUILDER
      end
   end
   
   methods (Static)
      function singleObj = getInstance()
         persistent localObj
         if isempty(localObj)
            localObj = LMI_BUILDER;
         end
         singleObj = localObj;
      end
   end
end