function Mout = ufrd(varargin)
%UFRD  Converts input/output model to uncertain FRD model.
%
%   M = UFRD(M,FREQS,FREQUNITS) converts the input/output model M to an  
%   uncertain FRD model of class @ufrd. For non-FRD models, UFRD computes the  
%   frequency response of the LTI portion of the model at each frequency point 
%   in the vector FREQS. The frequencies FREQS are expressed in the units 
%   specified by FREQUNITS (see "help ufrd.FrequencyUnit" for a list of 
%   valid frequency units). The default is 'rad/TimeUnit' when FREQUNITS is 
%   omitted. All blocks in M that do not represent uncertainty are replaced 
%   by their current value and the resulting model contains only uncertain
%   blocks.
%
%   M = UFRD(M,FREQS,FREQUNITS,TIMEUNITS) further specifies the time units
%   when converting a static model M to UFRD.
%
%   See also UFRD, FRD, CHGFREQUNIT, INPUTOUTPUTMODEL.

%   Author(s): P. Gahinet
%   Copyright 1986-2010 The MathWorks, Inc.
ni = nargin;
try
   if ni==1 && isa(varargin{1},'FRDModel')
      % UFRD(SYS) where SYS is FRD/GENFRD/IDFRD
      M = varargin{1};
      funit = M.FrequencyUnit;
      Mout = copyMetaData(M,ufrd_(M));
   else
      % Note: Support ufrd(sys,w,'Units','Hz') for backward compatibility
      if ni==4 && startsWith('units',varargin{3},'IgnoreCase',true)
         varargin(:,3) = [];  ni = ni-1;
      end
      % Note: parseFRDInputs doesn't handle UFRD(M,FREQS,FREQUNITS,TIMEUNITS)
      [M,w,funit] = FRDModel.parseFRDInputs('ufrd',varargin(1:min(3,ni)));
      % Resolve time units
      if isa(M,'DynamicSystem')
         tunit = M.TimeUnit;
      elseif ni>3
         % UFRD(M,FREQS,FREQUNITS,TIMEUNITS)
         tunit = ltipack.matchKey(varargin{4},ltipack.getValidTimeUnits());
         if isempty(tunit)
            ctrlMsgUtils.error('Control:ltiobject:setTimeUnit')
         end
      else
         tunit = 'seconds';  % default for static models
      end
      w = funitconv(funit,'rad/TimeUnit',tunit) * w;  % now in rad/TimeUnit
      Mout = copyMetaData(M,ufrd_(M,w));
      Mout.TimeUnit = tunit;  % for static M
   end
   % Enforce correct frequency units
   Mout = chgFreqUnit(Mout,funit);
catch ME
   throw(ME)
end