function M = uscale(M,fact)
%USCALE  Scales uncertainty in normalized units.
%
%   BLK = USCALE(BLK,FACT) scales the amount of uncertainty in BLK by the
%   factor FACT. This scaling is performed in normalized units. BLK is an
%   uncertain element (see UncertainBlock) and FACT is typically the
%   robustness margin returned by ROBSTAB or ROBGAIN, or the robust
%   performance returned by MUSYNPERF. USCALE returns an uncertain element
%   of the same type as BLK.
%
%   M = USCALE(M,FACT) scales all uncertain blocks in the model M by the
%   factor FACT. M can be a UMAT/USS/UFRD model or a generalized model. 
%   This leaves non-uncertain blocks unchanged.
%
%   Example: The uncertain element
%      blk = umargin('L',[0.5 3])
%   models gain uncertainty between 50% and 300% of the nominal loop gain.
%   If ROBSTAB says that the feedback loop is stable for up to 80% of this
%   uncertainty, the safe ranges of gain/phase variations are
%      uscale(blk,0.80)
%
%   Example: The uncertain element
%      blk = ureal('k',1,'Range',[0.5 3])
%   models an uncertain real parameter k taking values in the range [0.5 3].
%   If ROBGAIN says that the desired performance is maintained for up to 
%   130% of the specified uncertainty, the parameter k can vary in the range
%      uscale(blk,1.3)
%   without unacceptable performance degradation.
%
%   See also actual2normalized, normalized2actual, ROBSTAB, 
%   ROBGAIN, MUSYNPERF, UncertainBlock, USS, UFRD, GENSS, GENFRD.

%   Copyright 2020 The MathWorks, Inc.
narginchk(2,2)
if ~(isnumeric(fact) && isscalar(fact) && isreal(fact) && fact>0 && fact<Inf)
   error(message('Robust:umodel:uscale1'))
end
if isUncertain(M)
   try
      M = uscale_(M,fact);
   catch ME
      throw(ME)
   end
end