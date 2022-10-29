function varargout =  mussv(matin,blk,varargin)
%MUSSV  Upper and lower bounds for the Structured Singular Value,
%   and upper bounds for the Generalized structured singular value.
%
%   [BOUNDS,MUINFO] = MUSSV(M,BLK) computes upper and lower bounds of the
%   structured singular value of M with respect to the block diagonal
%   matrix structure described by BLK.  BLK is NBLK-by-2, with the i'th
%   row describing the i'th uncertainty block.   If the i'th block is
%   of the form:
%     delta EYE(N), and delta restricted to be real, set BLK(i,:) = [-N 0];
%     delta EYE(N), and delta allowed to be complex, set BLK(i,:) = [N 0];
%     a R-by-C complex matrix, set BLK(i,:) = [R C].
%
%   The format used in the 3rd output argument from LFTDATA is also
%   acceptable for describing the block structure.
%
%   If M is an ND array, then the computation is performed pointwise along
%   the 3rd and higher array dimensions.  In that case, SIZE(BOUNDS,1)=1,
%   SIZE(BOUNDS,2)=2, and for K > 2, SIZE(M,K) = SIZE(BOUNDS,K).
%
%   MUINFO is a structure containing information that verifies (ie.,
%   proves) the bounds.  The information must be extracted using
%   MUSSVEXTRACT.
%
%   If M is an FRD, the computations are performed pointwise in frequency
%   (as well the array dimensions) and BOUNDS is dimensioned accordingly.
%
%   If M is an SS, the computations are performed using state-space
%   algorithms.  Frequencies are adaptively selected, and upper bounds are
%   guaranteed to hold over each interval between frequencies.  BOUNDS is
%   returned as an FRD.  M must be a single system, without array
%   dimensions.
%
%   [BOUNDS,MUINFO] = MUSSV(M,BLK,OPT) specifies options described by the
%   optional options argument OPT.  OPT is a character string containing
%   any combinations of the following characters:
%
%       'a'   Force upper bound to greatest accuracy, using LMI solver.
%             This is the default behavior when the number of decision
%             variables within the D/G scalings is less than 45
%       'f'   Force "fast" upper bound (typically not as tight as default)
%       'G'   Force upper bound to use gradient method.  This is the
%             default behavior when the number of decision variables within
%             the D/G scalings is greater than or equal to 45
%       'U'   Upper bound "only".  Lower bound is still computed and
%             returned, but uses a very fast and cheap algorithm.
%       'g6'  Use gain-based lower bound multiple times (in this case
%             10+6*10 times), larger number typically gives better lower
%             bound.  This is an alternative to the default lower bound
%             which uses a power iteration.  If all uncertainty blocks
%             described by BLK are real, then 'g' is the default.
%       'i'   ReInitialize lower bound computation at each new matrix
%             (only relevant if M is ND array or FRD)
%       'm7'  Randomly reinitialize lower-bound iteration multiple times (in
%             this case 7 times), a larger number typically uses
%             more computation but often yields a better lower bound.
%       'p'   Use power-iteration lower bound.  This is the default when at
%             least one of the uncertainty blocks in BLK is complex.
%       's'   Suppress progress information (silent)
%       'd'   Display warnings
%       'x'   Decrease iterations in lower-bound computation (faster but
%             not as tight as default).  Use 'U' for an even faster lower
%             bound.
%       'an'  Same as 'a', but without automatic prescaling
%       'o'   Run "old" algorithms, from version 3.1.1 and before.
%             Included to allow exact replication of earlier calculations.
%
%   [UBOUND,Q] = MUSSV(M,F,BLK) computes an upper bound of the
%   Generalized Structured Singular Value of the pair (M,F), with respect
%   to the block-diagonal uncertainty structure described by BLK.  The
%   upper bound guarantees that for all matrices DELTA with structure
%   defined by BLK, and NORM(DELTA)<=1/UBOUND, the matrix [I-DELTA*M;F] is
%   full column rank.  The matrix Q satisfies MUSSV(M+Q*F,BLK,'a')<=UBOUND.
%
%   [UBOUND,Q] = MUSSV(M,F,BLK,'s') suppresses the progress information.
%
%   See also LFTDATA, MUSSVEXTRACT, ROBSTAB, ROBGAIN, WCGAIN, WCDISKMARGIN.

%       'c'  default
%       'C'  same as 'g'
%       'Cn' same as 'gn'
%       'r'  same as i
%       'R7' same as m7
%       't'  (still accept) increase iterations in lower bound computation
%       'w'  ignore, as it is default
% ignore w, ignore c, C->a, r->i. R->m

% Mathematically, [I-Delta*M;C] is
%   guaranteed not to lose
%   column rank for all perturbations Delta consistent with
%   BLK, with norm < 1/UBND.  This is verified by a matrix Q,
%   which satisfies  MUSSV(M+QC,BLK,'C')<=UBND.  The optimal
%   choice of Q (which minimizes the upper bound) is solved
%   by reformulating the optimization into a semidefinite
%   program.


%   Authors:  Matthew P. Newlin, Peter M. Young and MUSYN INC
%   Copyright 1991-2019 The MathWorks, Inc.
ni = nargin;
if ni<=1
    error('Need at least two input arguments.');
end
nargoutchk(0,2)
[varargin{:}] = convertStringsToChars(varargin{:});

if ni>=3 && ( isnumeric(varargin{1}) && ~isempty(varargin{1}) )
    % BLK is actually E, and VARARGIN{1} is BLK
    E = blk;
    try
        blk = blkstruct2N2(varargin{1});
    catch ME
        throw(ME)
    end
    if isa(matin,'ss') 
        error('GeneralizedMU is not calculated for SS objects.');
    end
    [varargout{1:max(1,nargout)}] = genmussvcalc(matin,E,blk,varargin{2:end});
else
   % Convert Block Structure
   try
      blk = blkstruct2N2(blk);
   catch ME
      throw(ME)
   end
   if isa(matin,'ss')
      % SYNTAX:
      %   mussv(matin,blk)
      %   mussv(matin,blk,opt)
      %   mussv(matin,blk,opt,fixedBlkIdx)
      %   mussv(matin,blk,opt,fixedBlkIdx,fGrid)
      % XXX Should we also allow TF, ZPK, here?
      if ni>=3
         userMuOpt = varargin{1};
      else
         userMuOpt = '';
      end
      if ni>=4
         fixedBlkIdx = varargin{2};
      else
         fixedBlkIdx = zeros(0,1);
      end
      if ni>=5
         fGrid = varargin{3};   Focus = [fGrid(1) fGrid(end)];
      else
         fGrid = [];  Focus = [];
      end      
      if nmodels(matin)>1
         error(message('Robust:analysis:SSArray'))
      end
      [A,B,C,D,Ts] = ssdata(matin);
      TimeUnit = matin.TimeUnit;
      
      % This is the correct code for SSMU set-up, e.g. see
      %   @lftdataSS\wcgain
      % This handles fixed blocks, set-up of LMI vars, etc.
      [blkD,userMuOpt] = ssmussvSetUp(blk,userMuOpt,fixedBlkIdx);
      [~,UBcert,LBcert] = ssmussvCurve('',A,B,C,D,Ts,blkD,userMuOpt,Focus,fGrid);
      
      idx = 1:numel(LBcert);
      if isinf(LBcert(end).w)
         idx = 1:numel(LBcert)-1;
      end
      wReturn = [LBcert(idx).w];
      
      doublebnds = reshape([[UBcert.gUB]; [LBcert.LB]],[1 2 numel(UBcert)]);
      bnds = frd(doublebnds(:,:,idx),wReturn,Ts,'FrequencyUnit','rad/TimeUnit',...
         'TimeUnit',TimeUnit);
      varargout{1} = bnds;
      [rowd,rowg,rowp,sens] = rctutil.mkRowDGP(blk, UBcert, LBcert, matin);
      
      rowd = frd(rowd(:,:,idx),wReturn,Ts,'FrequencyUnit','rad/TimeUnit',...
         'TimeUnit',TimeUnit);
      rowg = frd(rowg(:,:,idx),wReturn,Ts,'FrequencyUnit','rad/TimeUnit',...
         'TimeUnit',TimeUnit);
      rowp = frd(rowp(:,:,idx),wReturn,Ts,'FrequencyUnit','rad/TimeUnit',...
         'TimeUnit',TimeUnit);
      sens = frd(sens(:,:,idx),wReturn,Ts,'FrequencyUnit','rad/TimeUnit',...
         'TimeUnit',TimeUnit);
      info = struct('bnds',bnds,'dvec',rowd,'pvec',rowp,'gvec',rowg,'sens',sens,'blk',blk);
      varargout{2} = info;
   else
      if ni>=3 && any(varargin{1}=='o')
         % Call old mussvcalc code if option 'o' is called
         [varargout{1:max(1,nargout)}] = mussvcalc(matin,blk,varargin{:});
      else
         % Otherwise call new mussvcalc code
         % SYNTAX:
         %   mussv(matin,blk)
         %   mussv(matin,blk,opt)
         %   mussv(matin,blk,opt,fixedBlkIdx)
         %   mussv(matin,blk,opt,fixedBlkIdx,SETUP)
         [varargout{1:max(1,nargout)}] = mussvcalc2(matin,blk,varargin{:});
      end
   end
end
