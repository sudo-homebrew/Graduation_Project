function [M,Delta,BlkStruct,NormDeltaCell] = lftdata(sys,varargin)
%LFTDATA   Decomposes an uncertain matrix or system.
%
%   Uncertain matrices/systems are modeled as a fixed matrix/system
%   M in feedback with a normalized, block-diagonal matrix DELTA of 
%   uncertain elements. LFTDATA extracts the M, DELTA components
%   from UMAT, USS, or UFRD objects.
%
%   [M,DELTA] = LFTDATA(A) takes an uncertain matrix/system A and 
%   extracts the M, DELTA components such that A = LFT(DELTA,M).
%   If A is a UMAT (respectively, USS and UFRD), M is a DOUBLE array
%   (respectively, SS model and FRD model). DELTA is a UMAT or USS
%   model describing the normalized uncertainty in A (WCNORM(DELTA) is 1).  
%
%   [M,DELTA] = LFTDATA(A,LIST) only pulls out the uncertainty elements
%   specified in LIST (a string vector or cell array of char vectors). 
%   In this case, DELTA only includes the selected uncertain elements, 
%   the other uncertain elements remain in M, and M retains the class of A.
%
%   [M,DELTA,BLKSTRUCT] = LFTDATA(A) also returns an N-by-1 structure array
%   BLKSTRUCT where BLKSTRUCT(i) describes the i-th normalized uncertain 
%   element.  This uncertainty description can be passed directly to the  
%   low-level mu-analysis function MUSSV.
%
%   [M,DELTA,BLKSTRUCT,NORMUNC] = LFTDATA(A) also returns the cell array 
%   NORMUNC of normalized uncertain elements. Each normalized element 
%   has the string 'Normalized' appended to its original name to avoid
%   confusion. Note that LFT(BLKDIAG(NORMUNC{:}),M) is equivalent to A.
%
%   See also LFT, MUSSV.

%   Copyright 2003-2010 The MathWorks, Inc.
if nargin>1
   [VALID,varargin{1}] = ltipack.isNameList(varargin{1});
   if ~VALID
      error(message('Robust:umodel:lftdata1'))
   end
end
if nmodels(sys)==0
   M = sys;  Delta = [];  BList = cell(0,1);
   BlkStruct = cell2struct({},cell(1,0));  
else
   try
      [M,Delta,BlkStruct,BList] = lftdata_(sys,varargin{:});
   catch E
      ltipack.throw(E,'command','lftdata',class(sys))
   end
   % Metadata for dynamic models
   if isa(M,'DynamicSystem')
      iodiff = iosize(M)-iosize(sys);
      M.TimeUnit = sys.TimeUnit;
      M = transferInputOutput_(M,sys,iodiff(1),iodiff(2));
   end
end

% Return DOUBLE/SS/FRD if there are no blocks left
if isBlockFree(M)
   M = getValue(M);
end
% Optional outputs
no = nargout;
if no>3
   NormDeltaCell = cellfun(@getNormalizedForm,BList,'UniformOutput',false);
end

