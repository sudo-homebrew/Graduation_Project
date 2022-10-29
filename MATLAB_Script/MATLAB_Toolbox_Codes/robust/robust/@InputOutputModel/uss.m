function Mout = uss(M,varargin)
%USS  Converts input/output model to uncertain state-space model.
%       
%   M = USS(M) converts the input/output model M to an uncertain state-space
%   model of class @uss. All blocks in M that do not represent uncertainty 
%   are replaced by their current value and the resulting model contains 
%   only uncertain blocks.
%
%   See also USS, GENSS, SS, UNCERTAINBLOCK, INPUTOUTPUTMODEL.

%   Author(s): P. Gahinet, 5-1-96
%   Copyright 1986-2011 The MathWorks, Inc.
if nargin>1
   % Backward compatibility: Support uss(A,B,C,D,...) for UMAT's
   try
      Mout = ss(M,varargin{:});  return
   catch ME
      throw(ME)
   end
end
   
if isa(M,'FRDModel')
   ctrlMsgUtils.error('Robust:transformation:uss1',class(M))
end

try
   Mout = copyMetaData(M,uss_(M));
catch E
   throw(E)
end