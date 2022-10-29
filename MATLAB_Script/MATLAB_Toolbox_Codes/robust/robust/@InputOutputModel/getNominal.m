function M0 = getNominal(M)
%GETNOMINAL   Computes nominal value of uncertain model.
%
%   B = GETNOMINAL(A) replaces all uncertain elements in A by their
%   nominal value. All other Control Design blocks are left untouched.
%   If the resulting model is block-free, GETNOMINAL returns a double
%   array, SS model, or FRD model depending on the type of A.
%
%   See also USS, UFRD, UMAT, UREAL, UCOMPLEX, ULTIDYN, CONTROLDESIGNBLOCK.

%   Copyright 2003-2015 The MathWorks, Inc.
try
   M0 = getNominal_(M);
catch ME
   throw(ME)
end