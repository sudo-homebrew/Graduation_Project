function B = isuncertain(A)
%ISUNCERTAIN True if object is uncertain (atom, UMAT, UFRD or USS)
%
%  B = ISUNCERTAIN(A) is 1 if A is uncertain object and 0 otherwise.
%  The uncertain objects are UMAT, UFRD, USS and all uncertain elements,
%  namely UCOMPLEX, UCOMPLEXM, ULTIDYN, UDYN and UREAL.

%   Copyright 2003-2011 The MathWorks, Inc.
B = isa(A,'InputOutputModel') && isUncertain(A);
