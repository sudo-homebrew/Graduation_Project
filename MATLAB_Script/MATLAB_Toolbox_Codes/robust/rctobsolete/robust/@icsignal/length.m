%LENGTH Length of an ICSIGNAL object.
%
%N = LENGTH(A) returns the size of the longest dimension of A. If A 
%is an ICSGINAL vector, this is the same as its length. 
%
%See also: NDIMS, SIZE

function out = length(icsig)
% Copyright 2003-2004 The MathWorks, Inc.

out = size(icsig.System,1);