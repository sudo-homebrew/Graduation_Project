function z = iczero(dim)
% Z = ICZERO(DIM)
%
% Creates an identically-zero ICsignal for describing
% equations and constraints. 

% Copyright 2003-2004 The MathWorks, Inc.

if nargin==1
   if isa(dim,'double') & ndims(dim)==2 & all(size(dim)==[1 1]) & ...
      ceil(dim)==floor(dim) & dim>=0
      %z = icsignal(zeros(dim,1),{'_reserved_zerosignal'},1);
      %z = icsignal(zeros(dim,0),{'_reserved_zerosignal'},0);
      z = icsignal(zeros(dim,0),{},zeros(0,1));
   else
      error('DIM should be a nonnegative integer');
   end
elseif nargin==0
   disp('usage: z = iczero(dim)');
   return
end
