function blk = loadobj(s)
%LOADOBJ  Load filter for UDYN objects

%   Copyright 1986-2011 The MathWorks, Inc.
if isa(s,'udyn')
   blk = s;
   blk.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   if isfield(s,'atom')
      % Pre-MCOS
      Name = s.atom.Name;
      Nominal = s.atom.NominalValue;
      blk = udyn(Name,size(Nominal));
      blk.AutoSimplify = s.atom.AutoSimplify;
   end
end
