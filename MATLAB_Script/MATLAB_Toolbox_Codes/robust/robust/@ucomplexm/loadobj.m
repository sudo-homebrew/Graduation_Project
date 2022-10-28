function blk = loadobj(s)
%LOADOBJ  Load filter for UCOMPLEXM objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'ucomplexm')
   blk = s;
   blk.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   
   if isfield(s,'atom')
      % Pre-MCOS
      Nominal = s.atom.NominalValue;
      Name = s.atom.Name;
      blk = ucomplexm(Name,Nominal,'WL',s.WL,'WR',s.WR);
      blk.AutoSimplify = s.atom.AutoSimplify;
   end
end
