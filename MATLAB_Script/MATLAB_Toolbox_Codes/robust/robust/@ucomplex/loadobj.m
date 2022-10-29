function blk = loadobj(s)
%LOADOBJ  Load filter for UCOMPLEX objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'ucomplex')
   blk = s;
   blk.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   
   if isfield(s,'atom')
      % Pre-MCOS
      Radius = s.Data(1);
      Nominal = s.atom.NominalValue;
      Name = s.atom.Name;
      blk = ucomplex(Name,Nominal,'Radius',Radius);
      if s.Data(2)==2
         blk.Mode = 'Percentage';
      end
      blk.AutoSimplify = s.atom.AutoSimplify;
   end
end
