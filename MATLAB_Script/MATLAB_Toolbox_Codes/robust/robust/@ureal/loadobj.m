function blk = loadobj(s)
%LOADOBJ  Load filter for UREAL objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'ureal')
   blk = s;
   blk.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   
   if isfield(s,'atom')
      % Pre-MCOS
      Nominal = s.atom.NominalValue;
      Name = s.atom.Name;
      blk = ureal(Name,Nominal,'Range',s.Data(1:2));
      switch s.Data(3)
         case 2
            blk.Mode = 'PlusMinus';
         case 3
            blk.Mode = 'Percentage';
      end
      blk.AutoSimplify = s.atom.AutoSimplify;
   end
end
