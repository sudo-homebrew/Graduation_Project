function blk = loadobj(s)
%LOADOBJ  Load filter for ULTIDYN objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'ultidyn')
   blk = DynamicSystem.updateMetaData(s);
   blk.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   if isfield(s,'atom')
      % Pre-MCOS
      Name = s.atom.Name;
      Nominal = s.atom.NominalValue;
      blk = ultidyn(Name,size(Nominal));
      blk.Type = s.Type; 
      blk.Bound = s.Bound; % after Type to properly handle negative value
      blk.SampleStateDimension = s.SampleStateDim;
      blk.AutoSimplify = s.atom.AutoSimplify;
   elseif s.Version_<17
      % SampleStateDim -> SampleStateDimension
      blk = ultidyn(s.Name_,s.IOSize_);
      blk.Type = s.Type_; 
      blk.Bound = s.Bound_; % after Type to properly handle negative value
      blk.SampleStateDimension = s.SampleStateDim;
      blk.AutoSimplify = s.AutoSimplify;
      blk.Ts = s.Ts_;
      blk.UserData = s.UserData;
      if ~isempty(s.TimeUnit_)
         blk.TimeUnit = s.TimeUnit_;
      end
      blk.InputName = s.InputName_;
      blk.InputUnit = s.InputUnit_;
      blk.InputGroup = s.InputGroup_;
      blk.OutputName = s.OutputName_;
      blk.OutputUnit = s.OutputUnit_;
      blk.OutputGroup = s.OutputGroup_;
      blk.Notes = s.Notes_;
   end
end
