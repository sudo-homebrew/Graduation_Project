function [SM,DSV,INFO,ST] = fillNoBlocks(Need,M,B,muB)
% Populates ROBUSTSTAB outputs for systems with no blocks.

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.

DSV = struct;
SM = struct('LowerBound',Inf,'UpperBound',Inf,...
   'DestabilizingFrequency',NaN);
INFO = struct('Sensitivity',struct,...
   'Frequency',NaN,...
   'BadUncertainValues',DSV,...
   'MussvBnds',NaN(1,2),...
   'MussvInfo',[]);
if Need.Mussv4Info
   [~,INFO.MussvInfo] = mussv([],[0 0]);
   INFO.MussvBnds = [0 0];
end
ST = '';


if nargin==4 && Need.Mussv4Info
      if isfield(M.IC,'Frequency')
         Frequency = M.IC.Frequency;
      else
         Frequency = 0;
      end
      Mdata = fresp(M.IC,Frequency);
      szB = iosize(B);
      M11 = Mdata(1:szB(2),1:szB(1),:);  % ULFTDATA [Blocks;IO], M11, 3-D double-array   
      
      [~,tinfo] = mussv(M11,muB.muBlkNby2);
      Ts = M.IC.Ts;
      INFO.Frequency = Frequency;
      INFO.MussvInfo.bnds = ltipack.frddata(tinfo.bnds,Frequency,Ts);
      INFO.MussvInfo.dvec = ltipack.frddata(tinfo.dvec,Frequency,Ts);
      INFO.MussvInfo.pvec = ltipack.frddata(tinfo.pvec,Frequency,Ts);
      INFO.MussvInfo.gvec = ltipack.frddata(tinfo.gvec,Frequency,Ts);
      INFO.MussvInfo.sens = ltipack.frddata(tinfo.sens,Frequency,Ts);
      INFO.MussvBnds = ltipack.frddata(tinfo.bnds,Frequency,Ts);
end