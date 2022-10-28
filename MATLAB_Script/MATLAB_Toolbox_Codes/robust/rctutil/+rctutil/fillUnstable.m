function [SM,DSV,INFO,SensText] = fillUnstable(muB,M)
% Populates ROBUSTSTAB outputs for unstable systems.

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.

% DestabilizingFrequency=0 to be consistent w/ WCGAIN.
% This is the only way for the Margin to be 0.  In general, if the
% NominalValue is stable, the FRD calculation takes place.  The
% MUBNDS could be very large, but they will be finite, so that the
% margin will never be 0 in that case.
SM = struct('LowerBound',0,'UpperBound',0,...
   'DestabilizingFrequency',0);
INFO = struct('Sensitivity',struct,...
   'Frequency',0,...
   'BadUncertainValues',[],...
   'MussvBnds',NaN(1,2),...
   'MussvInfo',[]);

if isempty(muB.BlkInfo)
   % no blocks
   DSV = struct;
   [~,INFO.MussvInfo] = mussv([],[0 0]);
   INFO.MussvBnds = Inf(1,2);
else
   name = {muB.BlkInfo.BlockName}';
   atom = {muB.BlkInfo.BlockData}';
   for ii=1:length(atom)
      if isa(atom{ii},'DynamicSystem')
         atom{ii} = ltipack_ssdata(atom{ii});
      else
         atom{ii} = numeric_array(atom{ii});
      end
   end
   DSV = cell2struct(atom,name,1);
   [INFO.MussvBnds,INFO.MussvInfo] = mussv([],muB.muBlkNby2,'f',[],1);
   INFO.Sensitivity = cell2struct(num2cell(NaN(size(name))),name,1);
end
INFO.BadUncertainValues = DSV;
SensText = '';

Frequency = 0;
Ts = M.IC.Ts;
INFO.MussvBnds = ltipack.frddata(INFO.MussvBnds,Frequency,Ts);
