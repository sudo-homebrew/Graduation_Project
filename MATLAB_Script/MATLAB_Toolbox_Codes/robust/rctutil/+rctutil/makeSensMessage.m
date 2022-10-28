function SensReport = makeSensMessage(SensStruct,Type)
% Creates CHAR array sensitivity report.
%   TYPE = 1 -> margin
%   TYPE = 2 -> worst-case gain
%   TYPE = 3 -> dksynperf
%   TYPE = 4 -> worst-case margins

%   Copyright 2010-2016 The MathWorks, Inc.
BlockNames = fieldnames(SensStruct);
ExamplePercentage = 25;
Rtext = [' -- ' getString(message('Robust:analysis:SensitivityIntro'))];
MsgID = sprintf('Robust:analysis:SensitivityAnal%d',Type);
for ct = 1:numel(BlockNames)
   name = BlockNames{ct};
   sens = SensStruct.(name);
   drop = (sens/100)*ExamplePercentage;
   t1 = getString(message('Robust:analysis:SensitivityIs',num2str(sens,'%.3g'),name));
   t2 = getString(message(MsgID,name,ExamplePercentage,num2str(drop,'%.3g')));
   Rtext = char(Rtext, ['      ' t1 ' ' t2]);
end
SensReport = Rtext;