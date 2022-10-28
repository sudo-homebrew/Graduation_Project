function display(sys)
% Display method.

%   Copyright 1986-2011 The MathWorks, Inc.

% Variable name
VarName = inputname(1);
if isempty(VarName),
   VarName = 'ans';
end
fprintf('\n%s =\n\n',VarName)

s = size(sys);
ArraySize = getArraySize(sys);
if isct(sys)
   Domain = 'c';
else
   Domain = 'd';
end
nf = nfreqs(sys);
switch prod(ArraySize)
   case 0
      % Empty array
      AS = sprintf('%dx',ArraySize);
      fprintf('  %s\n\n',getString(message('Robust:umodel:ufrd4',AS(1:end-1),s(1),s(2))))
   case 1
      % Single model
      BD = getSummary(sys.Data_.Blocks);
      nblk = numel(BD);
      if nblk==0
         MsgID = sprintf('Robust:umodel:ufrd1%s',Domain);
         fprintf('  %s\n',getString(message(MsgID,s(1),s(2),nf)))
      else
         MsgID = sprintf('Robust:umodel:ufrd2%s',Domain);
         fprintf('  %s\n',getString(message(MsgID,s(1),s(2),nf)))
         for ct=1:nblk
            disp(['    ' BD{ct}]);
         end
      end
      fprintf('\n%s\n\n',getString(message('Robust:umodel:umodel1',VarName,VarName,VarName)))
   otherwise
      % UFRD array
      AS = sprintf('%dx',ArraySize);
      fprintf('  %s\n',getString(message(sprintf('Robust:umodel:ufrd3%s',Domain),AS(1:end-1))))
      if isequal(sys.Data_.Blocks)
         % All models have same block set
         BD = getSummary(sys.Data_(1).Blocks);
         nblk = numel(BD);
         if nblk==0
            fprintf('  %s\n',getString(message('Robust:umodel:ufrd5',s(1),s(2),nf)))
         else
            fprintf('  %s\n',getString(message('Robust:umodel:ufrd6',s(1),s(2),nf)))
            for ct=1:nblk
               disp(['    ' BD{ct}]);
            end
         end
      else
         nblk = nblocks(sys);
         if all(nblk==nblk(1))
            % Same number of blocks
            fprintf('  %s\n',getString(message('Robust:umodel:ufrd7',s(1),s(2),nf,nblk(1))))
         else
            fprintf('  %s\n',getString(message('Robust:umodel:ufrd8',s(1),s(2),nf,min(nblk(:)),max(nblk(:)))))
         end
      end
      fprintf('\n%s\n\n',getString(message('Robust:umodel:umodel1',VarName,VarName,VarName)))
end
