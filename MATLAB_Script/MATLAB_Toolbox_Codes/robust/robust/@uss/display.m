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
switch prod(ArraySize)
   case 0
      % Empty array
      AS = sprintf('%dx',ArraySize);
      fprintf('  %s\n\n',getString(message('Robust:umodel:uss8',AS(1:end-1),s(1),s(2))))
   case 1
      % Single model
      BD = getSummary(sys.Data_.Blocks);
      nblk = numel(BD);
      nx = order(sys);
      if nblk==0
         MsgID = sprintf('Robust:umodel:uss4%s',Domain);
         fprintf('  %s\n',getString(message(MsgID,s(1),s(2),nx)))
      else
         MsgID = sprintf('Robust:umodel:uss5%s',Domain);
         fprintf('  %s\n',getString(message(MsgID,s(1),s(2),nx)))
         fprintf('  %s\n',getString(message('Robust:umodel:umodel2')))
         for ct=1:nblk
            disp(['    ' BD{ct}]);
         end
      end
      fprintf('\n%s\n\n',getString(message('Robust:umodel:umodel1',VarName,VarName,VarName)))
   otherwise
      % USS array
      AS = sprintf('%dx',ArraySize);
      nx = order(sys);  nx = nx(:);
      fprintf('  %s\n',getString(message(sprintf('Robust:umodel:uss7%s',Domain),AS(1:end-1))))
      if isequal(sys.Data_.Blocks)
         % All models have same block set
         BD = getSummary(sys.Data_(1).Blocks);
         nblk = numel(BD);
         if nblk==0
            if all(nx==nx(1))
               fprintf('  %s\n',getString(message('Robust:umodel:uss9',s(1),s(2),nx(1))))
            else
               fprintf('  %s\n',getString(message('Robust:umodel:uss10',s(1),s(2),min(nx),max(nx))))
            end
         else
            if all(nx==nx(1))
               fprintf('  %s\n',getString(message('Robust:umodel:uss11',s(1),s(2),nx(1))))
            else
               fprintf('  %s\n',getString(message('Robust:umodel:uss12',s(1),s(2),min(nx),max(nx))))
            end
            for ct=1:nblk
               disp(['    ' BD{ct}]);
            end
         end
      else
         nblk = nblocks(sys); nblk = nblk(:);
         if all(nblk==nblk(1))
            if all(nx==nx(1))
               fprintf('  %s\n',getString(message('Robust:umodel:uss13',s(1),s(2),nx(1),nblk(1))))
            else
               fprintf('  %s\n',getString(message('Robust:umodel:uss14',s(1),s(2),min(nx),max(nx),nblk(1))))
            end
         else
            if all(nx==nx(1))
               fprintf('  %s\n',getString(message('Robust:umodel:uss15',s(1),s(2),nx(1),min(nblk),max(nblk))))
            else
               fprintf('  %s\n',getString(message('Robust:umodel:uss16',s(1),s(2),min(nx),max(nx),min(nblk),max(nblk))))
            end
         end
      end
      fprintf('\n%s\n\n',getString(message('Robust:umodel:umodel1',VarName,VarName,VarName)))
end
