function display(M)
% Display method.

%   Copyright 1986-2011 The MathWorks, Inc.

% Variable name
VarName = inputname(1);
if isempty(VarName),
   VarName = 'ans';
end
fprintf('\n%s =\n\n',VarName)

s = size(M);
ArraySize = getArraySize(M);
switch prod(ArraySize)
   case 0
      % Empty array
      AS = sprintf('%dx',ArraySize);
      fprintf('  %s\n\n',getString(...
         message('Robust:umodel:umat10',AS(1:end-1),s(1),s(2))))
   case 1
      % Single matrix
      BD = getSummary(M.Data_.Blocks);
      nblk = numel(BD);
      if nblk==0
         fprintf('  %s\n',getString(message('Robust:umodel:umat4',s(1),s(2))))
      else
         fprintf('  %s\n',getString(message('Robust:umodel:umat5',s(1),s(2))))
         fprintf('  %s\n',getString(message('Robust:umodel:umat11')))
         for ct=1:nblk
            disp(['    ' BD{ct}]);
         end
      end
      fprintf('\n%s\n\n',getString(message('Robust:umodel:umodel1',VarName,VarName,VarName)))
   otherwise
      % UMAT array
      AS = sprintf('%dx',ArraySize);
      if isequal(M.Data_.Blocks)
         % All models have same block set
         BD = getSummary(M.Data_(1).Blocks);
         nblk = numel(BD);
         if nblk==0
            fprintf('  %s\n',getString(message('Robust:umodel:umat6',AS(1:end-1),s(1),s(2))))
         else
            fprintf('  %s\n',getString(message('Robust:umodel:umat7',AS(1:end-1),s(1),s(2))))
            for ct=1:nblk
               disp(['    ' BD{ct}]);
            end
         end
      else
         nblk = nblocks(M);
         if all(nblk==nblk(1))
            fprintf('  %s\n',getString(message('Robust:umodel:umat8',AS(1:end-1),s(1),s(2),nblk(1))))
         else
            fprintf('  %s\n',getString(message('Robust:umodel:umat9',AS(1:end-1),s(1),s(2),...
               min(nblk(:)),max(nblk(:)))))
         end
      end
      fprintf('\n%s\n\n',getString(message('Robust:umodel:umodel1',VarName,VarName,VarName)))
end
