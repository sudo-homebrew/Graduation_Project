function outFJE = feasibleJPDAEvents(validationMatrix, skipValidation, classToUse)
% This is an internal function and may be modified or removed in a future
% release. 

% Copyright 2020 The MathWorks, Inc.

% FJE = feasibleJPDAEvents(validationMatrix) returns all the feasible
% events from the validationMatrix using the depth-first search (DFS)
% algorithm.

%#codegen

if nargin < 2
    skipValidation = false;
end

% Process input
if ~skipValidation
    validateattributes(validationMatrix,{'numeric','logical'},...
        {'binary','nonempty','nonsparse'},'jpdaEvents','validationMatrix');
end

if nargin < 3
    if isfloat(validationMatrix)
        classToUse = class(validationMatrix);
    else
        classToUse = 'double'; % Safe data type for numerical accuracy when user hasn't provided any data type
    end
end

ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
[numMeas, numTrks] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(validationMatrix); 
numTrks = numTrks - ONE;

if ~skipValidation
    % There must be at least two columns
    coder.internal.assert(size(validationMatrix,2)>1,'fusion:jpdaEvents:expectedMoreColumns')

    % First column must be all ones
    coder.internal.assert(sum(validationMatrix(:,1))==numMeas, 'fusion:jpdaEvents:expectedAllOnes');

    % The rest of the columns must sum to at least one
    coder.internal.assert(all(sum(validationMatrix(:,2:end),1)>0), 'fusion:jpdaEvents:expectedNonzeroColumn');
end

% Declare output
% An upper bound on the number of feasible joint events matrices
nPotentials = fusion.internal.assignment.numPotentialFeasibleEvents(validationMatrix, numMeas, numTrks, classToUse);

omega=[true(numMeas,1) false(numMeas,numTrks)];
FJE = repmat(omega,[1,1,nPotentials]);

% Define useful variables
uz = zeros(1,'uint32'); % One uint32 zero
XjL = uz;     % Counter for Xj at current exploration level
X = zeros(min(numTrks,numMeas)+1,2,'uint32') ; % List of [j, Xj] pairs

Xjs = zeros(numMeas,numTrks+1,'uint32');
ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
for j=1:numMeas
    z=matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntFind(validationMatrix(j,:)~=0)-ONE;
    for k=0:numTrks
        ind = find(z>k);
        if isempty(ind)
            Xjs(j,k+1)=uz;
        else
            Xjs(j,k+1)=cast(z(ind(1)),'uint32');
        end
    end
end

% Visit root of hypthesis tree
L=zeros(1,'uint32'); % pointer for list X
j=ones(1,'uint32'); % measurement row

nFJE = ONE;

while j<=numMeas
    while L<min(numTrks,numMeas) && j<=numMeas
        Xj = Xjs(j,XjL+1);
        if Xj == 0
            XjL = uz;
            j=j+1; % go to next measurement
        else
            XjL = Xj;
            if ~any(X(:,2)==Xj) % get_next(Xj,Zj)
                % add pair to list of pairs
                L=L+1;
                X(L,1) = j;
                X(L,2) = Xj;
                %go to next measurement
                j=j+1;
                
                % Generate matrix
                nFJE = nFJE + ONE;
                for k=1:L
                    FJE(X(k,1),1,nFJE) = false;
                    FJE(X(k,1),X(k,2)+1,nFJE)=true;
                end
                
                % reset track counter to 0
                XjL = uz;
            end
        end
    end
    
    if L>=1
        j = X(L,1);
        XjL = X(L,2);
        X(L,1) = uz;
        X(L,2) = uz;
        L=L-1; % backtracking
    end
end
outFJE = FJE(:,:,1:nFJE);
end
