function cliques = findMaxCliques(A)
% This function is for internal use and may be removed or modified
%
%findMaxCliques     Find maximal cliques using the Bron-Kerbosch algorithm
%   cliques = findMaxCliques(A) returns a Boolean matrix of maximal
%   cliques, in a graph described by its boolean adjacency matrix, A.
%
%   Inputs:
%       A       - an N-by-N Boolean adjacency matrix for an undirected 
%                 graph. The elements on the diagonal must be 0.
%
%   Outputs:
%       cliques - an N-by-P logical matrix. P is the number of maximal
%                 cliques in A. The value cliques(i,j) indicates whether
%                 vertex i is part of clique j.

%   Copyright 2018 MathWorks, Inc.

%   References: 
%       [1] Bron, Coen and Kerbosch, Joep, "Algorithm 457: finding all 
%           cliques of an undirected graph", Communications of the ACM,
%           vol. 16, no. 9, pp: 575-577, September 1973.
%       [2] Cazals, F. and Karande, C., "A note on the problem of reporting 
%           maximal cliques", Theoretical Computer Science (Elsevier), vol.
%           407, no. 1-3, pp: 564-568, November 2008.

%#codegen

% Input validation is done in the calling function

% Definitions and memory allocation
numVertices = size(A,1);        % number of vertices
cliques = false(numVertices,0); % storage for maximal cliques
currentClique = [];             % currently growing clique
potVertices = 1:numVertices;    % potential vertices connected to all vertices in currentClique
usedVertices = zeros(1,0);      % vertices already used

% Force codegen to perform run-time recursion
coder.varsize('currentClique')
coder.varsize('potVertices')
coder.varsize('usedVertices')

% Run the recursive Bron-Kerbosch Algorithm
cliques = bronKerbosch(A,cliques,numVertices,currentClique,potVertices,usedVertices);
end

% The Bron-Kerbosch algorithm
function cliques = bronKerbosch(A,cliques,numVertices,currentClique,potVertices,usedVertices)
% The Bron-Kerbosch algorithm
%   Inputs:
%       A               - a binary adjacency matrix
%       cliques         - the list of maximal cliques found in previous recursion
%       numVertices     - number of vertices, i.e., rows in A
%       currentClique   - set of vertices considered as part of the currently growing the clique
%       potVertices     - set of vertices considered as potential to the currently growing clique
%       usedVertices    - set of vertices not in consideration
%
%   Output:
%       cliques         - the list of cliques found in the current recursion

    coder.inline('never')
    if (isempty(potVertices) && isempty(usedVertices))
        % Add currentClique to the list of maximal clique
        clique = false(numVertices,1);
        clique(currentClique) = true;
        cliques = [cliques clique];
    else
        % choose pivot
        pivots = union(potVertices,usedVertices);
        binP = zeros(1,numVertices);
        binP(potVertices) = 1; % binP contains ones at indices equal to the values in potVertices          
        % rows of A(ppivots,:) contain ones at the neighbors of ppivots
        pcounts = A(pivots,:)*binP.';  % cardinalities of the sets of neighbors of each ppivots intersected with P
        [~,ind] = max(pcounts);
        u_p = pivots(ind);             % select one of the ppivots with the largest count
        
        d = intersect(find(~A(u_p,:)),potVertices);   % all potential vertices that are not neighbors of the pivot
        i = 0;
        while i < numel(d)
            i = i + 1;
            u = d(i);
            potVertices = setxor(potVertices,u);
            newCurrentClique = [currentClique u];
            Nu = find(A(u,:));
            newPotVertices = intersect(sort(potVertices),Nu);
            newUsedVertices = intersect(sort(usedVertices),Nu);
            cliques = bronKerbosch(A, cliques, numVertices,newCurrentClique, newPotVertices, newUsedVertices);
            usedVertices = [usedVertices u];
        end
    end
end