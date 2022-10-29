function assignment = hungarianAssignment(cost)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

%#codegen

% step 1: subtract row minima
cost = bsxfun(@minus, cost, min(cost, [], 2));

% step 2: make an initial assignment by "starring" zeros 
stars = makeInitialAssignment(cost);
% step 3: cover all columns containing starred zeros
colCover = any(stars);

while ~all(colCover)
    % uncover all rows and unprime all zeros
    rowCover = false(1, size(cost, 1));
    primes = false(size(stars));
    Z = ~cost; % mark locations of the zeros
    Z(:, colCover) = false;
    while 1
        shouldCreateNewZero = true;
        % step 4: Find a noncovered zero and prime it.
        [zi, zj] = findNonCoveredZero(Z);
        while zi > 0
            primes(zi, zj) = true;
            % find a starred zero in the column containing the primed zero
            starredRow = stars(zi, :);
            if any(starredRow)
                % if there is one, cover the its row and uncover
                % its column.
                rowCover(zi) = true;
                colCover(starredRow) = false;
                Z(zi, :) = false;
                Z(~rowCover, starredRow) = ~cost(~rowCover, starredRow);
                [zi, zj] = findNonCoveredZero(Z);
            else
                shouldCreateNewZero = false;
                % go to step 5
                break;
            end
        end
        
        if shouldCreateNewZero
            % step 6: create a new zero
            [cost, Z] = createNewZero(cost, rowCover, colCover);
        else
            break;
        end
    end
    
    % step 5: Construct a series of alternating primed and starred zeros.
    stars = alternatePrimesAndStars(stars, primes, zi, zj);
    % step 3: cover all columns containing starred zeros
    colCover = any(stars);
end
assignment = stars;

%-------------------------------------------------------------------------
function stars = makeInitialAssignment(cost)
rowCover = false(1, size(cost, 1));
colCover = false(1, size(cost, 2));
stars = false(size(cost));

[zr, zc] = find(cost == 0);
for i = 1:numel(zr)
    if ~rowCover(zr(i)) && ~colCover(zc(i))
        stars(zr(i), zc(i)) = true;
        rowCover(zr(i)) = true;
        colCover(zc(i)) = true;
    end
end

%-------------------------------------------------------------------------
function [zi, zj] = findNonCoveredZero(Z)
[i, j] = find(Z, 1);
if isempty(i)
    zi = -1;
    zj = -1;
else
    zi = i(1);
    zj = j(1);
end

%-------------------------------------------------------------------------
function [cost, Z] = createNewZero(cost, rowCover, colCover)
Z = false(size(cost));

% find a minimum uncovered value
uncovered = cost(~rowCover, ~colCover);
minVal = min(uncovered(:));

% add the minimum value to all intersections of covered rows and cols
cost(rowCover, colCover) = cost(rowCover, colCover) + minVal;
    
% subtract the minimum value from all uncovered entries creating at
% least one new zero
cost(~rowCover, ~colCover) = uncovered - minVal;
    
% mark locations of all uncovered zeros
Z(~rowCover, ~colCover) = ~cost(~rowCover, ~colCover);

%-------------------------------------------------------------------------
% Step 5.
% Construct a series of alternating primed and starred zeros.  
% Start with the primed uncovered zero Z0 at (zi, zj).  Find a starred zero 
% Z1 in the column of Z0. Star Z0, and unstar Z1. Find a primed zero Z2 in 
% the row of Z1. If the are no starred zeros in the column of Z2, stop.  
% Otherwise repeat with Z0 = Z2.
function stars = alternatePrimesAndStars(stars, primes, zi, zj)
nRows = size(stars, 1);
nCols = size(stars, 2);

% create a logical index of Z0
lzi = false(1, nRows);
lzj = false(1, nCols);
lzi(zi) = true;
lzj(zj) = true;

% find a starred zero Z1 in the column of Z0
rowInd = stars(1:nRows, lzj);

% star Z0
stars(lzi, lzj) = true;

while any(rowInd(:))
    % unstar Z1
    stars(rowInd, lzj) = false;
    
    % find a primed zero Z2 in Z1's row
    llzj = primes(rowInd, 1:nCols);
    lzj = llzj(1, :);
    lzi = rowInd;
    
    % find a starred zero in Z2's column
    rowInd = stars(1:nRows, lzj);
    
    % star Z2
    stars(lzi, lzj) = true;
end