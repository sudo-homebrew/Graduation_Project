function [ L ] = specialCholesky( inertiaMatrix, lambda )
%This class is for internal use only. It may be removed in the future.

%SPECIALCHOLESKY Sparsity-preserving Cholesky decomposition for rigid body 
%    tree inertia matrix ONLY. The first input argument, inertiaMatrix, is 
%    a valid vNum x vNum inertia matrix of a rigid body tree, where vNum is
%    the number of velocity variables. The second input, lambda, is a vNum x 1
%    vector. lambda(i) corrsponds to the index of the last non-zero of row
%    i below the main diagonal. The output, L, is a lower trangular matrix
%    that preserves the sparsity of inertiaMatrix, and L'*L = inertiaMatrix.

%#codegen
    
%   Copyright 2016 The MathWorks, Inc.

n = length(inertiaMatrix);
H = inertiaMatrix;

for i = n:-1:1
    H(i,i) = sqrt(H(i,i));
    k = lambda(i); % lambda(i) == 0 means no non-zeros found down the row
    while k > 0
        H(i,k) = H(i,k)/H(i,i);
        k = lambda(k);
    end
    
    k = lambda(i);
    while k > 0
        j = k;
        while j>0
            H(k,j) = H(k,j) - H(i,k)*H(i,j);
            j = lambda(j);
        end
        k = lambda(k);
    end
    
end

L = tril(H, 0);


end