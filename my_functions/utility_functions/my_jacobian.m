function out = my_jacobian(A,x)
    %   It is a function that takes as input a matrix and a vector 
    %   and calculates the Jacobian of the elements of that matrix with respect to the vector.
    syms out [size(A)], [n,m] = size(A);
    for i=1:n
        for j = 1:m
        out(i,j) = jacobian(A(i,j),x);
        end
    end