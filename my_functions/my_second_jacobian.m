function out = my_second_jacobian(A,x)
    syms out [size(A)], [n,m] = size(A);
    for i=1:n
        for j = 1:m
        out(i,j) = jacobian(A(i,j),x);
        end
    end 