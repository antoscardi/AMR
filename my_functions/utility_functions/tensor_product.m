function [matrix] = tensor_product(A,x,g)
    %% It's the tensor product function defined in the paper
    %   This function, defined in the paper, takes as input a second-order Jacobian matrix, and a vector. 
    %   It performs the product between the two, and to do this we also need the vector bases of the matrix passed as input.
[m, n] = size(A); matrix = zeros(m,n);
canonical_basis_matrix_l = eye(m);
canonical_basis_matrix_k = eye(n);
for i = 1:m
    for j=1:n
        matrix = matrix + (jacobian(A(i,j),x)*g)*canonical_basis_matrix_l(:,i)*canonical_basis_matrix_k(:,j)';
    end
end
end 