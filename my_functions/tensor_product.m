function [matrix] = tensor_product(A,x,g)
[m, n] = size(A); matrix = zeros(m,n);
canonical_basis_matrix_l = eye(m);
canonical_basis_matrix_k = eye(n);
for i = 1:m
    for j=1:n
        matrix = matrix + (jacobian(A(i,j),x)*g)*canonical_basis_matrix_l(:,i)*canonical_basis_matrix_k(:,j)';
    end
end
end 
