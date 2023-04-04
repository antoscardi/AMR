function didt=integralSens(~, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k, j)
    % Create matrixes from column vector
    sens = [i(1);
            i(2);
            i(3)];
    sensxhi = [i(4);
               i(5);
               i(6)];
    % Sensitivity component of the ODEs  
    dsens = f_p(:,:,k,j) + f_q(:,:,k,j)*sens + f_u(:,:,k,j)*(h_q(:,:,k,j)*sens + h_xhi(:,:,k,j)*sensxhi);
    % sensitivity_xhi component of the ODEs
    dsensxhi = g_q(:,:,k,j)*sens + g_xhi(:,:,k,j)*sensxhi;
    % Output
    didt = [dsens(1,1); dsens(2,1); dsens(3,1); dsensxhi(1,1); dsensxhi(2,1); dsensxhi(3,1)];
end 