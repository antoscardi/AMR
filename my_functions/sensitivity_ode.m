function didt=integralSens(~, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k)
    % Create matrixes from column vector
    sens = [i(1) i(2);
            i(3) i(4);
            i(5) i(6)];
    sensxhi = [i(7) i(8);
               i(9) i(10);
               i(11) i(12)];
    % Sensitivity component of the ODEs  
    dsens = f_p(:,:,k) + f_q(:,:,k)*sens + f_u(:,:,k)*(h_q(:,:,k)*sens + h_xhi(:,:,k)*sensxhi);
    % sensitivity_xhi component of the ODEs
    dsensxhi = g_q(:,:,k)*sens + g_xhi(:,:,k)*sensxhi;
    % Output
    didt = [dsens(1,1); dsens(1,2); dsens(2,1);dsens(2,2);dsens(3,1); dsens(3,2);dsensxhi(1,1); dsensxhi(1,2); dsensxhi(2,1);dsensxhi(2,2);dsensxhi(3,1);dsensxhi(3,2)];
end 