%Function used in the integration of the Sensitivity [3 2] (N.B. the
%sensitivty passed in the form of the vector)
function didt=sensitivity_dot(i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi)
    % Create matrixes from column vector
    %sens = [i(1) i(2);
    %        i(3) i(4);
    %       i(5) i(6)];
    sens  = reshape(i(1:6),2,[])';

    %sensxhi = [i(7) i(8);
    %          i(9) i(10);
    %          i(11) i(12)];
    sensxhi  = reshape(i(7:12),2,[])';
    % Sensitivity component of the ODEs  
    dsens = f_p + f_q*sens + f_u*(h_q*sens + h_xhi*sensxhi);
    % sensitivity_xhi component of the ODEs
    dsensxhi = g_q*sens + g_xhi*sensxhi;
    % Output
    didt = [dsens(1,1); dsens(1,2); dsens(2,1); dsens(2,2); dsens(3,1); dsens(3,2); dsensxhi(1,1); dsensxhi(1,2); dsensxhi(2,1); dsensxhi(2,2); dsensxhi(3,1); dsensxhi(3,2)];
end 