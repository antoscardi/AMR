%% Integration function Ode
%Function used in the integration of Sensitivity_ai  
function djdt = sensitivity_ai_dot(j, dfqqgamma, dfquuai, sens, f_q, dfpqgamma, dfpuuai,dfuqgamma,dfuuuai,h_q, h_xhi, sens_xhi, f_u, dhqqgamma,dhqxhigammaxhi,hq_ai,dhxhiqgamma, dhxhixhigammaxhi, h_xhi_ai, dgqqgamma,dgqxhigammaxhi,g_q_ai,dgxhiqgamma,dgxhixhigammaxhi,g_xhi_ai, g_q, g_xhi)
    % Create matrixes from column vector
    %sensai = [j(1) j(2);
    %       j(3) j(4);
    %       j(5) j(6)]
    sensai  = reshape(j(1:6),2,[])';

    %sensxhiai = [j(7) j(8);
    %           j(9) j(10);
    %           j(11) j(12)];
    sensxhiai = reshape(j(7:12),2,[])';

    %sens_mat = [sens(1) sens(2);
    %           sens(3) sens(4);
    %           sens(5) sens(6)];
    sens_mat  = reshape(sens(1:6),2,[])';

    %sensxhi_mat = [sens_xhi(1) sens_xhi(2);
    %               sens_xhi(3) sens_xhi(4);
    %              sens_xhi(5) sens_xhi(6)];
    sensxhi_mat = reshape(sens_xhi(1:6),2,[])';
    
    % Sensitivity component of the ODEs  
    dsensai = (dfqqgamma + dfquuai)*sens_mat + f_q*sensai + dfpqgamma + dfpuuai + (dfuqgamma + dfuuuai)*(h_q*sens_mat + h_xhi*sensxhi_mat) + f_u*((dhqqgamma + dhqxhigammaxhi + hq_ai)*sens_mat + (dhxhiqgamma + dhxhixhigammaxhi + h_xhi_ai)*sensxhi_mat + h_q*sensai + h_xhi*sensxhiai);
    % sensitivity_xhi component of the ODEs
    dsensxhiai = (dgqqgamma + dgqxhigammaxhi + g_q_ai)*sens_mat + (dgxhiqgamma + dgxhixhigammaxhi + g_xhi_ai)*sensxhi_mat + g_q*sensai + g_xhi*sensxhiai; 
    % Output
    djdt = [dsensai(1,1); dsensai(1,2); dsensai(2,1);dsensai(2,2);dsensai(3,1); dsensai(3,2);dsensxhiai(1,1); dsensxhiai(1,2); dsensxhiai(2,1);dsensxhiai(2,2);dsensxhiai(3,1);dsensxhiai(3,2)];
end 