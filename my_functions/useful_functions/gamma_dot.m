%Function used for the integration of Gamma 
function dgamma_tot=gamma_dot(~, gamma, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_a_i, g_a_i, k) 
    t_maius = [gamma(1:3)]; t_maius_xhi = [gamma(4:6)]; 
    %Gamma component of the ODEs
    dgamma = f_q*t_maius + f_u*(h_q*t_maius + h_xhi*t_maius_xhi + h_a_i);
    %Gamma_xhi component of the ODEs
    dgammaxhi = g_q*t_maius + g_xhi*t_maius_xhi + g_a_i;
    %Output
    dgamma_tot = [dgamma(1); dgamma(2); dgamma(3); dgammaxhi(1); dgammaxhi(2); dgammaxhi(3)];
end