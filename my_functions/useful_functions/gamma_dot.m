%Function used for the integration of Gamma 
function dgamma_tot=gamma_dot(gamma, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_a_i, g_a_i) 
     
    %Gamma component of the ODEs
    dgamma = f_q*gamma(1:3) + f_u*(h_q*gamma(1:3) + h_xhi*gamma(4:6) + h_a_i);
    %Gamma_xhi component of the ODEs
    dgammaxhi = g_q*gamma(1:3) + g_xhi*gamma(4:6) + g_a_i;
    %Output
    dgamma_tot = [dgamma; dgammaxhi];

end