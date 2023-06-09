function func = dhxhi_a2x(b,k_p,k_v,r,t,theta,xhi_v)
%DHXHI_A2X
%    FUNC = DHXHI_A2X(B,K_P,K_V,R,T,THETA,XHI_V)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 15:20:24

t2 = cos(theta);
t3 = sin(theta);
t4 = t.*6.0;
t5 = t.^2;
t6 = t.^3;
t9 = 1.0./r;
t7 = t2.^2;
t8 = t3.^2;
t10 = k_p.*t6;
t13 = k_v.*t5.*3.0;
t11 = t7.*xhi_v;
t12 = t8.*xhi_v;
t14 = t7+t8;
t16 = t4+t10+t13;
t15 = t11+t12;
t17 = 1.0./t15.^2;
t18 = b.*t3.*t9.*t14.*t16.*t17;
func = reshape([t18,-t18,0.0,0.0,0.0,0.0],[2,3]);
end
