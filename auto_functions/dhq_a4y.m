function func = dhq_a4y(b,r,theta,xhi_v)
%DHQ_A4Y
%    FUNC = DHQ_A4Y(B,R,THETA,XHI_V)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    30-Mar-2023 00:24:57

t2 = cos(theta);
t3 = sin(theta);
t6 = 1.0./r;
t4 = t2.^2;
t5 = t3.^2;
t7 = t4.*xhi_v;
t8 = t5.*xhi_v;
t9 = t7+t8;
t10 = 1.0./t9;
t11 = b.*t3.*t6.*t10.*2.0;
func = reshape([0.0,0.0,0.0,0.0,-t11,t11],[2,3]);
end