function h_a_5y_ = h_a_5y(in1,t,in3,in4)
%H_A_5Y
%    H_A_5Y_ = H_A_5Y(IN1,T,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    12-Apr-2023 23:01:02

b = in4(2,:);
r = in4(1,:);
theta = in1(3,:);
xhi_v = in3(1,:);
t2 = cos(theta);
t3 = sin(theta);
t6 = 1.0./r;
t4 = t2.^2;
t5 = t3.^2;
t7 = t4.*xhi_v;
t8 = t5.*xhi_v;
t9 = t7+t8;
t10 = 1.0./t9;
t11 = b.*t2.*t6.*t10.*2.8e+1;
h_a_5y_ = [t11;-t11];
end