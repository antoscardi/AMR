function [coeffMatrix, M] = coeff_generation(tsim, dx, dy)                     
% The coefficients of the polynomial are found as solution of the system: Ma = d.

%% This is the constraints matrix
M = [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
    % this correspond to define the polinomial to the initial position -> p(0) = a_01;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
     % this correspond to define the polinomial to the initial velocity -> v(0) = a_11;
     (tsim/3)^4 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1 0 0 0 0 -1 0 0 0 0 0;
     % this correspond to the constraint for which the point on the end of the first polynomial is equal to the start of the second polynomial -> p1(t/3) = p2(0);
     4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0 0 0 0 -1 0 0 0 0 0 0;
     % this correspond to the constraint for which the velocity on the end of the first polynomial is equal to the start of the second polynomial -> v1(t/3) = v2(0);
     12*(tsim/3)^2 6*(tsim/3)  2 0 0 0 0 -2 0 0 0 0 0 0 0;
     % this correspond to the constraint for which the acceleration on the end of the first polynomial is equal to the start of the second polynomial -> a1(t/3) = a2(0), so the continuity of velocity;
     0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
     % This is the definition of the first break point
     0 0 0 0 0 (tsim/3)^4 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1 0 0 0 0 -1;
     % this correspond to the constraint for which the point on the end of the first polynomial is equal to the start of the second polynomial -> p1(t/3) = p2(0);
     0 0 0 0 0 4*(tsim/3)^3 3*(tsim/3)^2 2*tsim/3 1 0 0 0 0 -1 0;
     % this correspond to the constraint for which the velocity on the end of the second polynomial is equal to the start of the third polynomial -> v2(t/3) = v3(0);
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;
     % This is the definition of the second break point
     %0 0 0 0 0 4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0 0 0 0 0 0;
     0 0 0 0 0 12*(tsim/3)^2 6*(tsim/3) 2 0 0 0 0 -2 0 0;
     % % this correspond to the constraint for which the acceleration on the end of the first polynomial is equal to the start of the second polynomial -> a2(t/3) = a3(0), so the continuity of velocity;
     0 0 0 0 0 0 0 0 0 0 (tsim/3)^4 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1;
     % this correspond to define the polinomial to the final position -> p(t/3) = a_03;
     0 0 0 0 0 0 0 0 0 0 4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0];
     % this correspond to define the polinomial to the final velocity -> v(t/3) = a_13;
     
ax = pinv(M) * dx;
ay = pinv(M) * dy;

coeffMatrix = [ax,ay];

end 