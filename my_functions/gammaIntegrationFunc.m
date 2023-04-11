function gammaMatrixAt_tf = gammaIntegrationFunc(fq, fu, gq, gxhi, hq, hxhi, hai, gai,t,q,u,a,p,xhi,delta,Nstep,wheelDistance,wheelRadius,timeVec)
    
    %COMMENTARE
    h = delta/1e+3;

    %% COMMENTARE
    syms gamma [3 1], syms gammaxhi [3 1]

    syms gammaVec [3 1], syms gammaxhiVec [3 1]
    
    gammaVec(1) = gamma(1); gammaVec(2) = sens(2);gammaVec(3) = gamma(3);

    gammaxhiVec(1) = gammaxhi(1); gammaxhiVec(2) = gammaxhi(2); gammaxhiVec(3) = gammaxhi(3);

    % Load data
    idealControlData = load('data/IDEALcontrol','q_history','u_history','xhi_history');
    q_history = idealControlData.q_history; u_history = idealControlData.u_history; xhi_history = idealControlData.xhi_history;
    trajectoryCoefficients = load('data/coeff_a','a_x','a_y');
    a_x = trajectoryCoefficients.a_x; a_y = trajectoryCoefficients.a_y;
    aMatrix = [a_x,a_y];
   
    % Symbolic equations, Ode
    % f_q(:,:,k)*t_maius + f_u(:,:,k)*(h_q(:,:,k)*t_maius + h_xhi(:,:,k)*t_maius_xhi + h_a_i(:,:,k));
    gammaDerivative = fq*gamma + fu*(hq*gamma + hxhi*gammaxhi + hai);
    sensxhiDerivative = gq*gamma + gxhi*gammaxhi + gai;
   
    % Generate functions
    gammaDerivativeFunc = matlabFunction(gammaDerivative,'Vars',{t,q,u,a,p,xhi,gammaVec,gammaxhiVec});
    sensxhiDerivativeFunc = matlabFunction(gammaxhiDerivative,'Vars',{t,q,u,a,p,xhi,sensVec,gammaxhiVec});