function sensMatrixAt_tf = sensitivityIntegrationFunc(fq,fp,fu,hq,hxhi,gq,gxhi,t,q,u,a,p,xhi,delta,Nstep,wheelDistance,wheelRadius,timeVec)
    
    %COMMENTARE
    h = delta/1e+3;

    %% COMMENTARE
    syms sens [3 2], syms sensxhi [3 2]
    syms sensVec [6 1], syms sensxhiVec [6 1]
    
    sensVec(1) = sens(1,1); sensVec(2) = sens(1,2);sensVec(3) = sens(2,1);
    sensVec(4) = sens(2,2);sensVec(5) = sens(3,1);sensVec(6) = sens(3,2);

    sensxhiVec(1) = sensxhi(1,1); sensxhiVec(2) = sensxhi(1,2);sensxhiVec(3) = sensxhi(2,1);
    sensxhiVec(4) = sensxhi(2,2); sensxhiVec(5) = sensxhi(3,1);sensxhiVec(6) = sensxhi(3,2);

    % Load data
    idealControlData = load('data/IDEALcontrol','q_history','u_history','xhi_history');
    q_history = idealControlData.q_history; u_history = idealControlData.u_history; xhi_history = idealControlData.xhi_history;
    trajectoryCoefficients = load('data/coeff_a','a_x','a_y');
    a_x = trajectoryCoefficients.a_x; a_y = trajectoryCoefficients.a_y;
    aMatrix = [a_x,a_y];
   
    % Symbolic equations, Ode
    sensDerivative = fq*sens + fp + fu*(hq*sens + hxhi*sensxhi);
    sensxhiDerivative = gq*sens + gxhi*sensxhi;
   
    % Generate functions
    sensDerivativeFunc = matlabFunction(sensDerivative,'Vars',{t,q,u,a,p,xhi,sensVec,sensxhiVec});
    sensxhiDerivativeFunc = matlabFunction(sensxhiDerivative,'Vars',{t,q,u,a,p,xhi,sensVec,sensxhiVec});

    % Initialize the matrixes
    sens_history = zeros(3,2,Nstep); sensxhi_history = zeros(3,2,Nstep);

    % Get nominal parameters
    params = [wheelRadius; 
              wheelDistance];
    
    % Euler forward Integration
    for k = 2:Nstep

        currentSensitivityMatrix = sens_history(:,:,k-1);
        currentSensitivityXhiMatrix = sensxhi_history(:,:,k-1);

        % COMMENTARE
        NStepAtbreak1 = Nstep/3; NStepAtbreak2 = 2*Nstep/3;
        if k<=NStepAtbreak1
            currentTrajectoryCoefficients = aMatrix(1:4,:);
        end
        if k>NStepAtbreak1 && k<=NStepAtbreak2
            currentTrajectoryCoefficients = aMatrix(5:8,:);
   
        end 
        if k>NStepAtbreak2
            currentTrajectoryCoefficients = aMatrix(9:12,:);
        end 

        nextSensMatrix = currentSensitivityMatrix + h*sensDerivativeFunc(timeVec(k),q_history(:,k),u_history(:,k),currentTrajectoryCoefficients,params,xhi_history(:,k),...
                                                 reshape(transpose(currentSensitivityMatrix),6,1),reshape(transpose(currentSensitivityXhiMatrix),6,1));
        nextSensXhiMatrix = currentSensitivityXhiMatrix + h*sensxhiDerivativeFunc(timeVec(k),q_history(:,k),u_history(:,k),currentTrajectoryCoefficients,params,xhi_history(:,k),...
                                                 reshape(transpose(currentSensitivityMatrix),6,1),reshape(transpose(currentSensitivityXhiMatrix),6,1));

        sens_history(:,:,k) = nextSensMatrix;
        sensxhi_history(:,:,k) = nextSensXhiMatrix;
    
    end
    
    sensMatrixAt_tf = [sens_history(:,:,Nstep); sensxhi_history(:,:,Nstep)];

    %plot(timeVec[sens_history;sensxhi_history]);
end 