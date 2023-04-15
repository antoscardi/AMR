function gamma_integration()

for  dadd
    % Creation of the matrix needed in the integration of the Gamma
    h_ax1(:,:,k) = h_a_1x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ay1(:,:,k) = h_a_1y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ax2(:,:,k) = h_a_2x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ay2(:,:,k) = h_a_2y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ax3(:,:,k) = h_a_3x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ay3(:,:,k) = h_a_3y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ax4(:,:,k) = h_a_4x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ay4(:,:,k) = h_a_4y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ax5(:,:,k) = h_a_5x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    h_ay5(:,:,k) = h_a_5y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
    
    g_ax1(:,:,k) = g_a_1x(q_history(:,k),timeVec(k));
    g_ay1(:,:,k) = g_a_1y(q_history(:,k),timeVec(k));
    g_ax2(:,:,k) = g_a_2x(q_history(:,k),timeVec(k));
    g_ay2(:,:,k) = g_a_2y(q_history(:,k),timeVec(k));
    g_ax3(:,:,k) = g_a_3x(q_history(:,k),timeVec(k));
    g_ay3(:,:,k) = g_a_3y(q_history(:,k),timeVec(k));
    g_ax4(:,:,k) = g_a_4x(q_history(:,k),timeVec(k));
    g_ay4(:,:,k) = g_a_4y(q_history(:,k),timeVec(k));
    g_ax5(:,:,k) = g_a_5x(q_history(:,k),timeVec(k));
    g_ay5(:,:,k) = g_a_5y(q_history(:,k),timeVec(k));
    end
    
    
    %% Gamma calculation from integration for every parameter x
    % In this part, we are going to calculate the integration of the gamma 
    % to obtain its value to be used in the final differential system
    gamma_k1 = zeros(6,1); gammax1_int= zeros(6,1,Nstep); gamma_k2 = zeros(6,1); gammax2_int= zeros(6,1,Nstep);
    gamma_k3 = zeros(6,1); gammax3_int= zeros(6,1,Nstep); gamma_k4 = zeros(6,1); gammax4_int= zeros(6,1,Nstep);
    gamma_k5 = zeros(6,1); gammax5_int= zeros(6,1,Nstep); gamma_k6 = zeros(6,1); gammax6_int= zeros(6,1,Nstep);
    gamma_k7 = zeros(6,1); gammax7_int= zeros(6,1,Nstep); gamma_k8 = zeros(6,1); gammax8_int= zeros(6,1,Nstep);
    gamma_k9 = zeros(6,1); gammax9_int= zeros(6,1,Nstep); gamma_k10 = zeros(6,1); gammax10_int= zeros(6,1,Nstep);
    gamma_k11 = zeros(6,1); gammax11_int= zeros(6,1,Nstep); gamma_k12 = zeros(6,1); gammax12_int= zeros(6,1,Nstep);
    gamma_k13 = zeros(6,1); gammax13_int= zeros(6,1,Nstep); gamma_k14 = zeros(6,1); gammax14_int= zeros(6,1,Nstep);
    gamma_k15 = zeros(6,1); gammax15_int= zeros(6,1,Nstep);
    for k=1:Nstep
        [~, gamma1] = ode45(@(t,gamma1) integralGamma(t,gamma1, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k1);
        gamma_k1= gamma1(end, :)'; gammax1_int(:,:,k) = gamma_k1;
        [~, gamma2] = ode45(@(t,gamma2) integralGamma(t,gamma2, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k2);
        gamma_k2= gamma2(end, :)'; gammax2_int(:,:,k) = gamma_k2;
        [~, gamma3] = ode45(@(t,gamma3) integralGamma(t,gamma3, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k3);
        gamma_k3= gamma3(end, :)'; gammax3_int(:,:,k) = gamma_k3;
        [~, gamma4] = ode45(@(t,gamma4) integralGamma(t,gamma4, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k4);
        gamma_k4= gamma4(end, :)'; gammax4_int(:,:,k) = gamma_k4;
        [~, gamma5] = ode45(@(t,gamma5) integralGamma(t,gamma5, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k5);
        gamma_k5= gamma5(end, :)'; gammax5_int(:,:,k) = gamma_k5;
        [~, gamma6] = ode45(@(t,gamma6) integralGamma(t,gamma6, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k6);
        gamma_k6= gamma6(end, :)'; gammax6_int(:,:,k) = gamma_k6;
        [~, gamma7] = ode45(@(t,gamma7) integralGamma(t,gamma7, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k7);
        gamma_k7= gamma7(end, :)'; gammax7_int(:,:,k) = gamma_k7;
        [~, gamma8] = ode45(@(t,gamma8) integralGamma(t,gamma8, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k8);
        gamma_k8= gamma8(end, :)'; gammax8_int(:,:,k) = gamma_k8;
        [~, gamma9] = ode45(@(t,gamma9) integralGamma(t,gamma9, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k9);
        gamma_k9= gamma9(end, :)'; gammax9_int(:,:,k) = gamma_k9;
        [~, gamma10] = ode45(@(t,gamma10) integralGamma(t,gamma10, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k10);
        gamma_k10= gamma10(end, :)'; gammax10_int(:,:,k) = gamma_k10;
        [~, gamma11] = ode45(@(t,gamma11) integralGamma(t,gamma11, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k11);
        gamma_k11= gamma11(end, :)'; gammax11_int(:,:,k) = gamma_k11;
        [~, gamma12] = ode45(@(t,gamma12) integralGamma(t,gamma12, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k12);
        gamma_k12= gamma12(end, :)'; gammax12_int(:,:,k) = gamma_k12;
        [~, gamma13] = ode45(@(t,gamma13) integralGamma(t,gamma13, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k13);
        gamma_k13= gamma13(end, :)'; gammax13_int(:,:,k) = gamma_k13;
        [~, gamma14] = ode45(@(t,gamma14) integralGamma(t,gamma14, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k14);
        gamma_k14= gamma14(end, :)'; gammax14_int(:,:,k) = gamma_k14;
        [~, gamma15] = ode45(@(t,gamma15) integralGamma(t,gamma15, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k15);
        gamma_k15= gamma15(end, :)'; gammax15_int(:,:,k) = gamma_k15;
    end
    
    %% Gamma calculation from integration for every parameter y
    % In this part, we are going to calculate the integration of the gamma 
    % to obtain its value to be used in the final differential system
    gamma_k1 = zeros(6,1); gammay1_int= zeros(6,1,Nstep); gamma_k2 = zeros(6,1); gammay2_int= zeros(6,1,Nstep);
    gamma_k3 = zeros(6,1); gammay3_int= zeros(6,1,Nstep); gamma_k4 = zeros(6,1); gammay4_int= zeros(6,1,Nstep);
    gamma_k5 = zeros(6,1); gammay5_int= zeros(6,1,Nstep); gamma_k6 = zeros(6,1); gammay6_int= zeros(6,1,Nstep);
    gamma_k7 = zeros(6,1); gammay7_int= zeros(6,1,Nstep); gamma_k8 = zeros(6,1); gammay8_int= zeros(6,1,Nstep);
    gamma_k9 = zeros(6,1); gammay9_int= zeros(6,1,Nstep); gamma_k10 = zeros(6,1); gammay10_int= zeros(6,1,Nstep);
    gamma_k11 = zeros(6,1); gammay11_int= zeros(6,1,Nstep); gamma_k12 = zeros(6,1); gammay12_int= zeros(6,1,Nstep);
    gamma_k13 = zeros(6,1); gammay13_int= zeros(6,1,Nstep); gamma_k14 = zeros(6,1); gammay14_int= zeros(6,1,Nstep);
    gamma_k15 = zeros(6,1); gammay15_int= zeros(6,1,Nstep);
    for k=1:Nstep
        [~, gamma1] = ode45(@(t,gamma1) integralGamma(t,gamma1, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k1);
        gamma_k1= gamma1(end, :)'; gammay1_int(:,:,k) = gamma_k1;
        [~, gamma2] = ode45(@(t,gamma2) integralGamma(t,gamma2, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k2);
        gamma_k2= gamma2(end, :)'; gammay2_int(:,:,k) = gamma_k2;
        [~, gamma3] = ode45(@(t,gamma3) integralGamma(t,gamma3, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k3);
        gamma_k3= gamma3(end, :)'; gammay3_int(:,:,k) = gamma_k3;
        [~, gamma4] = ode45(@(t,gamma4) integralGamma(t,gamma4, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k4);
        gamma_k4= gamma4(end, :)'; gammay4_int(:,:,k) = gamma_k4;
        [~, gamma5] = ode45(@(t,gamma5) integralGamma(t,gamma5, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k5);
        gamma_k5= gamma5(end, :)'; gammay5_int(:,:,k) = gamma_k5;
        [~, gamma6] = ode45(@(t,gamma6) integralGamma(t,gamma6, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k6);
        gamma_k6= gamma6(end, :)'; gammay6_int(:,:,k) = gamma_k6;
        [~, gamma7] = ode45(@(t,gamma7) integralGamma(t,gamma7, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k7);
        gamma_k7= gamma7(end, :)'; gammay7_int(:,:,k) = gamma_k7;
        [~, gamma8] = ode45(@(t,gamma8) integralGamma(t,gamma8, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k8);
        gamma_k8= gamma8(end, :)'; gammay8_int(:,:,k) = gamma_k8;
        [~, gamma9] = ode45(@(t,gamma9) integralGamma(t,gamma9, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k9);
        gamma_k9= gamma9(end, :)'; gammay9_int(:,:,k) = gamma_k9;
        [~, gamma10] = ode45(@(t,gamma10) integralGamma(t,gamma10, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k10);
        gamma_k10= gamma10(end, :)'; gammay10_int(:,:,k) = gamma_k10;
        [~, gamma11] = ode45(@(t,gamma11) integralGamma(t,gamma11, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k11);
        gamma_k11= gamma11(end, :)'; gammay11_int(:,:,k) = gamma_k11;
        [~, gamma12] = ode45(@(t,gamma12) integralGamma(t,gamma12, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k12);
        gamma_k12= gamma12(end, :)'; gammay12_int(:,:,k) = gamma_k12;
        [~, gamma13] = ode45(@(t,gamma13) integralGamma(t,gamma13, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k13);
        gamma_k13= gamma13(end, :)'; gammay13_int(:,:,k) = gamma_k13;
        [~, gamma14] = ode45(@(t,gamma14) integralGamma(t,gamma14, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k14);
        gamma_k14= gamma14(end, :)'; gammay14_int(:,:,k) = gamma_k14;
        [~, gamma15] = ode45(@(t,gamma15) integralGamma(t,gamma15, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k15);
        gamma_k15= gamma15(end, :)'; gammay15_int(:,:,k) = gamma_k15;
    end
end