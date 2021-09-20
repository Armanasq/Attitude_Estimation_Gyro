%% Attitude Determination using Gyroscope
clc
close all
clear all
% Load Data
load('slow_v4.mat')

input = XS2(:,2:10);
out_q = XS2(:,11:14);
for i = 1:length(out_q)
    q0 = out_q(i,1);
    q1 = out_q(i,2);
    q2 = out_q(i,3);
    q3 = out_q(i,4);
    
    [phi, theta, psi] = quat_2_eul(q0,q1,q2,q3);
    phi_ref(i) = phi;
    theta_ref(i)=theta;
    psi_ref(i)=psi;
end


    dt      =   1/100; % Sassari Dataset Sampling Rate = 100 Hz
    t       =   0:dt:(length(input) - 1)*dt;
	gyro_x  =   input(:,4);
    gyro_y  =   input(:,5);
    gyro_z  =   input(:,6);
	phi_gyro    =   zeros(1,length(t));
    theta_gyro  =   zeros(1,length(t));
    
     for i = 2:length(input)
        p   =   gyro_x(i);
        q   =   gyro_y(i);
        r   =   gyro_z(i);

        phi     =	phi_gyro(i-1);
        theta   =   theta_gyro(i-1);

        phi_gyro(i)   =     phi     +   dt*(p + (sin(phi)*tan(theta)*q) + (cos(phi)*tan(theta)*r));
        theta_gyro(i) =     theta   +   dt*((cos(phi)*q) - (sin(phi)*r));
     end
     
    figure(1)
    plot(t,phi_gyro,t,phi_ref)
    legend('Gyro','Ref')
    title('Phi')
    RMSE_phi_gyro_ = sqrt(mean( (rad2deg(phi_gyro(:,1))-rad2deg(phi_ref(1,:))').^2 ));
    
    figure(2)
    plot(t,theta_gyro,t,theta_ref)
    legend('Gyro','Ref')
    title('Theta')
    RMSE_theta_gyro_ = sqrt(mean( (rad2deg(theta_gyro(:,1))-rad2deg(theta_ref(1,:))').^2 ));