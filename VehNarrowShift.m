% Vehicle Simulation 

load('Structure.mat');
vehL = max(body_x)-min(body_x);

dt = 0.03; 
steerVel = pi/2; % 90 degree/s 
speedAcc = 10; % 1m/s^2

park_x = [-4, 0, 3];
park_y = [-2.5, -1.5, 1.5, 5.5];

% vehState = [x; y; theta; steerAng; speed]
init_x = 1.2; init_y = 1.5; init_theta = pi/2;
% init_x = -4; init_y = 0; init_theta = 0;
vehState = [init_x; init_y; init_theta; 0; 0];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WRITE PRE-PROCESSING CODE HERE
no_collision = 1;
speedIn = 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X = [];
Y = [];
THETA = [];
PHI = [];
for t = 0:dt:5
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % WRITE CONTROL CODE HERE
    % Control Input:
    %   1) steerAngIn :  steering angle control (lateral control)
    %   2) speedIn : speed control (longitudinal control)

    % Lateral control
    steerAngIn = pi/2;    
    
    % Longitudinal control
    if (IsCollision(vehState, body_x, body_y, park_x, park_y))
        if (no_collision)
            speedIn = -speedIn;
            no_collision = 0;
        end
    else
        no_collision = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Vehicle steering angle adjustment
    vehState = vehadjust(vehState, speedIn, steerAngIn, dt, speedAcc, J_1_o, J_2_o, vehState_o, t);
    PHI = [PHI,vehState(4)];
    % The three parameters of the vehicle
    J_1_o = 0.5 * ((vehState(1) - x_d)^2 + (vehState(2) - y_d)^2);
    J_2_o = 0.5 * (vehState(3) - theta_d)^2;
    vehState_o = vehState(4);
    %steerAng_diff = -miu_1 * diff(J_1, vehState(4)) - miu_2 * diff(J_2, vehState(4)); 
    
    
    % Vehicle dynamics simulation
    vehState = VehDynamics(vehState, steerAngIn, speedIn, dt, vehL, steerVel, speedAcc);
    X = [X,vehState(1)];
    Y = [Y,vehState(2)];
    THETA = [THETA,vehState(3)];
    % Visualization
    figure(1); clf; hold on; 
    patch([park_x(1), park_x(2), park_x(2), park_x(1)], [park_y(1), park_y(1), park_y(2), park_y(2)], 'k');
    patch([park_x(1), park_x(2), park_x(2), park_x(1)], [park_y(3)+1.5, park_y(3)+1.5, park_y(4), park_y(4)], 'k');
    %patch([park_x(3), park_x(3)+0.5, park_x(3)+0.5, park_x(3)], [park_y(1), park_y(1), park_y(4), park_y(4)], 'k');
    patch([park_x(1), park_x(1)-0.5, park_x(1)-0.5, park_x(1)], [park_y(1), park_y(1), park_y(4), park_y(4)], 'k');
    VehDraw(vehState); hold off;
    axis equal; xlim([park_x(1)-1.5, park_x(3)+1.5]); ylim([park_y(1)-1, park_y(4)+1.5]);
    pause(0.002);
end

