% 
% Dynamics: Rear-wheel motion bicycle model
%
%
% vehState = [x; y; theta; steerAng; speed]



function vehState = VehDynamics(vehState, steerAngIn, speedIn, dt, vehL, steerVel, speedAcc)

% steerAng : [-pi/5, -pi/5]
steerAngIn = max(-pi/5, steerAngIn);
steerAngIn = min(pi/5, steerAngIn);

% vehicle steering angle evolution
 

if (steerAngIn-vehState(4)>steerVel*dt)
    vehState(4) = vehState(4) + steerVel*dt;
    %vehState(4) = vehState(4) + miu_1 * diff(J_1, vehState(4)) + miu_2 * diff(J_2, vehState(4));
elseif (steerAngIn-vehState(4)<-steerVel*dt)
    vehState(4) = vehState(4) - steerVel*dt;
    %vehState(4) = vehState(4) - miu_1 * diff(J_1, vehState(4)) - miu_2 * diff(J_2, vehState(4));
else
    vehState(4) = steerAngIn;
    
    
end

% vehicle velocity evolution
if (speedIn-vehState(5)>speedAcc*dt)
    vehState(5) = vehState(5) + speedAcc*dt;
elseif (speedIn-vehState(5)<-speedAcc*dt)
    vehState(5) = vehState(5) - speedAcc*dt;
else
    vehState(5) = speedIn;
end

% vehicle pose (position and orientation) evolution
vehState(1) = vehState(1) + vehState(5)*dt*cos(vehState(3)+0.5*vehState(5)*dt*tan(vehState(4))/vehL);
vehState(2) = vehState(2) + vehState(5)*dt*sin(vehState(3)+0.5*vehState(5)*dt*tan(vehState(4))/vehL);
vehState(3) = vehState(3) + vehState(5)*dt*tan(vehState(4))/vehL;

end












