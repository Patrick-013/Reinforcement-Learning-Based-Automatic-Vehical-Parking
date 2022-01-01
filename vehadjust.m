function vehState = vehadjust(vehState, speedIn, steerAngIn, dt, speedAcc, J_1_o, J_2_o, vehState_o, t)
    x_d = -4;
    y_d = 0;
    theta_d = 0;
    miu_1 = 0.5;
    miu_2 = 0.5;
if (t >= dt)
    %当每一个dt经历过之后，vehState完成更新，此时需要计算J_1和J_2来调整Φ.
    J_1 = 0.5 * ((vehState(1) - x_d)^2 + (vehState(2) - y_d)^2);
    J_2 = 0.5 * (vehState(3) - theta_d)^2;
    if (speedIn-vehState(5)>speedAcc*dt)
        vehState(4) = vehState(4) + miu_1 * (J_1 - J_1_o)/(vehState(4) - vehState_o) + miu_2 * (J_2 - J_2_o)/(vehState(4) - vehState_o);
    else
        vehState(4) = vehState(4) - miu_1 * (J_1 - J_1_o)/(vehState(4) - vehState_o) - miu_2 * (J_2 - J_2_o)/(vehState(4) - vehState_o);
    end

    vehState(4) = max(-pi/5, vehState(4));
    vehState(4) = min(pi/5, vehState(4));
    
    
    if (vehState(1) == x_d && vehState(2) == y_d)
        return;
    end
end

end
