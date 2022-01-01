% 
% Judge whether the vehicle is in collisiion with the park walls
%
% vehState = [x; y; theta; steerAng; speed]

function is_collision = IsCollision(vehState, body_x, body_y, park_x, park_y)

    function seg_crossed = IsTwoSegsCrossed(segA, segB)
        % ptA1 + (ptA2-ptA1)*nmda = ptB1 + (ptB2-ptB1)*nmdb
        % [ptA2-ptA1  ptB1-ptB2]*[nmda; nmdb] = ptB1-ptA1
        segV = segB(1:2,1)-segA(1:2,1);
        segM = [segA(1:2,2)-segA(1:2,1), segB(1:2,1)-segB(1:2,2)];
        if (abs(det(segM))<0.000000001)
            seg_crossed = 0;
            return
        end
        nmd = segM\segV;  
        if (nmd(1)>=0 && nmd(1)<=1 && nmd(2)>=0 && nmd(2)<=1)
            seg_crossed = 1;
        else
            seg_crossed = 0;
        end
    end

    function plg_crossed = IsSegPolygonCrossed(seg, plg)
        plg_aug = [plg, plg(:,1)];
        for idx2 = 1:size(plg,2)
            if (IsTwoSegsCrossed(seg, plg_aug(:,[idx2,idx2+1])))
                plg_crossed = 1;
                return
            end
        end
        plg_crossed = 0;
    end

vehOrg = min(body_x);
vehCorners = [[max(body_x)-vehOrg;max(body_y)], [max(body_x)-vehOrg;min(body_y)], [0;min(body_y)], [0;max(body_y)]];
rotM1=[cos(vehState(3)), -sin(vehState(3)); sin(vehState(3)), cos(vehState(3))];
vehCorners = rotM1*vehCorners+vehState(1:2)*ones(1,4);

segAll = [[park_x(1);park_y(2)],[park_x(2);park_y(2)], ...
        [park_x(2);park_y(2)],[park_x(2);park_y(1)], ...
        [park_x(1);park_y(2)],[park_x(1);park_y(3) + 1.5], ...
        [park_x(1);park_y(3) + 1.5],[park_x(2);park_y(3) + 1.5], ...
        [park_x(2);park_y(3) + 1.5],[park_x(2);park_y(4)], ...
        [park_x(3);park_y(4)],[park_x(3);park_y(1)]];
    
for idx = 1:2:size(segAll,2)
    if (IsSegPolygonCrossed(segAll(:,idx:(idx+1)), vehCorners))
        is_collision = 1;
        %fprintf('Collision !!! \n');
        return
    end
end
is_collision = 0;

end












