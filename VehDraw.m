%
% Vehicle Draw
%
% body_x, body_y
% front_x, front_y
% rear_x, rear_y
%
% save('Structure.mat', 'body_x', 'body_y', 'front_x', 'front_y', 'rear_x', 'rear_y');

function VehDraw(vehState, previewL, previewW, vehOrg)

load('Structure.mat');

if (nargin<2)
    vehOrg = min(body_x);  % Consistent with the rear-wheel motion bicycle model
    previewL = 0; 
    previewW = 0;
elseif (nargin < 4)
    vehOrg = min(body_x);  % Consistent with the rear-wheel motion bicycle model
end
body_x = body_x-vehOrg; front_x = front_x-vehOrg; rear_x = rear_x-vehOrg; 

rotM1=[cos(vehState(3)), -sin(vehState(3)); sin(vehState(3)), cos(vehState(3))];
rotM2=[cos(vehState(4)), -sin(vehState(4)); sin(vehState(4)), cos(vehState(4))];

V_body1(1,:)=body_x;
V_body1(2,:)=body_y;
V_a1=ones(size(V_body1(1,:)));
V_body1=rotM1*V_body1+vehState(1:2)*V_a1;


V_rear1(1,:)=rear_x;
V_rear1(2,:)=rear_y;
V_a1=ones(size(V_rear1(1,:)));
V_rear1=rotM1*V_rear1+vehState(1:2)*V_a1;


V_b1=mean(front_x);
V_front1(1,:)=front_x-V_b1;
V_front1(2,:)=front_y;
V_a1=ones(size(V_front1(1,:)));
V_front1=rotM2*V_front1;
V_front1(1,:)=V_front1(1,:)+V_b1;
V_front1=rotM1*V_front1+vehState(1:2)*V_a1;


patch(V_rear1(1,:), V_rear1(2,:), 'k');
patch(V_front1(1,:), V_front1(2,:), 'k');
patch(V_body1(1,:), V_body1(2,:), 'r');
axis equal;

V_P11=[previewL; previewW/2];
V_P21=[previewL; -previewW/2];
V_P11=rotM1*V_P11+vehState(1:2);
V_P21=rotM1*V_P21+vehState(1:2);
line([V_P11(1) V_P21(1)], [V_P11(2) V_P21(2)], 'Color', 'r');

V_P11=[0; previewW/2];
V_P21=[0; -previewW/2];
V_P11=rotM1*V_P11+vehState(1:2);
V_P21=rotM1*V_P21+vehState(1:2);
line([V_P11(1) V_P21(1)], [V_P11(2) V_P21(2)], 'Color', 'r');

end








