function [ Irotate, Prot] = rotate_qr( I, P1, P2, P3 )
%ROTATE_QR Summary of this function goes here
%   Detailed explanation goes here
width = size(I,2);
height = size(I,1);

P = (P3-P1)';
Pn = norm(P);
xaxis = [0 1]';


P * xaxis;
px = P*xaxis;
theta = acos((P*xaxis)/Pn);
thetaDegrees = rad2deg(theta);
%Check negative direction
if(P(1)<0)
    theta = -acos((P*xaxis)/Pn); 
    thetaDegrees = -thetaDegrees;
end

Irotate = imrotate(I, thetaDegrees, 'bicubic'); % rotate image by degrees

costheta = cos(theta);
sintheta = sin(theta);
rotmatrix = [costheta -sintheta; sintheta costheta]';
Ptot = [P1'; P2'; P3'];


xmiddle = size(Irotate, 2)/2;
ymiddle = size(Irotate, 1)/2;

Ptot(:,2) = Ptot(:,2) - width/2;
Ptot(:,1) = Ptot(:,1) - height/2;
Prot = Ptot * rotmatrix;

Prot(:,2) = Prot(:,2) + xmiddle;
Prot(:,1) = Prot(:,1) + ymiddle;

% figure;
% imshow(Irotate);
% hold on;
% plot(Prot(1,2), Prot(1,1), 'ro');
% plot(Prot(2,2), Prot(2,1), 'go');
% plot(Prot(3,2), Prot(3,1), 'bo');

P1 = Prot(1,:);
P2 = Prot(2,:);
P3 = Prot(3,:);

end

