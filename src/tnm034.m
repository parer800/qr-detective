%%%%%%%%%%%%%%%%%%%%%%%%%% 
function strout = tnm034(im) 
% 
% Im: Input image of captured sheet music. Im should be in
% double format, normalized to the interval [0,1] 
% strout: The resulting character string of the coded 
% The string must have exactly the number of characters that 
% corresponds to the information in the code. 
%
% Your program code.
%

I = im;

%Get dimensions, to check if 
[x,y,z] = size(I);
if z == 3
    BW = rgb2gray(I);
    G = fspecial('disk',15);
    %Ig = imfilter(BW,G,'same');
    
    %BW = edge(BW, 'canny');
else
    BW = I;
    %BW = edge(BW, 'canny');
end
%adaptive threshold image
atImage = adaptivethres(double(BW));
atImage_unfiltered = atImage;
% imshow(atImage);
atImage = medfilt2(atImage,[3 3]);
% figure;
% imshow(atImage);


% Display the original image.
% subplot(2,1,1);
% figure;
% 
% title(im);

%LOOP THROUGH AND SAVE on every color switch, position (x,y), pixelcolor, counter. 
width = size(atImage,2);
height = size(atImage,1);


   
[result_image_horizontal, qr_locations_horizontal] = locate_qr(atImage, false);
[result_image_vertical, qr_locations_vertical] = locate_qr(atImage, true);
% 
%  figure;
%  imshow(atImage);
% 
combined = result_image_horizontal + result_image_vertical;
%  figure;
%  imshow(combined);

L = medfilt2(combined,[2 2]);
 %figure, imshow(L)




%================================================================
%                   FIND FIDUCIAL MARKS (CENTER POINTS)
%================================================================
[P1, P2, P3] = find_FIP(L);

% figure;
% imshow(I);
% hold on;
% plot(P1(2), P1(1), 'ro');
% plot(P2(2), P2(1), 'go');
% plot(P3(2), P3(1), 'bo');






%================================================================
%                   ROTATE BY FIP POINTS [P1 P2 P3]
%================================================================

[Irotate, Prot] = rotate_qr(BW, P1, P2, P3);

% figure;
% imshow(Irotate);
% hold on;
% plot(Prot(1,2), Prot(1,1), 'ro');
% plot(Prot(2,2), Prot(2,1), 'go');
% plot(Prot(3,2), Prot(3,1), 'bo');

%Just to debug finding FIP center point.
debug_find_fip = -1;
if (debug_find_fip ~= 1)
%================================================================
%                   FIND CORNER POINTS
%================================================================

Pcorner1 = locate_corners(Irotate, Prot(1,:), [-1, -1]);
%plot(Pcorner1(2), Pcorner1(1), 'r+');

Pcorner2 = locate_corners(Irotate, Prot(2,:), [-1, 1]);
%plot(Pcorner2(2), Pcorner2(1), 'g+');

Pcorner3 = locate_corners(Irotate, Prot(3,:), [1, -1]);
%plot(Pcorner3(2), Pcorner3(1), 'b+');



%================================================================
%                   FIND AP MARK
%================================================================
xLength = Pcorner3(2) - Pcorner1(2);
yLength = Pcorner2(1) - Pcorner1(1);
APTemplate = getTemplateOfAP(yLength, xLength);
centerX = ceil((Pcorner3(2) - Pcorner2(2))/2);
centerY = ceil((Pcorner2(1) - Pcorner3(1))/2);
w = size(Irotate,2);
h = size(Irotate,1);

%AP = normxcorr2(APTemplate, Icrop(ceil(centerY):size(Icrop,1), ceil(centerX):size(Icrop, 2)));
%AP = normxcorr2(APTemplate, Irotate(centerY:floor(centerY + yLength), centerX:floor(centerX + xLength)));
%figure;
%imshow(Irotate(centerY:h, centerX:w));
AP = normxcorr2(APTemplate, Irotate(centerY:h, centerX:w));
[i j] = find(AP==max(max(AP)));
i = i + centerY;
j = j + centerX;
APpos = [i j]';
%APpos = APpos + Pcorner1 - [size(APTemplate,1)/2; size(APTemplate,1)/2]
offsetAPX = size(APTemplate,2)/2;
offsetAPY = size(APTemplate,1)/2;
APpos = APpos - [offsetAPY; offsetAPX];

%================================================================
%                   PERSPECTIVE DISTORTION
%================================================================


%Calculate fixed points
%Start with P1_ _ _ _ P3
%           |
%           | 
%           |     
%           P2        
%
PPBX = (Pcorner3(2) - Pcorner1(2))/41; % Pixel per block in x
PPBY = (Pcorner2(1) - Pcorner1(1))/41; % Pixel per block in y

% BY FIP MARK
% P1f = round(Prot(1,:))';
% P2f = [P1f(1) + PPBY*(41-7); P1f(2)];
% P3f = [P1f(1); P1f(2) + PPBX*(41-7)];
% PAPf = round([P2f(1)-7*PPBY + PPBY/2;
%         P3f(2)-7*PPBX + PPBX/2
%         ]);

    
%BY CORNER POINTS     
P1f = Pcorner1;
P2f = [P1f(1) + PPBY*41; P1f(2)];
P3f = [P1f(1); P1f(2) + PPBX*41];
% PAPf = round([P2f(1)-7*PPBY + PPBY/2;
%         P3f(2)-7*PPBX + PPBX/2
%         ]);
   
%The alignment patter center point if the new center point of the fiducial
%marks should be in the fixed corner points. This will expand the QR-code
%image.
PAPf = round( [ P2f(1)-3.5*PPBY; P3f(2)-3.5*PPBX ]);
 
%FLIP THE VALUES, REQUIRED...
P1f = flip(P1f);
P2f = flip(P2f);
P3f = flip(P3f);
PAPf = flip(PAPf);


fixedPoints = round([P1f P2f P3f PAPf]');
movingPoints = [round(Prot(1,:))' round(Prot(2,:))' round(Prot(3,:))' APpos];
movingPoints = flip(movingPoints)';

cornerPoints = flip([Pcorner1, Pcorner2, Pcorner3])';

tform = cp2tform(movingPoints, fixedPoints, 'projective');
predicted_corner_points = tformfwd(tform, cornerPoints);
correctCornerPoints = flip(predicted_corner_points')';
[row, col] = size(Irotate);
Iwarp = imtransform(Irotate, tform, 'XData', [1 col], 'YData', [1 row]);

P1f = flip(P1f);
P2f = flip(P2f);
P3f = flip(P3f);
PAPf = flip(PAPf);
movingPoints = flip(movingPoints');


% figure;
% imshow(Irotate);
% hold on;
% 
% plot(APpos(2),APpos(1),'g+');
% plot(PAPf(2),PAPf(1),'ro');
% 
% plot(Prot(1,2), Prot(1,1), 'g+');
% plot(Prot(2,2), Prot(2,1), 'g+');
% plot(Prot(3,2), Prot(3,1), 'g+');
% plot(P1f(2),P1f(1),'ro');
% plot(P2f(2),P2f(1),'ro');
% plot(P3f(2),P3f(1),'ro');
% 
% 
% 
% figure;
% imshow(Iwarp);
% hold on;
% plot(APpos(2),APpos(1),'g+');
% plot(PAPf(2),PAPf(1),'ro');
% 
% plot(Prot(1,2), Prot(1,1), 'g+');
% plot(Prot(2,2), Prot(2,1), 'g+');
% plot(Prot(3,2), Prot(3,1), 'g+');
% 
% plot(P1f(2),P1f(1),'ro');
% plot(P2f(2),P2f(1),'ro');
% plot(P3f(2),P3f(1),'ro');



%================================================================
%                   FIND CORNER POINTS AGAIN
%================================================================
%If image had perspective distortion, the corner points are not 
% optimal to crop with! And the dimension of the QR-code have changed so
% recalculate the corner points.
% %figure;
% % imshow(Iwarp);
% % hold on;


Pcorner1 = locate_corners(Iwarp, P1f', [-1, -1]);
%plot(Pcorner1(2), Pcorner1(1), 'y+');

Pcorner2 = locate_corners(Iwarp, P2f', [-1, 1]);
%plot(Pcorner2(2), Pcorner2(1), 'y+');

Pcorner3 = locate_corners(Iwarp, P3f', [1, -1]);
%plot(Pcorner3(2), Pcorner3(1), 'y+');



%================================================================
%                   CROP IMAGE BY CORNER POINTS
%================================================================
xlength = Pcorner3(2) - Pcorner1(2);
ylength = Pcorner2(1) - Pcorner1(1);

cropRange = [Pcorner1(2) Pcorner1(1) xlength ylength];
Iwarp = imcrop(Iwarp, cropRange);



%================================================================
%                   TRANSLATE QR CODE TO STRING
%================================================================
level = graythresh(Iwarp);
Iwarp = im2bw(Iwarp, level);
finalstring = translate_qr(Iwarp);

else
    %Just a debug case
    finalstring = 'debug';

end

strout = finalstring;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%