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

I = imread(im);

%Get dimensions, to check if 
[x,y,z] = size(I);
if z == 3
    z
    BW = rgb2gray(I);
    G = fspecial('disk',15);
    %Ig = imfilter(BW,G,'same');
    
    %BW = edge(BW, 'canny');
else
    z
    BW = I;
    %BW = edge(BW, 'canny');
end
%adaptive threshold image
atImage = adaptivethres(double(BW));


[H, T, R] = hough(atImage);


% Display the original image.
subplot(2,1,1);
figure;

title(im);

%LOOP THROUGH AND SAVE on every color switch, position (x,y), counter, pixelcolor. 
width = size(atImage,2);
height = size(atImage,1);

%Store in block of 4: [x,y,colorSeq, counterSeq]
segment = -ones(1,height*width);
color_switch = atImage(1,1);
sequence_counter = 0;
ticker = 1;
for y=1:height
    %current_v = atImage(y,x);
    sequence_counter = 0;
    for x=1:width
        %IF color has switched, store previous sequence in segment
        current_h = atImage(y,x);
        if current_h ~= color_switch
            %Color has switched
            component = [x,y,color_switch, sequence_counter]
            segment(1,ticker:ticker+3) = component;
            ticker = ticker+4;
            sequence_counter = 0;
            color_switch = current_h;
        end
            sequence_counter = sequence_counter + 1;
            
        
    end
end
x = (0);
qr_locations = double.empty();
for i=1:1:width*height
    if segment(1,i) == -1 && segment(1,i+4) == -1
        break;
    end
    sum = 0;
    first_color = segment(i+2);
    position = 0;
    for j=0:4        
        sum = sum + segment(i+3+j*4); % Get every counter value, add that to sum. check if modulus(sum, 7) is equal to 0 
    end
    
    position = [segment(i+8); segment(i+9)];
    if(mod(sum,7) == 0)
        center_weight = segment(i+11);
        if(center_weight/(sum-center_weight) ~= (0.75))
            continue;
        end
        %Hmm interesting.. found correct counter sequence but mabye not the
        % corerct color sequence. Since the colors are always switched we
        % only need to look at the first color value which should be black
        % Since we have a binary image we look for c ==0 || c==0.0
       % if first_color == 0 || first_color == 0.0
            %Located QR code!!
            qr_locations = [qr_locations, position];
        %end
    end
    
    
end
figure;
imshow(atImage);
hold on;
size(qr_locations,2)
for i=1:size(qr_locations,2)
    plot(qr_locations(1,i),qr_locations(2,i), '+');
end

%imshow(segment(:,ticker+4));







% Display & calculate the Hough matrix.
% 
% P  = houghpeaks(H,5);
% 
% subplot(2,1,2);
% imshow(imadjust(mat2gray(H)),'XData',T,'YData',R,...
%       'InitialMagnification','fit');
% title('Hough Transform of Image');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal, hold on;
% plot(T(P(:,2)),R(P(:,1)),'s','color','red');
% %colormap(hot);
% lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
% figure, imshow(atImage), hold on
% max_len = 0;
% for k = 1:length(lines)
%    xy = [lines(k).point1; lines(k).point2];
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% 
%    % Determine the endpoints of the longest line segment
%    len = norm(lines(k).point1 - lines(k).point2);
%    if ( len > max_len)
%       max_len = len;
%       xy_long = xy;
%    end
% end
% 
% % highlight the longest line segment
% plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','blue');





strout=char(im);
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%