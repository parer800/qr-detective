function [ output_args ] = translate_qr(I)
%TRANSLATE_QR Summary of this function goes here
%   Detailed explanation goes here

figure;
imshow(I);

cropWidth = size(I,2);
cropHeight = size(I,1);
nrOfQrBlocks = 41;
pixelsPerBlockX = cropWidth/41;
pixelsPerBlockY = cropHeight/41;
centerpointX = round(pixelsPerBlockX/2);
centerpointY = round(pixelsPerBlockY/2);
pixelsPerBlockX = round(pixelsPerBlockX);
pixelsPerBlockY = round(pixelsPerBlockY);

hold on;
ticker = 1;

centerOfApX = cropWidth-7*pixelsPerBlockX + centerpointX;
centerOfApY = cropHeight-7*pixelsPerBlockY + centerpointY;
plot(centerOfApX, centerOfApY, 'g+');

bitsequence = '';
bitticker = 0;

finalstring = '';

FIP_ratio = (1+1+3+1+1+1);

for x=centerpointX:pixelsPerBlockX:cropWidth
    for y=centerpointY:pixelsPerBlockY:cropHeight
        if(bitticker== 8)
            %Translate 8bit
            value = bin2dec(bitsequence);
            finalstring = [finalstring char(value)];
            
            
            bitsequence = '';
            bitticker = 0;
        end
        
        if(abs(x-centerOfApX) <=2*pixelsPerBlockX && abs(y-centerOfApY) <= 2*pixelsPerBlockY) % 2 Block ratio times pixels per block should be skipped (AP-mark)
            continue;
        end
        if(x<= pixelsPerBlockX*FIP_ratio)
            if(y>=pixelsPerBlockY*FIP_ratio && y + pixelsPerBlockY*FIP_ratio < cropHeight)
                %READ
                bitsequence = [bitsequence num2str(I(y,x))];
                bitticker = bitticker+1;
                
                plot(x, y, 'r.');
            end
        elseif(x + pixelsPerBlockX*FIP_ratio < cropWidth)
            %READ
            bitsequence = [bitsequence num2str(I(y,x))];
            bitticker = bitticker+1;
            plot(x, y, 'r.');
            
        else
            if(y>=pixelsPerBlockY*FIP_ratio)
                %READ
                bitsequence = [bitsequence num2str(I(y,x))];
                bitticker = bitticker+1;
                plot(x, y, 'r.');
            end
            continue;

        end
        
    end
end

output_args = finalstring;

end

