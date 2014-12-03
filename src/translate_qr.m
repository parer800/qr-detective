function [ output_args ] = translate_qr(I)
%TRANSLATE_QR Summary of this function goes here
%   Detailed explanation goes here

figure;
imshow(I);

cropWidth = size(I,2);
cropHeight = size(I,1);
nrOfQrBlocks = 41;
pixelsPerBlock = cropWidth/41
centerpoint = round(pixelsPerBlock/2)
pixelsPerBlock = round(pixelsPerBlock);

hold on;
ticker = 1;

centerOfApX = cropWidth-7*pixelsPerBlock + centerpoint;
centerOfApY = cropHeight-7*pixelsPerBlock + centerpoint;
plot(centerOfApX, centerOfApY, 'g+');

bitsequence = '';
bitticker = 0;

finalstring = '';

FIP_ratio = (1+1+3+1+1+1);

for x=centerpoint:pixelsPerBlock:cropWidth
    for y=centerpoint:pixelsPerBlock:cropHeight
        if(bitticker== 8)
            %Translate 8bit
            bitsequence
            value = bin2dec(bitsequence);
            finalstring = [finalstring char(value)];
            
            
            bitsequence = '';
            bitticker = 0;
        end
        
        if(abs(x-centerOfApX) <=2*pixelsPerBlock && abs(y-centerOfApY) <= 2*pixelsPerBlock) % 2 Block ratio times pixels per block should be skipped (AP-mark)
            continue;
        end
        if(x<= pixelsPerBlock*FIP_ratio)
            if(y>=pixelsPerBlock*FIP_ratio && y + pixelsPerBlock*FIP_ratio < cropHeight)
                %READ
                bitsequence = [bitsequence num2str(I(y,x))];
                bitticker = bitticker+1;
                
                plot(x, y, 'r.');
            end
        elseif(x + pixelsPerBlock*FIP_ratio < cropWidth)
            %READ
            bitsequence = [bitsequence num2str(I(y,x))];
            bitticker = bitticker+1;
            plot(x, y, 'r.');
            
        else
            if(y>=pixelsPerBlock*FIP_ratio)
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

