function [ output_args ] = getTemplateOfAP( h, w )

nrOfBlocks = 41;

PPBX = w/nrOfBlocks %PPBX - PixelsPerBlockX
PPBY = h/nrOfBlocks %PPBY - PixelsPerBlockY

ratioOfAP = (1+1+1+1+1) %Ratio of blocks in AP is 1:1:1:1:1 

APTemplate = ones(round(PPBY*ratioOfAP), round(PPBX*ratioOfAP));
PPBX = size(APTemplate,2)/ratioOfAP;
PPBY = size(APTemplate,1)/ratioOfAP;

startSquareX = ceil(1/5 * size(APTemplate,2))+1;
endSquareX = ceil(4/5 * size(APTemplate,2))+1;
startSquareY = ceil(1/5 * size(APTemplate,1))+1;
endSquareY = ceil(4/5 * size(APTemplate,1))+1;

APTemplate(startSquareY:endSquareY,startSquareX:endSquareX) = 0; 
APTemplate(startSquareY+PPBY:endSquareY-1*PPBX, startSquareX+PPBX:endSquareX-1*PPBX ) = 1;
output_args = APTemplate;
end

