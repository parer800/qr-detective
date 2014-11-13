%Extension of the Weller's method [Wellner 1993]

function bw = adaptivethres(in)
    w = size(in,1)
    h = size(in,2)
    %calculate integral image
    intImg = integralImage(in);
    s=w/8;
    t = 15;
    
    for i=1:w
        for j=1:h
            sHalf = floor(s/2);
            x1 = i-(sHalf);
            x2 = i+(sHalf);
            y1 = j-sHalf;
            y2 = j+sHalf;
            
            if x1 < 2
                x1 = i+1;
            end
            if y1 < 2
                y1 = j+1;
            end
            if i==1
                x1=2;
            end
            if j==1
                y1=2;
            end
            if x2 > w
                x2 = w;
            end
            if y2 > h
                y2 = h;
            end
            count = (x2-x1)*(y2-y1);
            sum = intImg(x2,y2) - intImg(x2,y1-1) - intImg(x1-1,y2) + intImg(x1-1,y1-1);
            if in(i,j)*count <= (sum*(100-t)/100)
                bw(i,j) = 0;
            else
                bw(i,j) = 255;
            end
        end
    end
    
    %imshow(bw);
    
    

