function [ output_args ] = main()
slCharacterEncoding('UTF-8');
directory = dir('../../testImages');

fileID = fopen('QRdecode.txt','w');
fprintf(fileID,'%12s  %18s\n','Image','Text');

counter=1;
for i=1:length(directory)
    folder =strcat('../../testImages/Images_Training_',num2str(i));
    imagefiles = dir(folder); 
    for j=1:length(imagefiles)
        if(strcmp(imagefiles(j).name,'.')  || strcmp(imagefiles(j).name,'..'))
            continue;
        else
            %folder
           currentfilename = imagefiles(j).name;
           currentimage = imread(strcat(folder,'/',currentfilename));
%            
           code = tnm034(strcat(folder,'/',currentfilename));% String decoded
           fprintf(fileID,'%s \n\n%12s \n \n',currentfilename,code);
           fprintf(fileID,'--------------- \n \n');
        
        end
    end
end

fclose(fileID);
end

