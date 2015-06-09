function T = importHinter(fpath,N)
% imports the ground truth pose for the hinterstoisser dataset

filenameRot=[fpath '/rot' num2str(N) '.rot'];
filenameTr=[fpath '/tra' num2str(N) '.tra'];


T=[zeros(3,4); 0 0 0 1];

fidRot = fopen(filenameRot,'r');
fidTr = fopen(filenameTr,'r');

tlineRot = fgets(fidRot);
tlineTr = fgets(fidTr);

iter=1;
while ischar(tlineRot)
    
    tlineRot = fgets(fidRot);
    tlineTr = fgets(fidTr);
    
    if iter>=1 && iter <=3
        T(iter,1:3)=str2num(tlineRot);
        %converting from cm to mm
        T(iter,4)=10*str2num(tlineTr);
    end
    iter=iter+1;
    
end


fclose(fidRot);
fclose(fidTr);
end