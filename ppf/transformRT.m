function [R, t] = transformRT(p1)
%compute transformation to rotate n1 to x axis and p1 to origin

    angle=acos(p1(4));
    
    axis=[0 p1(6) -p1(5)];
    
    if p1(5)==0 && p1(6)==0
        axis=[0 1 0];
    else
        axisNorm=norm(axis);
        
        if axisNorm>0            
            axis=axis./axisNorm;
        end
    end
    
    R=vrrotvec2mat([axis angle]);
    t=-R*p1(1:3)';
    
end