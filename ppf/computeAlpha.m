function alpha = computeAlpha(p1,p2)
% compute angle alpha

    [R, t]=transformRT(p1);

    mpt=R*p2(1:3)'+t;
    
    alpha=atan2(-mpt(3),mpt(2));
    
    if sin(alpha)*mpt(3)>0
        alpha=-alpha;
    end
        
end