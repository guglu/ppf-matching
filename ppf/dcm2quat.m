function q = dcm2quat(R)
% rotation matrix to quaternion

    tr = trace(R);
    
    if tr > 0
        
        q=[tr+1; R(2,3)-R(3,2); R(3,1)-R(1,3); R(1,2)-R(2,1)];
        nr=q(1);

    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        
        q=[R(2,3)-R(3,2); 1+R(1,1)-R(2,2)-R(3,3); R(2,1)+R(1,2); R(3,1)+R(1,3)];
        nr=q(2);
        
    elseif R(2,2)>R(3,3)
        
        q=[R(3,1)-R(1,3); R(2,1)+R(1,2); 1+R(2,2)-R(1,1)-R(3,3); R(3,2)+R(2,3)];
        nr=q(3);
        
    else
        
        q=[R(1,2)-R(2,1); R(3,1)+R(1,3); R(3,2)+R(2,3); 1+R(3,3)-R(1,1)-R(2,2)];
        nr=q(4);
        
    end
    
    q=q.*(0.5/sqrt(nr));

end