function newPc = TransformPose(pc,pose)
%transform point cloud according to the given pose

    %newPc=zeros(size(pc));
    newPc=pc;
    
    p=(pose*[pc(:,1:3) ones(size(pc,1),1)]')';

    newPc(:,1:3)=[p(:,1)./p(:,4), p(:,2)./p(:,4), p(:,3)./p(:,4)];

    %with normals
    if size(pc,2)>3
        
        n=(pose(1:3,1:3)*pc(:,4:6)')';     
        nNorm=sqrt(sum((n.^2),2));      
        indx=(nNorm>0);
        
        n(indx,1)=n(indx,1)./nNorm(indx);
        n(indx,2)=n(indx,2)./nNorm(indx);
        n(indx,3)=n(indx,3)./nNorm(indx);
        
        newPc(:,4:6)=n;
        
        %newPc(:,4:6)=[n(:,1)./nNorm, n(:,2)./nNorm, n(:,3)./nNorm];
        
    end
end