function sampledPC = samplePCpoisson(pc,sampleStep, refitNormals)
% sample the point cloud using the poisson disc sampling
% sample step is given relative to the point cloud
% if third argument present, normal vectors are recomputed

if nargin > 2
    refitNormals = true;
else
    refitNormals = false;
end


rangeX=[min(pc(:,1)), max(pc(:,1))];
rangeY=[min(pc(:,2)), max(pc(:,2))];
rangeZ=[min(pc(:,3)), max(pc(:,3))];

dx=rangeX(2)-rangeX(1);
dy=rangeY(2)-rangeY(1);
dz=rangeZ(2)-rangeZ(1);

%length of diagonal of bounding box
d=sqrt(dx^2+dy^2+dz^2);

%minimal distance of the points
r=d*sampleStep;

rs=r^2;

boxSize=r/sqrt(3);

samplesInDimX=floor(dx/boxSize);
samplesInDimY=floor(dy/boxSize);
samplesInDimZ=floor(dz/boxSize);

map=zeros(samplesInDimX,samplesInDimY,samplesInDimZ);

%process points
for i=1:size(pc,1)
    
    xCell= floor(samplesInDimX * (pc(i,1)-rangeX(1)) / dx)+1;
    yCell= floor(samplesInDimY * (pc(i,2)-rangeY(1)) / dy)+1;
    zCell= floor(samplesInDimZ * (pc(i,3)-rangeZ(1)) / dz)+1;
    
    
    %select neighbors 5x5x5 points
    neigh = reshape(map(max((xCell-2), 1):min((xCell+2), end), max((yCell-2), 1):min((yCell+2), end), max((zCell-2), 1):min((zCell+2), end)) , [] ,1 );
    
    %no points aroud
    if neigh==0
        map(xCell,yCell,zCell)=i;
        
    %check distance    
    elseif  (sum(bsxfun(@minus,pc(nonzeros(neigh),1:3),pc(i,1:3)).^2 ,2 ) < rs) == 0
            map(xCell,yCell,zCell)=i;
    end
    
    
end

indx=reshape(nonzeros(map),[],1);

sampledPC=pc(indx,:);


if refitNormals
    windowWidth=(5*boxSize)^2;
    
    if size(sampledPC,2) < 4
       
        sampledPC=[sampledPC zeros(size(sampledPC))];
        
    end
    
    for i=1:size(sampledPC,1)
        
        %compute distance of the point i to all other points - sqrt not needed
        %dist2=sum((sampledPC(:,1:3) - repmat(sampledPC(i,1:3), size(sampledPC,1), 1 )).^2,2);
        dist2=(sum(bsxfun(@minus,sampledPC(:,1:3),sampledPC(i,1:3)).^2,2));
        
        %get indexes of nearest 10 points
        [elems,indxs]=getSmalestN(dist2,15);
        
        neigh=indxs(elems<windowWidth);
        
        if length(neigh)>5
            v = affine_fit(sampledPC(neigh,1:3))' ;
            if v(3)>0
                
                sampledPC(i,4:6) = v;
            else
                sampledPC(i,4:6) = -v;
            end
        end
    end
    
end


if size(pc,2)>3
    
    nNorm=sqrt(sum(sampledPC(:,4:6).^2,2));
    
    indx=nNorm>0;
    
    sampledPC(indx,4:6)=bsxfun(@rdivide, sampledPC(indx,4:6), nNorm(indx) );
    
end

end