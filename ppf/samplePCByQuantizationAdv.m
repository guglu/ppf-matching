function sampledPC = samplePCByQuantizationAdv(pc,sampleStep, useWeight, refitNormals)
% sample point cloud using the quantization
% sample step is given relative to the point cloud
% if fourth argument present, normal vectors are recomputed

if nargin > 3
    refitNormals = true;
else
    refitNormals = false;
end

%increase bounding box slightly
myEps=0.001;


rangeX=[min(pc(:,1)), max(pc(:,1))];
rangeY=[min(pc(:,2)), max(pc(:,2))];
rangeZ=[min(pc(:,3)), max(pc(:,3))];

rangeX(1)=rangeX(1) - abs(rangeX(1)*myEps);
rangeX(2)=rangeX(2) + abs(rangeX(2)*myEps);
rangeY(1)=rangeY(1) - abs(rangeY(1)*myEps);
rangeY(2)=rangeY(2) + abs(rangeY(2)*myEps);
rangeZ(1)=rangeZ(1) - abs(rangeZ(1)*myEps);
rangeZ(2)=rangeZ(2) + abs(rangeZ(2)*myEps);

    dx=rangeX(2)-rangeX(1);
    dy=rangeY(2)-rangeY(1);
    dz=rangeZ(2)-rangeZ(1);
    
    %length of diagonal of bounding box
    d=sqrt(dx^2+dy^2+dz^2);
    
    %length of one size of the box - approximating from cube box
    boxSize=d*sampleStep*sqrt(2);
    
    samplesInDimX=floor(dx/boxSize);
    samplesInDimY=floor(dy/boxSize);
    samplesInDimZ=floor(dz/boxSize);
    
    %samplesInDim=floor(1/sampleStep);
    
    %map=cell((samplesInDim+1)^3 , 1);
    map=cell((samplesInDimX)*(samplesInDimY)*(samplesInDimZ) + 1, 1);
    
    for i=1:size(pc,1)

       xCell= floor(samplesInDimX * (pc(i,1)-rangeX(1)) / dx);
       yCell= floor(samplesInDimY * (pc(i,2)-rangeY(1)) / dy); 
       zCell= floor(samplesInDimZ * (pc(i,3)-rangeZ(1)) / dz); 
       
       index=samplesInDimY*samplesInDimZ*xCell+samplesInDimZ*yCell+zCell;
       
       map{index+1}(end+1)=i;
    end

    %non emplty cells in map
    mapInd=(find(~cellfun(@isempty,map)));
    %number of non emplty cells in map
    mapSize=length(mapInd);
    %initialize output matrix
    
    pcSize=size(pc,2);
   
    sampledPC=zeros(mapSize,pcSize);
    
    iter=1;
    
    %iterating only over non-empty bins 
    for i=mapInd';
        
            curCell=map{i};
              
            if useWeight
            
                index=i-1;
                
                zCell=rem(index,samplesInDimZ);
                yCell=rem( (index-zCell)/samplesInDimZ , samplesInDimY);
                xCell=( index-zCell-yCell*samplesInDimZ )/( samplesInDimY*samplesInDimZ );
                centerCoord=[ ( xCell+0.5 )*( dx/samplesInDimX ) + rangeX(1), ( yCell+0.5 )*( dx/samplesInDimY ) + rangeY(1), ( zCell+0.5 )*( dx/samplesInDimZ ) + rangeZ(1)];
                
                %weghts -  1/distance from point to center of box
                %w = 1./sqrt(sum((pc(curCell,1:3)-repmat(centerCoord,length(curCell),1)).^2,2));
                w= 1./sqrt(sum((bsxfun(@minus,(pc(curCell,1:3)),centerCoord)).^2,2));
                point=sum(bsxfun(@times,pc(curCell,:),w),1)./sum(w);
                
            else
                
                point=sum(pc(curCell,:),1)./length(curCell);
                
            end
            
            %if working with normals
            if pcSize>3
                %normalize normals
                nrNorm=norm(point(4:6));
                if nrNorm>0
                    point(4:6)=point(4:6)./nrNorm;
                end
                
            end
            
            
            sampledPC(iter,:)=point;
            iter=iter+1;
        %end
    end

     %recompute normals
    if refitNormals
        
        for i=1:size(sampledPC,1)
            
            %compute distance of the point i to all other points - sqrt not needed
            %dist2=sum((sampledPC(:,1:3) - repmat(sampledPC(i,1:3), size(sampledPC,1), 1 )).^2,2);
            dist2=sum(bsxfun(@minus,sampledPC(:,1:3),sampledPC(i,1:3)).^2,2);
            
            %get indexes of nearest 10 points
            [~,indxs]=getSmalestN(dist2,10+1);
            
            sampledPC(i,4:6) = affine_fit(sampledPC(indxs,1:3))' ;
          
        end
        
    end

end