function pc2img(filename,scene, model, varargin)
% project scene and model into the image file 
%
% left half of the image is the scene, the right half of the image is model
% in detected pose
%
% if fourth argument provided, voters are also plotted
% reference points are depicted in red, paired points in green
% "darkness" represents the mutual angle of normals

if size(scene,2)>6
    withColor=true;
else
    withColor=false;
end

%image size
imWidth=2*640;
imHeight=2*480;
%create image

im=zeros(imHeight,2*imWidth,3);
%im=255.*ones(imHeight,2*imWidth,3);


%camera projection
angl=-pi/4;
T1=[cos(angl) 0 -sin(angl);0 1 0;sin(angl) 0 cos(angl)];
T2=[cos(angl) -sin(angl) 0 ;sin(angl)  cos(angl) 0;0 0 1];
T3=[1 0 0;0 cos(angl) -sin(angl); 0 sin(angl) cos(angl)];
P=[T3 [0;0;1]];
%P=eye(4);
%apply projection
pcMod=p2e(P*e2p(model(:,1:3)'))';
pcSce=p2e(P*e2p(scene(:,1:3)'))';
%get borders of the scene
minX=min(pcSce(:,1));
maxX=max(pcSce(:,1));
minY=min(pcSce(:,2));
maxY=max(pcSce(:,2));


%print scene
%normalize to fit in image / y coordinate is flipped
[xcoord, ycoord] = cropScreen( pcSce, maxX, minX, maxY, minY, imWidth, imHeight,0);
%print 3x3 neighborhood
xcoord=[xcoord; xcoord+1; xcoord;   xcoord-1; xcoord;   xcoord+1; xcoord-1; xcoord+1; xcoord-1];
ycoord=[ycoord; ycoord;   ycoord+1; ycoord;   ycoord-1; ycoord+1; ycoord-1; ycoord-1; ycoord+1];

filtered = (xcoord>1 & xcoord<imWidth & ycoord>1 & ycoord<imHeight);
pNum=sum(filtered);

if withColor
    sceColor=repmat(scene(:,7:9),9,1);
    
    im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),1.*ones(pNum,1)))=sceColor(filtered,1);
    im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),2.*ones(pNum,1)))=sceColor(filtered,2);
    im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),3.*ones(pNum,1)))=sceColor(filtered,3);
else
    im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),1.*ones(pNum,1)))=repmat(255,pNum,1);
    im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),2.*ones(pNum,1)))=repmat(255,pNum,1);
    im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),3.*ones(pNum,1)))=repmat(255,pNum,1);
end

%print model
% x coord is moved to the rigth side of the image
[xcoord, ycoord] = cropScreen( pcMod, maxX, minX, maxY, minY, imWidth, imHeight,1);

%xcoord=[xcoord; xcoord+1; xcoord;   xcoord-1; xcoord;   xcoord+1; xcoord-1; xcoord+1; xcoord-1];
%ycoord=[ycoord; ycoord;   ycoord+1; ycoord;   ycoord-1; ycoord+1; ycoord-1; ycoord-1; ycoord+1];

%filtered = (xcoord>imWidth+1 & xcoord<2*imWidth & ycoord>1 & ycoord<imHeight);
filtered = (xcoord>imWidth & xcoord<=2*imWidth & ycoord>0 & ycoord<=imHeight);
pNum=sum(filtered);

im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),1.*ones(pNum,1)))=repmat(255,pNum,1);
im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),2.*ones(pNum,1)))=repmat(255,pNum,1);
im( sub2ind(size(im),ycoord(filtered),xcoord(filtered),3.*ones(pNum,1)))=repmat(255,pNum,1);

%if voters are available
if nargin>3
    
    for vargs=1:length(varargin)
        
        greenColor=[0 255 0];
        redColor=[255 0 0];
        blueColor=[0 0 255];
        
        circleWidth=5;
        
        %get voters and pose
        voters=varargin{vargs}.voters;
        
        if ~voters==0
            
            pose=varargin{vargs}.pose;
            
            vNum=size(voters,1);
            
            eps=0.05;
            sceneNorm = (1 - abs(dot(voters(:,4:6),voters(:,10:12),2)) + eps)/(1+eps);
            %sceneNorm = ones(size(voters,1),1);
            sceneCol =  bsxfun(@times,repmat(greenColor,vNum,1),sceneNorm);
            modelNorm = (1 - abs(dot(voters(:,16:18),voters(:,22:24),2)) + eps)/(1+eps);
            modelCol =  bsxfun(@times,repmat(greenColor,vNum,1),modelNorm);
            %project scene voters
            sceneIproj=p2e(P*e2p(voters(:,1:3)'))';
            sceneYproj=p2e(P*e2p(voters(:,7:9)'))';
            
            %transform and project model voters
            modelI=TransformPose(voters(:,13:15),pose);
            modelY=TransformPose(voters(:,19:21),pose);
            modelIproj=p2e(P*e2p(modelI'))';
            modelYproj=p2e(P*e2p(modelY'))';
            
            %scene
            sceneIcircle = vision.ShapeInserter('Shape','Circles','Fill', true,'FillColor', 'Custom','CustomFillColor', redColor);
            sceneYcircle = vision.ShapeInserter('Shape','Circles','Fill', true,'FillColor', 'Custom','CustomFillColor', sceneCol);
            
            [xcoord, ycoord] = cropScreen( sceneIproj, maxX, minX, maxY, minY, imWidth, imHeight,0);
            im = step(sceneIcircle, im, [xcoord ycoord circleWidth*ones(vNum,1)]);
            [xcoord, ycoord] = cropScreen( sceneYproj, maxX, minX, maxY, minY, imWidth, imHeight,0);
            im = step(sceneYcircle, im, [xcoord ycoord circleWidth*ones(vNum,1)]);
            
            %model
            modelIcircle = vision.ShapeInserter('Shape','Circles','Fill', true,'FillColor', 'Custom','CustomFillColor', redColor);
            modelYcircle = vision.ShapeInserter('Shape','Circles','Fill', true,'FillColor', 'Custom','CustomFillColor', modelCol);
            
            [xcoord, ycoord] = cropScreen( modelIproj, maxX, minX, maxY, minY, imWidth, imHeight,1);
            im = step(modelIcircle, im, [xcoord ycoord circleWidth*ones(vNum,1)]);
            [xcoord, ycoord] = cropScreen( modelYproj, maxX, minX, maxY, minY, imWidth, imHeight,1);
            im = step(modelYcircle, im, [xcoord ycoord circleWidth*ones(vNum,1)]);
            
        end
    end
end


%write image
imwrite(uint8(im),filename)


end

function [xcoord, ycoord] = cropScreen( pc, maxX, minX, maxY, minY, imWidth, imHeight,model)

   xcoord=floor(imWidth.*bsxfun(@minus,pc(:,1),minX)./(maxX-minX));
   ycoord=floor(imHeight.*bsxfun(@minus,pc(:,2),minY)./(maxY-minY));
   
   if model
   xcoord=xcoord+imWidth;
   end
end


function [out] = p2e(input)

out=zeros(size(input,1)-1,size(input,2));

for i=1:size(input,1)-1
    
    out(i,:)=input(i,:)./input(end,:);
end

end


function [out] = e2p(input)

out=[input;ones(1,size(input,2))];

end