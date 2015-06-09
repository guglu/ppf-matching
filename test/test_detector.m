% example usage of the PPF3DDetector object

% load model
model=loadPLY('data/mian_T-rex_high.ply');

% initialize detector
dt=PPF3DDetector(0.04,-1,30);
% train on the model
dt=dt.trainModel(model);


% load scene
scene=loadPLY('data/rs1.ply');

% find object in scene
% every 5th point in scene is selected is the reference point
% weighted voting + matching score calculation enabled
% voter saving enabled
[result, clusters, matchTime]=dt.match(scene,1/5,true,false,true,true, -1, -1);



% load ground truth pose
groundPose=Pose3D(1,1,1);
groundPose.updatePose(importdata('data/T-rex-rs1.xf'));

% check if correct pose detected
if comparePoses(result{1}, groundPose, dt.modelDiameter, dt.angleRadians);
    disp('correct detection')
else
    disp('incorrect detection')
end


% transform pose to the scene
resPC=TransformPose(model(:,1:3),result{1}.pose);

% save the detection result into the jpg image
pc2img('result.jpg',scene,resPC,result{1})

% write resulting transformed model into the ply file
savePLY('resPC.ply',resPC)