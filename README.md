# ppf-matching
Matlab implementation of the Point-pair feature matching method proposed by Drost et al. [1] 

Several improvements which allow to speed-up the detection process and also to increase the detection rate are implemented. Detailed description of these improvements can be found in my [master's thesis](https://dspace.cvut.cz/handle/10467/62026?locale-attribute=en) 

If you use this code in your research please cite [Detection and Localization of Texture-Less Objects in RGB-D Images](https://dspace.cvut.cz/handle/10467/62026?locale-attribute=en)

```
@MASTERSTHESIS\{CTU2015-62026,
    author       = "P. Zednik",
    title        = "Detection and Localization of Texture-Less Objects in RGB-D Images",
    year         = "2015",
}
```


Directories:
---------------------

`mex/`   - MEX versions of some functions

`ppf/`  - Point-pair feature detector

`test/`  - Example usage of the detector

Example:
---------------------
```matlab

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

```

Bibliography:
---------------------
[1] Drost, Bertram, et al. "Model globally, match locally: Efficient and robust 3D object recognition." Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on. IEEE, 2010.

