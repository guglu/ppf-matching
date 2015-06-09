classdef PPF3DDetector < handle
    % PPF3DDetector The detector class
    % The detector is first initialized, then trained on particular model
    % using method trainModel. The detection is performed by passing the
    % scene to the method match.
    
    properties (SetAccess = private)
        
        samplingRelative = 0.05; % the sampling of the model
        
        distanceRelative = 0.05; % parameter not used 
        
        angleRelative = 30;	% number of bins for the angle sampling
        
        angleRadians;   % angle step in rad
        
        trained=false;  % is detector detected
        
        distanceStep;
        sampledPC;          % sampled model used for training
        hashTable;          % hash table with hashed features
        refPointNum;        % number of reference points used in matching
        modelDiameter;      % diameter of the model used for training
        poseList;           % list of pose hypotheses
        sceneDistanceStep;  % sampling step of the scene
        sceneDiameter;      % diameter of the scene
    end
    
    methods
        
        function obj = PPF3DDetector(samplingRelative, distanceRelative, angleRelative)
            % Constructor of the detector class. SamplingRelative specifies
            % the sampling of the model. Default value is 0.05.
            % distanceRelative parameter is not used. The angleRelative
            % parameter determines the number of bins for the angle
            % sampling, default value is 30
            
            obj.samplingRelative = samplingRelative;
            obj.distanceRelative = distanceRelative;
            obj.angleRelative = angleRelative;
            obj.angleRadians = (360/angleRelative)*pi/180;
        end
        
        function obj = clearTrainingModels(obj)
            % method removes the trained data
            
            obj.hashTable=0;
            obj.trained = false;
        end
        
        function obj = trainModel(obj,pc)
            % trainModel Train the detector on the passed model
            
            angleRadiansLoc=obj.angleRadians;
                 
            %compute bounding box
            rangeX=[min(pc(:,1)), max(pc(:,1))];
            rangeY=[min(pc(:,2)), max(pc(:,2))];
            rangeZ=[min(pc(:,3)), max(pc(:,3))];
            
            %diameter of the scene
            obj.modelDiameter = norm([rangeX(2)-rangeX(1) , rangeY(2)-rangeY(1) , rangeZ(2)-rangeZ(1)]);
            %diameter of one of the sampled boxes
            distanceStepLoc = obj.modelDiameter * obj.samplingRelative;           
            obj.distanceStep=distanceStepLoc;          
            
           % sampledPCLoc = samplePCByQuantizationAdv(pc,obj.samplingRelative,true);
             sampledPCLoc = samplePCpoisson(pc,obj.samplingRelative);
            obj.sampledPC=sampledPCLoc;
            
            refPointNumLoc=size(obj.sampledPC,1);
            obj.refPointNum=refPointNumLoc;
            
            disp(['sampled pc size: ' num2str(obj.refPointNum) ])

            hashTableLoc = containers.Map('KeyType','uint32', 'ValueType','any');
            
            lambda=0.98;
            
            for i=1:obj.refPointNum
               
                p1=sampledPCLoc(i,:);
                
                for j=1:refPointNumLoc
                    
                    %if selected points are different
                    if i~=j
                        
                        p2=sampledPCLoc(j,:);
                        
                        f=computePPFmex(p1,p2);
                        hash=hashPPF(f,angleRadiansLoc,distanceStepLoc);
                        alphaAngl = computeAlpha(p1,p2);
                        
                        % -1 to accomodate indexing from 1 
                        coordInd=(i-1)*refPointNumLoc + (j-1);
                        ppfInd=coordInd;
                        
                        %compute vote
                        dp=p1(4)*p2(4)+p1(5)*p2(5)+p1(6)*p2(6);
                        voteVal=1-lambda*abs(dp);

                        %use only a simple array
                        node=[i;ppfInd;alphaAngl;voteVal];
                        
                        if isKey(hashTableLoc,hash)
                            hashTableLoc(hash)=[hashTableLoc(hash), node];
                        else
                            hashTableLoc(hash)=node;
                        end

                        
                    end
                    
                end
                
                if mod(i,10)==0
                    
                    disp( ['trained: ', num2str(round(100*i/obj.refPointNum)), ' %'] )
                end
                
                
            end
 
            % save local values into the object
            obj.hashTable=hashTableLoc;
            obj.trained = true;
        end
        
        
        
        
        function [result, clustered, internalTime] = match(obj, pc, sceneFraction, averageVotingSwitch, filterPosesSwich, recomputeScoreSwich, saveVotersSwitch , sceneSampling, poseFilterThreshold)
        % match Find object in the scene. 
        %
        % [result, pose clusters, time for detection] = match(model of the scene, sceneFraction, averageVotingSwitch, filterPosesSwich, recomputeScoreSwich, saveVotersSwitch , sceneSampling, poseFilterThreshold)
        %
        % sceneFraction - fraction of the points used as the reference points
        % averageVotingSwitch - enable the weighted voting
        % filterPosesSwich - enable the hypotheses pruning
        % recomputeScoreSwich - enable matching score calculation
        % saveVotersSwitch - enable if point which voted should be saved
        % sceneSampling - sampling step of the scene (if < 0, the model sampling is used)
        % poseFilterThreshold - threshold for hypotheses pruning
        
            refPointNumLoc=obj.refPointNum;
            angleRadiansLoc=obj.angleRadians;
            distanceStepLoc=obj.distanceStep;
            angleRelativeLoc=obj.angleRelative;
            hashTableLoc=obj.hashTable;
            sampledPCLoc=obj.sampledPC;
           
           sceneStep=floor(1/sceneFraction);
           
           %compute bounding box
           rangeX=[min(pc(:,1)), max(pc(:,1))];
           rangeY=[min(pc(:,2)), max(pc(:,2))];
           rangeZ=[min(pc(:,3)), max(pc(:,3))];
            
            %diameter of the scene
            diameter = norm([rangeX(2)-rangeX(1) , rangeY(2)-rangeY(1) , rangeZ(2)-rangeZ(1)]);
            
            obj.sceneDiameter=diameter;
            
            %diameter of sampled boxes in scene will be the same as in the the model
            obj.sceneDistanceStep = 1/ (diameter / distanceStepLoc);
            
            
            % if sceneSampling is specified independently for the scene
            %sampling step is relative to the MODEL diameter
            if sceneSampling>0
               
              obj.sceneDistanceStep = 1/ ( diameter/(obj.modelDiameter*sceneSampling));
                
            end
        
            %sampledScene = samplePCByQuantizationAdv(pc,sceneDistanceStep,false);
            sampledScene = samplePCpoisson(pc,obj.sceneDistanceStep);
            
            sceneSize=size(sampledScene,1);
            disp(['sceneSize: ' num2str(sceneSize) ])
            %allocate poseList - expecting 3 hypotheses from each scene reference point 
            obj.poseList=cell( 3*round(sceneSize/sceneStep) , 1);
            disp(['scene ' num2str(sceneSize/sceneStep) ' poseList ' num2str(size(obj.poseList,1))])
            
            posesAdded=1;

            internalTic=tic;
            internalTime=[];
            
            for i=1:sceneStep:sceneSize
            
            %random selection
            %perm=randperm(sceneSize);
            %disp(num2str(perm(1:floor(sceneSize/sceneStep))))
            %for i=perm(1:floor(sceneSize/sceneStep))
               
                disp(['Matching:' num2str(i) ' of '  num2str(sceneSize) ])                          
                
                p1=sampledScene(i,:);
                
                accumulator=zeros( (refPointNumLoc + 1 ) * angleRelativeLoc , 1);
                
                if saveVotersSwitch
                    coordAccumulator = cell( (refPointNumLoc + 1 ) * angleRelativeLoc , 1);
                end
                
                [R, t] = transformRT(p1);
                
                %%%%%%%%%%%%%%%%%%%%%%
                %selection of the points in the scene
                %%%%%%%%%%%%%%%%%%%%%%
                %version 1
                %test everything
                %for j=1:sceneSize
                %%%%%%%%%%%%%%%%%%%%%%
                %version 2
                %test only points closer than model diameter
                ind=1:sceneSize;
                closePoints = ( sum(bsxfun(@minus,sampledScene(:,1:3),p1(:,1:3)).^2,2) < (obj.modelDiameter)^2 );                
                for j=ind(closePoints)
                %%%%%%%%%%%%%%%%%%%%%%
                  
                    if i~=j
                        
                        p2=sampledScene(j,:);
                        
                        f=computePPFmex(p1,p2);
                        hash=hashPPF(f,angleRadiansLoc,distanceStepLoc);
                    
                        p2t=R*p2(1:3)'+t;

                        alphaScene=atan2(-p2t(3),p2t(2));
                        
                        if sin(alphaScene)*p2t(3)>0
                            alphaScene=-alphaScene;
                        end
                        
                        %if some correspondence exist
                        if isKey(hashTableLoc,hash)
                            %get corresponding nodes from hash table
                            nodeList=hashTableLoc(hash);
                            
                            nNodes=size(nodeList,2);
                            
                            for nodeInd=1:nNodes
                              
                                modelI=nodeList(1,nodeInd);
                                ppfInd=nodeList(2,nodeInd);
                                alphaModel=nodeList(3,nodeInd);
                                                                
                                alphaAngl=alphaModel-alphaScene;
                                
                                %get alpha to range <-2pi,2pi>
                                if alphaAngl>pi
                                    alphaAngl=alphaAngl-2*pi;
                                elseif alphaAngl<-pi
                                    alphaAngl=alphaAngl+2*pi;
                                end
                                
                                %index in range < 0 , obj.angleRelative >
                                alphaIndex=round((angleRelativeLoc-1) * (alphaAngl + pi) / (2*pi));
                                
                                accuIndex = modelI * angleRelativeLoc + alphaIndex;
                                
                                %%%%%%%%%%%%%%%%%%%%%%
                                %adaptive vote in accumulator
                                %%%%%%%%%%%%%%%%%%%%%%
                                if ~averageVotingSwitch
                                    %version 1
                                    %add 1
                                    voteVal=1;
                                else
                                    %%%%%%%%%%%%%%%%%%%%%%
                                    %version 2
                                    %use dot product of corresponding model normals
                                    
                                    %modelY=ppfInd-(modelI-1)*refPointNumLoc + 1;
                                    %dp=sampledPCLoc(modelI,4)*sampledPCLoc(modelY,4)+sampledPCLoc(modelI,5)*sampledPCLoc(modelY,5)+sampledPCLoc(modelI,6)*sampledPCLoc(modelY,6);
                                    %voteVal=(1-abs(dp)+myEps)/(1+myEps);
                                    voteVal=nodeList(4,nodeInd);
                                    %%%%%%%%%%%%%%%%%%%%%%
                                end
                                accumulator(accuIndex,1) = accumulator(accuIndex,1) + voteVal;
                                
                                if saveVotersSwitch
                                    
                                    coordAccumulator{accuIndex,1}(end+1,:) = [j ppfInd];
                                end
                            end
                        end
                    end                  
                end
                
                %%%%%%%%%%%%%%%%%%%%%%
                %selection of the poses from accumulator
                %%%%%%%%%%%%%%%%%%%%%%
                %version 1
                %return all poses with more than 95% of votes of the best pose
                accuMax=0.95*max(accumulator);         
                %%%%%%%%%%%%%%%%%%%%%%
                %version 2
                %return poses with number of votes which is more than 0.1% of total votes 
                %accuMax=0.001*sum(accumulator);
                %%%%%%%%%%%%%%%%%%%%%%
                accuMaxInd=(accumulator>accuMax);
                
                
                %for each peak in accumulator
                for peak=1:length(accuMaxInd);
                   
                    %if peak exist
                    if accuMaxInd(peak)
                        %indexes to the accumulator
                        alphaMaxInd=mod(peak,angleRelativeLoc);                     
                        iMaxInd=(peak-alphaMaxInd)/angleRelativeLoc;
                        
                        
                        iR=R';
                        it=iR*t;
                        %stuct into one matrix
                        iT=[iR -it;0 0 0 1];
                        
                        %get corresponding point from model
                       	pMax=sampledPCLoc(iMaxInd,:);
                        
                        [RMax, tMax] = transformRT(pMax);                       
                        TMax= [RMax tMax; 0 0 0 1];

                        %get back alpha in range <-2pi , 2pi >
                        alphaAngl = (2*pi) * alphaMaxInd / (angleRelativeLoc - 1) - pi;
                        Talpha = XrotMat(alphaAngl);
                        
                        Tpose=iT*(Talpha*TMax);
                        
                        %%%%%%%%%%%%%%%%%%%%%%
                        %normalization of the votes from accumulator
                        %%%%%%%%%%%%%%%%%%%%%%
                        %version 1
                        %no normalization
                        numVotes=accumulator(peak,1);
                        %%%%%%%%%%%%%%%%%%%%%%
                        %version 2
                        %normalize number of votes by number of tested
                        %point-pair features in scene
                        %numVotes=accumulator(peak,1)/sum(closePoints);
                        %%%%%%%%%%%%%%%%%%%%%%
                                                
                         %use structure
                        newPose=Pose3D(alphaAngl,posesAdded,numVotes);
                        newPose.updatePose(Tpose);
                                           
                        % save points which voted
                        if saveVotersSwitch
                            
                            voted=coordAccumulator{peak,1};
                            
                            % compute coordinates of the voters
                            % i scene  j scene  i model j model
                            modelI= bsxfun(@plus,floor(voted(:,2)./refPointNumLoc) , 1);
                            modelY = bsxfun(@plus, bsxfun(@rem,voted(:,2),refPointNumLoc) , 1);
                            votes = [ repmat(sampledScene(i,1:6),size(voted,1),1) , sampledScene(voted(:,1),1:6) , obj.sampledPC(modelI,1:6) , obj.sampledPC(modelY,1:6)  ];
                            newPose.addVoter( votes );
                        else
                            
                            newPose.addVoter( 0 );
                            
                        end
                        
                        obj.poseList{posesAdded}=newPose;
                        posesAdded=posesAdded + 1;
                    end
                end

            end
           
             %remove empty cells
             obj.poseList(cellfun('isempty',obj.poseList ))=[];
             
             %save time to compute
             internalTime(end+1)=toc(internalTic);
             
             
             %show number of poses
             disp(['Poses:' num2str(posesAdded)])
            
             
             internalTic=tic;
             
             %if pose filtering enabled
             if filterPosesSwich             
                filterPoses(obj,poseFilterThreshold);
                disp(['Filtered poses: ' num2str(size(obj.poseList,1))])            
             end
             %save time after pose filter
             internalTime(end+1)=toc(internalTic);
             
             
             internalTic=tic;
             
             %cluster poses
             [clustered, votes]=clusterPoses(obj);              
             disp(['Clustered poses:' num2str(size(clustered,1))])
             
             %save time after pose clustering
             internalTime(end+1)=toc(internalTic);
             
             
             internalTic=tic;
             
             %average clusters
             resultOrig=sortPoses(averageClusters(clustered,votes));

             %save time after cluster averaging
             internalTime(end+1)=toc(internalTic);
             
             internalTic=tic;
             %if score recomputed
              if recomputeScoreSwich
                result = sortPoses(recomputeScore(obj,resultOrig,sampledScene));
              else
                result = resultOrig;
              end

             %save time after score recomputation
             internalTime(end+1)=toc(internalTic);
             
        end

               
        function [clusters, votes] = clusterPoses(obj)
            % clusterPoses will cluster pose list into the clusters
            
            modelDiameterLoc=obj.modelDiameter;
            angleRadiansLoc=obj.angleRadians;
            
            %sort poses by number of votes
            sorted = sortPoses(obj.poseList);
            
            %preallocate
            poseClusters=cell(size(sorted,1),1);
            votes=zeros(size(sorted,1),1);
            
            clusterNum=0;
            
            
            %search all poses
            for i=1:size(sorted,1)
                
                assigned = false;
                curPose=sorted{i};
                
                %search all clusters
                for j=1:clusterNum
                    
                    %get first pose from cluster
                    poseCenter=poseClusters{j}{1};
                    
                    if comparePosesMex(curPose.pose(1:3,4),poseCenter.pose(1:3,4),curPose.angle,poseCenter.angle,curPose.omega,poseCenter.omega,modelDiameterLoc,angleRadiansLoc)    
                        
                        poseClusters{j}{end+1}=curPose;
                        
                        votes(j)=votes(j)+curPose.numVotes;
                        assigned = true;
                        break;
                        
                    end
                end
                
                if ~assigned
                    
                    poseClusters{clusterNum+1,1}={curPose};
                    votes(clusterNum+1,1)=curPose.numVotes;
                    clusterNum=clusterNum+1;
                end
            end
  
            clusters = ( poseClusters(1:clusterNum,1) );
        end
 
        function outPoses = recomputeScore(obj,poses,scene)
            % recomputeScore will calculate the matching score for pose list
        
            outPoses = poses;
            
            model=obj.sampledPC;
            
            r=obj.sceneDistanceStep*obj.sceneDiameter;
            
            rs=r^2;
            
            %for each pose
            for i=1:size(poses,1)
            
                curMod=TransformPose(model,poses{i}.pose);
                
                score=0;
                
                %check each point on the model
                for j=1:size(curMod,1)
                
                    dist=(sum(bsxfun(@minus,scene(:,1:3),curMod(j,1:3)).^2 ,2 ) < rs);
                
                    if sum(dist)>0
                        score=score+1;
                    end
                    
                end

                outPoses{i}.updateScore(score/size(curMod,1));
                
            end
            
            
            
        end
        
        function filterPoses(obj,threshold)
        % filterPoses will prune the hypotheses according to the poseFilterThreshold
        
           %select which poses to remove 
           ind=cell2mat(cellfun(@(x)x.numVotes<threshold,obj.poseList,'UniformOutput',false));
           
           obj.poseList(ind)=[];
           
        end
        
    end
    
    
end



function resultPoses = averageClusters(clusters,votes)
    % resultPoses averages the poses in each cluster

resultPoses=cell(size(clusters,1),1);

for i=1:size(clusters,1)
    
    newPose=Pose3D(0,-1,votes(i));
    %get all quaternions
    qs=cell2mat(cellfun(@(x)x.q, clusters{i},'UniformOutput',false));
    %get all translations
    ts=cell2mat(cellfun(@(x)x.pose(1:3,4), clusters{i},'UniformOutput',false));
    %get all voters
    voters=cell2mat(cellfun(@(x)x.voters, clusters{i},'UniformOutput',false)');
    
    newPose.updatePoseT(mean(ts,2));
    newPose.updatePoseQuat(avg_quaternion_markley(qs')');
    newPose.addVoter(voters);
    
    resultPoses{i}=newPose;
 
end

end



function sorted = sortPoses(poses)
    % sortPoses will sort the poses according to their number of votes
    
    %extract number of votes
    %use Pose3D structure
    numVotes = cellfun(@(x)x.numVotes, poses);

    [~,sortIdx] = sort(numVotes,'descend');
    
    sorted = poses(sortIdx);

end

