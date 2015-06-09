classdef Pose3D < handle
% Pose3D class
    
    properties (SetAccess = private)
        alpha=0;
        modelIndex=0;
        numVotes=0;

        pose=zeros(4,4);
        
        voters;
        angle;
        omega;
        q;
    end
   
    methods
        function obj = Pose3D(alpha,modelIndex, numVotes)
         %  obj.voted=voted;
           obj.alpha=alpha;
           obj.modelIndex=modelIndex;
           obj.numVotes=numVotes;
                      
        end
        
        
        function updatePose(obj,newPose)
           
            %pose matrix
            obj.pose=newPose;
            %angle axis
            vrot=vrrotmat2vec(obj.pose(1:3,1:3));
            obj.omega=vrot(1:3);
            obj.angle=vrot(4);    
            %quaternion
            obj.q=dcm2quat(obj.pose(1:3,1:3));
        end
        
        function updatePoseQuat(obj,qNew)
           
            obj.q=qNew;
            %rotation matrix
            oldPose=obj.pose;
            
            newPose=[quat2dcm(qNew) oldPose(1:3,4); 0 0 0 1];
            obj.pose=newPose;
            
            %angle axis
            vrot=vrrotmat2vec(newPose(1:3,1:3));
            obj.omega=vrot(1:3);
            obj.angle=vrot(4);  
        end
        
        function updatePoseT(obj,t)
            
            obj.pose=[obj.pose(1:3,1:3) t; 0 0 0 1];
            
        end
        function addVoter(obj,voter)
            
        	obj.voters=cat(1,obj.voters,voter);
        end
        
        function updateScore(obj,newScore)
            
        	obj.numVotes=newScore;
            
        end
        
    end
    
    
end