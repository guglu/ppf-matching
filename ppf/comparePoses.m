        function out = comparePoses(pose1, pose2, objectDiameter, angleLim)
            
            %translation
            d=norm(pose1.pose(1:3,4) - pose2.pose(1:3,4));

            %angle
            phi=abs(pose1.angle-pose2.angle);

            if d < (0.10 * objectDiameter) && phi < angleLim 
                out=true;
            else
                out=false;
            end
            
        end