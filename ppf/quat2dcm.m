function R = quat2dcm(q)
% convert quaternion into the rotation matrix

    R=zeros(3,3);
    sq=q.^2;
  
    R(1,1)=(sq(2)-sq(3)-sq(4)+sq(1));
    R(2,2)=(-sq(2)+sq(3)-sq(4)+sq(1));
    R(3,3)=(-sq(2)-sq(3)+sq(4)+sq(1));
    
    R(1,2)=2*(q(2)*q(3)+q(4)*q(1));
    R(2,1)=2*(q(2)*q(3)-q(4)*q(1));
    
    R(1,3)=2*(q(2)*q(4)-q(3)*q(1));
    R(3,1)=2*(q(2)*q(4)+q(3)*q(1));
    
    R(2,3)=2*(q(3)*q(4)+q(2)*q(1));
    R(3,2)=2*(q(3)*q(4)-q(2)*q(1));
end