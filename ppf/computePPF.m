function f = computePPF(p1,p2)
% compute point pair feature for two points

% d=p2(1:3)-p1(1:3);
% n1=p1(4:6);
% n2=p2(4:6);
% if norm(d)>0
%     dNorm=d./norm(d);
%
%     f1=atan2(norm(cross( dNorm , n1 )), dot( dNorm , n1 ));
%     f2=atan2(norm(cross( dNorm , n2 )), dot( dNorm , n2 ));
%     f3=atan2(norm(cross( n1 , n2 )), dot(n1 , n2));
% else
%     f1=0;
%     f2=0;
%     f3=0;
% end
% f=[f1 f2 f3 norm(d)];



%fast version
d=(p2(1:3)-p1(1:3))';
%n1=p1(4:6);
%n2=p2(4:6);
if norm(d)>0
    dNorm=d./norm(d);
    
    a=[dNorm  dNorm  p1(4:6)'];
    b=[p1(4:6);  p2(4:6); p2(4:6)]';
    
    f=[ atan2(  sqrt(sum((	(cross(a,b)).^2 )))    ,    dot(a,b)) norm(d)];
else
    f=[0 0 0 norm(d)];
end