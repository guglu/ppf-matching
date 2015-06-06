function savePLY(filename,pc)
% write point cloud to the ply file

fid = fopen(filename,'wt');  % Note the 'wt' for writing in text mode

fprintf(fid,'ply\n');
fprintf(fid,'format ascii 1.0\n');
fprintf(fid,'comment VCGLIB generated\n');
fprintf(fid,'element vertex %i\n',size(pc,1));
fprintf(fid,'property float x\n');
fprintf(fid,'property float y\n');
fprintf(fid,'property float z\n');


if size(pc,2)>3
    
    fprintf(fid,'property float nx\n');
    fprintf(fid,'property float ny\n');
    fprintf(fid,'property float nz\n');
end

if size(pc,2)>6
    
    fprintf(fid,'property uchar red\n');
    fprintf(fid,'property uchar green\n');
    fprintf(fid,'property uchar blue\n');
end

if size(pc,2)==10
    fprintf(fid,'property uchar alpha\n');
end

fprintf(fid,'element face 0\n');
fprintf(fid,'property list uchar int vertex_indices\n');
fprintf(fid,'end_header\n');

if size(pc,2)==10
     for i=1:size(pc,1)
        fprintf(fid,'%f %f %f %f %f %f %d %d %d %d\n',pc(i,1),pc(i,2),pc(i,3),pc(i,4),pc(i,5),pc(i,6),pc(i,7),pc(i,8),pc(i,9),pc(i,10));
    end   
    
elseif size(pc,2)==9
    for i=1:size(pc,1)
        fprintf(fid,'%f %f %f %f %f %f %d %d %d\n',pc(i,1),pc(i,2),pc(i,3),pc(i,4),pc(i,5),pc(i,6),pc(i,7),pc(i,8),pc(i,9));
    end
    
elseif size(pc,2)==6
    for i=1:size(pc,1)
        fprintf(fid,'%f %f %f %f %f %f\n',pc(i,1),pc(i,2),pc(i,3),pc(i,4),pc(i,5),pc(i,6));
    end
    
elseif size(pc,2)==3
    for i=1:size(pc,1)
        fprintf(fid,'%f %f %f\n',pc(i,1),pc(i,2),pc(i,3));
    end
end


fclose(fid);

end