function mesh = loadPLY(fileName)
% load point cloud from the ply file

fid = fopen(fileName,'r');
tline = fgets(fid);
vertex=2;

cols=6;

% process header
while true
    
    if strfind(tline,'element vertex')
        vertexLine = textscan(tline,'%s');
        vertex = str2num(vertexLine{1}{3});
 
    elseif strfind(tline,'property uchar red')
        cols=9;
    elseif strfind(tline,'property uchar alpha')
        cols=10;
    elseif strfind(tline,'end_header')
        break;
    end

    tline = fgets(fid);
end


scan=fscanf(fid, '%f', cols*vertex);
mesh=reshape(scan',cols,[])';
