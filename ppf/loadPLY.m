function mesh = loadPLY(fileName, skipNormals)
% load point cloud from the ply file

if nargin > 1
    skipNormals = true;
else
    skipNormals = false;
end


fid = fopen(fileName,'r');
tline = fgets(fid);

vertex=0;
cols=0;
vertexEntered = false;

if skipNormals
    expectedIndexes = zeros(1,3);
else
    expectedIndexes = zeros(1,6);
end
% process header
while true
    
    if findInLine(tline,'format')
        formatLine = textscan(tline,'%s');
        format = formatLine{1}{2};
        if ~strcmp(format, 'ascii')
            error 'Binary format not supported!'
        end
    elseif findInLine(tline,'element vertex')
        vertexLine = textscan(tline,'%s');
        vertex = str2num(vertexLine{1}{3});
        vertexEntered = true;
    elseif (vertexEntered && findInLine(tline, 'property'))
        propertyLine = textscan(tline,'%s');
        property = propertyLine{1}{3};
        cols = cols + 1;
        
        if strcmp(property,'x')
            expectedIndexes(1) = cols;
        elseif strcmp(property,'y')
            expectedIndexes(2) = cols;
        elseif strcmp(property,'z')
            expectedIndexes(3) = cols;
        elseif ~skipNormals && strcmp(property,'nx')
            expectedIndexes(4) = cols;
        elseif ~skipNormals && strcmp(property,'ny')
            expectedIndexes(5) = cols;
        elseif ~skipNormals && strcmp(property,'nz')
            expectedIndexes(6) = cols;
        end
    elseif findInLine(tline,'element') == 1
        vertexEntered = false;
    elseif findInLine(tline,'end_header') == 1
        break;
    end

    tline = fgets(fid);
end

if (~isempty(find(expectedIndexes==0, 1)))
    error 'File does not contain all required attributes (x, y, z, nx, ny, nz)'
end

if (vertex > 0 && cols > 0)
    scan=fscanf(fid, '%f', cols*vertex);
    full_mesh=reshape(scan',cols,[])';
    mesh = full_mesh(:,expectedIndexes);
    
else
    error 'Unable to read file'
end

end

function res = findInLine(line, string)
    loc = strfind(line, string);
    if ~isempty(loc)
        res = (loc(1) == 1);
    else
        res = false;
    end
end