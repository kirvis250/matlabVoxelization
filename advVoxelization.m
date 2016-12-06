function advVoxelization
close all; clc; clf;
[FC, VR] = fileReader('teapot.obj');

figure(1); hold on, grid on, axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
patch('Faces',FC,'Vertices',VR, 'FaceColor',[0,0,1]);

VOX_SIZE = 4.6;
PADDING = VOX_SIZE;


[maxX, maxY, maxZ, minX, minY, minZ] = boundingBox(VR,PADDING);

voxSizeX = ceil((maxX - minX)/VOX_SIZE);
voxSizeY = ceil((maxY - minY)/VOX_SIZE);
voxSizeZ = ceil((maxZ - minZ)/VOX_SIZE);

voxels = zeros(voxSizeX , voxSizeY, voxSizeZ);
 


sizeFc = size(FC);

for f = 1 :sizeFc
    
    %Face vertexes
    vertices = [VR(FC(f,1),:); VR(FC(f,2),:); VR(FC(f,3),:)];
    
    %Face bvounding box
    [vMaxX, vMaxY, vMaxZ, vMinX, vMinY, vMinZ] = boundingBox(vertices,PADDING);
   
    %Max check indexes
    maxXindex = ceil((vMaxX - minX)/VOX_SIZE);
    maxYindex = ceil((vMaxY - minY)/VOX_SIZE);
    maxZindex = ceil((vMaxZ - minZ)/VOX_SIZE);
    
    %Min check indexes
    minXindex = floor((vMinX - minX)/VOX_SIZE);
    minYindex = floor((vMinY - minY)/VOX_SIZE);
    minZindex = floor((vMinZ - minZ)/VOX_SIZE);
    
    %Normalization if calculated index is 0 or less. 
    if(minXindex < 1); minXindex = 1; end;
    if(minYindex < 1); minYindex = 1; end;
    if(minZindex < 1); minZindex = 1; end;
    
   
    for i = minXindex : maxXindex
        for j = minYindex : maxYindex
            for k = minZindex : maxZindex
                
                % Voxel center coordinate. 
                X =  minX + double(i)*VOX_SIZE;
                Y =  minY + double(j)*VOX_SIZE;
                Z =  minZ + double(k)*VOX_SIZE;   
               
               intersects = isVoxelIntersectsPolygon34(VOX_SIZE, [X Y Z], vertices);
              
               if(intersects == 1)
                   voxels(i,j,k) = 1;
               end
               
           end
        end
    end
       aa = sprintf('faces done %d / %d', f, sizeFc);
       disp(aa);
end

figure(2); hold on, grid on, axis equal;
for j = 1 : voxSizeY
    for i = 1 : voxSizeX
        for k = 1 : voxSizeZ
            
            if(voxels(i,j,k) == 1)
                
                X =  minX + double(i)*VOX_SIZE;
                Y =  minY + double(j)*VOX_SIZE;
                Z =  minZ + double(k)*VOX_SIZE;
                              
                cube_plot([X,Y,Z],VOX_SIZE,VOX_SIZE,VOX_SIZE,'g');
            end

        end
    end
end

end



function intersects = isVoxelIntersectsPolygon12(VOX_SIZE, cCords, polygon)
    
    intersects = 0;

    points = zeros(8, 3);
    edges = zeros(12, 6);
    u = VOX_SIZE/2;
    
    points(1,:) = cCords + [+u +u +u];
    points(2,:) = cCords + [-u +u +u];
    points(3,:) = cCords + [-u -u +u];
    points(4,:) = cCords + [+u -u +u];
    points(5,:) = cCords + [-u -u -u];
    points(6,:) = cCords + [+u -u -u];
    points(7,:) = cCords + [+u +u -u];
    points(8,:) = cCords + [-u +u -u];
    
    edges(1,:) = [points(1,:) points(2,:)];
    edges(2,:) = [points(2,:) points(3,:)];
    edges(3,:) = [points(3,:) points(4,:)];
    edges(4,:) = [points(4,:) points(1,:)];
    edges(5,:) = [points(1,:) points(7,:)];
    edges(6,:) = [points(7,:) points(6,:)];
    edges(7,:) = [points(6,:) points(5,:)];
    edges(8,:) = [points(5,:) points(8,:)];
    edges(9,:) = [points(6,:) points(4,:)];
    edges(10,:) = [points(5,:) points(3,:)];
    edges(11,:) = [points(2,:) points(8,:)];
    edges(12,:) = [points(7,:) points(8,:)];
   
   
    for i = 1 : 12 
        
       dir1 = (edges(i,4) - edges(i,1));
       dir2 = (edges(i,5) - edges(i,2));
       dir3 = (edges(i,6) - edges(i,3));
       cline = [edges(i,1) edges(i,2) edges(i,3) dir1 dir2 dir3];
        
       [point, pos, isInside] = lineIntersectsTriangleMod( cline, polygon);
       distance = sqrt(sum( (point - cCords) .^2)); 
       
     
       if(isInside && distance < VOX_SIZE)
           intersects = 1;
           return
       end
    end
end


function intersects = isVoxelIntersectsPolygon6(VOX_SIZE, cCords, polygon)
    
    intersects = 0;

    points = zeros(8, 3);
    edges = zeros(12, 6);
    u = VOX_SIZE/2;
    
    points(1,:) = cCords + [+u 0 0];
    points(2,:) = cCords + [0 +u 0];
    points(3,:) = cCords + [0 0 +u];
    points(4,:) = cCords + [-u 0 0];
    points(5,:) = cCords + [0 -u 0];
    points(6,:) = cCords + [0 0 -u];
    
    edges(1,:) = [points(1,:) points(4,:)];
    edges(2,:) = [points(2,:) points(5,:)];
    edges(3,:) = [points(3,:) points(6,:)];

    for i = 1 : 12 
        
        dir1 = (edges(i,4) - edges(i,1));%/norm(edges(i,4) - edges(i,1));
        dir2 = (edges(i,5) - edges(i,2));%/norm(edges(i,5) - edges(i,2));
        dir3 = (edges(i,6) - edges(i,3));%/norm(edges(i,6) - edges(i,3));
        cline = [edges(i,1) edges(i,2) edges(i,3) dir1 dir2 dir3];
        
        [point, pos, isInside] = lineIntersectsTriangleMod( cline, polygon);
        distance = sqrt(sum( (point - cCords) .^2)); 
         
       if(isInside && distance < VOX_SIZE)
           intersects = 1;
           return
       end
    end
end


function intersects = isVoxelIntersectsPolygon34(VOX_SIZE, cCords, polygon)
    
    intersects = 0;

    points = zeros(8, 3);
    edges = zeros(12, 6);
    u = VOX_SIZE/2;
    
    points(1,:) = cCords + [+u 0 0];
    points(2,:) = cCords + [0 +u 0];
    points(3,:) = cCords + [0 0 +u];
    
    points(4,:) = cCords + [+u +u +u];
    points(5,:) = cCords + [ 0 +u +u];
    points(6,:) = cCords + [+u  0 +u];
    points(7,:) = cCords + [+u +u  0];

    points(8,:) = cCords + [-u 0 0];
    points(9,:) = cCords + [0 -u 0];
    points(10,:) = cCords + [0 0 -u];
    
    points(11,:) = cCords + [-u -u -u];
    points(12,:) = cCords + [ 0 -u -u];
    points(13,:) = cCords + [-u  0 -u];
    points(14,:) = cCords + [-u -u  0];
    
    edges(1,:) = [points(1,:) points(8,:)];
    edges(2,:) = [points(2,:) points(9,:)];
    edges(3,:) = [points(3,:) points(10,:)];
    edges(4,:) = [points(4,:) points(11,:)];
    edges(5,:) = [points(5,:) points(12,:)];
    edges(6,:) = [points(6,:) points(13,:)];
    edges(7,:) = [points(7,:) points(14,:)];
    

    for i = 1 : 12 
        
        dir1 = (edges(i,4) - edges(i,1));%/norm(edges(i,4) - edges(i,1));
        dir2 = (edges(i,5) - edges(i,2));%/norm(edges(i,5) - edges(i,2));
        dir3 = (edges(i,6) - edges(i,3));%/norm(edges(i,6) - edges(i,3));
        cline = [edges(i,1) edges(i,2) edges(i,3) dir1 dir2 dir3];
        
        [point, pos, isInside] = lineIntersectsTriangleMod( cline, polygon);
        distance = sqrt(sum( (point - cCords) .^2)); 
         
       if(isInside && distance < VOX_SIZE)
           intersects = 1;
           return
       end
    end
end


function [xMax, yMax, ZMax, xMin, yMin, ZMin] = boundingBox(vertices, padding)
%
% returns bounding box of polygon, mesh or any point cloud. 
% vertices must be given [x y z,  z, y, z] format
%   Note: This doesnt check validity of polygon. it 
    xMax = max(vertices(:,1)) + padding;
    yMax = max(vertices(:,2)) + padding;
    ZMax = max(vertices(:,3)) + padding;

    xMin = min(vertices(:,1)) - padding;
    yMin = min(vertices(:,2)) - padding;
    ZMin = min(vertices(:,3)) - padding;
   
end


