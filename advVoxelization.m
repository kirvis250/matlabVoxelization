function stlrr
close all; clc; clf;
[FC, VR] = fileReader('pestininkas.stl');

figure(1); hold on, grid on, axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
patch('Faces',FC,'Vertices',VR, 'FaceColor',[0,0,1]);

VOX_SIZE = 0.8;


[maxX, maxY, maxZ, minX, minY, minZ] = boundingBox(VR);

voxSizeX = ceil((maxX - minX)/VOX_SIZE);
voxSizeY = ceil((maxY - minY)/VOX_SIZE);
voxSizeZ = ceil((maxZ - minZ)/VOX_SIZE);

voxels = zeros(voxSizeX , voxSizeY, voxSizeZ);
 
figure(2); hold on, grid on, axis equal;

sizeFc = size(FC);

for f = 1 :sizeFc
    vertices = [VR(FC(f,1),:); VR(FC(f,2),:); VR(FC(f,3),:)];
    
    [vMaxX, vMaxY, vMaxZ, vMinX, vMinY, vMinZ] = boundingBox(vertices);
   

    maxXindex = ceil((vMaxX + VOX_SIZE - minX)/VOX_SIZE);
    maxYindex = ceil((vMaxY + VOX_SIZE - minY)/VOX_SIZE);
    maxZindex = ceil((vMaxZ + VOX_SIZE - minZ)/VOX_SIZE);
    
    minXindex = floor((vMinX - VOX_SIZE - minX)/VOX_SIZE);
    minYindex = floor((vMinY - VOX_SIZE - minY)/VOX_SIZE);
    minZindex = floor((vMinZ - VOX_SIZE - minZ)/VOX_SIZE);
    
    if(minXindex < 1); minXindex = 1; end;
    if(minYindex < 1); minYindex = 1; end;
    if(minZindex < 1); minZindex = 1; end;
    
    
    for i = minXindex : maxXindex
        found = 0;
        for j = minYindex : maxYindex
            for k = minZindex : maxZindex
              
                X =  minX + double(i)*VOX_SIZE;
                Y =  minY + double(j)*VOX_SIZE;
                Z =  minZ + double(k)*VOX_SIZE;   
               
               
               intersects = isVoxelIntersectsPolygon(VOX_SIZE, [X Y Z], vertices);
              
               if(intersects == 1)
                   voxels(i,j,k) = 1;
                   found = found + 1;
                   
               end
               
           end
       end
        
    end
    
       if(found == 0)
            sprintf('Error ')
       end
    
    sprintf('faces done %d/%d ',f, sizeFc)
    
    
end

for j = 1 : voxSizeY
    for i = 1 : voxSizeX
        for k = 1 : voxSizeZ
            
            if(voxels(i,j,k) == 1)
                
                Y =  minY + double(j)*VOX_SIZE + VOX_SIZE/2;
                X =  minX + double(i)*VOX_SIZE + VOX_SIZE/2;
                Z =  minX + double(k)*VOX_SIZE + VOX_SIZE/2;
                              
                cube_plot([X,Y,Z],VOX_SIZE,VOX_SIZE,VOX_SIZE,'r');
            end

        end
    end
end

end



function intersects = isVoxelIntersectsPolygon(VOX_SIZE, cCords, polygon)
    
    intersects = 0;

    points = zeros(8, 3);
    edges = zeros(12, 6);
    u = VOX_SIZE;
    
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
       isIntersect = isLineIntersectsPolygon(edges(i,:), polygon, VOX_SIZE);
      
       
       if(isIntersect == 1)
           intersects = 1;
           return
       end
      
    end
    
    for i = 1: size(polygon)
        if(     polygon(i,1) > cCords(1)- VOX_SIZE/2 && ...
                polygon(i,2) > cCords(2)- VOX_SIZE/2 && ...
                polygon(i,3) > cCords(3)- VOX_SIZE/2 && ...
                polygon(i,1) < cCords(1)+ VOX_SIZE/2 && ...
                polygon(i,2) < cCords(2)+ VOX_SIZE/2 && ...
                polygon(i,3) < cCords(3)+ VOX_SIZE/2 ...
            )
              intersects = 1;
               return
        end
    end
    
end


function isIntersect = isLineIntersectsPolygon(line, polygon, VOX_SIZE)
% 
%
% Should return 0 if not intersects, and 1 if intersects
% line must be given [x y z,  z y z]; 
% polygon must be given  [x y z,  z y z,  z y z,  z y z ....]; 
%
polygonVerticesCount = size(polygon);
isIntersect = 0;

   [point, pos, isInside] = isLineIntersectsTriangle( line, polygon);
    
%      [isInside, xx, yy, zz] = lineIntersectsTriangle(line(1:3),line(4:6), polygon(1,:),polygon(2,:),polygon(3,:));
%      point = [xx yy zz];
    
     distance = norm(point-line(1:3));
   
    if (isInside && distance < VOX_SIZE)
        isIntersect = 1;
    end
   
    return;
end



function [xMax, yMax, ZMax, xMin, yMin, ZMin] = boundingBox(vertices)
%
% returns bounding box of polygon, mesh or any point cloud. 
% vertices must be given [x y z,  z, y, z] format
%   Note: This doesnt check validity of polygon. it 
    xMax = max(vertices(:,1));
    yMax = max(vertices(:,2));
    ZMax = max(vertices(:,3));

    xMin = min(vertices(:,1));
    yMin = min(vertices(:,2));
    ZMin = min(vertices(:,3));
   
end


