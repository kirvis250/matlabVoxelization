function stlrr
close all; clc; clf;
[FC, VR] = fileReader('pestininkas.stl');

figure(1); hold on, grid on, axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
patch('Faces',FC,'Vertices',VR, 'FaceColor',[0,0,1]);

VOX_SIZE = 1.0;


[maxX, maxY, maxZ, minX, minY, minZ] = boundingBox(VR);

voxSizeX = ceil((maxX - minX)/VOX_SIZE);
voxSizeY = ceil((maxY - minY)/VOX_SIZE);
voxSizeZ = ceil((maxZ - minZ)/VOX_SIZE);

voxels = zeros(voxSizeX , voxSizeY, voxSizeZ);
 


sizeFc = size(FC);

for f = 1 :sizeFc
    vertices = [VR(FC(f,1),:); VR(FC(f,2),:); VR(FC(f,3),:)];
    
    [vMaxX, vMaxY, vMaxZ, vMinX, vMinY, vMinZ] = boundingBox(vertices);
   

    maxXindex = ceil((vMaxX - minX)/VOX_SIZE);
    maxYindex = ceil((vMaxY - minY)/VOX_SIZE);
    maxZindex = ceil((vMaxZ - minZ)/VOX_SIZE);
    
    minXindex = floor((vMinX - minX)/VOX_SIZE);
    minYindex = floor((vMinY - minY)/VOX_SIZE);
    minZindex = floor((vMinZ - minZ)/VOX_SIZE);
    
    if(minXindex < 1); minXindex = 1; end;
    if(minYindex < 1); minYindex = 1; end;
    if(minZindex < 1); minZindex = 1; end;
    
    found = 0;
    
    for i = minXindex : maxXindex
        for j = minYindex : maxYindex
            for k = minZindex : maxZindex
              
                X =  minX + double(i)*VOX_SIZE;
                Y =  minY + double(j)*VOX_SIZE;
                Z =  minZ + double(k)*VOX_SIZE;   
               
               
               intersects = isVoxelIntersectsPolygon(VOX_SIZE, [X Y Z], vertices);
              
               if(intersects == 1)
                   voxels(i,j,k) = 1;
                   found = found + 1;
%                    cube_plot([X,Y,Z],VOX_SIZE,VOX_SIZE,VOX_SIZE,'r');
%                     pause(0.02);
               end
               
           end
       end
        
    end
    
       if(found == 0)
            disp(sprintf('found zero, algorithm sucks.'));
       end
    
       aa = sprintf('faces done %d / %d', f, sizeFc);
        disp(aa);
    
    
end

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



function intersects = isVoxelIntersectsPolygon(VOX_SIZE, cCords, polygon)
    
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
   
   
%      for i = 1 : 12 
%            line([edges(i,1) edges(i,4)],[edges(i,2) edges(i,5)],[edges(i,3) edges(i,6)],'Marker','.','LineStyle','-')
%      end
%      
       line([polygon(1,1) polygon(2,1)],[polygon(1,2) polygon(3,2)],[polygon(1,3) polygon(2,3)],'Marker','.','LineStyle','-')
       line([polygon(2,1) polygon(3,1)],[polygon(2,2) polygon(3,2)],[polygon(2,3) polygon(3,3)],'Marker','.','LineStyle','-')
       line([polygon(1,1) polygon(3,1)],[polygon(1,2) polygon(3,2)],[polygon(1,3) polygon(3,3)],'Marker','.','LineStyle','-')
      
    
    for i = 1 : 12 
        
           
%         line([edges(i,1) edges(i,4)],[edges(i,2) edges(i,5)],[edges(i,3) edges(i,6)],'Marker','.','LineStyle','-','Color','r')
       
        cline = [edges(i,1)             edges(i,2)              edges(i,3) 
               (edges(i,4)-edges(i,1)) (edges(i,5)-edges(i,2)) (edges(i,6)- edges(i,3))];
        
       isIntersect = isLineIntersectsPolygon( edges(i,:), polygon, VOX_SIZE, cCords);
      
    
       if(isIntersect)
%            line([edges(i,1) edges(i,4)],[edges(i,2) edges(i,5)],[edges(i,3) edges(i,6)],'Marker','.','LineStyle','-','Color','g')
           intersects = 1;
           return
       end
      
    end
    
end


function isIntersect = isLineIntersectsPolygon(line, polygon, VOX_SIZE, cCords)
% 
%
% Should return 0 if not intersects, and 1 if intersects
% line must be given [x y z,  z y z]; 
% polygon must be given  [x y z,  z y z,  z y z,  z y z ....]; 
%
% polygonVerticesCount = size(polygon);
% isIntersect = 0;

   [point, pos, isInside] = lineIntersectsTriangleMod( line, polygon);
   
    
%      distance = norm(point-cCords);
   
%     if (isInside && distance < VOX_SIZE*2)
%         isIntersect = 1;
%     end
   
 
    isIntersect = isInside;
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


