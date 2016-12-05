function stlrr
close all; clc; clf;
[FC VR] = fileReader('pestininkas.stl');

figure(1); hold on, grid on, axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
patch('Faces',FC,'Vertices',VR, 'FaceColor',[0,0,1]);

VOX_SIZE = 0.6;

maxX = max(VR(:,1));
maxY = max(VR(:,2));
maxZ = max(VR(:,3));

minX = min(VR(:,1));
minY = min(VR(:,2));
minZ = min(VR(:,3));


voxSizeX = int16((maxX - minX)/VOX_SIZE);
voxSizeY = int16((maxY - minY)/VOX_SIZE);
voxSizeZ = int16((maxZ - minZ)/VOX_SIZE);

voxels = zeros(voxSizeX , voxSizeY, voxSizeZ);
 
figure(2); hold on, grid on, axis equal;


for i = 1 : voxSizeY
   
    curYEnd = minY + double(i)*VOX_SIZE + VOX_SIZE ;
     
     ELLayer = FC(ismember(sum([ ...
         VR(FC(:,1),2) < curYEnd ...
         VR(FC(:,2),2) < curYEnd ...
         VR(FC(:,3),2) < curYEnd]...
     ,2),[1 2]),:);
      
      for j = 1 : voxSizeX
              
                Ymid =  minY + double(i)*VOX_SIZE + VOX_SIZE/2;
                Xmid =  minX + double(j)*VOX_SIZE + VOX_SIZE/2;
                ZStart = minZ ;
                ZEnd = maxZ + VOX_SIZE * 2;
              
                Line1=[Xmid Ymid ZStart];
                Line2=[Xmid Ymid ZEnd];
                mindist = 0;
          
              for m=1:size(ELLayer)
                         ELs=ELLayer(m,:);
                         V=VR(ELs,:);
                                 [inSegment, IntersectionX, IntersectionY, IntersectionZ] = lineIntersectsTriangle(Line1,Line2, V(1,:),V(2,:),V(3,:));
               
                    if (inSegment)
                      indx = int16((IntersectionZ - minZ)/VOX_SIZE);
                      if(indx < 1) indx = 1; end;
                      voxels(j,i,indx) = 1;
                    end
              end
      end
      i
%       pause();
%      clf;  hold on, grid on, axis equal;
     
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
    j
end

end