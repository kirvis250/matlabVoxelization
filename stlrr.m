function stlrr
close all; clc; clf;
[FC VR] = stlRead('pestininkas.stl');

save(['T1_13.mat'], 'FC');

figure(1); hold on, grid on, axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
patch('Faces',FC,'Vertices',VR, 'FaceColor',[0,0,1]);


HPoints = 25;
CPoints = 25;

XX = zeros(HPoints,CPoints);
YY = zeros(HPoints,CPoints);
ZZ = zeros(HPoints,CPoints);

tol=1e-2*(max(VR(:,3))-min(VR(:,3)));

min_h = min(VR(:,3)) + tol;
max_h = max(VR(:,3)) - tol;


dt_rows = (max_h - min_h)/(HPoints-1);
dt_columns = 2*pi/CPoints;

yc =(max(VR(:,2)+min(VR(:,2))))/2;
yx =(max(VR(:,1)+min(VR(:,1))))/2;
center = [yx yc]

maxD = max(VR(:,2)) + max(VR(:,1));

rows =  [min_h:dt_rows:max_h+dt_rows];
columns = [0:dt_columns:2*pi];
 
                XX(1,:)=yx;
                YY(1,:)=yc;
                ZZ(1,:)=min_h;
rowww = size(rows)+1;

figure(2); hold on, grid on, axis equal;

for i=2:rowww(2)
    rowStart = rows(i-1);
    rowEnd = rowStart + dt_rows; 
    
    ELLayer = FC(ismember(sum([VR(FC(:,1),3)<rowEnd VR(FC(:,2),3)<rowEnd VR(FC(:,3),3)<rowEnd],2),[1 2]),:);
%     patch('faces',ELLayer,'vertices',Vert,'facecolor',[1 1 0],'EdgeColor',[0.2 0.2 0.2]);
%     pause
    XMidLayer=mean(VR(:,1)); 
    YMidLayer=mean(VR(:,2)); 
    
     Line1=[XMidLayer YMidLayer rowEnd];
    for j=1:CPoints+1
        
       column = columns(j); 
        xk=cos(column);
        yk=sin(column);
        X2Layer=XMidLayer+maxD*xk; 
        Y2Layer=YMidLayer+maxD*yk;
        DistanceM=0; 
        Line2=[X2Layer Y2Layer rowEnd]; 
        for m=1:size(ELLayer)
            ELs=ELLayer(m,:);
            V=VR(ELs,:);
        [inSegment, IntersectionX, IntersectionY, IntersectionZ]=lineIntersectsTriangle(Line1,Line2, V(1,:),V(2,:),V(3,:));
            distXYZ=norm([IntersectionX, IntersectionY, IntersectionZ]-Line1);
            
             if inSegment   && distXYZ>DistanceM
                
                XX(i,j)=IntersectionX;
                YY(i,j)=IntersectionY; 
                ZZ(i,j)=rowStart;
                
                cube_plot([  XX(i,j),  YY(i,j),  ZZ(i,j)],1,1,1,'r');
                
                DistanceM=distXYZ;
            end
        end
    end
    
    
end

  XX(HPoints+1,:)=yx;
  YY(HPoints+1,:)=yc;
  ZZ(HPoints+1,:)=max_h-dt_rows/2;
          


figure;
% surf(XX,YY,ZZ); axis equal;

TX=rows; 
TY=ZZ(:,1);
X=XX; Y=YY; Z=ZZ;
save(['T1_13.mat'], 'X', 'Y', 'Z','TX','TY')


end