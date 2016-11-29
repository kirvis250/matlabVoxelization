function demoObjRead()
% 
%  Demonstration that obj and stl loading works. 
%  
%  if this returns error about missing files.. add sampleData to matlab or 
%  replace all paths to full path to file (i.e. 'D:\myPancakes\matlabVoxelization\sampleData\airboat.obj')
% 

[F,V] = fileReader( '\sampleData\airboat.obj');

figure(2);
hold();
axis equal
axis vis3d
trisurf(F,V(:,1),V(:,2),V(:,3),'FaceColor',[0.26,0.33,1.0 ]);


[F,V] = fileReader(['sampleData\pestininkas.stl']);

figure(3);
hold();
axis equal
axis vis3d
trisurf(F,V(:,1),V(:,2),V(:,3),'FaceColor',[0.26,0.33,1.0 ]);


[F,V] = fileReader(['\sampleData\airboat.obj']);

figure(4);
hold();
axis equal
axis vis3d
trisurf(F,V(:,1),V(:,2),V(:,3),'FaceColor',[0.26,0.33,1.0 ]);
% light('Position',[-1.0,-1.0,100.0],'Style','infinite');
% lighting phong;

end