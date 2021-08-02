function mCADplot(uav,option)
% Plot ANAFI PARROT CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T

% Function calls:
    % A.mCADplot('non-rotate');
    % A.mCADplot('rotate');

    % Exemple:
    %         if t < 5
    %             A.mCADplot('non-rotate');
    %         else
    %             A.mCADplot('rotate');
    %         end

    

uav.pPos.Xc = uav.pPos.X;

if nargin < 2
    uav.pCAD.flagLines = 0;
    option = 'rotate'; 
end


if uav.pCAD.flagCreated == 0
    uav.pPos.psiHel = 0;
    
    mCADmake(uav)
    mCADplot(uav)
    
else
    
    switch option
        
        case 'non-rotate'
            model = 1;
            uav.pCAD.i3D{1}.Visible = 'on';

            for idx = 2:6
                uav.pCAD.i3D{idx}.Visible = 'off';
                uav.pCAD.i3D{idx}.FaceAlpha = 0;
            end
            
        case 'rotate'
            model = 2;
            uav.pCAD.i3D{1}.Visible = 'off';

            for idx = 2:6
                uav.pCAD.i3D{idx}.Visible = 'on';
                uav.pCAD.i3D{idx}.FaceAlpha = 1.0;
            end
    end
    
    uav.pCAD.i3D{model}.FaceAlpha = 1;
    
    
    
    % Update robot pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot uav.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[uav.pCAD.obj{1}.v; ones(1,size(uav.pCAD.obj{1}.v,2))];
    uav.pCAD.i3D{1}.Vertices = vertices(1:3,:)';

    
    
    % ----- Drone de pás rotativas:
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot uav.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[uav.pCAD.obj{2}.v; ones(1,size(uav.pCAD.obj{2}.v,2))];
    uav.pCAD.i3D{2}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Frente Dir.)
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [0.11+0.005/3.5; 0.19+0.005/2.75; 0.779-0.75+0.01]; 0 0 0 1];
    
    
    Rot_J1 = [ cos(uav.pPos.psiHel) sin(uav.pPos.psiHel) 0;
              -sin(uav.pPos.psiHel) cos(uav.pPos.psiHel) 0;
                        0                    0           1];
            
    vertices = H*H1*[Rot_J1*uav.pCAD.obj{3}.v; ones(1,size(uav.pCAD.obj{3}.v,2))];
    uav.pCAD.i3D{3}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Tras Esq.)
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [-0.134+0.005/3.5; -0.20+0.005/2.25; 0.806-0.75+0.01]; 0 0 0 1];

                    
    vertices = H*H1*[Rot_J1*uav.pCAD.obj{3}.v; ones(1,size(uav.pCAD.obj{3}.v,2))];
    uav.pCAD.i3D{4}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Frente Esq.)
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [0.11+0.005/3.5; -0.20+0.005*1.75; 0.779-0.75+0.01]; 0 0 0 1];
    
    Rot_J1 = [ cos(-(uav.pPos.psiHel + pi/2)) sin(-(uav.pPos.psiHel + pi/2)) 0;
              -sin(-(uav.pPos.psiHel + pi/2)) cos(-(uav.pPos.psiHel + pi/2)) 0;
                              0                              0               1];
                          
    vertices = H*H1*[Rot_J1*uav.pCAD.obj{3}.v; ones(1,size(uav.pCAD.obj{3}.v,2))];
    uav.pCAD.i3D{5}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Tras Dir.)
    RotX = [1 0 0; 0 cos(uav.pPos.Xc(4)) -sin(uav.pPos.Xc(4)); 0 sin(uav.pPos.Xc(4)) cos(uav.pPos.Xc(4))];
    RotY = [cos(uav.pPos.Xc(5)) 0 sin(uav.pPos.Xc(5)); 0 1 0; -sin(uav.pPos.Xc(5)) 0 cos(uav.pPos.Xc(5))];
    RotZ = [cos(uav.pPos.Xc(6)) -sin(uav.pPos.Xc(6)) 0; sin(uav.pPos.Xc(6)) cos(uav.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [-0.134+0.005/3.5; 0.19+0.005*1.75; 0.806-0.75+0.01]; 0 0 0 1];
                        
    vertices = H*H1*[Rot_J1*uav.pCAD.obj{3}.v; ones(1,size(uav.pCAD.obj{3}.v,2))];
    uav.pCAD.i3D{6}.Vertices = vertices(1:3,:)';
   

end


uav.pPos.psiHel = uav.pPos.psiHel + (1 + (1.5-1)*rand(1))*2*pi/6;



end

% =========================================================================
function mCADmake(uav)

for i = 1:length(uav.pCAD.obj)
    
    hold on
    uav.pCAD.i3D{i} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    uav.pCAD.mtl{3}(2).Kd = [0.8 0.8 0.8]';
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end
    
    uav.pCAD.i3D{i}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{i}.FaceColor = 'flat';
    uav.pCAD.i3D{i}.EdgeColor = 'none';
    uav.pCAD.i3D{i}.FaceAlpha = 0.0;
    uav.pCAD.i3D{i}.Visible = 'off';

end


i = 3;
for iP = 4:6
    hold on
    uav.pCAD.i3D{iP} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
       
    uav.pCAD.i3D{iP}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{iP}.FaceColor = 'flat';
    uav.pCAD.i3D{iP}.EdgeColor = 'none';
    uav.pCAD.i3D{iP}.FaceAlpha = 0.0;
    uav.pCAD.i3D{iP}.Visible = 'off';
end
    
uav.pCAD.flagCreated = 1;

drawnow limitrate nocallbacks

end