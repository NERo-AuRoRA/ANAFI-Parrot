function mCADcolor(drone,color)
% Modify drone color

if nargin > 1
    
    drone.pCAD.mtl{1}(4).Kd = color';
    
    if length(drone.pCAD.obj) > 1
        drone.pCAD.mtl{2}(3).Kd = color';
        drone.pCAD.i3D{4}.FaceColor = [0.1 0.1 0.1]';
        drone.pCAD.i3D{6}.FaceColor = [0.1 0.1 0.1]';
    end
    
end


for i = 1:length(drone.pCAD.obj)
    
    fvcd3 = [];
    
    for ii = 1:length(drone.pCAD.obj{i}.umat3)
        mtlnum = drone.pCAD.obj{i}.umat3(ii);
        for jj=1:length(drone.pCAD.mtl{i})
            if strcmp(drone.pCAD.mtl{i}(jj).name,drone.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = drone.pCAD.mtl{i}(jj).Kd';
    end

    drone.pCAD.i3D{i}.FaceVertexCData  = fvcd3;
end
end

