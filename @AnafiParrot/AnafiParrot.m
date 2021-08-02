classdef AnafiParrot < handle
    % In a methods block, set the method attributes
    % and add the function signature
    
    properties
        
        % Properties or Parameters
        pCAD   % ANAFI 3D image
        pPar   % Parameters
        pID    % Identification
        
        % Control variables
        pPos   % Posture
        pFlag  % Flags
        

    end
    
    methods
        function uav = AnafiParrot(ID)
            if nargin < 1
                ID = 1;
            end
            uav.pID = ID;
                
            iControlVariables(uav);
            iParameters(uav);
            mCADload(uav);
      
        end
        
        % ==================================================
        iControlVariables(uav);
        iParameters(uav);
        
        % ==================================================
        % ANAFI Parrot 3D Image
        mCADload(uav);
        mCADplot(uav,opt);
        mCADcolor(uav,color);

      
    end
end