function iControlVariables(uav)

% ANAFI Parrot
% ========================================================================
% Robot pose
uav.pPos.X    = zeros(12,1); % Current pose (point of control)
uav.pPos.Xa   = zeros(12,1); % Past pose

uav.pPos.Xc   = zeros(12,1); % Current pose (center of the robot)
uav.pPos.Xp   = zeros(12,1); % Current pose (computed by the robot)

uav.pPos.Xd   = zeros(12,1); % Desired pose
uav.pPos.Xda  = zeros(12,1); % Past desired pose

uav.pPos.Xr   = zeros(12,1); % Reference pose
uav.pPos.Xra  = zeros(12,1); % Past reference pose

% First time derivative 
uav.pPos.dX   = zeros(12,1); % Current pose
uav.pPos.dXd  = zeros(12,1); % Desired pose
uav.pPos.dXr  = zeros(12,1); % Reference pose

% Pose error
uav.pPos.Xtil = uav.pPos.Xd - uav.pPos.X; 

