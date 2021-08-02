function iParameters(uav)

uav.pPar.Model = 'ANAFIparrot'; % robot model

% Sample time
uav.pPar.Ts = 0.01;  % For numerical integration
uav.pPar.ti = tic;   % Flag time

% Dynamic Model Parameters 
uav.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

