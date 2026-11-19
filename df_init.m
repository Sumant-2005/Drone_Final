function df_init(scenario)
if nargin == 0, scenario = 'demonstration'; end
scenario = validatestring(scenario,{'demonstration','mission'});
p = df_parameters();
profile = p.(scenario);
t = (0:p.step_s:profile(end,1)).';
command = interp1(profile(:,1),profile(:,2:end),t,'previous');
height = p.initial_height_m + [0;cumsum(command(1:end-1,3))*p.step_s];
% Workspace time series share the flight solver grid. The auxiliary channels
% are right speed, climb rate, integrated height target and yaw rate.
assignin('base','df_forward',timeseries(command(:,1),t));
assignin('base','df_aux',timeseries([command(:,2:3),height,command(:,4)*pi/180],t));
assignin('base','df_p',p);
end
