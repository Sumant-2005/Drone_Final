function [out,replay] = run_drone(scenario,motor)
% Build/open the R2023a flight model, simulate, compare and save replay inputs.
if nargin < 1, scenario='demonstration'; end
if nargin < 2, motor=1; end
scenario=validatestring(scenario,{'demonstration','mission'});
validateattributes(motor,{'numeric'},{'scalar','real','integer','>=',1,'<=',4});
root=fileparts(mfilename('fullpath'));
mdl='DroneFlight';
file=fullfile(root,'generated',[mdl '.slx']);
if bdIsLoaded(mdl)
    opened=get_param(mdl,'FileName');
    assert(strcmp(opened,file),'A different DroneFlight model is open. Close it before running this project.');
elseif isfile(file)
    load_system(file);
else
    df_init(scenario);
    build_drone();
end
p=df_parameters();
profile=p.(scenario);
df_init(scenario);
% Leave the selected scenario active for the toolbar Run button, too.
set_param(mdl,'InitFcn',sprintf('df_init(''%s'');',scenario),'StopTime',num2str(profile(end,1)));
open_system(mdl);
out=sim(mdl,'ReturnWorkspaceOutputs','on');
ref=out.get('df_refLog'); measured=out.get('df_measureLog');
loads=out.get('df_loadLog'); force=out.get('df_thrustLog'); command=out.get('df_commandLog');
n=numel(ref.time);
assert(isequal(ref.time,measured.time,loads.time,force.time,command.time) && ...
    n==round(profile(end,1)/p.step_s)+1,'Flight logs have inconsistent timestamps.');
toMatrix=@(log) reshape(log.signals.values,n,[]);
w=toMatrix(ref); y=toMatrix(measured); T=toMatrix(force); u=toMatrix(loads);
assert(all(isfinite([w,y,T,u]),'all'),'Flight produced a nonfinite output.');
assert(max(abs(p.kT*w.^2-T),[],'all')<1e-7,'Thrust and speed references disagree.');
expectedTotal=min(max(u(:,1),0),4*p.max_thrust);
assert(max(abs(sum(T,2)-expectedTotal))<1e-7,'Mixer lost feasible collective.');
fixture=jsondecode(fileread(fullfile(root,'reference','expected.json')));
target=fixture.(scenario);
got=[y(end,3);y(end,4);min(y(:,2));max(y(:,2));y(end,2);max(y(:,1));max(w,[],'all')*30/pi];
want=[target.final_north_m;target.final_east_m;target.min_height_m;target.max_height_m; ...
    target.final_height_m;target.peak_body_forward_mps;target.max_ref_rpm];
tol=[0.01;0.01;0.01;0.01;0.01;0.01;0.5];
names={'Final north (m)','Final east (m)','Minimum height (m)','Maximum height (m)', ...
    'Final height (m)','Peak forward speed (m/s)','Peak speed reference (rpm)'};
for k=1:numel(got)
    passed=abs(got(k)-want(k))<=tol(k);
    verdict={'FAIL','PASS'};
    fprintf('%s: %.6f expected | %.6f got | %s\n',names{k},want(k),got(k),verdict{1+passed});
end
assert(all(abs(got-want)<=tol),'Flight response differs from the Python reference.');
% Columns 8:11 are actual rotor rpm and 12:15 are positive resisting loads.
rpm=w(:,motor)*30/pi;
q=y(:,11+motor);
replay=table(ref.time,w(:,motor),rpm,T(:,motor),y(:,7+motor),q, ...
    'VariableNames',{'time_s','speed_ref_rad_s','speed_ref_rpm','thrust_ref_N','flight_rotor_rpm','aero_load_Nm'});
speed_ref_rad_s=timeseries(w(:,motor),ref.time);
speed_ref_rpm=timeseries(rpm,ref.time);
aero_load_Nm=timeseries(q,ref.time);
folder=fullfile(root,'results');
if ~isfolder(folder), mkdir(folder); end
stem=fullfile(folder,sprintf('%s_motor%d',scenario,motor));
writetable(replay,[stem '.csv']);
save([stem '.mat'],'speed_ref_rad_s','speed_ref_rpm','aero_load_Nm','scenario','motor');
fprintf('Saved replay to results/%s_motor%d.csv and .mat\n',scenario,motor);
open_system([mdl '/Flight scopes']);
end
