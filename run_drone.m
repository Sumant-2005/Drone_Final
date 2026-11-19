function [out,replay,scopeData] = run_drone(scenario,motor)
% Build/open the R2023a flight model, simulate, compare and save replay inputs.
if nargin < 1, scenario='demonstration'; end
if nargin < 2, motor=1; end
scenario=validatestring(scenario,{'demonstration','mission'});
validateattributes(motor,{'numeric'},{'scalar','real','integer','>=',1,'<=',4});
root=fileparts(mfilename('fullpath'));
mdl='QuadXFlight';
file=fullfile(root,'generated',[mdl '.slx']);
if bdIsLoaded(mdl)
    opened=get_param(mdl,'FileName');
    assert(strcmp(opened,file),'A different QuadXFlight model is open. Close it before running this project.');
elseif isfile(file)
    load_system(file);
else
    setup_flight(scenario);
    build_drone();
end
p=drone_parameters();
profile=p.(scenario);
setup_flight(scenario);
% Record signals at existing block ports for comparisons and replay.
% All user-facing outputs go to ordinary Scopes.
paths={'Flight command','Flight controller','Motor mixer','Thrust to speed', ...
    'Drone dynamics','Flight scopes/Flight measurements'};
keys={'command','loads','thrust','speedRef','state','measurements'};
for k=1:numel(paths)
    ports=get_param([mdl '/' paths{k}],'PortHandles');
    set_param(ports.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',keys{k});
end
set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout');
% Leave the selected scenario active for the toolbar Run button, too.
set_param(mdl,'InitFcn',sprintf('setup_flight(''%s'');',scenario),'StopTime',num2str(profile(end,1)));
open_system(mdl);
out=sim(mdl,'ReturnWorkspaceOutputs','on');
open_system([mdl '/Flight scopes']);
signals=out.get('logsout');
time=(0:p.step_s:profile(end,1)).';
w=sampled(signals,'speedRef',time,4);
y=sampled(signals,'measurements',time,15);
T=sampled(signals,'thrust',time,4);
u=sampled(signals,'loads',time,4);
command=sampled(signals,'command',time,5);
state=sampled(signals,'state',time,16);
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
check_flight_response(root,scenario,p,time,command,state,T,w,y);
if nargout >= 3
    scopeData=struct('time',time,'command',command,'measurements',y, ...
        'requestedLoads',u,'thrust',T,'speedRef',w);
end
% Columns 8:11 are actual rotor rpm and 12:15 are positive resisting loads.
rpm=w(:,motor)*30/pi;
q=y(:,11+motor);
replay=table(time,w(:,motor),rpm,T(:,motor),y(:,7+motor),q, ...
    'VariableNames',{'time_s','speed_ref_rad_s','speed_ref_rpm','thrust_ref_N','flight_rotor_rpm','aero_load_Nm'});
speed_ref_rad_s=timeseries(w(:,motor),time);
speed_ref_rpm=timeseries(rpm,time);
aero_load_Nm=timeseries(q,time);
folder=fullfile(root,'results');
if ~isfolder(folder), mkdir(folder); end
stem=fullfile(folder,sprintf('%s_motor%d',scenario,motor));
writetable(replay,[stem '.csv']);
mass_kg=p.mass_kg;
save([stem '.mat'],'speed_ref_rad_s','speed_ref_rpm','aero_load_Nm','scenario','motor','mass_kg');
fprintf('Saved replay to results/%s_motor%d.csv and .mat\n',scenario,motor);
end

function values=sampled(dataset,name,time,width)
element=dataset.getElement(name);
v=element.Values;
assert(isa(v,'timeseries') && numel(v.Time)==numel(time) && ...
    all(abs(v.Time(:)-time)<1e-8),'Incorrect sample grid for %s.',name);
data=v.Data;
if ~v.IsTimeFirst
    data=permute(data,[ndims(data),1:ndims(data)-1]);
end
assert(numel(data)==width*numel(time),'Incorrect signal width for %s.',name);
values=reshape(data,numel(time),width);
end
