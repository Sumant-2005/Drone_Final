function results = run_final()
% Run the 300 kg flight, then replay its front-right motor on the EMRAX drive.
root = fileparts(mfilename('fullpath'));
p = drone_parameters();
fprintf('Running %.0f kg flight mission and checking four rotor references.\n',p.mass_kg);
[~,replay] = run_drone('mission',1);
e = setup_emrax();
mdl = 'EMRAXMission';
file = fullfile(root,[mdl '.slx']);
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'FileName'),file),'Another EMRAXMission is open. Close it first.');
else
    load_system(file);
end
panel = [mdl '/Drive results'];
groups = {'Speed','Torque','Current'};
for k = 1:numel(groups)
    ports = get_param([panel '/' groups{k} ' samples'],'PortHandles');
    set_param(ports.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',groups{k});
end
open_system(mdl);
fprintf('Simulating the 15 kHz switching drive, including %.0f s of spin-up and settling.\n',e.mission_start);
out = sim(mdl,'ReturnWorkspaceOutputs','on');
log = out.get('logsout');
expectedTime = (0:e.flight_step:e.stop_time).';
speed = samples(log,'Speed',2,expectedTime);
torque = samples(log,'Torque',2,expectedTime);
current = samples(log,'Current',3,expectedTime);
time = expectedTime-e.mission_start;
first = round(e.mission_start/e.flight_step)+1;
mission = first:numel(time);
assert(max(abs(speed(mission,1)-replay.speed_ref_rpm))<1e-5 && ...
    max(abs(torque(mission,1)-replay.aero_load_Nm))<1e-6,'Drive inputs differ from the flight replay.');
results = table(time,speed(:,1),speed(:,2),torque(:,1),torque(:,2),current(:,1),current(:,2),current(:,3), ...
    'VariableNames',{'mission_s','command_rpm','shaft_rpm','load_Nm','electromagnetic_Nm','iq_ref_A','iq_A','id_A'});
folder = fullfile(root,'results','emrax');
if ~isfolder(folder), mkdir(folder); end
writetable(results,fullfile(folder,'mission_motor1_drive.csv'));
% Separate panels and physical units for the report, with t=0 at mission start.
data = {speed,torque,current};
units = {'Speed (rpm)','Torque (N m)','Current (A peak)'};
labels = {{'Command','Shaft speed'},{'Propeller load','Electromagnetic torque'}, ...
    {'q-axis command','q-axis actual','d-axis actual'}};
fig = figure('Name','Front-right EMRAX mission','Color','white','Position',[70 70 1024 760]);
layout = tiledlayout(fig,3,1);
for k = 1:numel(groups)
    ax = nexttile(layout);
    plot(ax,time(mission),data{k}(mission,:),'LineWidth',1);
    ylabel(ax,units{k}); grid(ax,'on'); xlim(ax,[0 time(end)]);
    legend(ax,labels{k},'Location','best');
end
xlabel(ax,'Mission time (s)');
title(layout,'EMRAX 228 HV | front-right motor');
exportgraphics(fig,fullfile(folder,'mission_motor1_drive.png'),'Resolution',300);
open_system(panel);
err = speed(:,2)-speed(:,1);
settled = first-round(0.2/e.flight_step):first-1;
rmsErr = sqrt(mean(err(mission).^2));
maxErr = max(abs(err(mission)));
fprintf('Mission RMS speed error %.2f rpm, maximum %.2f rpm.\n',rmsErr,maxErr);
fprintf('Peak sampled dq current %.2f A, peak iq request %.2f A (limit %.2f A peak).\n', ...
    max(hypot(current(mission,2),current(mission,3))),max(abs(current(mission,1))),e.current_limit);
fprintf('Saved mission_motor1_drive.csv and .png under results/emrax.\n');
% Tracking targets for this BTP replay. Keep failed traces for diagnosis.
check('Loaded hover speed error before replay (rpm)',max(abs(err(settled))),0.01*p.hover_speed*30/pi);
hoverLoad = mean(torque(settled,1));
check('Mean electromagnetic torque error at loaded hover (N m)', ...
    abs(mean(torque(settled,2))-hoverLoad),0.1*hoverLoad);
check('RMS mission speed error (rpm)',rmsErr,0.01*p.hover_speed*30/pi);
check('Peak sampled shaft speed (rpm)',max(abs(speed(mission,2))),e.speed_limit);
check('Peak q-axis request (A peak)',max(abs(current(:,1))),e.current_limit+1e-6);
end

function values = samples(log,key,width,t)
el = log.getElement(key);
v = el.Values;
assert(isa(v,'timeseries') && isequal(size(v.Time(:)),size(t)) && ...
    max(abs(v.Time(:)-t))<1e-7,'Missing or misaligned %s samples.',key);
a = v.Data;
if ~v.IsTimeFirst, a = permute(a,[ndims(a),1:ndims(a)-1]); end
values = reshape(a,numel(t),width);
assert(all(isfinite(values),'all'),'Nonfinite values in %s.',key);
end

function check(label,value,limit)
passed = value<=limit;
status = {'FAIL','PASS'};
fprintf('%s | limit %.4f | got %.4f | %s\n',label,limit,value,status{1+passed});
assert(passed,'Drive acceptance check failed: %s',label);
end
