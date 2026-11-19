function export_scope_results(captureWindows)
% Run both flight scenarios. Export all ten Scope traces and, on a desktop,
% try to capture their native Simulink windows for the report.
if nargin<1, captureWindows=usejava('desktop'); end
root=fileparts(mfilename('fullpath'));
motors={'M1 front right','M2 back left','M3 front left','M4 back right'};
for mode={'demonstration','mission'}
    scenario=mode{1};
    [~,~,s]=run_drone(scenario);
    y=s.measurements;
    % Columns below are the same signals, order and units wired to the scopes.
    scopes={
        'Flight response','Forward speed - m per s','Forward speed','m/s',[s.command(:,1) y(:,1)],{'Command','Response'};
        'Flight response','Height - m','Height','m',[s.command(:,4) y(:,2)],{'Target','Response'};
        'Flight response','North and east - m','North and east position','m',y(:,3:4),{'North','East'};
        'Flight response','Roll pitch yaw - deg','Roll, pitch and yaw','deg',y(:,5:7),{'Roll','Pitch','Yaw'};
        'Mixer inputs','Collective thrust - N','Collective thrust demand','N',s.requestedLoads(:,1),{'Collective'};
        'Mixer inputs','Roll pitch yaw moments - Nm','Body moment demands','N m',s.requestedLoads(:,2:4),{'Roll','Pitch','Yaw'};
        'Motor outputs','Four thrust references - N','Rotor thrust references','N',s.thrust,motors;
        'Motor outputs','Four speed references - rpm','Rotor speed references','rpm',s.speedRef*30/pi,motors;
        'Motor outputs','Actual rotor speeds - rpm','Simulated rotor speeds','rpm',y(:,8:11),motors;
        'Motor outputs','Propeller loads - Nm','Aerodynamic load torque','N m',y(:,12:15),motors};
    folder=fullfile(root,'results','scope_figures',scenario);
    if ~isfolder(folder), mkdir(folder); end
    captured=0;
    for k=1:size(scopes,1)
        file=sprintf('%02d_%s.png',k,matlab.lang.makeValidName(scopes{k,3}));
        draw_trace(fullfile(folder,file),scenario,s.time,scopes(k,3:6));
        if captureWindows
            path=strjoin({'QuadXFlight','Flight scopes',scopes{k,1},scopes{k,2}},'/');
            captured=captured+capture_scope(path,fullfile(folder,['window_' file]));
        end
    end
    fprintf('%s: %d Simulink trace figures saved; %d native Scope windows captured.\n',folder,size(scopes,1),captured);
end
end

function draw_trace(file,scenario,t,spec)
fig=figure('Visible','off','Color','white','Position',[50 50 1100 530]);
cleanup=onCleanup(@()close(fig)); %#ok<NASGU>
ax=axes('Parent',fig);
hold(ax,'on');
styles={'-','--','-.',':'};
for k=1:size(spec{3},2)
    plot(ax,t,spec{3}(:,k),'LineStyle',styles{k},'LineWidth',1.6);
end
grid(ax,'on'); box(ax,'off'); xlim(ax,[t(1) t(end)]);
xlabel(ax,'Time (s)'); ylabel(ax,spec{2});
title(ax,{spec{1},['Simulink flight model | ' scenario]});
legend(ax,spec{4},'Location','southoutside','Orientation','horizontal');
exportgraphics(fig,file,'Resolution',300);
end

function captured=capture_scope(path,file)
% Some releases render Scope in a standalone app without a figure handle.
% Only capture an identifiable figure for the requested Scope.
captured=false;
try
    open_system(path); drawnow;
    leaf=get_param(path,'Name');
    deadline=tic;
    while toc(deadline)<1.5
        figs=findall(groot,'Type','figure','Visible','on');
        hit=false(size(figs));
        for j=1:numel(figs)
            name=strtrim(get(figs(j),'Name'));
            hit(j)=any(strcmp(name,{leaf,path,['Scope: ' path],['Scope: ' leaf]}));
        end
        if nnz(hit)==1
            frame=getframe(figs(hit));
            imwrite(frame.cdata,file);
            captured=true;
            return
        end
        pause(.1); drawnow;
    end
    fprintf('Window unavailable for %s. Its logged trace was exported.\n',path);
catch problem
    fprintf('Could not capture %s (%s). Its logged trace was exported.\n',path,problem.message);
end
end
