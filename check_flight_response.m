function check_flight_response(root,scenario,p,time,cmd,x,T,w,y)
% Compare actual logged signals with the physical requirements and Python traces.
profile=p.(scenario);
reference=interp1(profile(:,1),profile(:,2:5),time,'previous');
height=p.initial_height_m+[0;cumsum(reference(1:end-1,3))*p.step_s];
reference=[reference(:,1:3),height,reference(:,4)*pi/180];
check('Command profile, maximum absolute error',max(abs(cmd-reference),[],'all'),0,1e-8);
instruments=[cos(x(:,9)).*x(:,4)+sin(x(:,9)).*x(:,5),-x(:,3),x(:,1:2), ...
    x(:,7:9)*180/pi,x(:,13:16)*30/pi,p.kQ*x(:,13:16).^2];
check('Scope channels, maximum conversion error',max(abs(y-instruments),[],'all'),0,1e-7);
assert(all(T>=-1e-8 & T<=p.max_thrust+1e-8,'all'),'Thrust exceeded the actuator range.');
if strcmp(scenario,'demonstration')
    hover=time<profile(2,1);
    check('Hover thrust error per motor (N)',max(abs(T(hover,:)-p.hover_thrust),[],'all'),0,1e-6);
    check('Hover altitude error (m)',max(abs(y(hover,2)-p.initial_height_m)),0,1e-6);
    start=round(profile(2,1)/p.step_s)+1;
    brake=round(profile(3,1)/p.step_s)+1;
    rear=[2 4]; front=[1 3];
    assert(all(T(start,rear)>T(start,front)),'Expected greater rear thrust to accelerate.');
    assert(all(T(brake,front)>T(brake,rear)),'Expected greater front thrust to brake.');
    check('Final speed after braking (m/s)',norm(x(end,4:6)),0,0.1);
else
    check('Mission return distance (m)',norm(x(end,1:3)-p.x0(1:3).'),0,0.5);
    % Document the achieved yaw even when the endpoint position check passes.
    turns=find(profile(1:end-1,5)~=0);
    for j=turns(:).'
        start=round(profile(j,1)/p.step_s)+1;
        release=round(profile(j+1,1)/p.step_s)+1;
        gapEnd=round(profile(min(j+2,size(profile,1)),1)/p.step_s)+1;
        target=profile(j,5)*(profile(j+1,1)-profile(j,1));
        turn=(x(release,9)-x(start,9))*180/pi;
        afterGap=(x(gapEnd,9)-x(start,9))*180/pi;
        fprintf('Yaw at %.1f s: command integral %.2f deg | at release %.2f deg | after gap %.2f deg\n', ...
            time(start),target,turn,afterGap);
    end
end
gold=jsondecode(fileread(fullfile(root,'reference','checkpoints.json')));
gold=gold.(scenario);
idx=round(gold.time_s(:)/p.step_s)+1;
check('Trace maximum position error (m)',max(abs(x(idx,1:3)-gold.position),[],'all'),0,0.01);
check('Trace maximum velocity error (m/s)',max(abs(x(idx,4:6)-gold.velocity),[],'all'),0,0.01);
check('Trace maximum attitude error (rad)',max(abs(x(idx,7:9)-gold.attitude),[],'all'),0,0.002);
check('Trace maximum motor thrust error (N)',max(abs(T(idx,:)-gold.thrust),[],'all'),0,0.25);
check('Trace maximum reference error (rpm)',max(abs(w(idx,:)*30/pi-gold.ref_rpm),[],'all'),0,1);
check('Trace maximum shaft speed error (rpm)',max(abs(y(idx,8:11)-gold.rotor_rpm),[],'all'),0,1);
end

function check(label,value,target,tol)
passed=abs(value-target)<=tol;
verdict={'FAIL','PASS'};
fprintf('%s: %.6g +/- %.6g expected | %.6g got | %s\n',label,target,tol,value,verdict{1+passed});
assert(passed,'Flight check failed: %s',label);
end
