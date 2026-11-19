function thrust = df_limit(raw,p)
% Retain common thrust; scale moment-producing differences to fit all rotors.
raw = raw(:);
mean_thrust = mean(raw);
common = min(max(mean_thrust,0),p.max_thrust);
delta = raw-mean_thrust;
scale = 1;
if max(delta)>0, scale=min(scale,(p.max_thrust-common)/max(delta)); end
if min(delta)<0, scale=min(scale,common/(-min(delta))); end
thrust = min(max(common+scale*delta,0),p.max_thrust);
end
