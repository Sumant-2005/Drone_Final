function df_init(varargin)
% Previously saved models still request these workspace names in InitFcn.
[p,forward,other] = setup_flight(varargin{:});
assignin('base','df_p',p);
assignin('base','df_forward',forward);
assignin('base','df_aux',other);
end
