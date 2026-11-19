function sourceTime = held_source_time(t)
% For a discrete From Workspace block with interpolation off, make sample k
% available between hits k-1 and k. At each scheduled hit the value is
% unambiguous even when the solver and workspace round the hit differently.
% Only the source lookup uses these times. Physical/logged times stay on t.
sourceTime = [t(1); t(1:end-1) + diff(t)/2];
end
