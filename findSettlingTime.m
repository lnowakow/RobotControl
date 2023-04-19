function [settling_time, max_disp] = findSettlingTime(waveform,time)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

% Final settling value
f_val = waveform(end);
min_val = min(waveform);
max_disp = abs(f_val-min_val);

% Thresholds +/-5%
thresh_l = f_val-0.05*abs(f_val-min_val);
thresh_u = f_val+0.05*abs(f_val-min_val);

% flip waveform to find last crossing
flipped_wave = flip(waveform);
temp_mat = repmat(1:size(flipped_wave), 1)';
idxl = temp_mat .* (flipped_wave<=thresh_l);
idxu = temp_mat .* (flipped_wave>=thresh_u);

idxl(idxl == 0) = NaN;
idxu(idxu == 0) = NaN;

first_l = min(idxl);
first_u = min(idxu);

if any(isnan(first_l)) && any(isnan(first_u))
    last_cross = NaN;
elseif any(isnan(first_u)) && ~any(isnan(first_l))
    last_cross = first_l;
elseif any(isnan(first_l)) && ~any(isnan(first_u))
    last_cross = first_u;
elseif first_l < first_u
    last_cross = first_l;
else
    last_cross = first_u;
end

t_start = time(1);

if ~isnan(last_cross)
    t_end = time(end-last_cross);
    settling_time = t_end - t_start;
else
    settling_time = NaN;
end

end