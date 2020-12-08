
% Create obstacle data structure
obs{i}.X_pred{end} = X_i_pred;
obs{i}.X = X_i;

save(['joint_pred_data'], 'obs', '-append');