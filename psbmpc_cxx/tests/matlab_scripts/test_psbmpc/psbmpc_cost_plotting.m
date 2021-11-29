legend_strs = strings(n_do + 4, 1);
legend_strs(1) = 'total cost';
legend_strs(2) = 'h colregs';
legend_strs(3) = 'h so'; 
legend_strs(4) = 'h path'; 
count = 5;
for i = 1 : n_do
    legend_strs(count) = ['cost do=' num2str(i)]; 
    count = count + 1;
end

figure(3); 
title('MPC Cost function');
hold on; grid on;
plot(total_cost, 'b');
plot(cost_colregs, 'k');
plot(cost_so_path(1, :), 'r');
plot(cost_so_path(2, :), 'g');
for i = 1 : n_do
    plot(cost_do(i, :));
end
legend(legend_strs, 'location', 'northeast');
ylabel('cost');
xlabel('cb index');

index = 1;
h_do_i_ps_legend_strs = cell(1, n_do);
for i = 1 : n_do
    h_do_i_ps_legend_strs{i} = strings(1, n_ps(i));
    for ps = 1 : n_ps(i)
        h_do_i_ps_legend_strs{i}(ps) = strcat('ps=', num2str(ps));
    end
    figure(i + 3);
    title(strcat('Obstacle i = ', num2str(i), ' | h do i ps'));
    grid on;
    plot(h_do_i_ps(index : index + n_ps(i) - 1, :)');
    legend(h_do_i_ps_legend_strs{i});
    ylabel('cost');
    xlabel('cb index');
    index = index + n_ps(i);
end

h_so_j_legend_strs = cell(1, n_so);
figure(n_do + 4);
title(strcat('Static obstacles max cost'));
grid on; hold on;
for j = 1 : n_so
    h_so_j_legend_strs{j} = strcat('j=', num2str(j));
    plot(h_so_j(j, :));
end
legend(h_so_j_legend_strs);
ylabel('cost');
xlabel('cb index');

% printf index of optimal control behaviour
fprintf('Optimum control behaviour index = %d\n', opt_cb_index);
fprintf('Corresponding control behaviour:\n');
disp(cb_matrix(:, opt_cb_index));

if is_gpu == 1
    save('gpu_psbmpc_cost_data', 'total_cost', 'cost_do', 'cost_colregs', 'n_so', 'h_do_i_ps', 'cost_so_path', ...
        'h_so_j', 'n_do', 'n_ps', 'cb_matrix', 'opt_cb_index');
else
    save('cpu_psbmpc_cost_data', 'total_cost', 'cost_do', 'cost_colregs', 'n_so', 'h_do_i_ps', 'cost_so_path', ...
        'h_so_j', 'n_do', 'n_ps', 'cb_matrix', 'opt_cb_index');
end
