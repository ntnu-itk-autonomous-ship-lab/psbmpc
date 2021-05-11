legend_strs = strings(n_obst + 3, 1);
legend_strs(1) = 'total cost';
legend_strs(2) = 'h so'; 
legend_strs(3) = 'h path'; 
count = 4;
for i = 1 : n_obst
    legend_strs(count) = ['cost do=' num2str(i)]; 
    count = count + 1;
end

figure(1); 
hold on; grid on;
plot(total_cost, 'b');
plot(cost_so_path(1, :), 'r');
plot(cost_so_path(2, :), 'g');
for i = 1 : n_obst
    plot(cost_do(i, :));
end
legend(legend_strs, 'location', 'northeast');
ylabel('cost');
xlabel('cb index');

index = 1;
max_cost_legend_strs = cell(1, n_obst);
for i = 1 : n_obst
    max_cost_legend_strs{i} = strings(1, n_ps(i));
    for ps = 1 : n_ps(i)
        max_cost_legend_strs{i}(ps) = strcat('ps=', num2str(ps));
    end
    figure(i + 1);
    title(strcat('Obstacle i = ', num2str(i), ' | Max cost ps'));
    grid on;
    plot(max_cost_i_ps(index : index + n_ps(i) - 1, :)');
    legend(max_cost_legend_strs{i});
    ylabel('cost');
    xlabel('cb index');
    index = index + n_ps(i);
end

% printf index of optimal control behaviour
fprintf('Optimum control behaviour index = %d\n', opt_cb_index);
fprintf('Corresponding control behaviour:\n');
disp(cb_matrix(opt_cb_index));

save('gpu_psbmpc_cost_data', 'total_cost', 'cost_do', 'max_cost_ps', 'cost_so_path', ...
    'n_obst', 'n_ps', 'cb_matrix', 'opt_cb_index');