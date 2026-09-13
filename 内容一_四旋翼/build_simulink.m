function build_simulink()
% 生成 Quadrotor_Control.slx：轨迹/控制/分配在 Ctrl，动力学在 Plant

this_dir = fileparts(mfilename('fullpath'));
cd(this_dir);
addpath(this_dir);

model = 'Quadrotor_Control';
if bdIsLoaded(model)
    close_system(model, 0);
end
if exist([model '.slx'], 'file')
    delete([model '.slx']);
end

par = slx_config();
new_system(model);
open_system(model);

add_block('simulink/Sources/Clock', [model '/Clock'], ...
    'Position', [30 80 60 110]);
add_block('simulink/Signal Routing/Mux', [model '/MuxCtrl'], ...
    'Inputs', '2', 'Position', [140 70 145 140]);
add_block('simulink/Signal Routing/Mux', [model '/MuxPlant'], ...
    'Inputs', '2', 'Position', [520 90 525 160]);

ctrl_blk = add_interp_fcn(model, 'Ctrl', 'slx_ctrl', 10, [200 80 320 140]);
plant_blk = add_interp_fcn(model, 'Plant', 'slx_plant', 12, [560 100 680 160]);

add_block('simulink/Continuous/Integrator', [model '/Integrator'], ...
    'InitialCondition', mat2str(par.x0), ...
    'Position', [740 105 780 155]);

add_block('simulink/Signal Routing/Selector', [model '/SelU'], ...
    'IndexOptionArray', {'Index vector (dialog)'}, ...
    'IndexParamArray', {'1:4'}, ...
    'InputPortWidth', '10', ...
    'Position', [360 60 410 90]);
add_block('simulink/Signal Routing/Selector', [model '/SelAtt'], ...
    'IndexOptionArray', {'Index vector (dialog)'}, ...
    'IndexParamArray', {'5:7'}, ...
    'InputPortWidth', '10', ...
    'Position', [360 110 410 140]);
add_block('simulink/Signal Routing/Selector', [model '/SelRef'], ...
    'IndexOptionArray', {'Index vector (dialog)'}, ...
    'IndexParamArray', {'8:10'}, ...
    'InputPortWidth', '10', ...
    'Position', [360 160 410 190]);

add_block('simulink/Sinks/Scope', [model '/Scope_x'], 'Position', [860 40 890 70]);
add_block('simulink/Sinks/Scope', [model '/Scope_U'], 'Position', [460 30 490 60]);
add_tw(model, 'x_log', [860 110 920 130]);
add_tw(model, 'U_log', [460 200 520 220]);
add_tw(model, 'ref_log', [460 230 530 250]);
add_tw(model, 'attd_log', [460 260 540 280]);

add_line(model, 'Clock/1', 'MuxCtrl/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'MuxCtrl/2', 'autorouting', 'on');
add_line(model, 'MuxCtrl/1', [ctrl_blk '/1'], 'autorouting', 'on');
add_line(model, [ctrl_blk '/1'], 'SelU/1', 'autorouting', 'on');
add_line(model, [ctrl_blk '/1'], 'SelAtt/1', 'autorouting', 'on');
add_line(model, [ctrl_blk '/1'], 'SelRef/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'MuxPlant/1', 'autorouting', 'on');
add_line(model, 'SelU/1', 'MuxPlant/2', 'autorouting', 'on');
add_line(model, 'MuxPlant/1', [plant_blk '/1'], 'autorouting', 'on');
add_line(model, [plant_blk '/1'], 'Integrator/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'Scope_x/1', 'autorouting', 'on');
add_line(model, 'Integrator/1', 'x_log/1', 'autorouting', 'on');
add_line(model, 'SelU/1', 'Scope_U/1', 'autorouting', 'on');
add_line(model, 'SelU/1', 'U_log/1', 'autorouting', 'on');
add_line(model, 'SelRef/1', 'ref_log/1', 'autorouting', 'on');
add_line(model, 'SelAtt/1', 'attd_log/1', 'autorouting', 'on');

setup_cb = [ ...
    'fp=get_param(bdroot,''Filename'');', ...
    'p=fileparts(fp);', ...
    'if ~isempty(p), addpath(p); end'];
set_param(model, 'InitFcn', setup_cb);
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'ode4');
set_param(model, 'FixedStep', '0.001');
set_param(model, 'StopTime', num2str(par.traj.t_end));
set_param(model, 'SaveFormat', 'Dataset');

save_system(model, fullfile(this_dir, [model '.slx']));
fprintf('已生成 %s.slx\n', model);
end

function name = add_interp_fcn(model, name, fcn, nout, pos)
blk = [model '/' name];
try
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
        blk, 'Position', pos);
    set_param(blk, 'MATLABFcn', fcn);
    try
        set_param(blk, 'OutputDimensions', num2str(nout));
    catch
        set_param(blk, 'OutputWidth', num2str(nout));
    end
catch
    add_block('simulink/User-Defined Functions/MATLAB Fcn', ...
        blk, 'Position', pos);
    set_param(blk, 'MATLABFcn', fcn);
    set_param(blk, 'OutputWidth', num2str(nout));
end
end

function add_tw(model, varname, pos)
blk = [model '/' varname];
add_block('simulink/Sinks/To Workspace', blk, 'Position', pos);
set_param(blk, 'VariableName', varname);
try
    set_param(blk, 'SaveFormat', 'Timeseries');
catch
end
end
