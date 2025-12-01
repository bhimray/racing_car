%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;

%




  
  
  
  
  
  
  
  
  
  
  
  
  




SOURCES = { ...
            'racecar_lap_time_mpc_model/racecar_lap_time_mpc_expl_ode_fun.c', ...
            'racecar_lap_time_mpc_model/racecar_lap_time_mpc_expl_vde_forw.c', ...
            'racecar_lap_time_mpc_model/racecar_lap_time_mpc_expl_vde_adj.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_0_fun.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_0_fun_jac.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_0_fun_jac_hess.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_fun.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_fun_jac.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_fun_jac_hess.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_e_fun.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_e_fun_jac.c', ...
            'racecar_lap_time_mpc_cost/racecar_lap_time_mpc_cost_ext_cost_e_fun_jac_hess.c', ...
            'acados_solver_sfunction_racecar_lap_time_mpc.c', ...
            'acados_solver_racecar_lap_time_mpc.c'
          };

INC_PATH = 'C:\Users\rayyu\acados\interfaces\acados_matlab_octave\..\../include';

INCS = {['-I', fullfile(INC_PATH, 'blasfeo', 'include')], ...
        ['-I', fullfile(INC_PATH, 'hpipm', 'include')], ...
        ['-I', fullfile(INC_PATH, 'acados')], ...
        ['-I', fullfile(INC_PATH)]};



CFLAGS = 'CFLAGS=$CFLAGS';
LDFLAGS = 'LDFLAGS=$LDFLAGS';
COMPFLAGS = 'COMPFLAGS=$COMPFLAGS';
COMPDEFINES = 'COMPDEFINES=$COMPDEFINES';



LIB_PATH = ['-L', fullfile('C:\Users\rayyu\acados\interfaces\acados_matlab_octave\..\../lib')];

LIBS = {'-lacados', '-lhpipm', '-lblasfeo'};

% acados linking libraries and flags
LDFLAGS = [LDFLAGS ' '];
COMPFLAGS = [COMPFLAGS ' '];
LIBS{end+1} = '';
LIBS{end+1} = '';
LIBS{end+1} = '';

COMPFLAGS = [COMPFLAGS ' -O2'];
CFLAGS = [CFLAGS ' -O2'];

try
    %     mex('-v', '-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
    mex('-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
            LIB_PATH, LIBS{:}, SOURCES{:}, ...
            '-output', 'acados_solver_sfunction_racecar_lap_time_mpc' );
catch exception
    disp('make_sfun failed with the following exception:')
    disp(exception);
    disp(exception.message );
    disp('Try adding -v to the mex command above to get more information.')
    keyboard
end

fprintf( [ '\n\nSuccessfully created sfunction:\nacados_solver_sfunction_racecar_lap_time_mpc', '.', ...
    eval('mexext')] );


%% print note on usage of s-function, and create I/O port names vectors
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n';
i_in = 1;

global sfun_input_names
sfun_input_names = {};
input_note = strcat(input_note, num2str(i_in), ') lbx_0 - lower bound on x for stage 0,',...
                    ' size [7]\n ');
sfun_input_names = [sfun_input_names; 'lbx_0 [7]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx_0 - upper bound on x for stage 0,',...
                    ' size [7]\n ');
sfun_input_names = [sfun_input_names; 'ubx_0 [7]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') parameters - concatenated for all stages 0 to N,',...
                    ' size [357]\n ');
sfun_input_names = [sfun_input_names; 'parameter_traj [357]'];
i_in = i_in + 1;



input_note = strcat(input_note, num2str(i_in), ') lbx values concatenated for stages 1 to N-1, size [49]\n ');
sfun_input_names = [sfun_input_names; 'lbx [49]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx values concatenated for stages 1 to N-1, size [49]\n ');
sfun_input_names = [sfun_input_names; 'ubx [49]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lbu for stages 0 to N-1, size [50]\n ');
sfun_input_names = [sfun_input_names; 'lbu [50]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubu for stages 0 to N-1, size [50]\n ');
sfun_input_names = [sfun_input_names; 'ubu [50]'];
i_in = i_in + 1;


fprintf(input_note)

disp(' ')

output_note = 'Outputs are:\n';
i_out = 0;

global sfun_output_names
sfun_output_names = {};
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') u0, control input at node 0, size [2]\n ');
sfun_output_names = [sfun_output_names; 'u0 [2]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') acados solver status (0 = SUCCESS)\n ');
sfun_output_names = [sfun_output_names; 'solver_status'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') KKT residual\n ');
sfun_output_names = [sfun_output_names; 'KKT_residual'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') x1, state at node 1\n ');
sfun_output_names = [sfun_output_names; 'x1 [7]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') CPU time\n ');
sfun_output_names = [sfun_output_names; 'CPU_time'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') SQP iterations\n ');
sfun_output_names = [sfun_output_names; 'sqp_iter'];

fprintf(output_note)
modelName = 'racecar_lap_time_mpc_ocp_solver_simulink_block';
new_system(modelName);
open_system(modelName);

blockPath = [modelName '/racecar_lap_time_mpc_ocp_solver'];
add_block('simulink/User-Defined Functions/S-Function', blockPath);
set_param(blockPath, 'FunctionName', 'acados_solver_sfunction_racecar_lap_time_mpc');

Simulink.Mask.create(blockPath);
mask_str = sprintf([ ...
    'global sfun_input_names sfun_output_names\n' ...
    'for i = 1:length(sfun_input_names)\n' ...
    '    port_label(''input'', i, sfun_input_names{i})\n' ...
    'end\n' ...
    'for i = 1:length(sfun_output_names)\n' ...
    '    port_label(''output'', i, sfun_output_names{i})\n' ...
    'end\n' ...
    'disp("acados OCP")' ...
]);
mask = Simulink.Mask.get(blockPath);
mask.Display = mask_str;

save_system(modelName);
close_system(modelName);
disp([newline, 'Created the OCP solver Simulink block in: ', modelName])

% The mask drawing command is:
% ---
% global sfun_input_names sfun_output_names
% for i = 1:length(sfun_input_names)
%     port_label('input', i, sfun_input_names{i})
% end
% for i = 1:length(sfun_output_names)
%     port_label('output', i, sfun_output_names{i})
% end
% ---
% It can be used by copying it in sfunction/Mask/Edit mask/Icon drawing commands
%   (you can access it with ctrl+M on the s-function)
