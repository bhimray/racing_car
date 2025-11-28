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

SOURCES = [ 'acados_sim_solver_sfunction_racecar_lap_time_mpc.c ', ...
            'acados_sim_solver_racecar_lap_time_mpc.c ', ...
            'racecar_lap_time_mpc_model/racecar_lap_time_mpc_expl_ode_fun.c ', ...
            'racecar_lap_time_mpc_model/racecar_lap_time_mpc_expl_vde_forw.c ', ...
            'racecar_lap_time_mpc_model/racecar_lap_time_mpc_expl_vde_adj.c ', ...
];

INC_PATH = 'C:\Users\rayyu\acados\examples\acados_matlab_octave\..\../include';

INCS = [ ' -I', fullfile(INC_PATH, 'blasfeo', 'include'), ...
         ' -I', fullfile(INC_PATH, 'hpipm', 'include'), ...
        ' -I', INC_PATH, ' -I', fullfile(INC_PATH, 'acados'), ' '];

CFLAGS  = ' -O';

LIB_PATH = 'C:\Users\rayyu\acados\examples\acados_matlab_octave\..\../lib';

LIBS = '-lacados -lhpipm -lblasfeo';

try
    % eval( [ 'mex -v -output  acados_sim_solver_sfunction_racecar_lap_time_mpc ', ...
    eval( [ 'mex -output  acados_sim_solver_sfunction_racecar_lap_time_mpc ', ...
        CFLAGS, INCS, ' ', SOURCES, ' -L', LIB_PATH, ' ', LIBS ]);

catch exception
    disp('make_sfun_sim failed with the following exception:')
    disp(exception);
    disp(exception.message);
    disp('Try adding -v to the mex command above to get more information.')
    keyboard
end


fprintf( [ '\n\nSuccessfully created sfunction:\nacados_sim_solver_sfunction_racecar_lap_time_mpc', '.', ...
    eval('mexext')] );


global sfun_sim_input_names
sfun_sim_input_names = {};

%% print note on usage of s-function
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n1) x0, initial state, size [7]\n ';
i_in = 2;
sfun_sim_input_names = [sfun_sim_input_names; 'x0 [7]'];
input_note = strcat(input_note, num2str(i_in), ') u, size [2]\n ');
i_in = i_in + 1;
sfun_sim_input_names = [sfun_sim_input_names; 'u [2]'];
input_note = strcat(input_note, num2str(i_in), ') parameters, size [1]\n ');
i_in = i_in + 1;
sfun_sim_input_names = [sfun_sim_input_names; 'p [1]'];


fprintf(input_note)

disp(' ')

global sfun_sim_output_names
sfun_sim_output_names = {};

output_note = strcat('Outputs are:\n', ...
                '1) x1 - simulated state, size [7]\n');
sfun_sim_output_names = [sfun_sim_output_names; 'x1 [7]'];

fprintf(output_note)


% create the Simulink block for the integrator
modelName = 'racecar_lap_time_mpc_sim_solver_simulink_block';
new_system(modelName);
open_system(modelName);

blockPath = [modelName '/racecar_lap_time_mpc_sim_solver'];
add_block('simulink/User-Defined Functions/S-Function', blockPath);
set_param(blockPath, 'FunctionName', 'acados_sim_solver_sfunction_racecar_lap_time_mpc');

Simulink.Mask.create(blockPath);
mask_str = sprintf([ ...
    'global sfun_sim_input_names sfun_sim_output_names\n' ...
    'for i = 1:length(sfun_sim_input_names)\n' ...
    '    port_label(''input'', i, sfun_sim_input_names{i})\n' ...
    'end\n' ...
    'for i = 1:length(sfun_sim_output_names)\n' ...
    '    port_label(''output'', i, sfun_sim_output_names{i})\n' ...
    'end\n' ...
    'disp("acados sim")' ...
]);
mask = Simulink.Mask.get(blockPath);
mask.Display = mask_str;

save_system(modelName);
close_system(modelName);
disp([newline, 'Created the sim solver Simulink block in: ', modelName])


% The mask drawing command is:
% ---
% global sfun_sim_input_names sfun_sim_output_names
% for i = 1:length(sfun_sim_input_names)
%     port_label('input', i, sfun_sim_input_names{i})
% end
% for i = 1:length(sfun_sim_output_names)
%     port_label('output', i, sfun_sim_output_names{i})
% end
% ---
% It can be used by copying it in sfunction/Mask/Edit mask/Icon drawing commands
%   (you can access it with ctrl+M on the s-function)
