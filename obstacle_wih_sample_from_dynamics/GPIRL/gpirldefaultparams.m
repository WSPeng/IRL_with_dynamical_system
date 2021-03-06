% Fill in default parameters for the GPIRL algorithm.
function algorithm_params = gpirldefaultparams(algorithm_params)

% Create default parameters.
default_params = struct(...
    'seed',2,...
    'restarts',1,...
    'ame_init',0,... % originally 0
    'ame_params',struct(),...
    ...% Which parameters to learn.
    'learn_ard',1,...
    'learn_noise',0,...
    'learn_rbf',1,...
    ...% Parameters for random restarts.
    'restart_tolerance',1e1,...
    'initial_rewards',5,...
    ...% State sample parameters.
    'sample_mode','data',...  % or can be random 
    'samples',256,...
    ...% These are hyperparameter transformations for positivity constraints.
    'ard_xform','exp',...
    'noise_xform','exp',...
    'rbf_xform','exp',...
    ...% These are hyperparameter priors.
    'ard_prior','logsparsity',...
    'noise_prior','g0',...
    'rbf_prior','none',...
    ...% These are prior wights and parameters.
    'ard_prior_wt',1e-1,...
    'noise_prior_wt',1,...
    'rbf_prior_wt',1,...
    'gamma_shape',2,...
    ...% These are initial values.
    'ard_init',0.65,...
    'noise_init',1e-2,...
    'rbf_init',5);

% Set parameters.
algorithm_params = filldefaultparams(algorithm_params,default_params);
