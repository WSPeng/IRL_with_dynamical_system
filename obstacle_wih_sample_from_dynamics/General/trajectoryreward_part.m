% Return the reward obtained along trajectory determined by given inputs,
% as well as the gradient of the reward with respect to those inputs.
function [val_,grad] = trajectoryreward_part(u, s, mdp_data, mdp, reward)

% Reshape.
u = reshape(u,size(u,1)/mdp_data.udims,mdp_data.udims);

% Compute states and state Hessian.
[states,A,B] = feval(strcat(mdp,'control'),mdp_data,s,u);

% Compute reward and its gradient.
% [val,drdu] = feval(strcat(reward.type,'evalreward'),reward,mdp_data,s,u,states,A,B,[]);

val = feval(strcat(reward.type,'evalreward'),reward,mdp_data,s,u,states,A,B,[]);

% Compute gradient of reward with respect to control.
% grad = -drdu(:);


% need subsampling here
subsample = 1;
if subsample
    T = length(states);
    % index = linspace(50, T-50, 50);
    % index = floor(index);
    if mdp_data.num_obs == 2
        logi_index = states(:,1) > 0.5 & states(:,1) < 8.5;
        val = val(logi_index);
    else
        logi_index = states(:,2) > 4.3;
        %states = states(index, :);
        % val = val(index);
        val = val(logi_index);
    end
    % val = randsample(val, 50); % no random sampling!!!
    index = linspace(1, length(val), 50);
    index = floor(index);
    val = val(index);
end


% compute sum of val according to states x coordinate
% move upwards to reduce some computation
% for i = 1:length(states)
%     if states(i,1)<2 || states(i,1)>8
%         val(i) = 0;
%     end
% end
val_ = -sum(val);

% val_ = 0;
% for i = 1:length(u)
%     if (states(i,1)<6.5) && (states(i,1)>3.5)
%         val_ = val_ - val(i);
%     end
% end

grad = 0;