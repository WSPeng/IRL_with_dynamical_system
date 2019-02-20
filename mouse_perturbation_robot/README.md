# mouse_perturbation_robot

# process explained
	if the mouse is used "if(_mouseInUse or currentTime - _lastMouseTime < _commandLagDuration)", then this trajectory is successed (for now), when the distance is smaller than a tolerance, then it means we arrive the target. -> label sucess and update parameter.

	if the mouse is loose, then failed. wait untill the robot goes back the original position, then label it as failed and update the parameter