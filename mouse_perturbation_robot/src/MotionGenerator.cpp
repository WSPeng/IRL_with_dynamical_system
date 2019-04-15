#include "MotionGenerator.h"
#include <signal.h>
#include <fcntl.h>
#include <termios.h>

MotionGenerator* MotionGenerator::me = NULL;

MotionGenerator::MotionGenerator(ros::NodeHandle &n, double frequency): 
	_n(n), 
	_loopRate(frequency), 
	_dt(1 / frequency)
{
	me = this;
	ROS_INFO_STREAM("The motion generator node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz");

	// if the delay is introduced
	_delayIntroduce = 0;
	_delayInterval = 100; //100*2 = 200ms

	// 1 or 2 for obstacle number
	_numObstacle = 1;

	// to configer using in my PC or in the kuka lwr PC (the MouseInterface node is not working with kuka lwr PC.)
	_boolSpacenav = 0; // in my PC, do not use the spacenav

	// Recieve obstacle&target position from outside node OR use the position predefined.
	_obsPositionInput = 0;
	_recievedObsPositionInput = false;
	_recievedTarPositionInput = false;

	init_sf = 0.9f;
	init_rho = 0.9f;

	//obstacle definition
	_obs._a << 0.5f, 0.1f, 0.12f; // 0.5 0.1 0.12       0.15f, 0.10f, 0.5f
	_obs._p.setConstant(1.0f);
	_obs._safetyFactor = init_sf;// was 1.1
	_obs._tailEffect = true;
	_obs._bContour = false;
	_obs._rho = init_rho;// was 1.1

	//std::cerr << "ons ?" << _numObstacle << std::endl;

	_ifsendArduino = 0;
	_useArduino = false;

	// random generate rho and sf at each end of trails
	_randomInsteadIRL = false;

	// if use iiwa instead of the lwr
	_iiwaInsteadLwr = true;

	if (_numObstacle == 2)
	{
		//obstacle definition 1 
		//_obs._a << 0.03f, 0.02f, 0.5f; // 0.5 0.1 0.12
		_obs._a << 0.5f, 0.1f, 0.12f; // 0.12f, 0.1f, 0.5f
		_obs._p.setConstant(1.0f);
		_obs._safetyFactor = 1.0f;// was 1.1 // may 0.9f
		_obs._tailEffect = false;
		_obs._bContour = false;
		_obs._rho = 1.0f;// was 1.1 // may 0.9f

		//obstacle definition 2The motion generator is ready.The motion generator is ready.The motion generator is ready.
		//_obs2._a << 0.20f, 0.07f, 0.5f; // 0.5 0.1 0.12
		_obs2._a << 0.5f, 0.1f, 0.12f; //0.15f, 0.1f, 0.5f
		_obs2._p.setConstant(1.0f);
		_obs2._safetyFactor = 1.0f;// was 1.1
		_obs2._tailEffect = false;
		_obs2._bContour = false;
		_obs2._rho = 1.0f;// was 1.1
	}
}


bool MotionGenerator::init() 
{
	// Variable initialization
  	_wRb.setConstant(0.0f); 
  	_x.setConstant(0.0f); // end effector position
  	_q.setConstant(0.0f); // end effector direction
  	_x0.setConstant(0.0f); // target position

  	_qd.setConstant(0.0f); // target end effector direction during motion
  	_omegad.setConstant(0.0f); //angular velocity
  	_xd.setConstant(0.0f);
  	_vd.setConstant(0.0f);

  	_xp.setConstant(0.0f);
  	_mouseVelocity.setConstant(0.0f);
  	_targetOffset.col(Target::A) << 0.0f, 0.0f, 0.0f;
  	// The target B is the target
  	if (!_obsPositionInput)
  		_targetOffset.col(Target::B) << 0.0f, 0.85f, 0.0f; // 0.0f, 0.85f, 0.0f;
  	else
  		_targetOffset.col(Target::B) << 0.0f, 0.85f, 0.0f;
  	if (_iiwaInsteadLwr)
  		_targetOffset.col(Target::B) << 0.0f, -0.85f, 0.0f;

  	_targetOffset.col(Target::C) << -0.16f,0.25f,0.0f;
  	_targetOffset.col(Target::D) << -0.16f,-0.25f,0.0f;
  	_perturbationOffset.setConstant(0.0f);
  	_phaseDuration = 0.0f;
  	_minCleanMotionDuration = 5.0f;
  	_maxCleanMotionDuration = 12.0f;
  	_jerkyMotionDuration = 0.4f;
  	_initDuration = 10.0f;
  	_pauseDuration = 0.4f;
  	_commandLagDuration = 0.3f;
  	_reachedTime = 0.0f;
  	_trialCount = 0;
  	_perturbationCount = 0;
  	_lastMouseEvent = mouse_perturbation_robot::MouseMsg::M_NONE;
  	_errorButtonCounter = 0;
  	_eventLogger = 0;

  	_indexx = 0;
  	_indexy = 0;
  	_ifSentTraj = false;

	_firstRealPoseReceived = false;
	_firstMouseEventReceived = false;
	_stop = false;
	_perturbation = false;
	_mouseControlledMotion = true;
	_mouseInUse = false;
	_perturbationFlag = false;
	_switchingTrajectories = false;  //to be configured in dynamic reconfigure GUI
	_errorButtonPressed = false;
	_firstSpacenavDataReceived = false;
	_numOfDemo = 0;

	_state = State::INIT;
	//_previousTarget = Target::A;
	//_currentTarget = Target::B;
	_previousTarget = Target::B;
	_currentTarget = Target::A;

	Eigen::Vector3f temp;
	temp << 0.0f,0.0f,1.0f;
	_motionDirection = _targetOffset.col(_currentTarget)-_targetOffset.col(_previousTarget);
	_motionDirection.normalize();
	_perturbationDirection = temp.cross(_motionDirection);
	_perturbationDirection.normalize();

	_msg_para_up.data = 1.0f;
	_msgCommand.layout.dim.resize(1);
	_msgCommand.layout.data_offset = 0;
	_msgCommand.layout.dim[0].size = 10;
	_msgCommand.layout.dim[0].stride = 0;
	_msgCommand.data.resize(10, 0.);
	// resize(n, std_msgs::MultiArrayDimension())
	_numOfDemoCounter = 0;

	// Subscriber definitions
	_subMouse = _n.subscribe("/mouse", 1, &MotionGenerator::updateMouseData, this, ros::TransportHints().reliable().tcpNoDelay());
	if (!_iiwaInsteadLwr)
	{
		_subRealPose = _n.subscribe("/lwr/ee_pose", 1, &MotionGenerator::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
		_subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &MotionGenerator::updateRealTwist, this, ros::TransportHints().reliable().tcpNoDelay());	
	}
	else
	{
		_subRealPose = _n.subscribe("/iiwa/ee_pose", 1, &MotionGenerator::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
		_subRealTwist = _n.subscribe("/iiwa/ee_vel", 1, &MotionGenerator::updateRealTwist, this, ros::TransportHints().reliable().tcpNoDelay());			
	}
	
	if(_boolSpacenav)
		_subSpaceNav = _n.subscribe("/spacenav/joy", 1, &MotionGenerator::updateSpacenavData, this, ros::TransportHints().reliable().tcpNoDelay());
	
	_subIRL = _n.subscribe("/parameters_tuning", 1, &MotionGenerator::updateIRLParameter, this, ros::TransportHints().reliable().tcpNoDelay());
	if(_obsPositionInput)
	{
		_subPositionObs = _n.subscribe("/position_post/obstacle_position", 1, &MotionGenerator::subPositionObs, this, ros::TransportHints().reliable().tcpNoDelay());
		_subPositionTar = _n.subscribe("/position_post/target_position", 1, &MotionGenerator::subPositionTar, this, ros::TransportHints().reliable().tcpNoDelay());
	}
	_subMessageEEG = _n.subscribe("/eeg", 1, &MotionGenerator::subMessageEEG, this, ros::TransportHints().reliable().tcpNoDelay());

	// Publisher definitions
	if (!_iiwaInsteadLwr)
	{
		_pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
		_pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);		
	}
	else
	{
		_pubCommand = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/DSImpedance/command", 1);
	}

	//_pubFeedBackToParameter = _n.advertise<std_msgs::Float32>("/motion_generator_to_parameter_update", 1);
	_pubFeedBackToParameter = _n.advertise<geometry_msgs::PoseArray>("/motion_generator_to_parameter_update", 1);
	if(_delayIntroduce)
		_pubMouseMsgIRL = _n.advertise<mouse_perturbation_robot::MouseMsgPassIRL>("/mouse_message_update_to_irl", 1);
	_pubTarPosition = _n.advertise<geometry_msgs::Pose>("/send_position_target_marker", 1);
	_pubObsPosition = _n.advertise<geometry_msgs::Pose>("/send_position_obstacle_marker", 1);

	// Dynamic reconfigure definition
	_dynRecCallback = boost::bind(&MotionGenerator::dynamicReconfigureCallback, this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);

	// Open file to save data
	_outputFile.open ("/home/jason/catkin_ws/src/mouse_perturbation_robot/informationKUKA.txt");
	_outputFile << "NEW EXPERIMENT\n";
	
	// Catch CTRL+C event with the callback provided
	signal(SIGINT,MotionGenerator::stopNodeCallback);

	// Initialize arduino
	if(_useArduino)
	{
		initArduino();
	}

	sendTarPosition();

	// Check if node OK
	if (_n.ok()) 
	{ 
		// Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The motion generator is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


int MotionGenerator::getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering   
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 0;   
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

	return c;
}


void MotionGenerator::run()
{
	srand (time(NULL));
	
	// Initialize motion duration and initial time reference
	_phaseDuration = _minCleanMotionDuration+(_maxCleanMotionDuration-_minCleanMotionDuration)*((float)std::rand()/RAND_MAX);
	_initTime = ros::Time::now().toSec();

	while (!_stop) 
	{
		// Check if we received the robot pose and foot data
		if(_firstRealPoseReceived)
		{

			// Compute control command
			computeCommand();
			//std::cout << "here"<<std::endl;
			// Start monitoring the keyboard
			// if (getch() == ' ')
			// {
			// 	_errorButtonPressed = true;
			// 	_errorButtonCounter = 0;
			// 	if (_useArduino)
			// 	{
			// 		sendValueArduino(1);
			// 	}
			// }

			// Log data
			logData();
			if(!_ifsendArduino)
			{
				_eventLogger = 1;
				_ifsendArduino = 1;
			}

			// Publish data to topics
			publishData(); // publish the data to controller. 
		}
		else
		{
			_initTime = ros::Time::now().toSec();
		}

		ros::spinOnce();

		_loopRate.sleep();
	}

	// Close arduino communication
	if(_useArduino)
	{
		closeArduino();
	}

	// Send zero linear and angular velocity to stop the robot
	_vd.setConstant(0.0f);
	_omegad.setConstant(0.0f);
	_qd = _q;

	publishData();
	ros::spinOnce();
	_loopRate.sleep();

  // Close file
	_outputFile.close();

  // Close ros
	ros::shutdown();
}


void MotionGenerator::stopNodeCallback(int sig)
{
	me->_stop = true;
}


void  MotionGenerator::computeCommand()
{
  if(!_mouseControlledMotion)
  {
  	// Back and forth motion
    backAndForthMotion();
  }
  else
  {
  	// Mouse controlled motion
    
    processMouseEvents();

    mouseControlledMotion(); 
  }
}


void MotionGenerator::backAndForthMotion()
{

}


void MotionGenerator::mouseControlledMotion()
{
	// Update current time
	double currentTime = ros::Time::now().toSec();

	Eigen::Vector3f gains, error;
	gains.setConstant(0.0f);
	error.setConstant(0.0f);

	Target temporaryTarget;

	switch (_state)
	{
		case State::INIT:
		{
			_phaseDuration = _initDuration;
		}
		case State::CLEAN_MOTION:
		{
			// Check if mouse is in use
			_mouseInUse = true; // make the if statement alway ture -> never go back to the start point		
			if(_obsPositionInput && _recievedTarPositionInput)
			{
				// I put the Target::B redeclaration here.
				_targetOffset.col(Target::B) << _msgPositionTar.position.x, _msgPositionTar.position.y, _msgPositionTar.position.z;	
				sendTarPosition();
				_recievedTarPositionInput = false;
			}

			if(_mouseInUse or currentTime - _lastMouseTime < _commandLagDuration)
			{
				if (_mouseInUse)
				{
					_lastMouseTime = ros::Time::now().toSec();
				}
				// Save current target
				temporaryTarget = _previousTarget;

				// add code to make only change direction in two target positions=======================
				_xd = _x0 + _targetOffset.col(_currentTarget);
				
				float distance1 = (_xd-_x).norm();
				
				std::cout << "distance 1 : " << distance1 << std::endl;
				std::cout << "_x0 : " << _x0(0) << "|" << _x0(1) << "|" << _x0(2) <<std::endl;
				std::cout << "_xd : " << _xd(0) << "|" << _xd(1) << "|" << _xd(2) <<std::endl;
				std::cout << "_x : " << _x(0) << "|" << _x(1) << "|" << _x(2) <<std::endl;

				if(distance1 < TARGET_TOLERANCE) // push the trajectory to the IRL node.
				{
					// If 1 motion is completed before  and haven't publish yet. call the publisher to pushlish traj
					// which will decided by z direction velocity, (pressed, lifted, and no change)
					//std::cout << "mouse vvv"<<std::endl;
					if(fabs(_mouseVelocity(2))>=300.0f and !_ifSentTraj)
					{
						if (_mouseVelocity(2)>0.0f) // if the node is pressed
						{
							_ifSentTraj = true;
							sendMsgForParameterUpdate();
							// clear what stored before
							_msgRealPoseArray.poses.clear();
							_updateIRLParameter = true;
							if (_numOfDemo == 0)
							{
								_rhosfSave[_numOfDemo][0] = init_rho;
								_rhosfSave[_numOfDemo][1] = init_sf;
							}
							else
							{
								_rhosfSave[_numOfDemo][0] = _obs._rho;
								_rhosfSave[_numOfDemo][1] = _obs._safetyFactor;
							}
							_numOfDemo++;
							//for(int i=0;i<_numObstacle;++i)
							//for(int i=0;i<_numOfDemo;++i)
							//{
							if(_randomInsteadIRL)
								std::cout << "num of successful demo: " << _numOfDemo << "\n";
								//std::cout << "Saving rho is  " << _rhosfSave[i][0] << "\n";
								//std::cout << "Saving safetyFactor is  " << _rhosfSave[i][1] << "\n";
							//}
						}
						#ifndef BINARY_INPUT
						else if (_mouseVelocity(2)<0.0f) // if the node is lifted
						{
							_ifSentTraj = true;
							std::cout << "Cleaning the trjaectory ===== " << "\n";
							_msgRealPoseArray.poses.clear();
							_updateIRLParameter = true;
						}
						#endif
					}

					// Update target from mouse input
					if(fabs(_mouseVelocity(0))>fabs(_mouseVelocity(1)))// start the next motion
					{
						#ifdef BINARY_INPUT
						if (!_ifSentTraj)// if the mouse is pressed, then ifSentTraj is true. not pressed then go into this loop...
						{
							// use the previous set of parameters rho and sf
							_updateIRLParameter = false;
							//std::cout << "Use the previous set of parameters " << "\n";

							if (_randomInsteadIRL)
							{
								// move to random generating mode
								_obs._safetyFactor = 1.0f + 0.5f*(float)std::rand()/RAND_MAX;
								_obs._rho = 1.0f + 7*(float)std::rand()/RAND_MAX;
								ROS_INFO_STREAM("Switching Trajectory parameters. Safety Factor: " << _obs._safetyFactor << "Rho: " << _obs._rho);
							}

							//if (_numOfDemo>=1 && !_randomInsteadIRL)
							if (_numOfDemo>=0 && !_randomInsteadIRL)
							{
								//_obs._safetyFactor = _rhosfSave[_numOfDemo-1][1];
								//_obs._rho = _rhosfSave[_numOfDemo-1][0];
								_obs._safetyFactor = 0.9f;
								_obs._rho = 0.9f;
								//std::cout<<" Use the previous set of saftey factor : "<<_obs._safetyFactor << "\n";
								//std::cout<<" Use the previous set of rho : " <<_obs._rho << "\n";
								std::cout<<" Use the small value of saftey factor : "<<_obs._safetyFactor ;//<< "\n";
								std::cout<<"      Use the small value of rho : " <<_obs._rho << "\n";

							}

							//std::cout << "Cleaning the trjaectory ===== " << "\n";
							_msgRealPoseArray.poses.clear();
						}
						//else _updateIRLParameter = true;
						#endif
						_ifSentTraj = false;
						if(_mouseVelocity(0)>0.0f) // positice or negative for direction.
						{
							_currentTarget = Target::A;
						}
						else
						{
							_currentTarget = Target::B;
						}
					}
				}
				//======================================================================================
				
				//--
				// Compute desired target position
				_xd = _x0+_targetOffset.col(_currentTarget);

				// Compute distance to target
				float distance = (_xd-_x).norm();

				//--

				//If during the motion, and eeg send message 1, then ..
				if(_msgEGG == 1 && distance > TARGET_TOLERANCE)
				{
					//std::cout << "reveive eeg message" << std::endl;
					_mouseVelocity(1) = -1.0f;
				}

				if(_mouseVelocity(1)>0.0f) // if not updated, mousevelocity(1) is 0.
				{
					changeRhoEta(0);

				}
				else if (_mouseVelocity(1)<0.0f) // pressing
				{
					changeRhoEta(1);
					
					if(distance > TARGET_TOLERANCE)
					{
						_eventLogger = 3;
						//sendValueArduino(3);
						_ifsendArduino = 0;
					}
				}
				//else
				//{
					//std::cout << "_safetyFactor not changing " << _obs._safetyFactor << "\n";
					//std::cout << "_rho not changing " << _obs._rho << "\n";
				//}
				//======================================================================================

				//if (_currentTarget==Target::A)
				//{
				//	_eventLogger |= 1 << 1;
				//}
				//else
				//{
				//	_eventLogger |= 1 << 2;
				//}

				// If new target, updates previous one and compute new motion and perturbation direction.
				// Also updates the relative location of the obstacle
				if(_currentTarget != temporaryTarget)
				{
					_previousTarget = temporaryTarget;
					
					// Update motion and perturbation direction
					
					Eigen::Vector3f temp;
					temp << 0.0f,0.0f,1.0f;
					_motionDirection = _targetOffset.col(_currentTarget)-_targetOffset.col(_previousTarget);
					_motionDirection.normalize();
					_perturbationDirection = temp.cross(_motionDirection);
					_perturbationDirection.normalize();
					
					if (!_obsPositionInput)
					{
						_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
						_obs._x0(2) -= 0.05f; //0.05f move the obstacle lower, 0.1
						_obs._x0(1) -= 0.0f; //0.0
						if (_iiwaInsteadLwr)
							_obs._x0(0) += 0.1f;
						else
							_obs._x0(0) -= 0.1f; //-0.1 +0.001

						sendObsPosition(true);//the sending is very frequent..
					}
					//else if (_recievedObsPositionInput)
					else
					{
						if (_recievedObsPositionInput)
						{
							// use the position from input, but still using the difference!
							_obs._x0 = _targetOffset.col(_previousTarget);
							_obs._x0(2) += _msgPositionObs.position.z; //0.05f move the obstacle lower, 0.1
							_obs._x0(1) += _msgPositionObs.position.y; //0.0
							_obs._x0(0) += _msgPositionObs.position.x; //-0.1 +0.001
							//sendObsPosition();
							//_recievedObsPositionInput = false;
						}						
					}

					if (_numObstacle == 2)
					{
						//_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
						_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
						_obs._x0(2) -= 0.055f; // Z
						_obs._x0(1) -= 0.20f;
						_obs._x0(0) -= 0.10f; // x coordinate 
						
						//_obs2._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
						_obs2._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
						_obs2._x0(2) += 0.05f;
						_obs2._x0(1) += 0.20f;
						_obs2._x0(0) -= 0.10f;// x coordinate 
					}

					//std::cerr << "x0 : " << _x0(0) << " " <<_x0(1)<<" "<<_x0(2)<< std::endl;
					//std::cerr << _currentTarget << " " << temporaryTarget << " " << std::endl;
					//std::cerr << "obs.x0: "<< _obs._x0(0) << " " << _obs._x0(1) << " " << _obs._x0(2) << std::endl;

				}

				//--
				// Compute desired target position
				//_xd = _x0+_targetOffset.col(_currentTarget);

				// Compute distance to target
				//float distance = (_xd-_x).norm();
				if(distance < TARGET_TOLERANCE)
				{
					// Target is reached
					// make the eventlogger is 0 between two motions
					_eventLogger = 0;

					// Random change in trajectory parameters
					if (_previousTarget != _currentTarget)
					{
						//_eventLogger = 15;
						std::cout << "The number of trials : " << _trialCount << std::endl;
						_trialCount++;
						if (_switchingTrajectories and (float)std::rand()/RAND_MAX>0.25)
						{
							_msg_para_up.data = 1.0f;
							//sendMsgForParameterUpdate(_msg_para_up);//give success signal back to paramter updateing node
						}
					}
					else
					{
						_eventLogger = 0;
					}

					_previousTarget = _currentTarget;

					// Update target
					_reachedTime = ros::Time::now().toSec();
					// _state = State::PAUSE;
				}

				obsModulator.setObstacle(_obs, _obs2, _numObstacle);
				//std::cout<<'?' << std::endl;
				// Compute the gain matrix M = B*L*B'
				// B is an orthogonal matrix containing the directions corrected
				// L is a diagonal matrix defining the correction gains along the directions
				Eigen::Matrix3f B,L;
				// B.col(0) = _motionDirection;
				// B.col(1) = _perturbationDirection;//swap
				B.col(0) = _perturbationDirection;
				B.col(1) = _motionDirection;
				B.col(2) << 0.0f,0.0f,1.0f;
				//std::cout<<" -- "<<_perturbationDirection(0)<<" - "<<_perturbationDirection(1)<<" - "<<_perturbationDirection(2)<<std::endl;
				gains << 10.0f, 10.0f, 10.0f; //was 10 10 30

				// Compute error and desired velocity
				error = _xd-_x;
				// make the speed slower
				error = error; //  * 0.01f; -> unstable... which makes the gain small..
				L = gains.asDiagonal();
				_vd = B*L*B.transpose()*error;
				std::cout << "vd : " << _vd(0) << "|" << _vd(1) << "|" << _vd(2) << std::endl;
				_vd = obsModulator.obsModulationEllipsoid(_x, _vd, false, _numObstacle);
				_xp = _x;
				if (distance > TARGET_TOLERANCE)
				{
					//block_pose.position.x = _msgRealPose.position.x;
    				//block_pose.position.y = _msgRealPose.position.y;
					//block_pose.position.z = _msgRealPose.position.z;
					// push pose into pose array					
					_msgRealPoseArray.poses.push_back(_msgRealPose);
					//std::cerr << "error" << error(0)<<" "<<error(1)<<" "<<error(2) << std::endl;
					//std::cerr << "error z: " << error(2) << std::endl;
					//std::cerr << "vd z: " << _vd(2) << std::endl;

					_msgMouseI.position.x = _mouseVelocity(0);
    				_msgMouseI.position.y = _mouseVelocity(1);
    				_msgMouseI.position.z = _mouseVelocity(2);
    				// publish the mouse message to irl
    				_msgMouseIRL.xyz.push_back(_msgMouseI);
				}
			}
			else
			{

			}

			//if(_useArduino &&_ifsendArduino)
			if(_useArduino)
			{
				sendValueArduino(_eventLogger);
			}

			// Check for end of clean motion phase
			if(currentTime-_initTime > _phaseDuration and _v.norm()> 1.0e-2f and _perturbationFlag)
			{
				// Go to jerky motion phase
        		_perturbation = true;
				_state = State::JERKY_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = _jerkyMotionDuration;			
				if(_useArduino)
				{
					sendValueArduino(255);
				}
			}
			else if(currentTime-_initTime > _phaseDuration)
			{
				_initTime = ros::Time::now().toSec();
			}
			break;
		}
		case State::PAUSE:
		{
			_vd.setConstant(0.0f);

			// Check for end of pause
			if(currentTime-_reachedTime>_pauseDuration)
			{
				// Go back to clean motion phase
				_state = State::CLEAN_MOTION;
			}
			break;
		}
		case State::JERKY_MOTION:
    	{
	    	_perturbationDirection << 0.0f,0.0f,1.0f;
			// Update perturbation offset based on perturbation velocity + apply saturation
			_perturbationOffset += PERTURBATION_VELOCITY*(-1+2*(float)std::rand()/RAND_MAX)*_dt*_perturbationDirection;
			if(_perturbationOffset.norm()>MAX_PERTURBATION_OFFSET)
			{
				_perturbationOffset *= MAX_PERTURBATION_OFFSET/_perturbationOffset.norm();
			}
			while(_perturbationOffset.norm()< MIN_PERTURBATION_OFFSET)
			{
				_perturbationOffset = MAX_PERTURBATION_OFFSET*(-1+2*(float)std::rand()/RAND_MAX)*_perturbationDirection;
			}

			// Compute desired position by considering perturbation offset
			_xd = _x+_perturbationOffset;

			// Compute the gain matrix M = B*L*B'
			// The gain along the motion direction is set to zero to stay in place 
			// The gain along the z axis is kept to keep the height
			Eigen::Matrix3f B,L;
			B.col(0) = _motionDirection;
			B.col(1) = _perturbationDirection;
			B.col(2) << 1.0f,0.0f,0.0f;
			gains << 0, 30.0f, 10.0f;

			error = _xd-_x;
			L = gains.asDiagonal();
			_vd = B*L*B.transpose()*error;

			// Check for end of jerky motion phase
			if(currentTime-_initTime > _phaseDuration)
			{
				// Update perturbation count + go to clean motion phase
				_perturbationCount++;
        		_perturbation = false;
				_perturbationOffset.setConstant(0.0f);
				_state = State::CLEAN_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = 10+(20-10)*((float)std::rand()/RAND_MAX);
				if(_useArduino)
				{
					sendValueArduino(255);
				}
			}
			break;
		}
		default:
		{
			break;
		}
	}

	// Bound desired velocity
	if (_vd.norm()>0.3f)  // it was 0.3 before, 
	{
		_vd = _vd*0.3f/_vd.norm();
	}

	// Desired quaternion to have the end effector looking down
	_qd << 0.0f, 0.0f, 1.0f, 0.0f;
	//_qd << 0.0f, -0.7f, 0.05f, 0.7f;// if points in horizontal direction

}


void MotionGenerator::processMouseEvents() // process mouse events 
{
  uint8_t event;
  int buttonState, relX, relY, relZ, relWheel;
  float filteredRelX = 0.0f, filteredRelY = 0.0f, filetredRelZ = 0.0f;
  bool newEvent = false;

  // If new event received update last event
  // Otherwhise keep the last one
  if(_msgMouse.event > 0)
  {
    _lastMouseEvent = _msgMouse.event;
    buttonState = _msgMouse.buttonState;
    relX = _msgMouse.relX;
    relY = _msgMouse.relY;
    relZ = _msgMouse.relZ;
    relWheel = _msgMouse.relWheel;
    filteredRelX = _msgMouse.filteredRelX;
    filteredRelY = _msgMouse.filteredRelY;
    filetredRelZ = _msgMouse.filteredRelZ;
    newEvent = true;
  }
  else
  {
    buttonState = 0;
    relX = 0;
    relY = 0;
    relZ = 0;
    relWheel = 0;
    filteredRelX = 0;
    filteredRelY = 0;
    filetredRelZ = 0;
    newEvent = false;
  }

  event = _lastMouseEvent;
	
  // Process corresponding event
  if(!_boolSpacenav)
  {
  	  switch(event)
	  {
	    case mouse_perturbation_robot::MouseMsg::M_CURSOR:
	    {
	      processCursorEvent(filteredRelX, filteredRelY, filetredRelZ, newEvent);
	      break;
	    }
	    default:
	    {
	      break;
	    }
	  }
  }

  //--------
  // following working on the laboratory
  if(_boolSpacenav)
  {
  	//std::cout<< "here ====="<< 0.0f <<std::endl;
	//std::cout<< "here ====="<< _msgSpacenav <<std::endl;
  	processCursorEvent(-350.0f*_msgSpacenav.axes[1]/0.69f, -350.0f*_msgSpacenav.axes[0]/0.69f, -350.0f*_msgSpacenav.axes[2]/0.69f, true);
    //std::cerr << _mouseVelocity.transpose() << std::endl;
	// std::cerr << "a" << std::endl;
  	// std::cerr << _msgSpacenav.axes[0] << " " << _msgSpacenav.axes[1] << " " << _msgSpacenav.axes[2] << std::endl;
  	// std::cerr << "b" << std::endl;
  }
  //--------
}


void MotionGenerator::processCursorEvent(float relX, float relY, float relZ, bool newEvent)
{
	//std::cout<< "here ====="<<std::endl;
  if(!newEvent) // No new event received
  {
    _mouseVelocity.setConstant(0.0f);
  }
  else
  {
  	_mouseInUse = false;
  	// If absolute value higher than min mouse velocity threshold, the mouse is in use,
  	// otherwise mouse velocity is set to zero
    if(fabs(relX)>MIN_XY_REL)
    {
		_mouseVelocity(0) = relX;
    	//_eventLogger |= 1 << 3;
    	_eventLogger = 1;
      	_mouseInUse = true;
    }
    else
    {
    	_mouseVelocity(0) = 0.0f;
    	//_eventLogger &= ~(1 << 3);
    }

    // enable the y direction velocity
    if(fabs(relY)>MIN_Y_REL)
    {
    	_mouseVelocity(1) = relY;
    	
    	_mouseInUse = true;
    }
    else
    {
    	_mouseVelocity(1) = 0.0f;
    } 

    // z direction velocity for publishing
    if(fabs(relZ)>MIN_XY_REL)
    {
    	_mouseVelocity(2) = relZ;
    	//eventLogger
    }
    else
    {
    	_mouseVelocity(2) = 0.0f;
    }
    //std::cout << "----" << std::endl;
   
  }
}


void MotionGenerator::publishData()
{
	//std::cout << "===" << std::endl;
	_mutex.lock();

	if (!_iiwaInsteadLwr)
	{
		// Publish desired twist (passive ds controller)
		_msgDesiredTwist.linear.x  = _vd(0);
		_msgDesiredTwist.linear.y  = _vd(1);
		_msgDesiredTwist.linear.z  = _vd(2);
		_msgDesiredTwist.angular.x = _omegad(0);
		_msgDesiredTwist.angular.y = _omegad(1);
		_msgDesiredTwist.angular.z = _omegad(2);

		_pubDesiredTwist.publish(_msgDesiredTwist);

		// Publish desired orientation
		_msgDesiredOrientation.w = _qd(0);
		_msgDesiredOrientation.x = _qd(1);
		_msgDesiredOrientation.y = _qd(2);
		_msgDesiredOrientation.z = _qd(3);

		_pubDesiredOrientation.publish(_msgDesiredOrientation);				
	}
	else
	{
		// _vd(0) = -1*_vd(0);
		// _vd(1) = -1*_vd(1);
		// Publish desired twist (passive ds controller)
		for(int k = 0; k < 3; k++)
		{
			//std::cout << "k" << k << std::endl;		
		    _msgCommand.data[k]  = _vd(k);
		    _msgCommand.data[k+3]  = _omegad(k);
		}
		for(int k = 0; k < 4; k++)
		{
		    _msgCommand.data[k+6]  = _qd(k);
		}
		//for(int k=0;k<10;k++)
			//std::cout<< _msgCommand.data[k] << " ";
		//std::cout<<std::endl;
		_pubCommand.publish(_msgCommand);
	}

	//std::cout << "===" << std::endl;

	_mutex.unlock();
}


void MotionGenerator::logData()
{
	_outputFile << ros::Time::now() << " " << _x(0) << " " << _x(1) << " " << _x(2) << " " << (int)(_perturbationFlag) << " " << (int)(_switchingTrajectories) << " " 
	<< _obs._p(0) << " " << _obs._safetyFactor << " " << _obs._rho << " " << (int)(_errorButtonPressed) << " " << (int)_eventLogger << std::endl;

	//_errorButtonPressed?

	// if (_errorButtonPressed and _errorButtonCounter > 14)
	// {
	// 	_errorButtonPressed = false;
	// 	_errorButtonCounter = 0;
	// 	if (_useArduino)
	// 	{
	// 		sendValueArduino(0);
	// 	}
	// }
	// else if (_errorButtonPressed)
	// {
	// 	_errorButtonCounter++;
	// }
	// else
	// 	_eventLogger = 0;
}


void MotionGenerator::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
	_msgRealPose = *msg;

	// Update end effecotr pose (position+orientation)
	_x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
	_q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
    _wRb = quaternionToRotationMatrix(_q);

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		_xd = _x;
		_qd = _q;
		_x0 = _xd;
		_xp = _x;
		_vd.setConstant(0.0f);
		_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
		_obs._x0(2) -= 0.1f; //0.05f move the obstacle lower,
		_obs._x0(1) = 0.0f;
		_obs._x0(0) -= 0.1f;

		//obsModulator.setObstacle(_obs);
		obsModulator.setObstacle(_obs,_obs2, _numObstacle);
	}
	//_x[0] = -_x[0];
	//_x[1] = -_x[1];

}


void MotionGenerator::updateRealTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	_v << msg->linear.x, msg->linear.y, msg->linear.z;
}


void MotionGenerator::updateMouseData(const mouse_perturbation_robot::MouseMsg::ConstPtr& msg)
{
	_msgMouse = *msg;

  	if(!_firstMouseEventReceived && _msgMouse.event > 0)
  	{
    	_firstMouseEventReceived = true;
    	_lastMouseTime = ros::Time::now().toSec();
  	}
}


void MotionGenerator::updateSpacenavData(const sensor_msgs::Joy::ConstPtr& msg)
{
	_msgSpacenav = *msg;

	if(!_firstSpacenavDataReceived)
  	{
  		_firstSpacenavDataReceived = true;
  	}
}


void MotionGenerator::updateIRLParameter(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	std::cout<<"updating \n ";
	if(_updateIRLParameter)
	{
		_obs._safetyFactor = msg -> data[1];
		_obs._rho = msg -> data[0];
		std::cout<<"saftey factor \n"<<_obs._safetyFactor << "\n";
		std::cout<<"rho \n" <<_obs._rho << "\n";
	}
}


Eigen::Matrix3f MotionGenerator::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void MotionGenerator::dynamicReconfigureCallback(mouse_perturbation_robot::obstacleAvoidance_paramsConfig &config, uint32_t level)
{
	// ROS_INFO("Reconfigure Request: %d %s %s %f %f", 
 //            config.obstacle_shape_param,
 //            config.perturbation_flag?"True":"False",
 //            config.random_trajectory_switching?"True":"False",
 //            config.obstacle_safety_factor,
 //            config.obstacle_rho);

	_perturbationFlag = config.perturbation_flag;
	_switchingTrajectories = config.random_trajectory_switching;

	if (_switchingTrajectories)
		ROS_WARN("Cannot change safety factor or rho if random switching is on or if a particular trajectory is chosen");
	// else if (config.trajectory_1)
	// {
	// 	_obs._safetyFactor = 1.0f;
	// 	_obs._rho = 1.0f;
	// 	if (_useArduino)
	// 	{
	// 		sendValueArduino(16);
	// 	}
	// }
	// else if (config.trajectory_2)
	// {
	// 	_obs._safetyFactor = 1.5f;
	// 	_obs._rho = 8.0f;
	// 	if (_useArduino)
	// 	{
	// 		sendValueArduino(32);
	// 	}
	// }
	// else if (config.trajectory_3)
	// {
	// 	_obs._p.setConstant(2.0f);
	// 	_obs._safetyFactor = 1.0f;
	// 	_obs._rho = 2.0f;
	// 	if (_useArduino)
	// 	{
	// 		sendValueArduino(64);
	// 	}
	// }
	else
	{
		_obs._safetyFactor = config.obstacle_safety_factor;
		_obs._rho = config.obstacle_rho;
		_obs._p.setConstant(config.obstacle_shape_param);
	}
	//obsModulator.setObstacle(_obs);
	obsModulator.setObstacle(_obs, _obs2, _numObstacle);

	_jerkyMotionDuration = config.jerky_motion_duration;
	_commandLagDuration = config.lag_duration;
}


void MotionGenerator::closeArduino()
{
  close(farduino);
}


void MotionGenerator::initArduino()
{
  struct termios toptions;

  farduino = open("//dev//ttyACM0", O_RDWR | O_NONBLOCK );

  if (farduino == -1)
  {
    perror("serialport_init: Unable to open port ");
  }

  if (tcgetattr(farduino, &toptions) < 0)
  {
    perror("serialport_init: Couldn't get term attributes");
  }
     
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // No flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;

  tcsetattr(farduino, TCSANOW, &toptions);
  if( tcsetattr(farduino, TCSAFLUSH, &toptions) < 0)
  {
    perror("init_serialport: Couldn't set term attributes");
  }
}


void MotionGenerator::sendValueArduino(uint8_t value)
{
  write(farduino,&value,1);
  //std::cout << "Arduino message: " << (int)value << std::endl;
  if (value>0)
  {
    trigger_begin = ros::Time::now();
    trigger_raised = true;
  }
  else
  {
    trigger_raised = false;
  }
}


// the original version for fixed rho and eta
//void MotionGenerator::sendMsgForParameterUpdate(const std_msgs::Float32 value)
//{
	// _obs._safetyFactor = 1.0f + 0.5f*(float)std::rand()/RAND_MAX;
	// _obs._rho = 1.0f + 7*(float)std::rand()/RAND_MAX;

	// Publish to /motion_generator_to_parameter_update for getting new rho and safetyFactor
//	_pubFeedBackToParameter.publish(value);//TODO: which parameter is used for measure success or not
//	std::cout << "Publish for rho and saftey factor updating == " << "\n";
//	std::cout << "The value published == " << value <<"\n";
	// ROS_INFO_STREAM("Switching Trajectory parameters. Safety Factor: " << _obs._safetyFactor << "Rho: " << _obs._rho);	

//}

void MotionGenerator::sendMsgForParameterUpdate()
{
	_pubFeedBackToParameter.publish(_msgRealPoseArray);
	std::cout << "Publishing the trjaectory ===== " << "\n";

	if(_delayIntroduce)
	{
		_pubMouseMsgIRL.publish(_msgMouseIRL);
		//_msgMouseIRL.clear();		
		//_msgMouseIRL.deallocate();
		std::cout << "Publishing the trjaectory (mouse) ===== " << "\n";
	}
}


void MotionGenerator::sendTarPosition()
{
	geometry_msgs::Pose _msgTar;
	_msgTar.position.x = _targetOffset(0,Target::B);
	_msgTar.position.y = _targetOffset(1,Target::B);
	_msgTar.position.z = _targetOffset(2,Target::B);
	std::cout<< _msgTar.position.x <<" "<< _msgTar.position.y <<" "<< _msgTar.position.z << std::endl;	
	_pubTarPosition.publish(_msgTar);
}


void MotionGenerator::sendObsPosition(bool if_obs)
{
	geometry_msgs::Pose _msgObs;
	if (if_obs)
	{
		_msgObs.position.x = _obs._x0(0);
		_msgObs.position.y = _obs._x0(1);
		_msgObs.position.z = _obs._x0(2);	
	}
	else
	{
		_msgObs.position.x = _msgPositionObs.position.x;
		_msgObs.position.y = _msgPositionObs.position.y;
		_msgObs.position.z = _msgPositionObs.position.z;	
	}
	//std::cout<< "sent the obstacle position"<<_msgObs.position.x <<" "<< _msgObs.position.y <<" "<< _msgObs.position.z << std::endl;	
	_pubObsPosition.publish(_msgObs);
}


void MotionGenerator::subPositionObs(const geometry_msgs::Pose::ConstPtr& msg)
{
	_msgPositionObs = *msg;
	_recievedObsPositionInput = true;
	std::cout << "Recieved the position obstacle : " << _msgPositionObs.position.x << " " << _msgPositionObs.position.y << " " <<  _msgPositionObs.position.z << std::endl;
	sendObsPosition(false);
}


void MotionGenerator::subPositionTar(const geometry_msgs::Pose::ConstPtr& msg)
{
	_msgPositionTar = *msg;
	_recievedTarPositionInput = true;
	std::cout << "Recieved the position target : " << _msgPositionTar.position.x << " " << _msgPositionTar.position.y << " " <<  _msgPositionTar.position.z << std::endl;
}


void MotionGenerator::subMessageEEG(const std_msgs::String::ConstPtr& msg)
{
	_msgMessageEEG = *msg;
	//_msgEGG = (int)msg->data;
	//_msgEGG = (int)_msgMessageEEG.data;
	_msgEGG = std::stoi(_msgMessageEEG.data);
	std::cout << "reveived EEG data: " << _msgEGG << std::endl;
}


void MotionGenerator::changeRhoEta(int indcator)
{
	if (_numObstacle == 2)
	{
		// make change
		if (indcator)
		{	
			_indexx += 1;
			if (_indexx >= 5)
			{
				_indexx = 0;
				_obs._safetyFactor += 0.01/2;
				_obs._rho += 0.1/2;
				if (_obs._safetyFactor >= MAX_ETA)
				{
					_obs._safetyFactor = MAX_ETA;
				}
				if (_obs._rho >= MAX_RHO)
				{
					_obs._rho = MAX_RHO;
				}

				//if (_numObstacle == 2)
				//{
				//	_obs2._rho = _obs._rho;
				//	_obs2._safetyFactor = _obs._safetyFactor;
				//}
				std::cout << "_safetyFactor Increasing " << _obs._safetyFactor << "\n";
				std::cout << "_rho Increasing " << _obs._rho << "\n";
			}
		}
		else
		{
			_indexy += 1;
			if (_indexy >= 5)
			{
				_indexy = 0;
				_obs._safetyFactor -= 0.01;
				_obs._rho -= 0.1;
			if (_obs._safetyFactor <= MIN_ETA)
			{
				_obs._safetyFactor = MIN_ETA;
			}
			if (_obs._rho <= MIN_RHO)
			{
				_obs._rho = MIN_RHO;
			}
			//if (_numObstacle == 2)
			//	{
			//		_obs2._rho = _obs._rho;
			//		_obs2._safetyFactor = _obs._safetyFactor;
			//	}
			//	std::cout << "_safetyFactor Decreasing " << _obs._safetyFactor<<"\n";
			//	std::cout << "_rho Decreasing " << _obs._rho << "\n";
			}
		}
	}
	else
	{
		// make change
		if (indcator)
		{	
			_indexx += 1;
			if (_indexx >= 5)
			{
				_indexx = 0;
				
				#ifndef BINARY_INPUT
					_obs._safetyFactor += 0.01/2;
					_obs._rho += 0.1/2;
				#else
					_obs._safetyFactor += 0.01/2*4; // in binary feedback case.. originally value is 4.
					_obs._rho += 0.1/2*4;
				#endif

				if (_obs._safetyFactor >= MAX_ETA)
				{
					_obs._safetyFactor = MAX_ETA;
				}
				if (_obs._rho >= MAX_RHO)
				{
					_obs._rho = MAX_RHO;
				}

				if (_numObstacle == 2)
				{
					_obs2._rho = _obs._rho;
					_obs2._safetyFactor = _obs._safetyFactor;
				}
				std::cout << "_safetyFactor Increasing " << _obs._safetyFactor << "\n";
				std::cout << "_rho Increasing " << _obs._rho << "\n";
			}
		}
		else
		{
			#ifndef BINARY_INPUT
			{
				_indexy += 1;
				if (_indexy >= 5)
				{
					_indexy = 0;
					_obs._safetyFactor -= 0.01;
					_obs._rho -= 0.1;
				if (_obs._safetyFactor <= MIN_ETA)
				{
					_obs._safetyFactor = MIN_ETA;
				}
				if (_obs._rho <= MIN_RHO)
				{
					_obs._rho = MIN_RHO;
				}
				if (_numObstacle == 2)
					{
						_obs2._rho = _obs._rho;
						_obs2._safetyFactor = _obs._safetyFactor;
					}
					std::cout << "_safetyFactor Decreasing " << _obs._safetyFactor<<"\n";
					std::cout << "_rho Decreasing " << _obs._rho << "\n";
				}				
			}
			#endif
		}
	}	
}