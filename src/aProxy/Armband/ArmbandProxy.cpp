// Project Header files
#include "ArmbandProxy.hpp"
#include "Timer.hpp"
#include "utils.hpp"

// Standard Header files
#include <iomanip>
#include <chrono>
#include <ctime> 


#define M_PI        3.14159265358979323846264338327950288   /* pi             */

int Arr[2];
std::vector<HapticDevice> devices;
double previousTime = 0.00;
double currentTime = 0.00;
clock_t tStart;
bool first = true, second = false;


/**
* @brief A class to render different vibration patterns on the Armband device
*
*/
ArmbandPattern::ArmbandPattern(int n_motors) {

	// ------------ Custom Force Feedback variables ------------ //
	// Task variables
	task_started = -1;
	k = 0;
	start_penetration = false;

	// Pattern variables
	/// Pattern 2
	div = 16;
	is_puncturing = false;
	tmp_is_puncturing = is_puncturing;
	is_transition = false;
	/// Pattern 3
	num_motors = n_motors;
	layers_passed = 0;
	uprising = false;
	layers_updated = false;
	p2_alt_solo = true;
	motor_state = { 0, -1, -1, -1 };
	motor_elastic_f = { -1., -1., -1., -1. };
	motor_friction_f = { -1., -1., -1., -1. };
	penetrated = { false, false, false, false };
	// -------------------------------------------------------------- //

	// --------------- Custom User Feedback variables --------------- //
	rupture_times = { ".", ".", ".", "." };
	// -------------------------------------------------------------- //
}

/**
* @brief Function to render vibration pattern
*
*/
void ArmbandPattern::render(int pattern, const Eigen::VectorXf& f) {

	// Initialization
	Eigen::Vector3f fb_m = Eigen::Vector3f(f(0), f(1), f(2));
	Eigen::Vector3f friction = Eigen::Vector3f(f(3), f(4), f(5));
	Eigen::Vector3f elastic = Eigen::Vector3f(f(6), f(7), f(8));
	Eigen::Vector4i armBand_motorsi;
	armBand_motorsi.setZero();

	// Re-init global variables
	if ((task_started == -1) && (elastic(1) == 0) && (friction(1) == 0)) {

		// Re-init Pattern 1 variables
		k = 0;
		start_penetration = false;

		// Re-init Pattern 2 variables
		div = 16;
		is_puncturing = false;
		tmp_is_puncturing = is_puncturing;
		is_transition = false;

		// Re-init Pattern 3 variables
		task_started = 0;
		layers_passed= 0;
		uprising = false;
		layers_updated = false;
		motor_state = { 0, -1, -1, -1 };
		motor_elastic_f = { -1., -1., -1., -1. };
		motor_friction_f = { -1., -1., -1., -1. };
		penetrated = { false, false, false, false };
		rupture_times = { ".", ".", ".", "." };
	}

	// Force selection and computation
	/*std::cout << "____________________________________" << std::endl;
	std::cout << "friction(1) = " << friction(1) << std::endl;
	std::cout << "elastic(1) = " << elastic(1) << std::endl;*/
	float fb_m_component = fb_m(1);
	float elas_component = elastic(1);
	float fric_component = friction(1);
	float elastic_term = (MAX_Elastic_force_armband_effect - MIN_FRE_ARMBAND) / MAX_Elastic_force;
	if (friction(1) < 0) {
		uprising = true;
		fric_component = 0.5 * (-fric_component);
		fb_m_component = 0.5 * (-fb_m_component);
	}
	float motor_force_fric = friction(1) == 0 ? 0 : fric_component * elastic_term + MIN_FRE_ARMBAND + DELTA_FR_FORCE;
	float motor_force_elas = elastic(1) == 0 ? 0 : elas_component * elastic_term + MIN_FRE_ARMBAND + DELTA_EL_FORCE;
	float motor_force = friction(1) == 0 ? 0 : fb_m_component * elastic_term + MIN_FRE_ARMBAND;
	//std::cout << "motor force = " << motor_force << std::endl;
	//std::cout << "____________________________________" << std::endl;

	// Select pattern
	if (pattern == F_PATTERN_1)
		this->pattern1(armBand_motorsi, friction, elastic, fb_m, 
					   motor_force_fric, motor_force_elas, motor_force);
	else if (pattern == F_PATTERN_2)
		this->pattern2(armBand_motorsi, friction, elastic, fb_m,
					   motor_force_fric, motor_force_elas, motor_force);
	else if (pattern == F_PATTERN_3)
		this->pattern3(armBand_motorsi, friction, elastic, fb_m,
					   motor_force_fric, motor_force_elas, motor_force);

	// No vibration when out of tissues
	if (friction(1) == 0 && uprising)
		armBand_motorsi.setZero();

	/* Debug print
	std::cout << "____________________________________" << std::endl;
	std::cout << "motor state = [" << motor_state[0] << "|" << motor_state[1] << "|" << motor_state[2] << "|" << motor_state[3] << "]" << std::endl;
	std::cout << "motor forces = [" << armBand_motorsi[0] << "|" << armBand_motorsi[1] << "|" << armBand_motorsi[2] << "|" << armBand_motorsi[3] << "]" << std::endl;
	std::cout << "____________________________________" << std::endl;
	*/
	devices.at(0).run(armBand_motorsi);
}

/**
* @brief Rupture event registration function
*	This regisrates the rupture timestamp into a string and shows it
*/
void ArmbandPattern::register_rupture() {

	// Get timestamp of the rupture event
	auto rup = std::chrono::system_clock::now();
	auto rup_millis = std::chrono::duration_cast<std::chrono::milliseconds>(rup.time_since_epoch()).count() % 1000;
	std::time_t rup_time = std::chrono::system_clock::to_time_t(rup);
	auto time_struct = gmtime(&rup_time);
	std::stringstream ss;
	std::string hour = std::to_string(time_struct->tm_hour);
	std::string min = std::to_string(time_struct->tm_min);
	std::string sec = std::to_string(time_struct->tm_sec);
	ss << hour << ":" << min << ":" << sec << '.' << std::setfill('0') << std::setw(3) << rup_millis;
	std::string ssstr = ss.str();
	const char* str_time = ssstr.c_str();

	// Register timestamp
	for (int i = 0; i < num_motors; i++) {
		if (rupture_times[i] == ".") {
			rupture_times[i] = (char*)str_time;
			break;
		}
	}
	std::cout << "[system] - Rupture happened at time " << str_time << std::endl;
}

/**
* @brief Pattern1 rendering function
*	All the motors vibrate contemporary, and the
	vibration is 'alternate' when the elastic force
	is different than 0 (we are penetrating a tissue
*/
void ArmbandPattern::pattern1(
		Eigen::Vector4i& armBand_motorsi, Eigen::Vector3f friction, Eigen::Vector3f elastic, Eigen::Vector3f fb_m, 
		float motor_force_fric, float motor_force_elas, float motor_force) {

	// Render friction force and rupture event
	if (elastic(1) == 0) {
		if (is_puncturing && start_penetration) {
			k = 0;
			start_penetration = false;
			armBand_motorsi.setZero();
		}
		// Render rupture
		if (is_puncturing && (k == 2 || k == 3 || k == 4)) {
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				armBand_motorsi(i_motor) = 200;
			}
			if (k == 2)
				this->register_rupture();
		}
		else if (is_puncturing && (k == 5)) {
			armBand_motorsi.setZero();
			is_puncturing = false;
		}
		// Render friction
		else {
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				armBand_motorsi(i_motor) = motor_force;
			}
		}
		k++;
		if (k == 13) k = 0;
	}
	// Render elastic force
	else {
		is_puncturing = true;
		start_penetration = true;
		if (k == 13) k = 0;
		if (k == 0 || k == 1 || k == 2) armBand_motorsi(0) = motor_force;
		if (k == 3 || k == 4 || k == 5) armBand_motorsi(1) = motor_force;
		if (k == 6 || k == 7 || k == 8) armBand_motorsi(2) = motor_force;
		if (k == 9 || k == 10 || k == 11) armBand_motorsi(3) = motor_force;
		if (k == 12) armBand_motorsi.setZero();
		k++;
	}
	/*for (int i_motor = 0; i_motor <= 3; i_motor++) {
		armBand_motorsi(i_motor) = motor_force;
	}*/
}

/**
* @brief Pattern2 rendering function
*	The intensity of the force sent to
	the motors is the same during the
	entire process. The vibration is
	continue during the tissue break.
	The vibration during penetration occurs
	intermittently and the frequency increases
	according to the depth of the tissue.
*/
void ArmbandPattern::pattern2(
		Eigen::Vector4i& armBand_motorsi, Eigen::Vector3f friction, Eigen::Vector3f elastic, Eigen::Vector3f fb_m,
		float motor_force_fric, float motor_force_elas, float motor_force) {

	// TODO:
	/* Rendering of the vibration when the needle
		is coming back to the initial configuration*/
	// Debug print
	//std::cout << "motor force: " << motor_force << std::endl;
	// Original fixed force
	//if (motor_force != 0) motor_force = 90.0;

	if (elastic(1) >= 1e-3) {
		is_transition = true;
		is_puncturing = true;
		start_penetration = true;
		for (int i_motor = 0; i_motor <= 3; i_motor++) {
			if (uprising) motor_force = 45.0;
			armBand_motorsi(i_motor) = motor_force_elas;
		}
	}
	else if (elastic(1) == 0) {
		if (is_transition) {
			div = div == 2 ? 2 : div / 2;
			is_transition = false;
		}
		if (is_puncturing && start_penetration) {
			k = 0;
			start_penetration = false;
			armBand_motorsi.setZero();
		}
		// Render rupture
		if (is_puncturing && (k == 2 || k == 3 || k == 4)) {
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				armBand_motorsi(i_motor) = 200;
			}
			if (k == 2)
				this->register_rupture();
		}
		else if (is_puncturing && (k == 5)) {
			armBand_motorsi.setZero();
			is_puncturing = false;
		}
		else if (!is_puncturing) {
			if ((k % div == 0) || (((k-1) % div == 0) && (div > 2))) {
				for (int i_motor = 0; i_motor <= 3; i_motor++) {
					armBand_motorsi(i_motor) = motor_force_fric;
				}
			}
		}
	}

	if (uprising) {
		for (int i_motor = 0; i_motor <= 3; i_motor++) {
			armBand_motorsi(i_motor) = 45.0;
		}
	}
	else {
		motor_force_fric = motor_force_fric * 1.5;
	}

	k++;
	if (k == 20) k = 0;

	// Debug print
	/*std::cout << "elastic force: " << motor_force_elas << std::endl;
	std::cout << "friction force: " << motor_force_fric << std::endl;
	std::cout << "IS_PUNCTURING: " << IS_PUNCTURING << std::endl;
	std::cout << "is_transition: " << is_transition << std::endl;
	std::cout << "TMP_IS_PUNCTURING: " << TMP_IS_PUNCTURING << std::endl;
	std::cout << "div: " << div << std::endl;
	std::cout << "k: " << k << std::endl;
	std::cout << "________________________________" << std::endl;*/
}

/**
* @brief Pattern3 rendering function
*	Each time a tissue is penetrated, a
	different motor activates. Initially,
	the vibration is alternate; after the tissue break,
	the vibration becomes continue. 
	MOTOR_STATE: {
				-1 -> Inactive
	      		 0 -> Ready
				 1 -> Alternate mode
				 2 -> Continuous mode
	}
	TODO: implement friction tissue switch during uprising
*
*/
void ArmbandPattern::pattern3(
		Eigen::Vector4i& armBand_motorsi, Eigen::Vector3f friction, Eigen::Vector3f elastic, Eigen::Vector3f fb_m,
		float motor_force_fric, float motor_force_elas, float motor_force) {

	// Render elastic force
	if (elastic(1) > 1e-3) {
		// Update number of layers penetrated
		if (!layers_updated) {
			layers_passed++;
			layers_updated = true;
		}

		// Updated variables needed to render rupture
		is_puncturing = true;
		start_penetration = true;

		// Reset motor status after the 4th penetration
		// (eventually change with the first motor activated)
		// TODO: test with more than 4 layers
		if (layers_passed > num_motors) {
			int motor_index = (layers_passed - 1) % num_motors;
			motor_state[motor_index] = 0;
		}

		// Set status
		for (int i_motor = 0; i_motor <= 3; i_motor++)
		{
			if ((motor_state[i_motor] == 0)) {
				motor_state[i_motor] = 1;
				penetrated[i_motor] = true;
			}
		}
		// Assign vibration value
		for (int i_motor = 0; i_motor <= 3; i_motor++)
		{
			if (!p2_alt_solo) {
				if ((motor_state[i_motor] == 1) && (k % 3 == 0)) {
					if (motor_elastic_f[i_motor] == -1)
						motor_elastic_f[i_motor] = motor_force_elas;
					armBand_motorsi(i_motor) = motor_elastic_f[i_motor];
				}
				else if (motor_state[i_motor] == 2)
					armBand_motorsi(i_motor) = motor_friction_f[i_motor];
			}
			else {
				if ((motor_state[i_motor] == 1) && (k % 3 == 0)) {
					if (motor_elastic_f[i_motor] == -1)
						motor_elastic_f[i_motor] = motor_force_elas;
					armBand_motorsi(i_motor) = motor_elastic_f[i_motor];
				}
			}
		}
	}
	// Render friction force and rupture event
	else if (elastic(1) == 0) {
		// Update flag to update number of layers
		layers_updated = false;

		// Check status
		for (int i_motor = 0; i_motor <= 3; i_motor++)
		{
			if (penetrated[3]) motor_state[3] = 2;
			// TODO: check if <= needed with 4 motor active
			if (layers_passed < num_motors) {
				if ((i_motor > 0) && (motor_state[i_motor] == -1) && (motor_state[i_motor - 1] == 1)) {
					motor_state[i_motor - 1] = 2;
					motor_state[i_motor] = 0;
				}
			}
			else {		// TODO: test with more layers
				if (motor_state[i_motor] == 1) {
					motor_state[i_motor] = 2;
				}
			}
		}

		// Check for rupture initialization
		if (is_puncturing && start_penetration) {
			k = 0;
			start_penetration = false;
			armBand_motorsi.setZero();
		}
		// Render rupture
		if (is_puncturing && (k == 2 || k == 3 || k == 4)) {
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				armBand_motorsi(i_motor) = 200;
			}
			if (k == 2)
				this->register_rupture();
		}
		else if (is_puncturing && (k == 5)) {
			armBand_motorsi.setZero();
			is_puncturing = false;
		}
		else {
			// Assign vibration value
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				if (motor_state[i_motor] == 2) {
					if (motor_friction_f[i_motor] == -1)
						motor_friction_f[i_motor] = motor_force_fric;
					armBand_motorsi(i_motor) = motor_friction_f[i_motor];
				}
			}
		}
	}
	k++;
	if (k == 13) k = 0;
}

/**
* @brief Default Destructor of ArmbandPattern class
*
*/
ArmbandPattern::~ArmbandPattern() {
}


/**
* @brief A class to initialize the armband and connect to the bluetooth com port.
*
*/
VibBrac::VibBrac(int n) {
	devices.resize(n);
	int l_iHapticInitTrial = 1;

	// Set the COM port as seen in the device bluetooth settings
	std::string str = "COM3";
	std::wstring g_sHapticPort(str.length(), L' ');
	std::copy(str.begin(), str.end(), g_sHapticPort.begin());

	std::cout << "Starting" << std::endl;

	// Try to connect to all the armbands. n stands for number of armbands
	for (int i = 0; i < n; i++)
	{
		while (!devices.at(i).is_connected && l_iHapticInitTrial < 3)
		{
			devices.at(i).startCommunication(g_sHapticPort.c_str());
			std::wcout << "Try to connect : " << g_sHapticPort.c_str() << " " << l_iHapticInitTrial << std::endl;
			l_iHapticInitTrial++;
			Sleep(0.3);
		}

		if (l_iHapticInitTrial >= 3) {
			std::cout << "Exceeded number of trials for connecting the Armband. The device will not work." << std::endl;
		}

	}
	std::cout << "Done." << std::endl;
}

/**
* @brief Default Distructor of VibBrac class
*
*/
VibBrac::~VibBrac() {
	for (int i = 0; i < devices.size(); i++)
	{
		devices.at(i).closeCommunication();
	}
}

/**
* @brief Default contructor of ArmbandProxy class
*
*/
ArmbandProxy::ArmbandProxy() : HapticProxy() {//, HLInterface(){

	time_ = 0.0;
	
	// Init the class
	//this->init();

}

/**
* @brief Default destroyer of ArmbandProxy class
*
*/
ArmbandProxy::~ArmbandProxy() {

	delete this->vibs;

}

/**
* @brief Copy constructor of the ArmbandProxy class
* @param vp the ArmbandProxy
*/
ArmbandProxy::ArmbandProxy(ArmbandProxy& vp) {

}


/**
* @brief Default init function
*/
void ArmbandProxy::init() {

	// Create armband object to access it. The parameter is to select number of armbands.
	// Connect to the Armband 
	this->vibs = new VibBrac(1);

	std::cout << "Initializing new VibBrac dynamic class ..." << std::endl;

	this->hapticDOF = 6;
	this->hapticState.force.setZero(this->hapticDOF);
	this->setHapticPattern(F_PATTERN_1);
	this->availability(true);
	this->setRunning(true);
}



/**
* @brief Set function
* Set the feedback haptic force on the Armband device
* @param f: the feedback force to be set
*/
inline void ArmbandProxy::sendForce(const Eigen::VectorXf& f) {

	float fz = f(1); // TODO: Generalize this
	Eigen::Vector4i armBand_motorsi;
	armBand_motorsi.setZero();
	float elastic_offest = fz > 0 ? (fz / MAX_Elastic_force) * (MAX_Elastic_force_armband_effect - MIN_FRE_ARMBAND) + MIN_FRE_ARMBAND : 0;
	for (int i_motor = 0; i_motor <= 3; i_motor++)
	{
		armBand_motorsi(i_motor) = elastic_offest;
	}

	//std::cout << "[Armband] fz = " << armBand_motorsi.transpose() << std::endl;
	devices.at(0).run(armBand_motorsi);
}



/**
* @brief CUSTOM Set function
* Set the custom feedback haptic force on the Armband device
* @param f: the feedback force to be set -> flat 3x3 matrix with shape (1,9)
* @param i: counter1 to alternate vibrations
* @param j: counter2 to alternate vibrations
*/
inline void ArmbandProxy::sendForceCustom(const Eigen::VectorXf& f, ArmbandPattern& armband_pattern) {

	// Motor control
	int feedback_pattern = this->getHapticPattern();

	// Haptic rendering
	armband_pattern.render(feedback_pattern, f);
}


/**
* @Send PWM to each motor's ArmBand according to the EF position, warning thrwow level frecuncy PWM  when the trajectory isno in a straigh forward  way(z-axis).
*/
Eigen::Vector4i ArmbandProxy::Trajectory_alert(Eigen::Vector3f EF_Pos, Eigen::Vector3f z_obj_line, float Elastic_force)
{
	// The EF position is arranged in x,y,z order in other words 0,1,2 indexes
	EF_Error = z_obj_line - EF_Pos;

	Eigen::Vector4f armBand_motors;

	armBand_motors.setZero();
	if (abs(EF_Error(1)) > MAX_ERROR_THRESHOLD_LOWER_BOUND)
	{
		int startOffset = 30;
		// Proportional controller for each vibrator in the armband
		if (EF_Error(1) < 0) {
			armBand_motors(2) = -EF_Error(1) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;//  -MAX_ERROR_THRESHOLD_LOWER_BOUND is for the signal to change smoothly
		}
		else
		{
			armBand_motors(0) = EF_Error(1) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;
		}
	}
	if (abs(EF_Error(0)) > MAX_ERROR_THRESHOLD_LOWER_BOUND)
	{
		if (EF_Error(0) < 0) {
			armBand_motors(3) = -EF_Error(0) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;//  -MAX_ERROR_THRESHOLD_LOWER_BOUND is for the signal to change smoothly
		}
		else
		{
			armBand_motors(1) = EF_Error(0) - MAX_ERROR_THRESHOLD_LOWER_BOUND + ARMBAND_OFFSET;
		}
	}

	armBand_motors = armBand_motors / MAX_ERROR_THRESHOLD_UPPER_BOUND * MAX_FRE_ARMBAND;
	Eigen::Vector4i armBand_motorsi = armBand_motors.cast <int>();
	//armBand_motorsi.setZero();

	// std::cout << "EF:	"<< EF_Pos.transpose() << "	Error:	" << EF_Error.transpose() << "	PWM:	" << armBand_motorsi.transpose() << std::endl;

	// Sending data to the armband
	int armBand_motorsi0[] = { armBand_motors(0), armBand_motors(1), armBand_motors(2), armBand_motors(3) };

	Eigen::Vector4i armBand_motorsi2 = this->Cutting_force2motors(armBand_motorsi0, Elastic_force);
	devices.at(0).run(armBand_motorsi2);
	// std::cout << "Elastic_Force:	"<< Elastic_force << "	Motors PWM:	" << armBand_motorsi2.transpose()  << std::endl;
	return armBand_motorsi;

}

/**
* @Converts the pure forces into PWM signalas to be sent until ArmBand device
*/
Eigen::Vector4i ArmbandProxy::Forces2MotorsDim(Eigen::Vector6f _ForcesAndTorques) {

	//the forces are arranged in the following _ForcesAndTorques[0:3]=Fx,Fy,Fz
	Eigen::Vector4f armBand_motors;
	armBand_motors.setZero();
	//int temp = 150;
	//armBand_motors.get();
	//std::cout << _ForcesAndTorques << std::endl;

	// Proportional controller for each vibrator in the armband based on feedback
	if (_ForcesAndTorques(1) < 0) {
		armBand_motors(2) = -_ForcesAndTorques(0);
	}
	else
	{
		armBand_motors(0) = _ForcesAndTorques(0);
	}
	if (_ForcesAndTorques(2) < 0) {
		armBand_motors(3) = -_ForcesAndTorques(1);
	}
	else
	{
		armBand_motors(1) = _ForcesAndTorques(1);
	}
	armBand_motors = armBand_motors / MAX_FORCE * MAX_FRE_ARMBAND;
	Eigen::Vector4i armBand_motorsi = armBand_motors.cast <int>();

	return armBand_motorsi;
}



/**
* @brief Default run function
*/
void ArmbandProxy::run() {


	// Time variables
	double tictoc, dt, rate, tic, toc, tac, Ts;
	Timer clock;
	clock.setRate(20.0);
	rate = clock.getRate();
	Ts = 1.0 / rate;

	// Wait for some seconds to show the task menu on screen correctly (TODO: find a smart way and remove this horror)
	clock.timeSleep(1.0);

	// Send the haptic force on the device
	//this->sendForce(this->hapticState.force);
	//this->running = false;

	// Initialize custom pattern 
	this->setHapticPattern(F_PATTERN_1);

	// Probably useful counters
	int i = 0;
	int j = 0;

	// ArmbandPattern class instance
	ArmbandPattern armband_pattern(4);

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		//TEST
		//this->hapticState.force(1) = 0.005 * sin(time_);

		// Update counter
		// TODO: check better systems to update counter
		/*i++;
		j++;
		if (i == 20) i = 0;
		if (j == 14) j = 0;*/

		// Send the haptic force on the device
		this->sendForceCustom(this->hapticState.force, armband_pattern);

		//----------------------------------------------------------------//

		// Measure the ending time and the elapsed time
		toc = clock.getCurTime();
		tictoc = clock.elapsedTime(tic, toc);

		// Wait until Ts
		if (tictoc < Ts) {
			clock.timeSleep(Ts - tictoc);
		}

		// Measure the final time after sleep to check the actual rate of the thread
		tac = clock.getCurTime();
		dt = clock.elapsedTime(tic, tac);
		time_ += dt;
		//std::cout << "[ArmandProxy] Running rate:" << (1.0 / dt) << std::endl;

	}

	// Reset haptic force
	this->hapticState.force.setZero(this->getHapticDOF());
	this->sendForce(this->hapticState.force);

}

/**
* @brief Default clear function
*/
void ArmbandProxy::clear() {

	// Set running on false
	this->setRunning(false);

}


/**
* @adding  cutting forces into tissues effect
*/
Eigen::Vector4i ArmbandProxy::Cutting_force2motors(int ForcesAndTorques[], float Elastic_force) {

	Eigen::Vector4i armBand_motorsi;
	armBand_motorsi.setZero();
	//float elastic_offest = Elastic_force>0 ? (Elastic_force / MAX_Elastic_force)*(MAX_Elastic_force_armband_effect)+MIN_FRE_ARMBAND : 0;
	float elastic_offest = Elastic_force > 0 ? (Elastic_force / MAX_Elastic_force) * (MAX_Elastic_force_armband_effect - MIN_FRE_ARMBAND) + MIN_FRE_ARMBAND : 0;
	for (int i_motor = 0; i_motor <= 3; i_motor++)
	{
		armBand_motorsi(i_motor) = elastic_offest; //+ ForcesAndTorques[i_motor]+ elastic_offest;
	}

	return armBand_motorsi;
}


/**
* @brief Debug function
* Print the names of all the V-REP objects loaded for the simulation
*/
void ArmbandProxy::print_Forces_Set(Eigen::Vector6f sensorFe0et) {
	Eigen::Vector4i motor_pwm = this->Forces2MotorsDim(sensorFe0et);

	//motor_pwm=this->Cutting_force2motors(motor_pwm);
}