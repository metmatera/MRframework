// Project Header files
#include "ArmbandProxy.hpp"
#include "Timer.hpp"
#include "utils.hpp"


#define M_PI        3.14159265358979323846264338327950288   /* pi             */

int Arr[2];
std::vector<HapticDevice> devices;
double previousTime = 0.00;
double currentTime = 0.00;
clock_t tStart;
bool first = true, second = false;

// ------------ Custom Force Feedback initialization ------------ //
// Task variables
int TASK_STARTED = -1;
int k = 0;
bool START_PENETRATION = false;

// Pattern variables
// Pattern 2
int DIV = 8;
bool IS_PUNCTURING = false;
bool TMP_IS_PUNCTURING = IS_PUNCTURING;
bool IS_TRANSICTION = false;

// Pattern 3
int NUM_MOTORS = 4;
int LAYERS_PASSED = 0;
bool UPRISING = false;
bool LAYERS_UPDATED = false;
bool P2_ALT_SOLO = true;
std::vector<int> MOTOR_STATE = { 0, -1, -1, -1 };
std::vector<float> MOTOR_ELASTIC_F = { -1., -1., -1., -1. };
std::vector<float> MOTOR_FRICTION_F = { -1., -1., -1., -1. };
std::vector<bool> PENETRATED = { false, false, false, false };
// -------------------------------------------------------------- //


/**
* @brief A class to initialize the armband and connect to the bluetooth com port.
*
*/
VibBrac::VibBrac(int n) {
	devices.resize(n);
	int l_iHapticInitTrial = 1;

	// Set the COM port as seen in the device bluetooth settings
	std::string str = "COM8";
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
inline void ArmbandProxy::sendForceCustom(const Eigen::VectorXf& f, const int i, const int j) {

	// Initialization
	Eigen::Vector3f fb_m = Eigen::Vector3f(f(0), f(1), f(2));
	Eigen::Vector3f friction = Eigen::Vector3f(f(3), f(4), f(5));
	Eigen::Vector3f elastic = Eigen::Vector3f(f(6), f(7), f(8));
	Eigen::Vector4i armBand_motorsi;
	armBand_motorsi.setZero();

	// Re-init global variables
	if ((TASK_STARTED == -1) && (elastic(1) == 0) && (friction(1) == 0)) {

		// Re-init Pattern 1 variables
		k = 0;
		START_PENETRATION = false;

		// Re-init Pattern 2 variables
		DIV = 8;
		IS_PUNCTURING = false;
		TMP_IS_PUNCTURING = IS_PUNCTURING;
		IS_TRANSICTION = false;

		// Re-init Pattern 3 variables
		TASK_STARTED = 0;
		LAYERS_PASSED = 0;
		UPRISING = false;
		LAYERS_UPDATED = false;
		MOTOR_STATE = { 0, -1, -1, -1 };
		MOTOR_ELASTIC_F = { -1., -1., -1., -1. };
		MOTOR_FRICTION_F = { -1., -1., -1., -1. };
		PENETRATED = { false, false, false, false };
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
		UPRISING = true;
		fric_component = 0.5 * (-fric_component);
		fb_m_component = 0.5 * (-fb_m_component);
	}
	float motor_force_fric = friction(1) == 0 ? 0 : fric_component * elastic_term + MIN_FRE_ARMBAND + DELTA_FR_FORCE;
	float motor_force_elas = elastic(1) == 0 ? 0 : elas_component * elastic_term + MIN_FRE_ARMBAND + DELTA_EL_FORCE;
	float motor_force = friction(1) == 0 ? 0 : fb_m_component * elastic_term + MIN_FRE_ARMBAND;
	//std::cout << "motor force = " << motor_force << std::endl;
	//std::cout << "____________________________________" << std::endl;

	// Motor control
	int feedback_pattern = this->getHapticPattern();
	// ---------- Pattern 1 ---------- //
	/* All the motors vibrate contemporary, and the
	   vibration is 'alternate' when the elastic force
	   is different than 0 (we are penetrating a tissue */
	if (feedback_pattern == F_PATTERN_1) {
		/*for (int i_motor = 0; i_motor <= 3; i_motor++) {
			armBand_motorsi(i_motor) = motor_force;
		}*/
		if (elastic(1) == 0) {
			if (IS_PUNCTURING && START_PENETRATION) {
				k = 0;
				START_PENETRATION = false;
				armBand_motorsi.setZero();
			}
			if (IS_PUNCTURING && (k == 2 || k == 3 || k == 4)) {
				for (int i_motor = 0; i_motor <= 3; i_motor++)
				{
					armBand_motorsi(i_motor) = 200;
				}
			}
			else if (IS_PUNCTURING && (k == 5)) {
				armBand_motorsi.setZero();
				IS_PUNCTURING = false;
			}
			else {
				for (int i_motor = 0; i_motor <= 3; i_motor++)
				{
					armBand_motorsi(i_motor) = motor_force;
				}
			}
			k++;
			if (k == 13) k = 0;
		}
		else {
			IS_PUNCTURING = true;
			START_PENETRATION = true;
			if (k == 13) k = 0;
			if (k == 0 || k == 1 || k == 2) armBand_motorsi(0) = motor_force;
			if (k == 3 || k == 4 || k == 5) armBand_motorsi(1) = motor_force;
			if (k == 6 || k == 7 || k == 8) armBand_motorsi(2) = motor_force;
			if (k == 9 || k == 10 || k == 11) armBand_motorsi(3) = motor_force;
			if (k == 12) armBand_motorsi.setZero();
			k++;
		}
	}

	// ---------- Pattern 2 ---------- //
	/* The intensity of the force sent to
	   the motors is the same during the
	   entire process. The vibration is
	   continue during the tissue break.
	   The vibration during penetration occurs
	   intermittently and the frequency increases
	   according to the depth of the tissue.*/
	if (feedback_pattern == F_PATTERN_2) {
		// TODO:
		/* Rendering of the vibration when the needle
		   is coming back to the initial configuration*/

		   // Debug print
		   //std::cout << "motor force: " << motor_force << std::endl;
		if (motor_force != 0) motor_force = 90.0;

		if (elastic(1) >= 1e-3) IS_PUNCTURING = true;
		else if (elastic(1) == 0) IS_PUNCTURING = false;

		if (IS_PUNCTURING != TMP_IS_PUNCTURING) {
			IS_TRANSICTION = !IS_TRANSICTION;
			TMP_IS_PUNCTURING = IS_PUNCTURING;
		}

		if (IS_TRANSICTION) {
			IS_TRANSICTION = false;
			DIV--;
		}

		// Debug print
		/*
		std::cout << "elastic force: " << elastic(1) << std::endl;
		std::cout << "IS_PUNCTURING: " << IS_PUNCTURING << std::endl;
		std::cout << "IS_TRANSICTION: " << IS_TRANSICTION << std::endl;
		std::cout << "TMP_IS_PUNCTURING: " << TMP_IS_PUNCTURING << std::endl;
		std::cout << "DIV: " << DIV << std::endl;
		std::cout << "________________________________" << std::endl;
		*/

		if (IS_PUNCTURING || UPRISING) {
			for (int i_motor = 0; i_motor <= 3; i_motor++) {
				if (UPRISING) motor_force = 45.0;
				armBand_motorsi(i_motor) = motor_force;
			}
		}
		else if (!IS_PUNCTURING) {
			if (i % DIV == 0) {
				for (int i_motor = 0; i_motor <= 3; i_motor++) {
					armBand_motorsi(i_motor) = motor_force;
				}
			}
		}
	}

	// ---------- Pattern 3 ---------- //
	/* Each time a tissue is penetrated, a
	   different motor activates. Initially,
	   the vibration is alternate; after the tissue break,
	   the vibration becomes continue. 
	   MOTOR_STATE: {
					-1 -> Inactive
					 0 -> Ready
					 1 -> Alternate mode
					 2 -> Continuous mode
		}*/
	// TODO: implement friction tissue switch during uprising
	if (feedback_pattern == F_PATTERN_3) {
		if (elastic(1) > 1e-3) {
			// Update number of layers penetrated
			if (!LAYERS_UPDATED) {
				LAYERS_PASSED++;
				LAYERS_UPDATED = true;
			}

			// Reset motor status after the 4th penetration
			// (eventually change with the first motor activated)
			// TODO: test with more than 4 layers
			if (LAYERS_PASSED > NUM_MOTORS) {
				int motor_index = (LAYERS_PASSED - 1) % NUM_MOTORS;
				MOTOR_STATE[motor_index] = 0;
			}

			// Set status
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				if ((MOTOR_STATE[i_motor] == 0)) {
					MOTOR_STATE[i_motor] = 1;
					PENETRATED[i_motor] = true;
				}
			}
			// Assign vibration value
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				if (!P2_ALT_SOLO) {
					if ((MOTOR_STATE[i_motor] == 1) && (i % 3 == 0)) {
						if (MOTOR_ELASTIC_F[i_motor] == -1)
							MOTOR_ELASTIC_F[i_motor] = motor_force_elas;
						armBand_motorsi(i_motor) = MOTOR_ELASTIC_F[i_motor];
					}
					else if (MOTOR_STATE[i_motor] == 2)
						armBand_motorsi(i_motor) = MOTOR_FRICTION_F[i_motor];
				}
				else {
					if ((MOTOR_STATE[i_motor] == 1) && (i % 3 == 0)) {
						if (MOTOR_ELASTIC_F[i_motor] == -1)
							MOTOR_ELASTIC_F[i_motor] = motor_force_elas;
						armBand_motorsi(i_motor) = MOTOR_ELASTIC_F[i_motor];
					}
				}
			}
		}
		else if (elastic(1) == 0) {
			// Update flag to update number of layers
			LAYERS_UPDATED = false;

			// Check status
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				if (PENETRATED[3]) MOTOR_STATE[3] = 2;
				// TODO: check if <= needed with 4 motor active
				if (LAYERS_PASSED < NUM_MOTORS) {
					if ((i_motor > 0) && (MOTOR_STATE[i_motor] == -1) && (MOTOR_STATE[i_motor - 1] == 1)) {
						MOTOR_STATE[i_motor - 1] = 2;
						MOTOR_STATE[i_motor] = 0;
					}
				}
				else {		// TODO: test with more layers
					if (MOTOR_STATE[i_motor] == 1) {
						MOTOR_STATE[i_motor] = 2;
					}
				}
			}
			// Assign vibration value
			for (int i_motor = 0; i_motor <= 3; i_motor++)
			{
				if (MOTOR_STATE[i_motor] == 2) {
					if (MOTOR_FRICTION_F[i_motor] == -1)
						MOTOR_FRICTION_F[i_motor] = motor_force_fric;
					armBand_motorsi(i_motor) = MOTOR_FRICTION_F[i_motor];
				}
			}
		}
	}
	// Zeroed force when needle is out of the body
	if (friction(1) == 0 && UPRISING)
		armBand_motorsi.setZero();

	// Debug print
	std::cout << "____________________________________" << std::endl;
	std::cout << "motor state = [" << MOTOR_STATE[0] << "|" << MOTOR_STATE[1] << "|" << MOTOR_STATE[2] << "|" << MOTOR_STATE[3] << "]" << std::endl;
	std::cout << "motor forces = [" << armBand_motorsi[0] << "|" << armBand_motorsi[1] << "|" << armBand_motorsi[2] << "|" << armBand_motorsi[3] << "]" << std::endl;
	std::cout << "____________________________________" << std::endl;

	devices.at(0).run(armBand_motorsi);
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

	// Parameters for the sendCustom function
	int i = 0;
	int j = 0;

	while (this->isRunning()) {

		// Measure starting time
		tic = clock.getCurTime();

		//----------------------------------------------------------------//
		// Do stuff here... 

		//TEST
		//this->hapticState.force(1) = 0.005 * sin(time_);

		// Update counter
		// TODO: check better systems to update counter
		i++;
		j++;
		if (i == 20) i = 0;
		if (j == 14) j = 0;

		// Send the haptic force on the device
		this->sendForceCustom(this->hapticState.force, i, j);

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