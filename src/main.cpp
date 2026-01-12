#include "main.h"
#include "utils.h"
#include "Localization/Localization.h"
#include "Localization/Tracker/MotorTracker.h"
#include "PathFollowing/Feedback/Ramsette.h"
#include "Eigen/Eigen"
#include <chrono>
#include "PathGenerator/SplineTrajectoryGenerator.h"
#include "TaskScheduler/Task/taskInterface.h"
#include "TaskScheduler/taskScheduler.h"
#include "TaskScheduler/Task/MotorTask.h"
#include "TaskScheduler/Task/PneumaticTask.h"
#include "PathFollowing/Feedback/PID.h"
#include "PathFollowing/FeedForward/feedforward.h"
#include "Localization/particle_filter.h"
#include "Commands/RamseteCmd.h"

using namespace pros;

pros::Controller master(pros::E_CONTROLLER_MASTER);


// Input Drive motors here :


MotorGroup LeftDrive({2, 4},pros::v5::MotorGears::green,pros::v5::MotorUnits::degrees);
MotorGroup RightDrive({-1, -3},pros::v5::MotorGears::green,pros::MotorEncoderUnits::degrees);


// Configure Motor Groups here :

// Wheel diameter in inches, gear ratio, motor rpm

MotorConfig left_config = {2.75f, 1.0f, 200};
MotorConfig right_config = {2.75f, 1.0f, 200};


// IMU Sensor here :

IMU imu_sensor(10);


// Starting Pose here :

Pose start_pose = {0.0f, 0.0f, 0.0f};



// Intake and other motors/devices here :
/*
Motor intake = Motor(4);
Motor upper_intake = Motor(5);
ADIDigitalOut match_loader = ADIDigitalOut('F');
ADIDigitalOut upper_intake_piston = ADIDigitalOut('G');
ADIDigitalOut doinker = ADIDigitalOut('H');
*/

// Odometry and Localization setup here :
ConfiguredMotorGroup left_motors = ConfiguredMotorGroup(left_config, LeftDrive);
ConfiguredMotorGroup right_motors = ConfiguredMotorGroup(right_config, RightDrive);


//Define the tracker interface and what type of trackers are being used

// Encoders or Odometry
TrackerType tracker_type = ENCODERS;


//FwdSensorTracker, SideSensorTracker, FwdSideSensorTracker, and MotorTracker can also be used here
MotorTracker tracker = MotorTracker(left_motors, right_motors);


//Configure Distance Sensors here :
/*
std::vector<DistanceSensorWrapper> distance_sensors = {
	DistanceSensorWrapper(1,SensorIndex::FRONT), // Front
	DistanceSensorWrapper(2,SensorIndex::BACK), // Back
	DistanceSensorWrapper(3,SensorIndex::LEFT), // Left
	DistanceSensorWrapper(4,SensorIndex::RIGHT)  // Right
};


//Define walls of the field here along with any obstacles at the height of the distance sensors
//IMPORTANT: All distance sensors should be at the same height for accurate localization

std::vector<Wall> walls = {
	{{0.0f, 0.0f}, {0.0f, 144.0f}, "Left Wall"},
	{{0.0f, 144.0f}, {144.0f, 144.0f}, "Top Wall"},
	{{144.0f, 144.0f}, {144.0f, 0.0f}, "Right Wall"},
	{{144.0f, 0.0f}, {0.0f, 0.0f}, "Bottom Wall"}
};


*/
// Particle Filter setup here :

// Number of particles to use
int num_particles = 500;



//ignore below this line
Odom odom = Odom(tracker, imu_sensor);
Localization loc = Localization(&odom);
//DistanceLocalization d_loc = DistanceLocalization(distance_sensors);
//ParticleFilter pf = ParticleFilter(&d_loc, &loc, ENCODERS);



void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Initializing...");
	
	loc.CalibrateIMU();
	LeftDrive.set_zero_position_all(0);
	RightDrive.set_zero_position_all(0);
	LeftDrive.tare_position_all();
	RightDrive.tare_position_all();
	loc.setGlobalPose(start_pose);
	loc.StartOdomTracking();
	//pf.StartFilterTask(num_particles, &walls, 144.0f, 144.0f);
	
	pros::lcd::print(1, "Ready");
}

void disabled() {}


void competition_initialize() {}


void DriveToPoint(Point target)
{
	float distance = 10;
	while(distance > 3)
	{
		Pose current = loc.GetGlobalPose();

		PID linearPID = PID(2.3, 0.8, 0.01);
		PID angularPID = PID(0.3, 0.1, 0.01);

		linearPID.UpdateIntegralValues(3, 10);
		angularPID.UpdateIntegralValues(utils::toRadians(5), utils::toRadians(20));

		float dx = target.x - current.x;
		float dy = target.y - current.y;
		distance = sqrt(dx*dx + dy*dy);
		float target_heading = atan2(dy, dx);

		linearPID.UpdateTarget(distance, 3);
		angularPID.UpdateTarget(target_heading, utils::toRadians(5));


		float linear_velocity = linearPID.UpdateValues(0);
		float max_linear_velocity = 140;
		if(linear_velocity > max_linear_velocity) linear_velocity = max_linear_velocity;
		if(linear_velocity < -max_linear_velocity) linear_velocity = -max_linear_velocity;

		float angular_velocity = angularPID.UpdateValues(current.heading);
		if(angular_velocity > utils::toRadians(90)) angular_velocity = utils::toRadians(90);
		if(angular_velocity < -utils::toRadians(90)) angular_velocity = -utils::toRadians(90);

		LeftDrive.move_velocity((linear_velocity - utils::toDegrees(angular_velocity)));
		RightDrive.move_velocity((linear_velocity + utils::toDegrees(angular_velocity)));

		delay(20);
	}
	LeftDrive.move_velocity(0);
	RightDrive.move_velocity(0);
}

void turnPID(float target_angle)
{
	float kP = 0.7;
	float kI = 0.25;
	float kD = 0.01;
	float angle_error = 10;
	float integral = 0;
	float derivative = 0;
	float max_int = 20;
	float int_bound = 5;
	float prev_error = 0;
	while(abs(angle_error) > 2)
	{
		while(target_angle > 180) target_angle -= 360;
		while(target_angle < -180) target_angle += 360;

		Pose current = loc.GetGlobalPose();
		
		angle_error = target_angle - utils::toDegrees(current.heading);

		while(angle_error > 180) angle_error -= 360;
		while(angle_error < -180) angle_error += 360;

		integral += angle_error;
		if(integral > max_int) integral = max_int;
		if(angle_error < int_bound) integral = 0;
		derivative = angle_error - prev_error;
		prev_error = angle_error;
		
		float angular_velocity = (kP*angle_error)+(kI*integral)+(kD*derivative);
		if(angular_velocity > 90) angular_velocity = 90;
		if(angular_velocity < -90) angular_velocity = -90;

		LeftDrive.move_velocity(-angular_velocity);
		RightDrive.move_velocity(angular_velocity);

		delay(20);
	}
	LeftDrive.move_velocity(0);
	RightDrive.move_velocity(0);
}

void forwardPID(float target_distance)
{
	float start_x = loc.GetGlobalPose().x;
	float target_x = start_x + target_distance; // relative

	lcd::print(5,"start: %.2f",start_x);

	PID linearPID = PID(10, 0.01, 0.01);
	linearPID.UpdateIntegralValues(.5, 5);
	linearPID.UpdateTarget(target_x, 1);

	while(true)
	{
		float current_x = loc.GetGlobalPose().x;

		float error = target_x - current_x;
		lcd::print(6,"dx: %.2f",error);
		if (std::fabs(error) <= 1) {
            break;
        }

		float linear_velocity = linearPID.UpdateValues(current_x);
		float max_velocity = 200;
		//if(linear_velocity > max_linear_velocity) linear_velocity = max_linear_velocity;
		//if(linear_velocity < -max_linear_velocity) linear_velocity = -max_linear_velocity;
		
		double kV = 12000.0 / max_velocity;
		double voltage = std::clamp(linear_velocity, -max_velocity, max_velocity) * kV;
		LeftDrive.move_voltage(voltage);
		RightDrive.move_voltage(voltage);

		delay(20);
	}
	LeftDrive.move_velocity(0);
	RightDrive.move_velocity(0);
}


Eigen::Vector2f getRamsetteControl(Pose& current_pose, Trajectory& target_trajectory, Ramsette& ramsette)
{
	Eigen::Vector2f target_trajectory_vector = Eigen::Vector2f(target_trajectory.linearVelocity, target_trajectory.angularVelocity);
	return ramsette.CalculateControl(current_pose, target_trajectory.position, target_trajectory_vector);
}

Eigen::Vector2f getLeftRightVoltages(Eigen::Vector2f& ramsette_controls, Trajectory& target_trajectory)
{
	double targetLeftVeloc = (ramsette_controls(0)-(ramsette_controls(1)*LocalizationConfig::TRACK_WIDTH/2)) / (M_PI*left_config.wheel_diameter);
	double targetRightVeloc = (ramsette_controls(0)+(ramsette_controls(1)*LocalizationConfig::TRACK_WIDTH/2)) / (M_PI*right_config.wheel_diameter);
	
	double max_speed = 12000;
	double targetLeftVoltage = targetLeftVeloc/(left_config.motor_rpm*left_config.gear_ratio/60)*max_speed;
	double targetRightVoltage = targetRightVeloc/(right_config.motor_rpm*right_config.gear_ratio/60)*max_speed;

	if(targetLeftVoltage > max_speed) targetLeftVoltage = max_speed;
	if(targetLeftVoltage < -max_speed) targetLeftVoltage = -max_speed;
	if(targetRightVoltage > max_speed) targetRightVoltage = max_speed;
	if(targetRightVoltage < -max_speed) targetRightVoltage = -max_speed;

	return Eigen::Vector2f(targetLeftVoltage, targetRightVoltage);
}

Eigen::Vector2f getFeedforwardVoltages(Trajectory &target_trajectory, Feedforward leftFeedForward, Feedforward rightFeedForward, double targetLinAccel, double targetAngAccel)
{
	double targetLeftVeloc = (target_trajectory.linearVelocity-(target_trajectory.angularVelocity*LocalizationConfig::TRACK_WIDTH/2.0)) / (M_PI*left_config.wheel_diameter);
	double targetRightVeloc = (target_trajectory.linearVelocity+(target_trajectory.angularVelocity*LocalizationConfig::TRACK_WIDTH/2.0)) / (M_PI*right_config.wheel_diameter);
	
	double max_speed = 12000;

	double targetLeftAccel  = targetLinAccel - (LocalizationConfig::TRACK_WIDTH / 2.0) * targetAngAccel;
	double targetRightAccel = targetLinAccel + (LocalizationConfig::TRACK_WIDTH / 2.0) * targetAngAccel;
	double targetLeftVoltage = leftFeedForward.GetTargetVoltage(targetLeftVeloc,targetLeftAccel)*1000.0;
	double targetRightVoltage = rightFeedForward.GetTargetVoltage(targetRightVeloc,targetRightAccel)*1000.0;
	master.print(1,1,"L: %.2f R: %.2f",targetLeftVoltage, targetRightVoltage);

	//double targetLeftVoltage = targetLeftVeloc/(left_config.motor_rpm*left_config.gear_ratio/60)*max_speed;
	//double targetRightVoltage = targetRightVeloc/(right_config.motor_rpm*right_config.gear_ratio/60)*max_speed;

	if(targetLeftVoltage > max_speed) targetLeftVoltage = max_speed;
	if(targetLeftVoltage < -max_speed) targetLeftVoltage = -max_speed;
	if(targetRightVoltage > max_speed) targetRightVoltage = max_speed;
	if(targetRightVoltage < -max_speed) targetRightVoltage = -max_speed;

	return Eigen::Vector2f(targetLeftVoltage, targetRightVoltage);
}

Task* task_scheduler_task = nullptr;
float start_time;

void TaskSchedulerTask(void* param)
{
	TaskScheduler* taskScheduler = static_cast<TaskScheduler*>(param);
	
	float time;

	while (task_scheduler_task != nullptr && task_scheduler_task->get_state() != pros::E_TASK_STATE_DELETED) 
	{
		time = pros::millis() - start_time;
		taskScheduler->checkForTasks(time);
		delay(10);
	}
}

void main_loop(QuinticSpline& path, Ramsette& ramsette, PID& left_PID, PID& right_PID, Feedforward& left_feedforward, Feedforward& right_feedforward, std::vector<Trajectory>& point_list, float start_time, float PID_settle_error)
{
	float time;
	float prev_time = 0;

	while(1)
	{
		time = pros::millis() - start_time;

		//if(path.GetTrajectory(time/1000).position == point_list.back().position)
		if(time/1000 > point_list.back().time)
		{
			//master.print(1,1,"Test: %.2f",1.0);
			Pose current = loc.GetGlobalPose();
			float dx = point_list.back().position.x - current.x;
			float dy = point_list.back().position.y - current.y;
			float dh = point_list.back().position.heading - current.heading;
			//pros::lcd::print(3,"dx: %.2f, dy: %.2f", dx, dy);
			if(abs(dx) < 1 && abs(dy) < 1)
			{
				LeftDrive.move_voltage(0);
				RightDrive.move_voltage(0);
				break;
			}
		}
		float dt = time-prev_time;
		float estimated_next_time = time+dt;
		float estimated_prev_time = time-dt;

		Trajectory target_trajectory = path.GetTrajectory(time/1000);
		Trajectory target_trajectory_next = path.GetTrajectory(estimated_next_time/1000);
		Trajectory target_trajectory_prev = path.GetTrajectory(estimated_prev_time/1000);

		Pose current_pose = loc.GetGlobalPose();
		Eigen::Vector2f ramsette_controls = getRamsetteControl(current_pose, target_trajectory, ramsette);
		//lcd::print(5, "Act: %.2f, Exp: %.2f", ramsette_controls(1), path.GetTrajectory(time/1000).angularVelocity);
		//lcd::print(5, "Act: %.2f, Exp: %.2f", current_pose.heading, path.GetTrajectory(time/1000).position.heading);
		//lcd::print(5,"Vel: %.2f, Ang: %.2f",ramsette_controls(0),ramsette_controls(1));
		//lcd::print(6,"X: %.2f",path.GetTrajectory(time/1000).position.x);
		//lcd::print(7,"X2: %.2f",loc.GetGlobalPose().x);
		/*
		Eigen::Vector2f voltages = getLeftRightVoltages(ramsette_controls, target_trajectory);
		float targetLeftVoltage = voltages(0);
		float targetRightVoltage = voltages(1);

		left_PID.UpdateTarget(targetLeftVoltage, PID_settle_error);
		double leftVoltage = left_PID.UpdateValues(LeftDrive.get_voltage());

		right_PID.UpdateTarget(targetRightVoltage, PID_settle_error);
		double rightVoltage = right_PID.UpdateValues(RightDrive.get_voltage());
		*/
		double targetLinVel = ramsette_controls(0);
		double omega = ramsette_controls(1);

		double targetLeftVel = targetLinVel - omega * (LocalizationConfig::TRACK_WIDTH / 2.0);
		double targetRightVel = targetLinVel + omega * (LocalizationConfig::TRACK_WIDTH / 2.0);

		const double wheelCirc = M_PI * left_config.wheel_diameter;

		 targetLeftVel = (targetLeftVel / wheelCirc) * 60.0 * left_config.gear_ratio;
		 targetRightVel = (targetRightVel / wheelCirc) * 60.0 * right_config.gear_ratio;

		double actualLeftVel  = LeftDrive.get_actual_velocity();
		double actualRightVel = RightDrive.get_actual_velocity();

		left_PID.UpdateTarget(targetLeftVel, PID_settle_error);
		right_PID.UpdateTarget(targetRightVel, PID_settle_error);
		double leftVoltage = left_PID.UpdateValues(actualLeftVel);
		double rightVoltage = right_PID.UpdateValues(actualRightVel);


		float target_linear_accel = (target_trajectory_next.linearVelocity-target_trajectory_prev.linearVelocity)/((2*dt)/1000);
		float target_angular_accel = (target_trajectory_next.angularVelocity-target_trajectory_prev.angularVelocity)/((2*dt)/1000);

		Eigen::Vector2f feedforward_voltages = getFeedforwardVoltages(target_trajectory,left_feedforward,right_feedforward,target_linear_accel,target_angular_accel);
	
		LeftDrive.move_voltage(leftVoltage);
		RightDrive.move_voltage(rightVoltage); 

		
		prev_time = time;
		delay(10);
	}
}

FILE* usd_file_write = fopen("/usd/data.txt", "w");
char buffer[100];

void tuning() {
	float voltage = 0;
	float veloc = 0;
	float dt = 0.001;
	float prev_time = pros::millis();
	float time = pros::millis();
	float dist = 0;
	float prev_dist = 0;
	float prev_veloc = 0;
	float accel = 0;
	RightDrive.set_encoder_units(pros::MotorEncoderUnits::degrees);
	master.clear();
	RightDrive.tare_position();
	while(1)
	{
		if(master.get_digital_new_press(DIGITAL_UP)) voltage += 1000;
		if(master.get_digital_new_press(DIGITAL_DOWN)) voltage -=500;
		if(master.get_digital_new_press(DIGITAL_LEFT)) break;


		dist = (RightDrive.get_position()/360.0)*M_PI*3.25*(36.0/48.0);
		time = pros::millis();
		dt =(time-prev_time)/1000;
		float delta_d = dist-prev_dist;
		veloc = RightDrive.get_actual_velocity()*(36.0/48.0)*3.25*M_PI/60;
		float delta_v = veloc - prev_veloc;
		accel = delta_v/dt;
		
		if(!master.get_digital(DIGITAL_RIGHT)) master.print(1,1, "V1: %.2f", voltage);
		//master.print(2,1, "Veloc: %.2f", veloc);
		lcd::print(4,"Vol: %.2f Vel: %.2f", voltage,veloc);
		//master.print(2,1, "Accel: %.2f", accel);
		//snprintf(buffer, sizeof(buffer), "{Vol:%f,Vel:%f,A:%f}\n,", voltage, veloc, accel);
		//fputs(buffer, usd_file_write);
		if(master.get_digital(DIGITAL_RIGHT)) master.print(1,1,"veloc:%.2f", veloc);
		RightDrive.move_voltage(voltage);
		prev_dist = dist;
		prev_time = time;
		prev_veloc = veloc;
		delay(30);
	}

	//fclose(usd_file_write);
	return;
}

void autonomous() {
	
	LeftDrive.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	RightDrive.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	master.clear();

	std::vector<Trajectory> points_list1 = {
        {{0,0,0},0,0,0},
        {{72,0,M_PI/4},0,0,2},
        {{72,72,3*M_PI/4},0,0,4},
        {{-10,72,5*M_PI/4},0,0,6},
        {{0,0,7*M_PI/4},0,0,8},
	};

	Ramsette ramsette = Ramsette(2, 0.7);
	PID left_PID  = PID(0.3, 0.02, 0.005);
	PID right_PID = PID(0.3, 0.02, 0.005);

	// Optional: integral bounds
	left_PID.UpdateIntegralValues(100, 500);   // min/max integral clamp
	right_PID.UpdateIntegralValues(100, 500);

	float PID_settle_error = 700;

	Feedforward left_feedforward  = Feedforward(1.5, 0.35, 0.01);  // kS, kV, kA
	Feedforward right_feedforward = Feedforward(1.5, 0.35, 0.01);



	Pose target_pose;
	float time;
	float prev_time = 0;
	QuinticSpline path = QuinticSpline(points_list1, 80);
	
	start_time = pros::millis();
	main_loop(path, ramsette, left_PID, right_PID, left_feedforward, right_feedforward, points_list1, start_time, PID_settle_error);
	// 1. Load your path (your existing PathLoader)

	/*
	Path path1 = PathLoader::load("/static/blue.json");
    MotionProfile profile(path1);
    
    Ramsete* cmd = new Ramsete(&left_motors, &right_motors, &loc, &profile, 2.0, 0.7);
    cmd->initialize();  // Start timing
    
    while (!cmd->isFinished()) {
        cmd->execute();   // Run Ramsete + PID control
        pros::delay(10);  // 100Hz loop
    }
    
    cmd->end(false);    // ❌ MISSING! Stops motors
    delete cmd;         // ❌ MISSING! Free memory
    */

	//turnPID(-135);
	//DriveToPoint({10, -15});
	//forwardPID(24);
	/*
	while (true) {
		//pros::lcd::print(5,"x: %.2f", loc.GetGlobalPose().x);
		pros::delay(100);
	}
	*/
	
	//task_scheduler_task->remove();
}


bool matchLoaderActivated = false;
bool upperIntakeActivated = false;
bool doinkerActivated = false;

/*
void activateMatchLoader() {
	if (matchLoaderActivated) {
		matchLoaderActivated = false;
		match_loader.set_value(1);
	} else {
		matchLoaderActivated = true;
		match_loader.set_value(0);
	}
};
bool scoring_up = true;

void toggleUpperIntake() {
	scoring_up = !scoring_up;
	upperIntakeActivated = !upperIntakeActivated;
	upper_intake_piston.set_value(upperIntakeActivated);
}

void toggleMatchLoader() {
	matchLoaderActivated = !matchLoaderActivated;
	match_loader.set_value(matchLoaderActivated);
}

void raiseIntake()
{
	match_loader.set_value(1);
	upper_intake_piston.set_value(0);
	match_loader.set_value(0);
}

void toggleDoinker() {
	doinkerActivated = !doinkerActivated;
	doinker.set_value(doinkerActivated);
}
*/
void opcontrol() {
	autonomous();
	/*
	tuning();
	float right_multiplier = 1;
	float left_multiplier = 1;
	while (true) {
		// SAFER: Check if controller is connected before reading
		if (master.is_connected()) {
			float left_voltage = utils::convert_percent_to_voltage(utils::convert_joystick_exponential(master.get_analog(ANALOG_LEFT_Y)));
			left_voltage *= left_multiplier;

			float right_voltage = utils::convert_percent_to_voltage(utils::convert_joystick_exponential(master.get_analog(ANALOG_RIGHT_Y)));
			right_voltage *= right_multiplier;

			LeftDrive.move_voltage(left_voltage);
			RightDrive.move_voltage(right_voltage);
		}

		/*
		if(master.get_digital(DIGITAL_R1) || master.get_digital(DIGITAL_L1)) intake.move_voltage(13000);
		else if(master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_L2)) intake.move_voltage(-13000);
		else intake.move_voltage(0);

		if(master.get_digital(DIGITAL_L1)) upper_intake.move_voltage(13000);
		else if(master.get_digital(DIGITAL_L2))  upper_intake.move_voltage(-13000);
		else upper_intake.move_voltage(0);
		/
		delay(10);
	}*/
}