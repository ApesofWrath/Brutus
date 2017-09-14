#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DriveController.h>
#include <Flywheel.h>
#include <Elevator.h>
#include <GearRail.h>
#include <GroundPickup.h>
#include <Conveyor.h>
#include <Climber.h>
#include <TeleopStateMachine.h>
#include <Autonomous.h>
#include <LEDLightStrip.h>
#include <TrapezoidalProfile.h>

#define PI 3.14159265

class Robot: public frc::IterativeRobot {
public:
	DriveController *drive_controller;
	Joystick *joyOp, *joyThrottle, *joyWheel;
	Flywheel *fly_wheel;
	Elevator *elevator_;
	GearRail *gear_rail;
	Conveyor *conveyor_;
	Vision *vision_;
	TeleopStateMachine *teleop_state_machine;
	Autonomous *autonomous_;
	Climber *climber_;
	LEDLightStrip *light_;
	Compressor *compressor;
	PowerDistributionPanel *pdp;
	CameraServer *cam;
	GroundPickup *ground_pickup;
	TrapezoidalProfile *test_ground_pickup_profiler;

	const int JOY_OP = 1;
	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 2;

	const int GEAR_LIGHT_BUTTON = 8;
	const int BALL_LIGHT_BUTTON = 9;
	const int GEAR_AND_BALL_LIGHT_BUTTON = 10;
	const int CLIMB_BUTTON = 12;
	const int RAIL_OPEN_BUTTON = 778;
	const int RAIL_CLOSE_BUTTON = 88;
	const int FIRE_BUTTON = 4;
	const int POPCORN_BUTTON = 2;
	const int FIRE_BUTTON_2 = 1;
	const int STOP_SHOOT_BUTTON = 3;
	const int GEAR_PICKUP_BUTTON = 7;
	const int GEAR_SCORE_BUTTON = 8;
	const int OUTTAKE_BUTTON = 99;
	const int INTAKE_BUTTON = 99;
	const int ARM_UP_BUTTON = 99;
	const int ARM_DOWN_BUTTON = 99;
	const int RETURN_BUTTON = 6; //returning to wait for button state from climbing or gear

	const int HEADING_CONTROL_BUTTON = 6;
	const int VISION_TRACK_BUTTON = 5;
	const int FC_BUTTON = 1;
	const int REG_BUTTON = 2;

	const int STARTING_POS = PI / 2.0;

	frc::SendableChooser<std::string> autonChooser;
	frc::SendableChooser<std::string> allianceChooser;

	const std::string gearPlacementAndShoot = "Gear Placement and Shoot"; //move forward place a gear and then shoot the ten pre loaded balls
	const std::string gearPlacementUsualAuton = "Gear Placement Usual"; //gear to middle position
	const std::string gearPlacementUsualVision = "Gear Placement Usual Vision";
	const std::string gearPlacementRight = "Gear Placement Right"; //gear to side position
	const std::string gearPlacementLeft = "Gear Placement Left";
	const std::string shootAuton = "Shoot"; //shoot with pre-loaded balls
	const std::string shootAndLoadAuton = "Shoot and Load"; //get balls from hopper, then shoot
	const std::string driveForward = "Drive Forward";
	const std::string fancyDriveForward = "Fancy Drive Forward";
	const std::string stopAuton = "Stop Auton";

	const std::string redAlliance = "Red Alliance";
	const std::string blueAlliance = "Blue Alliance";

	std::string autoSelected;
	std::string allianceSelected;

	const std::string version = "1.0";

	const int HDrive = 0;
	const int Split = 1;
	const int Vis = 2;
	int driveMode = HDrive; //0 = HDRIVE 1 = split
	const int FC = 0;
	const int Reg = 1;
	int Type = Reg;

	std::vector<std::vector<double>> test_ground_pickup_profile = { };bool insert_profile =
			true;
	int state = 3;
	int test_ground_pickup_index = 0;

	bool is_heading; //used in hdrive wrapper to call the right drive function
	bool is_vision;bool is_fc;

	bool blue = false;bool red = false;

	double refT = 0.0;
	double velT = 0.0;
	double last_refT = 0.0;
	double last_velT = 0.0;

	Timer *time = new Timer();

	void RobotInit() {

		pdp = new PowerDistributionPanel(1);

		joyOp = new Joystick(JOY_OP);
		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);

		fly_wheel = new Flywheel();

		elevator_ = new Elevator();

		gear_rail = new GearRail();

		conveyor_ = new Conveyor();

		vision_ = new Vision();

		ground_pickup = new GroundPickup();

		drive_controller = new DriveController(vision_);

		autonomous_ = new Autonomous(drive_controller, elevator_, conveyor_,
				gear_rail, fly_wheel, ground_pickup);
		climber_ = new Climber();

		light_ = new LEDLightStrip();

		test_ground_pickup_profiler = new TrapezoidalProfile(0.25, 5, .01); //test ground pickup arm //max vel, max acc, time step

		teleop_state_machine = new TeleopStateMachine(fly_wheel, conveyor_,
				gear_rail, elevator_, drive_controller, vision_, climber_,
				ground_pickup);

		autonChooser.AddDefault(gearPlacementUsualAuton,
				gearPlacementUsualAuton);
		autonChooser.AddObject(gearPlacementAndShoot, gearPlacementAndShoot);
		autonChooser.AddObject(gearPlacementRight, gearPlacementRight);
		autonChooser.AddObject(gearPlacementLeft, gearPlacementLeft);
		autonChooser.AddObject(shootAuton, shootAuton);
		autonChooser.AddObject(shootAndLoadAuton, shootAndLoadAuton);
		autonChooser.AddObject(gearPlacementUsualVision,
				gearPlacementUsualVision);
		autonChooser.AddObject(driveForward, driveForward);
		autonChooser.AddObject(stopAuton, stopAuton);

		frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

		allianceChooser.AddDefault(blueAlliance, blueAlliance);
		allianceChooser.AddObject(redAlliance, redAlliance);

		frc::SmartDashboard::PutData("Alliance", &allianceChooser);

		is_heading = false;
		is_heading = false;
		is_fc = true;

		compressor = new Compressor(31);
		compressor->SetClosedLoopControl(true);

		cam = CameraServer::GetInstance();
		cam->StartAutomaticCapture(0);

		time->Start();

	}

	void AutonomousInit() override {

		autoSelected = autonChooser.GetSelected();

		allianceSelected = allianceChooser.GetSelected();

		fly_wheel->StartThread();

		ground_pickup->SetPos(STARTING_POS);
		ground_pickup->StartThreads();

		drive_controller->ZeroEncs(); //reset everything
		drive_controller->ZeroI();
		drive_controller->ahrs->ZeroYaw();
		drive_controller->ResetIndex();
		drive_controller->ResetVisionState();

		if (autoSelected == gearPlacementUsualAuton) { //choose auton sequence

			if (allianceSelected == redAlliance) {

				autonomous_->FillProfile("/home/lvuser/Gear_Profile_Red.csv");

			} else {

				autonomous_->FillProfile("/home/lvuser/Gear_Profile_Blue.csv");

			}

		} else if (autoSelected == gearPlacementUsualVision) {

			if (allianceSelected == redAlliance) {

			} else {

			}

		} else if (autoSelected == gearPlacementRight) {

			autonomous_->FillProfile("/home/lvuser/Gear_Right_Profile.csv");

		} else if (autoSelected == gearPlacementLeft) {

			autonomous_->FillProfile("/home/lvuser/Gear_Left_Profile.csv");

		} else if (autoSelected == driveForward) {

			autonomous_->FillProfile("/home/lvuser/Drive_Forward_Profile.csv");

		} else if (autoSelected == shootAuton) {

			if (allianceSelected == redAlliance) {

			} else {

			}

		} else if (autoSelected == shootAndLoadAuton) {

			if (allianceSelected == redAlliance) {

			} else {

			}

		} else if (autoSelected == gearPlacementAndShoot) {

			if (allianceSelected == redAlliance) {

			} else {

			}

		}

	}

	void AutonomousPeriodic() {

		conveyor_->ConStateMachine();
		elevator_->ElevatorStateMachine();
		fly_wheel->FlywheelStateMachine();
		//gear_rail->GearRailStateMachine();
		ground_pickup->GroundPickupStateMachine();

		autonomous_->RunAuton();

		//std::cout << "angle: " << drive_controller->ahrs->GetYaw() << std::endl;

	}

	void TeleopInit() {

		// 1. zero, 2. matlab/motionprofiles 3. vision? 4.vision profiles
		fly_wheel->DisableThreads(); //disable auton threads
		fly_wheel->StartThread(); //start teleop threads

		ground_pickup->DisableThreads();
		ground_pickup->StartThreads();

		drive_controller->DisableAutonThreads();
		drive_controller->StartTeleopThreads(joyThrottle, joyWheel, &is_heading,
				&is_vision, &is_fc);
		drive_controller->KickerDown();
		drive_controller->ahrs->ZeroYaw();
		drive_controller->ZeroI();

		ground_pickup->ZeroPos(); //TODO: take this out

		teleop_state_machine->Initialize(); //sets the state back to init

	}

	void TeleopPeriodic() {

//		std::cout<<"R1: "<< drive_controller->canTalonBackRight->GetOutputCurrent();//<<std::endl;
//		std::cout<<" R2: "<< drive_controller->canTalonFrontRight->GetOutputCurrent();//<<std::endl;
//		std::cout<<" L1: "<< drive_controller->canTalonBackLeft->GetOutputCurrent();//<<std::endl;
//		std::cout<< " L2: "<< drive_controller->canTalonFrontLeft->GetOutputCurrent()<<std::endl;

		SmartDashboard::PutNumber("Azimuth", vision_->findAzimuth());

		bool rail_open_button = joyOp->GetRawButton(RAIL_OPEN_BUTTON);
		bool rail_close_button = joyOp->GetRawButton(RAIL_CLOSE_BUTTON);
		bool stop_shoot_button = joyOp->GetRawButton(STOP_SHOOT_BUTTON);
		bool fire_button = joyOp->GetRawButton(FIRE_BUTTON);
		bool climb_button = joyOp->GetRawButton(CLIMB_BUTTON);
		bool return_button = joyOp->GetRawButton(RETURN_BUTTON);
		bool gear_pickup_button = joyOp->GetRawButton(GEAR_PICKUP_BUTTON);
		bool gear_score_button = joyOp->GetRawButton(GEAR_SCORE_BUTTON);
		bool outtake_button = joyOp->GetRawButton(OUTTAKE_BUTTON);
		bool intake_button = joyOp->GetRawButton(INTAKE_BUTTON);
		bool arm_up_button = joyOp->GetRawButton(ARM_UP_BUTTON);
		bool arm_down_button = joyOp->GetRawButton(ARM_DOWN_BUTTON);
		bool popcorn_button = joyOp->GetRawButton(POPCORN_BUTTON);
		bool second_fire_button = joyOp->GetRawButton(FIRE_BUTTON_2);

		bool gear_light_button = joyOp->GetRawButton(GEAR_LIGHT_BUTTON);
		bool ball_light_button = joyOp->GetRawButton(BALL_LIGHT_BUTTON);
		bool gear_and_ball_light_button = joyOp->GetRawButton(
				GEAR_AND_BALL_LIGHT_BUTTON);

		teleop_state_machine->StateMachine(rail_open_button, rail_close_button,
				fire_button, climb_button, return_button, popcorn_button,
				second_fire_button, stop_shoot_button, gear_pickup_button,
				gear_score_button, intake_button, outtake_button, arm_up_button,
				arm_down_button);

		//light_->LEDStateMachine(gear_light_button, ball_light_button,
		//		gear_and_ball_light_button);
		conveyor_->ConStateMachine();
		elevator_->ElevatorStateMachine();
		fly_wheel->FlywheelStateMachine();
		//	gear_rail->GearRailStateMachine();
		climber_->ClimberStateMachine();
		ground_pickup->GroundPickupStateMachine();

		//START DRIVE CODE
		const int HDrive = 0;
		const int Heading = 1;
		const int Vis = 2;

		bool headingDrive = joyWheel->GetRawButton(HEADING_CONTROL_BUTTON);
		bool visionTrack = joyWheel->GetRawButton(VISION_TRACK_BUTTON);

		switch (driveMode) {

		case HDrive:

			is_heading = false; //used in hdrive wrapper to call the right function
			is_vision = false;

			if (headingDrive) {

				drive_controller->StopAll();

				drive_controller->KickerDown(); //Kicker to stay down the whole time

				drive_controller->SetInitHeading();

				driveMode = Heading;

			} else if (visionTrack) {

				drive_controller->StopAll();

				drive_controller->KickerDown(); //Kicker to stay down the whole time

				drive_controller->SetInitHeading();

				drive_controller->SetAngle();

				driveMode = Vis;

			}

			break;

		case Heading:

			is_heading = true;

			if (!headingDrive) {

				drive_controller->StopAll();

				drive_controller->KickerDown();

				driveMode = HDrive;

			}

			break;

		case Vis:

			is_vision = true;

			//std::cout << "VIS" << std::endl;

			if (!visionTrack) {

				drive_controller->StopAll();

				drive_controller->KickerDown();

				driveMode = HDrive;

			}

			break;

		}
		//END DRIVECODE

		const int FC = 0; //Field Centric
		const int Reg = 1;

		bool fcButton = joyThrottle->GetRawButton(FC_BUTTON);
		bool regButton = joyThrottle->GetRawButton(REG_BUTTON);

		switch (Type) {

		case FC:

			SmartDashboard::PutString("Drive: ", "Field Centric");

			is_fc = true; //used in hdrive wrapper to call the right function

			if (regButton) {

				Type = Reg;

			}

			break;

		case Reg:

			SmartDashboard::PutString("Drive: ", "Regular");

			is_fc = false;

			if (fcButton) {

				Type = FC;

			}

			break;

		}

		if (joyThrottle->GetRawButton(3)) {

			drive_controller->ahrs->ZeroYaw();

		}

	} // TeleopPeriodic

	void DisabledInit() override {

		drive_controller->DisableTeleopThreads();
		drive_controller->DisableAutonThreads();

		fly_wheel->DisableThreads();

		ground_pickup->DisableThreads();

		refT = 0.0;
		velT = 0.0;

		drive_controller->ZeroI();
		ground_pickup->ClearIAccum();
	}

	void TestPeriodic() { //1-up 2-down 3-wfb 4-arm 5-arm 6-spin negative

		//std::cout << vision_->findAzimuth() << std::endl;
		std::cout << "state" << state << std::endl;

		if (joyOp->GetRawButton(3)) { //can go to state 2 at any time
			state = 2;
			insert_profile = true;
		}

	//	ground_pickup->canTalonPickupWheel->Set(0.25);


//		if (joyOp->GetRawButton(4)) {
//			ground_pickup->canTalonFloorPickupArm->Set(0.3);
//		}
//
//		else if (joyOp->GetRawButton(5)) {
//			ground_pickup->canTalonFloorPickupArm->Set(-0.3);
//		}
//
//		else {
//			ground_pickup->canTalonFloorPickupArm->Set(0);
//		}
		//if(joyOp->GetRawButton(11)) {


		//}

		std::cout << "current" << ground_pickup->canTalonPickupWheel->GetOutputCurrent() << std::endl;
	//	ground_pickup->canTalonFloorPickupArm->Set(1.0);
		std::cout << -ground_pickup->GetPos() << "   " << ground_pickup->GetVel() << std::endl;

		//    std::cout << "time: "  << time->Get() << std::endl;
		switch (state) {

		case 0: //up - not profiled

			//std::cout << "Up" << std::endl;

			if (time->HasPeriodPassed(.01)) {
				ground_pickup->MoveArm(refT, velT); //ref, profileVelocity
				time->Reset();
				last_velT = velT;
				last_refT = refT;
				if (refT < 25.0) {
					refT += (last_velT * .01);
				}
				if (velT < 25.0){
					if (refT < 1.0){
						velT += 23.0 * .01;
					}else if (velT > 0.1){
						velT -= 23.0 * .01;
					}
				}
			}
			if (joyOp->GetRawButton(2)) {
				state = 1;
				time->Reset();
			}




			break;

		case 1: //down - profiled

			//std::cout << "Down" << std::end

			if (time->HasPeriodPassed(.01)) {
				ground_pickup->MoveArm(0,0); //ref, profileVelocity
				time->Reset();
			}
			if (joyOp->GetRawButton(1)) {
				test_ground_pickup_index = 0; //for next time
				insert_profile = true;

				state = 0;
				time->Reset();
			}

			ground_pickup->canTalonPickupWheel->Set(-0.3);

			break;

		case 2: //wait for button state



			refT = 0.0;
			velT = 0.0;

			insert_profile = true;

			if (joyOp->GetRawButton(1)) {
				state = 0;
				time->Reset();

			} else if (joyOp->GetRawButton(2)) {
				state = 1;
				time->Reset();
			}

		//ground_pickup->canTalonFloorPickupArm->Set(0);

			if (ground_pickup->canTalonPickupWheel->GetOutputCurrent() < 6.0){
						ground_pickup->canTalonPickupWheel->Set(.3);
			} else {
				ground_pickup->canTalonPickupWheel->Set(0);
				state = 0;
			}


			break;

		case 3:

			ground_pickup->ZeroPos();
			state = 2;

			break;

		}

	}

private:

};

START_ROBOT_CLASS(Robot)
