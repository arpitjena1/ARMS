#include "main.h"
#include "ARMS/chassis.h"
#include "ARMS/config.h"
#include "ARMS/flags.h"
#include "ARMS/odom.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pid.hpp"

pros::Controller master(CONTROLLER_MASTER);
pros::Motor lB(-6);
pros::Motor rF(9);
pros::Rotation leftrot(1);
pros::Rotation rightRot(-7);
pros::Motor intaketop(16);
pros::Motor intakebottom(12);
pros::MotorGroup intake({intaketop, intakebottom});
pros::Distance distancesens(21);

pros::Motor slapper(15);
pros::Imu imu(10);
pros::ADIDigitalOut liftpiston('C'); // look at solenoid port and put the thing in here
pros::ADIDigitalOut blocker('F'); // for this one too


static float rollAngle180(float angle) {
    while (angle < -180) {
        angle += 360;
    }

    while (angle >= 180) {
        angle -= 360;
    }

    return angle;
}
PID turnPID = PID(2.4, 0, 12, 5, 1.5, 200, 5, 300, 625);
PID swingPID = PID(2.4, 0, 12, 5, 1.5, 200, 5, 300, 625);
PID drivePID = PID(10, 0, 30, 0, 0.75, 200, 2, 500, 7000);
PID headingPID = PID(2.4, 0, 12, 5, 1.5, 200, 5, 300, 625);

void turn(float heading) {
    turnPID.reset();

    do {
        float error = rollAngle180(heading - arms::odom::imu->get_heading());
        float pidOutput = turnPID.update(0, -error);

        arms::chassis::arcade(0, pidOutput);

        pros::delay(20);
    } while (!turnPID.isSettled());

    arms::chassis::arcade(0, 0);
}
void swing(float heading, bool isLeft) {
    swingPID.reset();
	if(isLeft){
		do {
        float error = rollAngle180(heading - arms::odom::imu->get_heading());
        float pidOutput = swingPID.update(0, -error);

        arms::chassis::tank( pidOutput,0);

        pros::delay(20);
    } while (!swingPID.isSettled());
	} else{
		do {
        float error = rollAngle180(heading - arms::odom::imu->get_heading());
        float pidOutput = turnPID.update(0, -error);

        arms::chassis::tank(0, pidOutput);

        pros::delay(20);
    } while (!swingPID.isSettled());
	}
    

    arms::chassis::arcade(0, 0);
}	
void arc(double targetAngle, double leftScaler, double rightScaler) {
    swingPID.reset();

    do {
		float error = rollAngle180(targetAngle - arms::odom::imu->get_heading());
        double stepVal = swingPID.update(0, -error);
        
        double left = stepVal * leftScaler;
        double right = stepVal * rightScaler;

        arms::chassis::tank(left, right);
        pros::delay(10);
    } while (!swingPID.isSettled());

    arms::chassis::tank(0, 0);
}
void move(double dist){
	drivePID.reset();
	float beginningLeft = arms::chassis::leftMotors->get_positions()[0];
    float beginningRight = arms::chassis::rightMotors->get_positions()[0];
    do {
		float deltaLeft = arms::chassis::leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = arms::chassis::rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * 3.25 * M_PI * 5/3;
        float pidOutput = drivePID.update(dist, distanceTravelled);


        arms::chassis::tank(pidOutput,-pidOutput);

        pros::delay(20);
    } while (!drivePID.isSettled());
}

void movewithturn(double dist, double angle){
	drivePID.reset();
	headingPID.reset();
	float beginningLeft = arms::chassis::leftMotors->get_positions()[0];
    float beginningRight = arms::chassis::rightMotors->get_positions()[0];
    do {
		float headingError = rollAngle180(angle - arms::odom::imu->get_heading());
        float headingpidOutput = turnPID.update(0, -headingError);
		float deltaLeft = arms::chassis::leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = arms::chassis::rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * 3.25 * M_PI * 5/3;
        float pidOutput = drivePID.update(dist, distanceTravelled);


        arms::chassis::arcade(pidOutput,headingpidOutput);

        pros::delay(20);
    } while (!drivePID.isSettled() && !headingPID.isSettled());
}

void mtp(double x, double y, double theta, arms::MoveFlags moveFlags){
    using namespace arms::chassis;
    move({{y/2,-x, -theta}}, 127, moveFlags);

}
void mtp2(double x, double y, double theta, double speed, arms::MoveFlags moveFlags){
    using namespace arms::chassis;
    move({{y/2,-x, -theta}}, speed, moveFlags);

}
void mtp3(double x, double y, double speed, arms::MoveFlags moveFlags){
    using namespace arms::chassis;
    move({{y/2,-x}}, speed, moveFlags);

}
void chainedmtpangle(std::vector<arms::Point> poses, bool forwards, double endAngle, double exitErrorPerPoint){
	using namespace arms;
	if(forwards){
		for (arms::Point p : poses){
			if(poses.back().x == p.x && poses.back().y == p.y){
				mtp(p.x,p.y,endAngle, arms::NONE);
			}
		mtp3(p.x,p.y,127, arms::NONE | arms::ASYNC | arms::THRU);
		chassis::waitUntilFinished(exitErrorPerPoint);

	}
	} else{
		for (arms::Point p : poses){
			if(poses.back().x == p.x && poses.back().y == p.y){
				mtp(p.x,p.y,endAngle, arms::REVERSE);
			}
		mtp3(p.x,p.y,127, arms::REVERSE | arms::ASYNC | arms::THRU);
		chassis::waitUntilFinished(exitErrorPerPoint);

	}
	}
}
void chainedmtp(std::vector<arms::Point> poses, bool forwards, double exitErrorPerPoint){
	using namespace arms;
	if(forwards){
		for (arms::Point p : poses){
			if(poses.back().x == p.x && poses.back().y == p.y){
				mtp3(p.x,p.y,127, arms::NONE);
			}
		mtp3(p.x,p.y,127, arms::NONE | arms::ASYNC | arms::THRU);
		chassis::waitUntilFinished(exitErrorPerPoint);

	}
	} else{
		for (arms::Point p : poses){
			if(poses.back().x == p.x && poses.back().y == p.y){
				mtp3(p.x,p.y,127, arms::REVERSE);
			}
		mtp3(p.x,p.y,127, arms::REVERSE | arms::ASYNC | arms::THRU);
		chassis::waitUntilFinished(exitErrorPerPoint);

	}
	}
}
void cataTaskForSkills(){
	using namespace arms;
    slapper.move(127);
	pros:delay(5000);
    slapper.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    slapper.move(0);
	//chainedmtp({Point{0,1}, Point{0,2}}, 10);
}
void climb(){
    double start = pros::millis();
    arms::chassis::tank(127, 127);
    pros::delay(1000);
    while(!(arms::odom::imu->get_pitch()>= -1 && arms::odom::imu->get_pitch()<= 1)){
        arms::chassis::tank(127, 127);
    }
    arms::chassis::tank(-90, -90);
    pros::delay(1000);
    arms::chassis::tank(0, 0);
    arms::odom::reset({0,0});
}


void initialize() {
	
	arms::init();
	
}

void disabled() {
}

void competition_initialize() {
}



pros::ADIDigitalOut frontwings('A');
void rightside(){
using namespace arms;
intake.move(127);
pros::delay(350);
intake.move(30);
mtp(0,-30,0, REVERSE);
mtp(45, -110, -90, REVERSE);
mtp(15, -60, -90, NONE);
mtp(60, -120, -180, ASYNC);
pros::delay(550);
intake.move(-127);
chassis::waitUntilFinished(1);
odom::reset({0,0});
mtp(-10, 0, 0, REVERSE);
mtp(-10, 0, 45, REVERSE);
intake.move(127);
mtp(-1, 41, 155, NONE);//-------7
turn(170);
intake.move(-127);
pros::delay(100);
turn(155);
intake.move(127);
mtp(15, 45, 55, NONE);
pros::delay(100);
mtp(15, 45, 180, NONE);
odom::reset({0,0}, 0);
mtp(0, 50, -25, ASYNC);
frontwings.set_value(1);
pros::delay(100);
intake.move(-127);
chassis::waitUntilFinished(1);
frontwings.set_value(0);
turn(130);

mtp(0, -54, 0, REVERSE| RELATIVE);
blocker.set_value(1);
}



void leftside(){
	using namespace arms;
	intake.move(127);
	pros::delay(500);
	intake.move(25);
	frontwings.set_value(1);
	mtp(0,15, 20, NONE);
	frontwings.set_value(0);
	mtp(0,-10, 0, REVERSE);

	mtp(0,0, -45, REVERSE | RELATIVE);
	mtp(0,-30, -45, REVERSE);
	mtp(0,-35, 0, REVERSE | RELATIVE);
	blocker.set_value(1);
}

void secondleftside(){
	using namespace arms;
	mtp(0,-10, 0, REVERSE);
	swing(45, false);
	arms::odom::reset({0,0});
	mtp(0,-20, 45, REVERSE);
	mtp(0,0, -45, REVERSE);
	

}

void skills(){
    using namespace arms;


    arms::odom::reset({0,0}, -20);


    cataTaskForSkills();
	turn(35);
	arms::odom::reset({0,0}, 0);
	
    mtp(0,85, 0, NONE);
	intake.move(-127);
	swing(-45,false);
	mtp(-24,110, -90, NONE);
	arms::odom::reset({0,0});
	mtp(2,-30, 0, REVERSE);

	/*
	mtp(0,-10, 0, REVERSE| RELATIVE);
	turn(180);
	mtp(-5,60, 180, NONE);
	turn(-45);
	mtp(-30, 110, 10,NONE );//shoot
	mtp(-5,60, -90, REVERSE);
	mtp(-30,60, -90, NONE);
	turn(0);
	mtp(-30, 120, 0,NONE );//shoot
	mtp(-30,60, 0, NONE);
	turn(90);
	mtp(-50,60, 45, REVERSE);
	mtp(-30, 110, 10,NONE );//shoot
	swing(-90, true);
	mtp(-63, 90, -90, NONE);
		
	turn(-135);
	mtp(0, -30, 0, REVERSE | RELATIVE);
	turn(-90);
	mtp(0, -40, 0, REVERSE | RELATIVE);

	//mtp(-8,60, -180, NONE);
	/*
	turn(-90);
	mtp(-30, 55, -90,NONE);
	turn(0);
	
	mtp(-30, 110, 0,NONE | ASYNC);
	pros::delay(100);
	//frontwings.set_value(1);
	chassis::waitUntilFinished(1);
	
	frontwings.set_value(0);
	mtp(-30, 40, 0,REVERSE);
	
	turn(90);
	mtp(-40, 40, 90,REVERSE);
	mtp(-20, 110, 0,NONE | ASYNC);
	pros::delay(100);
	//frontwings.set_value(1);
	chassis::waitUntilFinished(1);
	
	frontwings.set_value(0);
	mtp(-20, 40, 0,REVERSE);
	turn(-90);
	mtp(-5, 40, -90,REVERSE);
	mtp(-20, 110, 0,NONE | ASYNC);
	pros::delay(100);
	//frontwings.set_value(1);
	chassis::waitUntilFinished(1);
	frontwings.set_value(0);
	mtp(10, 40, -90,REVERSE);
	
*/



}
void sixball(){
	
}	
void rightsideelims(){
	using namespace arms;
	odom::reset({0,0}, 0);
	blocker.set_value(1);
	frontwings.set_value(1);
	pros::delay(1000);
	frontwings.set_value(0);
	intake.move(127);
	mtp(-1, 75, 0, NONE);
	pros::delay(100);
	turn(135);
	frontwings.set_value(1);
	intake.move(-127);
	mtp(38, 30, 135, NONE);
	odom::reset({0,0},0);
	frontwings.set_value(0);
	intake.move(127);
	mtp(0, -20, 0, REVERSE);
	turn(-90);
	mtp(0, 30, 0, RELATIVE);
	turn(-10);
	mtp(0, -53, 0, REVERSE | RELATIVE);
	turn(90);
	intake.move(-127);
	pros::delay(50);
	mtp(5, -10, 20, REVERSE | RELATIVE);
	mtp(-30, 60, -50, RELATIVE | ASYNC);
	pros::delay(50);
	intake.move(-127);
	chassis::waitUntilFinished(1);
	
	
	
}
void fivetrimidrush(){
	using namespace arms;
	odom::reset({0,0}, 0);
	blocker.set_value(1);
	frontwings.set_value(1);
	pros::delay(1000);
	frontwings.set_value(0);
	intake.move(127);
	mtp(-1, 75, 0, NONE);
	pros::delay(100);
	turn(135);
	frontwings.set_value(1);
	intake.move(-127);
	mtp(38, 30, 135, NONE);
	odom::reset({0,0},0);
	frontwings.set_value(0);
	intake.move(127);
	mtp(0, -20, 0, REVERSE);
	turn(-90);
	mtp(0, 30, 0, RELATIVE);
	turn(-10);
	mtp(0, -53, 0, REVERSE | RELATIVE);
	turn(90);
	intake.move(-127);
	mtp(5, -5, 0, RELATIVE | REVERSE);
	mtp2(0, 20, -10, 100, RELATIVE | ASYNC);
	pros::delay(50);
	frontwings.set_value(1);
	chassis::waitUntilFinished(5);
	arms::odom::imu->set_heading(0);
	turn(-90);
	frontwings.set_value(0);
	turn(0);
	
	mtp(0, -20, 0, REVERSE | RELATIVE);
	intake.move(-127);
	mtp(-30, 100, -50, RELATIVE);

}

void rightside6ball(){
	using namespace arms;
	intake.move(127);
	pros::delay(100);
	mtp2(0, -30, -45,90, REVERSE);
	mtp(0,5, -135, NONE | RELATIVE);
	
	mtp2(5, 40, 10, 90,RELATIVE | ASYNC);
	pros::delay(100);
	frontwings.set_value(1);
	chassis::waitUntilFinished(1);
	frontwings.set_value(0);
	mtp(5, -20, 10, REVERSE | RELATIVE);
	turn(150);
	mtp(45, -130, -180, ASYNC);
	pros::delay(300);
	intake.move(-127);
	chassis::waitUntilFinished(1);
	swing(0, false);
	mtp(0, -10, 0, REVERSE | RELATIVE);
	mtp(0, 30, 0, NONE | RELATIVE);
	swing(0, false);


}

void autonomous() {
	skills();
	

	
    
}


bool r1pressed;
  	bool r2pressed;
	bool xpressed;
	bool ypressed;
	bool apressed;
	bool bpressed;
	bool l1pressed;
	bool l2pressed;
	bool arup;
	bool ardown;
	bool arleft;
	bool arright;
	bool detected;

void exponentialDrive(){
	double left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	double right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

	if(left < 0){
		arms::chassis::leftMotors->move(-(pow(left,2)));
	}
	if(right < 0){
		arms::chassis::leftMotors->move(-(pow(right,2)));
	}
	if(left >= 0){
		arms::chassis::leftMotors->move(pow(left,2));
	}
	if(right >= 0){
		arms::chassis::leftMotors->move(pow(right,2));
	}
}
void opcontrol() {
	

	pros::Task([=](){
		while(true){
			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1) ){
            l1pressed = !l1pressed;
            if(l1pressed){
                liftpiston.set_value(1);
				pros::delay(400);
				slapper.move(127);
            } else{
                liftpiston.set_value(0);
				slapper.move(0);
            }
            
    }
		}
	});
	while (true) {

		if(distancesens.get() >= 90 && distancesens.get() <= 110  ){
				detected = true;
			} else{
				detected = false;
			}
			

	
		

		arms::chassis::tank(master.get_analog(ANALOG_LEFT_Y) * (double)127 / 127,
		                      master.get_analog(ANALOG_RIGHT_Y) * (double)127 /
		                          127);

	
	//intake/outtake
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
      r1pressed = !r1pressed;
      if(r1pressed){r2pressed = false;intake.move(-127);}
      else{intake.move(0);}
      
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      r2pressed = !r2pressed;
	  
      if(r2pressed){r1pressed = false;intake.move(127);}
      else{intake.move(0);}
      
    }


	
	if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_X) || master.get_digital_new_press(E_CONTROLLER_DIGITAL_A) || master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y) || master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
            xpressed = !xpressed;
            apressed = !apressed;
			bpressed = !bpressed;
			ypressed = !ypressed;
            if(xpressed || apressed || bpressed || ypressed){
                slapper.move(127);
            } else{
                slapper.move(0);
            }
            
    }

	
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
		l1pressed = !l1pressed;
		if(l1pressed){
			blocker.set_value(1);
		} else{
			blocker.set_value(0);
		}
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
		
        arup = !arup;
		arleft = !arleft;
		
        if( arleft || arup ){
			frontwings.set_value(1);
		} else{
			frontwings.set_value(0);
		}
	}
	if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) ){
		
        arright = !arright;
		ardown = !ardown;
		
        if( arright || ardown ){
			turn(180);
		} 
	}
	pros::delay(10);

	}
	//master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) 
	
	pros::delay(10);
	
}
