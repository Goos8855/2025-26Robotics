#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/lv_btn.h"
#include "liblvgl/widgets/lv_canvas.h"
#include "liblvgl/widgets/lv_label.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "liblvgl/lvgl.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include "liblvgl/lvgl.h"

//Initializing ports
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup rightDT({-8,9,10}, pros::MotorGearset::blue); 
pros::MotorGroup leftDT({18,-19,-20}, pros::MotorGearset::blue);
pros::Imu IMU(7);
pros::MotorGroup intake({6}, pros::MotorGearset::blue);
pros::Rotation yOdom(17);
pros::MotorGroup upperIntake({-5});
pros::Motor upintake(-5);
pros::MotorGroup rotateIntake({15});
pros::adi::DigitalOut elevate('B');
pros::adi::DigitalOut ext('A');

bool runGuiAuton = false;
bool autonActive = false;

//Lemlib and PID configs
lemlib::Drivetrain drivetrain(
    &leftDT,
    &rightDT,
    12,
    lemlib::Omniwheel::NEW_325,
    360,
    2
);
lemlib::TrackingWheel verticalTrackingWheel(&yOdom,  lemlib::Omniwheel::NEW_2,0);
lemlib::OdomSensors sensors(
    &verticalTrackingWheel,
    nullptr,
    nullptr,
    nullptr,
    &IMU
);
lemlib::ControllerSettings lateralController(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    sensors
);

static void initSensorsGui(lv_event_t * event){

}


//declaring all objects globally

// GUI stuff here

lv_obj_t* homeScreen;
lv_obj_t* sensorScreen;
lv_obj_t* autonScreen;
lv_obj_t* posLabel;
lv_obj_t* toSensorsButton;
lv_obj_t* toAutoSelButton;
lv_obj_t* sensorHeadingLabel;
lv_obj_t* sensorWheelLabel;
lv_obj_t* sensorBackButton;
lv_obj_t* sensorResetButton;
lv_obj_t* autonNameLabel;
lv_obj_t* autonPrevButton;
lv_obj_t* autonNextButton;
lv_obj_t* autonBackButton;
lv_obj_t* runAutonButton;

enum Auton {
    AUTON_NONE = 0,
    AUTON_LEFT,
    AUTON_RIGHT,
    AUTON_SKILLS,
    AUTON_COUNT
};

int selectedAUton = AUTON_NONE;
const char* autonNames[AUTON_COUNT] = {
    "None",
    "Left",
    "Right",
    "Skills"
};

static void create_home_screen();
static void create_sensor_screen();
static void create_auton_screen();

static void autonTest(lv_event_t* e){
    (void)e;
    runGuiAuton = true;
}

ASSET(bl_txt); //doesnt work

void runSelectedAuton(){
    ext.set_value(false);
    elevate.set_value(true);

    switch (selectedAUton) {
        case AUTON_LEFT: //probably works, I don't even remember what this does anymore (I think its 1-2 high, match load, and 1-2 mid)
            elevate.set_value(false);
            chassis.moveToPose(1.16, 38.52, 1.33, 2000,{.minSpeed=80},false);
            ext.set_value(true);
            chassis.turnToHeading(-90, 2000,{},false);
            chassis.moveToPoint(-15, 38.57, 1000,{.minSpeed=120},false);
            intake.move(127);
            pros::delay(750);
            chassis.moveToPoint(5, 38.52, 1000, {.forwards=false});

            chassis.moveToPoint(-5, 38.52, 2000,{.maxSpeed=127});
            
            chassis.turnToHeading(90, 1000,{},false);
            ext.set_value(false);
            pros::delay(250);
            chassis.moveToPoint(15.8, 38.52, 1000,{},false);
            chassis.turnToHeading(90, 500);
            upperIntake.move(127);
            rotateIntake.move(127);
            pros::delay(2000);
            upperIntake.move(0);
            elevate.set_value(true);
            rotateIntake.move((-127));
            chassis.moveToPoint(0, 37.51, 1000,{.forwards=false,.minSpeed=100});
            chassis.moveToPoint(17, 18, 1000);
            chassis.turnToHeading(135, 1000);
            chassis.moveToPoint(32, 3 , 5000,{.minSpeed=50});
            upperIntake.move(127);
            
            
            break;
        case AUTON_RIGHT: //tested and works: Scores one on low goal and moves to match loader

            chassis.moveToPoint(1.21, 36.72, 2000);
            chassis.turnToHeading(90, 1000);
            ext.set_value(true);
            intake.move(127);
            chassis.moveToPoint(12, 37.81, 1000,{.minSpeed=100});
            pros::delay(1500);
            chassis.moveToPoint(1.21, 37.72, 1000,{.forwards=false});
            chassis.turnToHeading(90, 1000);
            ext.set_value(false);
            chassis.moveToPoint(-13.5, 37, 1000);
            elevate.set_value(true);
            intake.move(127);
            upperIntake.move(127);
            rotateIntake.move(127);

            

            
            break;
        case AUTON_SKILLS: //match loaders hit (for the most part) and just need to fix parking part
            intake.move(127);
            upperIntake.move(127);
            elevate.set_value(false);
            ext.set_value(true);
            chassis.moveToPoint(0, 52, 5000);
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPose(-20, 52, -90, 5000,{.minSpeed=100});
            pros::delay(3000);
            chassis.moveToPoint(0, 52, 2000,{.forwards=false});
            chassis.moveToPose(-20, 52, -90, 1500,{.minSpeed=100});
            pros::delay(4000);
            chassis.moveToPoint(8, 50, 1000,{.forwards=false});
            ext.set_value(false);
            intake.move(-127);
            upperIntake.move(-127);
            pros::delay(3000);
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(5, -50, 3000,{.forwards=false});
            chassis.turnToHeading(-90, 1000);
            ext.set_value(true);
            intake.move(127);
            upperIntake.move(127);
            chassis.moveToPoint(-12, -50, 1000,{.minSpeed=100});
            pros::delay(3000);
            chassis.moveToPoint(1, -50, 1000,{.forwards=false});
            chassis.moveToPoint(-12, -50, 1000,{.minSpeed=100});
            pros::delay(3000);
            chassis.moveToPoint(1, -50, 1000,{.forwards=false});
            ext.set_value(false);
            intake.move(-127);
            upperIntake.move(-127);
            rotateIntake.move(-127);
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(23.88, -25.23, 3000);
            chassis.turnToHeading(-90, 1000);
            chassis.moveToPoint(90.03, -28.38, 3000,{.forwards=false});
            chassis.turnToHeading(133.99, 1000);
            chassis.moveToPoint(95.65, -52.51, 2000);
            chassis.turnToHeading(90, 1000);
            ext.set_value(true);
            upintake.move(127);
            intake.move(127);
            rotateIntake.move(127);
            chassis.moveToPoint(120.19, -51.69, 1000,{.minSpeed=100});
            pros::delay(3000);
            chassis.moveToPoint(95, -51.69, 1000,{.forwards=false});
            chassis.moveToPoint(120.19, -51.69, 1000,{.minSpeed=100});
            pros::delay(6000);
            chassis.moveToPoint(95, -50.69, 1000,{.forwards=false});
            leftDT.move(-127);
            rightDT.move(-127);
            ext.set_value(false);
            chassis.moveToPoint(90, -28, 2000,{.forwards=false});
            chassis.moveToPoint(0, -28, 3000,{.forwards=false});
            //fix this part (actually test first cause I changed some stuff)
            chassis.moveToPoint(20, -3, 2000);
            pros::delay(500);
            chassis.turnToHeading(90, 2000);
            pros::delay(500);
            chassis.moveToPoint(-200, -3, 5000, {.minSpeed=127});
            //chassis.setPose(0,0,0);
            //pros::delay(1000);
            //chassis.moveToPoint(0, 100, 3000,{.minSpeed=100});
            //leftDT.move(-127); 
            //rightDT.move(-127);





            break;
        default: 
            break;
    }
}

void autonomous() {
    runSelectedAuton();
}

static void on_toSensors(lv_event_t* e){
    lv_scr_load(sensorScreen);
}
static void  on_toAuton(lv_event_t* e){
    lv_scr_load(autonScreen);
}
static void on_backHome(lv_event_t* e){
    lv_scr_load(homeScreen);
}
static void on_resetSensors(lv_event_t* e){
    yOdom.reset_position();
    chassis.setPose(0,0,0);
}
static void on_autonPrev(lv_event_t* e){
    selectedAUton = (selectedAUton-1+AUTON_COUNT)%AUTON_COUNT;
    lv_label_set_text_static(autonNameLabel, autonNames[selectedAUton]);
}
static void on_autonNext(lv_event_t* e){
    selectedAUton = (selectedAUton+1) % AUTON_COUNT;
    lv_label_set_text(autonNameLabel, autonNames[selectedAUton]);
}

static void create_home_screen(){
    homeScreen = lv_scr_act();

    posLabel = lv_label_create(homeScreen);
    lv_label_set_text(posLabel, "X:0.00, Y:0.00, T:0.00");
    lv_obj_align(posLabel, LV_ALIGN_TOP_LEFT, 10, 10);

    toSensorsButton = lv_btn_create(homeScreen);
    lv_obj_set_size(toSensorsButton, 150, 75);
    lv_obj_align(toSensorsButton, LV_ALIGN_TOP_LEFT, 20, 40);
    lv_obj_add_event_cb(toSensorsButton, on_toSensors, LV_EVENT_CLICKED, NULL);

    lv_obj_t* sensLabel = lv_label_create(toSensorsButton);
    lv_label_set_text(sensLabel, "Sensors");
    lv_obj_center(sensLabel);

    toAutoSelButton = lv_btn_create(homeScreen);
    lv_obj_set_size(toAutoSelButton, 150, 75);
    lv_obj_align(toAutoSelButton, LV_ALIGN_TOP_LEFT, 20, 130);
    lv_obj_add_event_cb(toAutoSelButton, on_toAuton, LV_EVENT_CLICKED, NULL);

    lv_obj_t* autoLabel = lv_label_create(toAutoSelButton);
    lv_label_set_text(autoLabel, "Debug");
    lv_obj_center(autoLabel);

    //

    autonNameLabel = lv_label_create(homeScreen);
    lv_label_set_text(autonNameLabel, autonNames[selectedAUton]);
    lv_obj_align(autonNameLabel, LV_ALIGN_TOP_MID, 90, 10);

    autonPrevButton = lv_btn_create(homeScreen);
    lv_obj_set_size(autonPrevButton, 60, 200);
    lv_obj_align(autonPrevButton, LV_ALIGN_LEFT_MID, 200, 0);
    lv_obj_add_event_cb(autonPrevButton, on_autonPrev, LV_EVENT_CLICKED, NULL);

    lv_obj_t* prevLabel = lv_label_create(autonPrevButton);
    lv_label_set_text(prevLabel, "<");
    lv_obj_center(prevLabel);

    autonNextButton = lv_btn_create(homeScreen);
    lv_obj_set_size(autonNextButton, 60, 200);
    lv_obj_align(autonNextButton, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(autonNextButton, on_autonNext, LV_EVENT_CLICKED, NULL);

    lv_obj_t* nextLabel = lv_label_create(autonNextButton);
    lv_label_set_text(nextLabel, ">");
    lv_obj_center(nextLabel);
    
    runAutonButton = lv_btn_create(homeScreen);
    lv_obj_set_size(runAutonButton, 120, 50);
    lv_obj_align(runAutonButton, LV_ALIGN_BOTTOM_MID, 95, -30);
    lv_obj_add_event_cb(runAutonButton, autonTest, LV_EVENT_CLICKED, NULL);

    lv_obj_t* runAutoLabel = lv_label_create(runAutonButton);
    lv_label_set_text(runAutoLabel, "Run Auto");
    lv_obj_center(runAutoLabel);

}

static void create_sensor_screen(){
    sensorScreen = lv_obj_create(NULL);

    sensorHeadingLabel = lv_label_create(sensorScreen);
    lv_label_set_text(sensorHeadingLabel, "Heading: 0.0 ");
    lv_obj_align(sensorHeadingLabel, LV_ALIGN_TOP_LEFT, 10, 10);

    sensorWheelLabel = lv_label_create(sensorScreen);
    lv_label_set_text(sensorWheelLabel, "Y: 0.0 deg");
    lv_obj_align(sensorWheelLabel, LV_ALIGN_TOP_LEFT, 10, 30);

    sensorResetButton = lv_btn_create(sensorScreen);
    lv_obj_set_size(sensorResetButton, 140, 200);
    lv_obj_align(sensorResetButton, LV_ALIGN_RIGHT_MID,-10, 0);
    lv_obj_add_event_cb(sensorResetButton, on_resetSensors, LV_EVENT_CLICKED, NULL);

    lv_obj_t* resetLabel = lv_label_create(sensorResetButton);
    lv_label_set_text(resetLabel, "Reset Sensors");
    lv_obj_center(resetLabel);

    sensorBackButton = lv_btn_create(sensorScreen);
    lv_obj_set_size(sensorBackButton, 80, 35);
    lv_obj_align(sensorBackButton, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(sensorBackButton, on_backHome, LV_EVENT_CLICKED, NULL);

    lv_obj_t* backlabel = lv_label_create(sensorBackButton);
    lv_label_set_text(backlabel, "Back");
    lv_obj_center(backlabel);
}

static void create_auton_screen(){ //actually the debug screen
    autonScreen = lv_obj_create(NULL);

    autonBackButton = lv_btn_create(autonScreen);
    lv_obj_set_size(autonBackButton, 80,35);
    lv_obj_align(autonBackButton, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(autonBackButton, on_backHome, LV_EVENT_CLICKED, NULL);

    lv_obj_t* backLabel = lv_label_create(autonBackButton);
    lv_label_set_text(backLabel, "Back");
    lv_obj_center(backLabel);
}

void initGui(){
    create_home_screen();
    create_sensor_screen();
    create_auton_screen();
}

// ---------------

void disabled() {}
void competition_initialize() {}

void updatePose(void*){
    while(true){
        lemlib::Pose pose = chassis.getPose();

        if(posLabel != nullptr){
            char buf[64];
            sprintf(buf, "X: %.2f, Y: %.2f, T: %.2f", pose.x, pose.y, pose.theta);
            lv_label_set_text(posLabel, buf);
        }
        if(sensorHeadingLabel != nullptr){
            double heading = IMU.get_heading();
            char hBuf[32];
            sprintf(hBuf, "Heading: %.1f deg", heading);
            lv_label_set_text(sensorHeadingLabel, hBuf);
        }

        if(sensorWheelLabel != nullptr){
            double ticks = yOdom.get_position();
            double inches = (ticks/360.0) * (2*3.14159);
            char wBuf[32];
            sprintf(wBuf, "Y wheel: %.2f deg", inches);
            lv_label_set_text(sensorWheelLabel, wBuf);
        }
        pros::delay(50);
    }
}

pros::Task* ui_update = nullptr;
void initialize() {
	lvgl_init();
    initGui();
    chassis.calibrate();

    leftDT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    leftDT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightDT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    ui_update = new pros::Task(updatePose, nullptr, "UI Update Task");
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool up = false;
    bool extract = false;
    int count = 0;
    while (true) { //loop to update screen
        count += 1;
        if(count > 5){
            count = 0;
            master.set_text(0,0, autonNames[selectedAUton]);
        }
        
        if(runGuiAuton && !autonActive){ //dont change any of this
            autonActive = true;
            runGuiAuton = false;
            runSelectedAuton();
            autonActive = false;
            if(selectedAUton==AUTON_SKILLS){ 
                pros::delay(90000);
            } else {
                pros::delay(15000);
            }
                
        }
        if(!autonActive){
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.3; //number = sensitivity multiplier
            int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.3;
            chassis.arcade(leftY, leftX);

            int intakeSpd = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            intake.move(intakeSpd);

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){ //dont change this
                master.clear_line(0);
                selectedAUton = (selectedAUton-1+AUTON_COUNT)%AUTON_COUNT;
                lv_label_set_text_static(autonNameLabel, autonNames[selectedAUton]);
            }else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
                master.clear_line(0);
                selectedAUton = (selectedAUton+1) % AUTON_COUNT;
                lv_label_set_text(autonNameLabel, autonNames[selectedAUton]);
            }

            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
                up = !up;
            }
            if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
                extract = !extract;
            }

            ext.set_value(extract);
            elevate.set_value(up);

            if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                upperIntake.move(intakeSpd);
                rotateIntake.move(intakeSpd);
            } else {
                upperIntake.move(0);
                rotateIntake.move(0);
            }

        }

        pros::delay(10);
    }
}