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
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <cstddef>
#include <cstdio>
#include <string>
#include "liblvgl/lvgl.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup rightDT({-8,9,10}, pros::MotorGearset::blue); 
pros::MotorGroup leftDT({18,-19,-20}, pros::MotorGearset::blue);
pros::Imu IMU(7);
pros::MotorGroup intake({6}, pros::MotorGearset::blue);
pros::Rotation yOdom(17);
pros::MotorGroup upperIntake({-5,16});
pros::adi::DigitalOut elevate('B');
pros::adi::DigitalOut ext('A');

bool runGuiAuton = false;
bool autonActive = false;

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
lemlib::ControllerSettings lateralController(12, // proportional gain (kP)
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

//
//
//
//             LOOK HERE!!!!
//       VVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
//
//
//


ASSET(bl_txt);


// Use these statements to code the autons: 
// chassis.moveToPose(x, y, angle, timeout);
// chassis.moveToPoint(x, y, timeout);
// Timeout just means the time it'll take for it to move on if the desired position is not reached
// x, y, and angle is just the x, y, and t shown on the screen (Measured in inches and degrees)
// !! ALL lines end with a semicolon ; !!
// You can include the ",{.forwards=false},false" after the timeout if you need the robot to follow the path backwards
// Move around the robot to desired positions & then record their points in the respective autonomous sections
// You can tell the program to wait for an amount of time by using pros::delay(time)
// time is measured in miliseconds, so 1000 time is 1 second of waiting or timeout time
// To move the intakes, you can use upperintake.move(pwr); and intake.move(pwr);
// Motor movement values are between -127 to 127
void runSelectedAuton(){
    ext.set_value(true);

    switch (selectedAUton) {
        case AUTON_LEFT:
            //lines for the left autonomous code goes here
            chassis.moveToPose(144, 28, 90, 10000);
            chassis.moveToPose(10, 28, 90, 10000,{.forwards=false},false);
            chassis.moveToPose(0, 0, 0, 10000,{.forwards=false},false);
            
            
            //and ends here
            break;
        case AUTON_RIGHT:
            //lines for the right autonomous code goes here
            leftDT.move(127);
            rightDT.move(127);
            pros::delay(1000);
            leftDT.move(0);
            rightDT.move(0);
            //and ends here
            break;
        case AUTON_SKILLS:
            //lines for the skills autonomous code goes here

            //and ends here
            break;
        default: //nothing goes in default
            break;
    }
}

//
//
//
//
//          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//
//
//

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

    ui_update = new pros::Task(updatePose, nullptr, "UI Update Task");
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool up = false;
    bool extract = true;
    while (true) {
        if(runGuiAuton && !autonActive){
            autonActive = true;
            runGuiAuton = false;
            runSelectedAuton();
            autonActive = false;
        }
        if(!autonActive){
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            chassis.curvature(leftY, leftX);

            int intakeSpd = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            intake.move(intakeSpd);

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
            } else {
                upperIntake.move(0);
            }

        }

        pros::delay(10);
    }
}