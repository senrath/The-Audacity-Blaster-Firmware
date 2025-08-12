/**
 * The Audacity Blaster Firmware
 * 
 * ©2025 senrath
 * 
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 
 */

#include <Arduino.h>

#include "NewEncoder.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <EEPROM.h>

#include <ADCInput.h>

#include <PIO_DShot.h>
#include <PID_v1.h>

#include <OneButton.h>

#include <string>

#include "logo.h"

//uncomment this if you're using a 1.0 revision board
//#define V1_0_BOARD

//comment this out if you don't want to display a logo on startup
#define USE_LOGO

//what battery are you using (BAT_3S 3s, BAT_4S for 4s)
//this only matters for the default configuration
#define BAT_TYPE BAT_4S

//define what control system you're using for the solenoid (OPEN_LOOP or CLOSED_LOOP)
//if you use open loop you'll may want to to adjust the solenoid timings below
//they're ignored in closed loop
#define SOLENOID_CONTROL CLOSED_LOOP

//how many magnet poles each motor has
#define MOTOR_POLES 14
//each motor's KV rating
#define MOTOR_KV 2450

//whether or not each motor should be reversed
#define MOTOR1_REVERSE false
#define MOTOR2_REVERSE false

//default motor power (10-100, percentage)
#define MOTOR_SPEED 50

//what motor speed (%) to idle at, if idle mode is turned on
#define IDLE_SPEED 15

//default rate of fire (darts per second, 0 = max)
//these cap at 100 dps, just to allow sanity checking.
//if you actually have a solenoid and magazine that can handle more than 100 dps you can figure out
//what you need to edit
#define FULL_AUTO_ROF 0
#define BURST_FIRE_ROF 0

//how many darts per burst (minimum 2, max 20)
#define BURST_SIZE 3

//define how the rev trigger acts by default
//REV_DISABLED, REV_IDLE_TOGGLE, REV_IDLE_HOLD, REV_FULL_HOLD
#define REV_TRIGGER_OPTION REV_FULL_HOLD

//rotary encoder pins
#define ENC_DT 7
#define ENC_CLK 6
#define ENC_SW 8

//which pin the voltage divider connects to. has to be an ADC capable pin
#define VOLTAGE_PIN 29

//what pins are the motors on
#define MOTOR1 5
#define MOTOR2 4

//what pins are the triggers on
#define MAIN_TRIGGER 1
#define REV_TRIGGER 0

//how many seconds should the encoder be held to count as a long click
#define ENC_LONG_CLICK 2

//how many physical steps (detents) per full rotation
#define ENC_STEPS_PER_ROTATION 20

//number of detected steps per detent of the encoder
//the encoders I used report two steps per detent, but others might not
#define ENC_STEP_NORMALIZATION 2

//lower this number if you're having trouble getting short clicks to register
//raise it if you're having problems with double clicking
#define ENC_CLICK_DEBOUNCE 10

//trigger stuff
//a lower debounce theoretically increases trigger response but may cause double presses depending on switch
#define TRIGGER_DEBOUNCE 20
//how many seconds it takes to count for a long click
#define TRIGGER_LONG_CLICK 2

//solenoid values
#define SOLENOID_PIN 9
#define SOLENOID_SAFETY_TIMER 1000 //number of miliseconds to wait for the solenoid in the worst case scenario before going to error
#define SOLENOID_PULSE_3S_HIGH 35
#define SOLENOID_PULSE_3S_LOW 40
#define SOLENOID_PULSE_4S_HIGH 30
#define SOLENOID_PULSE_4S_LOW 35
#define SOLENOID_RETRACT 30

//how many milliseconds to hold the motors at max before letting them spin down
//this helps with a more responsive trigger during semi and burst
//and makes sure the motors hold the target speed for the full duration of a dart
//max hold time set to one second just for sanity checking
//if you really want to have a longer hold for some reason, just up this number
#define MOTOR_HOLD_TIME 200

//OLED values
//the default values here are on Wire1. Double check your board's pinout if you change these
#define OLED_SCL 3
#define OLED_SDA 2
#define OLED_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1 //-1 for no dedicated reset pin
#define WIRE1 //comment this out if you need to use Wire instead of Wire1 due to pin changes

//IR pins
#define IR_SENSOR1_PIN 27
#define IR_SENSOR2_PIN 26
//the emitter pins are only used with the v1.0 board, v1.1 and later have enough 3.3v pins to not use gpio
#define IR_EMITTER1_PIN 15
#define IR_EMITTER2_PIN 14

//PID control values
#define USE_PID true 
#define P 10
#define I 10
#define D 0

//used as a failsafe, if the motor hasn't reached the target within x milliseconds
#define SPINUP_SAFETY_TIME 1000

//sanity check definitions
#define MIN_MOTOR_SPEED 25
#define MAX_MOTOR_SPEED 100
#define MIN_IDLE_SPEED 0
#define MAX_IDLE_SPEED 100
#define MIN_ROF 0
#define MAX_ROF 100
#define MIN_BURST 2
#define MAX_BURST 20
#define MIN_PULSE 5
#define MAX_PULSE 1000
#define MIN_DEBOUNCE 0
#define MAX_DEBOUNCE 500
#define MIN_HOLD_TIME 0
#define MAX_HOLD_TIME 1000


//use debugging logic
#define DEBUG true

//You shouldn't need to adjust any values below this line
//------------------------------------------------------------------------------------------------------

//operation states
#define STATE_NORMAL 0
#define STATE_MENU 1
#define STATE_LOW_BATTERY 2
#define STATE_ERROR 3
#define STATE_FIRING 4
volatile uint8_t current_state = STATE_NORMAL; //volatile because the voltage sensor can change it

//fire mode
#define SEMI_AUTO 0
#define BINARY 1
#define BURST_FIRE 2
#define FULL_AUTO 3
uint8_t fire_mode = SEMI_AUTO;

//battery values
#define BAT_3S 0
#define BAT_4S 1
#define BAT_HIGH_3S 12.6
#define BAT_LOW_3S 10.5
#define BAT_HIGH_4S 16.8
#define BAT_LOW_4S 14.0

//encoder and trigger click types
#define NO_CLICK 0
#define SHORT_CLICK 1
#define LONG_CLICK 2


//solenoid states
#define SOLENOID_OFF 0
#define SOLENOID_FORWARD 1
#define SOLENOID_REVERSE 2
#define SOLENOID_COOLDOWN 3
#define SOLENOID_ERROR 4
uint8_t solenoid_state = SOLENOID_OFF;

//IR states
#define IR_NO_BEAM_BROKEN 0
#define IR_FRONT_BEAM_BROKEN 1
#define IR_BACK_BEAM_BROKEN 2
#define IR_BOTH_BEAM_BROKEN 3

//struct to hold the config values
//everything here is uint32_t for alignment purposes, even if a smaller type would also work
//we easily have the space for it
struct configuration {
    uint32_t target_motor_speed;
    uint32_t idle_motor_speed;
    uint32_t motor_hold_time;
    uint32_t full_auto_rof;
    uint32_t burst_fire_rof; 
    uint32_t burst_size;
    uint32_t solenoid_control;
    uint32_t solenoid_pulse_high_3s;
    uint32_t solenoid_pulse_low_3s;
    uint32_t solenoid_pulse_high_4s;
    uint32_t solenoid_pulse_low_4s;
    uint32_t solenoid_retract;
    uint32_t rev_trigger_option;
    uint32_t trigger_debounce;
    uint32_t battery_type;
};

//menu definitions
#define TARGET_MOTOR_SPEED_MENU 0
#define IDLE_MOTOR_SPEED_MENU 1
#define MOTOR_HOLD_TIME_MENU 2
#define FULL_AUTO_ROF_MENU 3
#define BURST_FIRE_ROF_MENU 4
#define BURST_SIZE_MENU 5
#define SOLENOID_CONTROL_MENU 6
#define SOLENOID_PULSE_HIGH_MENU 7
#define SOLENOID_PULSE_LOW_MENU 8
#define SOLENOID_RETRACT_MENU 9
#define REV_TRIGGER_OPTION_MENU 10
#define TRIGGER_DEBOUNCE_MENU 11
#define BATTERY_TYPE_MENU 12
#define EXIT_MENU 13
#define NUM_MENU_ITEMS 14

//rev trigger definitions
#define REV_DISABLED 0
#define REV_IDLE_TOGGLE 1
#define REV_IDLE_HOLD 2
#define REV_FULL_HOLD 3

//solenoid control definitions
#define OPEN_LOOP 0
#define CLOSED_LOOP 1

void setup_display();
void setup_encoder();
void setup_solenoid();
void setup_triggers();
void setup_ir();
void setup_motors();
void setup_pid();
int8_t handle_break_beam();
void handle_motors();
void tick_triggers();
void process_encoder();
int32_t encoder_get_normalized_steps();
void handle_encoder_rotation();
void handle_encoder_clicked();
uint8_t encoder_click_type();
void print_display();
void print_normal();
void print_menu();
void print_low_battery();
void print_error();
void write_config();
void read_config();
void load_default_config();
void adjust_config_by_menu_position(int32_t, uint8_t);
void create_menu_array();
void display_menu_item(uint8_t);
void read_voltage();
static void main_trigger_pressed();
static void main_trigger_clicked();
static void main_trigger_long_press_start();
static void main_trigger_long_press_held();
static void main_trigger_long_press_stop();
static void rev_trigger_long_press_start();
static void rev_trigger_long_press_stop();
int32_t rof_adjustment_time(int32_t);
void handle_firing();
void handle_firing_closed_loop();
void handle_firing_open_loop();
uint32_t mapped_pulse_time();
uint32_t percent_to_throttle(uint32_t);
uint32_t percent_to_rpm(uint32_t);
uint32_t rpm_to_throttle(uint32_t);
uint32_t erpm_to_rpm(uint32_t);
uint32_t clamp(uint32_t, uint32_t, uint32_t);
template <typename T> void debug_println(T);
template <typename T> void debug_print(T);

NewEncoder encoder;
uint32_t encoder_initial_steps;
Adafruit_SSD1306 display;
configuration config;
uint32_t size_of_config = sizeof(config);
uint32_t current_target_speed;
uint32_t current_speed;
ADCInput voltage_pin(VOLTAGE_PIN);
volatile float measured_voltage; //volatile since it's set in core1 but read in core0
const char* menu_array[NUM_MENU_ITEMS];

BidirDShotX1 *motor1;
BidirDShotX1 *motor2;

bool idle_on = false;
bool rev_without_firing = false;

float bat_low = (BAT_TYPE == BAT_3S ? BAT_LOW_3S : BAT_LOW_4S);
float bat_high = (BAT_TYPE == BAT_3S ? BAT_HIGH_3S : BAT_HIGH_4S);

uint32_t darts_to_fire = 0;
std::string error_string = "";

double target_rpm = 0;
double motor1_rpm, motor2_rpm;
double motor1_output, motor2_output;
PID motor1_pid(&motor1_rpm, &motor1_output, &target_rpm, P, I, D, P_ON_E, DIRECT);
PID motor2_pid(&motor2_rpm, &motor2_output, &target_rpm, P, I, D, P_ON_E, DIRECT);

OneButton main_trigger;
OneButton rev_trigger;

void setup()
{
    //we only need the serial connection if we're debugging
    if (DEBUG)
    {
        Serial.begin(9600);
    }

    //initialize the menu array
    create_menu_array();

    //initialize the fake eeprom for config storage
    EEPROM.begin(512);

    setup_encoder();

    setup_display();

    //attempt to read config.
    //the first time this runs it will instead load the default config, since there's nothing in the "eeprom" to read back
    read_config();

    setup_triggers();
    setup_ir();
    setup_solenoid();
    setup_pid();
    setup_motors(); //set up motors last so the loop can take over keeping the ESCs from resetting
}

void loop()
{
    tick_triggers();
    process_encoder();
    handle_motors();
    handle_firing();
    print_display();
}

//run the voltage reading on core1 because it throws off the timing for everything else
//it would be better to figure out why it's causing problems, but since core1 is otherwise unusued
//this works for now
void setup1()
{
    voltage_pin.begin(1000);
}

void loop1()
{
    read_voltage();
}

//set up the display
//we do this here because we use non-default SDA and SCL pins for routing reasons on the custom pcb
//the pins chosen are on wire1, not wire
void setup_display()
{
    #ifdef WIRE1
        Wire1.setSDA(OLED_SDA);
        Wire1.setSCL(OLED_SCL);
        Wire1.begin();

        display = Adafruit_SSD1306(OLED_WIDTH, OLED_HEIGHT, &Wire1, OLED_RESET);
    #else
        Wire.setSDA(OLED_SDA);
        Wire.setSCL(OLED_SCL);
        Wire.begin();

        display = Adafruit_SSD1306(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
    #endif

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    
    //setup initial display conditions and draw the logo
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    #ifdef USE_LOGO
        display.drawBitmap(0, 0, LOGO, OLED_WIDTH, OLED_HEIGHT, SSD1306_WHITE);
        display.display();
        delay(1000); //hold for 1 second so you can actually see the logo
    #endif
}

//set up the encoder and capture the initial reading
void setup_encoder()
{
    encoder.begin(ENC_DT, ENC_CLK, ENC_SW, ENC_STEPS_PER_ROTATION * ENC_STEP_NORMALIZATION);
    encoder_initial_steps = encoder.GetSteps();
}

//set up the two trigger switches
//main trigger has to handle press, click, and hold while the rev only needs long press
void setup_triggers()
{
    main_trigger.setup(MAIN_TRIGGER, INPUT_PULLUP, true);
    rev_trigger.setup(REV_TRIGGER, INPUT_PULLUP, true);

    main_trigger.setDebounceMs(config.trigger_debounce);
    rev_trigger.setDebounceMs(config.trigger_debounce);

    //long press timing will vary with fire mode but since we start on semi-auto we'll want to
    //set the default up here. the rev trigger only cares about being held (even for the toggle clicks)
    //so set it to instantly count as a long press
    main_trigger.setPressMs(TRIGGER_LONG_CLICK * 1000);
    rev_trigger.setPressMs(0);

    //attach all the types of clicks to the main trigger (initial click, binary release, held, released)
    main_trigger.attachPress(main_trigger_pressed);
    main_trigger.attachClick(main_trigger_clicked);
    main_trigger.attachLongPressStart(main_trigger_long_press_start);
    main_trigger.attachDuringLongPress(main_trigger_long_press_held);
    main_trigger.attachLongPressStop(main_trigger_long_press_stop);

    //the rev trigger doesn't actually care about being held, just the start and stop presses
    rev_trigger.attachLongPressStart(rev_trigger_long_press_start);
    rev_trigger.attachLongPressStop(rev_trigger_long_press_stop);
}

//set up the IR poximity sensors
void setup_ir()
{
    gpio_init(IR_SENSOR1_PIN);
    gpio_set_dir(IR_SENSOR1_PIN, GPIO_IN);
    gpio_pull_up(IR_SENSOR1_PIN);
    gpio_init(IR_SENSOR2_PIN);
    gpio_set_dir(IR_SENSOR2_PIN, GPIO_IN);
    gpio_pull_up(IR_SENSOR2_PIN);
    //the v1.0 board lacks enough 3.3v pins to power the emitters and the sensors
    //so power the emitters from gpio pins
    #ifdef V1_0_BOARD
        gpio_init(IR_EMITTER1_PIN);
        gpio_set_dir(IR_EMITTER1_PIN, GPIO_OUT);
        gpio_put(IR_EMITTER1_PIN, true);
        gpio_init(IR_EMITTER2_PIN);
        gpio_set_dir(IR_EMITTER2_PIN, GPIO_OUT);
        gpio_put(IR_EMITTER2_PIN, true);
    #endif
}


//get the solenoid pin ready to trigger the mosfet
void setup_solenoid()
{
    gpio_init(SOLENOID_PIN);
    gpio_set_dir(SOLENOID_PIN, GPIO_OUT);
    gpio_pull_down(SOLENOID_PIN);
}

uint32_t safety_timer = 0;
uint32_t motor_startup_time = 0;
//do initial motor setup
void setup_motors()
{
    motor1 = new BidirDShotX1(MOTOR1, 600); // set up escs with dshot 600
    motor2 = new BidirDShotX1(MOTOR2, 600);
    motor_startup_time = millis();
    while (millis() - motor_startup_time < 1000) //the esc needs a certain amount of throttle 0 commands to arm (~300 ms worth, but do a full 1 second in case of timing problems)
    {   
        motor1->sendThrottle(0);
        motor2->sendThrottle(0);
        delayMicroseconds(200); //the esc can't handle packets too fast, so delay
    }
    if (MOTOR1_REVERSE)
    {
        //the esc needs to get the reverse command 6 times for it to work
        for (uint8_t i = 0; i < 6; i++)
        {
            motor1->sendRaw12Bit(DSHOT_CMD_SPIN_DIRECTION_REVERSED);
            delayMicroseconds(200); //the esc can't handle packets too fast, so delay
        }
    }
    if (MOTOR2_REVERSE)
    {
        //the esc needs to get the reverse command 6 times for it to work
        for (uint8_t i = 0; i < 6; i++)
        {
            motor2->sendRaw12Bit(DSHOT_CMD_SPIN_DIRECTION_REVERSED);
            delayMicroseconds(200); //the esc can't handle packets too fast, so delay
        }
    }
    safety_timer = millis();
}


void setup_pid()
{
    //the PID library defaults to an output range of 0-255 because it's meant for pwm control on an arduino
    //we need to change it to instead output from 0-(KV * BAT_HIGH) to handle the full range of RPMs 
    motor1_pid.SetOutputLimits(0, bat_high * MOTOR_KV);
    motor2_pid.SetOutputLimits(0, bat_high * MOTOR_KV);
    //the default PID loop only runs every 100ms. The ESCs can respond much faster than that
    motor1_pid.SetSampleTime(1);
    motor2_pid.SetSampleTime(1);
    //all of the PID inputs and outputs need to be set to 0 since the motors start at idle
    motor1_rpm = 0;
    motor2_rpm = 0;
    motor1_output = 0;
    motor2_output = 0;
}

uint32_t break_millis = 0;

int8_t handle_break_beam()
{
    bool beam1_intact = gpio_get(IR_SENSOR1_PIN);
    bool beam2_intact = gpio_get(IR_SENSOR2_PIN);

    break_millis = millis();
    
    if (beam1_intact && beam2_intact)
    {
        //solenoid is fully extended
        debug_println("IR_NO_BEAM_BROKEN");
        return IR_NO_BEAM_BROKEN;
    }
    else if (beam1_intact && !beam2_intact)
    {
        //solenoid is moving
        debug_println("IR_FRONT_BEAM_BROKEN");
        return IR_FRONT_BEAM_BROKEN;
    } 
    else if (!beam1_intact && beam2_intact)
    {
        //if the front beam isn't broken but the back is, something has failed
        //make sure the solenoid is off and the error state is switched on
        solenoid_state = SOLENOID_OFF;
        current_state = STATE_ERROR;
        error_string = "Break Beam sensor failure";
        debug_println("Only back beam broken.");
        return IR_BACK_BEAM_BROKEN;
    }
    else
    {
        //solenoid is in its resting position
        debug_println("IR_BOTH_BEAM_BROKEN");
        return IR_BOTH_BEAM_BROKEN;
    }
}

uint32_t previous_rpm_print_timer = 0;
bool at_target_speed = false;
uint32_t previous_rpm1 = 0;
uint32_t previous_rpm2 = 0;
//process motor speed control
//we need to constantly be sending throttle commands or the esc will time out
void handle_motors()
{
    uint32_t motor1_erpm = 0;
    uint32_t motor2_erpm = 0;
	BidirDshotTelemetryType packet1 = motor1->getTelemetryErpm(&motor1_erpm);
	BidirDshotTelemetryType packet2 = motor2->getTelemetryErpm(&motor2_erpm);
    motor1_rpm = erpm_to_rpm(motor1_erpm);
    motor2_rpm = erpm_to_rpm(motor2_erpm);

    //sometimes the check happens before the esc has an eRPM packet ready.
    //in that case, use the previous rpm value
    if (packet1 == BidirDshotTelemetryType::NO_PACKET || packet1 == BidirDshotTelemetryType::OTHER_VALUE)
    {
        motor1_rpm = previous_rpm1;
    }
    else
    {
        previous_rpm1 = motor1_rpm;
    }

    if (packet2 == BidirDshotTelemetryType::NO_PACKET || packet1 == BidirDshotTelemetryType::OTHER_VALUE)
    {
        motor2_rpm = previous_rpm2;
    }
    else
    {
        previous_rpm2 = motor2_rpm;
    }

    switch(current_state) 
    {
        case STATE_NORMAL:
            if (rev_without_firing)
            {
                //rev trigger is enabled and held down, spin up the motors to full speed
                //this comes first because idle mode can be enabled and the rev trigger used
                target_rpm = percent_to_rpm(current_target_speed);
                if (USE_PID)
                {
                    //this is safe to call even if the PID loops are already on, so state checks aren't needed
                    motor1_pid.SetMode(AUTOMATIC);
                    motor2_pid.SetMode(AUTOMATIC);
                    motor1_pid.Compute();
                    motor2_pid.Compute();

                    motor1->sendThrottle(rpm_to_throttle(motor1_output));
                    motor2->sendThrottle(rpm_to_throttle(motor2_output));
                }
                else
                {
                    motor1->sendThrottle(percent_to_throttle(current_target_speed));
                    motor2->sendThrottle(percent_to_throttle(current_target_speed));
                }
            }
            else if (idle_on)
            {
                //idle mode is on, so spin up the motors
                target_rpm = percent_to_rpm(config.idle_motor_speed);
                if (USE_PID)
                {
                    //this is safe to call even if the PID loops are already on, so state checks aren't needed
                    motor1_pid.SetMode(AUTOMATIC);
                    motor2_pid.SetMode(AUTOMATIC);
                    motor1_pid.Compute();
                    motor2_pid.Compute();
                    motor1->sendThrottle(rpm_to_throttle(motor1_output));
                    motor2->sendThrottle(rpm_to_throttle(motor2_output));
                }
                else
                {
                    motor1->sendThrottle(percent_to_throttle(config.idle_motor_speed));
                    motor2->sendThrottle(percent_to_throttle(config.idle_motor_speed));
                }
            }
            else
            {
                //idle mode isn't on and the rev trigger isn't held, so motors should be stopped
                target_rpm = 0;
                if (USE_PID)
                {
                    //this is safe to call even if the PID loops are already off, so state checks aren't needed
                    motor1_pid.SetMode(MANUAL);
                    motor2_pid.SetMode(MANUAL);
                    motor1_output = 0;
                    motor2_output = 0;
                }
                //we don't need to use the PID loop's feedback to turn off the motors
                motor1->sendThrottle(0);
                motor2->sendThrottle(0);
            }
            break;
        case STATE_FIRING:
                //we're trying to fire, so rev to full
                target_rpm = percent_to_rpm(current_target_speed);
                if (USE_PID)
                {
                    //this is safe to call even if the PID loops are already on, so state checks aren't needed
                    motor1_pid.SetMode(AUTOMATIC);
                    motor2_pid.SetMode(AUTOMATIC);
                    motor1_pid.Compute();
                    motor2_pid.Compute();
                    motor1->sendThrottle(rpm_to_throttle(motor1_output));
                    motor2->sendThrottle(rpm_to_throttle(motor2_output));
                }
                else
                {
                    motor1->sendThrottle(percent_to_throttle(current_target_speed));
                    motor2->sendThrottle(percent_to_throttle(current_target_speed));
                }
            break;
        default:
            //motors should not be running during the menu, error, or low battery states
            target_rpm = 0;
            if (USE_PID)
            {
                //this is safe to call even if the PID loops are already off, so state checks aren't needed
                motor1_pid.SetMode(MANUAL);
                motor2_pid.SetMode(MANUAL);
                motor1_output = 0;
                motor2_output = 0;
            }
            //we don't need to use the PID loop's feedback to turn off the motors
            motor1->sendThrottle(0);
            motor2->sendThrottle(0);
            break;
    }
    
    //keep resetting the safety_timer if the motors are supposed to be stopped
    if (target_rpm == 0)
    {
        safety_timer = millis();
    }

    if (DEBUG && millis() - previous_rpm_print_timer > 10000000 && current_state != STATE_ERROR)
    {
        debug_print("Target RPM: ");
        debug_println(target_rpm);
        debug_print("Motor 1 RPM: ");
        debug_println(motor1_rpm);
        debug_print("Motor 2 RPM: ");
        debug_println(motor2_rpm);
        debug_print("Measured voltage: ");
        debug_println(measured_voltage);
        previous_rpm_print_timer = millis();
    }

    //eRPM is a bit noisy, so give a little room for error on the target RPM
    //also clamp on the top end to not fire if we're significantly overshooting
    if (motor1_rpm >= target_rpm * 0.95 && motor2_rpm >= target_rpm * 0.95 && target_rpm != 0 && motor1_rpm <= target_rpm * 1.05 && motor2_rpm <= target_rpm * 1.05)
    {
        at_target_speed = true;
        if (millis() - safety_timer > 10)
        {
            debug_print("Time taken: ");
            debug_println(millis() - safety_timer);
        }
        safety_timer = millis();
    }
    else
    {
        at_target_speed = false;
    }

    //at least one motor has taken too long to get to speed
    if (millis() - safety_timer >= SPINUP_SAFETY_TIME)
    {           
        //something's wrong, probably a jam, so stop the motors
        target_rpm = 0;
        if (USE_PID)
        {
            //this is safe to call even if the PID loops are already off, so state checks aren't needed
            motor1_pid.SetMode(MANUAL);
            motor2_pid.SetMode(MANUAL);
            motor1_output = 0;
            motor2_output = 0;
        }
        motor1->sendThrottle(0);
        motor2->sendThrottle(0);
        current_state = STATE_ERROR;
        error_string = "Motors failed to reach speed";
        debug_println("Took too long to reach target speed");
    }
    delayMicroseconds(200); //the esc can't handle packets too fast, so delay
}

//the trigger objects have to tick every loop to poll the states correctly
void tick_triggers()
{
    main_trigger.tick();
    rev_trigger.tick();
}

int32_t encoder_previous_steps = 0; 
bool encoder_clicked = false;
uint64_t encoder_last_clicked = 0;
uint64_t display_last_update = 0;
uint8_t current_menu_item = 0;
bool menu_item_selected = false;

void process_encoder()
{
    encoder.Update();
    handle_encoder_rotation();
    if (encoder_get_normalized_steps() - encoder_previous_steps != 0)
    {
        debug_print("Encoder step: ");
        debug_println(encoder_get_normalized_steps() - encoder_previous_steps);
    }
    handle_encoder_clicked();
    encoder_previous_steps = encoder_get_normalized_steps();
}

//number of encoder steps, after compensating for how many steps per detent and subtracting whatever the initial step amount was.
int32_t encoder_get_normalized_steps()
{
    return ceil(float(encoder.GetSteps()) / ENC_STEP_NORMALIZATION) - encoder_initial_steps;
}

void handle_encoder_rotation()
{
    int32_t new_steps = encoder_get_normalized_steps() - encoder_previous_steps;
    
    switch(current_state) 
    {
        case STATE_NORMAL:
            //we're in normal operation, so the encoder handles adjusting target speed
            current_target_speed += (new_steps * 5);
            //clamp between min and max speed
            current_target_speed = clamp(current_target_speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            break;
        case STATE_MENU:
            //we're in the menu, so we either need to select a new menu item or adjust the current item
            if(!menu_item_selected)
            {
                int8_t temp_menu_item = (current_menu_item + new_steps) % NUM_MENU_ITEMS;
                current_menu_item = (temp_menu_item < 0) ? NUM_MENU_ITEMS - 1 : temp_menu_item;
            }
            else
            {
                adjust_config_by_menu_position(new_steps, current_menu_item);
            }
            break;
        default:
            //we're either firing or in an error, so don't do anything
            break;
    }
}

void handle_encoder_clicked()
{
    switch(current_state) 
    {
        case STATE_NORMAL:
            //we're in normal operation, so short presses swap fire modes and long presses enter the menu
            switch(encoder_click_type()) 
            {
                case SHORT_CLICK:
                    debug_println("Switching fire modes");
                    fire_mode++;
                    if (fire_mode > FULL_AUTO)
                    {
                        fire_mode = SEMI_AUTO;
                    }
                    if (fire_mode == FULL_AUTO)
                    {
                        main_trigger.setPressMs(0);
                    }
                    else
                    {
                        main_trigger.setPressMs(TRIGGER_LONG_CLICK * 1000);
                    }
                    break;
                case LONG_CLICK:
                    current_state = STATE_MENU;
                    debug_println("Switching to menu");
                    break;
                default:
                    //no click detected, so don't do anything
                    break;
            }
            break;
        case STATE_MENU:
            //we're in the menu, so short presses either select the current item or save changes
            //long and short press do the same here
            switch(encoder_click_type()) 
            {
                case SHORT_CLICK:
                case LONG_CLICK:
                    if (current_menu_item == EXIT_MENU)
                    {
                        current_state = STATE_NORMAL;
                        write_config();
                        debug_println("Exit selected, saving config and switching to normal");
                    }
                    else
                    {
                        //toggle whether or not the current menu item is selected
                        menu_item_selected = !menu_item_selected;
                    }
                    break;
                default:
                    //no click detected, so don't do anything
                    break;
            }
            break;
        case STATE_ERROR:
            //we're in error state. it's probably a jam, so if the user wants to clear the error state let them
            switch(encoder_click_type()) 
            {
                case LONG_CLICK:
                    current_state = STATE_NORMAL;
                    solenoid_state = SOLENOID_OFF;
                    safety_timer = millis();
                    target_rpm = 0;
                    error_string = "";
                    debug_println("Clearing error");
                    break;
                default:
                    //Only a long press should clear the error state
                    break;
            }
            break;
        case STATE_LOW_BATTERY:
            //low battery *might* be a fine 3s battery in 4s mode, so let the user enter the menu to swap
            switch(encoder_click_type()) 
            {
                case LONG_CLICK:
                    current_state = STATE_MENU;
                    debug_println("Switching to menu");
                    break;
                default:
                    break;
            }
            break;

        default:
            //we're firing, so don't do anything
            break;
    }
}

//determine what type of click was detected
uint8_t encoder_click_type()
{
    if (!encoder_clicked && encoder.ButtonPressed())
    {
        //initial click detection, start counting how long the button is held for
        encoder_clicked = true;
        encoder_last_clicked = millis();
        return NO_CLICK;
    }
    else if (encoder_clicked && encoder.ButtonPressed())
    {
        //button still held, don't do anything yet
        return NO_CLICK;
    }
    else if (encoder_clicked && !encoder.ButtonPressed())
    {
        //button has been released, figure out how long the button has been held for
        uint64_t click_duration = millis() - encoder_last_clicked;
        //reset the clicked boolean
        encoder_clicked = false;
        if (click_duration <= ENC_LONG_CLICK * 1000)
        {
            //if the duration is too short it's noise, otherwise it's a short click
            if (click_duration <= ENC_CLICK_DEBOUNCE)
            {
                return NO_CLICK;
            }
            return SHORT_CLICK;
        }
        else
        {  
            return LONG_CLICK;
        }
    }
    //if we get here then no click was detected
    return NO_CLICK;
}

void print_display()
{
    if (millis() - display_last_update < 200)
    {
        //don't draw the screen if the last draw was less than 200 miliseconds ago
        return;
    }
    display.clearDisplay();
    switch(current_state) 
    {
        //both normal and firing display the normal view
        case STATE_NORMAL:
        case STATE_FIRING:
            print_normal();
            break;
        case STATE_MENU:
            print_menu();
            break;
        case STATE_LOW_BATTERY:
            print_low_battery();
            break;
        case STATE_ERROR:
            print_error();
            break;
        default:
            print_error();
            break;
    }
    display_last_update = millis();
}

void print_normal()
{
    display.setCursor(0, 0);
    display.print("Battery: ");
    display.print(measured_voltage);
    display.println("V");
    display.print("Fire Mode: ");
    switch (fire_mode)
    {
        case SEMI_AUTO:
            display.println("SEMI-AUTO");
            break;
        case BINARY:
            display.println("BINARY");
            break;
        case BURST_FIRE:
            display.println("BURST");
            break;
        case FULL_AUTO:
            display.println("FULL AUTO");
            break;
        default:
            break;
    }
    display.print("Motor Speed: ");
    display.print(current_target_speed);
    display.println("%");
    display.display();
}

void print_menu()
{
    display.setCursor(0, 0);
    display_menu_item(current_menu_item);
    display_menu_item((current_menu_item + 1) % NUM_MENU_ITEMS);
    display_menu_item((current_menu_item + 2) % NUM_MENU_ITEMS);
    display_menu_item((current_menu_item + 3) % NUM_MENU_ITEMS);
    display_menu_item((current_menu_item + 4) % NUM_MENU_ITEMS);
    display_menu_item((current_menu_item + 5) % NUM_MENU_ITEMS);
    display_menu_item((current_menu_item + 6) % NUM_MENU_ITEMS);
    display_menu_item((current_menu_item + 7) % NUM_MENU_ITEMS);
    display.display();

}

void print_low_battery()
{
    display.setCursor(0, 8);
    display.setTextSize(3);
    display.println("LOW");
    display.println("BATTERY");
    display.setTextSize(1);
    display.display();
}

void print_error()
{
    display.setCursor(0, 8);
    display.setTextSize(3);
    display.println("ERROR: ");
    display.setTextSize(1);
    display.println(error_string.c_str());
    display.display();
}

//write the config to flash memory, to persist through reboots
void write_config()
{
    char* config_bytes = reinterpret_cast<char*>(&config);
    for (uint32_t i = 0; i < size_of_config; i++)
    {
        EEPROM.write(i, config_bytes[i]);
    }
    if(!EEPROM.commit())
    {
        debug_println("Failed to save config!");
        error_string = "Failed to save config!";
        current_state = STATE_ERROR;
    }
}

//read the config back from memory, then do some sanity checks
//reload the base config if the sanity checks fail
void read_config()
{
    char* new_config_bytes = (char*) malloc(size_of_config);
    for (uint32_t i = 0; i < size_of_config; i++)
    {
        new_config_bytes[i] = EEPROM.read(i);
    }
    configuration new_config = *reinterpret_cast<configuration*>(new_config_bytes);
    free(new_config_bytes);
    bool corrupted_config = false;
    corrupted_config = new_config.target_motor_speed < MIN_MOTOR_SPEED || new_config.target_motor_speed > MAX_MOTOR_SPEED; //target speed needs to be between 30 and 100
    corrupted_config = corrupted_config || (new_config.idle_motor_speed < MIN_IDLE_SPEED || new_config.idle_motor_speed > MAX_IDLE_SPEED); //idle speed needs to be between 0 and 100
    corrupted_config = corrupted_config || (new_config.motor_hold_time < MIN_HOLD_TIME || new_config.motor_hold_time > MAX_HOLD_TIME); 
    corrupted_config = corrupted_config || (new_config.full_auto_rof < MIN_ROF || new_config.full_auto_rof > MAX_ROF); //rof caps at 100 dps, just to allow sanity checking
    corrupted_config = corrupted_config || (new_config.burst_fire_rof < MIN_ROF || new_config.burst_fire_rof > MAX_ROF);
    corrupted_config = corrupted_config || (new_config.burst_size < MIN_BURST || new_config.burst_size > MAX_BURST);
    corrupted_config = corrupted_config || (new_config.solenoid_control != OPEN_LOOP && new_config.solenoid_control != CLOSED_LOOP);
    corrupted_config = corrupted_config || (new_config.solenoid_pulse_high_3s < MIN_PULSE || new_config.solenoid_pulse_high_3s > MAX_PULSE); //check all of the open loop solenoid settings
    corrupted_config = corrupted_config || (new_config.solenoid_pulse_high_4s < MIN_PULSE || new_config.solenoid_pulse_high_4s > MAX_PULSE);
    corrupted_config = corrupted_config || (new_config.solenoid_pulse_low_3s < MIN_PULSE || new_config.solenoid_pulse_low_3s > MAX_PULSE);
    corrupted_config = corrupted_config || (new_config.solenoid_pulse_low_4s < MIN_PULSE || new_config.solenoid_pulse_low_4s > MAX_PULSE);
    corrupted_config = corrupted_config || (new_config.solenoid_retract < MIN_PULSE || new_config.solenoid_retract > MAX_PULSE);
    corrupted_config = corrupted_config || (new_config.rev_trigger_option < 0 || new_config.rev_trigger_option > 3); //if it's outside of the four options it's corrupt
    corrupted_config = corrupted_config || (new_config.trigger_debounce < MIN_DEBOUNCE || new_config.trigger_debounce > MAX_DEBOUNCE);
    corrupted_config = corrupted_config || (new_config.battery_type != BAT_3S && new_config.battery_type != BAT_4S);
    if (corrupted_config)
    {
        debug_println("Failed to load config, restoring default config");
        load_default_config();
        return;
    }
    config = new_config;
    current_target_speed = config.target_motor_speed;
    bat_low = (config.battery_type == BAT_3S ? BAT_LOW_3S : BAT_LOW_4S);
    bat_high = (config.battery_type == BAT_3S ? BAT_HIGH_3S : BAT_HIGH_4S);
}

//load up the default config, used on the first launch and whenever the config gets corrupted
void load_default_config()
{
    configuration new_config;
    new_config.target_motor_speed = MOTOR_SPEED;
    new_config.idle_motor_speed = IDLE_SPEED;
    new_config.motor_hold_time = MOTOR_HOLD_TIME;
    new_config.full_auto_rof = FULL_AUTO_ROF;
    new_config.burst_fire_rof = BURST_FIRE_ROF;
    new_config.burst_size = BURST_SIZE;
    new_config.solenoid_control = SOLENOID_CONTROL;
    new_config.solenoid_pulse_high_3s = SOLENOID_PULSE_3S_HIGH;
    new_config.solenoid_pulse_high_4s = SOLENOID_PULSE_4S_HIGH;
    new_config.solenoid_pulse_low_3s = SOLENOID_PULSE_3S_LOW;
    new_config.solenoid_pulse_low_4s = SOLENOID_PULSE_4S_LOW;
    new_config.solenoid_retract = SOLENOID_RETRACT;
    new_config.rev_trigger_option = REV_TRIGGER_OPTION;
    new_config.trigger_debounce = TRIGGER_DEBOUNCE;
    new_config.battery_type = BAT_TYPE;
    config = new_config;
    bat_low = (config.battery_type == BAT_3S ? BAT_LOW_3S : BAT_LOW_4S);
    bat_high = (config.battery_type == BAT_3S ? BAT_HIGH_3S : BAT_HIGH_4S);
    //if the default is loaded that means we want to save over the old one
    write_config();
}

//handle the adjustment of settings within the menu
void adjust_config_by_menu_position(int32_t value_to_add, uint8_t menu_position)
{
    switch(menu_position)
    {
        case TARGET_MOTOR_SPEED_MENU:
            config.target_motor_speed += (value_to_add * 5);
            //if we've looped around all the way to near intmax, set back to 0
            if (config.target_motor_speed > (UINT32_MAX - 100))
            {
                config.target_motor_speed = 0;
            }
            config.target_motor_speed = clamp(config.target_motor_speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
            current_target_speed = config.target_motor_speed;
            break;
        case IDLE_MOTOR_SPEED_MENU:
            config.idle_motor_speed += (value_to_add * 5);
            //if we've looped around all the way to near intmax, set back to 0
            if (config.idle_motor_speed > (UINT32_MAX - 100))
            {
                config.idle_motor_speed = 0;
            }
            config.idle_motor_speed = clamp(config.idle_motor_speed, MIN_IDLE_SPEED, MAX_IDLE_SPEED);
            break;
        case MOTOR_HOLD_TIME_MENU:
            config.motor_hold_time += (value_to_add * 5);
            //if we've looped around all the way to near intmax, set back to 0
            if (config.motor_hold_time > (UINT32_MAX - 100))
            {
                config.motor_hold_time = 0;
            }
            config.motor_hold_time = clamp(config.motor_hold_time, MIN_HOLD_TIME, MAX_HOLD_TIME);
        case FULL_AUTO_ROF_MENU:
            config.full_auto_rof += value_to_add;
            //if we've looped around all the way to near intmax, set back to 0
            if (config.full_auto_rof > (UINT32_MAX - 100))
            {
                config.full_auto_rof = 0;
            }
            config.full_auto_rof = clamp(config.full_auto_rof, MIN_ROF, MAX_ROF);
            break;
        case BURST_FIRE_ROF_MENU:
            config.burst_fire_rof += value_to_add;
            //if we've looped around all the way to near intmax, set back to 0
            if (config.burst_fire_rof > (UINT32_MAX - 100))
            {
                config.burst_fire_rof = 0;
            }
            config.burst_fire_rof = clamp(config.burst_fire_rof, MIN_ROF, MAX_ROF);
            break;
        case BURST_SIZE_MENU:
            config.burst_size += value_to_add;
            //if we've looped around all the way to near intmax, set back to 0
            if (config.burst_size > (UINT32_MAX - 100))
            {
                config.burst_size = 0;
            }
            config.burst_size = clamp(config.burst_size, MIN_BURST, MAX_BURST);
            break;
        case SOLENOID_CONTROL_MENU:
            if (value_to_add == 0)
            {
                break; //end early if there's no change, otherwise just having the control optionselected will toggle it
            }
            config.solenoid_control = (config.solenoid_control == OPEN_LOOP ? CLOSED_LOOP : OPEN_LOOP);
            break;
        case SOLENOID_PULSE_HIGH_MENU:
            //we're only displaying the solenoid timings for the current battery, to keep the menu size more reasonable
            if (config.battery_type == BAT_3S)
            {
                config.solenoid_pulse_high_3s += value_to_add;
                //if we've looped around all the way to near intmax, set back to 0
                if (config.solenoid_pulse_high_3s > (UINT32_MAX - 100))
                {
                    config.solenoid_pulse_high_3s = 0;
                }
                config.solenoid_pulse_high_3s = clamp(config.solenoid_pulse_high_3s, MIN_PULSE, MAX_PULSE);
            }
            else
            {
                config.solenoid_pulse_high_4s += value_to_add;
                //if we've looped around all the way to near intmax, set back to 0
                if (config.solenoid_pulse_high_4s > (UINT32_MAX - 100))
                {
                    config.solenoid_pulse_high_4s = 0;
                }
                config.solenoid_pulse_high_4s = clamp(config.solenoid_pulse_high_4s, MIN_PULSE, MAX_PULSE);
            }
            break;
        case SOLENOID_PULSE_LOW_MENU:
            if (config.battery_type == BAT_3S)
            {
                config.solenoid_pulse_low_3s += value_to_add;
                //if we've looped around all the way to near intmax, set back to 0
                if (config.solenoid_pulse_low_3s > (UINT32_MAX - 100))
                {
                    config.solenoid_pulse_low_3s = 0;
                }
                config.solenoid_pulse_low_3s = clamp(config.solenoid_pulse_low_3s, MIN_PULSE, MAX_PULSE);
            }
            else
            {
                config.solenoid_pulse_low_4s += value_to_add;
                //if we've looped around all the way to near intmax, set back to 0
                if (config.solenoid_pulse_low_4s > (UINT32_MAX - 100))
                {
                    config.solenoid_pulse_low_4s = 0;
                }
                config.solenoid_pulse_low_4s = clamp(config.solenoid_pulse_low_4s, MIN_PULSE, MAX_PULSE);
            }
            break;
        case SOLENOID_RETRACT_MENU:
            config.solenoid_retract += value_to_add;
            //if we've looped around all the way to near intmax, set back to 0
            if (config.solenoid_retract > (UINT32_MAX - 100))
            {
                config.solenoid_retract = 0;
            }
            config.solenoid_retract = clamp(config.solenoid_retract, MIN_PULSE, MAX_PULSE);
            break;
        case REV_TRIGGER_OPTION_MENU:
            config.rev_trigger_option += value_to_add;
            //if we've looped around all the way to near intmax, set to the max rev trigger option value
            if (config.rev_trigger_option > (UINT32_MAX - 100))
            {
                config.rev_trigger_option = 3;
            }
            //if we're only a bit over the max rev trigger option, set back to 0
            else if (config.rev_trigger_option > 3)
            {
                config.rev_trigger_option = 0;
            }
            break;
        case TRIGGER_DEBOUNCE_MENU:
            config.trigger_debounce += (value_to_add * 5);
            //if we've looped around all the way to near intmax, set back to 0
            if (config.trigger_debounce > (UINT32_MAX - 100))
            {
                config.trigger_debounce = 0;
            }
            config.trigger_debounce = clamp(config.trigger_debounce, MIN_DEBOUNCE, MAX_DEBOUNCE);
            //set the triggers to use the new debounce
            main_trigger.setDebounceMs(config.trigger_debounce);
            rev_trigger.setDebounceMs(config.trigger_debounce);
            break;
        case BATTERY_TYPE_MENU:
            if (value_to_add == 0)
            {
                break; //end early if there's no change, otherwise just having the battery type selected will toggle it
            }
            config.battery_type = (config.battery_type == BAT_3S ? BAT_4S : BAT_3S);
            bat_low = (config.battery_type == BAT_3S ? BAT_LOW_3S : BAT_LOW_4S);
            bat_high = (config.battery_type == BAT_3S ? BAT_HIGH_3S : BAT_HIGH_4S);
            //if we've swapped we need to adjust the pid's outputs to account for the new RPM range
            motor1_pid.SetOutputLimits(0, bat_high * MOTOR_KV);
            motor2_pid.SetOutputLimits(0, bat_high * MOTOR_KV);
            break;
        default:
            break;
    }
}

//set up the menu array to easily convert from current_menu_item to the right string
void create_menu_array()
{
    menu_array[TARGET_MOTOR_SPEED_MENU] = "Target Speed: ";
    menu_array[IDLE_MOTOR_SPEED_MENU] = "Idle Speed: ";
    menu_array[MOTOR_HOLD_TIME_MENU] = "Motor Hold: ";
    menu_array[FULL_AUTO_ROF_MENU] = "Auto ROF: ";
    menu_array[BURST_FIRE_ROF_MENU] = "Burst ROF: ";
    menu_array[BURST_SIZE_MENU] = "Burst Amount: ";
    menu_array[SOLENOID_CONTROL_MENU] = "Noid Control: ";
    menu_array[SOLENOID_PULSE_HIGH_MENU] = "Noid Hi: ";
    menu_array[SOLENOID_PULSE_LOW_MENU] = "Noid Low: ";
    menu_array[SOLENOID_RETRACT_MENU] = "Noid Return: ";
    menu_array[REV_TRIGGER_OPTION_MENU] = "Rev Trigger: ";
    menu_array[TRIGGER_DEBOUNCE_MENU] = "T.Debounce: ";
    menu_array[BATTERY_TYPE_MENU] = "Battery: ";
    menu_array[EXIT_MENU] = "Exit";
}

//handles rendering individual menu items on the screen
void display_menu_item(uint8_t menu_position)
{
    if (menu_position == current_menu_item)
    {
        display.print(menu_item_selected ? "*" : ">");
    }
    else
    {
        display.print(" ");
    }
    display.print(menu_array[menu_position]);
    switch(menu_position)
    {
        case TARGET_MOTOR_SPEED_MENU:
            display.println(config.target_motor_speed);
            break;
        case IDLE_MOTOR_SPEED_MENU:
            display.println(config.idle_motor_speed);
            break;
        case MOTOR_HOLD_TIME_MENU:
            display.println(config.motor_hold_time);
            break;
        case FULL_AUTO_ROF_MENU:
            if (config.full_auto_rof == 0)
            {
                display.println("MAX");
            }
            else
            {
                display.println(config.full_auto_rof);
            }
            break;
        case BURST_FIRE_ROF_MENU:
            if (config.burst_fire_rof == 0)
            {
                display.println("MAX");
            }
            else
            {
                display.println(config.burst_fire_rof);
            }
            break;
        case BURST_SIZE_MENU:
            display.println(config.burst_size);
            break;
        case SOLENOID_CONTROL_MENU:
            display.println(config.solenoid_control == OPEN_LOOP ? "Open" : "Closed");
            break;
        case SOLENOID_PULSE_HIGH_MENU:
            display.println(config.battery_type == BAT_3S ? config.solenoid_pulse_high_3s : config.solenoid_pulse_high_4s);
            break;
        case SOLENOID_PULSE_LOW_MENU:
            display.println(config.battery_type == BAT_3S ? config.solenoid_pulse_low_3s : config.solenoid_pulse_low_4s);
            break;
        case SOLENOID_RETRACT_MENU:
            display.println(config.solenoid_retract);
            break;
        case REV_TRIGGER_OPTION_MENU:
            switch (config.rev_trigger_option)
            {
                case 0:
                    display.println("Disable");
                    break;
                case 1:
                    display.println("Toggle");
                    break;
                case 2:
                    display.println("Idle");
                    break;
                case 3:
                    display.println("Full");
                    break;
                default:
                    break;
            }
            break;
        case TRIGGER_DEBOUNCE_MENU:
            display.println(config.trigger_debounce);
            break;
        case BATTERY_TYPE_MENU:
            display.println(config.battery_type == BAT_3S ? "3S" : "4S");
            break;
        case EXIT_MENU:
            display.println();
            break;
        default:
            break;
    }
}

//handle reading the voltage coming in on the ADC pin, so we know how low the battery is
void read_voltage()
{
    uint32_t read_voltage = 0;
    uint32_t read_voltage_total = 0;
    //get multiple samples to get an average, single reads are too jittery
    //also if we read really low, it's just noise so ignore it
    for (uint32_t i = 0; i < 1000; i++)
    {
        read_voltage = voltage_pin.read();
        read_voltage_total += (read_voltage < 30 ? 0 : read_voltage);
    }
    // ((total measured) / samples) / max_adc_range) * adc_reference_voltage * voltage_divisor
    measured_voltage = ((read_voltage_total / 1000) / 4095.f) * 3.3 * 7;
    //if we're debugging and the voltage is < 1v that means it's not connected to battery so don't trigger the low battery state
    //also don't trigger the low battery state in the menu
    if (measured_voltage <= bat_low && (!DEBUG || measured_voltage > 1) && current_state != STATE_MENU)
    {
        current_state = STATE_LOW_BATTERY;
    }
    else if (current_state == STATE_LOW_BATTERY)
    {
        //if the voltage goes back up while in the low battery state it was a temporary sag, go back to normal
        current_state = STATE_NORMAL;
    }
}

static void main_trigger_pressed()
{
    if (current_state != STATE_NORMAL && current_state != STATE_FIRING)
    {
        //trigger only functions in normal and fire state
        //as a failsafe set darts to 0
        darts_to_fire = 0;
        return;
    }
    current_state = STATE_FIRING;
    switch (fire_mode)
    {
        //incrementing the dart count here instead of setting it to a single pull's value causes problems if the trigger
        //is spammed while the flywheels get up to speed, but responsiveness is faster with incrementing
        //so check to see if the flywheels are already at speed first
        case SEMI_AUTO:
            if(darts_to_fire >= 2)
            {
                //only let a single extra dart be queued up, so it doesn't keep firing long after the trigger is no longer being pressed
                break;
            }
            //fall through to the BINARY case since the rest of the code is shared
        case BINARY:
            if(darts_to_fire >= 3)
            {
                //BINARY needs to be able to queue up more extra darts than SEMI to be able to handle an extra trigger pull
                break;
            }
            if (at_target_speed)
            {
                darts_to_fire++;
            }
            else
            {
                darts_to_fire = 1;
            }
            break;
        case BURST_FIRE:
            if (darts_to_fire >= config.burst_size + 1)
            {
                //we've already queued up an extra burst, so don't do anything
                break;
            }
            if (at_target_speed)
            {
                darts_to_fire += config.burst_size;
            }
            else
            {
                darts_to_fire = config.burst_size;
            }
            break;
        case FULL_AUTO:
            darts_to_fire = 99; //set this high so it doesn't run out
            break;
    }
}

static void main_trigger_long_press_start()
{
    if (current_state != STATE_NORMAL && current_state != STATE_FIRING)
    {
        //trigger only functions in normal and fire state
        //as a failsafe set darts to 0
        darts_to_fire = 0;
        return;
    }
    if (fire_mode == FULL_AUTO)
    {
        darts_to_fire = 99;
    }
    else if (fire_mode == SEMI_AUTO)
    {
        //long press during semi-auto means toggle idle mode
        idle_on = !idle_on;
    }
}

static void main_trigger_long_press_held()
{
    if (current_state != STATE_NORMAL && current_state != STATE_FIRING)
    {
        //trigger only functions in normal and fire state
        //as a failsafe set darts to 0
        darts_to_fire = 0;
        return;
    }
    if (fire_mode == FULL_AUTO)
    {
        darts_to_fire = 99;
    }
}

static void main_trigger_long_press_stop()
{
    darts_to_fire = 0;
}

static void main_trigger_clicked()
{
    if (current_state != STATE_NORMAL && current_state != STATE_FIRING)
    {
        //trigger only functions in normal and fire state
        //as a failsafe set darts to 0
        darts_to_fire = 0;
        return;
    }
    if (fire_mode == BINARY)
    {
        //don't let the binary trigger stack up a huge number of darts or it'll
        //keep firing long after they stop spamming the trigger
        if(darts_to_fire >= 2)
        {
            return;
        }
        //binary needs one more dart when the trigger is released
        darts_to_fire++;
    }
}

static void rev_trigger_long_press_start()
{ 
    switch (config.rev_trigger_option)
    {
        case REV_IDLE_TOGGLE:
            idle_on = !idle_on;
            break;
        case REV_IDLE_HOLD:
            idle_on = true;
            break;
        case REV_FULL_HOLD:
            rev_without_firing = true;
            break;
        default:
            break;
    }
}

static void rev_trigger_long_press_stop()
{
    switch (config.rev_trigger_option)
    {
        case REV_IDLE_HOLD:
            idle_on = false;
            break;
        case REV_FULL_HOLD:
            rev_without_firing = false;
            break;
        default:
            break;
    }
}


//calculate how long the solenoid has to remain off to reach the target rof
//forward and back timing is closed-loop, but we need to figure out how long to wait to hit the target ROF
//the adjustment can end up negative if the solenoid can't reach the target rof, which we don't want, so return zero in that case
int32_t rof_adjustment_time(int32_t time_so_far)
{
    int32_t adjustment_time = 0;
    switch (fire_mode)
    {
        case BURST_FIRE:
            if (config.burst_fire_rof != 0)
            {
                adjustment_time = 1000 / config.burst_fire_rof;
            }
            break;
        case FULL_AUTO:
            if (config.full_auto_rof != 0)
            {
                adjustment_time = 1000 / config.full_auto_rof;
            }
            break;
        default:
            break;
    }
    return max(adjustment_time - time_so_far, 0); 
}

uint32_t firing_timer = 0;
uint32_t firing_start_time = 0;
bool currently_firing = false;

void handle_firing()
{
    if (config.solenoid_control == CLOSED_LOOP)
    {
        handle_firing_closed_loop();
    }
    else
    {
        handle_firing_open_loop();
    }
}

void handle_firing_closed_loop()
{
    if (current_state == STATE_ERROR)
    {
        darts_to_fire = 0;
        gpio_put(SOLENOID_PIN, false);
        solenoid_state = SOLENOID_ERROR;
        currently_firing = false;
        return;
    }
    if (current_state == STATE_FIRING && darts_to_fire == 0)
    {
        if (millis() - firing_timer >= MOTOR_HOLD_TIME)
        {
            current_state = STATE_NORMAL;
            solenoid_state = SOLENOID_OFF;
            gpio_put(SOLENOID_PIN, false);
            currently_firing = false;
        }
    }
    if (darts_to_fire > 0)
    {
        if (!currently_firing && at_target_speed)
        {
            firing_timer = millis();
            firing_start_time = millis();
            currently_firing = true;
        }
        uint32_t current_time = millis();
        switch (solenoid_state)
        {
            case SOLENOID_OFF:
                //if we get here and the solenoid is off, turn it on, but only if we're at the target speed
                if(currently_firing)
                {
                    gpio_put(SOLENOID_PIN, true);
                    solenoid_state = SOLENOID_FORWARD;
                }
                break;
            case SOLENOID_FORWARD:
                if (handle_break_beam() == IR_NO_BEAM_BROKEN)
                {
                    gpio_put(SOLENOID_PIN, false);
                    solenoid_state = SOLENOID_REVERSE;
                    firing_timer = millis();
                }
                else if (millis() - firing_timer >= SOLENOID_SAFETY_TIMER)
                {
                    //took too long to extend, turn off the solenoid, stop firing, and go to error
                    gpio_put(SOLENOID_PIN, false);
                    darts_to_fire = 0;
                    solenoid_state = SOLENOID_ERROR;
                    currently_firing = false;
                    current_state = STATE_ERROR;
                    error_string = "Solenoid failed to extend";
                    debug_println("failed to extend");
                }
                break;
            case SOLENOID_REVERSE:
                if (handle_break_beam() == IR_BOTH_BEAM_BROKEN)
                {
                    solenoid_state = SOLENOID_COOLDOWN;
                    firing_timer = millis();
                }
                else if (millis() - firing_timer >= SOLENOID_SAFETY_TIMER)
                {
                    //took too long to retract, stop firing and go to error
                    darts_to_fire = 0;
                    solenoid_state = SOLENOID_ERROR;
                    currently_firing = false;
                    current_state = STATE_ERROR;
                    error_string = "Solenoid failed to retract";
                    debug_println("failed to retract");
                }
                break;
            case SOLENOID_COOLDOWN:
                //if the solenoid has stayed off for the full cooldown, move to off to start the cycle over
                //also decrement how many darts are left to fire since we just finished firing one
                if (current_time - firing_timer >= rof_adjustment_time(current_time - firing_start_time))
                {
                    solenoid_state = SOLENOID_OFF;
                    darts_to_fire--;
                    currently_firing = false;
                    firing_timer = millis();
                }
                break;
            default:
                break;
        }
        
    }

}

void handle_firing_open_loop()
{
    if (current_state == STATE_ERROR)
    {
        darts_to_fire = 0;
        gpio_put(SOLENOID_PIN, false);
        solenoid_state = SOLENOID_ERROR;
        currently_firing = false;
        return;
    }
    if (current_state == STATE_FIRING && darts_to_fire == 0)
    {
        if (millis() - firing_timer >= MOTOR_HOLD_TIME)
        {
            current_state = STATE_NORMAL;
            solenoid_state = SOLENOID_OFF;
            gpio_put(SOLENOID_PIN, false);
            currently_firing = false;
        }
    }
    if (darts_to_fire > 0)
    {
        if (!currently_firing && at_target_speed)
        {
            firing_timer = millis();
            firing_start_time = millis();
            currently_firing = true;
        }
        uint32_t current_time = millis();
        switch (solenoid_state)
        {
            case SOLENOID_OFF:
            //if we get here and the solenoid is off, turn it on, but only if we're at the target speed
                if(currently_firing)
                {
                    gpio_put(SOLENOID_PIN, true);
                    solenoid_state = SOLENOID_FORWARD;
                }
                break;
            case SOLENOID_FORWARD:
                if (millis() - firing_timer >= mapped_pulse_time())
                {
                    gpio_put(SOLENOID_PIN, false);
                    solenoid_state = SOLENOID_REVERSE;
                    firing_timer = millis();
                }
                break;
            case SOLENOID_REVERSE:
                if (millis() - firing_timer >= config.solenoid_retract)
                {
                    solenoid_state = SOLENOID_COOLDOWN;
                    firing_timer = millis();
                }
                break;
            case SOLENOID_COOLDOWN:
                //if the solenoid has stayed off for the full cooldown, move to off to start the cycle over
                //also subtract one from darts_to_fire
                if (current_time - firing_timer >= rof_adjustment_time(current_time - firing_start_time))
                {
                    solenoid_state = SOLENOID_OFF;
                    darts_to_fire--;
                    currently_firing = false;
                    firing_timer = millis();
                }
                break;
            default:
                break;
        }
        
    }
}

//the solenoid's forward stroke slows down with lower voltage, so at lower voltages we need to hold it on longer
//this function handles both that mapping and determining which set of pulse times to use, based on battery type
uint32_t mapped_pulse_time()
{
    uint32_t mapped_pulse_time = 0;
    if (config.battery_type == BAT_3S)
    {
        //multiply the voltages by 10 because they're floats and map takes longs
        mapped_pulse_time = map(measured_voltage * 10, BAT_LOW_3S * 10, BAT_HIGH_3S * 10, config.solenoid_pulse_low_3s, config.solenoid_pulse_high_3s);
    }
    else
    {
        mapped_pulse_time = map(measured_voltage * 10, BAT_LOW_4S * 10, BAT_HIGH_4S * 10, config.solenoid_pulse_low_4s, config.solenoid_pulse_high_4s);
    }
    return clamp(mapped_pulse_time, MIN_PULSE, MAX_PULSE);
}


//convert a percentage value (0-100) to throttle values (0-2000)
uint32_t percent_to_throttle(uint32_t percent)
{
    return (clamp(percent, 0, 100) / 100.0) * 2000;
}

//calculate what percent corresponds to what rpm value (theoretical)
//usually the eRPM ends up higher than this calculated value
uint32_t percent_to_rpm(uint32_t percent)
{

    return (clamp(percent, 0, 100) / 100.0 * (measured_voltage == 0 && DEBUG ? bat_high : measured_voltage) * MOTOR_KV);
}

uint32_t erpm_to_rpm(uint32_t eRPM)
{
    return eRPM /= MOTOR_POLES / 2;
}

//the PID loop operates on RPM values, we need to convert those to throttle values to feed the motors
uint32_t rpm_to_throttle(uint32_t rpm)
{
    return clamp((rpm / ((measured_voltage == 0 && DEBUG ? bat_high : measured_voltage) * MOTOR_KV) * 2000), 0, 2000);
}

uint32_t clamp(uint32_t value, uint32_t min, uint32_t max)
{
    assert(min < max);
    if (value < min)
    {  
        return min;
    } 
    else if (value > max)
    {
        return max;
    } 
    else
    {
        return value;
    }
}


//functions to write to serial for debugging, so I don't have to wrap each individual call in
//if(DEBUG)
template <typename T> void debug_println(T print)
{
    if(DEBUG)
    {
        Serial.println(print);
    }
}

template <typename T> void  debug_print(T print)
{
    if(DEBUG)
    {
        Serial.print(print);
    }
}