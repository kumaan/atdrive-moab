#ifndef ODrive_h
#define ODrive_h
#include <string>
#include <sstream>
#include <stdint.h>
#include "mbed.h"
#include <cstdlib>
class ODrive {
public:
    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };
 
    ODrive(Stream& serial);
 
    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    
    void FinishedSending();

    float GetVelocity(int motor_number);

    float cpsTOrpm(float cps);
    float rpmTOcps(float rpm);
    void SetRPM(int motor_number, float RPM);
    float GetRPM(int motor_number);
    int readError(int motor_number);

 
    // General params
    float readFloat();
    int32_t readInt();
    float readBattery();
 
    // State helper
    bool run_state(int axis, int requested_state, bool wait);
    std::string readString();

    long map(long x, long in_min, long in_max, long out_min, long out_max); 

    void DriveWheels(float rpmR, float rpmL);

    void DriveWheels(float rpmR, float rpmL, float readRPM[2]);

    void vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]);


 
    Stream& serial_;
private:
    float cpr = 258.0;
    float MAX_RPM = 200.0;         // Max RPM of the wheels, this is limited by wheels itself. Default is 144
    float ZERO_RPM = 0.0;          // No speed
    int MIN_STICK = 360;       
    int MAX_STICK = 1673;     

    int MIN_DEADBAND = 1014;
    int MAX_DEADBAND = 1034;

    int MID_STICK = 1024;
    int DIVIDER = 2;           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

};
 
#endif //ODrive_h