#include "odrive.hpp"
static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

#define MIN_STICK 360       
#define MAX_STICK 1673      

#define MIN_DEADBAND 1014
#define MAX_DEADBAND 1034

#define MID_STICK 1024
#define DIVIDER 2           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

float MAX_RPM = 250.0;         // Max RPM of the wheels, this is limited by wheels itself. Default is 144
float ZERO_RPM = 0.0;          // No speed

 
ODrive::ODrive(Stream& serial)
    : serial_(serial) {}
 
void ODrive::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}
 
void ODrive::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}
 
void ODrive::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    std::stringstream ss;
    ss << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
}
 
void ODrive::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}
 
void ODrive::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    std::stringstream ss;
    ss  << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
}
 
float ODrive::readFloat() {
    return atof(readString().c_str());
}
 
int32_t ODrive::readInt() {
    return atoi(readString().c_str());
}
 
void ODrive::FinishedSending(){
    
}

float ODrive::GetVelocity(int motor_number){
    std::stringstream ss;
    ss << "r axis" << motor_number << ".encoder.vel_estimate\n";
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());

    return readFloat();
}

float ODrive::cpsTOrpm(float cps){
    return (cps/cpr)*60.0;
}

float ODrive::rpmTOcps(float rpm){
    return (rpm/60.0)*cpr;
}

void ODrive::SetRPM(int motor_number, float RPM){
    float CPS;
    CPS = rpmTOcps(RPM);
    SetVelocity(motor_number,CPS);
}

float ODrive::GetRPM(int motor_number){
    float CPS;
    float RPM;
    CPS = GetVelocity(motor_number);
    RPM = cpsTOrpm(CPS);
    return RPM;
}

int ODrive::readError(int motor_number){
    std::stringstream ss;
    ss << "r axis" << motor_number << ".error\n";
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());

    return readInt();
}

 
bool ODrive::run_state(int axis, int requested_state, bool wait) {
    int timeout_ctr = 100;
    std::stringstream ss;
    ss   << "w axis" << axis << ".requested_state " << requested_state << '\n';
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    if (wait) {
        do {
            Thread::wait(100);
            std::stringstream ss2;
            ss2  << "r axis" << axis << ".current_state\n";
            serial_.write((const uint8_t *)ss2.str().c_str(),ss2.str().length());
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }
 
    return timeout_ctr > 0;
}
 
std::string ODrive::readString() {
    Timer timer;
    std::string str = "";
    static const unsigned long timeout = 1000;
    timer.reset();
    timer.start();
    for (;;) {
        while (!serial_.readable ()) {
            if (timer.read_ms() >= timeout) {
                return str;
            }
        }
        char c = serial_.getc ();
        if (c == '\n')
            break;
        str += c;
    }
    timer.stop();
    //buffered_pc.printf(str.c_str());
    return str;
}
 
float ODrive::readBattery(){
        std::stringstream ss;
   ss   <<"r vbus_voltage"<< '\n';
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    return readFloat();
    
    }

void ODrive::DriveWheels(float rpmR, float rpmL){
    SetRPM(0,rpmR);
    SetRPM(1,rpmL);
}


long ODrive::map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ODrive::vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2])
{   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

    float MIN_SCALER = 1000.0;
    float MAX_SCALER = 2000.0;
    

    /////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND)
    {
        MotorRPM[0] = 0.0;
        MotorRPM[1] = 0.0;
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && (UD_ch > MAX_DEADBAND || UD_ch < MIN_DEADBAND))
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[1] = -MotorRPM[0];

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND && (LR_ch >= MAX_DEADBAND || LR_ch <= MIN_DEADBAND))
    {
        MotorRPM[1] = (float)map(LR_ch, MIN_STICK, MAX_STICK, -MAX_RPM/2, MAX_RPM/2);
        MotorRPM[0] = MotorRPM[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = -(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[0] = -MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[1] = -MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[1] = -MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = -(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[0] = -MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }  
   
}