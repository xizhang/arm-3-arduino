#ifndef ARM_THREE_H
#define ARM_THREE_H

#include "Arduino.h"
#include <Servo.h>

// System params.
const int kPosQueueSize = 100;
const float kPi = 3.141592653589793238463;
// Delay and steps.
const int kHeartbeatDelay = 5;
const int kMoveToDelay = 100;
const int kStepsForArmLength = 200;
const int kMinServoMs = 544;
const int kMaxServoMs = 2400;

struct Position {
    float x;
    float y;
    float z;
    int d; //delay
};

class ArmThree
{
	public:  
		ArmThree(Servo* base_servo, Servo* upper_servo, Servo* fore_servo);
		void SetParams(float base_angle_offset, float base_angle_coef, 
		    float upper_angle_offset, float upper_angle_coef,
		    float fore_angle_offset, float fore_angle_coef,
		    float upper_len, float fore_len,
		    float base_pos_offset, float fore_pos_offset,
		    float x0, float y0);
        void Start();
        bool AddPosition(float x, float y, float z, int d);
        void MoveTo(float x, float y, float z);
        void Stop();
        bool Heartbeat();
        // tests
      	void PrintPositions();
    private:
    	// Arm's Servo pin #
    	Servo* base_servo_;
    	Servo* upper_servo_;
    	Servo* fore_servo_;
    	// Arm's base coordinates
      	float x0_;
    	  float y0_;
        // Arm angles' offsets and coeffecient
        float base_angle_offset_;
        float base_angle_coef_;
        float upper_angle_offset_;
        float upper_angle_coef_;
        float fore_angle_offset_;
        float fore_angle_coef_;
        // Arm's length
        float upper_len_;
        float fore_len_;
        // Arm's offset
        float base_pos_offset_;
        float fore_pos_offset_;
      	// Queued future positions.
      	Position positions_[kPosQueueSize];
      	int current_pos_index_;
      	int final_pos_index_;
      	int current_step_;
        // record last servo pos in case the arm trying to reach an impossible pos
        int last_base_ms_;
        int last_upper_ms_;
        int last_fore_ms_;
        int last_pos_cycles_;
      	// Private functions
      	void SetDefaultParams();
      	bool MoveToPos(float x, float y, float z);
      	float ConvertToDegree(float value);
        int ConvertToMs(float value);
      	int Round(float value);
      	Position CreatePosition(float x, float y, float z, int d);
      	int GetNextPosIndex(int pos);
      	float GetDistance(Position p1, Position p2);
      	float GetStepLength();

};

#endif