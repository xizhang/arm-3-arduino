#include "Arduino.h"
#include <Servo.h>
#include "arm_three.h"

ArmThree::ArmThree(Servo* base_servo, Servo* upper_servo, Servo* fore_servo) {
	base_servo_ = base_servo;
	upper_servo_ = upper_servo;
	fore_servo_ = fore_servo;
	SetDefaultParams();
}

void ArmThree::Start() {
	// Sets initial position
	float xx = x0_;
	float yy = (upper_len_ + fore_len_) / 4;
	float zz = (upper_len_ + fore_len_) / 4;
	MoveTo(xx, yy, zz);
	// Setup position queue
	current_pos_index_ = 0;
	final_pos_index_ = 0;
	current_step_ = 0;
	last_pos_cycles_ = 0;
	positions_[current_pos_index_] = CreatePosition(xx, yy, zz, 100);
}

void ArmThree::SetDefaultParams() {
	// Offset
	base_angle_offset_ = -8;
	base_angle_coef_ = 1;
	upper_angle_offset_ = 19;
	upper_angle_coef_ = 1;
	fore_angle_offset_ = 27;
    fore_angle_coef_ = 1;
    // Arm sizes
    upper_len_ = 1; // 13.9cm
	fore_len_ = 1.1;
	// pos offsets
	base_pos_offset_ = 0.09;
    fore_pos_offset_ = 0.21;
    // arm position
    x0_ = (upper_len_ + fore_len_) / 4;
    y0_ = -(upper_len_ + fore_len_) / 3;
}

void ArmThree::SetParams(float base_angle_offset, float base_angle_coef,
		float upper_angle_offset, float upper_angle_coef,
		float fore_angle_offset, float fore_angle_coef,
		float upper_len, float fore_len,
		float base_pos_offset, float fore_pos_offset,
		float x0, float y0) {
	base_angle_offset_ = base_angle_offset;
	base_angle_coef_ = base_angle_coef;
	upper_angle_offset_ = upper_angle_offset;
	upper_angle_coef_ = upper_angle_coef;
	fore_angle_offset_ = fore_angle_offset;
    fore_angle_coef_ = fore_angle_coef;
    upper_len_ = upper_len;
	fore_len_ = fore_len;
	base_pos_offset_ = base_pos_offset;
    fore_pos_offset_ = fore_pos_offset;
    x0_ = x0;
    y0_ = y0;
}

void ArmThree::Stop() {
	
}

bool ArmThree::AddPosition(float x, float y, float z, int d) {
	int nextfinalPosIndex = GetNextPosIndex(final_pos_index_);
	if (nextfinalPosIndex == current_pos_index_) {
		// Queue is full. No-op.
		return false;
	}
	Position newPos = CreatePosition(x, y, z, d);
	positions_[nextfinalPosIndex] = newPos;
	final_pos_index_ = nextfinalPosIndex;
}

void ArmThree::MoveTo(float x, float y, float z) {
	MoveToPos(x, y, z);
	delay(kMoveToDelay);
}

/**
 * Move to a given (x, y, z) as fast as possible.
 * The trail of the pen point might be a curve.
 */
bool ArmThree::MoveToPos(float x, float y, float z) {
	if (z == 0) {
		z = 0.00001;
	}
	// Calculating base angle
	float c2_tmp = sqrt((x - x0_) * (x - x0_) + (y - y0_) * (y - y0_));
	if (c2_tmp == 0) {
		c2_tmp = 0.00001;
	}
	float base_angle = acos((x0_ - x) / c2_tmp);
	// real base coordinates
	float x0 = x0_ - cos(base_angle) * base_pos_offset_;
	float y0 = y0_ + sin(base_angle) * base_pos_offset_;
	// real target coordinates
	float xx = x + cos(base_angle) * fore_pos_offset_;
	float yy = y - sin(base_angle) * fore_pos_offset_;
	float zz = z;
	
	// calculating distances
	float c2 = sqrt((xx - x0) * (xx - x0) + (yy - y0) * (yy - y0));
	float c1 = sqrt(c2 * c2 + z * z);
	if (c1 == 0) {
		c1 = 0.00001;
	}
	// upperarm angle
	float upper_angle1 = atan(zz / c2);
	float upper_angle2 = acos((c1 * c1 + upper_len_ * upper_len_ - fore_len_ * fore_len_) / (2 * upper_len_ * c1));
	float upper_angle = upper_angle1 + upper_angle2;
	// forearm angle
	float fore_angle1 = acos((c1 * c1 + fore_len_ * fore_len_ - upper_len_ * upper_len_) / (2 * fore_len_ * c1));
	float fore_angle2 = atan(c2 / z);
	float fore_angle = fore_angle1 + fore_angle2 - kPi / 2;
	if (fore_angle < 0) {
		fore_angle = kPi + fore_angle;
	}
	float base_degree = ConvertToDegree(base_angle) * base_angle_coef_ + base_angle_offset_;
	float upper_degree = ConvertToDegree(upper_angle) * upper_angle_coef_ + upper_angle_offset_;
	float fore_degree = ConvertToDegree(fore_angle) * fore_angle_coef_ + fore_angle_offset_; 
	int base_ms = ConvertToMs(base_degree);
	int upper_ms = ConvertToMs(upper_degree);
	int fore_ms = ConvertToMs(fore_degree);
	base_servo_->writeMicroseconds(base_ms);
	upper_servo_->writeMicroseconds(upper_ms);
	fore_servo_->writeMicroseconds(fore_ms);
	return true;
}

bool ArmThree::Heartbeat() {
	if (current_pos_index_ == final_pos_index_) {
		// No need to move. Already there.
		return false;
	}
	Position currentPos = positions_[current_pos_index_];
	int nextcurrentPosIndex = GetNextPosIndex(current_pos_index_);
	Position nextPos = positions_[nextcurrentPosIndex];
	float distance = GetDistance(currentPos, nextPos);
	float totalSteps = ceil(distance / GetStepLength());
	if (totalSteps > 0) {
		float percentage = current_step_ / totalSteps;
		// Move a step
		float nextX = currentPos.x + (nextPos.x - currentPos.x) * percentage;
		float nextY = currentPos.y + (nextPos.y - currentPos.y) * percentage;
		float nextZ = currentPos.z + (nextPos.z - currentPos.z) * percentage;
		MoveToPos(nextX, nextY, nextZ);
		current_step_++;
	}
	
	if (current_step_ > totalSteps || totalSteps == 0) {
		// Move to the next position.
		current_pos_index_ = nextcurrentPosIndex;
		current_step_ = 0;
		delay(nextPos.d);
		
	}
	delay(kHeartbeatDelay);
	return true;
}

//--- Private ------------
int ArmThree::Round(float value) {
	return (int)(value + .5);
}

float ArmThree::ConvertToDegree(float value) {
	return (value / kPi) * 180;
}

int ArmThree::ConvertToMs(float degree) {
	return Round(degree / 180 * (kMaxServoMs - kMinServoMs) + kMinServoMs);
}

Position ArmThree::CreatePosition(float x, float y, float z, int d) {
	Position position = Position();
	position.x = x;
	position.y = y;
	position.z = z;
	position.d = d;
	return position;
}

int ArmThree::GetNextPosIndex(int posIndex) {
	int nextPosIndex = posIndex + 1;
	if (nextPosIndex > kPosQueueSize - 1) {
		return 0;
	}
	return nextPosIndex;
}

float ArmThree::GetDistance(Position p1, Position p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

float ArmThree::GetStepLength() {
	return (upper_len_ + fore_len_) / kStepsForArmLength;
}

// ------- Testing -------------

void ArmThree::PrintPositions() {
}
