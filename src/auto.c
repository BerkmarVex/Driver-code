
#include "main.h"

int count;
int counts;
int line;

struct PID {
  	int sensor;
  	int target;
  	int error;
  	int previous_error;
  	double Kp;
  	double Ki;
  	double Kd;
  	int	bias;
  	int	iTime;
  	int	integral;
  	int	derivative;
  	double pid;
  	int output;
  };

struct PID leftMotor = {
  .Kp = 0.5,
  .Ki = 0.0,
  .Kd = 0.2,
  .error = 0,
  .previous_error = 0,
  .integral = 0,
  .derivative = 0,
  .target = 2038,
  .sensor = 0
};

struct PID rightMotor = {
  .Kp = 0.5,
  .Ki = 0.0,
  .Kd = 0.2,
  .error = 0,
  .previous_error = 0,
  .integral = 0,
  .derivative = 0,
  .target = -1991,
  .sensor = 0
};

struct PID glLift = {
  .Kp = 0.2,
  .Ki = 0.0,
  .Kd = 0.8,
  .error = 0,
  .previous_error = 0,
  .integral = 0,
  .derivative = 0,
  .target = 800,
  .sensor = 0
};

int pidDo(struct PID *this) {
	this->error = this->target - this->sensor;
	this->integral = this->integral + (this->error);
	this->derivative = (this->error - this->previous_error);
	this->pid = (this->Kp) * this->error + (this->Ki) * this->integral + (this->Kd) * this->derivative + this->bias;
	this->output = (int) this->pid;
	this->previous_error = this->error;
  return this->output;
};

void updates(){
  line = analogRead(2);
  imeGet(0, &count);
  imeGet(1, &counts);
}

void autonomous() {
  analogCalibrate(2);
	imeReset(IME_RIGHT_MOTOR);
	imeReset(IME_LEFT_MOTOR);

  pidDo(&rightMotor);
  pidDo(&leftMotor);

  while(1){
    motorSet(2, pidDo(&rightMotor));
    motorSet(3, -pidDo(&leftMotor));
    motorSet(4, pidDo(&glLift));
  }
}
