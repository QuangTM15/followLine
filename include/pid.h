#ifndef PID_H
#define PID_H

// update PID and return correction value
float updatePID(float error);

// reset PID state (optional)
void resetPID();

#endif
