#include "tm4c123gh6pm.h"
#include "OS.h"
#include "MCP9600.h"
#include "ST7735.h"
#include "ros.h"
// #include "Vector3.h"
#include "Float32.h"
#include "servo.h"

#define TIMESLICE TIME_2MS  //thread switch time in system time units

// void ROSVector3SubscriberExample(void) {
// 	rospacket_t *message;
// 	rosvector3_t *vector3message;

// 	ROS_SubscriberInit(ROS_FindSubscriber());
// 	while(1) {
// 		// Wait for message
// 		message = ROS_MailBox_Recv(&(ROS_FindSubscriber()->mailbox));
// 		if (message) {
// 			// Deserialize data
// 			vector3message = ROS_Vector3Deserialize(message);
// 			if (vector3message) {
// 				// Do whatever
//         ST7735_Message(ST7735_DISPLAY_TOP, 0, "X:", vector3message->x);
//         ST7735_Message(ST7735_DISPLAY_TOP, 1, "Y:", vector3message->y);
//         ST7735_Message(ST7735_DISPLAY_TOP, 2, "Z:", vector3message->z);


// 				// ## Helper parameters
// 				// # C -> distance between first joint and end effector joint
// 				// C = sqrt((data.y - E)**2 + (data.z - B)**2)
// 				// # theta -> angle that C makes with y axis
// 				// theta = degrees(atan((data.z - B)/(data.y - E)))

// 				// # Alpha equation
// 				// alpha = 180 - theta - degrees(acos((L2**2 - L1**2 - C**2)/(2*L1*C)))
// 				// # Beta equation
// 				// beta = degrees(acos((C**2 - L1**2 - L2**2)/(2*L1*L2))) - alpha

// 				// angles[0] = alpha - 70 # L1
// 				// angles[1] = 95 - beta # L2
// 				// angles[2] = degrees(atan2(data.y, data.x)) # base



// 				// Deallocate data
// 				ROS_Vector3Free(vector3message);
// 			}
//       // Deallocate packet 
//       ROS_PacketFree(message);
// 		}
// 	}
// }

void s1(void) {
	rospacket_t *message;
	rosfloat32_t *data;
	ROS_SubscriberInit(ROS_FindSubscriber());
	while(1) {
		// Wait for message
		message = ROS_MailBox_Recv(&(ROS_FindSubscriber()->mailbox));
		if (message) {
			// Deserialize data
			data = ROS_Float32Deserialize(message);
			if (data) {
        ST7735_Message(ST7735_DISPLAY_TOP, 0, "S1:", data->data);

				Servo_SetAngle(0, data->data);

				// Deallocate data
				ROS_Float32Free(data);
			}
      // Deallocate packet 
      ROS_PacketFree(message);
		}
	}
}

void s2(void) {
	rospacket_t *message;
	rosfloat32_t *data;
	ROS_SubscriberInit(ROS_FindSubscriber());
	while(1) {
		// Wait for message
		message = ROS_MailBox_Recv(&(ROS_FindSubscriber()->mailbox));
		if (message) {
			// Deserialize data
			data = ROS_Float32Deserialize(message);
			if (data) {
        ST7735_Message(ST7735_DISPLAY_TOP, 1, "S2:", data->data);

				Servo_SetAngle(1, data->data);

				// Deallocate data
				ROS_Float32Free(data);
			}
      // Deallocate packet 
      ROS_PacketFree(message);
		}
	}
}

void s3(void) {
	rospacket_t *message;
	rosfloat32_t *data;
	ROS_SubscriberInit(ROS_FindSubscriber());
	while(1) {
		// Wait for message
		message = ROS_MailBox_Recv(&(ROS_FindSubscriber()->mailbox));
		if (message) {
			// Deserialize data
			data = ROS_Float32Deserialize(message);
			if (data) {
        ST7735_Message(ST7735_DISPLAY_TOP, 2, "S3:", data->data);

				Servo_SetAngle(2, data->data);

				// Deallocate data
				ROS_Float32Free(data);
			}
      // Deallocate packet 
      ROS_PacketFree(message);
		}
	}
}


// OS and modules initialization
int main(void) {        // lab 4 real main

  // Initialize the OS
  OS_Init();

	// Initialize servos
	Servo_Init(0);
	Servo_Init(1);
	Servo_Init(2);

	ROS_AddSubscriber("s1", ROS_Float32MSG(), ROS_Float32MD5(), &s1, 200, 2);
	ROS_AddSubscriber("s2", ROS_Float32MSG(), ROS_Float32MD5(), &s2, 200, 2);
	ROS_AddSubscriber("s3", ROS_Float32MSG(), ROS_Float32MD5(), &s3, 200, 2);

  // Launch the OS
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}