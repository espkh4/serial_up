#ifndef COMMSTRUCT_H
#define COMMSTRUCT_H

#define FRAME_SIZE          76
#define Start_Bytes         2
#define Pos_Info            3
#define Vel_Info            3
#define Sonar_Info          6
#define IR_Info             6
#define Motor_Info          6
#define P_Info              3
#define I_Info              3
#define D_Info              3
#define u_Info              3
#define master_index        0
#define slave1_index        1
#define slave2_index        2
#define deg2rad             M_PI/180
#define rpm2radps           0.1047198
#define max(x,y) (x>y?x:y)
#define sgn(x) (x>0 ? 1: -1)

/*Structures and Unions for variables*/
union Data_Frame
{
    struct Individual_bytes
    {
    unsigned char   start[Start_Bytes];
    signed short    position[Pos_Info],vel[Vel_Info],P[P_Info],I[I_Info],D[D_Info],u[u_Info];
    unsigned char   ir[IR_Info];
    unsigned short  sonar[Sonar_Info], frames_sent, compass, motor_current[Motor_Info];
    unsigned short  timestamp;
    unsigned char   bumper,checksum;
    }INFO;

unsigned char Complete[FRAME_SIZE];

}GUI;

#endif