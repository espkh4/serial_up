#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "CerealPort.h"
#include <cstdlib>
#define m_pi 3.1415926536
#define REPLY_SIZE 1000000
#define TIMEOUT 1000
double gyrocalrate = 0;
double gyrocaltime = 0;

//*************************************************************************
int ReadCommandLine(int argc, char **argv);
//*************************************************************************

int main(int argc, char **argv)
{
    /// dong modify calibration
        int gyromode = 0;
        ReadCommandLine(argc,argv);
        if(gyrocaltime==0)gyromode = 0;else gyromode = 1;

        double anglestart = 0;
        double timestart = 0, timenow = 0;
        if(gyromode == 0){
            FILE *fp; int ret = 0;
            fp = fopen("gyro.cfg","r");
            if(fp!=NULL){
                ret = fscanf(fp,"%lf", &gyrocalrate);
                printf("Get stored %d stored value : %.6lf\n",ret,gyrocalrate);
                fclose(fp);
            };//end if fpnull
        };//end if
        if(gyromode == 1)printf("************** Now in Calibration ********** to %.2lf sec\n",gyrocaltime);
    /// end dong modify

	char reply[REPLY_SIZE];
	char command={'P'};//This selects output of gyro
    double tempangle=0;//vth = 0,

	/*Serial for Gyro Communication*/
	cereal::CerealPort device;	
    try{ device.open("/dev/ttyUSB1",38400); }//Opens serial port
	catch(cereal::Exception& e)
	{
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
	}
	if(device.portOpen())
	{
	ROS_INFO("The serial port is opened @ baudrate %d",device.baudRate());
	}
	
	switch (command)
	{
	case 'R':
	printf("Output switched to Rate\n");
	break;
	case 'A':
	printf("Output switched to Incremental Angle\n");
	break;
	case 'P':
	printf("Output switched to Integrated Angle\n");
	break;
	case 'Z':
	printf("Integrated Angle value has been zeroed\n");
	break;
	default:
	printf("Unknown or no command sent\n");
	}

	if(device.write(&command,sizeof(command))==1)
	{
	printf("Command sent\n");
	}
	else
	{
	printf("Error in sending requested command\n");
	}

  ros::init(argc, argv, "Gyro");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("Gyro_value", 1000);
  ros::Rate loop_rate(1000);
  std_msgs::Float32 msg;


  while (ros::ok())
  {
	try{ device.readLine(reply, TIMEOUT);}	//Reading
	catch(cereal::TimeoutException& e)	//Catch any possible errors
	{
	ROS_ERROR("Timeout!");
	}
	tempangle=::atof(reply);	//ASCII to float
	tempangle=360-tempangle;
    tempangle = (tempangle*m_pi)/180;

    /// dong modify calibration
    double timediff = 0, newangle = 0;
    timenow = ros::Time::now().toSec();
    if(timestart == 0){
        timestart = timenow;
        anglestart = tempangle;
    }
    switch(gyromode){
        case 0:
            timediff = timenow- timestart;
            newangle = tempangle - timediff * gyrocalrate;
            if(newangle<0)newangle+= (2*m_pi);
            if(newangle>2*m_pi)newangle-= (2*m_pi);
            break;
        case 1:
            // init or calibrate
            newangle = tempangle;
            timediff = timenow- timestart;
            if(timediff>0)gyrocalrate = (tempangle - anglestart) / timediff;
            printf("**** Now in Calibration *** %.2lf to %.2lf sec **** rate = %.6lf\n",timediff,gyrocaltime,gyrocalrate);
            if(timediff > gyrocaltime){
                gyrocaltime = 0;
                gyromode = 0;
                FILE *fp;
                fp = fopen("gyro.cfg","w");
                fprintf(fp,"%lf",gyrocalrate);
                fclose(fp);
            };//end if timediff
            break;
        default:
            printf("************* Wrong GyroMode %d **************\n",gyromode);
            break;
    };//end switch

    /// end dong modify
    ROS_INFO("%f   n=%lf   r=%lf\r",tempangle,newangle,gyrocalrate);
    msg.data=newangle;
    chatter_pub.publish(msg);

	ros::spinOnce();
	loop_rate.sleep();  
	}
  return 0;
}
//****************************************************************
/// dong modify
/// --run to run normal gyro with stored calibration data from gyro.cfg file
/// --calibrate to run 1000 sec calibration and store to gyro.cfg file
/// --cal sec to run "sec" seconds calibration and store to gyro.cfg file
int ReadCommandLine(int argc, char **argv)
{
    char tempc;
    int i=0;
    double tempvalue = 0;
    //handle command parameters
    if(argc<20 && argc>1){
        for(i=1;i<argc;i++){
            tempc=(argv[i])[0];
            if(tempc=='-'){
                printf("The %d cmd string is %s\n",i,argv[i]);
                if( strcmp("--help",argv[i])==0 ){
                    printf("--run or --calibrate or --cal sec\n");
                };//end if

                //---------------------- Speed -------------------------
                if(strcmp("--cal",argv[i])==0 && i<argc-1){
                    tempvalue = atof(argv[i+1]);
                    if(tempvalue>0 && tempvalue < 1000.0)gyrocaltime = tempvalue;
                    else {
                        printf("Wrong calibration time setting : the %d string is %s\n",i+1,argv[i+1]);
                    };//end if else
                    i++;
                };//end if

                if(strcmp("--calibrate",argv[i])==0 )gyrocaltime = 1000;
                if(strcmp("--run",argv[i])==0 )gyrocaltime = 0;
            }else{
                printf("Warning: I cannot recognize the %d string : %s\n",i,argv[i]);
            };//end if
        };//end for i
    };//end if argc
    if(argc>=20){
        printf("Too many arguments -- exceed processing maximum, quit program!\n");
    };//end if else
    if(argc==1)printf("--run or --calibrate or --cal sec\n");

    return i;
}
/*
Public Send_Input As Boolean
Public Stop_Navigation As Boolean
Public Navigate As Boolean
Public Begin_Send_Input As Boolean
Public InBuff_Index As Byte
Dim t As Transmit
Public Start_Index As Integer
Public Stop_Index As Integer
Public stop_tx As Boolean
Public NbrDataToRead As Integer
Public Index_Pointer_1 As Integer
Public Index_Pointer_2 As Integer
Public start_1 As Byte
Public start_2 As Byte

Private Data_Set_Index As Integer

Private Excel_Storage(5000, 23) As Integer

Private Type Transmit
    X(12) As Byte                   'Transmit Data
End Type

Private Type Receive
    Data_Array(75) As Byte          'Received Unprocessed Data
End Type

Private Type Display
    Start As Integer
    Position(3 - 1) As Integer
    Velocity(3 - 1) As Integer
    P(3 - 1) As Integer
    I(3 - 1) As Integer
    D(3 - 1) As Integer
    u(3 - 1) As Integer
    IR(6 - 1) As Byte
    Sonar(6 - 1) As Integer
    Frames_Sent As Integer
    Compass As Integer
    Motor_Current(6 - 1) As Integer
    'Displacement(3 - 1) As Integer
    'Wheel_ref_vel(3 - 1) As Integer
    'X_Y_Theta_ref_vel(3 - 1) As Integer
    Time As Integer
    Bumper_Switch As Byte
    Checksum As Byte
End Type
Private Sub Form_Load()

Index_Pointer_1 = 0
Index_Pointer_2 = 0
Data_Set_Index = 0

With MSComm1
    .CommPort = 9
    .Handshaking = comNone
    .RThreshold = 1
    .RTSEnable = True
    .Settings = "921600,n,8,1"
    .SThreshold = 0
End With

End Sub
Private Sub MSComm1_OnComm()

Dim InBuff() As Byte

Select Case MSComm1.CommEvent
   ' Handle each event or error by placing code below each case statement
    
    'Errors
    Case comEventBreak    ' A Break was received.
        Text5.Text = "Break"
        Stop_Navigation = True
        Text4(0).Text = 0
        Text4(1).Text = 0
        X_g = X_r
        Y_g = Y_r
        Text3(48).Text = 0
        Text3(49).Text = 0
        Send_Input = True
        Navigate = False
    Case comEventFrame    ' Framing Error
    Text5.Text = "Frame Error"
    Case comEventOverrun  ' Data Lost.
    Text5.Text = "Data Lost"
    Case comEventRxOver   ' Receive buffer overflow.
    Text5.Text = "Buffer Overflow"
    Case comEventRxParity ' Parity Error.
    Text5.Text = "Parity Error"
    Case comEventTxFull   ' Transmit buffer full.
    Text5.Text = "Transmit buffer Full"
    Case comEventDCB      ' Unexpected error retrieving DCB]
    Text5.Text = "DCB error"
    
    'Events
    Case comEvCD          ' Change in the CD line.
    Text5.Text = "Change in the CD line"
    Case comEvCTS         ' Change in the CTS line.
    Text5.Text = "Change in the CTS line"
    Case comEvDSR         ' Change in the DSR line.
    Text5.Text = "Change in the DSR line"
    Case comEvRing        ' Change in the Ring Indicator.
    Text5.Text = "Change in the Ring Indicator"
    Case comEvReceive     ' Received RThreshold # of chars.
    
    If (MSComm1.InBufferCount >= 76) Then
        NbrDataToRead = MSComm1.InBufferCount
        InBuff = MSComm1.Input
        Call HandleInput(InBuff())    'Function to handle input
    End If
        
    Case comEvSend        ' There are SThreshold no of characters in transmit buffer.
    Text5.Text = "There are SThreshold no of characters in transmit buffer."
    Case comEvEOF         ' An EOF charater was found in the input stream
    Text5.Text = "An EOF charater was found in the input stream"
   End Select
   
End Sub

Sub HandleInput(InBuff() As Byte)
Dim r As Receive
Dim t As Transmit
Dim FRAME_SIZE As Byte
Dim Data_Array_Index As Byte
Dim InBuff_Index As Integer
Dim xor_temp As Integer
Static Circular_Buffer() As Byte
Dim Circular_Buffer_Size As Integer
Static Counter_Receive As Long
Static Counter_Error As Long
Static mul As Integer
Static Bearing_Conversion As Integer
Dim start_byte_found As Boolean
Dim Input_Buff(3) As Integer
Dim Input_Buff_Index As Byte
Dim xor_send As Byte
Dim While_Loop As Byte
Dim disp As Display
Dim Pos_Info As Byte
Dim Vel_Info As Byte
Dim P_Info As Byte
Dim I_Info As Byte
Dim D_Info As Byte
Dim u_Info As Byte
Dim IR_Info As Byte
Dim Sonar_Info As Byte
Dim Bumper_Info As Byte
Dim Motor_current_Info As Byte
Static X_g As Integer
Static Y_g As Integer
Static X_r As Integer
Static Y_r As Integer
Static X_o(5) As Integer
Static Y_o(5) As Integer
Static Bumper_X_o(8) As Single
Static Bumper_Y_o(8) As Single
Static YdivX As Integer
Static Max_vel As Integer
Static Vel_mag As Long
Static X_vel_new As Long
Static X_vel_old As Long
Static Y_vel_new As Long
Static Y_vel_old As Long
Static xx As Single
Static yy As Single
Static x_neg As Integer
Static y_neg As Integer
Static Obstacle_mag(5) As Single    'No of sensors used in obstacle avoidance
Static Obstacle_Influence As Integer
Static F_rep_x(5) As Single
Static F_rep_y(5) As Single
Static F_att_x As Single
Static F_att_y As Single
Static Scaling_Factor As Long
Static Repulsive_Distances(5) As Integer
Static No_Sensors As Byte
Static IR_weight As Byte
Static Sonar_weight As Byte
Static Bump_angle_index As Byte
Static Bump_Summation_X_o As Single
Static Bump_Summation_Y_o As Single
Static Bump_Linger As Byte
Static Bumped As Boolean
Static Edge_Detect(3) As Byte
Static Edge_Detect_X_o(3) As Single
Static Edge_Detect_Y_o(3) As Single
Static Edge_Detect_index As Byte
Static Theta As Integer
Static sensor_index As Byte
Static edge_found As Boolean
Static Seconds As Integer
Static Minutes As Byte
Static Hours As Byte
Static ms As Long
Static crossing_index As Byte
Static sec_crossing(1) As Byte


IR_weight = 8
Sonar_weight = 8
Scaling_Factor = -800000 '500k
Obstacle_Influence = 50
FRAME_SIZE = 76
start_1 = 68
start_2 = 49
Circular_Buffer_Size = 5000
ReDim Circular_Buffer(0 To (Circular_Buffer_Size - 1)) As Byte
Pos_Info = 3
Vel_Info = 3
P_Info = 3
I_Info = 3
D_Info = 3
u_Info = 3
IR_Info = 6
Sonar_Info = 6
Bumper_Info = 8
Motor_current_Info = 6
Max_vel = 200
No_Sensors = Sonar_Info
'Frame(76Bytes):|Start(2)|Pos(6)|Vel(6)|P(6)|I(6)|D(6)|u(6)|Sonar(12)|IR(6)|Frames_sent(2)|Compass(2)|Current(12)|Displacement(6)|Time(2)|Bumper(1)|Checksum(1)|

Text7(1).Text = NbrDataToRead
While_Loop = NbrDataToRead / FRAME_SIZE

''''''''''''''''''''''''''''''Store RxBuffer to circular buffer''''''''''''''''''''''''''''''''''''
    For InBuff_Index = 0 To (NbrDataToRead - 1)
    Circular_Buffer(Index_Pointer_2) = InBuff(InBuff_Index)
    Index_Pointer_2 = Index_Pointer_2 + 1

        If Index_Pointer_2 = Circular_Buffer_Size Then
        Index_Pointer_2 = 0
        End If

    Next InBuff_Index

Text7(3).Text = Index_Pointer_2
    
''''''''''''''''''''''''''''''Check for start byte in circular buffer''''''''''''''''''''''''''''''''''''
Do While (While_Loop > 0)
While_Loop = While_Loop - 1
Data_Array_Index = 0
xor_temp = 0

   Do While (start_byte_found = False)

        If ((Circular_Buffer(Index_Pointer_1) = start_1) And (Circular_Buffer(Index_Pointer_1 + 1) = start_2)) Then
        start_byte_found = True
        Else
        Index_Pointer_1 = Index_Pointer_1 + 1
        start_byte_found = False

            If (Index_Pointer_1 = Circular_Buffer_Size) Then

                If ((Circular_Buffer(Index_Pointer_1) = start_1) And (Circular_Buffer(0) = start_2)) Then
                start_byte_found = True
                Else
                Index_Pointer_1 = 0
                End If

            End If
            
        End If

   Loop

Text7(2).Text = Index_Pointer_1
Counter_Receive = Counter_Receive + 1
Text1.Text = Counter_Receive

'''''''''''''''''''''''''''''''Obtain defined length frame in circular buffer''''''''''''''''''''''''''''''''''''
   'Actions if there will be a crossover in index limit
    If ((Index_Pointer_1 + FRAME_SIZE) > (Circular_Buffer_Size - 1)) Then
    tmp = Circular_Buffer_Size - Index_Pointer_1
    excess = FRAME_SIZE - tmp

        For InBuff_Index = Index_Pointer_1 To (Circular_Buffer_Size - 1)
        r.Data_Array(Data_Array_Index) = Circular_Buffer(InBuff_Index)
        Data_Array_Index = Data_Array_Index + 1
        Next InBuff_Index

        For InBuff_Index = 0 To excess - 1
        r.Data_Array(Data_Array_Index) = Circular_Buffer(InBuff_Index)
        Data_Array_Index = Data_Array_Index + 1
        Next InBuff_Index

    Else

        For InBuff_Index = Index_Pointer_1 To (Index_Pointer_1 + FRAME_SIZE - 1)
        r.Data_Array(Data_Array_Index) = Circular_Buffer(InBuff_Index)
        Data_Array_Index = Data_Array_Index + 1
        Next InBuff_Index

    End If

    If InBuff_Index = Circular_Buffer_Size Then
    Index_Pointer_1 = 0
    Else
    Index_Pointer_1 = InBuff_Index
    End If

        For Data_Array_Index = 0 To FRAME_SIZE - 2
        xor_temp = xor_temp Xor r.Data_Array(Data_Array_Index)
        Next Data_Array_Index

Text8.Text = xor_temp
Loop
'''''''''''''''''''''''''''Frame already obtained''''''''''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''Processing of Data''''''''''''''''''''''''''''''''''''
'If Checksum passes then proceed else indicate error
If xor_temp = r.Data_Array(FRAME_SIZE - 1) Then

LSet disp = r
Disp_Array_Index = 0

    'Position
    For Data_Array_Index = 0 To Pos_Info - 1
        If (disp.Position(Data_Array_Index) < 0) Then
        Text3(Disp_Array_Index).Text = ((disp.Position(Data_Array_Index)) And 65535)
        Else
        Text3(Disp_Array_Index).Text = disp.Position(Data_Array_Index)
        End If
        Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'Velocity
    For Data_Array_Index = 0 To Vel_Info - 1
    Text3(Disp_Array_Index).Text = (disp.Velocity(Data_Array_Index))
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'P
    For Data_Array_Index = 0 To P_Info - 1
    Text3(Disp_Array_Index).Text = disp.P(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'I
    For Data_Array_Index = 0 To I_Info - 1
    Text3(Disp_Array_Index).Text = disp.I(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'D
    For Data_Array_Index = 0 To D_Info - 1
    Text3(Disp_Array_Index).Text = disp.D(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'u
    For Data_Array_Index = 0 To u_Info - 1
    Text3(Disp_Array_Index).Text = disp.u(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'Sonar
    For Data_Array_Index = 0 To Sonar_Info - 1
    Text3(Disp_Array_Index).Text = disp.Sonar(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'IR
    For Data_Array_Index = 0 To IR_Info - 1
    Text3(Disp_Array_Index).Text = disp.IR(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'Compass
    Text3(Disp_Array_Index).Text = disp.Compass / 10
    Disp_Array_Index = Disp_Array_Index + 1
    'Motor Current
    For Data_Array_Index = 0 To Motor_current_Info - 1
    Text3(Disp_Array_Index).Text = disp.Motor_Current(Data_Array_Index)
    Disp_Array_Index = Disp_Array_Index + 1
    Next Data_Array_Index
    'Bumper
    Text3(Disp_Array_Index).Text = disp.Bumper_Switch And 1
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 2) / 2
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 4) / 4
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 8) / 8
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 16) / 16
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 32) / 32
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 64) / 64
    Disp_Array_Index = Disp_Array_Index + 1
    Text3(Disp_Array_Index).Text = (disp.Bumper_Switch And 128) / 128
    Disp_Array_Index = Disp_Array_Index + 1
    'Displacement
    'For Data_Array_Index = 0 To 2
    'Text3(Disp_Array_Index).Text = disp.Displacement(Data_Array_Index)
    'Disp_Array_Index = Disp_Array_Index + 1
    'Next Data_Array_Index
    'Wheel_ref_vel
   ' Text9(0).Text = disp.Wheel_ref_vel(0) / 9.549
   ' Text9(1).Text = disp.Wheel_ref_vel(1) / 9.549
   ' Text9(2).Text = disp.Wheel_ref_vel(2) / 9.549
    'X_Y_Theta_ref_vel
'    Text9(3).Text = disp.X_Y_Theta_ref_vel(0)
'    Text9(4).Text = disp.X_Y_Theta_ref_vel(1)
'    Text9(5).Text = disp.X_Y_Theta_ref_vel(2)
    'Frames sent by uP
    If (disp.Frames_Sent < 0) Then
    Text10.Text = disp.Frames_Sent And 65535
    Else
    Text10.Text = disp.Frames_Sent
    End If
    
    ''Time
    'Get unsigned value
    If (disp.Time < 0) Then
    ms = disp.Time And 65535
    Else
    ms = disp.Time
    End If
    
    'line below causes crossing from 0 to 1 seconds and back to 0
    sec_crossing(crossing_index) = ms \ 20000
    crossing_index = crossing_index + 1
    If crossing_index = 2 Then
    crossing_index = 0
    End If

    'Identify there has been a crossover and increment seconds
    If sec_crossing(0) <> sec_crossing(1) Then
    Seconds = Seconds + 1
    End If

    If Seconds >= 60 Then
    Seconds = 0
    sec_mul = 0
    Minutes = Minutes + 1
    End If
    If Minutes >= 60 Then
    Minutes = 0
    Hours = Hours + 1
    End If
    Label82(0).Caption = ms
    'Text11(0).Text = ms
    Label82(1).Caption = Seconds
    'Text11(1).Text = Seconds
    Label82(2).Caption = Minutes
    'Text11(2).Text = Minutes
    Label82(3).Caption = Hours
'    Text11(3).Text = Hours
'''''''''''''''''''''''''''''End of Processing of Data''''''''''''''''''''''''''''''''''''
 '--------------------Navigation Begin--------------------------
    If Stop_Navigation = False Then

        If Navigate = True Then
        Navigate = False
        'Goal Location
        X_g = Text3(48).Text
        Y_g = Text3(49).Text
        End If

    'Current Location
    X_r = Text3(45).Text
    Y_r = Text3(46).Text

    'Store Old Vel command
    X_vel_old = X_vel_new
    Y_vel_old = Y_vel_new

    'Generate New Vel Command
    F_att_x = X_g - X_r
    F_att_y = Y_g - Y_r

    X_vel_new = X_g
    Y_vel_new = Y_g
    

    'Change compass bearing to kinematics convention
    disp.Compass = 3600 - disp.Compass
        If disp.Compass >= 1800 Then
        disp.Compass = disp.Compass - 3600
        End If

'        '------repulsive force-------------------
'        '--------------------------------1.Edge Detection-----------------------------------
'        Edge_Detect(0) = 0
'        Edge_Detect(1) = 0
'        Edge_Detect(2) = 0
'
'        If ((disp.IR(0) > 35) Or (disp.IR(2) > 35) Or (disp.IR(4) > 35)) Then
'        edge_found = True
'        Edge_Detect_index = 0
'        For sensor_index = 0 To 4
'        Theta = 180 - 60 * sensor_index
'
'            If (disp.IR(sensor_index)) > 35 Then
'            Edge_Detect(Edge_Detect_index) = 1
'            Else
'            Edge_Detect(Edge_Detect_index) = 0
'            End If
'
'        Edge_Detect_X_o(Edge_Detect_index) = -20 * Edge_Detect(Edge_Detect_index) * Cos((Theta + disp.Compass / 10) * 0.01745)
'        Edge_Detect_Y_o(Edge_Detect_index) = -20 * Edge_Detect(Edge_Detect_index) * Sin((Theta + disp.Compass / 10) * 0.01745)
'        Edge_Detect_index = Edge_Detect_index + 1
'        sensor_index = sensor_index + 1
'        Next sensor_index
'
'        ii = 0
'        For Edge_Detect_index = 0 To 2
'        Text9(ii).Text = Edge_Detect_X_o(Edge_Detect_index)
'        ii = ii + 1
'        Text9(ii).Text = Edge_Detect_Y_o(Edge_Detect_index)
'        ii = ii + 1
'        Next Edge_Detect_index
'
'        For Edge_Detect_index = 0 To 2
'        Edge_Detect_X_o(0) = Edge_Detect_X_o(0) + Edge_Detect_X_o(Edge_Detect_index)
'        Edge_Detect_Y_o(0) = Edge_Detect_Y_o(0) + Edge_Detect_Y_o(Edge_Detect_index)
'        Next Edge_Detect_index
'
'        '---------------------2. Collision--------------------------------------
'        ElseIf disp.Bumper_Switch <> 0 Then
'            Bump_angle_index = 0
'                For sensor_index = 37 To 37 + Bumper_Info - 1
'                Theta = 45 * Bump_angle_index
'                    If Theta <= 180 Then
'                    Theta = -Theta
'                    Else
'                    Theta = 360 - Theta
'                    End If
'                Bumper_X_o(Bump_angle_index) = -1 * (Text3(sensor_index).Text) * Cos((Theta + disp.Compass / 10) * 0.01745)
'                Bumper_Y_o(Bump_angle_index) = -1 * (Text3(sensor_index).Text) * Sin((Theta + disp.Compass / 10) * 0.01745)
'                Bump_angle_index = Bump_angle_index + 1
'                Next sensor_index
'
'                'Bump Summation
'                Bump_angle_index = 0
'                Bump_Summation_X_o = 0
'                Bump_Summation_Y_o = 0
'                For Bump_angle_index = 0 To Bumper_Info - 1
'                Bump_Summation_X_o = Bump_Summation_X_o + Bumper_X_o(Bump_angle_index)
'                Bump_Summation_Y_o = Bump_Summation_Y_o + Bumper_Y_o(Bump_angle_index)
'                Next Bump_angle_index
'
'                Bumped = True
'                Bump_Linger = 25
'                Bump_Summation_X_o = 20 * Bump_Summation_X_o
'                Bump_Summation_Y_o = 20 * Bump_Summation_Y_o
'
'            Else
'
'            '3. Obstacles Avoidance
'            'Insert obstacle distances and identify each one.
'            Obstacle_mag(0) = disp.Sonar(1)
'            Obstacle_mag(3) = ((Sonar_weight / 10) * disp.Sonar(0) + (IR_weight / 10) * disp.IR(1)) / 2
'            Obstacle_mag(1) = ((Sonar_weight / 10) * disp.Sonar(3) + (IR_weight / 10) * disp.IR(3)) / 2
'            Obstacle_mag(2) = disp.Sonar(2)
'            Obstacle_mag(4) = disp.Sonar(4)
'            Obstacle_mag(5) = ((Sonar_weight / 10) * disp.Sonar(5) + (IR_weight / 10) * disp.IR(5)) / 2
'
'            'Compute its X_o and Y_o(array) for sonar and IR
'            For sensor_index = 0 To No_Sensors - 1
'            X_o(sensor_index) = (Cos((60 * sensor_index + disp.Compass / 10) * 0.01745)) * Obstacle_mag(sensor_index)
'            Y_o(sensor_index) = (Sin((60 * sensor_index + disp.Compass / 10) * 0.01745)) * Obstacle_mag(sensor_index)
'            Next sensor_index
'
'            'Individual Obstacle repulsion computation
'            For sensor_index = 0 To No_Sensors - 1
'                If Obstacle_mag(sensor_index) <> 0 Then
'                    If Obstacle_mag(sensor_index) > Obstacle_Influence Then
'                    F_rep_x(sensor_index) = 0
'                    F_rep_y(sensor_index) = 0
'                    Else
'                    F_rep_x(sensor_index) = Scaling_Factor * ((1 / Obstacle_mag(sensor_index)) - (1 / Obstacle_Influence)) * X_o(sensor_index) / ((Obstacle_mag(sensor_index)) * (Obstacle_mag(sensor_index)) * (Obstacle_mag(sensor_index)))
'                    F_rep_y(sensor_index) = Scaling_Factor * ((1 / Obstacle_mag(sensor_index)) - (1 / Obstacle_Influence)) * Y_o(sensor_index) / ((Obstacle_mag(sensor_index)) * (Obstacle_mag(sensor_index)) * (Obstacle_mag(sensor_index)))
'                    End If
'                End If
'            Next sensor_index
'
'            'Summation of repulsive force vectors into velocity command
'            For sensor_index = 0 To No_Sensors - 1
'            F_rep_x(0) = F_rep_x(0) + F_rep_x(sensor_index)
'            F_rep_y(0) = F_rep_y(0) + F_rep_y(sensor_index)
'            Next sensor_index
'            End If
'            '------end of repulsive force computation-------------------
            '-------------------------Begin of Action decision---------------------------
'                If edge_found = True Then
'                edge_found = False
'                X_vel_new = Edge_Detect_X_o(0)
'                Y_vel_new = Edge_Detect_Y_o(0)
'                ElseIf Bumped = True Then
'                Bump_Linger = Bump_Linger - 1
'                    If Bump_Linger = 0 Then
'                    Bumped = False
'                    End If
'                X_vel_new = Bump_Summation_X_o
'                Y_vel_new = Bump_Summation_Y_o
'                Else
'                X_vel_new = F_att_x + F_rep_x(0)
'                Y_vel_new = F_att_y + F_rep_y(0)
'                End If
                'To ignore Repulsive forces when goal is reached
        '        If (F_att_x <= 1) And (F_att_x >= -1) And (F_att_y >= -1) And (F_att_y <= 1) Then
'                If (F_att_x = 0) And (F_att_y = 0) Then
'                X_vel_new = 0
'                Y_vel_new = 0
'                End If
            '-----------------------Begin of Action decision-----------------------
'           ----------------------------One off circle motion------------------------


'dtmTest = TimeValue(Now)
''Text9(0).Text = dtmTest
''Text9(1).Text = Sgn(-2)
'
'If X_r = -30 Then
'    If Y_r > 0 Then
'    X_vel_new = -20
'    Else
'    X_vel_new = 20
'    End If
'Else
'Theta = Atn(Y_r / (X_r + 30))
'xxgg = Sgn(X_r) * (Sqr(30 * 30 / (1 + Tan(Theta) * Tan(Theta))))
'yygg = Tan(Theta) * xxgg
'
''X_vel_new = -20 * Sgn(X_r + 30) * Sin(Theta)
''Y_vel_new = 20 * Sgn(X_r + 30) * Cos(Theta)
'X_vel_new = -20 * Sgn(X_r + 30) * Sin(Theta) + 0.1 * (xxgg - X_r)
'Y_vel_new = 20 * Sgn(X_r + 30) * Cos(Theta) + 0.1 * (yygg - Y_r)
'End If
'           ----------------------------One off circle motion------------------------
            '------------Ensure limits in place--------------------------
                'Identify if there is a X_negative value
                If X_vel_new < 0 Then
                x_neg = -1
                Else
                x_neg = 1
                End If
                'x_neg = Sgn(X_vel_new)
                
                'Because of overflow in Vel_mag, a computational square limit is enforced
                'If (X_vel_new > Max_vel) Then
                'X_vel_new = Max_vel
                'ElseIf (X_vel_new < -Max_vel) Then
                'X_vel_new = -Max_vel
                'End If
                'If (Y_vel_new > Max_vel) Then
                
                'Y_vel_new = Max_vel
                'ElseIf (Y_vel_new < -Max_vel) Then
                'Y_vel_new = -Max_vel
                'End If
              
                'Obtain the velocity command's magnitude
                Vel_mag = Sqr(Y_vel_new * Y_vel_new + X_vel_new * X_vel_new)
                
                'Update if there is a difference between the old values
                'If (((X_vel_new <> X_vel_old) Or (Y_vel_new <> Y_vel_old))) And ((F_att_x <> 0) Or (F_att_y <> 0)) Then
                If (((X_vel_new <> X_vel_old) Or (Y_vel_new <> Y_vel_old))) Then
                Send_Input = True
                    If (X_vel_new = 0) Then
                    Text4(0).Text = X_vel_new
                        If Y_vel_new > Max_vel Then
                        Text4(1).Text = Max_vel
                        Else
                            If Y_vel_new < -Max_vel Then
                            Text4(1).Text = -Max_vel
                            Else
                            Text4(1).Text = Y_vel_new
                            End If
                        End If
                    Else
                        If Vel_mag > Max_vel Then
                        xx = x_neg * (Sqr(Max_vel * Max_vel / (1 + (Y_vel_new / X_vel_new) * (Y_vel_new / X_vel_new))))
                        yy = (Y_vel_new / X_vel_new) * xx
                        Text4(0).Text = xx
                        Text4(1).Text = yy
                        Else
                        Text4(0).Text = X_vel_new
                        Text4(1).Text = Y_vel_new
                        End If
                    End If
                End If
        
        Else    'stop navigation
        End If

'------------------------Tx to uP---------------------------------------------------------
                If stop_tx = False Then

                    If Send_Input = False Then
                    MSComm1.Output = "1"

                    ElseIf Send_Input = True Then
                    Send_Input = False
                        
                        '|Start_0|Start_1|CMD|X|X|Y|Y|theta|theta|checksum|stop_0|stop_1|
                        'Store input into buffer & Data(X,Y,theta) Dissected into bytes for Tx
                        For Input_Buff_Index = 0 To 2
                            If Input_Buff_Index = 2 Then
                               ' If Val(Text4(Input_Buff_Index).Text) > 180 Then
                               ' Bearing_Conversion = 360 - Val(Text4(Input_Buff_Index).Text)
                               ' Else
                                Bearing_Conversion = Val(Text4(Input_Buff_Index).Text)
                                'End If
                            t.X(Input_Buff_Index * 2) = (Bearing_Conversion And &HFF00&) / 256
                            t.X(Input_Buff_Index * 2 + 1) = (Bearing_Conversion) And &HFF&
                            Else
                            t.X(Input_Buff_Index * 2) = (Text4(Input_Buff_Index).Text And &HFF00&) / 256
                            t.X(Input_Buff_Index * 2 + 1) = (Text4(Input_Buff_Index).Text) And &HFF&
                            End If
                        Next Input_Buff_Index
    
                        'XOR Checksum
                        xor_send = 0
                        For Input_Buff_Index = 0 To 5
                        xor_send = xor_send Xor t.X(Input_Buff_Index)
                        Next Input_Buff_Index

                    t.X(6) = xor_send           'Store checksum
                    MSComm1.Output = Chr(50)    'Signal to uP Tx coming

                        'Transmit
                        For Input_Buff_Index = 0 To 6
                        MSComm1.Output = Chr(t.X(Input_Buff_Index))
                        Next Input_Buff_Index

                        Text6.Text = "Sent!"

                    End If
                Else
                'dont send anything to uP so it will stop sending data
                End If

        Text2.Text = "Good!"

'Checksum fail...Data Integrity lost~!
Else
Text2.Text = "Bad Checksum!"
MSComm1.Output = "1"
Counter_Error = Counter_Error + 1
Text7(0).Text = Counter_Error
End If
'Loop
'----------------------------------------Excel Temp Storage---------------------------
If Data_Set_Index <= 5000 Then
'Text11(0).Text = Data_Set_Index
'Excel_Storage(Data_Set_Index, 0) = Hours
'Excel_Storage(Data_Set_Index, 1) = Minutes
'Excel_Storage(Data_Set_Index, 2) = Seconds
'Excel_Storage(Data_Set_Index, 3) = ms
'Excel_Storage(Data_Set_Index, 4) = X_g 'Text3(48).Text ' X_g
Excel_Storage(Data_Set_Index, 5) = Text3(45).Text
'Excel_Storage(Data_Set_Index, 6) = Y_g 'Text3(49).Text  'Y_g
Excel_Storage(Data_Set_Index, 7) = Text3(46).Text
'Excel_Storage(Data_Set_Index, 8) = Text4(2).Text   'Theta_g
'Excel_Storage(Data_Set_Index, 9) = Text3(47).Text  'Theta_r
Excel_Storage(Data_Set_Index, 10) = Text4(0).Text
Excel_Storage(Data_Set_Index, 11) = Text4(1).Text
Excel_Storage(Data_Set_Index, 12) = Y_g ' Theta_ref_vel
'Excel_Storage(Data_Set_Index, 13) = disp.Wheel_ref_vel(0) / 9.549
'Excel_Storage(Data_Set_Index, 14) = disp.Velocity(0)
'Excel_Storage(Data_Set_Index, 15) = disp.Wheel_ref_vel(1) / 9.549
'Excel_Storage(Data_Set_Index, 16) = disp.Velocity(1)
'Excel_Storage(Data_Set_Index, 17) = disp.Wheel_ref_vel(2) / 9.549
'Excel_Storage(Data_Set_Index, 18) = disp.Velocity(2)
Excel_Storage(Data_Set_Index, 19) = 0
Excel_Storage(Data_Set_Index, 20) = 0
Excel_Storage(Data_Set_Index, 21) = 0
Excel_Storage(Data_Set_Index, 22) = disp.Compass

Data_Set_Index = Data_Set_Index + 1

End If

'----------------------------------------End of Excel Temp Storage---------------------------
End Sub
Public Sub Command3_Click()
'Write to file
stop_tx = True
Dim appxl As Excel.Application      'Declare object variables
Dim wb As Excel.Workbook
Dim ws As Excel.Worksheet
Dim my_variable As String

Set appxl = New Excel.Application   'Create a new instance of Excel
Set wb = appxl.Workbooks.Add        'Add a new workbook
Set ws = wb.Worksheets(1)           'Work with the first worksheet
appxl.Visible = True                'Show it to the user

ws.Cells(1, 1).Value = "Hours"
ws.Cells(1, 2).Value = "Minutes"
ws.Cells(1, 3).Value = "Seconds"
ws.Cells(1, 4).Value = "ms"
ws.Cells(1, 5).Value = "X_g"
ws.Cells(1, 6).Value = "X_r"
ws.Cells(1, 7).Value = "Y_g"
ws.Cells(1, 8).Value = "Y_r"
ws.Cells(1, 9).Value = "Theta_g"
ws.Cells(1, 10).Value = "Theta_r"
ws.Cells(1, 11).Value = "X_vel_ref"
ws.Cells(1, 12).Value = "Y_vel_ref"
ws.Cells(1, 13).Value = "Theta_vel_ref"
ws.Cells(1, 14).Value = "wheel1_vel_ref"
ws.Cells(1, 15).Value = "wheel1_act_ref"
ws.Cells(1, 16).Value = "wheel2_vel_ref"
ws.Cells(1, 17).Value = "wheel2_act_ref"
ws.Cells(1, 18).Value = "wheel3_vel_ref"
ws.Cells(1, 19).Value = "wheel3_act_ref"
ws.Cells(1, 20).Value = "X_vel_act"
ws.Cells(1, 21).Value = "Y_vel_act"
ws.Cells(1, 22).Value = "X_vel_act"
ws.Cells(1, 23).Value = "Compass"

For Row = 2 To 1000
    For Column = 1 To 23
    ws.Cells(Row, Column).Value = Excel_Storage(Row - 2, Column - 1)
    Next Column
Next Row

FileName = "C:\Documents and Settings\g0900600\Desktop\ran.xls"        'Add filename
ws.SaveAs (FileName)                'Save it
End Sub

Private Sub Command1_Click()
Stop_Navigation = True
Text4(0).Text = 0
Text4(1).Text = 0
X_g = X_r
Y_g = Y_r
Text3(48).Text = X_r
Text3(49).Text = Y_r
'Text4(2).Text = Bearing_Conversion
Send_Input = True
Navigate = False
End Sub
Private Sub Stop_Click()
stop_tx = True
End Sub
Private Sub Receive_Click()
stop_tx = False
MSComm1.Output = "1"
End Sub
Private Sub Connect_Click()
    If MSComm1.PortOpen = False Then
    MSComm1.PortOpen = True
    Connect.Caption = "Disconnect"
    ElseIf MSComm1.PortOpen = True Then
    MSComm1.PortOpen = False
    Connect.Caption = "Connect"
    End If
End Sub
Private Sub Command2_Click()
    If Navigate = False Then
    Navigate = True
    Send_Input = True
    Else
    Navigate = False
    Send_Input = True
    End If
    Stop_Navigation = False
End Sub

*/