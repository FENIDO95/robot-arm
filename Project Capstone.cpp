/****************************************************************************************
Course: ROBT1270

Program: Lab3: SCARA Robot Simulator Intermediate Control

Purpose: 

Author(s): Fernando Nieto Donaire
<A00882787>
<set A>

Declaration: I(We), <Fernando Nieto>,
declare that the following program was written by me(us) starting with the base code from the course.

Date Created: <3/13/15>

****************************************************************************************/

#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

//-------------------------- Standard library prototypes --------------------------------
#include <stdio.h>  // formatted i/o
#include <math.h>   // math functions
#include <stdlib.h> // system function
#include "robot.h"  // robot functions

CRobot robot;       // the global robot Class.  Can be used everywhere

//---------------------------- Program Constants ----------------------------------------
#define PI                    3.14159265358979323846

#define NUM_LINES             8           // number of lines for the main loop
#define MAX_POINTS            1550          // maximum number of points in a line
#define MAX_STRING            256         // for the commandString array size
#define LEFT_ARM_SOLUTION     0           // index that can be used to indicate left arm
#define RIGHT_ARM_SOLUTION    1           // index that can be used to indicate right arm
#define L1                    350.0       // inner arm length
#define L2                    250.0       // outer arm length
#define MAX_ABS_THETA1_DEG    150.0       // max angle of inner arm
#define MAX_ABS_THETA2_DEG    170.0       // max angle of outer arm relative to inner arm
#define SLOPE_TOL             (1.0e-5)    // for pen color based on slope
#define POINT_TOL             (1.0e-8)    // use to check if previous line end point is
                                          // the same as current line start point

//----------------------------- Program Structures --------------------------------------

// encapsulates both x and y for forward solution 
typedef struct FORWARD_SOLUTION
{
   double x, y; 
}
FORWARD_SOLUTION;
typedef struct PAST
{
   double x[MAX_POINTS], y[MAX_POINTS];
}PAST;
// encapsulates both right/left arm angles for inverse solution
typedef struct INVERSE_SOLUTION
{
   double theta1DegLeft, theta1DegRight; // right arm solution - angles in degrees!!!!!!!
   double theta2DegLeft, theta2DegRight; //  left arm solution - angles in degrees!!!!!!!
}
INVERSE_SOLUTION;
typedef struct RGB_COLOR // for describing pen RGB color
{
   int r, g, b;   // red, green, blue components (0-255 each)
}
RGB_COLOR;
// info needed to draw a line
typedef struct LINE_DATA
{
   double xA, yA, xB, yB;  // start point, end point
   int N; // number of points on line (includes endpoints)
   RGB_COLOR color; // the line color red, green, blue components
}
LINE_DATA;
typedef struct BOARD
{
   int spot[9];
}
BOARD;
typedef struct PLAYERS
{
   int numhum;
}
PLAYERS;
//----------------------------- Local Function Prototypes -------------------------------
double DegToRad(double);  // returns angle in radians from input angle in degrees
double RadToDeg(double);  // returns angle in degrees from input angle in radians
FORWARD_SOLUTION ForwardKinematics(double, double);  // solves forward kinematics
void RandCirc(void);//challange task
int wincheck(BOARD);//call after each move
void tictack(void);//play the game

PLAYERS gamesetup(void);
void defineCirc(void);
void SpecCirc(double, double, double, double);//challange task partial
void SpecX(double, double, double);

int HumTurn(int, int, BOARD);
int ComTurn(int, int, BOARD);
void Erase(BOARD);
INVERSE_SOLUTION InverseKinematics(double, double);  
RGB_COLOR ColorFind(LINE_DATA);//finds what color the line should be

void RS(LINE_DATA);

void strieghtline(void);
int ComThink(int, BOARD);
void Drawline(LINE_DATA);
int main()
{
   // open connection with robot
   if(!robot.Initialize()) return 0;
   
   //============================ START OF YOUR CODE =============================
   
   robot.Send("CYCLE_PEN_COLORS OFF\n");
   
  
      printf("Fernando Nieto Donaire A00882787");
	  Initialbox();
	  tictack();

	
   
   

   //============================= END OF YOUR CODE ==============================

   robot.Send("PEN_UP\n");
   robot.Send("MOTOR_SPEED HIGH\n");
   robot.Send("ROTATE_JOINT ANG1 0.0 ANG2 90.0\n");
   robot.Close(); // close remote connection
   getchar();
   return (0);
}

void Initialbox(void)
{
	int i;
	for (i = 0; i < 5; i++);
	{
		printf("\n\n                         %c          %c    \n", 179, 179);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                   %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c   \n", 196, 196, 196, 196, 196, 196, 197, 196, 196, 196, 196, 196, 196, 196, 196, 196, 196, 197, 196, 196, 196, 196, 196, 196, 196);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                   %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c   \n", 196, 196, 196, 196, 196, 196, 197, 196, 196, 196, 196, 196, 196, 196, 196, 196, 196, 197, 196, 196, 196, 196, 196, 196, 196);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                         %c          %c    \n", 179, 179, 179);
		printf("                         %c          %c    \n", 179, 179, 179);
	}
}
FORWARD_SOLUTION ForwardKinematics(double theta1Deg, double theta2Deg)
{
   FORWARD_SOLUTION fsol;

   double theta1=DegToRad(theta1Deg), theta2=DegToRad(theta2Deg);
   fsol.x=L1*cos(theta1)+L2*cos(theta1+theta2);
   fsol.y=L1*sin(theta1)+L2*sin(theta1+theta2);

   return fsol;
}
int winnerplayer(BOARD b)
{
	int p;
	p = 0;
	if (b.spot[0] * b.spot[1] * b.spot[2] == 8) 
	{ 
		printf("x wins\n"); 
		p = 1; 
	}
	if (b.spot[3] * b.spot[4] * b.spot[5] == 8){ printf("x wins\n"); p = 1; }
	if (b.spot[6] * b.spot[7] * b.spot[8] == 8){ printf("x wins\n"); p = 1; }
	if (b.spot[6] * b.spot[3] * b.spot[0] == 8){ printf("x wins\n"); p = 1; }
	if (b.spot[7] * b.spot[4] * b.spot[1] == 8){ printf("x wins\n"); p = 1; }
	if (b.spot[8] * b.spot[5] * b.spot[2] == 8){ printf("x wins\n"); p = 1; }
	if (b.spot[8] * b.spot[4] * b.spot[0] == 8) { printf("x wins\n"); p = 1; }
	if (b.spot[6] * b.spot[4] * b.spot[2] == 8) { printf("x wins\n"); p = 1; }



	if (b.spot[0] * b.spot[1] * b.spot[2] == 27) { printf("o wins\n"); p = 1; }
	if (b.spot[3] * b.spot[4] * b.spot[5] == 27) { printf("o wins\n"); p = 1; }
	if (b.spot[6] * b.spot[7] * b.spot[8] == 27) { printf("o wins\n"); p = 1; }
	if (b.spot[6] * b.spot[3] * b.spot[0] == 27){ printf("o wins\n"); p = 1; }
	if (b.spot[7] * b.spot[4] * b.spot[1] == 27) { printf("o wins\n"); p = 1; }
	if (b.spot[8] * b.spot[5] * b.spot[2] == 27) { printf("o wins\n"); p = 1; }
	if (b.spot[8] * b.spot[4] * b.spot[0] == 27) { printf("o wins\n"); p = 1; }
	if (b.spot[6] * b.spot[4] * b.spot[2] == 27) { printf("o wins\n"); p = 1; }
	return p;

	


}
void tictack(void)
{
   int x = 0, n, temp;
   double xa, ya, r;
   BOARD b;
   for (n = 0; n < 9; n++)//clear the board
   {
	   b.spot[n] = 0;
   }
   
   
   PLAYERS plrs;
   
   plrs = gamesetup();
   
   if (plrs.numhum == 2)
   {
	   for (n = 0; n < 9; n++)
      {
         x = n % 2;
        temp = (HumTurn(x, 0, b)-1);
		if (x == 0)
		{
			robot.Send("PEN_COLOR 0 0 255\n");
			b.spot[temp] = 3;
		}
		if (x == 1)
		{
			robot.Send("PEN_COLOR 0 255 0\n");
			b.spot[temp] = 2;
		}
		temp = wincheck(b);
		if (temp == 1) break;
      }
	 
	
   }
   if (plrs.numhum == 1)
   {
	   for (n = 0; n < 9; n++)
	   {
		   x = n % 2;
		   if (x == 0)
		   {
			   robot.Send("PEN_COLOR 0 0 255\n");
			   temp = (HumTurn(x, 0, b) - 1);

		   }
		   if (x == 1)
		   {
			   robot.Send("PEN_COLOR 0 255 0\n");
			   temp = (ComTurn(x, 0, b) - 1);
		   }
		   if (x == 0) b.spot[temp] = 3;
		   if (x == 1) b.spot[temp] = 2;
		   temp = winnerplayer(b);
		   if (temp == 1) break;
	   }

   }
   Erase(b);
}
void Erase(BOARD b)
{
	robot.Send("PEN_COLOR 200 200 200\n");
	int n, m, spot=10;
	double xa=165, ya=372, r = 40, tempx,tempy;
	for (n = 0; n < 3; n++)
	{
		tempx = xa + 100 * n;
		for (m = 0; m < 3; m++)
		{
			tempy = ya + 100 * m;
			SpecCirc(tempx, tempy, r, 15);
			SpecX(tempx, tempy, r);

		}
	}
	
	
}
PLAYERS gamesetup()
{
   PLAYERS plrs;
   printf("\n\n\n Please choose the game mode \n");
   printf("\n(computer vs computer) = 0\n");
   printf("(computer vs player) = 1\n");
   printf("(player vs player) = 2\n");
   scanf("%d", &players.numhum);
   return(plrs);
     
}
int ComThink(int turn, BOARD b)
{
	int spot =10, garbage[9], n, cond, row[8];
	if (turn == 0)
	{

	for (n = 0; n < 9; n++)
	{
	if (garbage[n] < .5) spot++;
	}
	}
	if (turn == 1)
	{
		if (b.spot[0] + b.spot[2] + b.spot[6] + b.spot[8] > 0) cond = 1;//player picked a corner
		if (b.spot[1] + b.spot[3] + b.spot[5] + b.spot[7] > 0) cond = 2;//player picked an edge
		if (b.spot[4] > 0) cond = 3; //player picked the middle
		if (cond == 3) spot = 9;
		if (cond == 2) spot = 5;
		if (cond == 1)
		{
			if (b.spot[0] + b.spot[6] > 0) spot = 4;
			if (b.spot[2] + b.spot[8] > 0) spot = 6;
		}
	}
	if (turn > 2)
	{
		cond = 0;
		row[0] = b.spot[0] + b.spot[1] + b.spot[2];
		row[1] = b.spot[3] + b.spot[4] + b.spot[5];
		row[2] = b.spot[6] + b.spot[7] + b.spot[8];
		row[3] = b.spot[6] + b.spot[3] + b.spot[0];
		row[4] = b.spot[7] + b.spot[4] + b.spot[1];
		row[5] = b.spot[8] + b.spot[5] + b.spot[2];
		row[6] = b.spot[8] + b.spot[4] + b.spot[0];
		row[7] = b.spot[6] + b.spot[4] + b.spot[2];
		for (n = 0; n < 8; n++)
		{
			if (row[n] == 4)
			{
				cond = 1;
			}
		}
		if (cond == 1)
		{
			if (row[0] == 4)
			{
				if (b.spot[0] == 0) spot = 1;
				if (b.spot[1] == 0) spot = 2;
				if (b.spot[2] == 0) spot = 3;
			}
			if (row[1] == 4)
			{
				if (b.spot[3] == 0) spot = 4;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[5] == 0) spot = 6;
			}
			if (row[2] == 4)
			{
				if (b.spot[6] == 0) spot = 7;
				if (b.spot[7] == 0) spot = 8;
				if (b.spot[8] == 0) spot = 9;
			}
			if (row[3] == 4)
			{
				if (b.spot[0] == 0) spot = 7;
				if (b.spot[3] == 0) spot = 4;
				if (b.spot[6] == 0) spot = 1;
			}
			if (row[4] == 4)
			{
				if (b.spot[1] == 0) spot = 8;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[7] == 0) spot = 2;
			}
			if (row[5] == 4)
			{
				if (b.spot[2] == 0) spot = 9;
				if (b.spot[5] == 0) spot = 6;
				if (b.spot[8] == 0) spot = 3;
			}
			if (row[6] == 4)
			{
				if (b.spot[0] == 0) spot = 9;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[8] == 0) spot = 1;
			}
			if (row[7] == 4)
			{
				if (b.spot[2] == 0) spot = 7;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[6] == 0) spot = 3;
			}
		}
		if (cond != 1)
		{
			if (row[0] == 6)
			{
				if (b.spot[0] == 0) spot = 1;
				if (b.spot[1] == 0) spot = 2;
				if (b.spot[2] == 0) spot = 3;
			}
			if (row[1] == 6)
			{
				if (b.spot[3] == 0) spot = 4;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[5] == 0) spot = 6;
			}
			if (row[2] == 6)
			{
				if (b.spot[6] == 0) spot = 7;
				if (b.spot[7] == 0) spot = 8;
				if (b.spot[8] == 0) spot = 9;
			}
			if (row[3] == 6)
			{
				if (b.spot[0] == 0) spot = 7;
				if (b.spot[3] == 0) spot = 4;
				if (b.spot[6] == 0) spot = 1;
			}
			if (row[4] == 6)
			{
				if (b.spot[1] == 0) spot = 8;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[7] == 0) spot = 2;
			}
			if (row[5] == 6)
			{
				if (b.spot[2] == 0) spot = 9;
				if (b.spot[5] == 0) spot = 6;
				if (b.spot[8] == 0) spot = 3;
			}
			if (row[6] == 6)
			{
				if (b.spot[0] == 0) spot = 9;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[8] == 0) spot = 1;
			}
			if (row[7] == 6)
			{
				if (b.spot[2] == 0) spot = 7;
				if (b.spot[4] == 0) spot = 5;
				if (b.spot[6] == 0) spot = 3;
			}
		}

	}






	if (spot == 10) //if there is no obvios move put an x in a random spot
	{

		for (n = 0; n < 9; n++)
		{
			if (b.spot[n] == 0)
			{
				spot = n + 1;
				if (n == 4) break;
			}
		}
	}



	return spot;
}
int ComTurn(int x, int y, BOARD b)
{
	double xa, ya, r;
	int spot, turn=0, n;
	r = 40;
	for (n = 0; n < 9; n++)
	{
		if (b.spot[n] != 0) turn++;
	}
	spot = ComThink(turn, b);
	
	
	if (spot == 1)
	{
		xa = 165;
		ya = 372;
	}
	if (spot == 2)
	{
		xa = 269;
		ya = 372;
	}
	if (spot == 3)
	{
		xa = 372;
		ya = 372;
	}
	if (spot == 4)
	{
		xa = 165;
		ya = 269;
	}
	if (spot == 5)
	{
		xa = 269;
		ya = 269;
	}
	if (spot == 6)
	{
		xa = 372;
		ya = 269;
	}
	if (spot == 7)
	{
		xa = 165;
		ya = 165;
	}
	if (spot == 8)
	{
		xa = 269;
		ya = 165;
	}
	if (spot == 9)
	{
		xa = 372;
		ya = 165;
	}

	
	SpecX(xa, ya, r);
	return(spot);

}
int HumTurn(int x, int y, BOARD b)
{
   double xa, ya, r;
   int spot;
   r = 40;
   do{
	   printf("Please Input the square number as indicated above (1-9)");
	   scanf("%d", &spot);
   } while (b.spot[(spot-1)] != 0);//loop to not let you over write an x or o
   if (spot == 1)
   {
	   xa = 165;
	   ya = 372;
   }
   if (spot == 2)
   {
	   xa = 269;
	   ya = 372;
   }
   if (spot == 3)
   {
	   xa = 372;
	   ya = 372;
   }
   if (spot == 4)
   {
	   xa = 165;
	   ya = 269;
   }
   if (spot == 5)
   {
	   xa = 269;
	   ya = 269;
   }
   if (spot == 6)
   {
	   xa = 372;
	   ya = 269;
   }
   if (spot == 7)
   {
	   xa = 165;
	   ya = 165;
   }
   if (spot == 8)
   {
	   xa = 269;
	   ya = 165;
   }
   if (spot == 9)
   {
	   xa = 372;
	   ya = 165;
   }
   
      
      if (x == 0) SpecCirc(xa, ya, r, 15);
      if (x != 0)   SpecX(xa, ya, r);

	  return(spot);

}
void SpecX(double xa, double ya, double r)
{
   LINE_DATA user;
   user.color = { 0, 255, 0 };
   user.xA = xa - (0.5*r);
   user.xB = xa + (0.5*r);
   user.yA = ya - (0.5*r);
   user.yB = ya + (0.5*r);
   user.N = 5;
   Drawline(user);
   user.xB = xa + (0.5*r);
   user.xA = xa - (0.5*r);
   user.yA = ya + (0.5*r);
   user.yB = ya - (0.5*r);
   user.N = 5;
   Drawline(user);

}
void RS(LINE_DATA user)//function to set motor speed base on number of points
{
   int n;
   n = user.N;
   if (n% 2 == 0) robot.Send("MOTOR_SPEED HIGH\n");
   if (n % 2 != 0) robot.Send("MOTOR_SPEED MEDIUM\n");
   if (n >10) robot.Send("MOTOR_SPEED LOW\n");
}
void RandCirc()
{
	int n;
	robot.Send("PEN_COLOR 255 0 0\n");
	SpecCirc(320, 220, 150, 15);
	robot.Send("PEN_COLOR 255 0 255\n");
	SpecCirc(320, 340, 30, 15);
	SpecCirc(320, 100, 30, 15);
	SpecCirc(200, 220, 30, 15);
	SpecCirc(440, 220, 30, 15);
	robot.Send("PEN_COLOR 0 0 255\n");
	SpecCirc(320 + 85, 220 + 85, 30, 15);
	SpecCirc(320 + 85, 220 - 85, 30, 15);
	SpecCirc(320 - 85, 220 + 85, 30, 15);
	SpecCirc(320 - 85, 220 - 85, 30, 15);
	robot.Send("PEN_COLOR 255 0 0\n");
	

	

}
void SpecCirc(double xa, double ya, double R, double t)
{
	int n;
	INVERSE_SOLUTION isol;
	char out[150];//char string for sending positions to robot
	double p, tempx, tempy;
	for (n = 0; n < t; n++)
	{
		if (n == 0){ robot.Send("PEN_UP\n"); }//lift pen to go to first point
		if (n == 1){ robot.Send("PEN_DOWN\n"); }//put pen down after that
		p = double(n) / (t - 1);
		tempx = xa + R*cos(2 * PI*p);
		tempy = ya + R*sin(2 * PI*p);
		isol = InverseKinematics(tempx, tempy);
		sprintf(out, "ROTATE_JOINT ANG1 %7.3lf ANG2 %7.3lf \n", isol.theta1DegRight, isol.theta2DegRight);
	//	printf(out);
		robot.Send(out);
	}
}

/*
void strieghtline(void)
{
   int N, n, y;
   LINE_DATA user;
   INVERSE_SOLUTION isol;
   const int numlines = 10;
   char out[150];//char string for sending positions to robot
   PAST past[numlines];
   double incrementer, tempx, tempy;
   N = user.N + 2;
   RS(user);
   y = 0;//index for multi dimensional array, possibly don't overwrite old lines
   user.color = ColorFind(user);//get the color
   sprintf(out, "PEN_COLOR %d %d %d\n", user.color);
   robot.Send(out);
   for (n = 0; n < N; n++)// find all coordinates
   {

	   incrementer = double(n) / (double(N) - 1);
	   tempx = user.xA + incrementer*(user.xB - user.xA);
	   tempy = user.yA + incrementer*(user.yB - user.yA);
	   past[y].x[n] = tempx;
	   past[y].y[n] = tempy;
   }
   for (n = 0; n < N; n++)
   {
	   isol = InverseKinematics(past[y].x[n], past[y].y[n]);
	  if (n == 0){ robot.Send("PEN_UP\n"); }//lift pen to go to first point
	  if (n == 1){ robot.Send("PEN_DOWN\n"); }//put pen down after that

      if ((user.xA < 0 && user.yA < 0) || (user.xB <0 && user.yB < 0))
      {
         sprintf(out, "ROTATE_JOINT ANG1 %7.3lf ANG2 %7.3lf \n", isol.theta1DegLeft, isol.theta2DegLeft);
     //    printf(out);
         if (isol.theta1DegLeft < MAX_ABS_THETA1_DEG &&isol.theta1DegLeft > -MAX_ABS_THETA1_DEG)
         {
            if (isol.theta2DegLeft < MAX_ABS_THETA2_DEG &&isol.theta2DegLeft > -MAX_ABS_THETA2_DEG)
            {
               robot.Send(out);
            }
         }
      }
      else
      {
         sprintf(out, "ROTATE_JOINT ANG1 %7.3lf ANG2 %7.3lf \n", isol.theta1DegRight, isol.theta2DegRight);
        // printf(out);
         if (isol.theta1DegRight < MAX_ABS_THETA1_DEG &&isol.theta1DegRight > -MAX_ABS_THETA1_DEG)
         {
            if (isol.theta2DegRight < MAX_ABS_THETA2_DEG &&isol.theta2DegRight > -MAX_ABS_THETA2_DEG)
            {
               robot.Send(out);
            }
         }
      }
   }
}
*/
/*
void Drawline(LINE_DATA user)
{
   int N, n, y;
   INVERSE_SOLUTION isol;
   const int numlines = 10;
   char out[150];//char string for sending positions to robot
   PAST past[numlines];
   double incrementer, tempx, tempy;
   fflush(stdin);
   N = user.N + 2;
   y = 0;//index for multi dimensional array, possibly don't overwrite old lines
   for (n = 0; n < N; n++)// find all coordinates
   {

      incrementer = double(n) / (double(N) - 1);
      tempx = user.xA + incrementer*(user.xB - user.xA);
      tempy = user.yA + incrementer*(user.yB - user.yA);
      past[y].x[n] = tempx;
      past[y].y[n] = tempy;
   }
   for (n = 0; n < N; n++)
   {
      isol = InverseKinematics(past[y].x[n], past[y].y[n]);
      if (n == 0){ robot.Send("PEN_UP\n"); }//lift pen to go to first point
      if (n == 1){ robot.Send("PEN_DOWN\n"); }//put pen down after that

      if ((user.xA < 0 && user.yA < 0) || (user.xB <0 && user.yB < 0))
      {
         sprintf(out, "ROTATE_JOINT ANG1 %7.3lf ANG2 %7.3lf \n", isol.theta1DegLeft, isol.theta2DegLeft);
       //  printf(out);
         if (isol.theta1DegLeft < MAX_ABS_THETA1_DEG &&isol.theta1DegLeft > -MAX_ABS_THETA1_DEG)
         {
            if (isol.theta2DegLeft < MAX_ABS_THETA2_DEG &&isol.theta2DegLeft > -MAX_ABS_THETA2_DEG)
            {
               robot.Send(out);
            }
         }
      }
      else
      {
         sprintf(out, "ROTATE_JOINT ANG1 %7.3lf ANG2 %7.3lf \n", isol.theta1DegRight, isol.theta2DegRight);
        // printf(out);
         if (isol.theta1DegRight < MAX_ABS_THETA1_DEG &&isol.theta1DegRight > -MAX_ABS_THETA1_DEG)
         {
            if (isol.theta2DegRight < MAX_ABS_THETA2_DEG &&isol.theta2DegRight > -MAX_ABS_THETA2_DEG)
            {
               robot.Send(out);
            }
         }
      }
   }
}*/
RGB_COLOR ColorFind(LINE_DATA user)
{
	RGB_COLOR col;
	if (((user.xA - user.xB) >= -1 * SLOPE_TOL && (user.xA - user.xB) <= SLOPE_TOL))
	{
		user.color.r = 0;
		user.color.g = 0;
		user.color.b = 0;
		printf("slope inf\n");
	}
	if (((user.xA - user.xB) / (user.yA - user.yB)) <0)
	{
		user.color.r = 255;
		user.color.g = 0;
		user.color.b = 0;
		printf("slope neg\n");
	}
	if (((user.xA - user.xB) / (user.yA - user.yB)) >0)
	{
		user.color.r = 0;
		user.color.g = 0;
		user.color.b = 255;
		printf("slope pos\n");
	}
	if (((user.yA - user.yB)) >= -1 * SLOPE_TOL && (user.yA - user.yB) <= SLOPE_TOL)
	{
		user.color.r = 0;
		user.color.g = 255;
		user.color.b = 0;
		printf("slope 0\n");
	}
	col = user.color;
	return(col);
}
INVERSE_SOLUTION InverseKinematics(double x, double y)
{
   INVERSE_SOLUTION isol;  // structure for solution left/right arm angles 
   double beta, len, alfa, tempradr, tempradl, temprad2, gr; //total arm angle and lenghth and an intermideate angle

   beta = atan2(y, x);
  
   len = sqrt(x*x + y*y);
   alfa = acos(((L2*L2) - (len*len) - (L1*L1)) / (-(2.0 * len*L1)));
   
   gr = acos((pow(L2, 2) - pow(len, 2) - pow(L1, 2)) / (-2 * len*L1));
   isol.theta1DegLeft = beta + alfa;
   isol.theta1DegRight = beta - alfa;
   tempradl = atan2((y - L1*sin(isol.theta1DegLeft)), (x - L1*cos(isol.theta1DegLeft))) - isol.theta1DegLeft;
   tempradr = atan2((y - L1*sin(isol.theta1DegRight)), (x - L1*cos(isol.theta1DegRight))) - isol.theta1DegRight;
   isol.theta2DegLeft = -1*RadToDeg(tempradr);
   isol.theta2DegRight = -1*RadToDeg(tempradl);
   isol.theta1DegLeft = RadToDeg(isol.theta1DegLeft);
   isol.theta1DegRight = RadToDeg(isol.theta1DegRight);
   alfa = RadToDeg(alfa);
   if (len > 600)
   {
	   printf("error      ");
   }
   if (len < 100)
   {
	   printf("error      ");
   }
   
   gr = RadToDeg(gr);
   while (isol.theta1DegLeft < -180) 
   {
      isol.theta1DegLeft = isol.theta1DegLeft + 2 * 180;
   }
   while (isol.theta1DegLeft > 180)/////////////////////////////////////////////////////////////////maybe360????????
   {
      isol.theta1DegLeft = isol.theta1DegLeft - 180;
   }
   while (isol.theta2DegLeft < -180)
   {
      isol.theta2DegLeft = isol.theta2DegLeft + 360;
   }
   while (isol.theta2DegLeft > 180)
   {
      isol.theta2DegLeft = isol.theta2DegLeft - 2 * 180;
   }
   if (isol.theta1DegLeft*isol.theta1DegLeft > 150 * 150)//dont let theta 1 be out of bounds
   {
      if (isol.theta1DegLeft > 150)
      {
         isol.theta1DegLeft = 150;
      }
      else
      {
         isol.theta1DegLeft = -150;
      }
   }
   if (isol.theta2DegLeft*isol.theta2DegLeft > 170 * 170)//dont let theta 2 be out of bounds
   {
	   if (isol.theta2DegLeft > 170)
	   {
		   isol.theta2DegLeft = 170;
	   }
	   else
	   {
		   isol.theta2DegLeft = -170;
	   }
   }


   
   while (isol.theta1DegRight < -180)
   {
      isol.theta1DegRight = isol.theta1DegRight + 2 * 180;
   }
   while (isol.theta1DegRight > 180)/////////////////////////////////////////////////////////////////maybe360????????
   {
      isol.theta1DegRight = isol.theta1DegRight - 360;
   }
   while (isol.theta2DegRight < -180)
   {
      isol.theta2DegRight = isol.theta2DegRight + 360;
   }
   while (isol.theta2DegRight > 180)
   {
      isol.theta2DegRight = isol.theta2DegRight - 2 * 180;
   }
     
     
   if (isol.theta1DegRight*isol.theta1DegRight > 150 * 150)//dont let theta 1 be out of bounds
   {
      if (isol.theta1DegRight > 150)
      {
         isol.theta1DegRight = 150;
      }
      else
      {
         isol.theta1DegRight = -150;
      }
   }
      if (isol.theta2DegRight*isol.theta2DegRight > 170 * 170)//dont let theta 1 be out of bounds
      {
         if (isol.theta2DegRight > 170)
         {
            isol.theta2DegRight = 170;
         }
         else
         {
            isol.theta2DegRight = -170;
         }
      }


   return isol;
}

double DegToRad(double angDeg)
{
   return (PI/180.0)*angDeg;
}

double RadToDeg(double angRad)
{
   return (180.0/PI)*angRad;
}
