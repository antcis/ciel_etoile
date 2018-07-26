#include <stdio.h>
#include <stdint.h>
#include <types.h>
#include <assert.h>

#include <stdlib.h>
#include <string.h>  
#include <time.h>
#include <unistd.h>
#include <math.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

/*
 *   Simulation parameters
 */

#define START_DATE 24937.3232248727
#define END_DATE 24937.3648915394
#define SECOND 1.0/(24.0*3600.0)
#define SPEEDUP 100
#define SIZE 4

double current_time = START_DATE;
double check_time = START_DATE;
double start_date = START_DATE;
double end_date = END_DATE;
double step = (double) (SECOND*SPEEDUP);

int i;

double pos[4];
double quat[4];

double period_s = 3600.0;  // s
double sma = 10000.0;  // km


/* 
 *   Forward declaration of primary functions 
 */
void compute_send_position(software__array_type* data_source);
void receive_compute_send(software__array_type data_sink);

/*
 *   Forward declaration of orbital functions
 */ 

void set_sqr_position(double ctime);
void set_crcl_position(double ctime);
void set_quaternion(double x, double y, double z);


/*
 *    Main functions
 */ 

void compute_send_position(software__array_type* data_source){

	//Setting position
	set_sqr_position(current_time);

	//Sending position-time vector
	for(int j=0 ;j < 4 ; j++){
		(*data_source)[j] = pos[j];
	}

  	printf ("Emetting position-time vector.\n");
  	printf ("\n");
  	fflush(stdout);

        //Incrementing Time
	current_time += step;
	
	if (current_time > END_DATE){
                //$$ voir comment arrêter le deuxieme noeud
		exit(0);
        }
}

void receive_compute_send(software__array_type data_sink){

	//Receiving position-time vector
	check_time += step;
	assert (check_time == data_sink[3]);


/*
 *   Socket Initialization
 */
    int sock;
    struct sockaddr_in server;
    char msg[5000];

/*
 *   Socket Configuration
 */

    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1)
    {
            perror("Could not create socket.\n");
            exit(0);
	    //exit(EXIT_FAILURE);

    }
    printf("Socket created.\n");
    fflush(stdout);

    //Prepare the sockaddr_in structure
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    //ERREUR HTONL ->HTONS
    server.sin_port = htons(8888);

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("Connect failed. Error\n");
        exit(0);
    }
    printf("Connected.\n");


    	//Sending Stream feed
	double instant = ((data_sink[3] - start_date) * 3600.0 * 24.0);
    	printf("Current time (%d): %lf s\n", i, instant);
    	fflush (stdout);

      	//Send TIME
        sprintf(msg,"TIME %lf\n",data_sink[3]);
        if( send(sock , msg , strlen(msg) , 0) < 0)
        {
            perror("Failed to send Time");
            exit(0);
        }

      	//Send Position
        sprintf(msg, "DATA %lf position \"%lf %lf %lf \"\n",data_sink[3],data_sink[0],data_sink[1],data_sink[2]);
        if( send(sock , msg , strlen(msg) , 0) < 0)
        {
            perror("Failed to send Position");
            exit(0);
        }

	     //Send Quaterion
        set_quaternion(pos[0],pos[1],pos[2]);
        sprintf(msg, "DATA %lf quaternion \"%lf %lf %lf %lf\"\n ",data_sink[3],quat[0],quat[1],quat[2], quat[3]);
        if( send(sock , msg , strlen(msg) , 0) < 0)
        {
            perror("Failed to send Quaternion");
            exit(0);
	    }

	i++;

}



/********************   SECUNDARY FUNCTIONS	**********************/

/*
 *    Orbital functions
 */ 

void set_sqr_position(double ctime){
	
  double t = fmod((ctime * 3600.0 *24.0),period_s);
  double tau = t/period_s;
  
  if (tau < 0.25){
    pos[0] =  sma;
    pos[1] = -sma + tau * 4.0 * 2.0 * sma;
  }
  else if ((0.25 < tau)&&(tau < 0.5)){
    pos[0] =  sma - (tau - 0.25) * 4.0 * 2.0 * sma;
    pos[1] =  sma;
  }
  else if ((0.5 < tau)&&(tau < 0.75)){
    pos[0] = -sma;
    pos[1] =  sma - (tau - 0.5) * 4.0 * 2.0 * sma;
  }
  else{
    pos[0] = -sma + (tau - 0.75) * 4.0 * 2.0 * sma;
    pos[1] = -sma;
}
    pos[2] = 0.0;
    pos[3] = ctime;

}


void set_crcl_position(double ctime){
  double t = fmod((ctime * 3600.0 * 24.0),period_s);
  double tau = t/period_s;

  pos[0] = sma * cos(2.0 * M_PI * (float)tau);
  pos[1] = sma * sin(2.0 * M_PI * (float)tau);
  pos[2] = 0.0;
  pos[3] = ctime;
}

void set_quaternion(double x, double y, double z){
	
	//Euler's angles
	float psi, theta, phi;
	
	psi = atan2(y,x) + M_PI ;
	theta = 0 ;
	phi = - atan2(z, pow(pow(x,2.0) + pow(y,2.0),0.5));
    
   	//Computing quaternion from euler's angle
    	quat[0] = cos(phi/2.0) * cos(theta/2.0) * cos(psi/2.0) - sin(phi/2.0) * sin(theta/2.0) * sin(psi/2.0);
    	quat[1] = sin(phi/2.0) * cos(theta/2.0) * cos(psi/2.0) + cos(phi/2.0) * sin(theta/2.0) * sin(psi/2.0);
    	quat[2] = cos(phi/2.0) * sin(theta/2.0) * cos(psi/2.0) - sin(phi/2.0) * cos(theta/2.0) * sin(psi/2.0);
    	quat[3] = cos(phi/2.0) * cos(theta/2.0) * sin(psi/2.0) + sin(phi/2.0) * sin(theta/2.0) * cos(psi/2.0);
}




