#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
typedef int SOCKET_HANDLE;

#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PORT			49152	/* Port the Ethernet DAQ always uses */
#define SPEED			100		/* 1000 / SPEED = Speed in Hz */
#define FILTER			6		/* 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz */
#define BIASING_ON		0xFF    /* Biasing on */
#define BIASING_OFF		0x00    /* Biasing off */

#define COMMAND_START	0x0002  /* Command for start streaming */
#define COMMAND_STOP	0x0000  /* Command for stop streaming */
#define COMMAND_BIAS	0x0042  /* Command for toggle biasing */
#define COMMAND_FILTER	0x0081  /* Command for setting filter */
#define COMMAND_SPEED	0x0082  /* Command for setting speed */

#define	FORCE_DIV	10000.0  // Default divide value
#define	TORQUE_DIV	100000.0 // Default divide value


typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;



struct Response {
	unsigned int sequenceNumber;    
	unsigned int sampleCounter;  
 	unsigned int status;		
	int32 fx;
	int32 fy;
	int32 fz;
	int32 tx;
	int32 ty;
	int32 tz;
};

class Communication
{
	SOCKET_HANDLE socketHandle;
	Response r;

	void mySleep(unsigned long ms);
	int Connect(const char * ipAddress, uint16 port);
	void showResponse(Response r);
	void sendCommand(uint16 command, uint32 data);
	void showResponce();

	public:

	Communication();
	~Communication();
	bool receive();
	void prepare();
	void start();
	void stop();
	geometry_msgs::Wrench getForce();
};

Communication::Communication()
{
	fprintf( stderr, "Usage: IPADDRESS: 10.42.0.50\n" );

	if (Connect("10.42.0.50", PORT) != 0) {
		fprintf(stderr, "Could not connect to device...");
	}
}

Communication::~Communication()
{
	close(socketHandle);
}

int Communication::Connect(const char * ipAddress, uint16 port)
{
	struct sockaddr_in addr;	
	struct hostent *he;	
	int err;

	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (socketHandle == -1) { 
		fprintf(stderr, "Socket could not be opened.\n");
		return -2;
	}
	he = gethostbyname(ipAddress);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	
	err = connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		return -3;
	}
	return 0;
}

void Communication::sendCommand(uint16 command, uint32 data)
{
	byte request[8];
	*(uint16*)&request[0] = htons(0x1234); 
	*(uint16*)&request[2] = htons(command); 
	*(uint32*)&request[4] = htonl(data); 
	send(socketHandle, (const char *)request, 8, 0);
	mySleep(5); // Wait a little just to make sure that the command has been processed by Ethernet DAQ
}

void Communication::mySleep(unsigned long ms)
{
	usleep(ms * 1000);
}

bool Communication::receive()
{
	byte inBuffer[36];
	unsigned int uItems = 0;
	int size = recv(socketHandle, (char *)inBuffer, 36, MSG_DONTWAIT);
	
	if (size == 36)
	{
		r.sequenceNumber = ntohl(*(uint32*)&inBuffer[0]);
		r.sampleCounter = ntohl(*(uint32*)&inBuffer[4]);
		r.status = ntohl(*(uint32*)&inBuffer[8]);
		r.fx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
		r.fy = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4])); 
		r.fz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
		r.tx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
		r.ty = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
		r.tz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
		return true;
	}
	else
		return false;
}

geometry_msgs::Wrench Communication::getForce()
{
    geometry_msgs::Wrench pomiar;
	    
	pomiar.force.x = r.fx / FORCE_DIV;
    pomiar.force.y = r.fy / FORCE_DIV;
    pomiar.force.z = r.fz / FORCE_DIV;
    pomiar.torque.x = r.tx / TORQUE_DIV;
    pomiar.torque.y = r.ty / TORQUE_DIV;
    pomiar.torque.z = r.tz / TORQUE_DIV;

    return pomiar;
}

void Communication::prepare()
{
	sendCommand(COMMAND_SPEED, SPEED);
	sendCommand(COMMAND_FILTER, FILTER);
	sendCommand(COMMAND_BIAS, BIASING_OFF);
	sendCommand(COMMAND_BIAS, BIASING_ON);//zerowanie
}

void Communication::start()
{
	sendCommand(COMMAND_START, 0);
}

void Communication::stop()
{
	sendCommand(COMMAND_STOP, 0);
}


void Communication::showResponce()
{
	double fx = r.fx / FORCE_DIV;
	double fy = r.fy / FORCE_DIV;
	double fz = r.fz / FORCE_DIV;
	double tx = r.tx / TORQUE_DIV;
	double ty = r.ty / TORQUE_DIV;
	double tz = r.tz / TORQUE_DIV;
	
	fprintf(stdout, "S:%u SN: %u SC: %u Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz);

	fflush(stdout);
}


void ResetForce(geometry_msgs::Wrench &pom)
{
    pom.force.y = 0;
	pom.force.x = 0;
	pom.force.z = 0;
    pom.torque.x = 0;
    pom.torque.y = 0;
    pom.torque.z = 0;
}


int main ( int argc, char ** argv ) 
{
	ros::init(argc, argv, "signal");
	ros::NodeHandle m;
	ros::Publisher signal = m.advertise<geometry_msgs::Wrench>("hex",1000);

	geometry_msgs::Wrench pomiar;
	ResetForce(pomiar);

	Communication com;
	
	com.prepare();
	com.start();
	
	while (ros::ok())
	{	
		if(com.receive())
		{
			pomiar = com.getForce();
			signal.publish(pomiar);
		}	
	}

	com.stop();

	return 0;
}
