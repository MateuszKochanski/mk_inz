#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef int SOCKET_HANDLE;
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

//struktura udbieranych wiadomosci
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

//Klasa obsugi komunikacji
class Communication
{
	const int command_start = 0x0002;	//komenda rozpoczecia wysylania danych 
	const int command_stop = 0x0000;	//komenda zakonczenia wysylania dancych
	const int command_bias = 0x0042;	//komenda zerowania
	const int command_filter = 0x0081;	//komenda ustawienia filtra
	const int command_speed = 0x0082;	//komenda ustawienia predkosci wysylania danych
	const int port = 49152;				//Port do komunikacji Ethernet

	const int biasing_on = 0xFF;
	const int biasing_off = 0x00;

	bool connected = false;				//Czy polaczono sie z urzadzeniem
	bool transmit = false;				//Czy czujnik ma wysylac dane

	SOCKET_HANDLE socketHandle;
	Response r;
	ros::Time lastReceiveTime;

	int speed;  // 1000/speed = Speed in Hz 
	int filter; // 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz 

	int Connect(const char * ipAddress);
	void sendCommand(uint16 command, uint32 data);
	void showResponce();
	void prepare();
	void reconnect();

public:

	Communication(int Speed = 100, int Filter = 6);
	~Communication();
	bool receive();
	void start();
	void stop();
	geometry_msgs::Wrench getForce();
};

//konstruktor proboje sie polaczyc z urzadzeniem, jesli sie uda wysyla komendy konfiguracyjne
Communication::Communication(int Speed, int Filter)
{
	fprintf( stderr, "Connection with IP: 10.42.0.50\n" );

	speed = Speed;
	filter = Filter;
	lastReceiveTime = ros::Time::now();
	
	if (Connect("10.42.0.50") != 0) 
	{
		ROS_INFO("No connection");
		connected = false;
	}
	else
	{
		prepare();
		connected = true;
	}
}

//destruktor, konczy komunikacje 
Communication::~Communication()
{
	close(socketHandle);
}

//laczy z urzadzeniem
int Communication::Connect(const char * ipAddress)
{
	struct sockaddr_in addr;	
	struct hostent *he;	
	int err;

	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);//AF_INET - IPv4, SOCK_DGRAM - UDP (dla TCP SOCK_STREAM), ostatni to protocol (najczęsciej 0? nie wiem)

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

//wysyla pojedyncza komende
void Communication::sendCommand(uint16 command, uint32 data)
{
	byte request[8];
	*(uint16*)&request[0] = htons(0x1234); 
	*(uint16*)&request[2] = htons(command); 
	*(uint32*)&request[4] = htonl(data); 
	send(socketHandle, (const char *)request, 8, 0);
	usleep(5000);
}

//proboje sie ponownie polaczyc z urzadzeniem, jesli sie uda wysyla komendy konfiguracyjne
void Communication::reconnect()
{
	ROS_INFO("Reconnect");
	lastReceiveTime = ros::Time::now();

	close(socketHandle);

	if (Connect("10.42.0.50") != 0) 
	{
		ROS_INFO("No connection");
		connected = false;
	}
	else
	{
		prepare();

		if(transmit)
			sendCommand(command_start, 0);

		connected = true;
	}
}

//odbieranie wiadomosci, jesli nie jest polaczony to proboje polaczyc ponownie
bool Communication::receive()
{
	//jezeli nie jest polaczone to sproboj polaczyc ponownie
	if(!connected)
	{
		reconnect();
		return false;
	}
	//jezeli nie rozpoczeto transmisji danych to przerwij
	if(!transmit)
		return false;

	byte inBuffer[36];
	unsigned int uItems = 0;
	int size = recv(socketHandle, (char *)inBuffer, 36, MSG_DONTWAIT);
	if (size == 36)
	{
		lastReceiveTime = ros::Time::now();
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
	{
		if((ros::Time::now() - lastReceiveTime) >= ros::Duration(1.0))
			connected = false;
		
		return false;
	}
		
	
}

//funkcja zwracajaca wartosci otrzymane w wiadomosci
geometry_msgs::Wrench Communication::getForce()
{
	static double force_div = 10000.0;
	static double torque_div = 100000.0;

    	geometry_msgs::Wrench pomiar;
	    
	pomiar.force.x = r.fx / force_div;
    	pomiar.force.y = r.fy / force_div;
    	pomiar.force.z = r.fz / force_div;
    	pomiar.torque.x = r.tx / torque_div;
    	pomiar.torque.y = r.ty / torque_div;
    	pomiar.torque.z = r.tz / torque_div;

    return pomiar;
}

//wyslanie komend z ustawieniami predkosci i filtra oraz zerowanie czujnika
void Communication::prepare()
{
	sendCommand(command_speed, speed);
	sendCommand(command_filter, filter);
	sendCommand(command_bias, biasing_off);
	sendCommand(command_bias, biasing_on);
}

//wyslanie komendy rozpoczecia wykonywania pomiarow
void Communication::start()
{
	sendCommand(command_start, 0);
	transmit = true;
}

//wyslanie komendy zaprzestania wykonywania pomiarow
void Communication::stop()
{
	sendCommand(command_stop, 0);
	transmit = false;
}

//wyswietlanie odebranych danych
void Communication::showResponce()
{
	static double force_div = 10000.0;
	static double torque_div = 100000.0;

	double fx = r.fx / force_div;
	double fy = r.fy / force_div;
	double fz = r.fz / force_div;
	double tx = r.tx / torque_div;
	double ty = r.ty / torque_div;
	double tz = r.tz / torque_div;
	
	fprintf(stdout, "S:%u SN: %u SC: %u Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz);

	fflush(stdout);
}


//zeroje wszystkie wartosci sil i momentow
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
	ros::init(argc, argv, "sensor");
	ros::NodeHandle m;
	ros::Publisher sensor = m.advertise<geometry_msgs::Wrench>("hex",1000);

	geometry_msgs::Wrench pomiar;
	ResetForce(pomiar);

	Communication com;
	
	com.start();
	
	ros::Rate loop_rate(1000); 
	while (ros::ok())
	{	
		//odebranie danych
		if(com.receive())
		{
			//jezeli odebrano dane to je pobierz i opublikuj na kanale "hex"
			pomiar = com.getForce();
			sensor.publish(pomiar);
		}	
		loop_rate.sleep();
	}

	com.stop();

	return 0;
}
