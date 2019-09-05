#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <vector>
#include <deque>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

using namespace std;

typedef unsigned char uchar;
vector<uchar> jpgbuff;
deque< vector<uchar> > conjpgbuffs;
vector<uchar> conjpgbuff;

#define endlen 17
#define framelen 10000
#define whomsglen 20
#define clientmsglen 10
uchar Frameend[endlen] = "This is the end!";
uchar whomsg[whomsglen];
uchar clientmsg[clientmsglen];

void carroad_R(void);
void coroad_R(void);
void coroad_S(void);
int ReFromCar(int sockServer, vector<uchar>* jpgbuff);
int tellend(vector<uchar> srcvector, const uchar* goal);
int SeToCar(int sockClient, uchar* msg);
int ReFromClient(int sockClient, uchar* msg);
int SeToClient(int sockClient, vector<uchar> jpgbuff);

int carfd;
int clientfd;
int Clientnetgood=0,Carnetgood=0;
mutex u_con;
int Found_I=0;
int main()
{
	int listenfd;
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in sockAddr;
	
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	sockAddr.sin_port = htons(1234);
	bind(listenfd, (sockaddr*)&sockAddr, sizeof(sockAddr));

	cout << "\n*************************************************************************\n";
	cout <<   "--------Linux Vision-based Control Cloud Server ( By Li Jiatong )--------\n";
	cout <<   "---------------------视觉控制云端服务器 作者：李佳桐---------------------\n\n";

	listen(listenfd, 5);
	cout << "\nlistening ... Please link car and controller!\n（请连接行驶车辆和操控端！）\n";
	
	while(true)
	{
		int temp = accept(listenfd, (struct sockaddr*)NULL, NULL);
		cout << "\nNew User Links in! Socket:"<< temp<<"\n";
		int num = 0;
		num = recv(temp, (char *)whomsg, whomsglen, 0);
		if(num==20)
		{
			if(Carnetgood==1)
				cout<<"Car has linked！Do not repeat linking car!（车辆未掉线，请勿重复接入！）\n";
			else
			{
				carfd=temp;
				Carnetgood=1;
				cout<<"Car links successfully!（车辆接入成功！）\n";
				thread t1(carroad_R);
				t1.detach();
			}
		}
		else if(num==10)
		{
			if(Clientnetgood==1)
				cout<<"Controller has linked！Do not repeat linking controller!（操控端未掉线，请勿重复接入）\n";
			else
			{
				clientfd=temp;
				Clientnetgood=1;
				cout<<"Controller links successfully!（操控端接入成功！）\n";
				thread t2(coroad_R);
				t2.detach();
				thread t3(coroad_S);
				t3.detach();
			}
		}
		else
		{
			cout<<"Link Error! Close SOCKET！（连接错误！关闭该连接！）\n";
			close(temp);
		}
	}
	close(listenfd);
	return 0;
}

void carroad_R(void)
{
	while(Carnetgood)
	{
		jpgbuff.clear();
		if(ReFromCar(carfd, &jpgbuff)==-1)
		{
		  cout<<"\nReFromCar Error!（从车辆接收过程出错！）\n";
		  break;
		}
		if(Clientnetgood==1)
		{
			u_con.lock();
			if(jpgbuff.at(jpgbuff.size()-endlen-1)!=(uchar)'P')
			{
				conjpgbuffs.clear();
				Found_I=1;
			}
			if(Found_I)
				conjpgbuffs.push_back(jpgbuff);
			u_con.unlock();
			cout<<"延迟队列长度："<<conjpgbuffs.size()<<"\n";
		}
	}
	cout<<"----Please relink car! Waiting ... （请重新连接车辆！等待中……）----\n";
	close(carfd);
	Carnetgood=0;
}

void coroad_S(void)
{
	while(Clientnetgood)
	{
		if(!conjpgbuffs.empty())
		{
			u_con.lock();
			conjpgbuff=conjpgbuffs.front();
			conjpgbuffs.pop_front();
			u_con.unlock();
			if(SeToClient(clientfd,conjpgbuff)==-1)
			{
				close(clientfd);
				Found_I=0;
				Clientnetgood=0;
				cout<<"\nSentToClient Error!（发送至操控端过程出错！）\n";
			}
		}
		this_thread::sleep_for(chrono::milliseconds(15));
		this_thread::yield();
	}
}

void coroad_R(void)
{
	while(Clientnetgood)
	{
		if(ReFromClient(clientfd,clientmsg)==-1)
		{
			cout<<"\nReFromClient Error!（从操控端接收过程出错！）\n";
			break;
		}
		if(Carnetgood==1)
		{
			if(SeToCar(carfd,clientmsg)==-1)
			{ 
				close(carfd);
				Carnetgood=0;
				cout<<"\nSeToCar Error!（发送至车辆过程出错！）\n";
			}
		}
	}
	cout<<"----Please relink controller! Waiting ... （请重新连接操控端！等待中……）----\n";
	close(clientfd);
	Found_I=0;
	Clientnetgood=0;
}

vector<uchar> restbuff;
int ReFromCar(int sockServer, vector<uchar>* jpgbuff)
{
	int num = 0;

	vector<uchar> samllbuff;
	size_t smallbuffbeginpos;
	int endlastpos = -1;

	char unitbuff[framelen];

	if (!restbuff.empty())
	{
		samllbuff = restbuff;
		endlastpos = tellend(restbuff, Frameend);
		if (endlastpos < 0)
		{
			for (vector<uchar>::iterator it = restbuff.begin(); it < restbuff.end(); it++)
				jpgbuff->push_back(*it);
			restbuff.clear();
		}
		else
		{
			for (vector<uchar>::iterator it = restbuff.begin(); it < restbuff.begin() + endlastpos + 1; it++)
				jpgbuff->push_back(*it);
			restbuff.erase(restbuff.begin(), restbuff.begin() + endlastpos + 1);
			return 0;
		}
	}

	while (endlastpos < 0)
	{
		if (samllbuff.size() > endlen)
			samllbuff.erase(samllbuff.begin(), samllbuff.end() - endlen);
		smallbuffbeginpos = samllbuff.size();

		num = recv(sockServer, (char *)(unitbuff), framelen, 0);

		if (num <1)
			return -1;

		for (int i = 0; i < num; i++)
			samllbuff.push_back((uchar)(unitbuff[i]));

		endlastpos = tellend(samllbuff, Frameend);
		if (endlastpos < 0)
		{
			for (vector<uchar>::iterator it = samllbuff.begin() + smallbuffbeginpos; it < samllbuff.end(); it++)
				jpgbuff->push_back(*it);
		}
		else
		{
			for (vector<uchar>::iterator it = samllbuff.begin() + smallbuffbeginpos; it < samllbuff.begin() + endlastpos + 1; it++)
				jpgbuff->push_back(*it);
			for (vector<uchar>::iterator it = samllbuff.begin() + endlastpos + 1; it < samllbuff.end(); it++)
				restbuff.push_back(*it);
		}
	}
	return 0;
}

int tellend(vector<uchar> srcvector, const uchar* goal)
{
	for (vector<uchar>::iterator it = srcvector.begin() ; it < srcvector.end() - endlen + 1; it++)
	{
		if (*it == (uchar)(*goal))
		{
			for (int i = 1; i < endlen; i++)
			{
				if (*(it + i) != (uchar)(*(goal + i)))
					break;

				if (i == endlen - 1)
					return (int)(it - srcvector.begin() + i);
			}
		}
	}
	return -1;
}

int SeToCar(int sockClient, uchar* msg)
{
	int num = 0, recvnum = 0;
	while (recvnum < clientmsglen)
	{
		num = send(sockClient, (char *)(msg+recvnum), clientmsglen-recvnum, 0);
		if (num <1)
			return -1;
		recvnum += num;
	}
	return 0;
}

int ReFromClient(int sockClient, uchar* msg)
{
	int num = 0;
	num = recv(sockClient, (char *)msg, clientmsglen, 0);
	if (num <1)
			return -1;
	return 0;
}

int SeToClient(int sockClient, vector<uchar> jpgbuff)
{
	long int num = 0, sentnum = 0;

	size_t ojpgsize = jpgbuff.size();
	while (sentnum < ojpgsize)
	{
		if (ojpgsize - sentnum<framelen)
			num = send(sockClient, (char *)(jpgbuff.data() + sentnum), ojpgsize - sentnum, 0);
		else
			num = send(sockClient, (char *)(jpgbuff.data() + sentnum), framelen, 0);
		if (num == -1)
			return -1;
		sentnum += num;
	}
	return 0;
}
