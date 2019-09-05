#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <vector>
#include <iostream>
#include <thread>
using namespace std;

typedef unsigned char uchar;
vector<uchar> jpgbuff;

#define endlen 17
#define framelen 10000
#define whomsglen 20
#define clientmsglen 10
uchar Frameend[endlen] = "This is the end!";
uchar whomsg[whomsglen];
uchar clientmsg[clientmsglen];

void coroad(void);
int ReFromCar(int sockServer, vector<uchar>* jpgbuff);
int tellend(vector<uchar> srcvector, const uchar* goal);
int SeToCar(int sockClient, uchar* msg);
int ReFromClient(int sockClient, uchar* msg);
int SeToClient(int sockClient, vector<uchar> jpgbuff);

int carfd;
int clientfd;
int netgood=1;
int main()
{
	int listenfd;
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in sockAddr;
	
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	sockAddr.sin_port = htons(1234);
	bind(listenfd, (sockaddr*)&sockAddr, sizeof(sockAddr));

	cout << "\n****************Linux Vision-based Control****************\n\n";

	listen(listenfd, 5);
	cout << "listening\n";

	while(carfd==0||clientfd==0)
	{
		int temp=accept(listenfd, (struct sockaddr*)NULL, NULL);
		cout << "\n新用户接入! Socket:"<< temp<<"\n";
		int num = 0;
		num = recv(temp, (char *)whomsg, clientmsglen, 0);
		if(num==20)
		{
			cout<<"该用户是小车！";
			carfd=temp;
		}
		else if(num==10)
		{
			cout<<"该用户是客户端！";
			clientfd=temp;
		}
		else
		{
			cout<<"接入错误!关闭SOCKET！";
			close(temp);
		}
	}
	thread t(coroad);
	t.detach();
	while(netgood)
	{
		char buff[framelen];
		int num2=recv(carfd, (char *)(buff), framelen, 0);
		if(num2==-1)
			break;
		send(clientfd, (char *)(buff), num2, 0);
	}
	sleep(30);
	return 0;
}

void coroad(void)
{
	while(netgood)
	{
		int num2=recv(carfd, (char *)(clientmsg), clientmsglen, 0);
		send(clientfd, (char *)(clientmsg), num2, 0);
		if(num2==-1)
			break;
	}
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
	for (vector<uchar>::iterator it = srcvector.end() - endlen; it > srcvector.begin(); it--)
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