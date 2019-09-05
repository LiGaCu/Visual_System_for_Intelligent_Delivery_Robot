#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <vector>
#include <iostream>
#include <thread>
using namespace std;

typedef unsigned char uchar;
vector<uchar> jpgbuff;
vector<uchar> conjpgbuff;
int controljpgflag;

#define endlen 17
#define framelen 10000
#define whomsglen 20
#define clientmsglen 10
uchar Frameend[endlen] = "This is the end!";
uchar whomsg[whomsglen];
uchar clientmsg[clientmsglen];

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
int main()
{
	int listenfd;
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in sockAddr;
	
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	sockAddr.sin_port = htons(1234);
	bind(listenfd, (sockaddr*)&sockAddr, sizeof(sockAddr));

	cout << "\n****************Linux Vision-based Control Cloud Server****************\n\n";

	listen(listenfd, 5);
	cout << "listening ... Please link car and controller!\n";

  while(true)
  {
	while(Clientnetgood==0||Carnetgood==0)
	{
		int temp=accept(listenfd, (struct sockaddr*)NULL, NULL);
		cout << "\nNew User Links in! Socket:"<< temp<<"\n";
		int num = 0;
		num = recv(temp, (char *)whomsg, whomsglen, 0);
		if(num==20)
		{
      if(Carnetgood==1)
        cout<<"Car has linked£¡Do not repeat linking car!\n";
      else
      {
        carfd=temp;
        Carnetgood=1;
        cout<<"Car links successfully!\n";
			}
		}
		else if(num==10)
		{
      if(Clientnetgood==1)
        cout<<"Controller has linked£¡Do not repeat linking controller!\n";
      else
      {
        clientfd=temp;
        Clientnetgood=1;
        cout<<"Controller links successfully!\n";
			}
		}
		else
		{
			cout<<"Link Error! Close SOCKET£¡\n";
			close(temp);
		}
	}
	cout<<"-----Everything good! Working ...----\n";
	thread t1(coroad_R);
	t1.detach();
	while(Clientnetgood&&Carnetgood)
	{
    jpgbuff.clear();
    if(ReFromCar(carfd, &jpgbuff)==-1)
    {
      Carnetgood=0;
      cout<<"ReFromCar Error!\n";
      break;
    }
    if(SeToCar(carfd,clientmsg)==-1)
    {    
      Carnetgood=0;
      cout<<"SeToCar Error!\n";
    }
    if(controljpgflag==0)
    {
      controljpgflag=1;
      conjpgbuff=jpgbuff;
      thread t2(coroad_S);
      t2.detach();
    }
	}
	if(Carnetgood==0)
	{
    close(carfd);
    cout<<"----Please relink car! Waiting ... ----\n";
  }
  if(Clientnetgood==0)
    close(clientfd);
    cout<<"----Please relink controller! Waiting ... ----\n";
  }
	return 0;
}

void coroad_S(void)
{
  if(SeToClient(clientfd,conjpgbuff)==-1)
  {
    Clientnetgood=0;
    cout<<"SentToClient Error!\n";
  }
  controljpgflag=0;
}

void coroad_R(void)
{
  while(Clientnetgood)
  {
    if(ReFromClient(clientfd,clientmsg)==-1)
    {
      Clientnetgood=0;
      cout<<"ReFromClient Error!\n";
      break;
    }
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
