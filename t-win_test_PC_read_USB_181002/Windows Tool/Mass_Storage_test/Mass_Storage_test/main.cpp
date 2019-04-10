#include <Windows.h>
#include <stdio.h>
#include <time.h>    
#include <fstream>
using namespace std;
#define size 8192 

int main() 
{
	int i=0;
	fstream  file ; 
	SYSTEMTIME sys;
	char s[45]={'\0'};

	HANDLE hdisk = CreateFile("\\\\.\\G:", 
		GENERIC_READ, 
		FILE_SHARE_READ | FILE_SHARE_WRITE, 
		nullptr, 
		OPEN_EXISTING, 
		0, NULL);
	if (hdisk == INVALID_HANDLE_VALUE)
	{
		int err = GetLastError();
		// report error...
		printf("error1\n");
		system("pause");
		return -err;
	}

	GetLocalTime( &sys );//get time
	int ss=sys.wSecond;
	int ms=sys.wMilliseconds;

	sprintf(s,"s=%d,ms=%d",ss,ms);
	printf("%s\n",s);

	{//write first.bin
		BYTE buf[size];
		DWORD read;
		LARGE_INTEGER position = { 0 };
		position.QuadPart=512;  //set position address
		SetFilePointerEx(hdisk, position, nullptr, FILE_BEGIN);
		ReadFile(hdisk, buf, size, &read, nullptr);

		file.open("EADC_file/first.bin",ios::out|ios::binary);
		file.write((char*)&buf,size);
		file.close();
	}

	while(1)
	{
		BYTE buf[size];
		DWORD read;
		LARGE_INTEGER position = { 0 };
		position.QuadPart=512;  //set position address
		SetFilePointerEx(hdisk, position, nullptr, FILE_BEGIN);
		ReadFile(hdisk, buf, size, &read, nullptr);

		sprintf(s,"EADC_file/%d.bin",i);
		
		GetLocalTime( &sys );//get time
		if(sys.wSecond>ss && sys.wMilliseconds>=ms)
		{//write end.bin
			file.open("EADC_file/end.bin",ios::out|ios::binary);
			file.write((char*)&buf,size);
			file.close();
			printf("s:%d,ms:%d\n",sys.wSecond,sys.wMilliseconds);
			system("pause");
			return 0;
		}

		file.open(s,ios::out|ios::binary);
		file.write((char*)&buf,size);
		file.close();
		i++;
	}
	return 0;
}