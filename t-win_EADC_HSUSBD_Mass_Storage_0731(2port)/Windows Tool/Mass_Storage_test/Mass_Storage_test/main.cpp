#include <Windows.h>
#include <stdio.h>
#include <time.h>    
#include <fstream>
using namespace std;
#define size 1024  //65536=64K  97280=95K
//°Ñ¦Òhttps://stackoverflow.com/questions/20725397/read-bytes-of-hard-drive

int main() 
{
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
	sprintf(s,"test/(%4d_%02d_%02d)(%02d_%02d_%02d-%03d).bin",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute, sys.wSecond,sys.wMilliseconds);
	printf("%s\n",s);
	//while(1)
	for(int i=0;i<1000;i++)
	{
		BYTE buf[size];
		DWORD read;
		LARGE_INTEGER position = { 0 };
		//position.QuadPart=512;  //set position address
		bool ok = SetFilePointerEx(hdisk, position, nullptr, FILE_BEGIN);
		if(!ok)
		{
			printf("error2\n");
			system("pause");
		}
		ok = ReadFile(hdisk, buf, size, &read, nullptr);
		if(!ok)
		{
			printf("error3\n");
			system("pause");
		}
		GetLocalTime( &sys );//get time
		sprintf(s,"EADC_file/%d.bin",i);
		//sprintf(s,"EADC_file/(%4d_%02d_%02d)(%02d_%02d_%02d-%03d).bin",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute, sys.wSecond,sys.wMilliseconds);
		file.open(s,ios::out|ios::binary);
		file.write((char*)&buf,size);
		file.close();
		//Sleep(1000);
		//system("CLS");
	}
	
	GetLocalTime( &sys );//get time
	sprintf(s,"test/(%4d_%02d_%02d)(%02d_%02d_%02d-%03d).bin",sys.wYear,sys.wMonth,sys.wDay,sys.wHour,sys.wMinute, sys.wSecond,sys.wMilliseconds);
	printf("%s\n",s);
	system("pause");

	return 0;
}