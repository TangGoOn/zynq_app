//============================================================================
// Name        : helloworld.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>

using namespace std;

int main()
{
	int a=1,b=1,c=0;

	if(b==1)
	{
		c = 1;
	}
	else if(a==1)
	{
		c = 2;
	}

	c = 3;
	//sleep(2);

	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
