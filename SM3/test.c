#include "stdio.h"
#include "sm3.h"

void main()
{
	int i;
	unsigned char inputBuf[32] = //{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//{0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x03,0x03,0x03,0x03,0x04,0x04,0x04,0x04};
{0x61,0x62,0x63};
	unsigned char outputBuf[32];
	sm3(inputBuf, 3, outputBuf); //

	for(i=0;i<32;i++)	
	{
		printf("0x%x ",outputBuf[i]);
	}
	printf("\n");
}
