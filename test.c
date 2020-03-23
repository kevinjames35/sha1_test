/*
SHA1 tests by Philip Woolford <woolford.philip@gmail.com>
100% Public Domain
 */

#include "sha1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>



#include <time.h>
#include <ctype.h>
#include "lmbinc.h"
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#define SUCCESS 0

/* The suite initialization function.
 * Returns zero on success, non-zero otherwise.
 */
int init_suite(
    void
)
{
  return 0;
}

/* The suite cleanup function.
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite(
    void
)
{
  return 0;
}


/* Test Vector 7 */
void testvec7(
    void
)
{
SHA1_CTX ctx;
  unsigned char string[40];
  
  unsigned char result[20];
  char hexresult[41];
  size_t offset;
string[0]=0x31;
string[1]=0x00;
string[2]=0x32;
string[3]=0x00;
string[4]=0x33;
string[5]=0x00;
string[6]=0x34;
string[7]=0x00;
string[8]=0x35;
string[9]=0x00;
string[10]=0x36;
string[11]=0x00;
string[12]=0x37;
string[13]=0x00;
string[14]=0x38;
string[15]=0x00;
string[16]=0x39;
string[17]=0x00;
string[18]=0x30;
string[19]=0x00;

string[20]=0x31;
string[21]=0x00;
string[22]=0x32;
string[23]=0x00;
string[24]=0x33;
string[25]=0x00;
string[26]=0x34;
string[27]=0x00;
string[28]=0x35;
string[29]=0x00;
string[30]=0x36;
string[31]=0x00;
string[32]=0x37;
string[33]=0x00;
string[34]=0x38;
string[35]=0x00;
string[36]=0x39;
string[37]=0x00;
string[38]=0x30;
string[39]=0x00;

//char *string2;
//string2=0x31;
  /* calculate hash */
  //SHA1( result, string, 2 );
SHA1Init(&ctx);
SHA1Update(&ctx,string,40);
//printf("ctx :%x\n",ctx.state[0]);
//printf("ctx :%x\n",ctx.state[1]);
//printf("ctx :%x\n",ctx.state[2]);
//printf("ctx :%x\n",ctx.state[3]);
//printf("ctx :%x\n",ctx.state[4]);
//printf("ctx count:%x\n",ctx.count[0]);
SHA1Final(result,&ctx);
  /* format the hash for comparison */
  for( offset = 0; offset < 20; offset++) {
    sprintf( ( hexresult + (2*offset)), "%02x", result[offset]&0xff);
  }
	printf("sha1 :%s\n",hexresult);
}
void __printf_usage(char *argv0)
{
	printf("Usage: %s PASSWORD		:generate SHA1 code using PASSWORD\n", argv0);
	printf("       %s -e PASSWORD		:genarate SHA1 code using PASSWORD and write to eeprom\n", argv0);


}
int main(int argc, char *argv[])
{
int xi,yi;
unsigned char *buff;
unsigned char result[20];
char hexresult[41];
size_t offset;
SHA1_CTX ctx;

uint8_t bSlot=0xFF;
int iRet;
uint16_t wAddr=0;
uint8_t bData=0;




	if ( argc < 2 ) {
		__printf_usage(argv[0]);
		return -1;
	}
	if ( getuid() != 0 ) {
		printf("\e[1;31m<Warning> Please uses root user !!!\e[m\n");
		return -1;
	}

	for ( xi= 1; xi< argc ; xi++ ) {
		if( strcmp("-e", argv[xi]) == 0 ) 
		{
//------------------------------------------------------------------------------------
			printf("input password: %s\tlength: %ld\n",argv[1],strlen(argv[1]));
	
			buff=malloc(2*strlen(argv[1]));
			for(yi=0;yi<(strlen(argv[1]));yi++)
			{
				*(buff+(2*yi))=*(argv[1]+yi);
				*(buff+(2*yi+1))=0x00;
			}
			for(yi=(2*strlen(argv[1]));yi<40;yi++)
			{
				*(buff+yi)=0x00;
			}
			SHA1Init(&ctx);
			SHA1Update(&ctx,buff,40);

			SHA1Final(result,&ctx);
  			/* format the hash for comparison */
  			for( offset = 0; offset < 20; offset++) {
    				sprintf( ( hexresult + (2*offset)), "%02x", result[offset]&0xff);
  			}
			printf("sha1 :%s\n",hexresult);
			free(buff);
				

			iRet = LMB_DLL_Init();
			if ( iRet != ERR_Success ) {
				printf("please confirm the API librraies is matched this platform\n");
				return -1;
			}

			for(wAddr=0;wAddr<20;wAddr++)
			{
				//bData = (uint8_t)(*(buff+wAddr));
				bData = (uint8_t)(result[wAddr]);
	
				iRet = LMB_EEP_WriteByte(bSlot, wAddr, bData);
				if ( iRet == ERR_Success ) printf("%c", bData);
			}



	//------------------------------------------------------------------------------------------------------------------------
		}
		else {
			printf("input password: %s\tlength: %ld\n",argv[1],strlen(argv[1]));
	
			buff=malloc(2*strlen(argv[1]));
			for(yi=0;yi<(strlen(argv[1]));yi++)
			{
				*(buff+(2*yi))=*(argv[1]+yi);
				*(buff+(2*yi+1))=0x00;
			}
			for(yi=(2*strlen(argv[1]));yi<40;yi++)
			{
				*(buff+yi)=0x00;
			}
			SHA1Init(&ctx);
			SHA1Update(&ctx,buff,40);

			SHA1Final(result,&ctx);
  			/* format the hash for comparison */
  			for( offset = 0; offset < 20; offset++) {
    				sprintf( ( hexresult + (2*offset)), "%02x", result[offset]&0xff);
  			}
			//for( yi=0; yi < 20; yi++) {
			//	printf("sha1 :%2X\n",result[yi]);
			//}
			printf("sha1 :%s\n",hexresult);

			free(buff);		
		}
	}
//testvec7();
}

