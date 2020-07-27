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


void __printf_usage(char *argv0)
{
	printf("Usage: %s PASSWORD		:generate SHA1 code using PASSWORD\n", argv0);
	printf("       %s -e PASSWORD		:genarate SHA1 code using PASSWORD and write to eeprom\n", argv0);
	printf("       %s -c			:clear PASSWORD\n", argv0);


}
int main(int argc, char *argv[])
{
int xi,yi;
unsigned char *buff;
unsigned char result[20];
char hexresult[41];
size_t offset;
SHA1_CTX ctx;

uint8_t bSlot=0x0;
int iRet;
uint16_t wAddr=0;
uint8_t bData=0;




	if ( argc < 2 ) {
		__printf_usage(argv[0]);
		return -1;
	}
	
	for ( xi= 1; xi< argc ; xi++ ) {
		if( strcmp("-e", argv[xi]) == 0 ) 
		{
			if ( getuid() != 0 ) {
				printf("\e[1;31m<Warning> Please uses root user !!!\e[m\n");
				return -1;
			}
			iRet = LMB_DLL_Init();
			if ( iRet != ERR_Success ) {
				printf("please confirm the API librraies is matched this platform\n");
				return -1;
			}
//------------------------------------------------------------------------------------
			printf("input password: %s\tlength: %ld\n",argv[2],strlen(argv[2]));
	
			buff=malloc(2*strlen(argv[2]));
			for(yi=0;yi<(strlen(argv[2]));yi++)
			{
				*(buff+(2*yi))=*(argv[2]+yi);
				*(buff+(2*yi+1))=0x00;
			}
			for(yi=(2*strlen(argv[2]));yi<40;yi++)
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
			
				



			for(wAddr=0;wAddr<20;wAddr++)
			{
				//bData = (uint8_t)(*(buff+wAddr));
				bData = (uint8_t)(result[wAddr]);
				printf("->%x\n",bData);
			//	bSlot=0;
				iRet = LMB_EEP_WriteByte(bSlot, wAddr, bData);
				if ( iRet == ERR_Success ) printf("%c", bData);
			}
		
		LMB_DLL_DeInit();
		free(buff);
		return 0;

	//------------------------------------------------------------------------------------------------------------------------
		}
		else if( strcmp("-c", argv[xi]) == 0 ) 
		{
			if ( getuid() != 0 ) {
				printf("\e[1;31m<Warning> Please uses root user !!!\e[m\n");
				return -1;
			}
			iRet = LMB_DLL_Init();
			if ( iRet != ERR_Success ) {
				printf("please confirm the API librraies is matched this platform\n");
				return -1;
			}
//------------------------------------------------------------------------------------
			printf("clear password\n");


			for(wAddr=0;wAddr<20;wAddr++)
			{
				//bData = (uint8_t)(*(buff+wAddr));
				bData = 0x20;
				printf("->%x\n",bData);
			//	bSlot=0;
				iRet = LMB_EEP_WriteByte(bSlot, wAddr, bData);
				if ( iRet == ERR_Success ) printf("%c", bData);
			}
		
		LMB_DLL_DeInit();
		return 0;

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

			//free(buff);
			return 0;		
		}
	}

return 0;
}

