

#include "spi_nor_flash.h"
#include "cmd_parser.h"


#define IS_UPPER(x) ((x)>='A' && (x)<='Z')
#define IS_LOWER(x) ((x)>='a' && (x)<='z')
#define IS_DIGIT(x) ((x)>='0' && (x)<='9')
#define IS_LC_HEX(x) (IS_DIGIT(x) || ((x)>='a' && (x)<='f') )


extern volatile uint8_t cmd_buf[256];
extern volatile uint8_t pCmdHead, pCmdTail;


// <SIGOS>
#define NUM_PARAM 5
typedef struct {
	int n_params;	 /*Number of parameters in Command*/
	char *params[NUM_PARAM];
} t_params_list;
//</SIGOS>

typedef struct  {

	int nParams;
	int* params;

}param_block;

typedef int(*pCmdfunc)(param_block);

typedef int(*pCmdfunct)(t_params_list* pList);


struct cmd_entryT {
	char* pName;
	pCmdfunct pFunc;
	int nParams;
}cmd_entryT;



int cmd_read_status(t_params_list * args) {

	// result_t read_status_register( uint8_t* reg_val );
	return 1;
}

int cmd_read(t_params_list * args) {

	//result_t read_random(uint32_t address, uint32_t size, uint8_t* dest_buf);
	return 2;
}
int cmd_erase_sector(t_params_list * args) {

	//result_t erase_sector(uint32_t sector);
	return 3;
}
int cmd_erase_subsector(t_params_list * args) {

	//result_t erase_subsector(uint32_t subsector);
	return 3;
}
int cmd_program(t_params_list * args) {

	//result_t program(uint32_t address, uint16_t size, uint8_t* source);
	return 3;
}


// page      = 256bytes
// subsector = 4k
// sector    = 64k
// program limitation is 1 page
// In order to preserve existing data in same subsector as destination page,
// subsector data must be cached
// Also, beware of writes that cross a page or subsector boundary


struct cmd_entryT cmd_dictionaryT[] = {

	"status", cmd_read_status, 0,
	"read", cmd_read, 2,
	"erase_sector", cmd_erase_sector, 1,
	"erase_subsector", cmd_erase_subsector, 1,
	"progam", cmd_program, 3

};

pCmdfunct pCmdFunc;
t_params_list paramsList;



//ParseError parse(char* p, pCmdfunct* ppCmdFunc, t_params_list* pList) {
ParseError parse(char* p){
	
	pCmdfunct* ppCmdFunc = &pCmdFunc;
	t_params_list* pList = &paramsList;

	int i = 0;
	//cmdfunc cmdFound = 0;
	*ppCmdFunc = NULL;
	char* px;
	char* pCmdChar;

	// eat leading non-alpha ...0-9...A-Z...a-z...
	while (*p && ((*p < 'A' || *p > 'z') || (*p > 'Z' && *p < 'a'))) { p++; }	
	
	// are there remaining chars to process?
	if (!*p) { return PARSE_BAD_COMMAND; } //no command found

	// for each command in dict, compare with each successive char in candidate
	while (i < sizeof(cmd_dictionaryT) / sizeof(cmd_entryT) && *ppCmdFunc == NULL) {

		px = p;
		pCmdChar = cmd_dictionaryT[i].pName;

		while (*px) {

			if (IS_UPPER(*px)) { *px += 0x20; } // convert alpha to lowercasse

			if (*px != *pCmdChar) { break; }// next if unmatched

			pCmdChar++;
			px++;

			if (!*pCmdChar) {

				// end of pName reached without failed match: success
				*ppCmdFunc = cmd_dictionaryT[i].pFunc;
				goto commandFound;// continue;
			}
		}
		i++;

	}// end while i

	return PARSE_BAD_COMMAND; 

commandFound:
	// we have a valid command, now parse params

	pList->n_params = 0;

	while( *px ){
		
		while(*px == ' '){ px++; }; // eat leading spaces

		// get valid param char or fail
		if( IS_UPPER(*px) ){ *px += 0x20; } ///make lower
		
		if( IS_LOWER(*px) || IS_DIGIT(*px) ){

			pList->params[pList->n_params++] = px;

			if( pList->n_params >= NUM_PARAM ){ return PARSE_TOO_MANY_PARAMS; }
		
			while( IS_LC_HEX(*px) ){ px++; }; // eat valid chars

			// next must be space or EOL
			// if space, we need to terminate the string
			if( *px == ' '){ *px++ = '\0'; }

			
		}else{
			// bad character
			return PARSE_BAD_PARAM;
		}

	}//end while !EOL 

	(*ppCmdFunc)(pList);

	//cmd_dictionary[cmdflag].pFunc(pb);

	return PARSE_OK;
}

//ParseError parse_circular_buffer(void){
//		
//	pCmdfunct* ppCmdFunc = &pCmdFunc;
//	t_params_list* pList = &paramsList;

//	int i = 0;
//	//cmdfunc cmdFound = 0;
//	*ppCmdFunc = NULL;
//	char* px;
//	char* pCmdChar;

//	// eat leading non-alpha ...0-9...A-Z...a-z...
//	while (*p && ((*p < 'A' || *p > 'z') || (*p > 'Z' && *p < 'a'))) { p++; }	
//	
//	// are there remaining chars to process?
//	if (!*p) { return PARSE_BAD_COMMAND; } //no command found

//	// for each command in dict, compare with each successive char in candidate
//	while (i < sizeof(cmd_dictionaryT) / sizeof(cmd_entryT) && *ppCmdFunc == NULL) {

//		px = p;
//		pCmdChar = cmd_dictionaryT[i].pName;

//		while (*px) {

//			if (IS_UPPER(*px)) { *px += 0x20; } // convert alpha to lowercasse

//			if (*px != *pCmdChar) { break; }// next if unmatched

//			pCmdChar++;
//			px++;

//			if (!*pCmdChar) {

//				// end of pName reached without failed match: success
//				*ppCmdFunc = cmd_dictionaryT[i].pFunc;
//				goto commandFound;// continue;
//			}
//		}
//		i++;

//	}// end while i

//	return PARSE_BAD_COMMAND; 

//commandFound:
//	// we have a valid command, now parse params

//	pList->n_params = 0;

//	while( *px ){
//		
//		while(*px == ' '){ px++; }; // eat leading spaces

//		// get valid param char or fail
//		if( IS_UPPER(*px) ){ *px += 0x20; } ///make lower
//		
//		if( IS_LOWER(*px) || IS_DIGIT(*px) ){

//			pList->params[pList->n_params++] = px;

//			if( pList->n_params >= NUM_PARAM ){ return PARSE_TOO_MANY_PARAMS; }
//		
//			while( IS_LC_HEX(*px) ){ px++; }; // eat valid chars

//			// next must be space or EOL
//			// if space, we need to terminate the string
//			if( *px == ' '){ *px++ = '\0'; }

//			
//		}else{
//			// bad character
//			return PARSE_BAD_PARAM;
//		}

//	}//end while !EOL 

//	(*ppCmdFunc)(pList);

//	//cmd_dictionary[cmdflag].pFunc(pb);
//	return PARSE_OK;	
//}

