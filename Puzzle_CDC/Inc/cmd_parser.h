#ifndef _CMD_PARSER_H_
#define _CMD_PARSER_H_

#include "stm32f0xx_hal.h"

typedef enum ParseError{
	PARSE_OK,
	PARSE_BAD_COMMAND,
	PARSE_BAD_PARAM,
	PARSE_TOO_MANY_PARAMS

}ParseError;

ParseError parse(char* p);

ParseError parse_circular_buffer(void);

#endif /* _CMD_PARSER_H_ */
