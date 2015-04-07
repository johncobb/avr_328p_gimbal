/*
 * cli.h
 *
 *  Created on: Apr 6, 2015
 *      Author: jcobb
 */

#ifndef CLI_H_
#define CLI_H_

/*
 * command line interface variables
 */

// command line interface parsing results
enum cli_parse_result {
 CLI_TKNOTFOUND = 0,
 CLI_TKFOUND,
 CLI_TKERROR
};


// token start and end
#define	CLI_DELIM			" "
#define CLI_TKSTART			'*'
#define CLI_TKEND			'\r'
#define CLI_RX_BUFFER_SIZE 	512


typedef struct
{
	unsigned char buffer[CLI_RX_BUFFER_SIZE];
	int head;
	int tail;
} CLI_BUFFER;

void cli_put_char(unsigned char c);
void cli_tick();

#endif /* CLI_H_ */
