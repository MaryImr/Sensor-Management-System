#define gen 0x35 // generator polynomial

unsigned char CRC3(unsigned char command) {
	unsigned char crc = command & 0xE0;// isolate first 3 bits, 11100000
	
	crc = crc>>2; // align with generator
	if( crc >= 0x20)
	crc = crc^gen;

	for( int i = 0; i < 2; i++) {
		crc = crc<<1;
		if( crc >= 0x20)
		crc = crc^gen;
		
	}
	
	command |= crc;
	return command;
	
}

unsigned char CRC_CHECK3(unsigned char command) {
	return CRC3(command & 0xE0) == command;
}

unsigned char CRC11(unsigned char command, char TOS){
	unsigned char crc_bits = command&0x1F;
	unsigned char data = TOS;
	unsigned char temp = 0;
	//unsigned char org_com = command, org_dat = data;
	
	command = command & 0xE0;// isolate first 3 bits, 11100000
	//data ready
	// packet DDDDDDDD
	
	temp = data & 0b11;
	// temp   000000DD
	
	temp = temp<<6;
	// temp	  DD000000
	
	data = data>>2; // align with polynomial
	// data   00DDDDDD
	
	if( data >= 0x20 )
	data ^= gen;
	
	data = data<<1 | temp>> 7; // data 0DDDDDDT (lsb of data from temp)
	temp = temp<<1; // T0000000
	if( data >= 0x20 )
	data ^= gen;
	
	
	data = data<<1 | temp>>7; // data DDDDDDTT ( lsb of data from temp)
	if( data >= 0x20 )
	data ^= gen;
	
	// data DDDDDDDD
	for (int i=0; i < 8; i++){
		data = data<<1 | command>> 7;// data DDDDDDDC
		command = command<<1;		// cmd CCCCCCC0
		if ( data >= 0x20 )
		data ^= gen;
	}
	return data;
}

unsigned char CRC_CHECK11(unsigned char command, char TOS) {
	// data variable now holds the remainder, 5 bits long
	// we will now check if the CRC which we obtained matches [4:0] in command variable
	if ((command & 0x1F) == CRC11(command, TOS))
	return 0xFF;
	
	return 0x00;
	
}