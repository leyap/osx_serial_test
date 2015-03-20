/*
 ***************************************************************************
 *
 * Author: Teunis van Beelen
 *
 * Copyright (C) 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013, 2014 Teunis van Beelen
 *
 * teuniz@gmail.com
 *
 ***************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 ***************************************************************************
 *
 * This version of GPL is at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 ***************************************************************************
 */

/* last revision: Januari 31, 2014 */

/* For more info and how to use this library, visit: http://www.teuniz.net/RS-232/ */


#include "rs232.h"

#if defined (__APPLE__) && defined (__GNUC__) /* osx */


int Cport;
int  error;

struct termios Opt;
//struct termios old_port_settings;


int serialBegin (char* comports, int baudrate, int timeout) {
	int status;

	Cport = open(comports, O_RDWR | O_NOCTTY | O_NDELAY);

	//fcntl (Cport[comport_number], F_SETFL, 0);// ///

	if(Cport==-1) {
		perror("unable to open comport ");
		return(-1);
	}

	error = tcgetattr(Cport, &Opt);
	if(error==-1) {
		close(Cport);
		perror("unable to read portsettings ");
		return(-1);
	}

	//memset(&Opt, 0, sizeof(Opt));  /* clear the new struct, for linux*/
	//Opt.c_cflag = baudr | CS8 | CLOCAL | CREAD;
	//Opt.c_iflag = IGNPAR;
	//Opt.c_oflag = 0;
	//Opt.c_lflag = 0;
	//Opt.c_cc[VMIN] = 0;      /* block untill n bytes are received */
	//Opt.c_cc[VTIME] = timeout;     /* block untill a timer expires (n * 100 mSec.) */

	tcflush (Cport, TCIOFLUSH);	//for osx
	cfsetispeed (&Opt, baudrate);	//for osx
	cfsetospeed (&Opt, baudrate);	//for osx

	error = tcsetattr (Cport, TCSANOW, &Opt);
	if(error==-1) {
		close(Cport);
		perror("unable to adjust portsettings ");
		return(-1);
	}
	tcflush (Cport, TCIOFLUSH);	//for osx

	if(ioctl(Cport, TIOCMGET, &status) == -1) {
		perror("unable to get portstatus");
		return(-1);
	}

	status |= TIOCM_DTR;    /* turn on DTR */
	status |= TIOCM_RTS;    /* turn on RTS */

	if(ioctl(Cport, TIOCMSET, &status) == -1) {
		perror("unable to set portstatus");
		return(-1);
	}

	return(Cport);
}


int serialRead (int comport_number, unsigned char *buf, int size) {
	int n;

	n = read(Cport, buf, size);

	return(n);
}


int serialWriteByte (int comport_number, unsigned char byte) {
	int n;

	n = write(Cport, &byte, 1);
	if(n<0)  return(1);

	return(0);
}


int serialWrite (int comport_number, unsigned char *buf, int size) {
	return(write(Cport, buf, size));
}


void serialClose (int comport_number) {
	int status;

	if(ioctl(Cport, TIOCMGET, &status) == -1) {
		perror("unable to get portstatus");
	}

	status &= ~TIOCM_DTR;    /* turn off DTR */
	status &= ~TIOCM_RTS;    /* turn off RTS */

	if(ioctl(Cport, TIOCMSET, &status) == -1) {
		perror("unable to set portstatus");
	}

	//tcsetattr(Cport, TCSANOW, &old_port_settings);
	close(Cport);
}

/*
   Constant  Description
   TIOCM_LE  DSR (data set ready/line enable)
   TIOCM_DTR DTR (data terminal ready)
   TIOCM_RTS RTS (request to send)
   TIOCM_ST  Secondary TXD (transmit)
   TIOCM_SR  Secondary RXD (receive)
   TIOCM_CTS CTS (clear to send)
   TIOCM_CAR DCD (data carrier detect)
   TIOCM_CD  Synonym for TIOCM_CAR
   TIOCM_RNG RNG (ring)
   TIOCM_RI  Synonym for TIOCM_RNG
   TIOCM_DSR DSR (data set ready)

http://linux.die.net/man/4/tty_ioctl
 */

int RS232_IsDCDEnabled(int comport_number) {
	int status;

	ioctl(Cport, TIOCMGET, &status);

	if(status&TIOCM_CAR) 
		return(1);
	else 
		return(0);
}

int RS232_IsCTSEnabled(int comport_number) {
	int status;

	ioctl(Cport, TIOCMGET, &status);

	if(status&TIOCM_CTS) 
		return(1);
	else 
		return(0);
}

int RS232_IsDSREnabled(int comport_number) {
	int status;

	ioctl(Cport, TIOCMGET, &status);

	if (status&TIOCM_DSR) 
		return(1);
	else 
		return(0);
}

void RS232_enableDTR(int comport_number) {
	int status;

	if(ioctl(Cport, TIOCMGET, &status) == -1) {
		perror("unable to get portstatus");
	}

	status |= TIOCM_DTR;    /* turn on DTR */

	if(ioctl(Cport, TIOCMSET, &status) == -1) {
		perror("unable to set portstatus");
	}
}

void RS232_disableDTR(int comport_number) {
	int status;

	if(ioctl(Cport, TIOCMGET, &status) == -1) {
		perror("unable to get portstatus");
	}

	status &= ~TIOCM_DTR;    /* turn off DTR */

	if(ioctl(Cport, TIOCMSET, &status) == -1) {
		perror("unable to set portstatus");
	}
}

void RS232_enableRTS(int comport_number) {
	int status;

	if(ioctl(Cport, TIOCMGET, &status) == -1) {
		perror("unable to get portstatus");
	}

	status |= TIOCM_RTS;    /* turn on RTS */

	if(ioctl(Cport, TIOCMSET, &status) == -1) {
		perror("unable to set portstatus");
	}
}

void RS232_disableRTS(int comport_number) {
	int status;

	if(ioctl(Cport, TIOCMGET, &status) == -1) {
		perror("unable to get portstatus");
	}

	status &= ~TIOCM_RTS;    /* turn off RTS */

	if(ioctl(Cport, TIOCMSET, &status) == -1) {
		perror("unable to set portstatus");
	}
}


#else         /* windows */

#ifdef _win32

HANDLE Cport[16];


char comports[16][10]={"\\\\.\\COM1",  "\\\\.\\COM2",  "\\\\.\\COM3",  "\\\\.\\COM4",
	"\\\\.\\COM5",  "\\\\.\\COM6",  "\\\\.\\COM7",  "\\\\.\\COM8",
	"\\\\.\\COM9",  "\\\\.\\COM10", "\\\\.\\COM11", "\\\\.\\COM12",
	"\\\\.\\COM13", "\\\\.\\COM14", "\\\\.\\COM15", "\\\\.\\COM16"};

char baudr[64];


int serialBegin (int comport_number, int baudrate, int timeout)
{
	if((comport_number>150)||(comport_number<0))
	{
		printf("illegal comport number\n");
		return(1);
	}

	switch(baudrate)
	{
		case     110 : strcpy(baudr, "baud=110 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case     300 : strcpy(baudr, "baud=300 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case     600 : strcpy(baudr, "baud=600 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case    1200 : strcpy(baudr, "baud=1200 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case    2400 : strcpy(baudr, "baud=2400 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case    4800 : strcpy(baudr, "baud=4800 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case    9600 : strcpy(baudr, "baud=9600 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case   19200 : strcpy(baudr, "baud=19200 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case   38400 : strcpy(baudr, "baud=38400 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case   57600 : strcpy(baudr, "baud=57600 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case  115200 : strcpy(baudr, "baud=115200 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case  128000 : strcpy(baudr, "baud=128000 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case  256000 : strcpy(baudr, "baud=256000 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case  500000 : strcpy(baudr, "baud=500000 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		case 1000000 : strcpy(baudr, "baud=1000000 data=8 parity=N stop=1 dtr=on rts=on");
			       break;
		default      : printf("invalid baudrate\n");
			       return(1);
			       break;
	}

	Cport[comport_number] = CreateFileA(comports[comport_number],
			GENERIC_READ|GENERIC_WRITE,
			0,                          /* no share  */
			NULL,                       /* no security */
			OPEN_EXISTING,
			0,                          /* no threads */
			NULL);                      /* no templates */

	if(Cport[comport_number]==INVALID_HANDLE_VALUE)
	{
		printf("unable to open comport\n");
		return(1);
	}

	DCB port_settings;
	memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
	port_settings.DCBlength = sizeof(port_settings);

	if(!BuildCommDCBA(baudr, &port_settings))
	{
		printf("unable to set comport dcb settings\n");
		CloseHandle(Cport[comport_number]);
		return(1);
	}

	if(!SetCommState(Cport[comport_number], &port_settings))
	{
		printf("unable to set comport cfg settings\n");
		CloseHandle(Cport[comport_number]);
		return(1);
	}

	COMMTIMEOUTS Cptimeouts;

	Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
	Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
	Cptimeouts.ReadTotalTimeoutConstant    = timeout;
	Cptimeouts.WriteTotalTimeoutMultiplier = 0;
	Cptimeouts.WriteTotalTimeoutConstant   = timeout;

	if(!SetCommTimeouts(Cport[comport_number], &Cptimeouts))
	{
		printf("unable to set comport time-out settings\n");
		CloseHandle(Cport[comport_number]);
		return(1);
	}

	return(0);
}


int serialRead (int comport_number, unsigned char *buf, int size)
{
	int n;

	/* added the void pointer cast, otherwise gcc will complain about */
	/* "warning: dereferencing type-punned pointer will break strict aliasing rules" */

	ReadFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL);

	return(n);
}


int serialWriteByte (int comport_number, unsigned char byte)
{
	int n;

	WriteFile(Cport[comport_number], &byte, 1, (LPDWORD)((void *)&n), NULL);

	if(n<0)  return(1);

	return(0);
}


int serialWrite (int comport_number, unsigned char *buf, int size)
{
	int n;

	if(WriteFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL))
	{
		return(n);
	}

	return(-1);
}


void serialClose (int comport_number)
{
	CloseHandle(Cport[comport_number]);
}

/*
http://msdn.microsoft.com/en-us/library/windows/desktop/aa363258%28v=vs.85%29.aspx
 */

int RS232_IsDCDEnabled(int comport_number)
{
	int status;

	GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

	if(status&MS_RLSD_ON) return(1);
	else return(0);
}


int RS232_IsCTSEnabled(int comport_number)
{
	int status;

	GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

	if(status&MS_CTS_ON) return(1);
	else return(0);
}


int RS232_IsDSREnabled(int comport_number)
{
	int status;

	GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

	if(status&MS_DSR_ON) return(1);
	else return(0);
}


void RS232_enableDTR(int comport_number)
{
	EscapeCommFunction(Cport[comport_number], SETDTR);
}


void RS232_disableDTR(int comport_number)
{
	EscapeCommFunction(Cport[comport_number], CLRDTR);
}


void RS232_enableRTS(int comport_number)
{
	EscapeCommFunction(Cport[comport_number], SETRTS);
}


void RS232_disableRTS(int comport_number)
{
	EscapeCommFunction(Cport[comport_number], CLRRTS);
}


#endif
#endif


void serialPrint (int comport_number, const char *text)  /* sends a string to serial port */
{
	while(*text != 0)   serialWriteByte (comport_number, *(text++));
}


