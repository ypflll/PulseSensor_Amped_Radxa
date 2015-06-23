#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>

#include "wiringX.h"

//  Variables
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 33;                // pin to blink led at each beat

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
volatile char Pulse = 0;     // "True" when User's live heartbeat is detected. "False" when not a "live beat". 
volatile char QS = 0;        // becomes true when Arduoino finds a beat.

volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P =512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile char firstBeat = 1;        // used to seed rate array so we startup with reasonable BPM
volatile char secondBeat = 0;      // used to seed rate array so we startup with reasonable BPM

long long runningTotal = 0; 

void timer_handler (int signum)
{
	int i=0;

	Signal = wiringXanalogRead(pulsePin);              // read the Pulse Sensor 

	sampleCounter += 2;                         // keep track of the time in mS with this variable
	int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

	//  find the peak and trough of the pulse wave
	if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
		if (Signal < T){                        // T is the trough
			T = Signal;                         // keep track of lowest point in pulse wave 
		}
	}

	if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
		P = Signal;                             // P is the peak
	}                                        // keep track of highest point in pulse wave

	//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	// signal surges up in value every time there is a pulse
	if (N > 250){ 				// avoid high frequency noise
		if ( (Signal > thresh) && (Pulse == 0) && (N > (IBI/5)*3) ){      
			Pulse = 1;                        // set the Pulse flag when we think there is a pulse
			digitalWrite(blinkPin,HIGH);                // turn on LED
			IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
			lastBeatTime = sampleCounter;               // keep track of time for next pulse
			
			if(secondBeat){                 // if this is the second beat, if secondBeat == TRUE
				secondBeat = 0;                  // clear secondBeat flag
				for(i=0; i<=9; i++){ // seed the running total to get a realisitic BPM at startup
					rate[i] = IBI;                      
				}
			}

			if(firstBeat){               // if it's the first time we found a beat, if firstBeat == TRUE
				firstBeat = 0;                   // clear firstBeat flag
				secondBeat = 1;                   // set the second beat flag
				return;                              // IBI value is unreliable so discard it
			}   

			// keep a running total of the last 10 IBI values
			runningTotal = 0;                  // clear the runningTotal variable    

			for(i=0; i<=8; i++){                // shift data in the rate array
				rate[i] = rate[i+1];                  // and drop the oldest IBI value 
				runningTotal += rate[i];              // add up the 9 oldest IBI values
			}

			rate[9] = IBI;                          // add the latest IBI to the rate array
			runningTotal += rate[9];                // add the latest IBI to runningTotal
			runningTotal /= 10;                     // average the last 10 IBI values 
			BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
			QS = 1;                              // set Quantified Self flag 
			// QS FLAG IS NOT CLEARED INSIDE THIS ISR
		}                     
	}

	if (Signal < thresh && Pulse == 1){   // when the values are going down, the beat is over
		digitalWrite(blinkPin,LOW);            // turn off LED
		Pulse = 0;                         // reset the Pulse flag so we can do it again
		amp = P - T;                           // get amplitude of the pulse wave
		thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
		P = thresh;                            // reset these for next time
		T = thresh;
	}

	if (N > 2500){                           // if 2.5 seconds go by without a beat
		thresh = 512;                          // set thresh default
		P = 512;                               // set P default
		T = 512;                               // set T default
		lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
		firstBeat = 1;                      // set these to avoid noise
		secondBeat = 0;                    // when we get the heartbeat back
	}
}

void interruptSetup() {
	struct sigaction sa;
	struct itimerval timer;

	/* Install timer_handler as the signal handler for SIGVTALRM. */
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &timer_handler;
	//sigaction (SIGVTALRM, &sa, NULL);
	sigaction (SIGALRM, &sa, NULL);

	/* Configure the timer to expire after 250 msec... */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = 2000;
	/* ... and every 250 msec after that. */
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = 2000;
	/* Start a virtual timer. It counts down whenever this process is
	executing. */
	//setitimer (ITIMER_VIRTUAL, &timer, NULL);
	setitimer (ITIMER_REAL, &timer, NULL);
}

int main(void) {
	int fd;
	char s[5];

	wiringXSetup();

	pinMode(blinkPin, OUTPUT);
	
	if ((fd = wiringXserialOpen ("/dev/ttyS0", 115200)) < 0)	{
	    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	    return -1;
	}

	interruptSetup();

	while(1){
		
		wiringXserialPutchar(fd, 0x53);
		sprintf(s, "%d", Signal);
		wiringXserialPuts (fd , s);
		wiringXserialPutchar(fd, 0x0D);
		wiringXserialPutchar(fd, 0x0A);

		if (QS == 1){     //  A Heartbeat Was Found
				       // BPM and IBI have been Determined
				       // Quantified Self "QS" true when arduino finds a heartbeat
			digitalWrite(blinkPin,LOW);     // Blink LED, we got a beat. 

			wiringXserialPutchar(fd, 0x42);
			sprintf(s, "%d", BPM);
			wiringXserialPuts (fd , s);
			wiringXserialPutchar(fd, 0x0D);
			wiringXserialPutchar(fd, 0x0A);

			wiringXserialPutchar(fd, 0x51);
			sprintf(s, "%d", IBI);
			wiringXserialPuts (fd , s);
			wiringXserialPutchar(fd, 0x0D);
			wiringXserialPutchar(fd, 0x0A);

			QS = 0;                      // reset the Quantified Self flag for next time  
		} 
		else { 
			digitalWrite(blinkPin,HIGH);            // There is not beat, turn off pin 13 LED
		}
		usleep(20000);
	}
}






