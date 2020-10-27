/* =========================================================================
 * BMC HexDM control server
 * 
 * uses shared memory (ImageStreamIO library by O. Guyon)
 * ========================================================================= */

#include <stdio.h>

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#	define _GNUC3_
#  endif
#endif

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <math.h>
#include <pthread.h>
#include <curses.h>

#include "ImageStruct.h"
#include "ImageStreamIO.h"

#include "bmc_mdlib.h"

/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define LINESIZE 256

int wxsz, wysz; // window size
IMAGE *shmarray = NULL; // shared memory img pointer (defined in ImageStreamIO.h)
int nch         = 4;    // number of DM channels (default = 4)
int nseg        = 169;  // number of segments on the DM
int ndof        = 3;    // number of d.o.f per segment (piston, tip & tilt)
int csz         = 1024; // size of the command expected by the driver
int keepgoing   = 0;    // flag to control the Hex DM update loop
int allocated   = 0;    // flag to control whether the shm structures are allocated
char dashline[80] =
  "-----------------------------------------------------------------------------\n";

static tBMC sBMC = NULL; // handle
static int sNdx = 0;     // board index

/* =========================================================================
 *                       function prototypes
 * ========================================================================= */
void print_help();
int log_action(char *msg);
int shm_setup(int nch0);
int reset_channel(int chindex);
void* hexdm_loop(); //void *params);
float* ptt_2_actuator(float* ptt);
static void MakeOpen(void);

/* =========================================================================
 *                                BMC API
 * ========================================================================= */
static void MakeOpen(void) {
  tE	err;
  
  if (NULL != sBMC) { return; }
  
  if (kBMCEnoErr != (err = BMCopen(sNdx,&sBMC))) {
    endwin(); // from curses back to regular env!
    printf("MakeOpen: %d gave %s\n", sNdx, BMCgetErrStr(err));
    exit(1);
  }
  return;
}

/* =========================================================================
 *                       Displays the help menu
 * ========================================================================= */
void print_help() {
  char fmt[20] = "%15s %20s %40s\n";
  attron(COLOR_PAIR(3));
  mvprintw(6, 0, dashline);
  printw("            camera control shell help menu\n");
  printw("%s", dashline);
  printw(fmt, "command", "parameters", "description");
  printw("%s", dashline);
  printw(fmt, "help", "",          "prints this help message");
  printw(fmt, "quit", "",          "stops the HexDM!");
  printw(fmt, "set_nch", "integer", "sets the number of channels to val");
  printw(fmt, "start", "",          "starts the HexDM (set_nch first!)");
  printw(fmt, "get_nch", "",        "returns the current number of channels");
  printw(fmt, "reset",   "integer", "reset channel #k (-1 for all channels)");
  printw("%s", dashline);
  attroff(COLOR_PAIR(3));
}

/* =========================================================================
 *                 log server interaction in a file
 * ========================================================================= */

int log_action(char *msg) {
  struct timespec now;       // clock readout
  struct tm* ptm;

  FILE* fd = fopen("log_HexDM.log", "a");

  clock_gettime(CLOCK_REALTIME, &now); // what is the time ??
  ptm = localtime(&(now.tv_sec));

  fprintf(fd, "%02d:%02d:%02d  %s\n",
		  ptm->tm_hour, ptm->tm_min, ptm->tm_sec,
		  msg);
  
  fclose(fd);
  return 0;
}

/* =========================================================================
 *                      shared memory setup
 * ========================================================================= */
int shm_setup(int nch0) {
  /* Allocates shared memory data structures for the new settings
   *
   * Parameters:
   * ----------
   * - nch0: the number of channels there was before the update (for deallocation)
   */
  int ii;
  int shared    = 1;
  int NBkw      = 10;
  long naxis    = 2;
  uint8_t atype = _DATATYPE_FLOAT;
  uint32_t *imsize;
  char shmname[20];
  
  imsize = (uint32_t *) malloc(sizeof(uint32_t)*naxis);
  imsize[0] = ndof; // piston, x-tip and y-tilt only
  imsize[1] = nseg; // for each of the 169 segments


  if (shmarray != NULL) { // structure must be freed before reallocation!
	for (ii = 0; ii < nch0 + 1; ii++) {
	  ImageStreamIO_destroyIm(&shmarray[ii]);
	}
	free(shmarray);
	shmarray = NULL;
  }
  
  shmarray = (IMAGE*) malloc((nch+1) * sizeof(IMAGE));
  printw("allocating for %d channels!", nch);

  // individual channels
  for (ii = 0; ii < nch; ii++) {
	sprintf(shmname, "ptt%02d", ii); // building the root name of the shm
	ImageStreamIO_createIm_gpu(&shmarray[ii], shmname, naxis, imsize, atype, -1,
				   shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
  }
  // the combined array!
  sprintf(shmname, "ptt");           // building the root name of the shm
  ImageStreamIO_createIm_gpu(&shmarray[nch], shmname, naxis, imsize, atype, -1,
			     shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);

  free(imsize);
  return 0;
}

/* =========================================================================
 *                     reset values on a DM channel
 * ========================================================================= */
int reset_channel(int chindex) {
  int ii, kk;
  char loginfo[LINESIZE];
  
  if (chindex < 0) {
	// reset all DM channels
	for (kk = 0; kk < nch; kk++) {
	  shmarray[kk].md->write = 1;       // signaling about to write
	  for (ii = 0; ii < nseg*ndof; ii++) 
		shmarray[kk].array.F[ii] = 0.0; // reset the DM channel #kk
	  
	  shmarray[kk].md->cnt1 = 0;
	  shmarray[kk].md->cnt0++;
	  ImageStreamIO_sempost(&shmarray[kk], -1);
	  shmarray[kk].md->write = 0;
	}
	log_action("All DM channels were reset");
  }
  else {
	if (chindex < nch) {
	  shmarray[chindex].md->write = 1;       // signaling about to write
	  for (ii = 0; ii < nseg*ndof; ii++)      
		shmarray[chindex].array.F[ii] = 0.0; // reset the DM channel
	  
	  shmarray[chindex].md->cnt1 = 0;
	  shmarray[chindex].md->cnt0++;
	  ImageStreamIO_sempost(&shmarray[chindex], -1);
	  shmarray[chindex].md->write = 0;

	  sprintf(loginfo, "channel %02d was reset", chindex);
	  log_action(loginfo);
	}
  }
  return 0;
}

/* =========================================================================
 *                       DM surface control thread
 * ========================================================================= */
void* hexdm_loop() {//void *params) {
  uint64_t cntrs[nch];
  int ii, kk;
  //FILE* fd;
  int updated = 0;
  float tmp_sum[nseg*ndof]; // temporary sum array
  float* cmd;               // array storing the DM cmd before upload
  tU16 pokeval;             // array uploaded to the driver
  
  // init image counters
  for (ii = 0; ii < nch; ii++) {
	cntrs[ii] = shmarray[nch].md->cnt0;
  }
  
  while (keepgoing > 0) {
	// look for updates to the shm counters

	updated = 0; // reset the marker
	
	for (ii = 0; ii < nch; ii++) {
	  if (shmarray[ii].md->cnt0 > cntrs[ii]) {
		updated += 1;
		cntrs[ii] = shmarray[ii].md->cnt0;
	  }
	}

	if (updated > 0) { // a counter was updated!
	  log_action("HexDM state updated");
	  for (ii = 0; ii < ndof * nseg; ii++)
		tmp_sum[ii] = 0.0; // init temp sum array

	  for (kk = 0; kk < nch; kk++) // combine the channels
		for (ii = 0; ii < ndof * nseg; ii++) 
		  tmp_sum[ii] += shmarray[kk].array.F[ii];
	  
	  shmarray[nch].md->write = 1;         // signaling about to write
	  for (ii = 0; ii < ndof * nseg; ii++) // update the combined channel
		shmarray[nch].array.F[ii] = tmp_sum[ii];
	  shmarray[nch].md->cnt1 = 0;
	  shmarray[nch].md->cnt0++;
	  ImageStreamIO_sempost(&shmarray[nch], -1);
	  shmarray[nch].md->write = 0;

	  // compute the commands to actually send to the DM here!
	  cmd = ptt_2_actuator(shmarray[nch].array.F);
	  
	  log_action("the driver command was sent!");
	  
	  // upload the command to the driver
	  // !!!!!!!! HERE !!!!!!!!!!
	  for (ii = 0; ii < ndof*nseg; ii++) {
	    pokeval = (tU16)(cmd[ii] * 65536);
	    BMCpokeDM(sBMC, (tU32)ii, pokeval);
	  }
	  
	  /* fd = fopen("dm_cmd.txt", "w"); */
	  /* for (ii = 0; ii < 1024; ii++) { */
	  /* 	fprintf(fd, "%.2f\n", cmd[ii]); */
	  /* } */
	  /* fclose(fd); */
	  
	  // release the memory allocated for the computation of the command
	  free(cmd);
	}
	usleep(100000); // to be adjusted or removed!
  }
  return NULL;
}

/* =========================================================================
 *    conversion from PTT commands to actuator command for the driver
 * 
 * expects the 3 column ptt argument to consist in:
 * - piston values (in nanometers)
 * - tip and tilt values (in mrad)
 *
 * Code developed in part by KERNEL student Coline Lopez.
 * ========================================================================= */
float* ptt_2_actuator(float* ptt) {
  int ii;               // dummy variable
  int csz = 1024;       // size of the command expected by the driver
  float again = 4000.0; // actuator gain: 4 um per ADU ? To be refined
  float a0 = 218.75;    // actuator location radius in microns

  float *res = (float*) malloc(csz * sizeof(float));
  
  for (ii = 0; ii < nseg; ii++) {
	res[ii*ndof]   = ptt[ii*ndof] +
	  a0 * sqrt(3.0)/2.0 * ptt[ii*ndof+1] + a0/2.0 * ptt[ii*ndof+2] ; 
	res[ii*ndof+1] = ptt[ii*ndof] -
	  a0 * ptt[ii*ndof+2]; 
	res[ii*ndof+2] = ptt[ii*ndof] -
	  a0 * sqrt(3.0)/2.0 * ptt[ii*ndof+1] + a0/2.0 * ptt[ii*ndof+2];
  }
  
  for (ii = 0; ii < nseg*ndof; ii++)
  	res[ii] /= again;
  return res;				  
}

/* =========================================================================
 *                                Main program
 * ========================================================================= */
int main() {
  char cmdstring[LINESIZE];
  char loginfo[LINESIZE];
  int cmdOK = 0;
  int ii = 0;
  int ival = 0;
  char str0[20];
  int nch_prev = 0; // temporary integer value
  pthread_t tid_loop; // thread ID for DM control loop
  
  // ----- curses specifics -----
  initscr(); // start curses mode
  start_color();
  getmaxyx(stdscr, wysz, wxsz);
  init_pair(1, COLOR_RED,    COLOR_BLACK);
  init_pair(2, COLOR_GREEN,  COLOR_BLACK);
  init_pair(3, COLOR_YELLOW, COLOR_BLACK);
  init_pair(4, COLOR_BLUE,   COLOR_BLACK);

  // --------------------- set-up the prompt --------------------
  attron(COLOR_PAIR(2));
  printw("%s", dashline);
  printw("                   HexDM CONTROL INTERACTIVE SHELL\n");
  printw("\nDid you launch this program from within a tmux as it is meant?\n");
  printw("\n");
  printw("%s", dashline);
  attroff(COLOR_PAIR(2));

  // --------------------------
  // Connect to the DM driver
  // --------------------------
  MakeOpen();               
  
  // --------------------------
  //   start command line
  // --------------------------
  for (;;) { // infinite loop
    cmdOK = 0;

	attron(COLOR_PAIR(3));
	move(wysz-3, 0);
	clrtoeol();
	printw("HexDM > ");
	attroff(COLOR_PAIR(3));
	getstr(cmdstring);

  // --------------------------
  //     process commands
  // --------------------------
	
	// =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "start", strlen("start")) == 0) {
		if (allocated == 0) {
		  attron(COLOR_PAIR(1) | A_BOLD);
		  printw("set the desired number of channels\n");
		  clrtoeol();
		  attroff(COLOR_PAIR(1) | A_BOLD);
		}
		else {
		  if (keepgoing == 0) {
			keepgoing = 1; // raise the flag!
			sprintf(loginfo, "DM control loop START");
			log_action(loginfo);
			pthread_create(&tid_loop, NULL, hexdm_loop, NULL);
		  }
		}
		cmdOK = 1;
	  }
	
	// =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "stop", strlen("stop")) == 0) {
		cmdOK = 1;
		if (keepgoing == 1) {
		  sprintf(loginfo, "DM control loop STOP");
		  log_action(loginfo);
		  keepgoing = 0;		  
		}
		cmdOK = 1;
	  }

	// =====================================================
	//    set the desired number of HexDM channels
	// =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "set_nch", strlen("set_nch")) == 0) {
		ival = 0; // reset value
		sscanf(cmdstring, "%s %d", str0, &ival);
		
		attron(COLOR_PAIR(2));
		printw("requesting %d channels\n", ival);
		attroff(COLOR_PAIR(2));
		
		if (ival == 0) {
		  attron(COLOR_PAIR(1));
		  printw("wrong command? %s\n", cmdstring);
		  clrtoeol();
		  attroff(COLOR_PAIR(1));
		  print_help();
		  
		} else {
		  printw("updating number of channels\n");
		  sprintf(loginfo, "%s", cmdstring);
		  log_action(loginfo);
		  nch_prev = nch; // memory of the previous number of channels
		  nch = ival;
		  shm_setup(nch_prev);
		  allocated = 1;
		}
		cmdOK = 1;
      }

	// =====================================================
	//  returns the current number of available channels
	// =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "get_nch", strlen("get_nch")) == 0) {
		attron(COLOR_PAIR(4));
		printw("number of channels = %d\n", nch);
		clrtoeol();
		attroff(COLOR_PAIR(4));
		cmdOK = 1;
      }
	
	// =====================================================
	//       resets one or all channels (-1 for all)
	// =====================================================
    if (cmdOK == 0)
	  if (strncmp(cmdstring, "reset", strlen("reset")) == 0) {
		ival = 0; // reset value
		sscanf(cmdstring, "%s %d", str0, &ival);

		reset_channel(ival);
		cmdOK = 1;
	  }
	
	// =====================================================
	//             displays the help menu
	// =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "help", strlen("help")) == 0) {
		move(wysz-2,0); clrtoeol();
		move(wysz-1,0); clrtoeol();
		print_help();
		cmdOK = 1;
      }

	
	// =====================================================
    if (cmdOK == 0)
      if (strncmp(cmdstring, "quit", strlen("quit")) == 0) {

		attron(COLOR_PAIR(1) | A_BOLD);
		printw("HexDM shell closed (press key to continue)!\n");
		clrtoeol();
		attroff(COLOR_PAIR(1) | A_BOLD);
		getch();
		endwin(); // from curses back to regular env!
		
		if (shmarray != NULL) { // free the data structure
		  for (ii = 0; ii < nch + 1; ii++) {
			ImageStreamIO_destroyIm(&shmarray[ii]);
		  }
		  free(shmarray);
		  shmarray = NULL;
		}
		log_action("HexDM control program quit");

		if (NULL != sBMC) {
		  (void)BMCclose(sBMC);
		  printf("\nClosing DM link\n");
		}

		exit(0);
      }
	
	// =====================================================    
    if (cmdOK == 0) {
	  attron(COLOR_PAIR(1));
      printw("Unkown command: %s\n", cmdstring);
	  clrtoeol();
	  attroff(COLOR_PAIR(1));
	  
      print_help();
    }
	
	// -------------------------
	// end of command processing
	// -------------------------
  }
  // --------------------------
  //  clean ending the program
  // --------------------------
  exit(0);
}
