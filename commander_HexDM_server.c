/* =========================================================================
 * BMC Hex DM control server - Frantz Martinache
 * 
 * The program mostly handles the creation, update and destruction of several
 * shared memory data structures (ImageStreamIO library by O. Guyon), that
 * are refered to as channels.
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
#include <unistd.h>

#include <commander/commander.h>
#include <ImageStreamIO.h>

#include <BMCApi.h>  // the new API for the HexDM

/* =========================================================================
 *                            Global variables
 * ========================================================================= */
#define LINESIZE 256
#define CMDSIZE 200

int ii;                  // dummy index value
IMAGE *shmarray = NULL;  // shared memory img pointer (defined in ImageStreamIO.h)
int nch         = 4;     // number of DM channels (default = 4)
int nseg        = 169;   // number of segments on the DM
int ndof        = 3;     // number of d.o.f per segment (piston, tip & tilt)
int nvact = ndof * nseg; // number of voltage actuators
int csz         = 1024;  // size of the command expected by the driver

int keepgoing   = 0;     // flag to control the DM update loop
int allocated   = 0;     // flag to control whether shm structures are allocated
int nch_prev    = 0;     // keep track of the # of channels before a change
char dashline[80] =
  "-----------------------------------------------------------------------------\n";

int ndm = 1; // the number of DMs to be connected
DM *hdm;            // the handles for the different deformable mirrors
BMCRC rv;           // result of every interaction with the driver (check status)
uint32_t *map_lut;  // the DM actuator mappings

int simmode = 1;  // flag to set to "1" to not attempt to connect to the driver
char drv_status[8] = "idle"; // to keep track of server status

const char *snumber = "27BW007#051";  // our DM identifier

pthread_t tid_loop;      // thread ID for DM control loop

/* =========================================================================
 *                       function prototypes
 * ========================================================================= */
int shm_setup();
void* dm_control_loop(void *dummy);
void MakeOpen(DM* hdm);
double* ptt_2_actuator(double* ptt);

/* =========================================================================
 *                           DM setup function
 * ========================================================================= */
void MakeOpen(DM* hdm) {
  memset(hdm, 0, sizeof(DM));
  printf("Attempting to open device %s\n", snumber);
  rv = BMCOpen(hdm, snumber);
  
  if (rv) {
    printf("Error %d opening the driver type %u.\n", rv, (unsigned int)hdm->Driver_Type);
    printf("%s\n\n", BMCErrorString(rv));

    printf("Press any key to exit.\n");
    getc(stdin);
    exit(0);
  }
  printf("Opened Device %d with %d actuators.\n", hdm->DevId, hdm->ActCount);
  
  rv = BMCLoadMap(hdm, NULL, map_lut);  // load the mapping into map_lut
}

/* =========================================================================
 *    conversion from PTT commands to actuator command for the driver
 * 
 * expects the 3 column ptt argument to consist in:
 * - piston values (in nanometers)
 * - tip and tilt values (in mrad)
 * ========================================================================= */
double* ptt_2_actuator(double* ptt) {
  int ii, ii0;           // dummy variables
  int csz = 1024;        // size of the command expected by the driver
  double again = 4000.0; // actuator gain: 4 um per ADU ? To be refined
  double a0 = 218.75;    // actuator location radius in microns
  double sq3_2 = 0;      // short hand for sqrt(3) / 2
  double *res = (double*) malloc(csz * sizeof(double));

  sq3_2 = sqrt(3.0)/2.0;
  for (ii = 0; ii < nseg; ii++) {
    ii0 = ii * ndof;
    res[ii0]   = ptt[ii0] + a0 * sq3_2 * ptt[ii0+1] + a0/2.0 * ptt[ii0+2];
    res[ii0+1] = ptt[ii0] - a0 * ptt[ii0+2];
    res[ii0+2] = ptt[ii0] - a0 * sq3_2 * ptt[ii0+1] + a0/2.0 * ptt[ii0+2];
  }
  
  for (ii = 0; ii < nvact; ii++)
    res[ii] /= again;
  return res;			  
}

/* =========================================================================
 *      Allocates shared memory data structures for the new settings
 * ========================================================================= */
int shm_setup() {
  int ii;
  int shared = 1;
  int NBkw = 10;
  long naxis = 2;
  uint8_t atype = _DATATYPE_DOUBLE;
  uint32_t imsize[2] = {(uint32_t)ndof, (uint32_t)nseg};
  char shmname[20];

  if (shmarray != NULL) { // structure must be freed before reallocation!
    for (ii = 0; ii < nch_prev; ii++) 
      ImageStreamIO_destroyIm(&shmarray[ii]);
    free(shmarray);
    shmarray = NULL;
  }

  // allocates nch arrays (+ 1 for the combined channel)
  shmarray = (IMAGE*) malloc((nch+1) * sizeof(IMAGE));

  // individual channels
  for (ii = 0; ii < nch; ii++) {
    sprintf(shmname, "ptt%02d", ii); // root name of the shm
    ImageStreamIO_createIm_gpu(&shmarray[ii], shmname, naxis, imsize, atype, -1,
			       shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);
  }
  // the combined array
  sprintf(shmname, "ptt");           // root name of the shm
  ImageStreamIO_createIm_gpu(&shmarray[nch], shmname, naxis, imsize, atype, -1,
			     shared, IMAGE_NB_SEMAPHORE, NBkw, MATH_DATA);

  return 0;
}

/* =========================================================================
 *                     DM surface control thread
 * ========================================================================= */
void* dm_control_loop(void *dummy) {
  uint64_t cntrs[nch];
  int ii, kk;  // array indices
  double *cmd; //
  double tmp_map[nvact];  // to store the combination of channels

  for (ii = 0; ii < nch; ii++)
    cntrs[ii] = shmarray[ii].md->cnt0;  // init shm counters

  while (keepgoing > 0) {

    ImageStreamIO_semwait(&shmarray[nch], 1);  // waiting for a DM update!

    for (ii = 0; ii < nch; ii++)
      cntrs[ii] = shmarray[ii].md->cnt0; // update counter values

    // -------- combine the channels -----------
    for (ii = 0; ii < nvact; ii++) {
      tmp_map[ii] = 0.0; // init temp sum array
      for (kk = 0; kk < nch; kk++) {
	tmp_map[ii] += shmarray[kk].array.D[ii];
      }
    }

    // ------- update the shared memory ---------
    shmarray[nch].md->write = 1;   // signaling about to write
    for (ii = 0; ii < nvact; ii++) // update the combined channel
      shmarray[nch].array.D[ii] = tmp_map[ii];
    shmarray[nch].md->cnt1 = 0;
    shmarray[nch].md->cnt0++;
    shmarray[nch].md->write = 0;  // signaling done writing

    // ------ converting into a command the driver --------
    // sending to the DM
    if (simmode != 1) {
      cmd = ptt_2_actuator(shmarray[nch].array.D);

      // !!!! ensure values are within acceptable range: TBD !!!!!

      rv = BMCSetArray(hdm, cmd, map_lut);  // send cmd to DM
      if (rv) {
	printf("%s\n\n", BMCErrorString(rv));
      }
      free(cmd);
    }
  }
  return NULL;
}

/* =========================================================================
 *            Functions registered with the commander server
 * ========================================================================= */

void start() {
  /* -------------------------------------------------------------------------
   *          Starts the monitoring of shared memory data structures
   * ------------------------------------------------------------------------- */
  if (keepgoing == 0) {
    keepgoing = 1; // raise the flag
    printf("DM control loop START\n");
    pthread_create(&tid_loop, NULL, dm_control_loop, NULL);
  }
  else
    printf("DM control loop already running!\n");
  sprintf(drv_status, "%s", "running");
}

void stop() {
  /* -------------------------------------------------------------------------
   *            Stops the monitoring of shared memory data structures
   * ------------------------------------------------------------------------- */
  if (keepgoing == 1)
    keepgoing = 0;
  else
    printf("DM control loop already off\n");
  sprintf(drv_status, "%s", "idle");
}

std::string status() {
  /* -------------------------------------------------------------------------
   *                          Returns server status
   * ------------------------------------------------------------------------- */
  return drv_status;
}

int get_nch() {
  /* -------------------------------------------------------------------------
   *               Returns the number of virtual channels per DM
   * ------------------------------------------------------------------------- */
  return nch;
}

void set_nch(int ival) {
  /* -------------------------------------------------------------------------
   *               Updates the number of virtual channels per DM
   * ------------------------------------------------------------------------- */
  nch_prev = nch; // memory of the previous number of channels
  nch = ival;
  shm_setup();
  printf("Success: # channels = %d\n", ival);
}

void reset(int channel) {
  /* -------------------------------------------------------------------------
   *                     Resets a DM channel (or all)
   * ------------------------------------------------------------------------- */

  double reset_map[nvact] = {0};  // reset command map 
  double *live_channel;
  int kk;

  if (channel < 0) {
    for (kk = 0; kk < nch; kk++) {
      live_channel = shmarray[kk].array.D;  // live pointer
      shmarray[kk].md->write = 1;           // signaling about to write
      memcpy(live_channel, (double *) reset_map, sizeof(double) * nvact);
      shmarray[kk].md->cnt0++;
      ImageStreamIO_sempost(&shmarray[kk], -1);
      shmarray[kk].md->write = 0;   // done writing
    }
  }
  else if (channel < nch) {
    kk = channel;
    live_channel = shmarray[kk].array.D;  // live pointer
    shmarray[kk].md->write = 1;           // signaling about to write
    memcpy(live_channel, (double *) reset_map, sizeof(double) * nvact);
    shmarray[kk].md->cnt0++;
    ImageStreamIO_sempost(&shmarray[kk], -1);
    shmarray[kk].md->write = 0;   // done writing
  }
  else
    printf("Virtual channels 0-%d have been set-up!\n", nch);
}

void quit() {
  /* -------------------------------------------------------------------------
   *                       Clean exit of the program.
   * ------------------------------------------------------------------------- */
  if (keepgoing == 1) stop();
  
  printf("DM driver server shutting down!\n");
  
  if (simmode != 1) {
    rv = BMCClearArray(hdm);
    if (rv) {
      printf("%s\n\n", BMCErrorString(rv));
      printf("Error %d clearing voltages.\n", rv);
    }
    
    rv = BMCClose(hdm);
    if (rv) {
      printf("%s\n\n", BMCErrorString(rv));
      printf("Error %d closing the driver.\n", rv);
    }
    printf("%s\n\n", BMCErrorString(rv));
  }
  free(map_lut);

  if (shmarray != NULL) { // free the data structure
    for (int ii = 0; ii < nch + 1; ii++) {
      ImageStreamIO_destroyIm(&shmarray[ii]);
    }
    free(shmarray);
    shmarray = NULL;
  }
  exit(0);
}

namespace co=commander;

COMMANDER_REGISTER(m) {
  using namespace co::literals;
  m.def("start", start, "Starts monitoring shared memory data structures.");
  m.def("stop", stop, "Stops monitoring shared memory data structures.");
  m.def("status", status, "Returns status of the DM server.");
  m.def("get_nch", get_nch, "Returns the number of virtual channels per DM.");
  m.def("set_nch", set_nch, "Updates the number of virtual channels per DM.");
  m.def("reset", reset, "Resets DM channel #arg_0 (all if arg_0=-1).");
}

/* =========================================================================
 *                                Main program
 * ========================================================================= */
int main(int argc, char **argv) {

  hdm = (DM *) malloc(sizeof(DM));
  map_lut = (uint32_t *) malloc(sizeof(uint32_t)*MAX_DM_SIZE);

  if (simmode != 1) 
    MakeOpen(hdm);
  else {
    printf("Simulated DM scenario: the driver is not connected\n");
    printf("Simulated DM - serial number = %s.\n", snumber);
  }
  shm_setup();  // set up startup configuration
 

  // --------------------- set-up the prompt --------------------
  printf("%s", dashline);
  printf(" ____  _           _              _                ____  __  __ \n");
  printf("|  _ \\| |__   ___ | |_ ___  _ __ (_) ___ ___      |  _ \\|  \\/  |\n");
  printf("| |_) | '_ \\ / _ \\| __/ _ \\| '_ \\| |/ __/ __|_____| | | | |\\/| |\n");
  printf("|  __/| | | | (_) | || (_) | | | | | (__\\__ \\_____| |_| | |  | |\n");
  printf("|_|   |_| |_|\\___/ \\__\\___/|_| |_|_|\\___|___/     |____/|_|  |_|\n");
  printf("%s", dashline);

  // start the commander server
  co::Server s(argc, argv);
  s.run();
  
  // --------------------------
  //  clean ending the program
  // --------------------------
  quit();
}
