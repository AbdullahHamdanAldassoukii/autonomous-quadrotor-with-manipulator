#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include "simple_eqep.h"

#define EQEP_MODE_REL 1
#define EQEP_MODE_ABS 0

const char* eqep_path[3] = {
    "/sys/devices/platform/ocp/48300000.epwmss/48300180.eqep", \
    "/sys/devices/platform/ocp/48302000.epwmss/48302180.eqep", \
    "/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep" };

int simple_init_eqep(int eQEPnum){
    if((eQEPnum > 2) | (eQEPnum < 0)){
        printf("ERROR: Invalid eQEP");
        return -1;
    }

    simple_write_eqep(eQEPnum, 0);
    return 0;
}

int simple_write_eqep(int eQEPnum, int position){
    FILE *fp;
    char fname[60];
    sprintf(fname,"%s/position", eqep_path[eQEPnum]);
    fp = fopen(fname, "w");
    fprintf(fp, "%d\n", position);
    fclose(fp);
    return 0;
}

int simple_set_period_eqep(int eQEPnum, uint64_t period){
    FILE *fp;
    char fname[60];
    sprintf(fname,"%s/period", eqep_path[eQEPnum]);
    fp = fopen(fname, "w");
    fprintf(fp, "%"PRId64"\n", period);
    fclose(fp);
    return 0;
}

int simple_set_mode_eqep(int eQEPnum, int mode){
    FILE *fp;
    char fname[60];
    sprintf(fname,"%s/mode", eqep_path[eQEPnum]);
    fp = fopen(fname, "w");
    
    if(mode == EQEP_MODE_REL){
        fprintf(fp, "%d\n", EQEP_MODE_REL);
    }
    else if(mode == EQEP_MODE_ABS) {
        fprintf(fp, "%d\n", EQEP_MODE_ABS);
    }
    else {
        printf("ERROR: Invalid Mode\n");
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

int simple_read_eqep(int eQEPnum){
    int position;
    FILE *fp;
    char fname[60];
    sprintf(fname,"%s/position", eqep_path[eQEPnum]);
    fp = fopen(fname, "r");
    fscanf(fp, "%d", &position);
    fclose(fp);
    return position;
}

uint64_t simple_get_period_eqep(int eQEPnum){
    uint64_t period;
    FILE *fp;
    char fname[60];
    sprintf(fname,"%s/period", eqep_path[eQEPnum]);
    fp = fopen(fname, "r");
    fscanf(fp, "%"PRId64"", &period);
    fclose(fp);
    return period;
}

int simple_get_mode_eqep(int eQEPnum){
    int mode;
    FILE *fp;
    char fname[60];
    sprintf(fname,"%s/mode", eqep_path[eQEPnum]);
    fp = fopen(fname, "r");
    fscanf(fp, "%d", &mode);
    fclose(fp);
    return mode;
}