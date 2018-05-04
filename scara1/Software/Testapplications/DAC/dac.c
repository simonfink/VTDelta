#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <comedilib.h>

#define SLEEP_TIME 1
#define DEFAULT_DEV "/dev/comedi0"
#define DEFAULT_SUBDEV 1
#define DEFAULT_CHAN 0
#define DEFAULT_VAL 0

int main(int argc,char *argv[]) {
    comedi_t *it;
    char c;
    char* devName = DEFAULT_DEV;
    int subdev = DEFAULT_SUBDEV;
    int chan = DEFAULT_CHAN;
    lsampl_t data = DEFAULT_VAL;
    int retval;
    int n = 1;

    /* Compute command line arguments */
    while((c = getopt (argc, argv, "d:s:c:n:v:")) != -1) {
        switch(c) {
            case 'd': // device
                devName = optarg;
                break;
            case 's': // subdevice
                subdev = atoi(optarg);
                break;
            case 'c': // channel
                chan = atoi(optarg);
                break;
            case 'v': // value
                data = atoi(optarg);
                break;
            case 'n': // number of repeats
                n = atoi(optarg);
                break;
            case '?':
                if (optopt == 'd' || optopt == 's' || optopt == 'c') fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if(isprint(optopt)) fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                return -1;
            default:
                abort();
        }
    }

    /* open device */
    printf("Opening device: %s\n", devName);
    it = comedi_open(devName);
    if(it == NULL) {
        comedi_perror("comedi_open");
        return -1;
    }

    while(n > 0) {
        printf("Writing to DAC%d: %d\n", chan, data);
        retval = comedi_data_write(it, subdev, chan, 0, AREF_GROUND, data);
        if(retval < 0) {
            comedi_perror("comedi_data_write");
            return -1;
        }
        n--;
        sleep(SLEEP_TIME);
    }
    return 0;
}
