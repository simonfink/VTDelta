#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <comedilib.h>

#define SLEEP_TIME 1
#define READ 0
#define WRITE 1
#define HIGH 1
#define LOW 0
#define UNDEFINED -1
#define DEFAULT_DEV "/dev/comedi0"
#define DEFAULT_SUBDEV 2
#define DEFAULT_CHAN 0

int main(int argc,char *argv[]) {
    comedi_t *it;
    char c;
    int action = UNDEFINED;
    char* devName = DEFAULT_DEV;
    int subdev = DEFAULT_SUBDEV;
    int chan = DEFAULT_CHAN;
    lsampl_t data = UNDEFINED;
    int retval;
    int n = 1;

    /* Compute command line arguments */
    while((c = getopt (argc, argv, "rwhld:s:c:n:")) != -1) {
        switch(c) {
            case 'r': // read
                if(action != -1) {
                    printf("Warning: Use -r OR -w, but not both.");
                }
                action = READ;
                break;
            case 'w': // write
                if(action != -1) {
                    printf("Warning: Use -r OR -w, but not both.");
                }
                action = WRITE;
                break;
            case 'h': // high
                if(data != -1) {
                    printf("Warning: Use -l OR -h, but not both.");
                }
                data = HIGH;
                break;
            case 'l': // low
                if(data != -1) {
                    printf("Warning: Use -l OR -h, but not both.");
                }
                data = LOW;
                break;
            case 'd': // device
                devName = optarg;
                break;
            case 's': // subdevice
                subdev = atoi(optarg);
                break;
            case 'c': // channel
                chan = atoi(optarg);
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

    /* configure GPIO */
    if(action == READ) {
        printf("Configuring GPIO%d as input\n", chan);
        comedi_dio_config(it, subdev, chan, COMEDI_INPUT);
        
    }
    else if(action == WRITE) {
        printf("Configuring GPIO%d as output\n", chan);
        comedi_dio_config(it, subdev, chan, COMEDI_OUTPUT);
        
    }
    else {
        printf("Missing argument: use -r for read or -w for write\n");
        return -1;
    }
    while(n > 0) {
        if(action == WRITE) {
            printf("Writing to GPIO%d: %d\n", chan, data);
            retval = comedi_dio_write(it, subdev, chan, data);
        }
        else {
            printf("Reading from GPIO%d: ", chan);
            retval = comedi_dio_read(it, subdev, chan, &data);
            printf("%d\n", data);
        }
        if(retval < 0) {
            comedi_perror("comedi_dio_read/comedi_dio_write");
            return -1;
        }
        n--;
        sleep(SLEEP_TIME);
    }
    return 0;
}
