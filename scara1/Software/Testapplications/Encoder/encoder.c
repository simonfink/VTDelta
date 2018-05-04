#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <comedilib.h>

#define SLEEP_TIME 1
#define DEFAULT_DEV "/dev/comedi0"
#define DEFAULT_SUBDEV 11
#define DEFAULT_CHAN_A 8
#define DEFAULT_CHAN_B 10
#define DEFAULT_CHAN_I 9

int ni_gpct_start_encoder(comedi_t *device, unsigned subdevice, unsigned int initial_value, int a, int b, int z) {
    int retval;
    lsampl_t counter_mode;

    retval = comedi_reset(device, subdevice);

    /* set initial counter value by writing to channel 0 */
    retval = comedi_data_write(device, subdevice, 0, 0, 0, initial_value);
    
    /* set "load a" register to initial_value by writing to channel 1 */
    retval = comedi_data_write(device, subdevice, 1, 0, 0, initial_value);
    
    /* set "load b" register to initial_value by writing to channel 2 */
    retval = comedi_data_write(device, subdevice, 2, 0, 0, initial_value);

    comedi_set_gate_source(device, subdevice, 0, 0, NI_GPCT_DISABLED_GATE_SELECT);
    comedi_set_gate_source(device, subdevice, 0, 1, NI_GPCT_DISABLED_GATE_SELECT);
    /* note, the comedi_set_other_source calls will fail on 660x boards, since they
     * don't support user selection of the inputs used for the A/B/Z signals. */
    comedi_set_other_source(device, subdevice, 0, NI_GPCT_SOURCE_ENCODER_A, NI_GPCT_PFI_OTHER_SELECT(a));
    comedi_set_other_source(device, subdevice, 0, NI_GPCT_SOURCE_ENCODER_B, NI_GPCT_PFI_OTHER_SELECT(b));
    comedi_set_other_source(device, subdevice, 0, NI_GPCT_SOURCE_ENCODER_Z, NI_GPCT_PFI_OTHER_SELECT(z));

    counter_mode = (NI_GPCT_COUNTING_MODE_QUADRATURE_X4_BITS |
        NI_GPCT_COUNTING_DIRECTION_HW_UP_DOWN_BITS);
    if (z != NI_GPCT_DISABLED_GATE_SELECT) {
        counter_mode |= (NI_GPCT_INDEX_ENABLE_BIT |
            NI_GPCT_INDEX_PHASE_HIGH_A_HIGH_B_BITS);
    }
    retval = comedi_set_counter_mode(device, subdevice, 0, counter_mode);
    if(retval < 0) return retval;

    retval = comedi_arm(device, subdevice, NI_GPCT_ARM_IMMEDIATE);
    if(retval < 0) return retval;

    return 0;
}

int main(int argc,char *argv[]) {
    comedi_t *it;
    char c;
    char* devName = DEFAULT_DEV;
    int subdev = DEFAULT_SUBDEV;
    int chan_a = DEFAULT_CHAN_A;
    int chan_b = DEFAULT_CHAN_B;
    int chan_i = DEFAULT_CHAN_I;
    lsampl_t data;
    int retval;
    int n = 1;

    /* Compute command line arguments */
    while((c = getopt (argc, argv, "a:b:i:d:s:n:")) != -1) {
        switch(c) {
            case 'd': // device
                devName = optarg;
                break;
            case 's': // subdevice
                subdev = atoi(optarg);
                break;
            case 'a': // A channel
                chan_a = atoi(optarg);
                break;
            case 'b': // B channel
                chan_b = atoi(optarg);
                break;
            case 'i': // I channel
                chan_i = atoi(optarg);
                break;
            case 'n': // number of repeats
                n = atoi(optarg);
                break;
            case '?':
                if (optopt == 'd' || optopt == 's' || optopt == 'a' || optopt == 'b' || optopt == 'i' || optopt == 'n') fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if(isprint(optopt)) fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                return -1;
            default:
                abort();
        }
    }

    /* open device */
    printf("Opening device: %s\n!", devName);
    it = comedi_open(devName);
    if(it == NULL) {
        comedi_perror("comedi_open");
        return -1;
    }
    
    /* initialize */
    retval = ni_gpct_start_encoder(it, subdev, 0, chan_a, chan_b, chan_i);

    while(n > 0) {
        printf("Reading from Counter: ");
        retval = comedi_data_read(it, subdev, 0, 0, 0, &data);
        printf("%d\n", data);
        if(retval < 0) {
            comedi_perror("comedi_data_read");
            return -1;
        }
        n--;
        sleep(SLEEP_TIME);
    }
    return 0;
}
