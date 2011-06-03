#include <stdio.h>   /* for printf() */
#include <comedilib.h>

//gcc testdevice.c -lcomedi -lm

int main(int argc,char *argv[])
{
	comedi_t *it;
	lsampl_t data;
	int res;
	unsigned int maxdata, rangetype;
	unsigned int ddata;
	int chan = 1;
	int range = 2;
	int subdevice = 0;
	int aref = AREF_GROUND;

	it=comedi_open("/dev/comedi0");

	maxdata = comedi_get_maxdata(it, 0, chan);
	rangetype = comedi_get_rangetype(it, 0, chan);
	printf("maxdata %d and rangetype %d\n", maxdata, rangetype);

	/* analog input */
	subdevice = 0; chan = 0; range = 2;
	res = comedi_data_read(it, subdevice, chan, range, aref, &data);
	printf("analog input data for channel %d: %d (%d)\n", chan, data, res);

	/* analog output */
	subdevice = 1; chan = 0; range = 0; data = 2000;
	res = comedi_data_write(it, subdevice, chan, range, aref, data);
	printf("analog output data for channel %d: %d (%d)\n", chan, data, res);

	/* digital input */
	subdevice = 2; chan = 12;
	res = comedi_dio_read(it, subdevice, chan, &ddata);
	printf("digital input data for channel %d: %d (%d)\n", chan, ddata, res);

	/* digital output */
	subdevice = 3; chan = 2; ddata = 0; 
	res = comedi_dio_write(it, subdevice, chan, ddata);
	printf("digital output for channel %d: %d (%d)\n", chan, ddata, res);

	return 0;
}
