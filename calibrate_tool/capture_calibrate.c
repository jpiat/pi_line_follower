/*

 Copyright (c) by Emil Valkov,
 All rights reserved.

 License: http://www.opensource.org/licenses/bsd-license.php

*/

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "RaspiCamCV.h"
#include <pthread.h>
#include <signal.h>
#include <sys/resource.h>

#include <jpeglib.h>
#include <jerror.h>
#include <wiringPi.h>

#define TRIGGER_PIN 17

char path [128];
char * path_fmt = "%s/image_%04d.pgm";
char path_base [128];
char image_path [256];
RaspiCamCvCapture * capture ;

unsigned char * jpeg_buffer ;
long unsigned int jpeg_length ;

int pin_state = 0 ;

void writePGM(const char *filename, IplImage * img)
{
    FILE *pgmFile;
    int i, j;
    int hi, lo;
    pgmFile = fopen(filename, "wb");
    if (pgmFile == NULL) {
        perror("cannot open file to write");
        exit(EXIT_FAILURE);
    }
    fprintf(pgmFile, "P5 \n");
    //TODO : insert data as comments ... Maybe use Json file format
   /* if(comment != NULL){
    	fprintf(pgmFile, "#%s \n", comment);
    }*/
    fprintf(pgmFile, "%d %d \n%d \n", img->width, img->height, 255);
    fwrite(img->imageData, 1, img->height*img->width, pgmFile);
    //fflush(pgmFile);
    fclose(pgmFile);
}

int write_jpegmem(char * frame, unsigned short width, unsigned short height, unsigned short step, unsigned short nbChannels, unsigned char **outbuffer, long unsigned int *outlen, int quality)
{
  JSAMPROW row_ptr[1];
  struct jpeg_compress_struct jpeg;
  struct jpeg_error_mgr jerr;
  char *line, *image;
  int y, x, line_width;
  *outbuffer = NULL;
  *outlen = 0;
  line =(char *) malloc((width * nbChannels) * sizeof(char));
  if (!line)
    return 0;
  jpeg.err = jpeg_std_error (&jerr);
  jpeg_create_compress (&jpeg);
  jpeg.image_width = width;
  jpeg.image_height= height;
  jpeg.input_components = nbChannels; 
  if(nbChannels == 1){
	jpeg.in_color_space =JCS_GRAYSCALE;
   }else{
        jpeg.in_color_space = JCS_RGB;
   }
  jpeg_set_defaults (&jpeg);
  jpeg_set_quality (&jpeg, quality, TRUE);
  jpeg.dct_method = JDCT_FASTEST;
  jpeg_mem_dest(&jpeg, outbuffer, outlen);
  jpeg_start_compress (&jpeg, TRUE);
  row_ptr[0] = (JSAMPROW) line;
  line_width = width * nbChannels;
  for (y = 0; y < height; y++) {
    for (x = 0; x < line_width; x++) {
      line[x]   = frame[x];
    }
    if (!jpeg_write_scanlines (&jpeg, row_ptr, 1)) {
      jpeg_destroy_compress (&jpeg);
      free (line);
      return 0;
    }
    frame += step;
  }
  jpeg_finish_compress (&jpeg);
  jpeg_destroy_compress (&jpeg);
  free (line);
  return 1;
}

void acq_image_thread_func(){
	int i = 0 ;
	int image_index = 0;
	char image_path [256];
    	unsigned long t_start, t_stop ;
	unsigned long t_diff ;

	RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
        RASPIVID_PROPERTIES * properties = (RASPIVID_PROPERTIES*)malloc(sizeof(RASPIVID_PROPERTIES));
        config->width=640;
        config->height=480;
        config->bitrate=0;      // zero: leave as default
        config->framerate=30;
        config->monochrome=1;
	properties->hflip = 0 ;
        properties->vflip = 0 ;
        properties -> sharpness = 0 ;
        properties -> contrast = 0 ;
        properties -> brightness = 50 ;
        properties -> saturation = 0 ;
        properties -> exposure = SPORTS;
        properties -> shutter_speed = 0 ; // 0 is autoo
	//printf("Init sensor \n");
        capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture3(0, config, properties, 1);
	free(config);
	//printf("Start Capture !\n");
	while(1){
                int success = 0 ;
                success = raspiCamCvGrab(capture);
                if(success){
                                IplImage* image = raspiCamCvRetrieve(capture);
				write_jpegmem(image->imageData, image->width, image->height,  image->widthStep, image->nChannels, &jpeg_buffer, &jpeg_length, 70);
				write(1,jpeg_buffer,jpeg_length);
				free(jpeg_buffer);
				if(digitalRead(TRIGGER_PIN) == 1 && pin_state == 0 ) { //Rising edge ends program
                        		sprintf(image_path, "%s/%04d.pgm", path_base, image_index);
					writePGM(image_path, image);
					image_index ++ ;
				}
				pin_state = digitalRead(TRIGGER_PIN);
                }
        }
        raspiCamCvReleaseCapture(&capture);
}


int main(int argc, char *argv[]){
	char line_buffer[128] ;
	int frame_index = -1 ;
	if(argc == 1){
		printf("Usage : capture_calibrate <capture folder> \n");
		exit(0);
	}
	if(argc > 1){
		sprintf(path_base, "%s", argv[1]);
	}else{
       		sprintf(path_base, "./");
	}
	wiringPiSetupSys();
        /*pinMode (0, INPUT);//setup pin 0 to trigger capture
        pullUpDnControl(0, PUD_UP);*/
        pin_state = digitalRead(TRIGGER_PIN);
	acq_image_thread_func();
	return 1 ;
}
