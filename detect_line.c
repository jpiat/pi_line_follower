

char line_detection_kernel [9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1} ;


typedef struct curve{
	float params [3];
};


void kernel_line(IplImage * img, char * kernel ,char * kernel_response, int v){
	unsigned int i ;
	for(i = 1 ; i < (img->width-1) ; i ++){
		kernel_response[i] = 0. ;
		
	}
}

int detect_line(IplImage * img, curve * c ){


}
