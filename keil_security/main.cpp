#include "mbed.h"
#include "motordriver.h"

Serial pc(SERIAL_TX, SERIAL_RX);
Serial bt(PA_11, PA_12); 

DigitalOut r(A1);
DigitalOut g(PC_6);
DigitalOut b(A3);

Motor A(D11, PC_8); // pwm, dir

AnalogIn x_axis(PC_2);
AnalogIn y_axis(PC_3);
DigitalIn button(PA_14,PullUp);

DigitalOut myled(LED1);

DigitalOut Trig(D10);
DigitalIn Echo(D7);
DigitalOut buzzer(PA_13);

DigitalIn pir_sensor(D5);
DigitalOut GreenLed(D14); 
DigitalOut RedLed(D15); 

Timer t;

float i;
int x, y;

int state = 0;
int last_button_state = 1;
int button_state = 0;

int main() {
	
	t.start();
	
	pc.printf("\x1B\x48\x1B\x4A");
	pc.printf("---====[ Ultrsonic Range (SR04) ]===---");
		
	bt.baud(9600);

	//prints data on mobile
	bt.printf("Connection Established");

	//print data on pc terminal
	pc.printf("Connection Established");
	
	int pir_value = 0;
	
	while(1) {
		
		/* joystick, bluetooth, UART to Motor */
		
		x = x_axis.read() *1000;
		y = y_axis.read() *1000;
		
		if (bt.readable()) {
			char input_key=  bt.putc(bt.getc());
			
			if(input_key == 'o') {
				myled = 1;
				//bt.printf("Open the window");
				A.forward(0.4f);
				wait_ms(2000);
				A.stop();
			}

			if(input_key == 'c') {
				myled = 0;
				//bt.printf("Close the window");
				A.backward(0.4f);
				wait_ms(2000);
				A.stop();
			}
		}

			
		if (pc.readable()) {
			char input_key=  pc.putc(pc.getc());
			
			if(input_key == 'o') {
				pc.printf("Open the window");
			}
			
			if(input_key == 'c') {
				pc.printf("Close the window");
			}
		}
		if(state%2 == 0) {		
			if (y<10){
				A.forward(0.4f);
			}
			
			else if (y>900){
				A.backward(0.4f);
			}
		}

		if (x>450 && x<550 && y>400 && y<600){
			A.stop();
		}
				
		/* ultra sonic to Buzzer */
			
		Trig = 1;
		wait(0.00004);
		Trig = 0;
		int cnt=0;	 
		while(!Echo);
					
		t.reset(); 
		while(Echo);
		i = t.read_us();

		//pc.printf("\x1B\x48");
		//pc.printf("\n\n\rPulselength %6.0f uS", i);
		i = i/58;
		//pc.printf("\n\n\rDistance %4.0f cm", i);
		r=0;
		g=1;
		b=0;
		
		if(state%2 == 1) {
			r=0;
			g=0;
			b=0;
		}
		
		
		if(i<10){					
			r=1;
			g=0;
			b=0;
			
			if(state%2 == 1) {
				r=0;
				g=0;
				b=0;
			}
			
			while(cnt < 500){
				if(state%2 == 0) {
					buzzer=1;
					wait(0.001);
					buzzer=0;
					wait(0.001);
				}
					cnt++;
			}
		}
		
		/* pir sensor to led */ 
		pir_value = pir_sensor.read(); 
		if(pir_value == 1) {
			RedLed = 1;
			GreenLed = 0;
			
			if(state%2 == 1) {
				RedLed = 0;
				GreenLed = 0;
			}
			
		}
		else {
			GreenLed = 1;
			RedLed = 0;
			
			if(state%2 == 1) {
				RedLed = 0;
				GreenLed = 0;
			}
			
		}
		
		/* button to control system */
		button_state = button.read();
		if(last_button_state == 1 && button_state == 0) {
			state++;
		}
		last_button_state = button_state;
		
		
	}								
} 


