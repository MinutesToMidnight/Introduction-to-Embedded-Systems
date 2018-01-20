# Course Workbook

MinutesToMidnight

Now with more fun in every byte!

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

PRODUCT AND COMPANY NAMES MENTIONED HEREIN MAY BE THE TRADEMARKS OF THEIR RESPECTIVE OWNERS. TRADEMARK SYMBOLS MAY NOT APPEAR WITH EVERY OCCURRENCE OF A TRADEMARKED NAME, HOWEVER THIS DOES NOT CONSTITUTE INTENTION OF INFRINGEMENT ON THE TRADEMARK.

- [exercise-one: hello-arduino_b0001](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-one---hello-arduino---b0001)
- [exercise-two: led-there-be-light_b0010](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-two---led-there-be-light---b0010)
- [exercise-three: make-some-noise_b0011](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-three---make-some-noise---b0011)
- [exercise-four: sound-smart_b0100](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-four---sound-smart---b0100)
- [exercise-five: push-a-button_b0101](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-five---push-a-button---b0101)
- [exercise-six: detect-body-heat-in-motion_b0110](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-six---detect-body-heat-in-motion---b0110)
- [exercise-seven: usb-for-fun-and-profit_b0111](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-seven---usb-for-fun-and-profit---b0111)
- [exercise-eight: look-inside-a-wire_b1000](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-eight---look-inside-a-wire---b1000)
- [exercise-nine: hear-the-light_b1001](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-nine---hear-the-light---b1001)
- [exercise-ten: reading-lcd-leaves_b1010](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-ten---reading-lcd-leaves---b1010)
- [exercise-eleven: pet-an-electron_b1011](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-eleven---pet-an-electron---b1011)
- [exercise-twelve: how-fast_b1100](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-twelve---how-fast---b1100)
- [exercise-thirteen: flying-circus_b1101](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-thirteen---flying-circus---b1101)
- [exercise-fourteen: serial-servo_b1110](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-fourteen---serial-servo---b1110)
- [exercise-fifteen: stream-cypher_b1111](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#exercise-fifteen---stream-cypher---b1111)
- [Further Reading List](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#further-reading-list)
- [Unofficial Kit](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/README.md#unofficial-kit)

## Exercise One - Hello Arduino - B0001

### Making a Piezo Beep Out a Chip Tune

This exercise will walk you through the basics of using the Arduino software. You will learn fundamentals of using the IDE that you will use as a basis for the following exercises. There is a familiar tune that will play at the end if everything is working.

### Setup for the Exercise

If this is your first time using the Arduino software, please follow the software install guide: <https://www.arduino.cc/en/Guide/HomePage>

- Connect the Piezo speaker, as shown below.
- Hardware diagrams generated using Fritzing. ![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Speaker.png?raw=true)

  ### [Code](https://github.com/MinutesToMidnight/Chip_Tune_Arduino/blob/master/Chip_Tune_Arduino.ino)

## Exercise Two - LED There Be Light - B0010

### Blinking the Integrated LED

In this exercise, you will learn how to blink the integrated LED on pin 13 of the Arduino board and the difference between Input / Output pins and how to interface with them through code. Arduinos have what are called General Purpose Input / Output pins, commonly referred to as GPIO. These pins have three main modes (tri-state). They can be:

1. A high impedance input
2. A low impedance output
3. A high impedance disconnected state (sometimes called "Z" or "NC")

There is also another mode often used for I2C buses and push button inputs. It is an input mode, with an internal resistor pulling the input weakly to a logic high voltage. This is handy when we are trying to keep the external components to a minimum in our projects. More on this later... On the Arduino Uno, there is an integrated LED wired into pin 13\. It is configured in such a way as to not affect the pin 13 current limits, due to an integrated buffer circuit. Note: that the Arduino Uno board has both per pin and total micro-controller IC current limits. Sometimes the source and sink limits are not the same, to further complicate things. This is not a deal breaker for high current loads, it only means you have to use a transistor, relay, buffer, or some other component in series to achieve the desired arbitrary current. Here is a quick reference diagram from: <http://playground.arduino.cc/Main/ArduinoPinCurrentLimitations>

![](http://arduino-info.wikispaces.com/file/view/ArduinoPinCurrentLimits.jpg/230512334/ArduinoPinCurrentLimits.jpg)

While you can control the duty cycle by manually setting on/off delay statements in software, it waists processor time. Therefore, it is often better to let the built in hardware control the duty cycle using PWM on analog output pins (The analog output capable pins are marked with a "~" next to the pin number on the board). We will discuss this in more detail in a later section. When you want to change the state of an output pin, you write to a special location in memory that is wired to the output pin. This special memory location is called an "Output Register". The code for this exercise uses the following functions:

- pinMode(Pin_number, OUTPUT); // sets the named pin to be a low impedance output.
- digitalWrite(Pin_number, HIGH); // sets the named pin to HIGH (5 volts), using LOW sets it to 0 volts.
- delay(1000); // causes the Arduino to wait for 1000 milliseconds (1 second). Note: there is also a delay function that waits in microsecond intervals too: delayMicroseconds().

  ### Setup for the Exercise

  ![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Uno.png?raw=true)

  ### Code:

  ```
    #define LED_Pin 13

    // the setup function runs once when you press reset or power the board
    void setup()
    {
      pinMode(LED_Pin, OUTPUT);
    }

    // the loop function runs over and over again forever
    void loop()
    {
      digitalWrite(LED_Pin, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_Pin, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);                       // wait for a second
    }
  ```

## Exercise Three - Make Some Noise - B0011

### Random Tune

In this exercise, you will learn how to use the piezo speaker to generator noise using the Tone function.

Syntax:

- random(min, max)
- tone(pin, frequency)
- tone(pin, frequency, duration)

Parameters:

- pin: unsigned integer; the pin on which to generate the tone
- frequency: unsigned integer; the frequency of the tone in Hertz (cycles / sec)
- duration: unsigned long; the duration of the tone in milliseconds (optional)

The tone function generates a square-wave of the specified frequency with 50% duty cycle (ratio of time the signal is low to time the signal is high) on the specified pin location. A duration can be specified, otherwise the square-wave continues until a call to the noTone() function. The pin can be connected to the included piezo speaker or other speaker to play tones.

Only one tone can be generated at a time. If a tone is already playing on a different pin, the call to tone() will have no effect. If the tone is playing on the same pin, the call will set its frequency.

There are some issues related to the implementation of the "duration" parameter, so it is best to control the duration of the tone using the delay() function or through some other method.

Note: Clicking on the check mark compiles the project, but does not upload it to the Arduino board (This is good for using the error checker built into the GCC compiler). Clicking on the run arrow compiles and then uploads the project if there are no compiler errors.

Note: Pressing CTRL and T simultaneously in the editor automatically cleans up the formatting. It is a good practice to use a #define statement for constant assignments like pin numbers. The #defined NAME VALUE, does not need an equal sign or semicolon - as it is a compiler pre-processor directive. Every time you use NAME in your code it is automatically replaced with VALUE by the compiler. This allows for values to be edited later in one convenient location.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Speaker.png?raw=true)

### Code:

```
    #define speaker_pin 10
    //the setup function runs once when you press reset or power the board
    void setup()
    {
      pinMode(speaker_pin, OUTPUT);
    }

    //the loop function runs over and over again
    void loop()
    {
      for(int i = 64; i<180; i++)
      {
      tone(speaker_pin, random(64, 640)); //Play a pseudo random tone
      delay(i);
      }

      for(int i = 180; i>64; i--)
      {
      tone(speaker_pin, random(64, 640)); //Play a pseudo random tone
      delay(i);
      }
    }
```

Bonus awesome points for using one of these

- [Make it talk on pin 3](https://github.com/MinutesToMidnight/Talkie)

## Exercise Four - Sound Smart - B0100

### Morse Code

This exercise will show you how to turn your Arduino into a Morse Code blinking and beeping machine! We will combine our blinking and beeping skills to accomplish this. Note the alphabet lookup table implemented with the switch case statements.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Speaker.png?raw=true)

### [Code](https://github.com/MinutesToMidnight/Morse_Code_Arduino/blob/master/Morse_Code_Arduino.ino)

## Exercise Five - Push a Button - B0101

### The Infamous Blue Box

In this exercise, you will learn about pull-up and pull-down resistors and how they can be used with the Arduino board to prevent a push button switch input from floating.

Back in the days of in band PBX signaling, this could get you free long distance.... wink wink... <https://en.wikipedia.org/wiki/Blue_box>

Below is a note from: <https://learn.sparkfun.com/tutorials/pull-up-resistors>

What is a Pull-up Resistor Let's say you have an MCU with one pin configured as an input. If there is nothing connected to the pin and your program reads the state of the pin, will it be high (pulled to VCC) or low (pulled to ground)? It is difficult to tell. This phenomena is referred to as "floating". To prevent this unknown state, a pull-up or pull-down resistor will ensure that the pin is in either a high or low state, while also using a low amount of current. For simplicity, we will focus on pull-ups since they are more common than pull-downs. They operate using the same concepts, except the pull-up resistor is connected to the high voltage (this is usually 3.3v or 5v and is often referenced to as VCC) and the pull-down resistor is connected to ground. Pull-ups are often used with buttons and switches.

![](https://cdn.sparkfun.com/assets/6/f/b/c/7/511568b6ce395f1b40000000.jpg)

With a pull-up resistor, the input pin will read a high state when the button is not pressed. In other words, a small amount of current is flowing between VCC and the input pin (not to ground), thus the input pin reads close to VCC. When the button is pressed, it connects the input pin directly to ground. The current flows through the resistor to ground, thus the input pin reads a low state. Keep in mind, if the resistor wasn't there, your button would connect VCC to ground, which is very bad and is also known as a short.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Speaker-and-Button.png?raw=true)

Debouncing is not really needed when the triggered event duration is much longer than the switch settling time. For instance we beep for 5 seconds in this project, which is much longer than the switch will be bouncing. If this were not the case, however, we would run the risk of triggering multiple executions of an event for a single button press.

### Code:

```
    //A small replica of the infamous 2600 Blue Box
    #define speaker_pin 10
    #define Push_button_pin 3
    #define LED_pin 13

    void setup()
    {
            // put your setup code here, to run once:
            pinMode(speaker_pin, OUTPUT);
            pinMode(LED_pin, OUTPUT);
            pinMode(Push_button_pin, INPUT);
            digitalWrite(Push_button_pin, HIGH); //Doing this to an output pin turns on an internal ~30K Ohm pull-up resistor
            // Newer versions of the Arduino IDE support INPUT_PULLUP);
    }

    void loop()
    {
            // put your main code here, to run repeatedly:
            if (Debounced_input(Push_button_pin))
            {
                    tone(speaker_pin, 2600);
                    digitalWrite(LED_pin, HIGH);
                    delay(5000);
            }
            else
            {
                    noTone(speaker_pin);
                    digitalWrite(LED_pin, LOW);
            }
    }

    bool Debounced_input(int Pin)
    {
            if (digitalRead(Pin) == 0)
            {
                    //If the switch is constant for 1/10th of a second we assume it is through bouncing
                    int timeout = 0;
                    do
                    {
                            timeout = timeout + 1;
                            if (timeout >= 5) //Somthing is wrong
                            {
                                    return false;
                            }
                            delay(100); //  wait for the switch to settle
                    } while (digitalRead(Pin) == 1);
                    return true;
            }
            else
            {
                    return false;
            }
    }
```

## Exercise Six - Detect Body Heat in Motion - B0110

### Creating a Motion Activated Alarm Siren (using digital GPIO)

In this exercise, you will learn how to use digital general purpose input/output pins (GPIO) on the Arduino to read a signal from a sensor and respond accordingly, depending on if the signal is logic low or high.

This project uses a passive infrared (PIR) sensor to detect intruders. Warm-blooded intruders are generally hotter, ~98.6F, than the background office environment, ~75F, provided the air-conditioning is working ;-). Since everything warmer than absolute zero glows, due to black body radiation, we are able to measure the far infrared (FIR) light signatures emitted from people with our fancy $2 micro-bolometer sensor and our friend, the Arduino. Your sensors are pre-configured to toggle a logic HIGH (5V) upon detecting the increased FIR signature from a human.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/motion_sensor.png?raw=true) <http://www.simplelabs.co.in/328-thickbox_leometr/pir-sensor-motion-detection-sensor.jpg> ![](http://www.simplelabs.co.in/328-thickbox_leometr/pir-sensor-motion-detection-sensor.jpg)

### Code:

```
    #define Motion_sensor_pin 3
    #define speaker_pin 10
    #define LED_pin 13

    void setup()
    {
            // put your setup code here, to run once:
            pinMode(Motion_sensor_pin, INPUT); //Motion sensor
            pinMode(speaker_pin, OUTPUT);
            pinMode(LED_pin, OUTPUT);
    }

    void loop()
    {
            // put your main code here, to run repeatedly:
            if (digitalRead(Motion_sensor_pin) == 1)
            { // Motion detected
                    sound_siren();
            }
    }

    void sound_siren()
    {
            digitalWrite(LED_pin, 1); // Turn LED on
            for(int i = 100; i < 4000; i++)
            {
                    tone(speaker_pin, i);
                    delayMicroseconds(500);
            }
            for(int i = 4000; i > 100; i--)
            {
                    tone(speaker_pin, i);
                    delayMicroseconds(500);
            }
            noTone(speaker_pin);
    }
```

## Exercise Seven - USB for Fun and Profit - B0111

### Using the integrated UART

This exercise will walk you through the basics of using the Arduino's USB/Serial interface. You will learn fundamentals of exchanging data between a computer and a running Arduino board. All Arduino boards have at least one serial port (also known as a UART or USART). You can use the Arduino environment's built-in serial monitor to communicate with an Arduino board. Compile Code and open the Serial Monitor under Tools.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Uno.png?raw=true)

### Code:

```
    void setup()
    {
            // put your setup code here, to run once:
            Serial.begin(9600);     //sets the baud rate and must match the speed set on the other end
            Serial.println("The answer is:");
    }
    void loop() {
            // put your main code here, to run repeatedly:
            int x = 42;
            Serial.println();
            Serial.println(F("Default    DEC        HEX        OCT
    BIN")); // println starts a new line after the print operation
            Serial.print(x); // while print does not append the carriage return
            Serial.print(F("         "));
            Serial.print(x,DEC);
            Serial.print(F("         "));
            Serial.print(x,HEX);
            Serial.print(F("         "));
            Serial.print(x,OCT);
            Serial.print(F("         "));
            Serial.println(x,BIN);
            delay(5000);
    }
```

## Exercise Eight - Look Inside a Wire - B1000

### Pulse Width Modulation

You will see how controlling the ration of on time to off time (pulse width modulation or PWM) lets you create a sort of Digital to Analog converter (DAC). Pulse Width Modulation, or PWM, is a technique for getting analog results with digital means. Digital control is used to create a square wave, a signal switched between on and off. This on-off pattern can simulate voltages in between full on (5 Volts) and off (0 Volts) by changing the portion of the time the signal spends on versus the time that the signal spends off. The duration of "on time" is called the pulse width. To get varying analog values, you change, or modulate, that pulse width. If you repeat this on-off pattern fast enough with an LED for example, the result is as if the signal is a steady voltage between 0 and 5v controlling the brightness of the LED. Source: <https://www.arduino.cc/en/Tutorial/PWM> Use the serial plotter to view the waveform.

SEE HOW ADDING A CAPACITOR ACROSS THE PWM OUTPUT AND GND VARIES THE WAVEFORM

Further reading for later - Fading an LED using Pulse Width Modulation: <https://www.arduino.cc/en/Tutorial/Fading>

Note: a protection diode is required for inductive loads.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/PWM1.png?raw=true) ![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/PWM2.png?raw=true)

### Code

```
    #define speaker_pin 10
    #define LED_pin 9
    #define pi 3.14159
    #define ADC A0

    void setup()
    {
            // put your setup code here, to run once:
            pinMode(speaker_pin, OUTPUT);
            pinMode(LED_pin, OUTPUT);
            pinMode(ADC, INPUT);
            Serial.begin(9600);
    }

    void loop()
    {
            // put your main code here, to run repeatedly:
            for (int i = 0; i < 360; i++)
            {
                    static float waveform = 0;
                    waveform = 127 * sin(i * (pi / 180)) + 127;
                    tone(speaker_pin, 30 + (waveform * 10));
                    analogWrite(LED_pin, waveform);
                    Serial.println(analogRead(ADC));
                    delay(100);
            }
    }
```

## Exercise Nine - Hear the Light - B1001

### Analog to Digital Conversion and Signal Graphing (Light sensor)

In this exercise, you will learn the basics of using the Arduino's analog to digital converter (ADC) by measuring the analog voltage from a photocell voltage divider circuit. Even though the Arduino is a digital tool, it's possible for it to get information from analog sensors to measure things like temperature or light. To do this, you'll take advantage of the Arduino's built in Analog-to Digital Converter (ADC). Analog in pins A0-A5 can report back a value between 0-1023 which maps to a range from 0 volts to 5 volts. Using a photo-resistor and a Piezo element, you're going to make a light-based theremin. By waving your hand over the photo-resistor, you'll change the amount of light that falls on the photo-resistor's face. The change in voltage on the analog pin will determine what frequency to play on the piezo speaker.

For fun, try replacing the photocell with a potentiometer.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Speaker-and-Photocell.png?raw=true)

### Code:

```
    #define light_sensor_pin A0
    #define speaker_pin 10
    #define sensor_averaging 3
    unsigned long light_zero_cal = 0;

    void setup()
    {
            // put your setup code here, to run once:
            pinMode(light_sensor_pin, INPUT);//Light sensor
            pinMode(speaker_pin, OUTPUT);//Speaker sensor
            Serial.begin(115200);
            for (int i = 0; i < sensor_averaging; i++)
            {
                    light_zero_cal += analogRead(light_sensor_pin);
            }
            light_zero_cal = light_zero_cal / sensor_averaging;
    }

    void loop()
    {
            // put your main code here, to run repeatedly:
            static unsigned int measurement = 0;
            measurement = 0;
            for (int i = 0; i < sensor_averaging; i++)
            {
                    measurement = measurement + analogRead(light_sensor_pin);
            }
            measurement = measurement / sensor_averaging;
            Serial.println(map(measurement, light_zero_cal, 1023, 0, 100));
            tone(speaker_pin, map(measurement, light_zero_cal, 1023, 60, 2000));
    }
```

## Exercise Ten - Reading LCD Leaves - B1010

### Using a LCD

In this exercise, you learn the basics of using a shield with the Arduino by writing text to the LCD display. LCDs for micro-controllers come in two different flavors: serial communication and parallel communication. The main advantage of serial LCDs is that they are easier to wire up since the Arduino talks to the LCD over a single pair of wires. The LCD you will be using is a 16x2 Character LCD. In order to use the LCD shield, the library for it will need to be loaded into the Arduino IDE. If you are using Arduino IDE version 1.6.2 or later, you can install the library using the built-in library manager. In the toolbar, navigate to Sketch->Include Library->Manage Libraries and search for "Adafruit RGB LCD Shield". Click on the first option, then click "Install". You can now use the library in your sketches.

A note on RAM conservation using the F("") and PROGMEM macros: F() forces constant strings to reside solely in FLASH program memory, and not be copied into RAM (AKA dynamic memory in the Arduino IDE nomenclature). PROGMEM forces constant arrays to reside solely in FLASH program memory, and not be copied into RAM (AKA dynamic memory in the Arduino IDE nomenclature).

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/LCD.png?raw=true)

### [Code](https://github.com/MinutesToMidnight/8_Ball_Arduino/blob/master/Arduino_8_Ball.ino)

## Exercise Eleven - Pet an Electron - B1011

### Capacitive Touch Sensor

Arduino Unos can do about 16 million things a second! They are so fast, in fact, that you can literally use one to monitor electrons spreading out on a surface, as you will see in this exercise. When you move your hand close to the conductive plate, the effective permittivity that the propagating electrons see is altered. Your body's electric permittivity replaces that of the surrounding air. This change in effective permittivity alters the capacitance. In turn, the increase in capacitance increases the RC time constant of the circuit causing the applied charge to take a longer time to equalize on the surface.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/pet_me.png?raw=true)

### [Code](https://github.com/MinutesToMidnight/Arduino_Capacitive_Touch_Sensor/blob/master/Arduino_Capacitive_Touch_Sensor.ino)

## Exercise Twelve - How Fast - B1100

### Measuring the Speed of Sound

We will use an ultrasound transducer to bounce a sound wave off of an object a known distance away. By measuring the time of flight we can determine the speed of sound with surprising accuracy.

### Setup for the Exercise

### Code

```
    #define ping_pin  3
    #define echo_pin  4
    #define sample_averages 4

    void setup()
    {
            double time_of_flight_microseconds = 0;
            double time_of_flight_seconds = 0;
            double distance_in_inches = 0;
            double distance_in_feet  = 0;
            double speed_of_sound = 0;
            pinMode(ping_pin, OUTPUT);
            pinMode(echo_pin, INPUT);
            for (int i = 0; i < sample_averages; i++)
            {
                    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
                    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
                    digitalWrite(ping_pin, LOW);
                    delayMicroseconds(200);
                    digitalWrite(ping_pin, HIGH);
                    delayMicroseconds(200);
                    digitalWrite(ping_pin, LOW);
                    // HIGH
                    // pulse whose duration is the time (in microseconds) from the sending
                    // of the ping to the reception of its echo off of an object.
                    time_of_flight_microseconds += pulseIn(echo_pin, HIGH);
            }
            time_of_flight_microseconds /= sample_averages; //average samples to minimize the influence of measurement jitter
            time_of_flight_seconds = time_of_flight_microseconds / 1000000;
            Serial.begin(9600);
            Serial.println(F("Distance measured in inches: 12"));
            //distance_in_inches = Serial.parseInt();
            distance_in_inches = 12;
            distance_in_inches = distance_in_inches * 2; //The sound has to travel out and back so the total
            distance is two times that measured
                    distance_in_feet = distance_in_inches / 12;
            //Speed = distance / time
            speed_of_sound =  distance_in_feet / time_of_flight_seconds;
            Serial.println(F("########################################"));
            Serial.print(F("The speed of sound in feet per second is: "));
            Serial.println(speed_of_sound);
            Serial.println(F("########################################"));
    }

    void loop()
    {
    }
    // According to Parallax's data-sheet for the PING, there are
    // 73.746 microseconds per inch (i.e. sound travels at ~1130 feet per
    // second at room temperature and standard pressure).
```

## Exercise Thirteen - Flying Circus - B1101

### PID Closed Loop Control for Magnetic Levitation

This exercise is a work in progress. Please, skip it for now.

Learn more about PID Feedback here: <http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/>

### Setup for the Exercise

### Code

## Exercise Fourteen - Serial Servo - B1110

### Receiving Serial Communication

This exercise will walk you through a simple sketch that receives serial data from the computer and uses that data to control a servo motor. In order to use the servo motor, you will have to use the Servo library. It should be built-in to the Arduino IDE, but if not you can install it using the same steps from the LCD exercise. Once you are sure that the library is installed, set up the circuit shown, copy the code below into the Arduino IDE and upload it. Make sure that the wires are connected correctly, as the pinout on the connectors is sometimes different than the pinout on the motor. After programming, open the Serial Monitor and set the baud rate to 9600\. You should see instructions, and you can now type in a value between 0 and 155\. If everything is set up correctly, the servo should immediately spin to the desired angle. You can also try inputting values outside the correct range to see what happens (and why you should always include some sort of error checking into your sketches).

It is left to the reader to figure out how to fabricate a rubber band launcher out of this, outside of class... ;-P

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/servo.png?raw=true)

### Code

See instructor...

## Exercise Fifteen - Stream Cypher - B1111

### Use the XOR operator and a PRNG to build a toy stream cypher

This exercise will walk you through a simple sketch that receives serial data from the computer and uses the XOR ^ operator to encrypt and decrypt messages. Copy the code below into the Arduino IDE and upload it. After programming, open the Serial Monitor and set the baud rate to 9600\. You should see instructions, and you can now type in the secret key below, hit enter, and copy in the example encrypted message. If everything is set up correctly, the terminal will display a fun message, and possibly a link...

Note: There is no need to specify encrypt or decrypt due to the symmetric nature of the cypher.

### Setup for the Exercise

![](https://github.com/MinutesToMidnight/Introduction-to-Embedded-Systems/blob/master/Uno.png?raw=true)

### [Code](https://github.com/MinutesToMidnight/XOR-Toy-Stream-Cypher-Arduino/blob/master/XOR-Stream_Cypher_Arduino.ino)

### Test Message

Secret Key: `1337`

Encoded Message:



## Further Reading List:

- <https://www.amazon.com/Programming-Arduino-Next-Steps-Electronics/dp/0071830251>
- <https://www.amazon.com/Understanding-Cryptography-Textbook-Students-Practitioners/dp/3642041000/ref=sr_1_1?s=books&ie=UTF8&qid=1504670414&sr=1-1&keywords=paar>
- <https://www.amazon.com/Art-Electronics-Paul-Horowitz/dp/0521809266/ref=sr_1_1?s=books&ie=UTF8&qid=1504670485&sr=1-1&keywords=art+of+electronics>

## Unofficial Kit:

Although we do not endorse any particular vender, here is a list of hardware we liked using when we were preparing this course:

- [Arduino Uno Compatible Kit (Keyestudio ARDUBLOCK Graphical Programming Starter Kit) ~$30](https://www.amazon.com/Keyestudio-ARDUBLOCK-Graphical-Programming-Starter/dp/B01K4AL680)
- [Adafruit Character LCD ~$25](https://www.adafruit.com/product/714)
- LCR Component Identifier (LCR-T4 Mega328 ebay) ~$8
- [LED Shield (Keyestudio 2812) ~$10](https://www.amazon.com/Keyestudio-Pixel-Matrix-Shield-Arduino/dp/B01E8LZD58)
- RFID Reader and Tags (MFRC-522 ebay) ~$3
- [Breadboard ~$22](https://www.mpja.com/3220-Tie-Point-Solderless-Breadboard-Mounted-Panel-with-Jumpers/productinfo/24447+TE/)
