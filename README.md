# Embedded Systems Coursework 2 Report

## Features of our synthesizer:

Our synthesizer has multtiple features, all of which are listed below:

- LFO: Our synthesizer contains a Low Frequency Oscillator, designed to produce both Vibrato and Tremolo effects. Vibrato is a form of frequency modulation while Tremolo is a form of amplitude modulation
- Waveform selection: The user is able to change the Timbre of the sound produced by selecting A square, traingle, saw-tooth or Sine wave waveform.
- Polyphony: The user can select multiple tones to be played simeltaneously. 
- User Interface: Our sytnehsizer containes a intuitive User Interface, displaying information such as the selected volume and octave level, Key being pressed and waveform being produced. The waveform it self is shown graphically, aiding the user's experience. Furthermore, the LFO's tremolo and vibrato values are also displayed when a second otn , ADSR graph --> changes dynamically with different values fo ADSR  
- Key-board auto-detect: Our synthesizer allows the user to cascade multipkle keyboards, in any order. By default, the left most keyboard will be the master keyboard, while each succesive keyboard to the masters right will be 1 octave higher. Keboards can be disconnected and reconnected while playing, however, some synchronization erros might appear. This would require the user to rest the board using the reset button. 
- Stereo: When the user uses multiple keyboards, each of the speakers connected are also employed. This enable the Stereo effect, where the resulting sound is given more depth. 
- 

## Analysis:
[Timing and CPU analysis of our synthesizer](doc/CPUanalysisandtiming.md)




## User Manual:
[How to use our synthesizer](doc/howtouse.md)



## Documentation specifications: 

    The report shall be presented as documentation in the GitHub repository for your code, con-sisting of one or more markdown files linked to a table of contents
    
    The report shall contain: 
    - An identification of all the tasks that are performed by the system with their method of implementation, thread or interrupt
    - A characterisation of each task with its theoretical minimum initiation interval and measured maximum execution time
    - A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are met under worst-case conditions. 
    - A quantification of total CPU utilisation. 
    - An identification of all the shared data structures and the methods used to guarantee safe accessand synchronisation.
    - An analysis of inter-task blocking dependencies that shows any possibility of deadlock


```sh

```
  
