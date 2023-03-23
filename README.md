# Embedded Systems Coursework 2 Report

## Features of our synthesizer:

Our synthesizer includes multiple base and advanced features, all of which are listed below. On how to use these features, refer to the User Manual. 

**Base Features:**
- **Key scanning task:** Scanning the system peripherals such as the key matrix, knobs, and western and eastern keyboard connections. 
- **Display update task:** Displays the UI.
- **Knob class:** Updates the rotation value using the latest state of the key matrix, sets upper and lower limits and reads the current rotation value. 
- **CAN bus communication:** Messages are sent and received between all three Keyboards. Sender and Receiver are configured based on auto-detect. 


**Advanced Features:**

- **Waveform selection:** The user is able to change the Timbre of the sound produced by selecting A square, traingle, saw-tooth or Sine wave waveform. The graph of the selected waveform is displayed on the display. 
- **Polyphony:** The user can select multiple tones to be played simeltaneously. 
- **User Interface:** Our sytnehsizer containes a intuitive User Interface. Different information is displayed depending on the number of Keyboards connected. (Guide to user display is in User Manuel section)
    - Master Keyboard: Displays selected volume and octave band, current key being pressed and waveform being produced. The waveform it self is shown graphically, aiding the user experience. If more than one key is pressed, the last key pressed is displayed. 
    - 2nd Keyboard: Displays the note and octave band being played on the 2nd keyboard, and the LFO's tremolo and vibrato values. 
    - 3rd Keyboard: Displays  the note and octave band being played on the 3rd keyboard in addition to the A, D, S and R values for the synthesizers ADSR. These ADSR values are displayed alongside a dynamic visualization of the ADSR envolpe, who's slope's change depending on selected ADSR values. This aids the user's ability to select the required envelope.* 
    - Note: The Octave level for each Keyboard is also dynamically allocated. If 3 keyboards are connected, the maximum octave level of the master keyboard is automatically set to 6, as the 2nd and 3rd keyboard are set to octave band 7 and 8 respectively. 
- **Key-board auto-detect:** Our synthesizer allows the user to cascade multiple keyboards, in any order. By default, the left most keyboard will be the master keyboard, while each succesive keyboard to the masters right will be 1 octave band higher. Keyboards can be disconnected and reconnected while playing, however, some synchronization erros might appear on startup. This would require the user to rest the board using the reset button. 
- **Stereo:** When the user uses multiple keyboards, each of the speakers on the seperate keyboards are also employed. This enables the Stereo effect, where the resulting sound is given more depth. 
- **LFO:** Our synthesizer contains a Low Frequency Oscillator, designed to produce both Vibrato and Tremolo effects. Vibrato is a form of frequency modulation while Tremolo is a form of amplitude modulation. Both paramters can be tuned from a range of 0 Hz to 20 Hz. See user manual on tuning of LFO. 
- **Double Buffer**: Given our advanced features, the generation of  audio samples becomes computationally expensive. Doing these computations on demand, once per sample period, results in a high ISR workload leading to conflicts with the RTOS and locking of the MUTEX. We thus calculate audio samples in batches and synchornize their delivery to the DAC using a double buffer. The buffer is divided into two halves, with one half being accessed by the writing task and the other half being accessed by the reading task. Once each task has completed accessing its assigned half of the buffer, the pointers are swapped to ensure that the other half is accessed. This synchronization is carefully orchestrated to ensure that the pointer swap occurs atomically, thereby preventing both tasks from accessing the same half of the buffer. 

* Although an ADSR UI was implemented, we failed in achieving a working ADSR.  

## Analysis:
[Timing and CPU analysis of our synthesizer](doc/CPUanalysisandtiming.md)


## User Manual:
[How to use our synthesizer](doc/howtouse.md)




  
