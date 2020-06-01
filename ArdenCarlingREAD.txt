Welcome to the README

Operation
>>After starting the program, voltage range can be adjusted up
or down using S1 and S2 on the boosterpack
>>To switch the trigger direction click in the joystick.
>>Switch to fft mode by tilting the joystick down (toward the screen)
>>Return to V/T mode by tilting the joystick down again.
>>red arrow in top right coner shows trigger direction (left-->right)

Shared Data
>>All tasks are semaphore protected and only executed in order
>>no global variables shared by the button taks and display tasks
the exeption of the bool that controls fft mode, Vset, and, Trig,
switching these in between two tasks that use them would produce 
 at most one non-useful frame, but no erros would be caused 
 and the system would return to functioning normally 60ms or less 
 (one frame time)
 >>user input task only runs when a button is pressed, no data lost
 
Weird Stuff
>>FPS is the average frames displayed per second over the entire run
>>I only use vertical lines to draw the waveforms because the 
change in x is always 1 and vertical lines draw faster than angled
>>There are always 2 errors when you start it, but the hwi is in no risk 
of missing deadlines
>>Running on my board I achived 17fps (wooohoooo), so something is wrong, 
if you get significatly less performance than that 
