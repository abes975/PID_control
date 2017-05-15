The effect of the Kp parameter is to make the car steering in a proportional way to the distance from the "center of the road"..ie 
cross track error = 0). The higher it's the parameter the faster the car returns to the "desired" point, but if this gain is too 
hight then the controller can start oscillating. Unfortunately using only the proportional gain the car continuously overshoot 
the desired point in either direction.
The effect of the Kd is to add a sort of "resistance" the quickness the car is moving toward the line as asked from the proportional
term. So it "smooth" the effect of the Kp term. If the term is too small the effect is quite neglectible and so the sistem 
continue to oscillate, and if it's too high the response of the sistem will be too slow to change the trajectory.
The integral Error (as it's the sum of the cross track errros) gives us the indication whether the car is spending time on the 
let's say posive or negative side of the road (considering zero value as the center of the road).So it is used to correct
"misalignment" or offset from the "zero" center of the road.

The final parameter choice was made all automatically using multiple iteration of twiddle algorithm.
My PID parameters start at with Kp = 0, Kd = 0, Ki = 0 and then using twiddle they are set to a point where the car
does not goes out of the track always trying to run as fast as it can.
In order to check if the car is still on the track I used speed (that should be positive and >= 0.1 and the absolute value 
of the cross track error (cte) should be inside a certain limit (in my case 4).
In this case a reset command is sent to the simulator and the tuning process continues.
In case the PID is not in tuning mode, if the car goes out of the road tuning mode is started again.
After some experimental test I choose to udate the parameter adding o subtracting a certain amount (0.2 incremented by 30% 
or decremented by 50% every iteration).
Each time that the error is measured and new parameter set the simulator is reset in order to restart the measurement.
I experienced some proble when restarting the simulator, data that where already in the socket (and they belongs to the previous session)
where delivered to the callback, but must be descarded in order not to have a wrong measurement.
This problem was solved by chekcing the value of speed and angle of the first sample sent by the simulator and after a reset was issued.
