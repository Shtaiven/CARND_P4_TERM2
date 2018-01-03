# PID Controller Project Reflection

By Steven Eisinger

## Describe the effect each of the P, I, D components had in your implementation

[Here is a video of my PID controller taking the car simulation aroud the track](pid_in_action.mp4)

### The P Component

The proportional component of the controller had one of the most obvious effects on the car's control, which was how quickly it turned toward the center of the road. If this was too large, the car would overshooot a large amount, sometimes getting the car to fly off the track if going fast enough. This overshooting would only get worse over time if the I and D components weren't included.

### The I Component

The integral component is what takes the past actions of the car into account. As the sum of previous errors gets larger, the car will stabilize toward a steady state around the path, as any errors in adhering to the path will result in a large accumulation of CTE. Including this component straightened out the vehicle.

### The D Component

The derivative component helps the car approach a straight line. As the car straightens out, the derivative component goes close to 0. There are two issues here if we don't include the I component. The car can approach a constant which isn't where CTE is 0, and the car still over and undershoots slightly (but nowhere near as drastically as with P alone).

## Describe how the final hyperparameters were chosen

The final hyperparameters were chosen using a combination of twiddle and manual tuning. The twiddle algorithm was included in the `PID::UpdateError()` function and all hyperparameters (except dp) were passed in as inputs. The hyperparameters for twiddle did the following:

* `twiddle_enable`: Choose whether to use twiddle or not. This is disabled once hyperparams were chosen.
* `twiddle_tol`: Twiddle stops when the sum of `twiddle_dp` is less than this number. The lower the number, the more twiddle will try to tune the K parameters (Kp, Ki, and Kd).
* `twiddle_n`: The number of iterations before modifying a K parameter.
* `twiddle_multiplier`: The percentage of `twiddle_dp` to add or subtract from itself after a twiddle iteration.
* `twiddle_dp`: An array which stores the amount to change the K parameters by at every iteration.

I noticed that depending on the hyperparameters I chose, certain K values would stay stuck at a certain value even thought the car wasn't driving smoothly yet. Reducing `twiddle_n` and increasing `twiddle_multiplier` helped achieve better results faster, as well as keeping `twiddle_tol` low.

Twiddle didn't work perfectly and hyperparameter tuning was taking a long time so I resorted to some experimentation with manual turning, using some of the better results from my experiments with twiddle as a guideline. The video linked at the top of this reflection shows the results of my final choices for the K parameters, which are the following:

* Kp = 0.4
* Ki = 0.03
* Kd = 0.01

It doesn't run perfectly, but the car always remains on the track and goes back to the center even when thrown off momentarily, as seen at the end of the brick bridge segment.

## References

[PID Theory Explained, National Instruments](http://www.ni.com/white-paper/3782/en/)