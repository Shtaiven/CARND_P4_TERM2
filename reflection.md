# PID Controller Project Reflection

By Steven Eisinger

## Describe the effect each of the P, I, D components had in your implementation

[Here is a video of my PID controller taking the car simulation aroud the track](pid_in_action.mp4)

### The P Component

The proportional component of the controller had one of the most obvious effects on the car's control, which was how quickly it turned toward the center of the road. If this was too large, the car would overshooot a large amount, sometimes getting the car to fly off the track if going fast enough. This overshooting would only get worse over time if the I and D components weren't included.

### The I Component

The integral component is what takes the past actions of the car into account. As the sum of previous errors gets larger, the car will stabilize toward a steady state around the path, as any errors in adhering to the path will result in a large accumulation of CTE. Including this component straightened out the vehicle.

### The D Component

## Describe how the final hyperparameters were chosen

## References

[PID Theory Explained, National Instruments](http://www.ni.com/white-paper/3782/en/)