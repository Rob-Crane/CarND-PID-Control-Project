# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## PID Gain Tuning
The project uses an iterative tuning technique based on the "twiddle" algorithm.  A cost value is computed by driving the vehicle a set distance and computing the squared cross-track-error at each time step.  The sum of the cross-track errors is minimized.  The distance travelled between updates is set to approximate the length of the test track in order to provide the vehicle the most representative sample while tuning every set of parameters.

## Twiddle Implementation
Because of the asynchronous design of the simulator-controller, the twiddle algorithm is implemented as a state machine.  State transitions occur after the distance threshold is met or after a max-CTE condition occurs.  A max CTE condition is treated as a sub-optimal result and the program pauses to allow for simulator reset.

## Training Approach
In order to allow the simulation to collect sufficient data (drive sufficient laps), it is critical to attain a degree of gain tuning that allows the car to drive around the track without reaching a max-CTE condition.  Sufficient gain values were obtained by trial and error.  The initial set of gains discovered this way are: `Kp = 0.16`, `Kd = 0.8`, and `Ki = 0.0001`.  With these gain values, the car can reliably complete a lap.

Next, the simulation was left unsupervized for roughly 12 hours.  During that time, the simulator recorded 356 laps.  The last optimal gain value was recorded the 276th lap.  The best gain values discovered are `Kp = 0.496113`, `Kd = 9.69385`, and `Ki = 0.000253635`.
