/*
Authors: Hunter Burnham
Start Date: 3/31
Completion Date: 4/1
Description of Code: This is an outline of the state machine used for demo 2
*/
enum class state {
    IDLE,
    SEARCH,
    CENTER,
    DRIVE,
    TURN,
    CIRCLE,
    STOP
};
state machineState = state::IDLE;

void loop() {
    static state machineState = state::IDLE;
    //state machine to control different parts of task
    switch (machineState){
        case state::IDLE:
            machineState = state::SEARCH;
            break;

        case state::SEARCH:
            //turn in 30 degree increments until aruco marker is found
            //if marker found go to next state
            machineState = state::CENTER;
            break;

        case state::CENTER:
            //short pause to allow PI to get correct angle
            machineState = state::DRIVE;
            break;

        case state::DRIVE:
            // drive forward a little less than 7 feet
            machineState = state::TURN;
            //if last marker
            machineState = state::STOP;
            break;

        case state::TURN:
            // turn 90 degrees to set up circle.
            machineState = state::CIRCLE;
            break;

        case state::CIRCLE:
            //drive in circle using p controller with semi equivalent velocities.
            //if marker detected
            machineState = state::CENTER;
            break;
        
        case state::STOP:
            //Stop case for the end that goes when encoders get near 360 degrees around a circle
            //do nothing
            break;

    }
}