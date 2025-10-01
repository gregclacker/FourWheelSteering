#ifndef TURNRADIUSCALC_H
#define TURNRADIUSCALC_H

#define WA_LW_Slope 0.3117f     //This is the slope of the left wheel, wheel angle vs turn radius
#define WA_LW_Intercept -1.6651f     //This is the Y intercept of the left wheel, wheel angle vs turn radius
#define WA_RW_Slope 0.2286f     //This is the slope of the right wheel, wheel angle vs turn radius
#define WA_RW_Intercept -0.194f     //This is the Y intercept of the right wheel, wheel angle vs turn radius
#define TR_LW_Coefficient 2677.7f  //This is the left wheel coefficient of the power function for Turn Radius vs Steering Angle
#define TR_LW_Power -1.147f    //This is the left wheel power of the power function for Turn Radius vs Steering Angle
#define TR_RW_Coefficient 2760.0f  //This is the right wheel coefficient of the power function for Turn Radius vs Steering Angle
#define TR_RW_Power -1.222f    //This is the right wheel power of the power function for Turn Radius vs Steering Angle

#endif
