// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include "math.h"
#include <fstream>
#include <vector>

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{ 
  // INSERT YOUR CODE HERE
    //define the constants
    double Kh, Kp, lander_mass, delta, h, Pout;
    lander_mass = FUEL_CAPACITY * fuel * FUEL_DENSITY + UNLOADED_LANDER_MASS;
    delta = (GRAVITY * MARS_MASS * lander_mass / position.abs2()) / MAX_THRUST;
    h = position.abs() - MARS_RADIUS;
    
    Kp = 2; //+ constant controls the gain Pout
    Kh = 0.01; // + constant controls descent

    double error = -(0.5 + Kh * h + velocity * position.norm());
    Pout = Kp * error; //Controller gain

    // use && when multiple conditions required
    if (parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute() && (h < 40000)) {
        parachute_status == DEPLOYED;
    } //Auto-Activation of parachute

    //descent rate should decrease linearly as the lander approaches the surface
    //error is positive if the lander is descending too quickly and negative if the lander is descending too slowly
    // even when the error e approaches zero, we still need a certain amount of thrust to balance the weight.T
    if (Pout <= -delta) { throttle = 0; } 
    else if (Pout < 1 - delta) { throttle = delta + Pout; }
    else { throttle = -1; }

    ofstream fout;
    if (simulation_time == 0) { fout.open("autopilot.txt"); } //time = 0, open file, empty file every time
    else { fout.open("autopilot.txt", fstream::app); } //when time != 0, write into the file 
    
    //write variable values to file 
    if (fout) { // file opened successfully
        fout << h << ' ' << Kh * h << ' ' << -1* velocity * position.norm() << endl;
        //Target descent rate = Kh * h, Actual descent rate = v * er
    }
    else { // file not opened
        cout << "Could not open autopilot file for writing" << endl;
    }
    fout.close();
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
  //FIND acceleration
    //define variables
    double mass, rho, lander_area, chute_area;
    vector3d drag_lander, drag_chute, drag_f, gravity_f, thr, acceleration;

    //calculate total mass = lander + fuel, m = density*volume
    mass = FUEL_CAPACITY * fuel * FUEL_DENSITY + UNLOADED_LANDER_MASS;
    //calculate gravity force = GMm/r^2 * r^
    gravity_f = (GRAVITY * MARS_MASS * mass) * (- position.norm()) / position.abs2();
    //calculate thrust
    thr = thrust_wrt_world();

    rho = atmospheric_density(position);
    lander_area = 3.1415 * pow(LANDER_SIZE, 2);
    chute_area = 5 * pow(2.0 * LANDER_SIZE, 2);
    drag_lander = -0.5 * rho * DRAG_COEF_LANDER * lander_area * velocity.abs2() * velocity.norm();
    drag_chute = -0.5 * rho * DRAG_COEF_CHUTE * chute_area * velocity.abs2() * velocity.norm();
    
    //calculate drag force
    if (parachute_status == DEPLOYED) {
        drag_f = drag_lander + drag_chute;
    }
    else {
        drag_f = drag_lander;
    }

    //calculate accleration = F/m
    acceleration = (thr + gravity_f + drag_f) / mass;
  
  //Verlet integrator
    static vector3d previous_position;
    vector3d new_position;
    if (simulation_time == 0.0) {
        new_position = position + delta_t * velocity;
        velocity = velocity + delta_t * acceleration;
    }

    else {
        new_position = position + delta_t * velocity;
        velocity = velocity + delta_t * acceleration;
        //new_position = 2 * position - previous_position + pow(delta_t, 2) * acceleration;
        //velocity = (new_position - position) / delta_t;
    }

    previous_position = position;
    position = new_position;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
