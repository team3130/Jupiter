package frc.team3130.robot.motionProfiling;
/*
 * Copyright (C) 2019 The ERRORS FRC Team 3130
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


import java.util.ArrayList;
import java.util.List;

/**
 * Performs spline like interpolation given a finish point and angle(tangent).
 * Starting point is always (0,0) with the direction straight along the X axis.
 *
 * Synopsis:
 *         CubicPath path = new CubicPath(2, 4.5)
 *                 .withEnterVelocity(1.25)
 *                 .withExitVelocity(0)
 *                 .withDestination(10, 2, 0.3)
 *                 .generateSequence(0.005)
 *                 .generateProfiles(16);
 */
public class CubicPath {

    private double a ,b;
    private double L = 1;
    private double D = 0;
    private double A = 0;
    private double destination;
    private List<Double> mmPosition = new ArrayList<>();
    private List<Double> mmVelocity = new ArrayList<>();
    private List<Double> mmAlpha = new ArrayList<>();
    private double enterVelocity = 0;
    private double exitVelocity = 0;
    private double cruiseVelocity;
    private double maxAcceleration;
    private double dt;

    public CubicPath(double maxAcceleration, double cruiseVelocity) {
        this.maxAcceleration = maxAcceleration;
        this.cruiseVelocity = cruiseVelocity;
    }

    public CubicPath withEnterVelocity(double velocity) {
        enterVelocity = velocity;
        update();
        return this;
    }

    public CubicPath withExitVelocity(double velocity) {
        exitVelocity = velocity;
        update();
        return this;
    }

    private void update() {
        a = -2*D/(L*L*L) +A/(L*L);
        b = 3*D/(L*L) -A/L;
        destination = L * Math.sqrt(1 + Math.pow(3*a*L*L + 2*b*L, 2));
    }

    /**
     * Creates a cubic curve segment from origin to a given control point.
     * The spline is guaranteed to end at the control point exactly at the given slope.
     *
     * @param x
     *            The X component of the end point
     * @param y
     *            The Y component of the end point
     * @param slope
     *            The slope, tan of the angle, first derivative, at the end point
     * @throws IllegalArgumentException
     *             if the X is negative or any parameter is NaN.
     *
     * Example:
     * If the robot has to move 5 units forward and 2 units to the left (negative to right)
     * and approach the finish at tangent 1.0 (i.e. 45 degree) then the parameters would be:
     * .withDestination(5.0, 2.0, 1.0);
     * Units can be feet, meters, encoder units or whatever. Just convert them properly.
     */
    public CubicPath withDestination(double x, double y, double slope) {
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(slope) || x <= 0) {
            throw new IllegalArgumentException("The end point has to be strictly positive");
        }
        L = x;
        D = y;
        A = slope;
        update();
        return this;
    }

    /**
     * Generate the main sequence of the way points for the center of the robot
     * @param deltaTime time steps duration. Think of 0.01 seconds
     * @return this
     */
    public CubicPath generateSequence(double deltaTime) {
        dt = deltaTime;
        if (Double.isNaN(maxAcceleration) || Double.isNaN(cruiseVelocity) || maxAcceleration <= 0 || cruiseVelocity <= 0) {
            throw new IllegalArgumentException("Max acceleration and cruise velocity must be positive");
        }
        double enterAcceleration = cruiseVelocity > enterVelocity ? maxAcceleration : -maxAcceleration;
        double exitAcceleration = exitVelocity > cruiseVelocity ? maxAcceleration : -maxAcceleration;
        double enterPeriod = (cruiseVelocity - enterVelocity)/enterAcceleration;
        double exitPeriod = (exitVelocity - cruiseVelocity)/exitAcceleration;
        double enterDistance = enterPeriod*(enterVelocity + cruiseVelocity)/2.0;
        double exitDistance = exitPeriod*(exitVelocity + cruiseVelocity)/2.0;
        System.out.println("dest="+ destination +" up=" + enterDistance + " dn=" + exitDistance);
        double t = 0; // to track time
        double x = 0; // to track the X coordinate for rendering derivative
        double s = 0; // to track the linear travel distance
        double dS;
        // Loop throughout the whole path until we cover the entire distance
        do {
            double togoDistance = destination - s;
            double velocity = enterVelocity + enterAcceleration * t;
            // Not final value for the velocity yet, let's check if cruise is reached.
            if((cruiseVelocity >= enterVelocity && velocity >= cruiseVelocity)
                    ||(cruiseVelocity < enterVelocity && velocity < cruiseVelocity)) {
                velocity = cruiseVelocity;
            }
            // Now check if it's time to decelerate/accelerate at the end.
            double tailDuration = (exitVelocity - velocity)/exitAcceleration;
            double tailDistance = 0.5*tailDuration*(velocity + exitVelocity);
            if(togoDistance <= 0.0) {
                // at or beyond the last point
                velocity = exitVelocity;
            }
            else if(togoDistance <= tailDistance) {
                // Time to (de)accelerate to catch the exit velocity
                velocity = Math.sqrt(exitVelocity*exitVelocity - 2*exitAcceleration*togoDistance);
            }
            // The velocity is defined. Now render all the values for this point
            dS = velocity * dt;
            // Main curve is y = a*x^3 + b*x^2 therefore its derivative is:
            double dydx = 3*a*x*x + 2*b*x;
            // dx is how much we move along the X axis:
            double dx = dS / Math.sqrt(1 + dydx*dydx);
            mmPosition.add(s);
            mmVelocity.add(velocity);
            mmAlpha.add(Math.atan(dydx));
            s += dS;
            x += dx;
            t += dt;
        } while (s <= destination + dS);

        return this;
    }

    public int size() {
        return mmPosition.size();
    }

    public double length() {
        return destination;
    }

    public double getPosition(int i) {
        return mmPosition.get(i);
    }

    public double getAlpha(int i) {
        return mmAlpha.get(i);
    }

    public double getVelocity(int i) {
        return mmVelocity.get(i);
    }

    public double[][] profileLeft;
    public double[][] profileRight;

    /**
     * Generate motion profiles for two sides of the robot
     * @param width The width of the robot between the wheels. Must be in the same distance units.
     * @return this
     */
    public CubicPath generateProfiles(double width) {
        int N = mmPosition.size();
        double R = width/2.0;
        profileLeft = new double[N][3];
        profileRight = new double[N][3];
        double oldAlpha = 0;
        for(int i = 0; i < N; i++) {
            double yaw = mmAlpha.get(i) - oldAlpha;
            double deltaS = yaw * R;
            double deltaV = deltaS/dt;
            profileLeft[i][0] = mmPosition.get(i) - deltaS;
            profileLeft[i][1] = mmVelocity.get(i) - deltaV;
            profileLeft[i][2] = dt;
            profileRight[i][0] = mmPosition.get(i) + deltaS;
            profileRight[i][1] = mmVelocity.get(i) + deltaV;
            profileRight[i][2] = dt;
            oldAlpha = mmAlpha.get(i);
        }
        return this;
    }

}
