/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | Copyright (C) 2011-2017 OpenFOAM Foundation
     \\/     M anipulation  |
-------------------------------------------------------------------------------
License
    This file is part of OpenFOAM.

    OpenFOAM is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenFOAM is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License
    along with OpenFOAM.  If not, see <http://www.gnu.org/licenses/>.

Application
    gen6DoF

Description
    Generate simple sinusoidal 6-DoF motion control-file.

\*---------------------------------------------------------------------------*/

#include "List.H"
#include "vector.H"
#include "Vector2D.H"
#include "Tuple2.H"
#include "OFstream.H"
#include "mathematicalConstants.H"

using namespace Foam;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// Variables for this case:

// Rotor radius [m]
const scalar R = 0.070;

// Tension member length [m]
const scalar L = 0.074;

// Tension member fixed length from blade pivoting point [m]
const scalar d = 0.027;

// Shift arm length [m]
const scalar e = 0.013;

// Eccentric phase angle [deg]
const scalar epsilon = 292;

// Blade inclination angle [deg]
scalar theta;

// Position of the rotor blade [deg]
//scalar psi;

// Distance between shift arm and blade pivoting point [m]
scalar a;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// Sub routine:

scalar angleCalc1(const scalar& rotAmpZ, const scalar& t)
{
    scalar psi_rad = ((rotAmpZ * t * 20) + 270) * M_PI/180; // 270 degree phase shift
    scalar epsilon_rad = epsilon * M_PI/180;
    scalar theta_rad;
    scalar a_sq;
    
    a_sq =  e*e + R*R - 2*e*R * Foam::cos(psi_rad + epsilon_rad + M_PI/2);
    a = Foam::sqrt(a_sq);
    
    theta_rad = (M_PI/2) - Foam::asin((e/a) * Foam::cos(psi_rad + epsilon_rad)) - Foam::acos((a*a + d*d - L*L)/(2*a*d));
    theta = theta_rad * 180/M_PI;
    
    return rotAmpZ * t * 20 + theta;
}

scalar angleCalc2(const scalar& rotAmpZ, const scalar& t)
{
    scalar psi_rad = ((rotAmpZ * t * 20) + 90) * M_PI/180; // 90 degree phase shift
    scalar epsilon_rad = epsilon * M_PI/180;
    scalar theta_rad;
    scalar a_sq;
    
    a_sq =  e*e + R*R - 2*e*R * Foam::cos(psi_rad + epsilon_rad + M_PI/2);
    a = Foam::sqrt(a_sq);
    
    theta_rad = (M_PI/2) - Foam::asin((e/a) * Foam::cos(psi_rad + epsilon_rad)) - Foam::acos((a*a + d*d - L*L)/(2*a*d));
    theta = theta_rad * 180/M_PI;

    return rotAmpZ * t * 20 + theta;
}

scalar angleCalc3(const scalar& rotAmpZ, const scalar& t)
{
    scalar psi_rad = ((rotAmpZ * t * 20) + 0) * M_PI/180; // non phase shift
    scalar epsilon_rad = epsilon * M_PI/180;
    scalar theta_rad;
    scalar a_sq;
    
    a_sq =  e*e + R*R - 2*e*R * Foam::cos(psi_rad + epsilon_rad + M_PI/2);
    a = Foam::sqrt(a_sq);
    
    theta_rad = (M_PI/2) - Foam::asin((e/a) * Foam::cos(psi_rad + epsilon_rad)) - Foam::acos((a*a + d*d - L*L)/(2*a*d));
    theta = theta_rad * 180/M_PI;
    
    return rotAmpZ * t * 20 + theta;
}

scalar angleCalc4(const scalar& rotAmpZ, const scalar& t)
{
    scalar psi_rad = ((rotAmpZ * t * 20) + 180) * M_PI/180; // 180 degree phase shift
    scalar epsilon_rad = epsilon * M_PI/180;
    scalar theta_rad;
    scalar a_sq;
    
    a_sq =  e*e + R*R - 2*e*R * Foam::cos(psi_rad + epsilon_rad + M_PI/2);
    a = Foam::sqrt(a_sq);
    
    theta_rad = (M_PI/2) - Foam::asin((e/a) * Foam::cos(psi_rad + epsilon_rad)) - Foam::acos((a*a + d*d - L*L)/(2*a*d));
    theta = theta_rad * 180/M_PI;
    
    return rotAmpZ * t * 20 + theta;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// Main program:

int main(int argc, char *argv[])
{
    // End-time of the table [sec]
    const scalar endTime = 1;

    // Number of entries in the table
    const label nTimes = 1000;

    // Amplitude of the translation [m]
    const vector transAmp(0.07, 0.07, 0);

    // Frequency of the translation [rad/s]
    const vector transOmega(125.6637, 125.6637, 0); // 1200 rpm = 20 rps

    // Amplitude of the rotation [deg]
    const vector rotAmp(0, 0, 360);

    // Frequency of the rotation [rad/s]
    const vector rotOmega(0, 0, 0);
    
    List<Tuple2<scalar,  Vector2D<vector>>> timeValues(nTimes);

    forAll(timeValues, i)
    {
        scalar t = (endTime*i)/(nTimes - 1);
        timeValues[i].first() = t;
        
        timeValues[i].second()[0] = vector
        (
            transAmp.x()*Foam::sin(transOmega.x()*t) * -1,
            transAmp.y()*Foam::cos(transOmega.y()*t) - 0.07,
            transAmp.z()*Foam::sin(transOmega.z()*t)
        );

        timeValues[i].second()[1] = vector
        (
            rotAmp.x()*Foam::sin(rotOmega.x()*t),
            rotAmp.y()*Foam::sin(rotOmega.y()*t),
            angleCalc1(rotAmp.z(), t)
        );
    }

    {
        OFstream dataFile("6DoF1.dat");
        dataFile.precision(6);
        dataFile.setf(std::ios::fixed);
        dataFile << timeValues << endl;
    }

    forAll(timeValues, i)
    {
        scalar t = (endTime*i)/(nTimes - 1);
        timeValues[i].first() = t;

        timeValues[i].second()[0] = vector
        (
            transAmp.x()*Foam::sin(transOmega.x()*t),
            transAmp.y()*Foam::cos(transOmega.y()*t) * -1 + 0.07,
            transAmp.z()*Foam::sin(transOmega.z()*t)
        );

        timeValues[i].second()[1] = vector
        (
            rotAmp.x()*Foam::sin(rotOmega.x()*t),
            rotAmp.y()*Foam::sin(rotOmega.y()*t),
            angleCalc2(rotAmp.z(), t)
        );
    }

    {
        OFstream dataFile("6DoF2.dat");
        dataFile.precision(6);
        dataFile.setf(std::ios::fixed);
        dataFile << timeValues << endl;
    }

    forAll(timeValues, i)
    {
        scalar t = (endTime*i)/(nTimes - 1);
        timeValues[i].first() = t;

        timeValues[i].second()[0] = vector
        (
            transAmp.x()*Foam::cos(transOmega.x()*t) * -1 + 0.07,
            transAmp.y()*Foam::sin(transOmega.y()*t) * -1,
            transAmp.z()*Foam::sin(transOmega.z()*t)
        );

        timeValues[i].second()[1] = vector
        (
            rotAmp.x()*Foam::sin(rotOmega.x()*t),
            rotAmp.y()*Foam::sin(rotOmega.y()*t),
            angleCalc3(rotAmp.z(), t)
        );
    }

    {
        OFstream dataFile("6DoF3.dat");
        dataFile.precision(6);
        dataFile.setf(std::ios::fixed);
        dataFile << timeValues << endl;
    }

    forAll(timeValues, i)
    {
        scalar t = (endTime*i)/(nTimes - 1);
        timeValues[i].first() = t;

        timeValues[i].second()[0] = vector
        (
            transAmp.x()*Foam::cos(transOmega.x()*t) - 0.07,
            transAmp.y()*Foam::sin(transOmega.y()*t),
            transAmp.z()*Foam::sin(transOmega.z()*t)
        );

        timeValues[i].second()[1] = vector
        (
            rotAmp.x()*Foam::sin(rotOmega.x()*t),
            rotAmp.y()*Foam::sin(rotOmega.y()*t),
            angleCalc4(rotAmp.z(), t)
        );
    }

    {
        OFstream dataFile("6DoF4.dat");
        dataFile.precision(6);
        dataFile.setf(std::ios::fixed);
        dataFile << timeValues << endl;
    }

    Info<< "End\n" << endl;

    return 0;
}


// ************************************************************************* //
