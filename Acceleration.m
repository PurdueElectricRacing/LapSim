%% Acceleration Lap Simulation for PER20.

%% GOAL
% To simulate a single acceleration run with a simplified
% point mass representation of the car.

%% PURPOSE
% The purpose of this simulation is to create a realistic model of the
% tractive system and how global variables such as mass, battery voltage,
% and motor torque curve will effect the overall performence of the car.

%% MAIN FUNCTION
function [time, carSpeed] = Acceleration(carMass)
    %% CONSTANTS
    carMass;              % Mass of the car + driver in (kG)
    totalDistance = 75;        % Distance of acceleration event (m)
    timeResolution = 0.001;      % Step time between each acceleration update.
    mechAdvantage = getMechAdvantage();
    
    %% DYNAMICS
    carPosition = 0;
    carSpeed = 0;
    time = 0;

    %% MAIN SIMULATION
    % This is where the main "simulation" takes place.
    while carPosition < totalDistance
        torqReq = 85;
        
        % I'm pretty sure this is the wrong way to do it, probably not
        % correct
        wheelForce = getMotorTorque(torqReq) * mechAdvantage;
        carAcceleration = wheelForce / carMass;
        
        carSpeed = carSpeed + carAcceleration * timeResolution;
        deltaP = carSpeed * timeResolution;
        
        carPosition = carPosition + deltaP;
        
        time = time + timeResolution;
    end
    % End of Simulation.
    
    %% CAR SPECIFIC FUNCTIONS
    % These functions are specific to how our car will perform on the 
    % track and will be developed upon the most. These functions will
    % simulate how our components will perform on their own without much
    % regard for the rest of the car.
    
    
    % Function getMotorTorque
    %   I. request = Torque request from driver (N/m)
    %   O. torque = Output torque from the motor (N/m)
    % Returns the actual torque output from the motor given a requested
    % torque. This mimics the torque curve of the Emrax/Reinhart response.
    function torque = getMotorTorque(request)
        torque = clamp(request, 0, 700);
    end

    function advantage = getMechAdvantage()
       diff = 0.8;
       sprocket = 3.7;
       friction = 0.9;
       advantage = sprocket * diff * friction;
    end

    %% PHYSICS FUNCTIONS
    % These functions are derived from equations given in physics
    % textbooks. They will not vary from car to car and will remain
    % constant for the purpose of this simulation.
    
    
    % Function getKE
    %  I. speed = Speed of the car (m/s)
    %  O. energy = Total kenetic energy of the car at a given speed.
    % Returns kenetic energy of the car going a given speed
    function energy = getKE()
        energy = 1/2 * carMass * carSpeed ^ 2;
    end


    %% HELPER FUNCTIONS
    % Utility functions made to make computations easier.


    % return clamped value clipped between upper and lower
    function y = clamp(x,lower, upper)
        y=min(max(x,lower),upper);
    end

end