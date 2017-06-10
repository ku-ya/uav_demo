#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jun  2 14:40:46 2017

@author: simonlibine
"""

from __future__ import division
import numpy as np
import sys


def initialisation(a, b, c):
    x0 = [a, b, c]
    x0_ship = [-2, -1.4, 0] #a changer dans la fonction shipPosition
    v_ship = [0.1 , 0.1, 0]
    dictionnary ={'landingCompleted' : 0, 'phase': np.array([])}
    dictionnary['turnValues'] = np.array([-1, -1, -1, -1, -1, -1, -1, -1, 1])
    zFirstPhases = 1.8
    tolerateAltitudeError = 0.35
    if zFirstPhases + tolerateAltitudeError <= x0[2] or x0[2] <= zFirstPhases - tolerateAltitudeError:
    #   startAltitudeValues = [status, x_droneStart, y_droneStart, z_droneStart, velocity];
        startAltitudeValues = np.array([1, x0[0], x0[1], x0[2], 0., 0., 0.])
        print('')
        dictionnary['phase'] = -1
        p = 'x0 = ' + repr(x0)
        print(p)
        p = 'phase = ' + repr(dictionnary['phase'])
        print(p)
        print('')
        if x0[2] <= zFirstPhases - tolerateAltitudeError:
            startAltitudeValues[4] = -1
            startAltitudeValues[5] = (startAltitudeValues[3] + zFirstPhases)/2
            startAltitudeValues[6] = -(startAltitudeValues[3] - zFirstPhases)/2
        elif x0[2] >= zFirstPhases + tolerateAltitudeError:
            startAltitudeValues[4] = 1
            startAltitudeValues[5] = (startAltitudeValues[3] + zFirstPhases)/2
            startAltitudeValues[6] = (startAltitudeValues[3] - zFirstPhases)/2
        dictionnary['startAltitudeValues'] = startAltitudeValues
        print('startAltitudeValues' + repr(startAltitudeValues))
    else:
        dictionnary['startAltitudeValues'] = np.array([0, 0, 0, 0, 0, 0, 0])
        dictionnary['phase'] = 0
        print('')
        p = 'x0 = ' + repr(x0)
        print(p)
        p = 'phase = ' + repr(dictionnary['phase'])
        print(p)
        print('')
    #    landingCompleted = 0
    dictionnary['side'] = 0
    dictionnary['previousVelocity'] = np.array([0, 0, 0])
    dictionnary['accelerationValuesX'] = np.array([0, -1, -1, -1, -1, -1])
    dictionnary['accelerationValuesY'] = np.array([0, -1, -1, -1, -1, -1])
    dictionnary['startPosition'] = x0
    dictionnary['landingValues'] = np.array([])
    dictionnary['startT'] = 0
    dictionnary['previousPhase'] = 0
    dictionnary['pointT1'] = 0
    dictionnary['refreshTime'] = 0.05
    dictionnary['droneVelocity'] = np.array([0,0,0])
    dictionnary['shipVelocity'] = np.array([0 ,0, 0])
    dictionnary['positionShip_list'] = np.array([[x0_ship[0]],[x0_ship[1]],[x0_ship[2]], [0]])
    dictionnary['shipVelocityGlobal'] = np.array(v_ship)
    dictionnary['desiredPosition'] = np.array(x0)
    dictionnary['errorPhase4_list'] = np.array([])
    dictionnary['errorCheckpoint_list'] = np.array([])
    dictionnary['checkPoint'] = np.array([0 , 0, 0])
    
    return dictionnary


def ShipPosition(t, dictionnary):
    """ Documentation :
        This function is a simulation of the position of the ship
    """
    shipVelocityGlobal = dictionnary['shipVelocityGlobal']

#     initialisation of the ship movement variables
    x0_ship = -2
    y0_ship = -1.4
    z0_ship = 0
    
    a_ship = shipVelocityGlobal[0] # a_ship = 0 -> y = cst
    b_ship = shipVelocityGlobal[1] # b_ship = 0 -> x = cst
    c_ship = shipVelocityGlobal[2]
    
#    shipVariables=np.array([x0_ship, y0_ship, z0_ship, a_ship, b_ship, c_ship])
#    
#     calcul of the position with error
#     x_ship = shipVariables(1) + t*shipVariables(4) + 0.5*rand(1);
#     y_ship = shipVariables(2) + t*shipVariables(5) + 0.5*rand(1);
#     z_ship = shipVariables(3) + t*shipVariables(6) + 0.5*rand(1);
#    
#     calcul of the position without error
    x_ship = x0_ship + t*a_ship;
    y_ship = y0_ship + t*b_ship;
    z_ship = z0_ship + t*c_ship;
    
    
    shipPosition = np.array([x_ship, y_ship, z_ship])
    
    return shipPosition


def UpdatePositionShip_list(t, shipPosition, dictionnary):
    """Documentation :
    This function will updated the list of the last positions of the ship
    
    """

    sizeList = 200;# number of the last ship positions in the memory
    
    positionShip_list = dictionnary['positionShip_list']
    shipPosition = np.array([[shipPosition[0]], [shipPosition[1]], [shipPosition[2]], [t]])

    if positionShip_list.shape[0] != sizeList:#positionShip_list.shape[1]
        positionShip_list = np.hstack((positionShip_list, shipPosition))
    else:    
        for k in range(sizeList-1):
            positionShip_list[0,k + 1] = positionShip_list[0,k];
            positionShip_list[1,k + 1] = positionShip_list[1,k];
            positionShip_list[2,k + 1] = positionShip_list[2,k];
            positionShip_list[3,k + 1] = positionShip_list[3,k];

        positionShip_list[0,-1] = shipPosition[0];
        positionShip_list[1,-1] = shipPosition[1];
        positionShip_list[2,-1] = shipPosition[2];    
        positionShip_list[3,-1] = shipPosition[3]; 
    
    dictionnary['positionShip_list'] = positionShip_list
    
    return 0


def ShipDirection(dictionnary):
    """Documentation :
        This finction will find the line equation of the ship direction
        
    """
    
    positionShip_list = dictionnary['positionShip_list']
    
    sizeList = positionShip_list.shape[1]
    
    if sizeList >= 2:
        
        xi = positionShip_list[0,:]
        yi = positionShip_list[1,:]
        
        test_xi = np.mean(np.abs(xi - xi[0]))
        test_yi = np.mean(np.abs(yi - yi[0]))
        
        if test_xi <= 5*10^-2: # if the difference between the successive points is less than 50cm, the equation will be x + c = 0
            a = -1
            b = 0
            c = xi[-1];
        elif test_yi <= 5*10^-2: # if the difference between the successive points is less than 5cm, the equation will be y + c = 0
            a = 0
            b = -1
            c = yi[-1]
        else:
            linearCoef = np.polyfit(xi,yi,1)
            a = linearCoef[0]
            b = -1
            c = linearCoef[1]
        
#         test_x = (positionShip_list(1,sizeList-1) - positionShip_list(1,sizeList));
#         if test_x >= 10^-2
#             a = (positionShip_list(2,sizeList-1) - positionShip_list(2,sizeList))/(positionShip_list(1,sizeList-1) - positionShip_list(1,sizeList));
#             b = -1;
#             c = positionShip_list(2,sizeList) - positionShip_list(1,sizeList);
#         else
#             a = -1;
#             b = 0;
#             c = positionShip_list(1,sizeList);
#         end
    else:  #default settings (d) : y = x
        a = 1 
        b = -1
        c = 0
        
    shipDirection = np.array([a, b, c])
    
    return shipDirection


def ShipVelocity(dictionnary):
    """ Documentation :
        This function returns the velocity of the ship
        
    """

    positionShip_list = dictionnary['positionShip_list']
#    refreshTime = dictionnary['refreshTime']
#    sizeList = positionShip_list.shape[1]
    
    deltaTime = positionShip_list[3,-1] - positionShip_list[3,1]
    if deltaTime != 0:
        v_x = (positionShip_list[0,-1] - positionShip_list[0,1])/(deltaTime)
        v_y = (positionShip_list[1,-1] - positionShip_list[1,1])/(deltaTime)
        v_z = 0
    else:
        v_x = 0.1
        v_y = 0.1
        v_z = 0
        
    shipVelocity = [v_x, v_y, v_z]

#    shipVelocityGlobal = dictionnary['shipVelocityGlobal']
#    shipVelocity = shipVelocityGlobal
    
    return shipVelocity


def Lines4Projection(distance, a, b, c, dictionnary):
    """ Documentation :
        This function will find the coefficients of a line parallele to the choosen line at a choosen distance

    """
    
    a1 = a # slope of the first line which we are looking for
    b1 = b
    
    a2 = a # slope of the second line which we are looking for
    b2 = b
    
    if b == 0: # case x = constante
        c1 = c - distance
        c2 = c + distance
    else :
        c1 = c - distance*np.sqrt((-a/b)**2 + 1)
        c2 = c + distance*np.sqrt((-a/b)**2 + 1)
    
    lines4Projection = np.array([a1, b1, c1, a2, b2, c2]);
    
    return lines4Projection


def Phase012(distance_droneShipDirection, secureDistance, shipDirection, x_drone, y_drone, Vmax_phase1, Vmax_phase2, dictionnary): # [newPhase, Vuav]

    shipVelocity = dictionnary['shipVelocity']
    
    if distance_droneShipDirection <= secureDistance:
        newPhase = 1
        if np.max(np.array([np.abs(shipVelocity[0]), np.abs(shipVelocity[1])])) <= Vmax_phase1:
            Vuav = np.array([shipVelocity[0], -shipVelocity[1]])
        else:
            if np.abs(shipVelocity[0]) <= np.abs(shipVelocity[1]):
                K = np.abs(shipVelocity[0]/shipVelocity[1])
                Vy = -np.sign(shipVelocity[1])*Vmax_phase1
                Vx = np.sign(shipVelocity[0])*np.abs(K*Vy)
                Vuav = np.array([Vx, Vy])
            else:
                K = np.abs(shipVelocity[1]/shipVelocity[0])
                Vx = -np.sign(shipVelocity[0])*Vmax_phase1
                Vy = np.sign(shipVelocity[1])*np.abs(K*Vx);
                Vuav = np.array([Vx, Vy]);
        a = shipDirection[0]
        b = shipDirection[1]
        c = shipDirection[2]
        lines4Projection = Lines4Projection(2*secureDistance, a, b, c, dictionnary)
        A = Phase1Point_A(x_drone, y_drone, lines4Projection, dictionnary)
        Drone_A = np.array([(A[0] - x_drone), (A[1] - y_drone)])
        Vuav = Vuav*np.sign(np.dot(Drone_A, Vuav))
        
    else:
        newPhase = 2
        if np.max(np.array([np.abs(shipVelocity[0]), np.abs(shipVelocity[1])])) <= Vmax_phase2:
            Vuav = np.array([-shipVelocity[0], -shipVelocity[1]])
        else:
            if np.abs(shipVelocity[0]) <= np.abs(shipVelocity[1]):
                K = np.abs(shipVelocity[0]/shipVelocity[1])
                Vy = -np.sign(shipVelocity[1])*Vmax_phase2
                Vx = -np.sign(shipVelocity[0])*np.abs(K*Vy)
                Vuav = np.array([Vx, Vy])
            else:
                K = np.abs(shipVelocity[1]/shipVelocity[0])
                Vx = -np.sign(shipVelocity[0])*Vmax_phase2
                Vy = -np.sign(shipVelocity[1])*np.abs(K*Vx)
                Vuav = np.array([Vx, Vy])
    
    phase012 = np.array([newPhase, Vuav[0], Vuav[1]])
    
    return phase012


def Phase1Point_A(x_drone, y_drone, lines4Projection, dictionnary):
    """ Documentation :
        This function will find Point A for the phase1
        
    """
    a1 = lines4Projection[0]
    b1 = lines4Projection[1]
    c1 = lines4Projection[2]
    a2 = lines4Projection[3]
    b2 = lines4Projection[4]
    c2 = lines4Projection[5]
    
    distance_droneLine1 = np.abs(a1*x_drone +b1*y_drone + c1)/np.sqrt(a1**2 + b1**2);
    distance_droneLine2 = np.abs(a2*x_drone +b2*y_drone + c2)/np.sqrt(a2**2 + b2**2);
    
    if distance_droneLine1 <= distance_droneLine2:
        if b1 == 0:
            X_projection = -c1/a1
            Y_projection = y_drone
        else:
            X_projection = (x_drone - a1*c1 + a1*y_drone)/(a1**2 + b1**2)
            Y_projection = -(a1*X_projection + c1)/b1
    else:
        if b1 == 0:
            X_projection = -c2/a2
            Y_projection = y_drone
        else:
            X_projection = (x_drone - a2*c2 + a2*y_drone)/(a2**2 + b2**2)
            Y_projection = -(a2*X_projection + c2)/b2
    
    A = np.array([X_projection, Y_projection]);

    return A


def FindCheckPoint(shipDirection, zCheckPoint, r, dictionnary):
    """ Documentation
        This function gives the position of the checkoint
    
    """
    
    positionShip_list = dictionnary['positionShip_list']
    
    A = Phase1Point_A(positionShip_list[0,-1], positionShip_list[1,-1], np.hstack((shipDirection, shipDirection)), dictionnary)
    x_projLast = A[0]
    y_projLast = A[1]
    shipPosition = np.array([x_projLast, y_projLast, 0]) # last position of the shiip projeted on the line direction
    
    A = Phase1Point_A(positionShip_list[0,0], positionShip_list[1,0], np.hstack((shipDirection, shipDirection)), dictionnary)
    x_projFirst = A[0]
    y_projFirst = A[1]
    FirstPositionShip = np.array([x_projFirst, y_projFirst, 0]) # first position of the ship in the list
    
    a = shipDirection[0]
    b = shipDirection[1]
    c = shipDirection[2]
    x_ship = shipPosition[0]
    y_ship = shipPosition[1]
    
#     two possible checkpoints
    
    if a == 0: #case y = constante
        checkpoint1 = np.array([x_ship - 2*r, y_ship, zCheckPoint])
        checkpoint2 = np.array([x_ship + 2*r, y_ship, zCheckPoint])
        
    elif b == 0: # case x = constante
        checkpoint1 = np.array([x_ship, y_ship - 2*r, zCheckPoint])
        checkpoint2 = np.array([x_ship, y_ship + 2*r, zCheckPoint])
        
    else:
        x_check1 = x_ship + (2*b*r)/np.sqrt(1 + a**2)
        x_check2 = x_ship - (2*b*r)/np.sqrt(1 + a**2)
        checkpoint1 = np.array([x_check1, -(a*x_check1 + c)/b, zCheckPoint])
        checkpoint2 = np.array([x_check2, -(a*x_check2 + c)/b, zCheckPoint])
            
    distance_Check1FirstPositionShip2D = np.sqrt((checkpoint1[0]-FirstPositionShip[0])**2+(checkpoint1[1]-FirstPositionShip[1])**2) # distance betwen checkpoint1 and FirstPositionShip in 2D (altitude does not mater)
    distance_Check2FirstPositionShip2D = np.sqrt((checkpoint2[0]-FirstPositionShip[0])**2+(checkpoint2[1]-FirstPositionShip[1])**2) # distance betwen checkpoint2 and FirstPositionShip in 2D (altitude does not mater)
    
    if distance_Check1FirstPositionShip2D <= distance_Check2FirstPositionShip2D: # checkpoint1 is closest to the older point of the ship
        checkpoint = checkpoint1
    else: # checkpoint2 is closest to the older point of the ship
        checkpoint=checkpoint2
    
    return checkpoint


def TrajectoryCreation(t, x_drone, y_drone, z_drone, dictionnary):
    """ Documentation
        This function create the best point for  targeting
    
    """            

    shipVelocity = dictionnary['shipVelocity']
    errorPhase4_list = dictionnary['errorPhase4_list']
    errorCheckpoint_list = dictionnary['errorCheckpoint_list']
    positionShip_list = dictionnary['positionShip_list']
    phase = dictionnary['phase']
    droneVelocity = dictionnary['droneVelocity']
    previousPhase = dictionnary['previousPhase']
    turnValues = dictionnary['turnValues']
    startPosition = dictionnary['startPosition']
    startT = dictionnary['startT']
    side = dictionnary['side']
    accelerationValuesX = dictionnary['accelerationValuesX']
    accelerationValuesY = dictionnary['accelerationValuesY']
    startAltitudeValues = dictionnary['startAltitudeValues']
    previousVelocity = dictionnary['previousVelocity']
    landingValues = dictionnary['landingValues']
    landingCompleted = dictionnary['landingCompleted']
    
#    %%%%%%%%%%%%%%% ELEMENTS OF THE SHIP AREA %%%%%%%%%%%%%%%
    shipDirection = ShipDirection(dictionnary) # direction of the ship
    shipVelocity = ShipVelocity(dictionnary)
    dictionnary['shipVelocity'] = shipVelocity
    shipPosition = np.array([positionShip_list[0,-1], positionShip_list[1,-1], positionShip_list[2,-1]]) # last position of the shiip
    
#    %%%%%%%%%%%%%%% ELEMENTS FOR THE TRAJECTORY %%%%%%%%%%%%%%%
    r = 0.8 # security area
    zFirstPhases = 1.8
    zLanding = 1.8 # altitude of the landing
    landingCompletedAlt = 0.15
    Vmax_phase1 = 0.5
    Vmax_phase2 = 0.5
    lim_acceleration_diff = 0.05
    accelerationDuration = 1
    sizeErrorList = 20 # time in refreshTime which the drone has to wait befor going from Phase 3 to Phase 4
    toleratePositionError = 0.25
    toleratePositionError_changePhase = 0.40
    tolerateAltitudeError = 0.35
#    velocityFasterShip = np.min([1 + 0.8/np.max(np.abs(shipVelocity)), 1.8])
    velocityFasterShip = 3#np.max([1 + 0.8/np.max(np.abs(shipVelocity)), 2])
    maxVelocityDiff = 0.7
    
#    %%%%%%%%%%%%%%% RELATIVE POSITION OF THE SHIP TO THE DRONE %%%%%%%%%%%%%%%
    distance_shipDrone2D = np.sqrt((x_drone - shipPosition[0])**2+(y_drone - shipPosition[1])**2) # distance between the ship and the drone in 2D (altitude does not mater)
    distance_droneShipDirection = np.abs(shipDirection[0]*x_drone + shipDirection[1]*y_drone + shipDirection[2])/np.sqrt(shipDirection[0]**2 + shipDirection[1]**2) # distance between the ship direction (a line) and the drone in 2D (altitude does not mater)
    
#    %%%%%%%%%%%%%%% ELEMENTS OF THE CHECKPOINT %%%%%%%%%%%%%%%
    zCheckPoint = zFirstPhases # altitude of Point C
    checkpoint = FindCheckPoint(shipDirection, zCheckPoint, r, dictionnary) # function in order to find the checkPoint
    checkPoint = checkpoint
    distance_droneCheckpoint2D = np.sqrt((x_drone-checkpoint[0])**2+(y_drone-checkpoint[1])**2) # distance between the checkpoint and the drone
    
    if phase == -1:
#        startAltitudeValues = [status, x_droneStart, y_droneStart, z_droneStart, velocity];
        if z_drone <= zFirstPhases - tolerateAltitudeError:
            startAltitudeValues[4] = -1
            startAltitudeValues[5] = (startAltitudeValues[3] + zFirstPhases)/2
            startAltitudeValues[6] = -(startAltitudeValues[3] - zFirstPhases)/2
        elif z_drone >= zFirstPhases + tolerateAltitudeError:
            startAltitudeValues[4] = 1
            startAltitudeValues[5] = (startAltitudeValues[3] + zFirstPhases)/2
            startAltitudeValues[6] = (startAltitudeValues[3] - zFirstPhases)/2
        else:
            phase = 0
            startAltitudeValues = np.array([0, 0, 0, 0, 0, 0, 0])
            positionShip_list = np.array([])
            startPosition[2] = zFirstPhases
            dictionnary['startPosition'] = startPosition
            
            p = 'phase = ' + repr(phase)                        #%info
            print(p)                                            #%info
            p = 't = ' + repr(t)                                #%info
            print(p)                                            #%info  
            print('')
            
            
    else:
        startAltitudeValues = np.array([0, 0, 0, 0, 0, 0, 0])
        
#    %%%%%%%%%%%%%%% DETERMINATION OF THE TRAJECTORY %%%%%%%%%%%%%%%
        if positionShip_list.shape[1] <= 20: # When the movement is starting (the drone does not know the real direction of the ship (default settings (d) : y = x), the drone stay in its position
            Vdrone = np.array([0, 0, 0])
            newPhase = 0
            if positionShip_list.shape[1] >= 10:
                if positionShip_list.shape[1] >= 10:
                    proj_startPosition = Phase1Point_A(x_drone, y_drone, np.hstack((shipDirection, shipDirection)), dictionnary)
                    startPosition = np.array([proj_startPosition[0], proj_startPosition[1], zFirstPhases])
            
            
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        else:
            if phase == 5:
                newPhase = 5
                Vuav = np.array([shipVelocity[0], shipVelocity[1]])
                Vdrone = np.array([Vuav[0], Vuav[1], 0])
                if z_drone <= landingCompletedAlt:
                    if landingCompleted == 0:
                        #% stop engines
                        print('Landing completed, STOP ENGINES t = ' + repr(t))
                        print('Final position of the ship = ' + repr(shipPosition))
                    landingCompleted = 1                
                
            elif phase == 4 or phase == 41 or phase == 42: # The drone has to go to Point D
                if np.abs(z_drone - zLanding) <= tolerateAltitudeError or phase == 41 or phase == 42:

                    
                    #    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 42 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    if distance_shipDrone2D >= toleratePositionError_changePhase and phase != 41:
                        newPhase = 42
                        if abs(shipVelocity[0]) >= abs(shipVelocity[1]):
                            if abs(velocityFasterShip*shipVelocity[0]) >= maxVelocityDiff:
                                velocityFasterShip = abs((shipVelocity[0] + maxVelocityDiff)/shipVelocity[0])
                        else:
                            if abs(velocityFasterShip*shipVelocity[1]) >= maxVelocityDiff:
                                velocityFasterShip = abs((shipVelocity[1] + maxVelocityDiff)/shipVelocity[1])
                                
                        Vuav = np.array([velocityFasterShip*shipVelocity[0], velocityFasterShip*shipVelocity[1]])
                        Vdrone = np.array([Vuav[0], Vuav[1], 0])
                        
                        
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 41 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    else:
                        Vuav = np.array([shipVelocity[0], shipVelocity[1]])
                        Vdrone = np.array([Vuav[0], Vuav[1], 0])
                        newPhase = 41
                        errorPhase4 = np.sqrt((shipPosition[0]-x_drone)**2 + (shipPosition[1]-y_drone)**2 + (zLanding-z_drone)**2)
                        errorPhase4 = np.array([errorPhase4])
                        if errorPhase4_list.shape[0] < sizeErrorList:
                            errorPhase4_list = np.hstack((errorPhase4_list, errorPhase4))
                        else:
                            for k in range(sizeErrorList-1):
                                errorPhase4_list[k] = errorPhase4_list[k+1]
                            errorPhase4_list[-1] = errorPhase4
                            
                            nError = 0
                            for k in range(errorPhase4_list.shape[0]):
                                if errorPhase4_list[k] <= toleratePositionError:
                                    nError = nError + 1;
                            if nError == sizeErrorList and accelerationValuesX[0] == 0 : # the drone is on Point D for ? seconds, the Landing mode can start
                                newPhase = 5 # landing
                    
                    
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                else: # the drone has to decrease its altitude
                    Vuav = np.array([shipVelocity[0], shipVelocity[1]])
                    Vdrone = np.array([Vuav[0], Vuav[1], 0])
                    newPhase = 4                
                
#            elif (distance_shipDrone2D >= distance_droneCheckpoint2D) or (phase == 3) or (phase == 31): # the drone is in Phase 3
#
#
##    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                if distance_droneCheckpoint2D >= toleratePositionError_changePhase:
##                    p = 'distance_droneCheckpoint2D = ' + repr(distance_droneCheckpoint2D)
##                    print(p)
##                    p = 'distance_droneShipDirection = ' + repr(distance_droneShipDirection)
##                    print(p)
#                    newPhase = 3
#                    Vuav = np.array([velocityFasterShip*shipVelocity[0], velocityFasterShip*shipVelocity[1]])
#                    Vdrone = np.array([Vuav[0], Vuav[1], 0])
##                    print(Vdrone)
#                    
#                    if phase == 31:
#                        newPhase = 31
#                        Vuav = np.array([shipVelocity[0], shipVelocity[1]])
#                        Vdrone = np.array([Vuav[0], Vuav[1], 0])
#                    
#                    
##    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 31 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                else:
##                    print(distance_droneCheckpoint2D)
#                    newPhase = 31
#                    Vuav = np.array([shipVelocity[0], shipVelocity[1]])
#                    Vdrone = np.array([Vuav[0], Vuav[1], 0])
#                    errorCheckpoint = np.sqrt((checkpoint[0]-x_drone)**2 + (checkpoint[1]-y_drone)**2 + (zCheckPoint-z_drone)**2);
#                    errorCheckpoint = np.array([errorCheckpoint])
#                    if errorCheckpoint_list.shape[0] < sizeErrorList:
#                        errorCheckpoint_list = np.hstack((errorCheckpoint_list, errorCheckpoint))
#                    else:
#                        for k in range(sizeErrorList - 2):
#                            errorCheckpoint_list[k] = errorCheckpoint_list[k+1]
#                        errorCheckpoint_list[-1] = errorCheckpoint
#                        nError = 0
#                        for k in range(errorCheckpoint_list.shape[0]):
#                            if errorCheckpoint_list[k] <= toleratePositionError: # acceptable distance for holding position
#                                nError = nError + 1
#                        if nError == sizeErrorList:
#                            newPhase = 4 # the drone starts Phase 4
#                            Vuav = np.array([shipVelocity[0], shipVelocity[1]])
#                            Vdrone = np.array([Vuav[0], Vuav[1], 0])
#                             
                            
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elif phase == 0: # the program is starting
                secureDistance = 1.5*r
                phase012 = Phase012(distance_droneShipDirection, secureDistance, shipDirection, x_drone, y_drone, Vmax_phase1, Vmax_phase2, dictionnary)
                newPhase = phase012[0]
                Vuav = np.array([phase012[1], phase012[2]])
                Vdrone = np.array([Vuav[0], Vuav[1], 0])
                newPhase = 4
                Vdrone = np.array([0,0,0])
                
                
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elif phase == 1:
                secureDistance = 1.5*r
                phase012 = Phase012(distance_droneShipDirection, secureDistance, shipDirection, x_drone, y_drone, Vmax_phase1, Vmax_phase2, dictionnary)
                newPhase = phase012[0]
                Vuav = np.array([phase012[1], phase012[2]])
                Vdrone = np.array([Vuav[0], Vuav[1], 0])
                
                
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PHASE 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elif phase == 2: # Phase 2 : the drone has to go to the projection of the checkpoint on the line which is parallele to the ship direction Point B
                secureDistance = 1*r
                phase012 = Phase012(distance_droneShipDirection, secureDistance, shipDirection, x_drone, y_drone, Vmax_phase1, Vmax_phase2, dictionnary)
                newPhase = phase012[0]
                Vuav = np.array([phase012[1], phase012[2]])
                Vdrone = np.array([Vuav[0], Vuav[1], 0])
                
                
#    %%%%%%%%%%%%%%% CHANGE OF PHASE %%%%%%%%%%%%%%%
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (newPhase - phase) != 0:
            startT = t
            previousPhase = phase
            phase = newPhase
            if phase <= 4:
                startPosition = np.array([x_drone, y_drone, zFirstPhases])
            else:
                startPosition = np.array([x_drone, y_drone, zLanding])
            newVelocity = Vdrone
            droneVelocity = Vdrone
            
            if previousPhase == 0 and phase == 3:
                if np.abs(shipVelocity[0]) <= np.abs(shipVelocity[1]):
                    K = np.abs(shipVelocity[0]/shipVelocity[1])
                    Vy = -np.sign(shipVelocity[1])*Vmax_phase2
                    Vx = -np.sign(shipVelocity[0])*np.abs(K*Vy)
                    previousVelocity = np.array([Vx, Vy, 0])
                    dictionnary['previousVelocity'] = previousVelocity
                else:
                    K = np.abs(shipVelocity[1]/shipVelocity[0])
                    Vx = -np.sign(shipVelocity[0])*Vmax_phase2
                    Vy = -np.sign(shipVelocity[1])*np.abs(K*Vx)
                    previousVelocity = np.array([Vx, Vy, 0])
                    dictionnary['previousVelocity'] = previousVelocity
            
#    %%%%%%%%%%%%%%% CALCULS FOR TURNVALUES %%%%%%%%%%%%%%%
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if phase == 3:
                angle = np.pi
                
                R = distance_droneShipDirection/2
                
                P = Phase1Point_A(x_drone, y_drone, np.hstack((shipDirection, shipDirection)), dictionnary)
                O = np.array([((x_drone + P[0])/2), ((y_drone + P[1])/2)])
                
                x0 = O[0]
                y0 = O[1]
                startAngle = np.sign((y_drone - y0)/R)*np.arccos((x_drone - x0)/R)
                if np.sin(startAngle) == 0:
                    side = np.sign((previousVelocity[1]/np.linalg.norm(previousVelocity))*(1/(R*np.cos(startAngle))))
                else:
#                    print('previousVelocity')
#                    print(previousVelocity)
#                    print('startAngle')
#                    print(startAngle)
                    side = np.sign(-(previousVelocity[0]/np.linalg.norm(previousVelocity))*(1/(R*np.sin(startAngle))))
                if np.abs(np.sin(startAngle)) <= np.abs(np.cos(startAngle)):
                    limitVelocity = np.max(np.abs(previousVelocity))/(R*np.abs(np.cos(startAngle)))
                else:
                    limitVelocity = np.max(np.abs(previousVelocity))/(R*np.abs(np.sin(startAngle)));
            else:
                if phase > 3:
                    angle = 0 
                else:
                    if np.all(shipVelocity == 0) or np.all(previousVelocity == 0):
                        angle = 0
                    else:
                        costheta = np.dot(previousVelocity,newVelocity)/(np.linalg.norm(previousVelocity)*np.linalg.norm(newVelocity))
                        angle = np.arccos(costheta)
                
                R = 0.5
                
                O = np.array([x_drone + R*(-shipVelocity[0]/np.linalg.norm(shipVelocity)), y_drone + R*(-shipVelocity[1]/np.linalg.norm(shipVelocity))])
                
                if side == 0:
                    scal = np.cross(previousVelocity, newVelocity)
                    side = np.sign(scal[2]);
                
                x0 = O[0]
                y0 = O[1]
                startAngle = np.sign((y_drone - y0)/R)*np.arccos((x_drone - x0)/R)
                
                xO = x_drone - (R*np.cos(startAngle))
                yO = y_drone - (R*np.sin(startAngle))
                O = np.array([xO, yO]) # center of the circle for turn
                
                if np.abs(np.sin(startAngle)) <= np.abs(np.cos(startAngle)):
                    limitVelocity = np.max(np.abs(previousVelocity))/(R*np.abs(np.cos(startAngle)))
                else:
                    limitVelocity = np.max(np.abs(previousVelocity))/(R*np.abs(np.sin(startAngle)))
            
            if previousPhase == 1 and phase == 3:
                angle = 3*np.pi/2
                
                R = distance_droneShipDirection
                
                O = np.array([x_drone + R*(-shipVelocity[0]/np.linalg.norm(shipVelocity)), y_drone + R*(-shipVelocity[1]/np.linalg.norm(shipVelocity))])
                
                x0 = O[0]
                y0 = O[1]
                
                startAngle = np.sign((y_drone - y0)/R)*np.arccos((x_drone - x0)/R)
                
                if np.sin(startAngle) == 0:
                    side = np.sign((previousVelocity[2]/np.linalg.norm(previousVelocity))*(1/(R*np.cos(startAngle))))
                else:
                    side = np.sign(-(previousVelocity[0]/np.linalg.norm(previousVelocity))*(1/(R*np.sin(startAngle))));
                
                if np.abs(np.sin(startAngle)) <= np.abs(np.cos(startAngle)):
                    limitVelocity = np.max(np.abs(previousVelocity))/(R*np.abs(np.cos(startAngle)))
                else:
                    limitVelocity = np.max(np.abs(previousVelocity))/(R*np.abs(np.sin(startAngle)));
            
            if limitVelocity == 0:
                limitVelocity =1
                
            turnValues = np.array([t, np.abs(angle), O[0], O[1], startAngle, side, zFirstPhases, R, limitVelocity])
            
            if phase == 2 or phase == 3:
                finalTime = turnValues[0] + turnValues[1]/turnValues[8];
            else :
                finalTime = -1
            
            if angle == 0:
                turnValues[1] = -1
            
            if phase == 0 or phase >= 4: # there is no turn during these phases
                turnValues[1] = -1
            
#    %%%%%%%%%%%%%%% CALCULS FOR ACCELERATIONVALUES %%%%%%%%%%%%%%%
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            security = dictionnary['security']
            if np.abs((np.linalg.norm(newVelocity[0:2]) - np.linalg.norm(previousVelocity[0:2]))) > lim_acceleration_diff and accelerationValuesX[0] == 0:
                truefalse = 1
                
                if phase == 41:
#                    proj_startPosition = Phase1Point_A(x_drone, y_drone, np.hstack((shipDirection, shipDirection)))
                    accelerationDuration = 2*np.abs(security[0] - shipPosition[0])/np.abs(newVelocity[0]  - previousVelocity[0])
                a_x = (newVelocity[0] - previousVelocity[0])/accelerationDuration
                a_y = 0
                a_z = 0                
                start_V = previousVelocity
                start_V[2] = 0                
                if t < turnValues[0] + turnValues[1]/turnValues[8]:
                    tStart = t + turnValues[1]/turnValues[8]
                    tFinal = tStart + accelerationDuration
                else:
                    tStart = t
                    tFinal = tStart + accelerationDuration
                if phase >= 41:
                    z_acceleration = zLanding
                else:
                    z_acceleration = zFirstPhases
                    
                afterAcceleration_X = (security[0] + accelerationDuration*(newVelocity[0] + previousVelocity[0])/2)
                accelerationValuesX = np.array([truefalse, tStart, tFinal, a_x, 0, a_z, start_V[0], 0, start_V[2], security[0], 0, z_acceleration, accelerationDuration, afterAcceleration_X, newVelocity[0]])
            ##############################
            ##############################
                if phase == 41:
                    accelerationDuration = 2*np.abs(security[1] - shipPosition[1])/np.abs(newVelocity[1]  - previousVelocity[1])
                a_x = 0
                a_y = (newVelocity[1] - previousVelocity[1])/accelerationDuration
                a_z = 0                
                start_V = previousVelocity
                start_V[2] = 0                
                if t < turnValues[0] + turnValues[1]/turnValues[8]:
                    tStart = t + turnValues[1]/turnValues[8]
                    tFinal = tStart + accelerationDuration
                else:
                    tStart = t
                    tFinal = tStart + accelerationDuration
                if phase >= 41:
                    z_acceleration = zLanding
                else:
                    z_acceleration = zFirstPhases
                    
                afterAcceleration_Y = (security[1] + accelerationDuration*(newVelocity[1] + previousVelocity[1])/2)
                accelerationValuesY = np.array([truefalse, tStart, tFinal, 0, a_y, a_z, 0, start_V[1], start_V[2], 0, security[1], z_acceleration, accelerationDuration, afterAcceleration_Y, newVelocity[1]])
            else:
                truefalse = 0
                accelerationValuesX = np.array([truefalse, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1])
                accelerationValuesY = np.array([truefalse, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1])
            
#    %%%%%%%%%%%%%%% CALCULS FOR LANDING %%%%%%%%%%%%%%%
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
            if phase == 5 and landingValues.shape[0] == 0:
                
                t0 = t
                v1 = -0.5
                v2 = -0.15
                h3 = 0
                h2 = 0.2
                h0 = z_drone
                h1 = (h0 - h2)/2
    
                k13 = h0
                k12 = 0
                k11 = (v1**2)/(2*h1 - 2*k13)
#                %t1 = (v1 - k12)/k11;
                t1 = (2*h1 - 2*k13)/v1
    
                k21 = -k11
                k22 = v1 - k21*t1
#                %t2 = (v2 - k22)/k21;
                k23 = -0.5*k21*t1**2 - t1*k22 + h1
                delta = k22**2 - 2*k21*(k23 - h2)
                t2 = min((-k22 - np.sqrt(delta))/k21, (-k22 + np.sqrt(delta))/k21)
    
    
                k31 = 0
                k32 = v2
                k33 = h2 - k32*t2
                t3 = (h3 - k33)/k32
    
                t1Landing = t1
                t2Landing = t2
                t3Landing = t3
#                %xLand = desiredPosition(1);
#                %yLand = desiredPosition(2);
                landingValues = np.array([t0, t1Landing, t2Landing, t3Landing, k11, k12, k13, k21, k22, k23, k31, k32, k33, h3])
                
                finalTime = t + t3Landing

#    %%%%%%%%%%%%%%% INFO %%%%%%%%%%%%%%%
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            p = 'phase = ' + repr(phase)                                #%info
            print(p)                                                    #%info
            p = 'droneVelocity = ' + repr(droneVelocity)                #%info
            print(p)                                                    #%info
            p = 'turnValues = ' + repr(turnValues)                      #%info
            print(p)                                                    #%info
            if finalTime != -1:
                p = 'finalTime = ' + repr(finalTime)                    #%info
                print(p)                                                #%info
            if phase == 3 or phase == 31 or phase == 41 or phase == 42:
                p = 'accelerationValuesX = ' + repr(accelerationValuesX)  #%info
                print(p)                                                #%info
                p = 'accelerationValuesY = ' + repr(accelerationValuesY)  #%info
                print(p)                                                #%info
            if landingValues.shape[0] != 0:
                p = 'landingValues = ' + repr(landingValues)        #%info
                print(p)                                            #%info
            print('')
    
    dictionnary['shipDirection'] = shipDirection
    dictionnary['phase'] = phase
    dictionnary['droneVelocity'] = droneVelocity
    dictionnary['previousPhase'] = previousPhase
    dictionnary['turnValues'] = turnValues
    dictionnary['startPosition'] = startPosition
    dictionnary['startT'] = startT
    dictionnary['side'] = side
    accelerationValuesX = dictionnary['accelerationValuesX']
    accelerationValuesY = dictionnary['accelerationValuesY']
    dictionnary['startAltitudeValues'] = startAltitudeValues
    dictionnary['previousVelocity'] = previousVelocity
    dictionnary['landingValues'] = landingValues
    dictionnary['landingCompleted'] = landingCompleted
    dictionnary['checkPoint'] = checkPoint
    dictionnary['errorCheckpoint_list'] = errorCheckpoint_list
    dictionnary['errorPhase4_list'] = errorPhase4_list
    
    
    return 0

def desired_pos(t, x, dictionnary, x_ship):
#    R = np.reshape(X[6:15],(3,3));  # rotation from body to inertial
#    W = X[15:];   # angular rate
#    x = X[:3];  # position
#    v = X[3:6];    # velocity

    xd = np.array([x[0], x[1], x[2]])
    xd_dot = np.array([0, 0, 0])
    xd_ddot = np.array([0, 0, 0])
    xd_dddot = np.array([0, 0, 0])
    xd_ddddot = np.array([0, 0, 0])
    b1d = np.array([1., 0., 0.])
    b1d_dot=np.array([0., 0., 0.])
    b1d_ddot=np.array([0., 0., 0.])
    Rd = np.eye(3)
    Wd = np.array([0.,0.,0.])
    Wd_dot = np.array([0.,0.,0.])
    f = np.array([0,0,0])
    M = np.array([0,0,0])
    
    positionShip_list = dictionnary['positionShip_list']
    startAltitudeValues = dictionnary['startAltitudeValues']
    landingValues = dictionnary['landingValues']
    accelerationValuesX = dictionnary['accelerationValuesX']
    accelerationValuesY = dictionnary['accelerationValuesY']
    startT = dictionnary['startT']
    turnValues = dictionnary['turnValues']
    startPosition = dictionnary['startPosition']
    checkPoint = dictionnary['checkPoint']
    phase = dictionnary['phase']
    droneVelocity = dictionnary['droneVelocity']
    desiredPosition= dictionnary['desiredPosition']
    pointT1= dictionnary['pointT1']
    refreshTime = dictionnary['refreshTime']
    shipDirection = ShipDirection(dictionnary)
    zFirstPhases = 1.8
    zLanding = 1.8 # altitude of the landing
    limitVelocity = 0.5;
    Xmax = 1.70
    Ymax = 2.9
    
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                                                                                                         %
    deltaT1 = t - pointT1 # 0.05 seconde                                                                 %
    if deltaT1 >= refreshTime:
        pointT1 = t
        dictionnary['pointT1'] = pointT1
        shipPosition = x_ship#ShipPosition(t, dictionnary) # simulation of the ship position
        UpdatePositionShip_list(t, shipPosition, dictionnary) #  update the ship position lsit
        positionShip_list = dictionnary['positionShip_list']
        
        if phase != 666:
            positionShip_list = dictionnary['positionShip_list']
        else:
            print('SECURITY STOP')
        TrajectoryCreation(t, desiredPosition[0], desiredPosition[1], desiredPosition[2], dictionnary);
    if x[0] >= Xmax or x[1] >= Ymax or phase == 666:
        phase = 666
        dictionnary['phase'] = phase
        security = dictionnary['security']
        xd = np.array([security[0], security[1], (security[2] - 0.1*(t - security[3]))])
        xd_dot = np.array([0, 0, -0.1])
        xd_ddot = np.array([0, 0, 0])
    else:
                
        if t < turnValues[0] + turnValues[1]/turnValues[8]: # TURN
    #         turnValues = [t, angle, Ox, Oy, startAngle, side, z_drone, R, limitVelocity];
            xd = np.array([turnValues[2] + turnValues[7]*np.cos(turnValues[4] + turnValues[5]*turnValues[8]*(t - turnValues[0])), turnValues[3] + turnValues[7]*np.sin(turnValues[4] + turnValues[5]*turnValues[8]*(t - turnValues[0])), turnValues[6]])
            xd_dot = np.array([-turnValues[5]*turnValues[7]*turnValues[8]*np.sin(turnValues[4] + turnValues[5]*turnValues[8]*(t - turnValues[0])), turnValues[5]*turnValues[7]*turnValues[8]*np.cos(turnValues[4] + turnValues[5]*turnValues[8]*(t - turnValues[0])), 0])
            xd_ddot = np.array([-turnValues[7]*turnValues[8]*turnValues[8]*np.cos(turnValues[4] + turnValues[5]*turnValues[8]*(t - turnValues[0])), -turnValues[7]*turnValues[8]*turnValues[8]*np.sin(turnValues[4] + turnValues[5]*turnValues[8]*(t - turnValues[0])), 0])
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
            
            
        elif accelerationValuesX[0] == 1 and (t < accelerationValuesX[2] or t < accelerationValuesY[2]) and (phase == 3 or phase == 31 or phase == 41 or phase == 42): # ACCELERATION
    #         accelerationValues = [truefalse, tStart, tFinal, a_x, a_y, a_z, start_V(1,1), start_V(1,2), start_V(1,3), x_drone, y_drone, z_drone, accelerationDurationafterAcceleration_X, newVelocity[0]])
            if t < accelerationValuesX[2]:
                xd_ddot = accelerationValuesX[3:6]
                xd_dot = (accelerationValuesX[6:9] + xd_ddot*(t - accelerationValuesX[1]))
                xd = accelerationValuesX[9:12] + accelerationValuesX[6:9]*(t - accelerationValuesX[1]) + 0.5*accelerationValuesX[3:6]*((t - accelerationValuesX[1])**2)
            else:
                xd_ddot = np.array([0, 0, 0])
                xd_dot = np.array([accelerationValuesX[-1], 0, 0])
                xd = np.array([accelerationValuesX[-1]*(t - (accelerationValuesX[1] + accelerationValuesX[-3])) + accelerationValuesX[-2], 0, 0])
            
            if t < accelerationValuesY[2]:
                xd_ddot = xd_ddot + np.array([0, 1, 0])*(accelerationValuesY[3:6])
                xd_dot = xd_dot + np.array([0, 1, 0])*(accelerationValuesY[6:9] + xd_ddot*(t - accelerationValuesY[1]))
                xd = xd + np.array([0, 1, 0])*(accelerationValuesY[9:12] + accelerationValuesY[6:9]*(t - accelerationValuesY[1]) + 0.5*accelerationValuesY[3:6]*((t - accelerationValuesY[1])**2))
            else:
                xd_ddot = xd_ddot + np.array([0, 0, 0])
                xd_dot = xd_dot + np.array([0, 1, 0])*(np.array([0, accelerationValuesY[-1], 0]))
                xd = xd + np.array([0, 1, 0])*(np.array([0, accelerationValuesY[-1]*(t - (accelerationValuesY[1] + accelerationValuesY[-3])) + accelerationValuesY[-2], 0]))
                
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
    #        S = Phase1Point_A(xd[0], xd[1], np.hstack((dictionnary['shipDirection'], dictionnary['shipDirection'])))
            startPosition = xd#np.array([S[0], S[1], x[2]])
#            proj_startPosition = Phase1Point_A(xd[0], xd[1], np.hstack((shipDirection, shipDirection)))
#            startPosition = np.array([proj_startPosition[0], proj_startPosition[1], zLanding])
            dictionnary['startPosition'] = startPosition
            startT = t
            dictionnary['startT'] = startT
            
            
    #    elif phase == 1:
    #        startPosition[2] = zFirstPhases
    #        dictionnary['startPosition'] = startPosition
    #        xd = (startPosition + droneVelocity*(t - startT))
    #        xd_dot = droneVelocity
    #        xd_ddot = np.array([0, 0, 0])
    #        
    #        
    #    elif phase == 2:
    #        startPosition[2] = zFirstPhases
    #        dictionnary['startPosition'] = startPosition
    #        xd = (startPosition + droneVelocity*(t - startT))
    #        xd_dot = droneVelocity
    #        xd_ddot = np.array([0, 0, 0])
    #        
    #        
    #    elif phase == 3:
    #        accelerationValues[0] = 0
    #        startPosition[2] = zFirstPhases
    #        dictionnary['startPosition'] = startPosition
    #        xd = (startPosition + droneVelocity*(t - startT))
    #        xd_dot = droneVelocity
    #        xd_ddot = np.array([0, 0, 0])
    #        
    #        
    #    elif phase == 31: # stay on checkPoint
    #        accelerationValues[0] = 0
    #        xd = checkPoint
    #        xd_dot = droneVelocity
    #        xd_ddot = np.array([0, 0, 0])
            
    
        elif phase == 42:
#            proj_startPosition = Phase1Point_A(xd[0], xd[1], np.hstack((shipDirection, shipDirection)))
#            startPosition = np.array([proj_startPosition[0], proj_startPosition[1], zLanding])
            accelerationValuesX[0] = 0
            accelerationValuesY[0] = 0
            dictionnary['startPosition'] = startPosition
            xd = (startPosition + droneVelocity*(t - startT))
            xd_dot = droneVelocity
            xd_ddot = np.array([0, 0, 0])
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
            
            
        elif phase == 41:
#            proj_startPosition = Phase1Point_A(xd[0], xd[1], np.hstack((shipDirection, shipDirection)), dictionnary)
#            startPosition = np.array([proj_startPosition[0], proj_startPosition[1], zLanding])
            accelerationValuesX[0] = 0
            accelerationValuesY[0] = 0
            xd = np.array([positionShip_list[0, -1], positionShip_list[1, -1], zLanding])
            xd_dot = droneVelocity
            xd_ddot = np.array([0, 0, 0])
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
            
            
        elif phase == 42:
#            proj_startPosition = Phase1Point_A(xd[0], xd[1], np.hstack((shipDirection, shipDirection)), dictionnary)
#            startPosition = np.array([proj_startPosition[0], proj_startPosition[1], zLanding])
            accelerationValuesX[0] = 0
            accelerationValuesY[0] = 0
            startPosition[2] = zLanding
            dictionnary['startPosition'] = startPosition
            xd = (startPosition + droneVelocity*(t - startT))
            xd_dot = droneVelocity
            xd_ddot = np.array([0, 0, 0])
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
            
    #    elif phase == 5: # landing duration = 11s
    ##         landingValues = [t0, t1Landing, t2Landing, t3Landing, k11, k12, k13, k21, k22, k23, k31, k32, k33, h3];
    #        if t <= startT + landingValues[1]:
    #            xd = np.array([startPosition[0], startPosition[1], 0.5*(landingValues[4]*(t - startT)**2) + landingValues[6]])
    #            xd_dot = np.array([0, 0, landingValues[4]*(t - startT)])
    #            xd_ddot = np.array([0, 0, landingValues[4]])
    #            
    #        elif t <= startT + landingValues[2]:
    #            xd = np.array([startPosition[0], startPosition[1], 0.5*(landingValues[7]*(t - startT)**2) + landingValues[8]*(t - startT) + landingValues[9]])
    #            xd_dot = np.array([0, 0, landingValues[7]*(t - startT) + landingValues[8]])
    #            xd_ddot = np.array([0, 0, landingValues[7]])
    #            
    #        elif t <= startT + landingValues[3]:
    #            xd = np.array([startPosition[0], startPosition[1], landingValues[11]*(t - startT) + landingValues[12]])
    #            xd_dot = np.array([0, 0, landingValues[11]])
    #            xd_ddot = np.array([0, 0, 0])
    #            
    #        else:
    #            xd = np.array([startPosition[0], startPosition[1], 0])
    #            xd_dot = np.array([0, 0, 0])
    #            xd_ddot = np.array([0, 0, 0])        
            
        elif phase == 5: # landing duration = 11s
    #         landingValues = [t0, t1Landing, t2Landing, t3Landing, k11, k12, k13, k21, k22, k23, k31, k32, k33, h3];
            if t <= startT + landingValues[1]:
                xd = np.array([positionShip_list[0,-1], positionShip_list[1,-1], 0.5*(landingValues[4]*(t - startT)**2) + landingValues[6]])
                xd_dot = np.array([droneVelocity[0], droneVelocity[1], landingValues[4]*(t - startT)])
                xd_ddot = np.array([0, 0, landingValues[4]])
                
            elif t <= startT + landingValues[2]:
                xd = np.array([positionShip_list[0, -1], positionShip_list[1,-1], 0.5*(landingValues[7]*(t - startT)**2) + landingValues[8]*(t - startT) + landingValues[9]])
                xd_dot = np.array([droneVelocity[0], droneVelocity[1], landingValues[7]*(t - startT) + landingValues[8]])
                xd_ddot = np.array([0, 0, landingValues[7]])
                
            elif t <= startT + landingValues[3]:
                xd = np.array([positionShip_list[0, -1], positionShip_list[1,-1], landingValues[11]*(t - startT) + landingValues[12]])
                xd_dot = np.array([droneVelocity[0], droneVelocity[1], landingValues[11]])
                xd_ddot = np.array([0, 0, 0])
                
            else:
                xd = np.array([positionShip_list[0,-1], positionShip_list[1,-1], 0])
                xd_dot = np.array([droneVelocity[0], droneVelocity[1], 0])
                xd_ddot = np.array([0, 0, 0])    
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
            
            
        elif phase == 0:
            startPosition[2] = zFirstPhases;
            dictionnary['startPosition'] = startPosition
            xd = startPosition
            xd_dot = np.array([0, 0, 0])
            xd_ddot = np.array([0, 0, 0])
            startT = t
            dictionnary['startT'] = startT
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security 
            
        elif phase == -1:
    #         startAltitudeValues = [status, x_droneStart, y_droneStart, z_droneStart, side, k, k']
            xd = np.array([startAltitudeValues[1], startAltitudeValues[2], startAltitudeValues[5] + startAltitudeValues[6]*startAltitudeValues[4]*np.sin(-limitVelocity*t + np.pi/2)])
            xd_dot = np.array([0, 0, -startAltitudeValues[6]*startAltitudeValues[4]*limitVelocity*np.cos(-limitVelocity*t + np.pi/2)])
            xd_ddot = np.array([0, 0, -startAltitudeValues[6]*startAltitudeValues[4]*limitVelocity*limitVelocity*np.sin(limitVelocity*t + np.pi/2)])
            startT = t
            dictionnary['startT'] = startT
            startPosition = xd
            dictionnary['startPosition'] = startPosition
            security = np.array([xd[0], xd[1], xd[2], t])
            dictionnary['security'] = security
#        pdb.set_trace()
                    
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%% FOR PLOTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#    
#    posTime = [posTime [x_drone, y_drone, z_drone]'];
#    desiredPosition = xd';
#    xdTime = [xdTime xd ];
#    tPlot = [tPlot, t];
#    xd_dotTime = [xd_dotTime xd_dot];
#    xd_ddotTime = [xd_ddotTime xd_ddot];
        previousVelocity = xd_dot
        dictionnary['previousVelocity'] = previousVelocity
        desiredPosition = xd
        dictionnary['desiredPosition'] = desiredPosition
    
    if np.linalg.norm(xd_dot[0:2]) != 0:
        b1d[0] = xd_dot[0]/np.linalg.norm(xd_dot[0:2])
        b1d[1] = xd_dot[1]/np.linalg.norm(xd_dot[0:2])
#    
#    if deltaT1 >= refreshTime
#        list_velocityUAV = [list_velocityUAV xd_dot];
#        list_desiredPos = [list_desiredPos, desiredPosition'];          % for ploting
#        UAVPosition =[x_drone, y_drone, z_drone];                       % for ploting
#        list_positionUAV = [list_positionUAV UAVPosition'];             % for ploting
#        list_positionShip = [list_positionShip shipPosition'];          % for ploting
#    end
#        
#    if deltaT1 >= refreshTime and acce == 1:
#        print (t)
#        print(xd_ddot)
#        print(xd_dot)
#        print(xd)
##                                                                                                         % 
#                                                                                                         %
#    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#    
#     xd = [0, 0, 0]';
#     xd_dot = [0, 0, 0]';
#     xd_ddot = [0, 0, 0]';
#    dictionnary['positionShip_list'] = positionShip_list
#    dictionnary['startAltitudeValues'] = startAltitudeValues
#    dictionnary['landingValues'] = landingValues
#    dictionnary['accelerationValues'] = accelerationValues
#    dictionnary['startT'] = startT
#    dictionnary['turnValues'] = turnValues
#    dictionnary['startPosition'] = startPosition
#    dictionnary['checkPoint'] = checkPoint
#    dictionnary['phase'] = phase
#    dictionnary['droneVelocity'] = droneVelocity
#    dictionnary['desiredPosition'] = desiredPosition
#    dictionnary['pointT1'] = pointT1
#    dictionnary['refreshTime'] = refreshTime
    
#    uav_t.dictionnary = dictionnary

#    (f, M) = uav_t.position_control(t, R, W, x, v, d_in)
#
#    R_dot = np.dot(R,hat(W))
#    W_dot = np.dot(la.inv(uav_t.J), M - np.cross(W, np.dot(uav_t.J, W)))
#    x_dot = v
#    v_dot = uav_t.g*uav_t.e3 - f*R.dot(uav_t.e3)/uav_t.m
#    uav_t.v_dot = v_dot
#    uav_t.xd_ddot = xd_ddot
#    X_dot = np.concatenate((x_dot, v_dot, R_dot.flatten(), W_dot))
#    uav_t.xd = xd
#    uav_t.xd_dot = xd_dot
#    uav_t.command = np.insert(M,0,f)
#    if deltaT1 >= refreshTime and acce == 1:
#        print (t)
#        print(v_dot)
#        print(xd_ddot)
#        print(xd)
    return [xd, xd_dot, xd_ddot, b1d]



#ukf_flag = False
#anim_flag = True
##J = np.diag([0.0820, 0.0845, 0.1377])
#J = np.diag([9.773E-3, 9.773E-3, 1.749E-2])#LAB DRONE
#e3 = np.array([0.,0.,1.])
#uav_t = UAV(J, e3)
##uav_t.
#uav_t.dictionnary = dictionnary
#t_max = 140
#N = 100*t_max + 1
#t = np.linspace(0,t_max,N)
#xd = np.array(x0)
## Initial Conditions
#R0 = [[1., 0., 0.],
#        [0., 1, 0],
#        [0., 0, 1]] # initial rotation
#R0 = np.eye(3)
#W0 = [0.,0.,0.];   # initial angular velocity
##x0 = [0.,0.,0.];  # initial position (altitude?0)
#v0 = [0.,0.,0.];   # initial velocity
#R0v = np.array(R0).flatten().T
#y0 = np.concatenate((x0, v0, R0v, W0))
#
## sim = odeint(uav_t.dydt,y0,t)
#
#solver = ode(desired_pos)
#solver.set_integrator('dopri5').set_initial_value(y0, 0)
#dt = 1./30
#sim = []
#xd = []
#xd_dot = []
#xd_ddot = []
#v_dot = []
#command_val = []
#while solver.successful() and solver.t < t_max:
#    solver.set_f_params(uav_t)
#    solver.integrate(solver.t+dt)
#    sim.append(solver.y)
#    xd.append(uav_t.xd)
#    xd_dot.append(uav_t.xd_dot)
#    xd_ddot.append(uav_t.xd_ddot)
#    v_dot.append(uav_t.v_dot)
#    command_val.append(uav_t.command)
#
#sim = np.array(sim)
#xd = np.array(xd)
#xd_dot = np.array(xd_dot)
#xd_ddot = np.array(xd_ddot)
#v_dot = np.array(v_dot)
## def animate(i):
##fig = plt.figure()
##ax = fig.gca(projection = '3d')
##ax.set_aspect('equal')
##ax.plot(sim[:,0],sim[:,1],sim[:,2])
##ax.set_xlim(0, 10)
##ax.set_ylim(-5, 5)
##ax.set_zlim(-5, 5)
##plt.show()
#if anim_flag:
#    f, ax = plt.subplots(3,4)
#    ax[0][0].plot(xd[:,0], 'r')
#    ax[0][0].plot(sim[:,0])
#    ax[1][0].plot(xd_dot[:,0], 'r')
#    ax[1][0].plot(sim[:,3])
#
#    ax[0][1].plot(xd[:,1], 'r')
#    ax[1][1].plot(xd_dot[:,1], 'r')
#    ax[0][1].plot(sim[:,1])
#    ax[1][1].plot(sim[:,4])
#
#    ax[0][2].plot(xd[:,2], 'r')
#    ax[1][2].plot(xd_dot[:,2], 'r')
#    ax[0][2].plot(sim[:,2])
#    ax[1][2].plot(sim[:,5])
#
#    ax[2][0].plot(xd_ddot[:,0], 'r')
#    ax[2][0].plot(v_dot[:,0])
#    ax[2][1].plot(xd_ddot[:,1], 'r')
#    ax[2][1].plot(v_dot[:,1])
#    ax[2][2].plot(xd_ddot[:,2], 'r')
#    ax[2][2].plot(v_dot[:,2])
#
#    ax[2][3].plot(xd[:,0], xd[:,1], 'r')
#    ax[2][3].plot(sim[:,0], sim[:,1])
#    
#    ax[0][3].plot(xd[:,0]-sim[:,0])
#    ax[1][3].plot(xd[:,1]-sim[:,1])
#    
#    plt.show()
#    pass
