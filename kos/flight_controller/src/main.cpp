#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <future>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

#define EARTH_RADIUS 6371000


#define ALLOWED_DISTANCE 3

#define MAX_SPEED 2.5
#define MIN_SPEED 0.5
#define TARGET_SPEED 2

#define MAX_ALT 250
#define MIN_ALT 150
#define TARGET_ALT 200
#define HOME_ALT 7900 // somthing like this

#define DROP_RADIUS 2.5

using namespace std;

double distance2Point(CommandWaypoint point1, CommandWaypoint point2);
double degToRad(int32_t deg);



struct Coridor{
    CommandWaypoint point1 = CommandWaypoint(0,0,0);
    CommandWaypoint point2 = CommandWaypoint(0,0,0);

    Coridor(CommandWaypoint _point1,CommandWaypoint _point2){
         point1 = _point1;
         point2 = _point2;
    }

    double length(){
        return distance2Point(point1,point2);
    }

    double distanceToCoridor(CommandWaypoint point){
        double a = distance2Point (point, point1);
        double b = distance2Point (point, point2);
        double c = length();
        if(c*c>=(a*a+b*b)){
            double p = (a+b+c)/2;
            double h = 2*sqrt(p*(p-a)*(p-b)*(p-c))/c;
            return h;
        }else{
            if(a < b) return a;
            else return b;
        }
    }
};

void printCoords();
//void coridorCheck(vector<Coridor> coridors);
uint32_t coridorCheck(vector<CommandWaypoint> waypoints, uint32_t idAct);
double speedCheck();
void cargoLock(CommandWaypoint point);
void heightCheck();


int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);

    while (!signMessage(message, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response)) {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
}

int main(void) {
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);

    //Copter need to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    vector<CommandWaypoint> waypoints = vector<CommandWaypoint>();
    vector<Coridor> coridors = vector<Coridor>();
    CommandWaypoint dropWaypoint = CommandWaypoint(0,0,0);
    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();

            
            MissionCommand* commands = getCommands();
            uint32_t numCommands = getNumCommands();
            for (uint32_t i=0; i < numCommands; i++){
                if(commands[i].type == WAYPOINT or commands[i].type == HOME or commands[i].type == LAND){
                    waypoints.push_back(commands[i].content.waypoint);
                }
                if(commands[i].type == SET_SERVO){
                    dropWaypoint = commands[i-1].content.waypoint;
                    fprintf(stderr,"\n\n\n\n\n\nlatDROP: %d, ", dropWaypoint.latitude);
                    fprintf(stderr,"lonDROP: %d\n\n\n\n\n\n", dropWaypoint.longitude);
                }
            }
            // for (int i = 1; i < waypoints.size(); i++){
            //     Coridor coridor(waypoints[i-1], waypoints[i]);
            //     coridor.point1 = waypoints[i-1];
            //     coridor.point2 = waypoints[i];
            //     coridors.push_back(coridor);
            // }

            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    //The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        //When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true)) {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on
    //Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused

    uint32_t idActiveWaypoint = 0;

    while (true){

        
	    //printCoords();
        char respone[1024] = {0};
        sendSignedMessage("/api/arm", respone, "arm", RETRY_DELAY_SEC);
        //fprintf(stderr,"Arm Response: %s\n", respone);
        if (strstr(respone, "$Arm: 1#") != NULL) {
            pauseFlight();
            while (true){
                //sleep(0.5);
                sendSignedMessage("/api/arm", respone, "arm", RETRY_DELAY_SEC);
                if (strstr(respone, "$Arm: 0#") != NULL) {
                    resumeFlight();
                    break;
                }
            }
        }


        //dropWaypoint
        cargoLock(dropWaypoint);
        // CommandWaypoint point1(466144868,1428115903,0);
        // CommandWaypoint point2(466144359,1428119683,0);
        // fprintf(stderr,"lat1d: %d, ", point1.latitude);
        // fprintf(stderr,"lon1d: %d\n", point1.longitude);

        // fprintf(stderr,"lat2d: %d, ", point2.latitude);    
        // fprintf(stderr,"lon2d: %d\n", point2.longitude);

        // double d = distance2Point(point1, point2);
        // fprintf(stderr,"Distance check(29.5m): %lf\n", d);

    	double speed = speedCheck();
    	fprintf(stderr, "Speed: %lf\n", speed);
    	if (speed > MAX_SPEED)
		    changeSpeed(TARGET_SPEED);

	    if (speed > MIN_SPEED){
            idActiveWaypoint=coridorCheck(waypoints, idActiveWaypoint);
    
            heightCheck();
	    }
        
        //sleep(1); //speed check - 1 sec
    }

    return EXIT_SUCCESS;
}

double degToRad(int32_t deg){
    double r = double(deg)/10000000 * M_PI / 180.0;
    return r;
}

double distance2Point(CommandWaypoint point1, CommandWaypoint point2){
    double phi1 = degToRad(point1.latitude);
    double phi2 = degToRad(point2.latitude);

    double deltaPhi = phi2 - phi1;
    double deltaLambda = degToRad(point2.longitude) - degToRad(point1.longitude);

    // fprintf(stderr,"lat1d: %d, ", point1.latitude);
    // fprintf(stderr,"lon1d: %d\n", point1.longitude);

    // fprintf(stderr,"lat2d: %d, ", point2.latitude);    
    // fprintf(stderr,"lon2d: %d\n", point2.longitude);

    // fprintf(stderr,"lat1r: %20.17lf, ", phi1);
    // fprintf(stderr, "lat2r: %20.17lf\n", phi2);

    double d = asin(sqrt(pow(sin(deltaPhi/2),2) + cos(phi1) * cos(phi2) * pow(sin(deltaLambda/2),2)));

    return EARTH_RADIUS * d * 2;
}



void printCoords(){
    int32_t lat,lon,alt;
        if(getCoords(lat,lon,alt)){
            fprintf(stderr, "latitude: [%d]\n",lat);
            fprintf(stderr, "longitude: [%d]\n",lon);
            fprintf(stderr, "altitude: [%d]\n", alt);
        }
}


// void coridorCheck(vector<Coridor> coridors){
//     int32_t lat,lon,alt;
//     getCoords(lat,lon,alt);
//     CommandWaypoint drone = CommandWaypoint(lat,lon,alt);
//     double minDistance = coridors[0].distanceToCoridor(drone);
//     for(uint32_t i =1; i < coridors.size(); i++){
//         double d = coridors[i].distanceToCoridor(drone);
//         if (d < minDistance) minDistance = d;
//     }
//     if (minDistance > ALLOWED_DISTANCE) pauseFlight();

// }


uint32_t coridorCheck(vector<CommandWaypoint> waypoints, uint32_t idAct){
    fprintf(stderr,"\n\n IDAct = %d\n\n", idAct);
    int32_t lat,lon,alt;
    double distToStart, distToEnd, distToStartMoved, distToEndMoved;
    getCoords(lat,lon,alt);


    CommandWaypoint drone = CommandWaypoint(lat,lon,alt);
    distToStart = distance2Point(drone, waypoints[idAct]);
    distToEnd = distance2Point(drone, waypoints[idAct+1]);
    sleep(2);
    getCoords(lat,lon,alt);
    CommandWaypoint movedDrone = CommandWaypoint(lat,lon,alt);
    distToStartMoved = distance2Point(movedDrone, waypoints[idAct]);
    distToEndMoved = distance2Point(movedDrone, waypoints[idAct+1]);

    fprintf(stderr, "\n\n Dist End = %lf .............. Dist End Moved = %lf", distToEndMoved, distToEndMoved);
    fprintf(stderr, "\n\n Dist Start = %lf ............... Dist Start Moved = %lf", distToStart, distToStartMoved);

    Coridor activeCoridor = Coridor(CommandWaypoint(0,0,0), CommandWaypoint(0,0,0));
    if (idAct< waypoints.size()-1)
        activeCoridor = Coridor(waypoints[idAct], waypoints[idAct+1]);
    else
        activeCoridor = Coridor(waypoints[0], waypoints[1]);
    // if (activeCoridor.distanceToCoridor(drone) > ALLOWED_DISTANCE){
    //     //setKillSwitch(0);
    // }

    // if ((distToEnd+0.1)<distToEndMoved){
    //     //setKillSwitch(0);        
    // }
    if (distToStart>(distToStartMoved+0.1)){
        setKillSwitch(0);
    }

    
    if (distance2Point(drone, waypoints[idAct+1]) < ALLOWED_DISTANCE)
        return idAct + 1;
    else
        return idAct;


}

void cargoLock(CommandWaypoint point){
    int32_t lat,lon,alt;
    getCoords(lat,lon,alt);
    CommandWaypoint drone = CommandWaypoint(lat,lon,alt);
    double Dist_to_DP = distance2Point(drone, point);
    fprintf(stderr, "Distance to droppoint: %lf\n",Dist_to_DP);
    if (Dist_to_DP<=DROP_RADIUS){
        setCargoLock(1);
        fprintf(stderr, "Drop that shit\n");
    }else
        setCargoLock(0);
}

double speedCheck(){
    int32_t lat1, lat2, lon1, lon2 , alt1, alt2;
    getCoords(lat1, lon1, alt1);
    CommandWaypoint point1 = CommandWaypoint(lat1, lon1, alt1);
    sleep(1);
    getCoords(lat2, lon2, alt2);
    CommandWaypoint point2 = CommandWaypoint(lat2, lon2, alt2);

    double speed = distance2Point(point1, point2);

    return speed;
}

void heightCheck(){
    int32_t lat, lon, alt;
    getCoords(lat, lon, alt);
    alt -= HOME_ALT;
    if (alt > MAX_ALT) {
        changeAltitude(TARGET_ALT);
    }
    if (alt < MIN_ALT) {
        changeAltitude(TARGET_ALT);
    }
    return;
}