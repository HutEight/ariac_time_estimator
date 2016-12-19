//Created by Kevin Bradner 12/18/16

#include <stdio.h>      /* printf */
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <ctime>

bool timeElapsed(){
    using namespace std;

    clock_t request_time = clock();
    clock_t completion_time = findCompletionTime(requestedJoints)//pass in requested joints from the service message

    res.t_elapsed_secs = double(completion_time - request_time) / CLOCKS_PER_SEC;//is this correct syntax?

    return true;
}

std::chrono::system_clock::time_point findCompletionTime(vector<double> joints){
}

//TODO: define a service message that goes along with this
	//make sure it supplies the joint angles, preferably by reference for speed
	//make sure the response section includes a t_elapsed_secs field
//TODO: figure out how to listen for the joint states of interest efficiently
