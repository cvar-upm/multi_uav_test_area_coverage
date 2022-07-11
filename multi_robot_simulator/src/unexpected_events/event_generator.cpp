/*!******************************************************************************
 * \authors   Elisa Martinez-Abad
 * \copyright Copyright (c) 2022 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "unexpected_events/event_generator.h"

EventGenerator::EventGenerator(string path) {
    ifstream file;
    filesystem::path file_path = path;

    file_path = expand_path(file_path);
    file.open(file_path);
  
  	vector<vector<string>> content;
	vector<string> row;
	string line, word;

    if (file.is_open()) {
        while(getline(file, line)) {
			row.clear();
			stringstream str(line);
 
			while(getline(str, word, ','))
				row.push_back(word);
			
            Event event;
            if (row.size() > 3)
                event = Event(stoi(row[0]), stoi(row[1]), row[2], stoi(row[3]));
                
            else 
                event = Event(stoi(row[0]), stoi(row[1]), row[2]);

            if (event.getType() != EventType::WRONG_EVENT) 
                events.push_back(event);
		}

        sort(events.begin(), events.end());
    }
    else {
        cerr << "\033[33;1mWARNING: Cannot open events file: \033[0m '" << path << "'" << endl;
    }
}

// Generates a list of the events that should be simulated depending on times.
vector<Event> EventGenerator::genEvents() {
    if (events.size() <= 0) return events;

    vector<Event> eventsToDo = vector<Event>();

    int now = ros::Time::now().toSec();
    int time = events[0].getTime();
    
    while (events.size() > 0 && start > -1 && now - start >= time) {
        eventsToDo.push_back(Event(events[0].getDroneID(), events[0].getType(), events[0].getParam()));

        events.erase(events.begin());
        if (events.size() > 0)
            time = events[0].getTime();
    }

    return eventsToDo;
}

filesystem::path EventGenerator::expand_path (filesystem::path path) {
    filesystem::path expanded = "";

    for(auto it = path.begin(); it != path.end(); it++) {
        string filename = (*it).string();
        filesystem::path new_p = *it;

        if (filename.rfind("$", 0) == 0) {
            char* var = getenv(filename.erase(0,1).c_str());
            if (var != NULL) new_p = var;
        }
        
        expanded /= new_p;
    }
    return expanded;
}
