//
//  Map.cpp
//  scntest
//
//  Created by 刘伟伟 on 2018/5/17.
//  Copyright © 2018年 刘伟伟. All rights reserved.
//

#include "Map.h"
#include <vector>
//#include "mapview.h"

using namespace std;
#ifdef __cplusplus
extern "C" {
#endif

int getMAPI() {
    vector<int> a;
    a.push_back(1);
    return (int)a.size();
}

void SetFile(const char* filename) {
    setFile(filename);
}

CMap GetMap(){
    return getMap();
}
    
CRoadObjects GetRoadObjects(){
    return getRoadObjects();
}
    
CRoadObjects GetStations() {
    return getStations();
}
   
#ifdef __cplusplus
}
#endif
