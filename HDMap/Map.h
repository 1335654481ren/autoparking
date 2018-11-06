//
//  Map.hpp
//  scntest
//
//  Created by 刘伟伟 on 2018/5/17.
//  Copyright © 2018年 刘伟伟. All rights reserved.
//

#ifndef Map_hpp
#define Map_hpp

#include <stdio.h>
#include "mapview.h"

#ifdef __cplusplus
extern "C" {
#endif


int getMAPI();

void SetFile(const char* filename);
    
CMap GetMap();
    
CRoadObjects GetRoadObjects();

CRoadObjects GetStations();

#ifdef __cplusplus
}
#endif

#endif /* Map_hpp */
