/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: mapview.h
*   Author  : lubing.han
*   Date    : 2018-05-17
*   Describe: mapview.h
*
********************************************************/
#ifndef __HOBOT_MAPVIEW__
#define __HOBOT_MAPVIEW__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  double x;
  double y;
  double z;
  double s;
  double k;
  double theta;
  int type;
} CPoint;

typedef struct
{
  double sOffset;
  char* type;
  char* weight;
  char* color;
  char* material;
  double width;
  char* laneChange;
  double height;
} CRoadMark;

typedef struct
{
  int id;
  int points_size;
  CPoint* points;
  int roadMarks_size;
  CRoadMark* roadMarks;
} CLane;

typedef struct
{
  int lanes_size;
  CLane* lanes;
  double sOffset;
} CLaneSection;

typedef struct
{
  char* id;
  int laneSections_size;
  CLaneSection* laneSections;
} CRoad;

typedef struct
{
  int roads_size;
  CRoad* roads;
} CMap;

typedef struct
{
  double x;
  double y;
  double z;
} CGeoPoint;

typedef struct
{
  double w;
  double x;
  double y;
  double z;
} CGeoQuaternion;
  
typedef struct
{
  char* id;
  char* type;
  char* subtype;
  char* name;
  double height;
  CGeoPoint position;
  CGeoQuaternion orientation;
  int polygon_size;
  CGeoPoint* polygon;
} CRoadObject;

typedef struct
{
  int objs_size;
  CRoadObject* objs;
} CRoadObjects;

void setFile(const char* file);

CMap getMap();

CRoadObjects getRoadObjects();
    
CRoadObjects getStations();
 
#ifdef __cplusplus
}
#endif

#endif
